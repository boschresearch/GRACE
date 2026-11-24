// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <chrono>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include "../inc/planner/roadmap/CCBS/CCBS_Binary.h"
#include "../inc/env/ICapabilities.h"
#include "../inc/util/TempDirGuard.h"
#include "../inc/util/ProcRunner.h"
#include "../inc/util/Logger.h"

#include <pugixml.hpp>
#include <../extern/roadmap/CCBS/const.h> // external CCBS constants

namespace fs = std::filesystem;

namespace {
inline std::optional<bool> parseBool(const std::string& s) {
    if (s == "1" || s == "true"  || s == "TRUE"  || s == "True")  return true;
    if (s == "0" || s == "false" || s == "FALSE" || s == "False") return false;
    return std::nullopt;
}
inline std::optional<int> parseInt(const std::string& s) {
    try { return std::stoi(s); } catch(...) { return std::nullopt; }
}
inline std::optional<double> parseDouble(const std::string& s) {
    try { return std::stod(s); } catch(...) { return std::nullopt; }
}

static int findNodeIdByPos(const IRoadmap& R, b2Vec2 p, float tol=1e-5f)
{
    const int N = R.numNodes();
    for (int i = 0; i < N; ++i) {
        const auto q = R.nodePos(i);
        if (std::abs(q.x - p.x) <= tol && std::abs(q.y - p.y) <= tol) return i;
    }
    return -1;
}
static inline std::string Node(int idx) { return fmt::format("n{}", idx); }
static inline std::string Edge(int idx) { return fmt::format("e{}", idx); }

static fs::path ccbsLogPathFromTask(const fs::path& taskXml)
{
    std::string value = taskXml.filename().string();
    const size_t dotPos = value.find_last_of('.');
    if (dotPos != std::string::npos) value.insert(dotPos, CN_LOG);
    else                             value += CN_LOG;
    return taskXml.parent_path() / value;
}
} // namespace

// ---------------- settings ----------------
void CCBS_Binary::configure(const PlannerSettings& s)
{
    applySettings(s, m_cfg);
}

void CCBS_Binary::applySettings(const PlannerSettings& s, CCBS_BinaryConfig& cfg)
{
  if (auto v = s.get<std::string>("binary"))                   cfg.binary = *v;
  if (auto v = s.get<bool>("use_cardinal"))                    cfg.use_cardinal = *v;
  if (auto v = s.get<bool>("use_disjoint_splitting"))          cfg.use_disjoint_splitting = *v;
  if (auto v = s.get<int>("hlh_type"))                         cfg.hlh_type = *v;
  if (auto v = s.get<int>("connectedness"))                    cfg.connectedness = *v;
  if (auto v = s.get<double>("focal_weight"))                  cfg.focal_weight = *v;
  if (auto v = s.get<double>("agent_size"))                    cfg.agent_size = *v;
  if (auto v = s.get<double>("timelimit_s"))                   cfg.timelimit_s = *v;
  if (auto v = s.get<double>("precision"))                     cfg.precision = *v;
  if (auto v = s.get<bool>("keepTemps"))                       cfg.keepTemps = *v;
  // common alias
  if (auto v = s.get<double>("cutoff_s"))                      cfg.timelimit_s = *v;
  // string fallbacks
  if (auto v = s.get<std::string>("use_cardinal"))             if (auto x=parseBool(*v))   cfg.use_cardinal=*x;
  if (auto v = s.get<std::string>("use_disjoint_splitting"))   if (auto x=parseBool(*v))   cfg.use_disjoint_splitting=*x;
  if (auto v = s.get<std::string>("hlh_type"))                 if (auto x=parseInt(*v))    cfg.hlh_type=*x;
  if (auto v = s.get<std::string>("connectedness"))            if (auto x=parseInt(*v))    cfg.connectedness=*x;
  if (auto v = s.get<std::string>("focal_weight"))             if (auto x=parseDouble(*v)) cfg.focal_weight=*x;
  if (auto v = s.get<std::string>("agent_size"))               if (auto x=parseDouble(*v)) cfg.agent_size=*x;
  if (auto v = s.get<std::string>("timelimit_s"))              if (auto x=parseDouble(*v)) cfg.timelimit_s=*x;
  if (auto v = s.get<std::string>("cutoff_s"))                 if (auto x=parseDouble(*v)) cfg.timelimit_s=*x;
  if (auto v = s.get<std::string>("precision"))                if (auto x=parseDouble(*v)) cfg.precision=*x;
  if (auto v = s.get<std::string>("keepTemps"))                if (auto x=parseBool(*v))   cfg.keepTemps=*x;
}

// ---------------- writers ----------------
bool CCBS_Binary::writeGraphML(const fs::path& path, const IRoadmap& R)
{
    std::ofstream ofs(path);
    if (!ofs) return false;

    ofs << R"(<?xml version="1.0" encoding="UTF-8"?>)"
        << "\n<graphml xmlns=\"http://graphml.graphdrawing.org/xmlns\" "
        << "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
        << "xsi:schemaLocation=\"http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd\">\n"
        << "  <key id=\"key0\" for=\"node\" attr.name=\"coords\" attr.type=\"string\" />\n"
        << "  <key id=\"key1\" for=\"edge\" attr.name=\"weight\" attr.type=\"double\" />\n"
        << "  <graph id=\"G\" edgedefault=\"directed\" parse.nodeids=\"free\" parse.edgeids=\"canonical\" parse.order=\"nodesfirst\">\n";

    const int N = R.numNodes();
    for (int i = 0; i < N; ++i) {
        const auto p = R.nodePos(i);
        ofs << "    <node id=\"" << Node(i) << "\">\n"
            << "      <data key=\"key0\">" << std::setprecision(9) << p.x << "," << p.y << "</data>\n"
            << "    </node>\n";
    }

  int e = 0;
  for (int u = 0; u < N; ++u) {
      const auto pu = R.nodePos(u);
      for (int v : R.adj(u)) {
          const auto pv = R.nodePos(v);
          const double w = std::hypot(static_cast<double>(pv.x - pu.x),
                                      static_cast<double>(pv.y - pu.y));
          ofs << "    <edge id=\"" << Edge(e++) << "\" source=\"" << Node(u) << "\" target=\"" << Node(v) << "\">\n"
              << "      <data key=\"key1\">" << std::setprecision(17) << w << "</data>\n"
              << "    </edge>\n";
      }
  }

    ofs << "  </graph>\n</graphml>\n";
    ofs.flush();
    return static_cast<bool>(ofs);
}

bool CCBS_Binary::writeTaskXml(const fs::path& path, const IRoadmap& R, const std::vector<StartGoal>& sgs)
{
    std::ofstream ofs(path);
    if (!ofs) return false;

    ofs << R"(<?xml version="1.0" ?>)"
        << "\n<root>\n";

    for (const auto& sg : sgs) {
        const int s = findNodeIdByPos(R, sg.start);
        const int g = findNodeIdByPos(R, sg.goal);
        if (s < 0 || g < 0) {
            Logger::instance().error("CCBS (bin): Start/goal not on roadmap nodes.");
            return false;
        }
        ofs << "  <agent start_id=\"" << s << "\" goal_id=\"" << g << "\"/>\n";
    }

    ofs << "</root>\n";
    ofs.flush();
    return static_cast<bool>(ofs);
}

bool CCBS_Binary::writeConfigXml(const fs::path& path, const CCBS_BinaryConfig& c)
{
    std::ofstream ofs(path);
    if (!ofs) return false;

    ofs << R"(<?xml version="1.0" ?>
<root>
  <algorithm>
    <use_cardinal>)"            << (c.use_cardinal ? "true" : "false") << R"(</use_cardinal>
    <use_disjoint_splitting>)"  << (c.use_disjoint_splitting ? "true" : "false") << R"(</use_disjoint_splitting>
    <hlh_type>)"                << c.hlh_type        << R"(</hlh_type>
    <connectedness>)"           << c.connectedness   << R"(</connectedness>
    <focal_weight>)"            << std::setprecision(17) << c.focal_weight << R"(</focal_weight>
    <agent_size>)"              << std::setprecision(17) << c.agent_size   << R"(</agent_size>
    <timelimit>)"               << std::setprecision(17) << c.timelimit_s  << R"(</timelimit>
    <precision>)"               << std::setprecision(17) << c.precision    << R"(</precision>
  </algorithm>
</root>
)";
    ofs.flush();
    return static_cast<bool>(ofs);
}

// ---------------- runner ----------------
CCBS_Binary::RunResult
CCBS_Binary::runSolver(const fs::path& workdir,
                       const fs::path& mapPath,
                       const fs::path& taskPath,
                       const double time_limit_s) const
{
    const fs::path outPath   = workdir / "ccbs_stdout.txt";
    const fs::path errPath   = workdir / "ccbs_stderr.txt";
    const fs::path configXml = workdir / "config.xml";

    CCBS_BinaryConfig cfg = m_cfg;
    cfg.timelimit_s = (time_limit_s > 0.0 ? time_limit_s : cfg.timelimit_s);

    if (!writeConfigXml(configXml, cfg)) {
        Logger::instance().error("CCBS (bin): failed to write config.xml");
        return {false,false,0};
    }

    std::vector<std::string> args{
        mapPath.string(),
        taskPath.string(),
        configXml.string()
    };

    const auto timeout_ms = std::chrono::milliseconds(
        static_cast<long>(cfg.timelimit_s * 1000.0) + 500);

    const auto res = util::runWithTimeout(cfg.binary, args, workdir, outPath, errPath, timeout_ms);

    if (res.timed_out) return {false, true, 0};
    if (res.exit_code != 0) return {false, false, res.exit_code};
    return {true, false, 0};
}

// ---------------- parser ----------------
std::vector<Plan>
CCBS_Binary::parseLogToPlans(const fs::path& taskXmlPath)
{
    const fs::path logPath = ccbsLogPathFromTask(taskXmlPath);

    pugi::xml_document doc;
    pugi::xml_parse_result ok = doc.load_file(logPath.string().c_str());
    if (!ok) {
        Logger::instance().error("CCBS (bin): failed to read log: " + logPath.string());
        return {};
    }

    auto root = doc.child(CNS_TAG_ROOT);
    auto log  = root.child(CNS_TAG_LOG);
    if (!log) {
        Logger::instance().error("CCBS (bin): invalid log: missing <log>");
        return {};
    }

    int maxAgentIdx = -1;
    for (auto agent : log.children(CNS_TAG_AGENT)) {
        maxAgentIdx = std::max(maxAgentIdx, agent.attribute(CNS_TAG_ATTR_NUM).as_int(-1));
    }
    if (maxAgentIdx < 0) return {};

    std::vector<Plan> plans(static_cast<size_t>(maxAgentIdx + 1));

    for (auto agent : log.children(CNS_TAG_AGENT)) {
        const int idx = agent.attribute(CNS_TAG_ATTR_NUM).as_int(-1);
        if (idx < 0 || idx >= static_cast<int>(plans.size())) continue;

        auto path = agent.child(CNS_TAG_PATH);
        if (!path) continue;

        Plan P;
        float t = 0.0f;
        bool first = true;

        for (auto sec : path.children(CNS_TAG_SECTION)) {
            const double si = sec.attribute(CNS_TAG_START_I).as_double();
            const double sj = sec.attribute(CNS_TAG_START_J).as_double();
            const double gi = sec.attribute(CNS_TAG_GOAL_I).as_double();
            const double gj = sec.attribute(CNS_TAG_GOAL_J).as_double();
            const double dur= sec.attribute(CNS_TAG_ATTR_DURATION).as_double();

            if (first) {
                P.waypoints.push_back(Waypoint{ b2Vec2{ static_cast<float>(si), static_cast<float>(sj) }, t });
                first = false;
            }

            t += static_cast<float>(dur);
            P.waypoints.push_back(Waypoint{ b2Vec2{ static_cast<float>(gi), static_cast<float>(gj) }, t });
        }

        plans[static_cast<size_t>(idx)] = std::move(P);
    }

    return plans;
}

// ---------------- entry ----------------
PlanResult CCBS_Binary::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids = req.ids;
    out.status = PlanStatus::Error;
    out.plans.clear();

    const IRoadmap* R = req.env.roadmapCapability();
    if (!R) {
        Logger::instance().error("CCBS (bin): environment is not a MAPFRoadmapEnv.");
        return out;
    }
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("CCBS (bin): ids.size() != sgs.size().");
        return out;
    }

    // Per-call overrides (don’t mutate defaults)
    CCBS_BinaryConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    const double time_limit_s = resolvePlannerTimeLimit(
        req.time_limit_s, cfg.timelimit_s);

    const auto t0 = std::chrono::steady_clock::now();

    util::TempDirGuard tmpdir(fs::temp_directory_path(), "ccbs_", cfg.keepTemps);
    const fs::path base   = tmpdir.path();
    const fs::path mapXml = base / "map.xml";
    const fs::path taskXml= base / "task.xml";

    if (!writeGraphML(mapXml, *R)) {
        Logger::instance().error("CCBS (bin): failed to write GraphML.");
        return out;
    }
    if (!writeTaskXml(taskXml, *R, req.sgs)) {
        Logger::instance().error("CCBS (bin): failed to write task.xml.");
        return out;
    }

    const auto rr = runSolver(base, mapXml, taskXml, time_limit_s);
    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!rr.ok) {
        out.status = rr.timed_out ? PlanStatus::Timeout : PlanStatus::Error;
        if (cfg.keepTemps) { Logger::instance().info("CCBS (bin): temp kept at " + base.string()); tmpdir.set_keep(true); }
        return out;
    }

    out.plans  = parseLogToPlans(taskXml);
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;

    if (cfg.keepTemps) {
        Logger::instance().info("CCBS (bin): temp kept at " + base.string());
        tmpdir.set_keep(true);
    }
    return out;
}
