// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/roadmap/PSIPPCTC/PSIPPCTC_Binary.h"

#include "../inc/util/Logger.h"
#include "../inc/util/TempDirGuard.h"
#include "../inc/util/ProcRunner.h"
#include "../inc/env/ICapabilities.h"

#include <fmt/format.h>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cctype>
#include <algorithm>
#include <stdexcept>
#include <charconv>

namespace fs = std::filesystem;

namespace {
inline std::optional<bool> parseBool(const std::string& s) {
    if (s == "1" || s == "true"  || s == "TRUE"  || s == "True")  return true;
    if (s == "0" || s == "false" || s == "FALSE" || s == "False") return false;
    return std::nullopt;
}
inline std::optional<int> parseInt(const std::string& s) {
    int v{};
    auto* b = s.data(); auto* e = b + s.size();
    if (std::from_chars(b, e, v).ec == std::errc{}) return v;
    return std::nullopt;
}
inline std::optional<double> parseDouble(const std::string& s) {
    try { return std::stod(s); } catch(...) { return std::nullopt; }
}
} // namespace

// ---------------- Settings ----------------
void PSIPPCTC_Binary::configure(const PlannerSettings& s)
{
    if (auto v = s.get<std::string>("binary"))              m_cfg.binary = *v;
    if (auto v = s.get<double>("timelimit_s"))              m_cfg.timelimit_s = *v;
    if (auto v = s.get<double>("cutoff_s"))                 m_cfg.timelimit_s = *v; // common alias
    if (auto v = s.get<int>("priority_strategy"))           m_cfg.priority_strategy = *v;
    if (auto v = s.get<int>("takeK"))                       m_cfg.takeK = *v;
    if (auto v = s.get<bool>("keepTemps"))                  m_cfg.keepTemps = *v;
    if (auto v = s.get<double>("agent_radius"))             m_agent_radius = *v;
    if (auto v = s.get<double>("agent_size"))               m_agent_radius = *v; // alias

    // string fallbacks
    if (auto v = s.get<std::string>("timelimit_s"))         if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("cutoff_s"))            if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("priority_strategy"))   if (auto x=parseInt(*v))    m_cfg.priority_strategy=*x;
    if (auto v = s.get<std::string>("takeK"))               if (auto x=parseInt(*v))    m_cfg.takeK=*x;
    if (auto v = s.get<std::string>("keepTemps"))           if (auto x=parseBool(*v))   m_cfg.keepTemps=*x;
    if (auto v = s.get<std::string>("agent_radius"))        if (auto x=parseDouble(*v)) m_agent_radius=*x;
    if (auto v = s.get<std::string>("agent_size"))          if (auto x=parseDouble(*v)) m_agent_radius=*x;
}

// --------------- Problem writer (text; directed edges) -----------------------
void PSIPPCTC_Binary::writeRoadmapProblemTxt(const IRoadmap& R,
                                             const std::vector<std::pair<int,int>>& tasks,
                                             double agent_radius,
                                             const fs::path& outTxt)
{
    const int V = R.numNodes();
    long long E = 0;
    for (int u=0; u<V; ++u) E += static_cast<long long>(R.adj(u).size());
    const int N = static_cast<int>(tasks.size());

    std::ofstream f(outTxt);
    if (!f) throw std::runtime_error("PSIPPCTC_Binary: failed to open problem file: " + outTxt.string());

    // Header
    f << V << ' ' << E << ' ' << N << '\n';

    f.setf(std::ios::fixed);
    f << std::setprecision(17);
    for (int i=0; i<V; ++i) {
        const b2Vec2 p = R.nodePos(i);
        f << p.x << ' ' << p.y << '\n';
    }

    // Directed edges: one line per (u -> v)
    for (int u=0; u<V; ++u) for (int v : R.adj(u)) f << u << ' ' << v << '\n';

    // Tasks: start_id goal_id
    for (const auto& [s,g] : tasks) f << s << ' ' << g << '\n';

    // Radius at end
    f << agent_radius << '\n';
    f.flush();
}

// --------------- Stdout plan parser ------------------------------------------
bool PSIPPCTC_Binary::parsePlannerStdoutPlan(const fs::path& planTxt,
                                             std::vector<Plan>& outPlans)
{
    std::ifstream in(planTxt);
    if (!in) return false;

    int agents = 0; double rad = 0.0;
    if (!(in >> agents >> rad)) return false;

    outPlans.assign(static_cast<size_t>(agents), {});

    for (int a=0; a<agents; ++a) {
        int K = 0; if (!(in >> K)) return false;
        Plan P; P.waypoints.reserve(static_cast<size_t>(K));
        for (int i=0;i<K;++i) {
            double t=0.0, x=0.0, y=0.0;
            if (!(in >> t >> x >> y)) return false;
            P.waypoints.push_back(Waypoint{
                b2Vec2{ static_cast<float>(x), static_cast<float>(y) },
                static_cast<float>(t)
            });
        }
        outPlans[static_cast<size_t>(a)] = std::move(P);
    }
    return true;
}

// --------------- Helpers -----------------------------------------------------
std::vector<std::pair<int,int>>
PSIPPCTC_Binary::nodePairsFromMeters(const IRoadmap& R,
                                     const std::vector<StartGoal>& sgs)
{
    std::vector<std::pair<int,int>> out;
    out.reserve(sgs.size());
    for (const auto& sg : sgs) {
        const int s = R.nearestNode(sg.start);
        const int g = R.nearestNode(sg.goal);
        out.emplace_back(s, g);
    }
    return out;
}

PSIPPCTC_Binary::RunResult
PSIPPCTC_Binary::run(const fs::path& workdir,
                     const double timelimit_s,
                     const fs::path& problemTxt,
                     const fs::path& stdoutTxt,
                     const fs::path& stderrTxt) const
{
    std::vector<std::string> args = {
        fmt::format("{}", timelimit_s),
        problemTxt.string()
    };
    const auto timeout_ms = std::chrono::milliseconds(static_cast<long>(timelimit_s*1000.0) + 2000);
    const auto res = util::runWithTimeout(m_cfg.binary, args, workdir, stdoutTxt, stderrTxt, timeout_ms);
    if (res.timed_out) return {false,true,0};
    if (res.exit_code != 0) return {false,false,res.exit_code};
    return {true,false,0};
}

// --------------- IPlanner::plan ----------------------------------------------
PlanResult PSIPPCTC_Binary::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids    = req.ids;
    out.status = PlanStatus::Error;

    const IRoadmap* R = req.env.roadmapCapability();
    if (!R) {
        Logger::instance().error("PSIPP-CTC (bin): Environment is not MAPFRoadmapEnv.");
        return out;
    }
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("PSIPP-CTC (bin): ids.size() != sgs.size().");
        return out;
    }

    // Per-call overrides
    PSIPPCTC_BinaryConfig cfg = m_cfg;
    PSIPPCTC_Binary shadow(cfg); shadow.configure(req.settings); cfg = shadow.m_cfg;
    std::optional<double> agent_radius = shadow.m_agent_radius;

    const double timelimit_s = resolvePlannerTimeLimit(
        req.time_limit_s, cfg.timelimit_s);
    const auto   t0          = std::chrono::steady_clock::now();

    util::TempDirGuard tmp(fs::temp_directory_path(), "psipp_", cfg.keepTemps);
    const fs::path base       = tmp.path();
    const fs::path problemTxt = base / "problem.txt";
    const fs::path stdoutTxt  = base / "stdout_plan.txt";
    const fs::path stderrTxt  = base / "stderr_log.txt";

    // Build task pairs in node space (optionally cap K)
    auto pairs_all = nodePairsFromMeters(*R, req.sgs);
    if (cfg.takeK && *cfg.takeK > 0 && *cfg.takeK < static_cast<int>(pairs_all.size()))
        pairs_all.resize(static_cast<size_t>(*cfg.takeK));

    const double radius = agent_radius.value_or(0.25);

    try {
        writeRoadmapProblemTxt(*R, pairs_all, radius, problemTxt);
    } catch (const std::exception& e) {
        Logger::instance().error(e.what());
        return out;
    }

    const auto rr = run(base, timelimit_s, problemTxt, stdoutTxt, stderrTxt);
    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!rr.ok) {
        out.status = rr.timed_out ? PlanStatus::Timeout : PlanStatus::Error;
        if (cfg.keepTemps) { Logger::instance().info("PSIPP-CTC (bin): kept temp dir: " + base.string()); tmp.set_keep(true); }
        return out;
    }

    std::vector<Plan> plans;
    if (!parsePlannerStdoutPlan(stdoutTxt, plans)) {
        Logger::instance().warn("PSIPP-CTC (bin): failed to parse plan from stdout.");
        out.status = PlanStatus::Error;
        return out;
    }

    // Trim/match ids
    if (plans.size() != pairs_all.size()) {
        Logger::instance().warn("PSIPP-CTC (bin): plan agent count != requested tasks.");
        plans.resize(std::min(plans.size(), pairs_all.size()));
    }

    out.plans = std::move(plans);
    out.ids.resize(out.plans.size());
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;

    if (cfg.keepTemps) {
        Logger::instance().info("PSIPP-CTC (bin): kept temp dir: " + base.string());
        tmp.set_keep(true);
    }
    return out;
}
