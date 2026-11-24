// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/continuous/DBCBS/DBCBS_Binary.h"
#include "../inc/planner/continuous/DBCBS/DBCBS_IO.h"

#include "../inc/util/Logger.h"
#include "../inc/util/TempDirGuard.h"
#include "../inc/util/ProcRunner.h"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <chrono>
#include <charconv>

namespace fs = std::filesystem;

// tiny parse helpers
namespace {
inline std::optional<bool> parseBool(const std::string& s) {
    if (s=="1"||s=="true"||s=="TRUE"||s=="True") return true;
    if (s=="0"||s=="false"||s=="FALSE"||s=="False") return false;
    return std::nullopt;
}
inline std::optional<int> parseInt(const std::string& s) {
    int v{}; auto* b=s.data(); auto* e=b+s.size();
    if (std::from_chars(b,e,v).ec==std::errc{}) return v;
    return std::nullopt;
}
inline std::optional<double> parseDouble(const std::string& s) {
    try { return std::stod(s); } catch(...) { return std::nullopt; }
}
template <typename T>
bool asPair2(const YAML::Node& n, T& x, T& y) {
    if (!n || !n.IsSequence() || n.size()<2) return false;
    try { x=n[0].as<T>(); y=n[1].as<T>(); } catch (...) { return false; }
    return true;
}
} // namespace

// ---------------- settings -> configure ----------------
void DBCBS_Binary::configure(const PlannerSettings& s)
{
    // typed first
    if (auto v=s.get<std::string>("binary"))                 m_cfg.binary=*v;
    if (auto v=s.get<double>("timelimit_s"))                 m_cfg.timelimit_s=*v;
    if (auto v=s.get<double>("cutoff_s"))                    m_cfg.timelimit_s=*v; // alias
    if (auto v=s.get<bool>("keepTemps"))                     m_cfg.keepTemps=*v;

    if (auto v=s.get<double>("alpha"))                       m_cfg.alpha=*v;
    if (auto v=s.get<bool>("filter_duplicates"))             m_cfg.filter_duplicates=*v;
    if (auto v=s.get<std::string>("heuristic1"))             m_cfg.heuristic1=*v;
    if (auto v=s.get<double>("heuristic1_delta"))            m_cfg.heuristic1_delta=*v;
    if (auto v=s.get<double>("delta0"))                      m_cfg.delta0=*v;
    if (auto v=s.get<int>("num_primitives0"))                m_cfg.num_primitives0=*v;
    if (auto v=s.get<double>("delta_rate"))                  m_cfg.delta_rate=*v;
    if (auto v=s.get<double>("num_primitives_rate"))         m_cfg.num_primitives_rate=*v;

    if (auto v=s.get<std::string>("robot_type"))             m_cfg.robot_type=*v;
    if (auto v=s.get<double>("step_dt_s"))                   if (*v>0.0) m_cfg.step_dt_s=*v;
    if (auto v=s.get<int>("takeK"))                          m_cfg.takeK=*v;
    if (auto v=s.get<double>("agent_radius"))                m_agent_radius=*v; // reserved/alias
    if (auto v=s.get<double>("agent_size"))                  m_agent_radius=*v;

    // string fallbacks
    if (auto v=s.get<std::string>("timelimit_s"))            if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v=s.get<std::string>("cutoff_s"))               if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v=s.get<std::string>("keepTemps"))              if (auto x=parseBool(*v))   m_cfg.keepTemps=*x;
    if (auto v=s.get<std::string>("step_dt_s"))              if (auto x=parseDouble(*v)) if (*x>0.0) m_cfg.step_dt_s=*x;
    if (auto v=s.get<std::string>("takeK"))                  if (auto x=parseInt(*v))    m_cfg.takeK=*x;

    if (auto v=s.get<std::string>("alpha"))                  if (auto x=parseDouble(*v)) m_cfg.alpha=*x;
    if (auto v=s.get<std::string>("filter_duplicates"))      if (auto x=parseBool(*v))   m_cfg.filter_duplicates=*x;
    if (auto v=s.get<std::string>("heuristic1_delta"))       if (auto x=parseDouble(*v)) m_cfg.heuristic1_delta=*x;
    if (auto v=s.get<std::string>("delta0"))                 if (auto x=parseDouble(*v)) m_cfg.delta0=*x;
    if (auto v=s.get<std::string>("num_primitives0"))        if (auto x=parseInt(*v))    m_cfg.num_primitives0=*x;
    if (auto v=s.get<std::string>("delta_rate"))             if (auto x=parseDouble(*v)) m_cfg.delta_rate=*x;
    if (auto v=s.get<std::string>("num_primitives_rate"))    if (auto x=parseDouble(*v)) m_cfg.num_primitives_rate=*x;
    if (auto v=s.get<std::string>("agent_radius"))           if (auto x=parseDouble(*v)) m_agent_radius=*x;
    if (auto v=s.get<std::string>("agent_size"))             if (auto x=parseDouble(*v)) m_agent_radius=*x;

    // YAML ingestion
    auto parseRobotMeta = [&](const std::string& yaml){
        using ::Logger;
        try {
            YAML::Node root = YAML::Load(yaml);
            m_robot_meta.clear();
            if (!root || !root.IsSequence()) {
                Logger::instance().warn("DBCBS (bin): robot_meta(_yaml) is not a sequence; ignoring.");
                return;
            }
            m_robot_meta.reserve(root.size());
            for (const auto& r : root) {
                ContinuousRobotMeta rm;
                if (r && r.IsMap()) {
                    if (auto ty=r["type"]; ty && ty.IsScalar()) rm.type=ty.as<std::string>();
                    if (auto fp=r["footprint"]; fp && fp.IsMap()) {
                        auto s = fp["shape"];
                        if (s && s.IsScalar()) {
                            rm.fp.shape = s.as<std::string>();
                            if (rm.fp.shape=="circle") {
                                if (auto rad=fp["radius"]; rad && rad.IsScalar()) { rm.fp.radius=rad.as<double>(); rm.fp.valid=true; }
                            } else if (rm.fp.shape=="box") {
                                auto sz=fp["size"];
                                if (sz && sz.IsSequence() && sz.size()>=2) { rm.fp.size_x=sz[0].as<double>(); rm.fp.size_y=sz[1].as<double>(); rm.fp.valid=true; }
                            } else if (rm.fp.shape=="multibox") {
                                auto boxes=fp["boxes"];
                                if (boxes && boxes.IsSequence()) {
                                    for (const auto& b: boxes) {
                                        if (!b || !b.IsMap()) continue;
                                        auto c=b["center"], s2=b["size"];
                                        if (c && s2 && c.IsSequence() && c.size()>=2 && s2.IsSequence() && s2.size()>=2) {
                                            ContinuousRobotMeta::Part p{};
                                            p.cx=c[0].as<double>(); p.cy=c[1].as<double>();
                                            p.sx=s2[0].as<double>(); p.sy=s2[1].as<double>();
                                            p.angle=(b["angle"] && b["angle"].IsScalar())?b["angle"].as<double>():0.0;
                                            rm.fp.boxes.push_back(p);
                                        }
                                    }
                                    if (!rm.fp.boxes.empty()) rm.fp.valid=true;
                                }
                            }
                        }
                    }
                }
                m_robot_meta.push_back(std::move(rm));
            }
        } catch (const std::exception& e) {
            Logger::instance().warn(std::string("DBCBS (bin): failed to parse robot_meta yaml: ")+e.what());
            m_robot_meta.clear();
        }
    };
    auto parseObstacles = [&](const std::string& yaml){
        using ::Logger;
        try {
            YAML::Node root = YAML::Load(yaml);
            m_env_obs.clear();
            if (!root || !root.IsSequence()) {
                Logger::instance().warn("DBCBS (bin): obstacles(_yaml) is not a sequence; ignoring.");
                return;
            }
            for (const auto& o : root) {
                if (!o || !o.IsMap()) continue;
                DBCBSObstacle obs;
                obs.type = o["type"] ? o["type"].as<std::string>() : "";
                if (obs.type=="box") {
                    double x=0,y=0,w=0,h=0;
                    if (asPair2(o["center"],x,y) && asPair2(o["size"],w,h)) { obs.cx=x; obs.cy=y; obs.sx=w; obs.sy=h; m_env_obs.push_back(obs); }
                } else if (obs.type=="circle") {
                    double x=0,y=0; double r=0;
                    if (asPair2(o["center"],x,y) && o["radius"] && o["radius"].IsScalar()) { obs.cx=x; obs.cy=y; obs.radius=o["radius"].as<double>(); m_env_obs.push_back(obs); }
                } else if (obs.type=="capsule") {
                    double p0x=0,p0y=0,p1x=0,p1y=0;
                    if (asPair2(o["p0"],p0x,p0y) && asPair2(o["p1"],p1x,p1y)) {
                        obs.p0x=p0x; obs.p0y=p0y; obs.p1x=p1x; obs.p1y=p1y;
                        obs.radius=(o["radius"]&&o["radius"].IsScalar())?o["radius"].as<double>():0.0;
                        m_env_obs.push_back(obs);
                    }
                } else if (obs.type=="polygon") {
                    obs.polygon.clear();
                    if (auto pts=o["points"]; pts && pts.IsSequence()) {
                        for (const auto& p: pts) { double px=0,py=0; if (asPair2(p,px,py)) obs.polygon.push_back({px,py}); }
                        if (obs.polygon.size()>=3) m_env_obs.push_back(obs);
                    }
                } else {
                    Logger::instance().warn("DBCBS (bin): unknown obstacle type; skipped.");
                }
            }
        } catch (const std::exception& e) {
            Logger::instance().warn(std::string("DBCBS (bin): failed to parse obstacles yaml: ")+e.what());
            m_env_obs.clear();
        }
    };

    if (auto v=s.get<std::string>("robot_meta"))       parseRobotMeta(*v);
    if (auto v=s.get<std::string>("robot_meta_yaml"))  parseRobotMeta(*v);
    if (auto v=s.get<std::string>("obstacles"))        parseObstacles(*v);
    if (auto v=s.get<std::string>("obstacles_yaml"))   parseObstacles(*v);
}

// ---------------- YAML emit ----------------
void DBCBS_Binary::writeCfgYaml(const DBCBS_BinaryConfig& c, const fs::path& outYaml)
{
    std::ofstream y(outYaml);
    if (!y) throw std::runtime_error("DBCBS (bin): failed to write cfg YAML: " + outYaml.string());
    y <<
      "alpha: " << c.alpha << "\n"
      "filter_duplicates: " << (c.filter_duplicates ? "true" : "false") << "\n"
      "heuristic1: " << c.heuristic1 << "\n"
      "heuristic1_delta: " << c.heuristic1_delta << "\n"
      "delta_0: " << c.delta0 << "\n"
      "num_primitives_0: " << c.num_primitives0 << "\n"
      "delta_rate: " << c.delta_rate << "\n"
      "num_primitives_rate: " << c.num_primitives_rate << "\n";
    y.flush();
}

// ---------------- entry ----------------
PlanResult DBCBS_Binary::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids    = req.ids;
    out.status = PlanStatus::Error;

    if (!req.env.continuousCapability()) {
        Logger::instance().error("DBCBS (bin): Environment does not support IContinuousSpace.");
        return out;
    }
    if (req.ids.size() != req.sgs.size()) { Logger::instance().error("DBCBS (bin): ids.size() != sgs.size()."); return out; }

    // per-call overrides via shadow
    DBCBS_BinaryConfig base = m_cfg;
    DBCBS_Binary shadow(base);

    // >>> carry over preconfigured metadata (fix for lost obstacles/meta) <<<
    shadow.m_robot_meta   = this->m_robot_meta;
    shadow.m_env_obs      = this->m_env_obs;
    shadow.m_agent_radius = this->m_agent_radius;

    // apply per-request overrides
    shadow.configure(req.settings);

    const double timelimit_s = resolvePlannerTimeLimit(
        req.time_limit_s, shadow.m_cfg.timelimit_s);

    // Optional cap
    std::vector<StartGoal> sgs_use = req.sgs;
    if (shadow.m_cfg.takeK && *shadow.m_cfg.takeK > 0 && *shadow.m_cfg.takeK < static_cast<int>(sgs_use.size()))
        sgs_use.resize(static_cast<size_t>(*shadow.m_cfg.takeK));

    // If only a global agent radius was provided, synthesize simple circle footprints.
    if (!shadow.m_robot_meta.empty() == false && shadow.m_agent_radius && *shadow.m_agent_radius > 0.0) {
        shadow.m_robot_meta.assign(sgs_use.size(), {});
        for (auto& r : shadow.m_robot_meta) {
            r.type = shadow.m_cfg.robot_type;
            r.fp.shape  = "circle";
            r.fp.radius = *shadow.m_agent_radius;
            r.fp.valid  = true;
        }
    }

    util::TempDirGuard tmp(fs::temp_directory_path(), "dbcbs_", shadow.m_cfg.keepTemps);
    const fs::path baseDir   = tmp.path();
    const fs::path problem   = baseDir / "problem.yaml";
    const fs::path cfgYaml   = baseDir / "cfg.yaml";
    const fs::path outYaml   = baseDir / "solution.yaml";
    const fs::path jointYaml = baseDir / "joint.yaml";
    const fs::path optYaml   = baseDir / "opt.yaml";
    const fs::path stdoutTxt = baseDir / "stdout.txt";
    const fs::path stderrTxt = baseDir / "stderr.txt";

    try {
        dbcbs_writeProblemYaml(req.env, sgs_use, shadow.m_robot_meta, shadow.m_cfg.robot_type, shadow.m_env_obs, problem);
        writeCfgYaml(shadow.m_cfg, cfgYaml);
    } catch (const std::exception& e) {
        Logger::instance().error(e.what());
        return out;
    }

    // run external db_cbs
    std::vector<std::string> args = {
        "--input",        problem.string(),
        "--output",       outYaml.string(),
        "--joint",        jointYaml.string(),
        "--optimization", optYaml.string(),
        "--cfg",          cfgYaml.string()
    };

    const auto t0 = std::chrono::steady_clock::now();
    const auto timeout_ms = std::chrono::milliseconds(static_cast<long>(timelimit_s*1000.0) + 2000);
    const auto res = util::runWithTimeout(shadow.m_cfg.binary, args, baseDir, stdoutTxt, stderrTxt, timeout_ms);
    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (res.timed_out) {
        Logger::instance().warn("DBCBS (bin): planner timed out.");
        out.status = PlanStatus::Timeout;
        if (shadow.m_cfg.keepTemps) { Logger::instance().info("DBCBS (bin): kept temp dir: " + baseDir.string()); tmp.set_keep(true); }
        return out;
    }
    if (res.exit_code != 0) {
        Logger::instance().warn(fmt::format("DBCBS (bin): exit code {}. See {}", res.exit_code, stderrTxt.string()));
        // still attempt to parse output
        if (shadow.m_cfg.keepTemps) { Logger::instance().info("DBCBS (bin): kept temp dir: " + baseDir.string()); tmp.set_keep(true); }
    }

    // parse output (prefer optimization)
    std::vector<Plan> plans;
    auto try_parse = [&](const fs::path& p)->bool {
        try { return dbcbs_parseSolutionYaml(p, shadow.m_cfg.step_dt_s, plans); }
        catch (...) { return false; }
    };

    bool parsed = false;
    if (fs::exists(optYaml) && try_parse(optYaml)) {
        Logger::instance().info("DBCBS (bin): using opt.yaml");
        parsed = true;
    } else if (try_parse(outYaml)) {
        Logger::instance().info("DBCBS (bin): using solution.yaml");
        parsed = true;
    }

    if (!parsed) {
        Logger::instance().warn("DBCBS (bin): failed to parse both opt.yaml and solution.yaml.");
        out.status = PlanStatus::Error;
        if (shadow.m_cfg.keepTemps) { Logger::instance().info("DBCBS (bin): kept temp dir: " + baseDir.string()); tmp.set_keep(true); }
        return out;
    }

    if (plans.size() != sgs_use.size())
        plans.resize(std::min(plans.size(), sgs_use.size()));

    out.plans = std::move(plans);
    out.ids.resize(out.plans.size());
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;

    if (shadow.m_cfg.keepTemps) {
        Logger::instance().info("DBCBS (bin): kept temp dir: " + baseDir.string());
        tmp.set_keep(true);
    }
    return out;
}
