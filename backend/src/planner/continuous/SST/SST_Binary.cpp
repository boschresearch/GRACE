// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/continuous/SST/SST_Binary.h"

#include <charconv>
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>
#include <fstream>
#include <chrono>

#include "../inc/util/Logger.h"
#include "../inc/util/TempDirGuard.h"
#include "../inc/util/ProcRunner.h"

namespace fs = std::filesystem;

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
    try { return std::stod(s); } catch (...) { return std::nullopt; }
}
template <typename T>
bool asPair2(const YAML::Node& n, T& x, T& y) {
    if (!n || !n.IsSequence() || n.size()<2) return false;
    try { x=n[0].as<T>(); y=n[1].as<T>(); } catch (...) { return false; }
    return true;
}
} // namespace

// ---------------- Configure ----------------
void SST_Binary::configure(const PlannerSettings& s) {
    // typed
    if (auto v = s.get<std::string>("binary"))                 m_cfg.binary = *v;
    if (auto v = s.get<double>("timelimit_s"))                 m_cfg.timelimit_s = *v;
    if (auto v = s.get<double>("cutoff_s"))                    m_cfg.timelimit_s = *v; // alias
    if (auto v = s.get<bool>("keepTemps"))                     m_cfg.keepTemps = *v;
    if (auto v = s.get<std::string>("planner"))                m_cfg.planner = *v;
    if (auto v = s.get<double>("propagation_step_size"))       m_cfg.propagation_step_size = *v;
    if (auto v = s.get<int>("control_duration_min"))           m_cfg.control_duration_min = *v;
    if (auto v = s.get<int>("control_duration_max"))           m_cfg.control_duration_max = *v;
    if (auto v = s.get<double>("goal_epsilon"))                m_cfg.goal_epsilon = *v;
    if (auto v = s.get<double>("goal_bias"))                   m_cfg.goal_bias = *v;
    if (auto v = s.get<double>("selection_radius"))            m_cfg.selection_radius = *v;
    if (auto v = s.get<double>("pruning_radius"))              m_cfg.pruning_radius = *v;
    if (auto v = s.get<std::string>("robot_type"))             m_cfg.robot_type = *v;
    if (auto v = s.get<double>("step_dt_s"))                   if (*v>0.0) m_cfg.step_dt_s = *v;
    if (auto v = s.get<int>("takeK"))                          m_cfg.takeK = *v;
    if (auto v = s.get<double>("agent_radius"))                m_agent_radius = *v; // reserved
    if (auto v = s.get<double>("agent_size"))                  m_agent_radius = *v; // alias

    // string fallbacks
    if (auto v = s.get<std::string>("timelimit_s"))            if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("cutoff_s"))               if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("keepTemps"))              if (auto x=parseBool(*v))   m_cfg.keepTemps=*x;
    if (auto v = s.get<std::string>("propagation_step_size"))  if (auto x=parseDouble(*v)) m_cfg.propagation_step_size=*x;
    if (auto v = s.get<std::string>("control_duration_min"))   if (auto x=parseInt(*v))    m_cfg.control_duration_min=*x;
    if (auto v = s.get<std::string>("control_duration_max"))   if (auto x=parseInt(*v))    m_cfg.control_duration_max=*x;
    if (auto v = s.get<std::string>("goal_epsilon"))           if (auto x=parseDouble(*v)) m_cfg.goal_epsilon=*x;
    if (auto v = s.get<std::string>("goal_bias"))              if (auto x=parseDouble(*v)) m_cfg.goal_bias=*x;
    if (auto v = s.get<std::string>("selection_radius"))       if (auto x=parseDouble(*v)) m_cfg.selection_radius=*x;
    if (auto v = s.get<std::string>("pruning_radius"))         if (auto x=parseDouble(*v)) m_cfg.pruning_radius=*x;
    if (auto v = s.get<std::string>("step_dt_s"))              if (auto x=parseDouble(*v)) if (*x>0.0) m_cfg.step_dt_s=*x;
    if (auto v = s.get<std::string>("takeK"))                  if (auto x=parseInt(*v))    m_cfg.takeK=*x;
    if (auto v = s.get<std::string>("agent_radius"))           if (auto x=parseDouble(*v)) m_agent_radius=*x;
    if (auto v = s.get<std::string>("agent_size"))             if (auto x=parseDouble(*v)) m_agent_radius=*x;

    // YAML ingestion (robot_meta / obstacles)
    auto parseRobotMetaYaml = [&](const std::string& yamlStr){
        using ::Logger;
        try {
            YAML::Node root = YAML::Load(yamlStr);
            m_robot_meta.clear();
            if (!root || !root.IsSequence()) {
                Logger::instance().warn("SST (bin): robot_meta(_yaml) is not a sequence; ignoring.");
                return;
            }
            m_robot_meta.reserve(root.size());
            for (const auto& r : root) {
                SST_RobotMeta rm;
                if (r && r.IsMap()) {
                    if (auto ty=r["type"]; ty && ty.IsScalar()) rm.type = ty.as<std::string>();
                    if (auto fp=r["footprint"]; fp && fp.IsMap()) {
                        auto s = fp["shape"];
                        if (s && s.IsScalar()) {
                            rm.fp.shape = s.as<std::string>();
                            if (rm.fp.shape == "circle") {
                                if (auto rad=fp["radius"]; rad && rad.IsScalar()) {
                                    rm.fp.radius = rad.as<double>(); rm.fp.valid = true;
                                }
                            } else if (rm.fp.shape == "box") {
                                auto sz=fp["size"];
                                if (sz && sz.IsSequence() && sz.size()>=2) {
                                    rm.fp.size_x=sz[0].as<double>();
                                    rm.fp.size_y=sz[1].as<double>();
                                    rm.fp.valid = true;
                                }
                            } else if (rm.fp.shape == "multibox") {
                                auto boxes=fp["boxes"];
                                if (boxes && boxes.IsSequence()) {
                                    for (const auto& b : boxes) {
                                        SST_RobotMeta::Part p{};
                                        if (auto c=b["center"]; c && c.IsSequence() && c.size()>=2) { p.cx=c[0].as<double>(); p.cy=c[1].as<double>(); }
                                        if (auto s2=b["size"];   s2 && s2.IsSequence() && s2.size()>=2) { p.sx=s2[0].as<double>(); p.sy=s2[1].as<double>(); }
                                        p.angle = (b["angle"] && b["angle"].IsScalar()) ? b["angle"].as<double>() : 0.0;
                                        rm.fp.boxes.push_back(p);
                                    }
                                    if (!rm.fp.boxes.empty()) rm.fp.valid = true;
                                }
                            }
                        }
                    }
                }
                m_robot_meta.push_back(std::move(rm));
            }
        } catch (const std::exception& e) {
            Logger::instance().warn(std::string("SST (bin): failed to parse robot_meta yaml: ")+e.what());
            m_robot_meta.clear();
        }
    };
    auto parseObstaclesYaml = [&](const std::string& yamlStr){
        using ::Logger;
        try {
            YAML::Node root = YAML::Load(yamlStr);
            m_env_obs.clear();
            if (!root || !root.IsSequence()) {
                Logger::instance().warn("SST (bin): obstacles(_yaml) is not a sequence; ignoring.");
                return;
            }
            for (const auto& o : root) {
                if (!o || !o.IsMap()) continue;
                DBCBSObstacle obs;
                obs.type = o["type"] ? o["type"].as<std::string>() : "";
                if (obs.type == "box") {
                    double x=0,y=0,w=0,h=0;
                    if (asPair2(o["center"], x, y) && asPair2(o["size"], w, h)) {
                        obs.cx=x; obs.cy=y; obs.sx=w; obs.sy=h; m_env_obs.push_back(obs);
                    }
                } else if (obs.type == "circle") {
                    double x=0,y=0; double r=0;
                    if (asPair2(o["center"], x, y) && o["radius"] && o["radius"].IsScalar()) {
                        obs.cx=x; obs.cy=y; obs.radius=o["radius"].as<double>(); m_env_obs.push_back(obs);
                    }
                } else if (obs.type == "capsule") {
                    double p0x=0,p0y=0,p1x=0,p1y=0;
                    if (asPair2(o["p0"], p0x, p0y) && asPair2(o["p1"], p1x, p1y)) {
                        obs.p0x=p0x; obs.p0y=p0y; obs.p1x=p1x; obs.p1y=p1y;
                        obs.radius = (o["radius"] && o["radius"].IsScalar()) ? o["radius"].as<double>() : 0.0;
                        m_env_obs.push_back(obs);
                    }
                } else if (obs.type == "polygon") {
                    obs.polygon.clear();
                    if (auto pts=o["points"]; pts && pts.IsSequence()) {
                        for (const auto& p : pts) { double px=0,py=0; if (asPair2(p, px, py)) obs.polygon.push_back({px,py}); }
                        if (obs.polygon.size() >= 3) m_env_obs.push_back(obs);
                    }
                } else {
                    Logger::instance().warn("SST (bin): unknown obstacle type; skipped.");
                }
            }
        } catch (const std::exception& e) {
            Logger::instance().warn(std::string("SST (bin): failed to parse obstacles yaml: ")+e.what());
            m_env_obs.clear();
        }
    };

    if (auto v = s.get<std::string>("robot_meta"))       parseRobotMetaYaml(*v);
    if (auto v = s.get<std::string>("robot_meta_yaml"))  parseRobotMetaYaml(*v);
    if (auto v = s.get<std::string>("obstacles"))        parseObstaclesYaml(*v);
    if (auto v = s.get<std::string>("obstacles_yaml"))   parseObstaclesYaml(*v);
}

// ---------------- Writers / Parser ----------------
void SST_Binary::writeProblemYaml(const BaseEnvironment& E,
                                  const std::vector<StartGoal>& sgs,
                                  const fs::path& outYaml)
{
    auto seq2 = [](double a, double b){ YAML::Node n(YAML::NodeType::Sequence); n.SetStyle(YAML::EmitterStyle::Flow); n.push_back(a); n.push_back(b); return n; };
    auto vecN = [](const std::vector<double>& v){ YAML::Node n(YAML::NodeType::Sequence); n.SetStyle(YAML::EmitterStyle::Flow); for (double x:v) n.push_back(x); return n; };

    YAML::Node doc;
    const auto [W,H] = E.boundsInMeter();

    YAML::Node env(YAML::NodeType::Map);
    env["min"] = seq2(0.0, 0.0);
    env["max"] = seq2(W, H);

    YAML::Node obstacles(YAML::NodeType::Sequence);
    for (const auto& o : m_env_obs) {
        YAML::Node n(YAML::NodeType::Map);
        n["type"] = o.type;
        if (o.type == "box") {
            n["center"] = seq2(o.cx, o.cy);
            n["size"]   = seq2(o.sx, o.sy);
        } else if (o.type == "circle") {
            n["center"] = seq2(o.cx, o.cy);
            n["radius"] = o.radius;
        } else if (o.type == "capsule") {
            n["p0"]     = seq2(o.p0x, o.p0y);
            n["p1"]     = seq2(o.p1x, o.p1y);
            n["radius"] = o.radius;
        } else if (o.type == "polygon") {
            YAML::Node pts(YAML::NodeType::Sequence);
            for (const auto& p : o.polygon) pts.push_back(seq2(p[0], p[1]));
            n["points"] = pts;
        } else {
            continue;
        }
        obstacles.push_back(n);
    }
    env["obstacles"]     = obstacles;
    doc["environment"]   = env;

    YAML::Node robots(YAML::NodeType::Sequence);
    robots.SetStyle(YAML::EmitterStyle::Block);

    auto defaultDimForType = [](const std::string& ty){
        if (ty.find("double_integrator") != std::string::npos) return 4;
        if (ty.find("unicycle") != std::string::npos)          return 3;
        if (ty.find("car_") != std::string::npos)              return 3;
        return 2;
    };

    for (size_t i=0; i<sgs.size(); ++i) {
        YAML::Node r(YAML::NodeType::Map);
        std::string ty = m_cfg.robot_type;
        if (i < m_robot_meta.size() && !m_robot_meta[i].type.empty())
            ty = m_robot_meta[i].type;

        r["type"] = ty;
        const int dim = defaultDimForType(ty);

        std::vector<double> s(dim, 0.0), g(dim, 0.0);
        s[0] = sgs[i].start.x; s[1] = sgs[i].start.y;
        g[0] = sgs[i].goal.x;  g[1] = sgs[i].goal.y;
        r["start"] = vecN(s);
        r["goal"]  = vecN(g);

        if (i < m_robot_meta.size() && m_robot_meta[i].fp.valid) {
            const auto& fp = m_robot_meta[i].fp;
            YAML::Node f(YAML::NodeType::Map);
            f["shape"] = fp.shape;
            if (fp.shape=="circle") {
                f["radius"] = fp.radius;
            } else if (fp.shape=="box") {
                f["size"] = vecN({fp.size_x, fp.size_y});
            } else if (fp.shape=="multibox") {
                YAML::Node boxes(YAML::NodeType::Sequence);
                for (const auto& p : fp.boxes) {
                    YAML::Node b(YAML::NodeType::Map);
                    b["center"] = vecN({p.cx, p.cy});
                    b["size"]   = vecN({p.sx, p.sy});
                    b["angle"]  = p.angle;
                    boxes.push_back(b);
                }
                f["boxes"] = boxes;
            }
            r["footprint"] = f;
        } else {
            // mild defaults
            if (ty.find("double_integrator") != std::string::npos) {
                YAML::Node f; f["shape"]="circle"; f["radius"]=0.15; r["footprint"]=f;
            } else if (ty.find("unicycle_first_order_0_sphere") != std::string::npos) {
                YAML::Node f; f["shape"]="circle"; f["radius"]=0.40; r["footprint"]=f;
            } else if (ty.find("unicycle") != std::string::npos || ty.find("car_") != std::string::npos) {
                YAML::Node f; f["shape"]="box"; f["size"]=vecN({0.5,0.25}); r["footprint"]=f;
            }
        }

        robots.push_back(r);
    }
    doc["robots"] = robots;

    YAML::Emitter em; em << doc;
    std::ofstream y(outYaml);
    if (!y) throw std::runtime_error("SST (bin): failed to write problem yaml");
    y << em.c_str() << std::endl;
}

void SST_Binary::writeCfgYaml(const SST_BinaryConfig& c, const fs::path& outYaml) const {
    std::ofstream y(outYaml);
    if (!y) throw std::runtime_error("SST (bin): failed to write cfg yaml");
    y <<
      "propagation_step_size: " << c.propagation_step_size << "\n"
      "control_duration: [" << c.control_duration_min << ", " << c.control_duration_max << "]\n"
      "goal_epsilon: " << c.goal_epsilon << "\n"
      "goal_bias: " << c.goal_bias << "\n";
    if (c.planner == "sst") {
        y << "selection_radius: " << c.selection_radius << "\n"
          << "pruning_radius: "   << c.pruning_radius   << "\n";
    }
    y.flush();
}

bool SST_Binary::parseSolutionYaml(const fs::path& file,
                                   const double step_dt,
                                   std::vector<Plan>& outPlans)
{
    YAML::Node root = YAML::LoadFile(file.string());
    if (!root) return false;
    auto result = root["result"];
    if (!result || !result.IsSequence() || result.size()==0) return false;

    outPlans.clear();
    outPlans.resize(result.size());

    double dt = step_dt;
    if (root["dt"] && root["dt"].IsScalar()) {
        try { dt = root["dt"].as<double>(); } catch (...) {}
    }

    for (size_t i=0;i<result.size();++i) {
        const auto& agent = result[i];
        const auto states = agent["states"];
        if (!states || !states.IsSequence() || states.size()==0) return false;

        std::vector<double> times;
        if (agent["times"] && agent["times"].IsSequence()) {
            times.reserve(states.size());
            for (const auto& t: agent["times"]) {
                if (!t.IsScalar()) { times.clear(); break; }
                times.push_back(t.as<double>());
            }
            if (times.size()!=states.size()) times.clear();
        }

        Plan P;
        P.waypoints.reserve(states.size());
        for (size_t k=0;k<states.size();++k) {
            const auto& s = states[k];
            if (!s.IsSequence() || s.size()<2) return false;
            const double x = s[0].as<double>();
            const double y = s[1].as<double>();
            const float  t = times.empty() ? static_cast<float>(k*dt)
                                           : static_cast<float>(times[k]);
            P.waypoints.push_back(Waypoint{ b2Vec2{static_cast<float>(x), static_cast<float>(y)}, t });
        }
        outPlans[i] = std::move(P);
    }
    return true;
}

// ---------------- Plan ----------------
PlanResult SST_Binary::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids    = req.ids;
    out.status = PlanStatus::Error;

    if (!req.env.continuousCapability()) {
        Logger::instance().error("SST (bin): Environment does not support IContinuousSpace.");
        return out;
    }
    if (req.ids.size() != req.sgs.size()) { Logger::instance().error("SST (bin): ids.size() != sgs.size()."); return out; }

    SST_BinaryConfig cfg = m_cfg;
    SST_Binary shadow(cfg);
    // carry over preconfigured metadata / env to the shadow
    shadow.m_robot_meta   = this->m_robot_meta;
    shadow.m_env_obs      = this->m_env_obs;
    shadow.m_agent_radius = this->m_agent_radius;
    // apply per-request overrides
    shadow.configure(req.settings);

    // Determine time limit
    const double timelimit_s = resolvePlannerTimeLimit(
        req.time_limit_s, shadow.m_cfg.timelimit_s);

    // Optional cap
    std::vector<StartGoal> sgs_use = req.sgs;
    if (shadow.m_cfg.takeK && *shadow.m_cfg.takeK > 0 && *shadow.m_cfg.takeK < static_cast<int>(sgs_use.size()))
        sgs_use.resize(static_cast<size_t>(*shadow.m_cfg.takeK));

    // If no robot_meta provided but agent_radius/agent_size is set, synthesize circle footprints.
    if (shadow.m_robot_meta.empty() && shadow.m_agent_radius && *shadow.m_agent_radius > 0.0) {
        shadow.m_robot_meta.assign(sgs_use.size(), {});
        for (auto& r : shadow.m_robot_meta) {
            r.type       = shadow.m_cfg.robot_type;
            r.fp.shape   = "circle";
            r.fp.radius  = *shadow.m_agent_radius;
            r.fp.valid   = true;
        }
    }

    util::TempDirGuard tmp(fs::temp_directory_path(), "sst_", shadow.m_cfg.keepTemps);
    const fs::path base = tmp.path();

    const fs::path problemYaml = base / "problem.yaml";
    const fs::path cfgYaml     = base / "cfg.yaml";
    const fs::path outYaml     = base / "solution.yaml";
    const fs::path stdoutTxt   = base / "stdout.txt";
    const fs::path stderrTxt   = base / "stderr.txt";
    const fs::path statsYaml   = base / "stats.yaml";

    // Build IO
    try {
        shadow.writeProblemYaml(req.env, sgs_use, problemYaml);
        shadow.writeCfgYaml(shadow.m_cfg, cfgYaml);
    } catch (const std::exception& e) {
        Logger::instance().error(e.what());
        return out;
    }

    // Run
    std::vector<std::string> args = {
        "--input",   problemYaml.string(),
        "--output",  outYaml.string(),
        "--cfg",     cfgYaml.string(),
        "--planner", shadow.m_cfg.planner,
        "--timelimit", std::to_string(static_cast<int>(timelimit_s)),
        "--stats",   statsYaml.string()
    };

    const auto t0 = std::chrono::steady_clock::now();
    const auto timeout_ms = std::chrono::milliseconds(static_cast<long>(timelimit_s*1000.0) + 2000);
    const auto res = util::runWithTimeout(shadow.m_cfg.binary, args, base, stdoutTxt, stderrTxt, timeout_ms);
    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (res.timed_out) {
        Logger::instance().warn("SST (bin): planner timed out.");
        out.status = PlanStatus::Timeout;
        if (shadow.m_cfg.keepTemps) { Logger::instance().info("SST (bin): kept temp dir: " + base.string()); tmp.set_keep(true); }
        return out;
    }
    if (res.exit_code != 0) {
        Logger::instance().warn(fmt::format("SST (bin): exit code {}. See {}", res.exit_code, stderrTxt.string()));
        // keep parsing attempt below
        if (shadow.m_cfg.keepTemps) { Logger::instance().info("SST (bin): kept temp dir: " + base.string()); tmp.set_keep(true); }
    }

    // Parse
    std::vector<Plan> plans;
    bool ok = false;
    try { ok = parseSolutionYaml(outYaml, shadow.m_cfg.step_dt_s, plans); } catch (...) { ok = false; }
    if (!ok) {
        Logger::instance().warn("SST (bin): failed to parse solution.yaml");
        out.status = PlanStatus::Error;
        if (shadow.m_cfg.keepTemps) { Logger::instance().info("SST (bin): kept temp dir: " + base.string()); tmp.set_keep(true); }
        return out;
    }

    // Trim/match ids
    if (plans.size() != sgs_use.size()) plans.resize(std::min(plans.size(), sgs_use.size()));
    out.plans = std::move(plans);
    out.ids.resize(out.plans.size());
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;

    if (shadow.m_cfg.keepTemps) {
        Logger::instance().info("SST (bin): kept temp dir: " + base.string());
        tmp.set_keep(true);
    }
    return out;
}
