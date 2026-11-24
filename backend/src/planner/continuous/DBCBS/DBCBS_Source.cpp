// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/continuous/DBCBS/DBCBS_Source.h"
#include "../inc/planner/continuous/DBCBS/DBCBS_IO.h"

#include "../inc/util/Logger.h"
#include "../inc/util/TempDirGuard.h"

#include <yaml-cpp/yaml.h>
#include <fmt/format.h>
#include <algorithm>
#include <chrono>
#include <charconv>

// external db-cbs core lib (from external repo)
#include <../lib/dbcbs_core.hpp>

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
    try { return std::stod(s); } catch(...) { return std::nullopt; }
}
template <typename T>
static bool asPair2(const YAML::Node& n, T& x, T& y) {
    if (!n || !n.IsSequence() || n.size()<2) return false;
    try { x=n[0].as<T>(); y=n[1].as<T>(); } catch (...) { return false; }
    return true;
}
} // namespace

// ---------------- settings -> configure ----------------
void DBCBS_Source::configure(const PlannerSettings& s)
{
    if (auto v=s.get<double>("timelimit_s"))                m_cfg.timelimit_s=*v;
    if (auto v=s.get<double>("cutoff_s"))                   m_cfg.timelimit_s=*v; // alias
    if (auto v=s.get<double>("step_dt_s"))                  if (*v>0.0) m_cfg.step_dt_s=*v;
    if (auto v=s.get<std::string>("robot_type"))            m_cfg.robot_type=*v;
    if (auto v=s.get<bool>("keepTemps"))                    m_cfg.keepTemps=*v;


    if (auto v=s.get<double>("alpha"))                      m_cfg.alpha=*v;
    if (auto v=s.get<bool>("filter_duplicates"))            m_cfg.filter_duplicates=*v;
    if (auto v=s.get<std::string>("heuristic1"))            m_cfg.heuristic1=*v;
    if (auto v=s.get<double>("heuristic1_delta"))           m_cfg.heuristic1_delta=*v;
    if (auto v=s.get<double>("delta0"))                     m_cfg.delta0=*v;
    if (auto v=s.get<int>("num_primitives0"))               m_cfg.num_primitives0=*v;
    if (auto v=s.get<double>("delta_rate"))                 m_cfg.delta_rate=*v;
    if (auto v=s.get<double>("num_primitives_rate"))        m_cfg.num_primitives_rate=*v;

    if (auto v=s.get<bool>("useOptimizer"))                 m_cfg.useOptimizer=*v;
    if (auto v=s.get<bool>("optSumRobotCost"))              m_cfg.optSumRobotCost=*v;

    if (auto v=s.get<int>("takeK"))                         m_cfg.takeK=*v;
    if (auto v=s.get<double>("agent_radius"))               m_agent_radius=*v;
    if (auto v=s.get<double>("agent_size"))                 m_agent_radius=*v;

    // string fallbacks
    if (auto v=s.get<std::string>("timelimit_s"))           if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v=s.get<std::string>("cutoff_s"))              if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v=s.get<std::string>("step_dt_s"))             if (auto x=parseDouble(*v)) if (*x>0.0) m_cfg.step_dt_s=*x;
    if (auto v=s.get<std::string>("takeK"))                 if (auto x=parseInt(*v))    m_cfg.takeK=*x;
    if (auto v=s.get<std::string>("keepTemps"))             if (auto x=parseBool(*v))   m_cfg.keepTemps=*x;

    if (auto v=s.get<std::string>("alpha"))                 if (auto x=parseDouble(*v)) m_cfg.alpha=*x;
    if (auto v=s.get<std::string>("filter_duplicates"))     if (auto x=parseBool(*v))   m_cfg.filter_duplicates=*x;
    if (auto v=s.get<std::string>("heuristic1_delta"))      if (auto x=parseDouble(*v)) m_cfg.heuristic1_delta=*x;
    if (auto v=s.get<std::string>("delta0"))                if (auto x=parseDouble(*v)) m_cfg.delta0=*x;
    if (auto v=s.get<std::string>("num_primitives0"))       if (auto x=parseInt(*v))    m_cfg.num_primitives0=*x;
    if (auto v=s.get<std::string>("delta_rate"))            if (auto x=parseDouble(*v)) m_cfg.delta_rate=*x;
    if (auto v=s.get<std::string>("num_primitives_rate"))   if (auto x=parseDouble(*v)) m_cfg.num_primitives_rate=*x;

    if (auto v=s.get<std::string>("useOptimizer"))          if (auto x=parseBool(*v))   m_cfg.useOptimizer=*x;
    if (auto v=s.get<std::string>("optSumRobotCost"))       if (auto x=parseBool(*v))   m_cfg.optSumRobotCost=*x;

    // robot_meta YAML (parity with binary)
    if (auto v=s.get<std::string>("robot_meta")) {
        try {
            YAML::Node root = YAML::Load(*v);
            m_robot_meta.clear();
            if (root && root.IsSequence()) {
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
                                        for (const auto& b : boxes) {
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
            }
        } catch (const std::exception& e) {
            Logger::instance().warn(std::string("DBCBS (src): failed to parse robot_meta yaml: ")+e.what());
            m_robot_meta.clear();
        }
    }
    if (auto v=s.get<std::string>("robot_meta_yaml")) { PlannerSettings t; t.kv.emplace("robot_meta", *v); configure(t); }

    // obstacles (optional)
    if (auto v=s.get<std::string>("obstacles")) {
        try {
            YAML::Node root = YAML::Load(*v);
            m_env_obs.clear();
            if (root && root.IsSequence()) {
                for (const auto& o : root) {
                    if (!o || !o.IsMap()) continue;
                    DBCBSObstacle ob;
                    ob.type = o["type"] ? o["type"].as<std::string>() : "";
                    if (ob.type=="box") {
                        double x=0,y=0,w=0,h=0;
                        if (asPair2(o["center"],x,y) && asPair2(o["size"],w,h)) { ob.cx=x; ob.cy=y; ob.sx=w; ob.sy=h; m_env_obs.push_back(ob); }
                    } else if (ob.type=="circle") {
                        double x=0,y=0; double r=0;
                        if (asPair2(o["center"],x,y) && o["radius"] && o["radius"].IsScalar()) { ob.cx=x; ob.cy=y; ob.radius=o["radius"].as<double>(); m_env_obs.push_back(ob); }
                    } else if (ob.type=="capsule") {
                        double p0x=0,p0y=0,p1x=0,p1y=0;
                        if (asPair2(o["p0"],p0x,p0y) && asPair2(o["p1"],p1x,p1y)) {
                            ob.p0x=p0x; ob.p0y=p0y; ob.p1x=p1x; ob.p1y=p1y;
                            ob.radius=(o["radius"]&&o["radius"].IsScalar())?o["radius"].as<double>():0.0;
                            m_env_obs.push_back(ob);
                        }
                    } else if (ob.type=="polygon") {
                        ob.polygon.clear();
                        if (auto pts=o["points"]; pts && pts.IsSequence()) {
                            for (const auto& p : pts) { double px=0,py=0; if (asPair2(p,px,py)) ob.polygon.push_back({px,py}); }
                            if (ob.polygon.size()>=3) m_env_obs.push_back(ob);
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            Logger::instance().warn(std::string("DBCBS (src): failed to parse obstacles yaml: ")+e.what());
            m_env_obs.clear();
        }
    }
    if (auto v=s.get<std::string>("obstacles_yaml")) { PlannerSettings t; t.kv.emplace("obstacles", *v); configure(t); }
}

// ---------------- entry ----------------
PlanResult DBCBS_Source::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids    = req.ids;
    out.status = PlanStatus::Error;

    if (!req.env.continuousCapability()) {
        Logger::instance().error("DBCBS (src): Environment does not support IContinuousSpace.");
        return out;
    }
    if (req.ids.size() != req.sgs.size()) { Logger::instance().error("DBCBS (src): ids.size() != sgs.size()."); return out; }

    // per-call overrides via shadow
    DBCBS_SourceConfig base = m_cfg;
    DBCBS_Source shadow(base);

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

    util::TempDirGuard tmp(fs::temp_directory_path(), "dbcbs_src_", shadow.m_cfg.keepTemps);
    const fs::path baseDir     = tmp.path();
    const fs::path problemYaml = baseDir / "problem.yaml";
    const fs::path solutionYaml= baseDir / "solution.yaml";
    const fs::path jointYaml   = baseDir / "joint.yaml";
    const fs::path optYaml     = baseDir / "opt.yaml";

    // 1) Build in-memory problem for library call
    dbcbs::Environment Denv{};
    const auto [W,H] = req.env.boundsInMeter();
    Denv.minx = 0.f; Denv.miny = 0.f; Denv.maxx = static_cast<float>(W); Denv.maxy = static_cast<float>(H);

    // Serialize typed obstacles as YAML string for core lib
    {
        YAML::Node obs(YAML::NodeType::Sequence);
        auto seq2 = [](double a, double b){ YAML::Node n(YAML::NodeType::Sequence); n.SetStyle(YAML::EmitterStyle::Flow); n.push_back(a); n.push_back(b); return n; };
        for (const auto& o : shadow.m_env_obs) {
            YAML::Node n(YAML::NodeType::Map);
            n["type"] = o.type;
            if (o.type=="box")      { n["center"]=seq2(o.cx,o.cy); n["size"]=seq2(o.sx,o.sy); }
            else if (o.type=="circle"){ n["center"]=seq2(o.cx,o.cy); n["radius"]=o.radius; }
            else if (o.type=="capsule"){ n["p0"]=seq2(o.p0x,o.p0y); n["p1"]=seq2(o.p1x,o.p1y); n["radius"]=o.radius; }
            else if (o.type=="polygon") {
                YAML::Node pts(YAML::NodeType::Sequence);
                for (const auto& p : o.polygon) pts.push_back(seq2(p[0],p[1]));
                n["points"]=pts;
            } else continue;
            obs.push_back(n);
        }
        YAML::Emitter em; em << obs;
        Denv.obstacles_yaml = em.c_str();
        Denv.obstacles.clear(); // don't mix legacy
    }

    auto dimForType = [](const std::string& ty){
        if (ty.find("double_integrator") != std::string::npos) return 4;
        if (ty.find("unicycle") != std::string::npos)          return 3;
        if (ty.find("car_") != std::string::npos)              return 3;
        return 2;
    };

    std::vector<dbcbs::RobotSpec> Drobots(sgs_use.size());
    for (size_t i=0;i<sgs_use.size();++i) {
        const auto& sg = sgs_use[i];
        std::string ty = shadow.m_cfg.robot_type;
        if (i < shadow.m_robot_meta.size() && !shadow.m_robot_meta[i].type.empty()) ty = shadow.m_robot_meta[i].type;

        Drobots[i].type = ty;

        // footprint
        if (i < shadow.m_robot_meta.size() && shadow.m_robot_meta[i].fp.valid) {
            const auto& fp = shadow.m_robot_meta[i].fp;
            if (fp.shape=="circle" && fp.radius>0.0) {
                Drobots[i].fp.kind = dbcbs::Footprint::Circle; Drobots[i].fp.radius = static_cast<float>(fp.radius);
            } else if (fp.shape=="box" && fp.size_x>0.0 && fp.size_y>0.0) {
                Drobots[i].fp.kind = dbcbs::Footprint::Box;
                dbcbs::Footprint::Part p{0,0,(float)fp.size_x,(float)fp.size_y,0};
                Drobots[i].fp.parts = {p};
            } else if (fp.shape=="multibox" && !fp.boxes.empty()) {
                Drobots[i].fp.kind = dbcbs::Footprint::MultiBox;
                Drobots[i].fp.parts.reserve(fp.boxes.size());
                for (const auto& b : fp.boxes)
                    Drobots[i].fp.parts.push_back(dbcbs::Footprint::Part{(float)b.cx,(float)b.cy,(float)b.sx,(float)b.sy,(float)b.angle});
            } else if (shadow.m_agent_radius && *shadow.m_agent_radius > 0.0) {
            // synthesize simple circular footprint
            Drobots[i].fp.kind   = dbcbs::Footprint::Circle;
            Drobots[i].fp.radius = static_cast<float>(*shadow.m_agent_radius);
         }
        }

        const int dim = dimForType(ty);
        Drobots[i].start.assign(static_cast<size_t>(dim), 0.0);
        Drobots[i].goal .assign(static_cast<size_t>(dim), 0.0);
        Drobots[i].start[0]=sg.start.x; Drobots[i].start[1]=sg.start.y;
        Drobots[i].goal [0]=sg.goal.x;  Drobots[i].goal [1]=sg.goal.y;
    }

    dbcbs::Settings C{};
    C.alpha               = static_cast<float>(shadow.m_cfg.alpha);
    C.filter_duplicates   = shadow.m_cfg.filter_duplicates;
    C.heuristic1          = shadow.m_cfg.heuristic1;
    C.heuristic1_delta    = static_cast<float>(shadow.m_cfg.heuristic1_delta);
    C.delta0              = static_cast<float>(shadow.m_cfg.delta0);
    C.num_primitives0     = shadow.m_cfg.num_primitives0;
    C.delta_rate          = static_cast<float>(shadow.m_cfg.delta_rate);
    C.num_primitives_rate = static_cast<float>(shadow.m_cfg.num_primitives_rate);
    C.timelimit_s         = timelimit_s;

    // 2) Run core solver
    const auto t0 = std::chrono::steady_clock::now();
    dbcbs::Solution S;
    const bool ok = dbcbs::solve(Denv, Drobots, C, S);
    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!ok || S.agents.empty()) {
        Logger::instance().warn("DBCBS (src): solver failed or returned empty solution.");
        out.status = (out.diag.runtime_s >= timelimit_s && timelimit_s > 0) ? PlanStatus::Timeout : PlanStatus::Error;
        return out;
    }

    // 3) Emit artifacts (for optional optimizer / debugging)
    dbcbs_writeProblemYaml(req.env, sgs_use, shadow.m_robot_meta, shadow.m_cfg.robot_type, shadow.m_env_obs, problemYaml);
    dbcbs::export_solution_yaml(S, Drobots, solutionYaml.string());
    dbcbs::export_joint_yaml(S, Drobots,   jointYaml.string());

    // 4) Try optimizer first (if enabled), then fall back to solution.yaml
    std::vector<Plan> plans;
    auto try_parse = [&](const fs::path& p)->bool {
        try { return dbcbs_parseSolutionYaml(p, shadow.m_cfg.step_dt_s, plans); }
        catch (...) { return false; }
    };

    bool parsed = false;
    if (shadow.m_cfg.useOptimizer) {
        const bool opt_ok = dbcbs::optimize_to_yaml(
            problemYaml.string(), solutionYaml.string(), optYaml.string(), shadow.m_cfg.optSumRobotCost);

        std::error_code ec;
        const bool present = fs::exists(optYaml, ec) && !ec && fs::file_size(optYaml, ec) > 0 && !ec;

        if (opt_ok && present && try_parse(optYaml)) {
            Logger::instance().info("DBCBS (src): using opt.yaml");
            parsed = true;
        } else {
            Logger::instance().warn("DBCBS (src): optimizer unavailable/failed; using raw solution if possible.");
        }
    }
    if (!parsed) {
        if (try_parse(solutionYaml)) {
            Logger::instance().info("DBCBS (src): using solution.yaml");
            parsed = true;
        }
    }

    if (!parsed) {
        // last resort: convert in-memory solution
        plans.resize(S.agents.size());
        for (size_t i=0;i<S.agents.size();++i) {
            auto& P = plans[i];
            float t = 0.f;
            for (const auto& st : S.agents[i].states) {
                const float x = st.size()>=1 ? static_cast<float>(st[0]) : 0.f;
                const float y = st.size()>=2 ? static_cast<float>(st[1]) : 0.f;
                P.waypoints.push_back(Waypoint{ b2Vec2{x,y}, t });
                t += static_cast<float>(shadow.m_cfg.step_dt_s > 0.0 ? shadow.m_cfg.step_dt_s : 0.1);
            }
        }
        parsed = !plans.empty();
    }

    if (!parsed) {
        out.status = PlanStatus::Error;
        return out;
    }

    if (plans.size() != sgs_use.size())
        plans.resize(std::min(plans.size(), sgs_use.size()));

    out.plans = std::move(plans);
    out.ids.resize(out.plans.size());
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;
    if (shadow.m_cfg.keepTemps) { Logger::instance().info("DBCBS (src): kept temp dir: " + baseDir.string()); tmp.set_keep(true); }
    return out;
}
