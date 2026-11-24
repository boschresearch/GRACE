// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/continuous/SST/SST_Source.h"
#include "../inc/util/Logger.h"

#include <yaml-cpp/yaml.h>
#include <charconv>
#include <algorithm>
#include <chrono>

// include the external API
#include <sst_core.hpp>

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

// ---------------- Configure ----------------
void SST_Source::configure(const PlannerSettings& s) {
    if (auto v = s.get<std::string>("planner"))               m_cfg.planner = *v;
    if (auto v = s.get<double>("timelimit_s"))                m_cfg.timelimit_s = *v;
    if (auto v = s.get<double>("cutoff_s"))                   m_cfg.timelimit_s = *v; // alias
    if (auto v = s.get<double>("propagation_step_size"))      m_cfg.propagation_step_size = *v;
    if (auto v = s.get<int>("control_duration_min"))          m_cfg.control_duration_min  = *v;
    if (auto v = s.get<int>("control_duration_max"))          m_cfg.control_duration_max  = *v;
    if (auto v = s.get<double>("goal_epsilon"))               m_cfg.goal_epsilon = *v;
    if (auto v = s.get<double>("goal_bias"))                  m_cfg.goal_bias = *v;
    if (auto v = s.get<double>("selection_radius"))           m_cfg.selection_radius = *v;
    if (auto v = s.get<double>("pruning_radius"))             m_cfg.pruning_radius = *v;
    if (auto v = s.get<std::string>("robot_type"))            m_cfg.robot_type = *v;
    if (auto v = s.get<double>("step_dt_s"))                  if (*v>0.0) m_cfg.step_dt_s = *v;
    if (auto v = s.get<int>("takeK"))                         m_cfg.takeK = *v;
    if (auto v = s.get<double>("agent_radius"))               m_agent_radius = *v; // reserved
    if (auto v = s.get<double>("agent_size"))                 m_agent_radius = *v; // alias

    // string fallbacks
    if (auto v = s.get<std::string>("timelimit_s"))           if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("cutoff_s"))              if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("propagation_step_size")) if (auto x=parseDouble(*v)) m_cfg.propagation_step_size=*x;
    if (auto v = s.get<std::string>("control_duration_min"))  if (auto x=parseInt(*v))    m_cfg.control_duration_min=*x;
    if (auto v = s.get<std::string>("control_duration_max"))  if (auto x=parseInt(*v))    m_cfg.control_duration_max=*x;
    if (auto v = s.get<std::string>("goal_epsilon"))          if (auto x=parseDouble(*v)) m_cfg.goal_epsilon=*x;
    if (auto v = s.get<std::string>("goal_bias"))             if (auto x=parseDouble(*v)) m_cfg.goal_bias=*x;
    if (auto v = s.get<std::string>("selection_radius"))      if (auto x=parseDouble(*v)) m_cfg.selection_radius=*x;
    if (auto v = s.get<std::string>("pruning_radius"))        if (auto x=parseDouble(*v)) m_cfg.pruning_radius=*x;
    if (auto v = s.get<std::string>("step_dt_s"))             if (auto x=parseDouble(*v)) if (*x>0.0) m_cfg.step_dt_s=*x;
    if (auto v = s.get<std::string>("takeK"))                 if (auto x=parseInt(*v))    m_cfg.takeK=*x;

    // robot_meta YAML (parity with binary)
    if (auto v = s.get<std::string>("robot_meta")) {
        try {
            YAML::Node root = YAML::Load(*v);
            m_robot_meta.clear();
            if (root && root.IsSequence()) {
                m_robot_meta.reserve(root.size());
                for (const auto& r : root) {
                    Meta m{};
                    if (r && r.IsMap()) {
                        if (auto ty=r["type"]; ty && ty.IsScalar()) m.type = ty.as<std::string>();
                        if (auto fp=r["footprint"]; fp && fp.IsMap()) {
                            auto s = fp["shape"];
                            if (s && s.IsScalar()) {
                                m.fp.shape = s.as<std::string>();
                                if (m.fp.shape=="circle") {
                                    if (auto rad=fp["radius"]; rad && rad.IsScalar()) { m.fp.radius=rad.as<double>(); m.fp.valid=true; }
                                } else if (m.fp.shape=="box") {
                                    auto sz=fp["size"];
                                    if (sz && sz.IsSequence() && sz.size()>=2) {
                                        m.fp.size_x=sz[0].as<double>();
                                        m.fp.size_y=sz[1].as<double>();
                                        m.fp.valid=true;
                                    }
                                } else if (m.fp.shape=="multibox") {
                                    auto boxes=fp["boxes"];
                                    if (boxes && boxes.IsSequence()) {
                                        for (const auto& b: boxes) {
                                            Meta::Part p{};
                                            if (auto c=b["center"]; c && c.IsSequence() && c.size()>=2) { p.cx=c[0].as<double>(); p.cy=c[1].as<double>(); }
                                            if (auto s2=b["size"];   s2 && s2.IsSequence() && s2.size()>=2) { p.sx=s2[0].as<double>(); p.sy=s2[1].as<double>(); }
                                            p.angle = (b["angle"] && b["angle"].IsScalar()) ? b["angle"].as<double>() : 0.0;
                                            m.fp.boxes.push_back(p);
                                        }
                                        if (!m.fp.boxes.empty()) m.fp.valid=true;
                                    }
                                }
                            }
                        }
                    }
                    m_robot_meta.push_back(std::move(m));
                }
            }
        } catch (const std::exception& e) {
            Logger::instance().warn(std::string("SST (src): failed to parse robot_meta yaml: ")+e.what());
            m_robot_meta.clear();
        }
    }
    if (auto v = s.get<std::string>("robot_meta_yaml")) { PlannerSettings t; t.kv.emplace("robot_meta", *v); configure(t); }

    // optional obstacles (parity with binary; kept minimal)
    if (auto v = s.get<std::string>("obstacles")) {
        try {
            YAML::Node root = YAML::Load(*v);
            m_env_obs.clear();
            if (root && root.IsSequence()) {
                for (const auto& o : root) {
                    if (!o || !o.IsMap()) continue;
                    DBCBSObstacle ob;
                    ob.type = o["type"] ? o["type"].as<std::string>() : "";
                    auto seq2ok = [](const YAML::Node& n, double& a, double& b){
                        if (!n || !n.IsSequence() || n.size()<2) return false;
                        try { a=n[0].as<double>(); b=n[1].as<double>(); } catch(...) { return false; }
                        return true;
                    };
                    if (ob.type=="box") {
                        seq2ok(o["center"], ob.cx, ob.cy); seq2ok(o["size"], ob.sx, ob.sy);
                    } else if (ob.type=="circle") {
                        seq2ok(o["center"], ob.cx, ob.cy); ob.radius = o["radius"] ? o["radius"].as<double>() : 0.0;
                    } else if (ob.type=="capsule") {
                        seq2ok(o["p0"], ob.p0x, ob.p0y); seq2ok(o["p1"], ob.p1x, ob.p1y);
                        ob.radius = o["radius"] ? o["radius"].as<double>() : 0.0;
                    } else if (ob.type=="polygon") {
                        if (auto pts=o["points"]; pts && pts.IsSequence()) {
                            for (const auto& p : pts) { double x=0,y=0; if (seq2ok(p,x,y)) ob.polygon.push_back({x,y}); }
                        }
                    }
                    m_env_obs.push_back(ob);
                }
            }
        } catch (...) { m_env_obs.clear(); }
    }
    if (auto v = s.get<std::string>("obstacles_yaml")) { PlannerSettings t; t.kv.emplace("obstacles", *v); configure(t); }
}

// ---------------- Builders ----------------
void SST_Source::buildProblem(const BaseEnvironment& E,
                              const std::vector<StartGoal>& sgs,
                              dbcbs_sst::ProblemSpec& out) const
{
    const auto [W,H] = E.boundsInMeter();
    out.min_x = 0.0; out.min_y = 0.0;
    out.max_x = W;   out.max_y = H;

    YAML::Node obs(YAML::NodeType::Sequence);
    auto seq2 = [](double a, double b) {
        YAML::Node n(YAML::NodeType::Sequence);
        n.SetStyle(YAML::EmitterStyle::Flow);
        n.push_back(a); n.push_back(b); return n;
    };
    for (const auto& o : m_env_obs) { // usually empty unless provided via settings
        YAML::Node n(YAML::NodeType::Map);
        n["type"] = o.type;
        if (o.type == "box") {
            n["center"] = seq2(o.cx, o.cy);
            n["size"]   = seq2(o.sx, o.sy);
        } else if (o.type == "circle") {
            n["center"] = seq2(o.cx, o.cy);
            n["radius"] = o.radius;
        } else if (o.type == "capsule") {
            n["p0"] = seq2(o.p0x, o.p0y);
            n["p1"] = seq2(o.p1x, o.p1y);
            n["radius"] = o.radius;
        } else if (o.type == "polygon") {
            YAML::Node pts(YAML::NodeType::Sequence);
            for (const auto& p : o.polygon) pts.push_back(seq2(p[0], p[1]));
            n["points"] = pts;
        } else {
            continue;
        }
        obs.push_back(n);
    }
    YAML::Emitter em; em << obs;
    out.obstacles_yaml = em.c_str();

    auto defaultDimForType = [](const std::string& ty){
        if (ty.find("double_integrator") != std::string::npos) return 4;
        if (ty.find("unicycle") != std::string::npos)          return 3;
        if (ty.find("car_") != std::string::npos)              return 3;
        return 2;
    };

    out.robots.clear();
    out.robots.reserve(sgs.size());
    for (size_t i=0;i<sgs.size();++i) {
        dbcbs_sst::ProblemSpec::Robot R{};
        R.meta.type = (i<m_robot_meta.size() && !m_robot_meta[i].type.empty())
                      ? m_robot_meta[i].type : m_cfg.robot_type;

        if (i < m_robot_meta.size() && m_robot_meta[i].fp.valid) {
            R.meta.fp.shape  = m_robot_meta[i].fp.shape;
            R.meta.fp.radius = m_robot_meta[i].fp.radius;
            R.meta.fp.size_x = m_robot_meta[i].fp.size_x;
            R.meta.fp.size_y = m_robot_meta[i].fp.size_y;
            R.meta.fp.valid  = true;
            for (const auto& p : m_robot_meta[i].fp.boxes) {
                dbcbs_sst::Footprint::Part q{p.cx,p.cy,p.sx,p.sy,p.angle};
                R.meta.fp.boxes.push_back(q);
            }
        }

        const int dim = defaultDimForType(R.meta.type);
        R.start.assign(static_cast<size_t>(dim), 0.0);
        R.goal .assign(static_cast<size_t>(dim), 0.0);
        R.start[0] = sgs[i].start.x; R.start[1] = sgs[i].start.y;
        R.goal [0] = sgs[i].goal.x;  R.goal [1] = sgs[i].goal.y;

        out.robots.push_back(std::move(R));
    }
}

void SST_Source::buildConfig(dbcbs_sst::SSTConfig& out) const {
    out.planner               = m_cfg.planner;
    out.timelimit_s           = static_cast<int>(m_cfg.timelimit_s);
    out.propagation_step_size = m_cfg.propagation_step_size;
    out.control_duration_min  = m_cfg.control_duration_min;
    out.control_duration_max  = m_cfg.control_duration_max;
    out.goal_epsilon          = m_cfg.goal_epsilon;
    out.goal_bias             = m_cfg.goal_bias;
    out.selection_radius      = m_cfg.selection_radius;
    out.pruning_radius        = m_cfg.pruning_radius;
}

// ---------------- Plan ----------------
PlanResult SST_Source::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids    = req.ids;
    out.status = PlanStatus::Error;

    if (!req.env.continuousCapability()) {
        Logger::instance().error("SST (src): Environment does not support IContinuousSpace.");
        return out;
    }
    if (req.ids.size() != req.sgs.size()) { Logger::instance().error("SST (src): ids.size() != sgs.size()."); return out; }

    // Per-call overrides via shadow
    SST_SourceConfig cfg = m_cfg;
    SST_Source shadow(cfg);
    // carry over preconfigured meta/env/radius
    shadow.m_robot_meta   = this->m_robot_meta;
    shadow.m_env_obs      = this->m_env_obs;
    shadow.m_agent_radius = this->m_agent_radius;
    shadow.configure(req.settings);

    const double timelimit_s = resolvePlannerTimeLimit(
        req.time_limit_s, shadow.m_cfg.timelimit_s);

    std::vector<StartGoal> sgs_use = req.sgs;
    if (shadow.m_cfg.takeK && *shadow.m_cfg.takeK > 0 && *shadow.m_cfg.takeK < static_cast<int>(sgs_use.size()))
        sgs_use.resize(static_cast<size_t>(*shadow.m_cfg.takeK));

    // synthesize circle footprints if user only supplied agent_radius
    if (shadow.m_robot_meta.empty() && shadow.m_agent_radius && *shadow.m_agent_radius > 0.0) {
        shadow.m_robot_meta.assign(sgs_use.size(), {});
        for (auto& m : shadow.m_robot_meta) {
            m.type         = shadow.m_cfg.robot_type;
            m.fp.shape     = "circle";
            m.fp.radius    = *shadow.m_agent_radius;
            m.fp.valid     = true;
        }
    }

    dbcbs_sst::ProblemSpec P;
    shadow.buildProblem(req.env, sgs_use, P);

    dbcbs_sst::SSTConfig  C;
    shadow.buildConfig(C);
    C.timelimit_s = static_cast<int>(timelimit_s);

    dbcbs_sst::SSTResult  R;

    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = dbcbs_sst::sst_solve(P, C, R, nullptr);
    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!ok || !R.solved) {
        Logger::instance().warn(ok ? "SST (src): no solution within limits." : "SST (src): sst_solve returned failure.");
        out.status = (out.diag.runtime_s >= timelimit_s && timelimit_s > 0) ? PlanStatus::Timeout : PlanStatus::Error;
        return out;
    }

    // Convert to platform Plans: (x,y) with dt = step_dt_s
    std::vector<Plan> plans(R.states.size());
    for (size_t i=0;i<R.states.size();++i) {
        Plan Pn;
        const auto& seq = R.states[i];
        Pn.waypoints.reserve(seq.size());
        for (size_t k=0;k<seq.size();++k) {
            const auto& s = seq[k];
            if (s.size()<2) continue;
            const float t = static_cast<float>(k * shadow.m_cfg.step_dt_s);
            Pn.waypoints.push_back(Waypoint{ b2Vec2{ static_cast<float>(s[0]), static_cast<float>(s[1]) }, t });
        }
        plans[i] = std::move(Pn);
    }

    out.plans = std::move(plans);
    out.ids.resize(out.plans.size());
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;
    return out;
}
