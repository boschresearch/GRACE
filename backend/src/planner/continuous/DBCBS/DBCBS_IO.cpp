// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/continuous/DBCBS/DBCBS_IO.h"
#include "../inc/util/Logger.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace fs = std::filesystem;

static inline YAML::Node seq2(double a, double b) {
    YAML::Node n(YAML::NodeType::Sequence);
    n.SetStyle(YAML::EmitterStyle::Flow);
    n.push_back(a); n.push_back(b);
    return n;
}
static inline YAML::Node vec_flow(const std::vector<double>& vals) {
    YAML::Node n(YAML::NodeType::Sequence);
    n.SetStyle(YAML::EmitterStyle::Flow);
    for (double v : vals) n.push_back(v);
    return n;
}
static inline int defaultDimForType(const std::string& ty){
    if (ty.find("double_integrator") != std::string::npos) return 4;
    if (ty.find("unicycle") != std::string::npos)          return 3;
    if (ty.find("car_") != std::string::npos)              return 3;
    return 2;
}

void dbcbs_writeProblemYaml(const BaseEnvironment& E,
                            const std::vector<StartGoal>& sgs,
                            const std::vector<ContinuousRobotMeta>& robot_meta,
                            const std::string& robot_type_default,
                            const std::vector<DBCBSObstacle>& env_obstacles,
                            const fs::path& outYaml)
{
    YAML::Node doc;

    const auto [W,H] = E.boundsInMeter();
    YAML::Node env(YAML::NodeType::Map);
    env["min"] = seq2(0.0, 0.0);
    env["max"] = seq2(W, H);

    // ---- obstacles ----
    YAML::Node obstacles(YAML::NodeType::Sequence);
    for (const auto& o : env_obstacles) {
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
            for (const auto& p : o.polygon) {
                pts.push_back(seq2(p[0], p[1]));
            }
            n["points"] = pts;
        } else {
            // ignore unknown types silently (keeps output valid)
            continue;
        }
        obstacles.push_back(n);
    }
    env["obstacles"] = obstacles;
    doc["environment"] = env;

    YAML::Node robots(YAML::NodeType::Sequence);
    robots.SetStyle(YAML::EmitterStyle::Block);

    for (size_t i=0; i<sgs.size(); ++i) {
        const auto& sg = sgs[i];
        std::string ty = robot_type_default;
        if (i < robot_meta.size() && !robot_meta[i].type.empty()) ty = robot_meta[i].type;

        const int dim = defaultDimForType(ty);
        std::vector<double> s(static_cast<size_t>(dim), 0.0), g(s.size(), 0.0);
        s[0]=sg.start.x; s[1]=sg.start.y; g[0]=sg.goal.x; g[1]=sg.goal.y;

        YAML::Node r(YAML::NodeType::Map);
        r["type"]  = ty;
        r["start"] = vec_flow(s);
        r["goal"]  = vec_flow(g);

        // footprint: use meta if valid, else minimal defaults (same as Binary)
        bool wrote_fp = false;
        if (i < robot_meta.size() && robot_meta[i].fp.valid) {
            const auto& fp = robot_meta[i].fp;
            if (fp.shape == "circle" && fp.radius > 0.0) {
                YAML::Node f; f["shape"]="circle"; f["radius"]=fp.radius; r["footprint"]=f; wrote_fp=true;
            } else if (fp.shape=="box" && fp.size_x>0.0 && fp.size_y>0.0) {
                YAML::Node f; f["shape"]="box"; YAML::Node sz = vec_flow({fp.size_x, fp.size_y}); f["size"]=sz; r["footprint"]=f; wrote_fp=true;
            } else if (fp.shape=="multibox" && !fp.boxes.empty()) {
                YAML::Node f; f["shape"]="multibox";
                YAML::Node boxes(YAML::NodeType::Sequence); boxes.SetStyle(YAML::EmitterStyle::Block);
                for (const auto& p : fp.boxes) {
                    YAML::Node one(YAML::NodeType::Map);
                    one["center"]=seq2(p.cx,p.cy); one["size"]=seq2(p.sx,p.sy); one["angle"]=p.angle;
                    boxes.push_back(one);
                }
                f["boxes"]=boxes; r["footprint"]=f; wrote_fp=true;
            }
        }
        if (!wrote_fp) {
            if      (ty.find("double_integrator") != std::string::npos) { YAML::Node f; f["shape"]="circle"; f["radius"]=0.15; r["footprint"]=f; }
            else if (ty.find("unicycle_first_order_0_sphere") != std::string::npos) { YAML::Node f; f["shape"]="circle"; f["radius"]=0.40; r["footprint"]=f; }
            else if (ty.find("unicycle") != std::string::npos || ty.find("car_") != std::string::npos) {
                YAML::Node f; f["shape"]="box"; f["size"]=vec_flow({0.5,0.25}); r["footprint"]=f;
            }
        }

        robots.push_back(r);
    }
    doc["robots"] = robots;

    YAML::Emitter em; em << doc;
    std::ofstream y(outYaml);
    if (!y) throw std::runtime_error("dbcbs_writeProblemYaml: failed to write " + outYaml.string());
    y << em.c_str() << std::endl;
}

bool dbcbs_parseSolutionYaml(const fs::path& file,
                             const double step_dt,
                             std::vector<Plan>& outPlans)
{
    YAML::Node root = YAML::LoadFile(file.string());
    if (!root) return false;

    auto result = root["result"];
    if (!result || !result.IsSequence() || result.size() == 0) return false;

    outPlans.clear();
    outPlans.resize(result.size());

    // Optional global dt in file; otherwise use step_dt
    double dt = step_dt;
    if (root["dt"] && root["dt"].IsScalar()) {
        try { dt = root["dt"].as<double>(); } catch (...) {}
    }

    for (size_t i = 0; i < result.size(); ++i) {
        const auto& agent = result[i];
        const auto states = agent["states"];
        if (!states || !states.IsSequence() || states.size() == 0) return false;

        // Optional per-agent times
        std::vector<double> times;
        if (agent["times"] && agent["times"].IsSequence()) {
            times.reserve(states.size());
            for (const auto& t : agent["times"]) {
                if (!t.IsScalar()) { times.clear(); break; }
                times.push_back(t.as<double>());
            }
            if (times.size() != states.size()) times.clear();
        }

        Plan P;
        P.waypoints.reserve(states.size());
        for (size_t k = 0; k < states.size(); ++k) {
            const auto& s = states[k];
            if (!s.IsSequence() || s.size() < 2) return false;
            const double x = s[0].as<double>();
            const double y = s[1].as<double>();
            const float  t = times.empty() ? static_cast<float>(k * dt)
                                           : static_cast<float>(times[k]);
            P.waypoints.push_back(Waypoint{ b2Vec2{
                static_cast<float>(x), static_cast<float>(y) }, t });
        }
        outPlans[i] = std::move(P);
    }
    return true;
}
