// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <chrono>
#include <algorithm>
#include <cmath>

#include "inc/planner/roadmap/CCBS/CCBS_Source.h"
#include "inc/env/ICapabilities.h"
#include "inc/util/Logger.h"

#include <fmt/format.h>

// External CCBS headers (in-process)
#include <cbs.h>
#include <map.h>
#include <task.h>
#include <config.h>

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

inline bool feq(float a, float b, float tol=1e-5f) {
    return std::abs(a - b) <= tol;
}
} // namespace

// ---------------- settings ----------------
void CCBS_Source::configure(const PlannerSettings& s)
{
    if (auto v = s.get<bool>("use_cardinal"))             m_cfg.use_cardinal = *v;
    if (auto v = s.get<bool>("use_disjoint_splitting"))   m_cfg.use_disjoint_splitting = *v;
    if (auto v = s.get<int>("hlh_type"))                  m_cfg.hlh_type = *v;
    if (auto v = s.get<int>("connectedness"))             m_cfg.connectedness = *v;
    if (auto v = s.get<double>("focal_weight"))           m_cfg.focal_weight = *v;
    if (auto v = s.get<double>("agent_size"))             m_cfg.agent_size = *v;
    if (auto v = s.get<double>("timelimit_s"))            m_cfg.timelimit_s = *v;
    if (auto v = s.get<double>("precision"))              m_cfg.precision = *v;

    // string fallbacks
    if (auto v = s.get<std::string>("use_cardinal"))           if (auto x=parseBool(*v))   m_cfg.use_cardinal=*x;
    if (auto v = s.get<std::string>("use_disjoint_splitting")) if (auto x=parseBool(*v))   m_cfg.use_disjoint_splitting=*x;
    if (auto v = s.get<std::string>("hlh_type"))               if (auto x=parseInt(*v))    m_cfg.hlh_type=*x;
    if (auto v = s.get<std::string>("connectedness"))          if (auto x=parseInt(*v))    m_cfg.connectedness=*x;
    if (auto v = s.get<std::string>("focal_weight"))           if (auto x=parseDouble(*v)) m_cfg.focal_weight=*x;
    if (auto v = s.get<std::string>("agent_size"))             if (auto x=parseDouble(*v)) m_cfg.agent_size=*x;
    if (auto v = s.get<std::string>("timelimit_s"))            if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("cutoff_s"))               if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("precision"))              if (auto x=parseDouble(*v)) m_cfg.precision=*x;

    // common alias for time limit
    if (auto v = s.get<double>("cutoff_s"))               m_cfg.timelimit_s = *v;
}

// ---------------- helpers ----------------
int CCBS_Source::findNodeIdByPos(const IRoadmap& R, b2Vec2 p, float tol)
{
    const int N = R.numNodes();
    for (int i = 0; i < N; ++i) {
        const auto q = R.nodePos(i);
        if (feq(p.x, q.x, tol) && feq(p.y, q.y, tol)) return i;
    }
    return -1;
}

void CCBS_Source::collectRoadmap(const IRoadmap& R,
                                 std::vector<std::pair<double,double>>& coords,
                                 std::vector<std::vector<int>>& outAdjacency)
{
    const int N = R.numNodes();
    coords.resize(N);
    outAdjacency.assign(N, {});

    for (int u = 0; u < N; ++u) {
        const b2Vec2 p = R.nodePos(u);
        coords[u] = { static_cast<double>(p.x), static_cast<double>(p.y) };

        const auto& nbrs = R.adj(u);
        outAdjacency[u].reserve(nbrs.size());
        for (int v : nbrs) outAdjacency[u].push_back(v);
    }
}

// ---------------- entry ----------------
PlanResult CCBS_Source::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids = req.ids;
    out.status = PlanStatus::Error;
    out.plans.clear();

    const IRoadmap* R = req.env.roadmapCapability();
    if (!R) {
        Logger::instance().error("CCBS (src): environment is not a MAPFRoadmapEnv.");
        return out;
    }
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("CCBS (src): ids.size() != sgs.size().");
        return out;
    }

    // Per-call overrides
    CCBS_SourceConfig cfg = m_cfg;
    { CCBS_Source tmp(cfg); tmp.configure(req.settings); cfg = tmp.m_cfg; }
    const double time_limit_s = resolvePlannerTimeLimit(
        req.time_limit_s, cfg.timelimit_s);

    const auto t0 = std::chrono::steady_clock::now();

    // 1) Collect roadmap
    std::vector<std::pair<double,double>> coords;      // (x, y)
    std::vector<std::vector<int>> adjacency;
    collectRoadmap(*R, coords, adjacency);

    // 2) Build CCBS Map (external)
    Map ccbsMap(cfg.agent_size, cfg.connectedness);
    if (!ccbsMap.initRoadmap(coords, adjacency)) {
        Logger::instance().error("CCBS (src): Map::initRoadmap failed.");
        return out;
    }

    // 3) Build Task from node ids
    std::vector<std::pair<int,int>> nodePairs;
    nodePairs.reserve(req.sgs.size());
    for (const auto& sg : req.sgs) {
        const int s = findNodeIdByPos(*R, sg.start);
        const int g = findNodeIdByPos(*R, sg.goal);
        if (s < 0 || g < 0) {
            Logger::instance().error("CCBS (src): Start/goal not on roadmap nodes.");
            return out;
        }
        nodePairs.emplace_back(s, g);
    }

    Task ccbsTask;
    ccbsTask.init_from_pairs(ccbsMap, nodePairs);

    // 4) Config
    Config C;
    C.connectdness           = cfg.connectedness;
    C.use_cardinal           = cfg.use_cardinal;
    C.use_disjoint_splitting = cfg.use_disjoint_splitting;
    C.hlh_type               = cfg.hlh_type;
    C.focal_weight           = cfg.focal_weight;
    C.agent_size             = cfg.agent_size;
    C.timelimit              = time_limit_s;
    C.precision              = cfg.precision;

    // 5) Run CCBS
    CBS cbs;
    Solution sol = cbs.find_solution(ccbsMap, ccbsTask, C);

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!sol.found) {
        Logger::instance().warn("CCBS (src): no solution found within limits.");
        out.status = (out.diag.runtime_s >= time_limit_s && time_limit_s > 0) ? PlanStatus::Timeout
                                                                              : PlanStatus::Error;
        return out;
    }

    // 6) Convert to platform plans
    out.plans.resize(req.ids.size());

    for (size_t i = 0; i < sol.paths.size() && i < out.plans.size(); ++i) {
        const auto& sp = sol.paths[i];
        Plan P;
        P.waypoints.reserve(sp.nodes.size());
        for (const auto& sn : sp.nodes) {
            const int nid = sn.id;
            const float x = static_cast<float>(coords[nid].first);
            const float y = static_cast<float>(coords[nid].second);
            const float t = static_cast<float>(sn.g);
            P.waypoints.push_back(Waypoint{ b2Vec2{x,y}, t });
        }
        out.plans[i] = std::move(P);
    }

    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;
    return out;
}
