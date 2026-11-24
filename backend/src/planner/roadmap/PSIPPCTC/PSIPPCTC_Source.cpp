// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/roadmap/PSIPPCTC/PSIPPCTC_Source.h"

#include "../inc/util/Logger.h"
#include "../inc/env/ICapabilities.h"

#include <fmt/format.h>
#include <chrono>
#include <algorithm>
#include <charconv>
#include <base/input.hpp>

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
void PSIPPCTC_Source::configure(const PlannerSettings& s)
{
    if (auto v = s.get<double>("timelimit_s"))            m_cfg.timelimit_s = *v;
    if (auto v = s.get<double>("cutoff_s"))               m_cfg.timelimit_s = *v; // common alias
    if (auto v = s.get<int>("priority_strategy"))         m_cfg.priority_strategy = *v;
    if (auto v = s.get<int>("takeK"))                     m_cfg.takeK = *v;
    if (auto v = s.get<double>("agent_radius"))           m_agent_radius = *v;
    if (auto v = s.get<double>("agent_size"))             m_agent_radius = *v; // alias

    // string fallbacks
    if (auto v = s.get<std::string>("timelimit_s"))       if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("cutoff_s"))          if (auto x=parseDouble(*v)) m_cfg.timelimit_s=*x;
    if (auto v = s.get<std::string>("priority_strategy")) if (auto x=parseInt(*v))    m_cfg.priority_strategy=*x;
    if (auto v = s.get<std::string>("takeK"))             if (auto x=parseInt(*v))    m_cfg.takeK=*x;
    if (auto v = s.get<std::string>("agent_radius"))      if (auto x=parseDouble(*v)) m_agent_radius=*x;
    if (auto v = s.get<std::string>("agent_size"))        if (auto x=parseDouble(*v)) m_agent_radius=*x;
}

// --------------- Node pair mapping ------------------------------------------
std::vector<std::pair<int,int>>
PSIPPCTC_Source::nodePairsFromMeters(const IRoadmap& R,
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

// --------------- Plan conversion --------------------------------------------
static inline b2Vec2 v2(float x, float y) { return b2Vec2{x,y}; }

bool PSIPPCTC_Source::convertPSIPPPlanToPlatform(
    ProblemInstance& P,
    const ProblemInstance::Plan& planIn,
    std::vector<Plan>& outPlans)
{
    const int A = static_cast<int>(planIn.size());
    outPlans.assign(static_cast<size_t>(A), {});

    for (int a=0; a<A; ++a) {
        Plan Pout;
        float last_t = 0.0f;

        int u = P.getAgentInitialVertex(a);
        const int goal = P.getAgentGoalVertex(a);

        // Start at t=0
        Pout.waypoints.push_back(
            Waypoint{ v2((float)P.getCoordinateX(u),(float)P.getCoordinateY(u)), 0.0f });

        for (const auto& mv : planIn[a]) {
            const int    edge_id    = mv.edge_id;
            const double start_time = mv.start_time;

            if (start_time > last_t) {
                Pout.waypoints.push_back(
                    Waypoint{ v2((float)P.getCoordinateX(u),(float)P.getCoordinateY(u)),
                              (float)start_time });
                last_t = (float)start_time;
            }

            const int v = P.edges[u][edge_id];
            const double L = P.getEdgeLength(u, edge_id);
            const double t_arr = start_time + L;

            Pout.waypoints.push_back(
                Waypoint{ v2((float)P.getCoordinateX(v),(float)P.getCoordinateY(v)),
                          (float)t_arr });
            last_t = (float)t_arr;
            u = v;
        }

        if (u != goal) {
            Pout.waypoints.push_back(
                Waypoint{ v2((float)P.getCoordinateX(goal),(float)P.getCoordinateY(goal)), last_t });
        }

        outPlans[static_cast<size_t>(a)] = std::move(Pout);
    }
    return true;
}

// --------------- IPlanner::plan ----------------------------------------------
PlanResult PSIPPCTC_Source::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids    = req.ids;
    out.status = PlanStatus::Error;

    const IRoadmap* R = req.env.roadmapCapability();
    if (!R) {
        Logger::instance().error("PSIPP-CTC (src): Environment is not MAPFRoadmapEnv.");
        return out;
    }
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("PSIPP-CTC (src): ids.size() != sgs.size().");
        return out;
    }

    // Per-call overrides
    PSIPPCTC_SourceConfig cfg = m_cfg;
    PSIPPCTC_Source shadow(cfg); shadow.configure(req.settings); cfg = shadow.m_cfg;
    const double timelimit_s = resolvePlannerTimeLimit(
        req.time_limit_s, cfg.timelimit_s);
    const double radius = shadow.m_agent_radius.value_or(0.25);

    const auto t0 = std::chrono::steady_clock::now();

    // Map to node IDs
    auto pairs_all = nodePairsFromMeters(*R, req.sgs);
    if (cfg.takeK && *cfg.takeK > 0 && *cfg.takeK < static_cast<int>(pairs_all.size()))
        pairs_all.resize(static_cast<size_t>(*cfg.takeK));

    // Build PSIPP problem directly from memory
    const int V = R->numNodes();

    // 1) coordinates (x,y)
    std::vector<std::pair<double,double>> coords_xy;
    coords_xy.reserve(V);
    for (int i = 0; i < V; ++i) {
        const b2Vec2 p = R->nodePos(i);
        coords_xy.emplace_back((double)p.x, (double)p.y);
    }

    // 2) directed edges as (u->v) pairs
    std::vector<std::pair<int,int>> arcs;
    arcs.reserve([&]{ size_t s=0; for (int u=0; u<V; ++u) s += R->adj(u).size(); return s; }());
    for (int u=0; u<V; ++u) for (int v : R->adj(u)) arcs.emplace_back(u, v);

    // 3) tasks as (s,g)
    std::vector<std::pair<int,int>> tasks_pairs(pairs_all.begin(), pairs_all.end());

    // 4) build PSIPP problem
    auto geom    = std::make_shared<ProblemInstanceWithGeometry<Geometry2D>>(
        coords_xy, arcs, tasks_pairs, radius);
    auto instCTC = std::make_shared<ProblemInstanceWithExplicitCollisions>(*geom);

    // Plan (in-process)
    PrioritizedSIPP psipp(/*priority_strategy=*/cfg.priority_strategy,
                          /*time_limit=*/timelimit_s);
    psipp.setInstance(instCTC);
    psipp.solve();

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!psipp.succeeded()) {
        Logger::instance().warn("PSIPP-CTC (src): solver reported failure/timeout.");
        out.status = (out.diag.runtime_s >= timelimit_s && timelimit_s > 0)
                     ? PlanStatus::Timeout : PlanStatus::Error;
        return out;
    }

    const auto planRaw = psipp.getPlan();
    std::vector<Plan> plans;
    if (!convertPSIPPPlanToPlatform(*instCTC, planRaw, plans)) {
        Logger::instance().warn("PSIPP-CTC (src): failed to convert plan to platform format.");
        out.status = PlanStatus::Error;
        return out;
    }

    // Trim/match ids
    if (plans.size() != pairs_all.size()) {
        plans.resize(std::min(plans.size(), pairs_all.size()));
    }
    out.plans  = std::move(plans);
    out.ids.resize(out.plans.size());
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;
    return out;
}
