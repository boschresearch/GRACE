// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/sim/PlanVerifier.h"
#include <cmath>
#include <fmt/format.h>

bool PlanVerifier::endpointInGoalCell(const IDiscreteGrid& grid, b2Vec2 end, b2Vec2 goal) const {
    const auto [cg, rg] = grid.worldToCell(goal);
    const auto [c, r] = grid.worldToCell(end);
    return (r == rg) && (c == cg);
}

bool PlanVerifier::endpointWithinTol(b2BodyId id, b2Vec2 end, b2Vec2 goal, const GoalTol& tol) const {
    const RobotMeta* meta = m_robots.meta(id);
    return within_goal_tolerance(meta, end, goal, tol);
}

bool PlanVerifier::accept(const std::vector<b2BodyId>& ids,
                          const PlanResult& res,
                          const PlanVerifyOptions& opt)
{
    const auto* grid = m_env.gridCapability();

    // Map: id -> plan index
    std::unordered_map<b2BodyId, std::size_t, BodyIdHash, BodyIdEq> idx;
    idx.reserve(res.ids.size());
    for (std::size_t i = 0; i < res.ids.size(); ++i) idx.emplace(res.ids[i], i);

    bool ok = true; int bad = 0;

    for (std::size_t i = 0; i < ids.size(); ++i) {
        const b2BodyId id = ids[i];
        const RobotMeta* meta = m_robots.meta(id);

        if (!meta || !meta->requested_goal_m) {
            ok = false; ++bad;
            m_log.error("AcceptanceGate: missing RobotMeta or requested_goal_m.");
            continue;
        }

        auto it = idx.find(id);
        if (it == idx.end()) {
            ok = false; ++bad;
            m_log.warn("AcceptanceGate: planner did not return a plan for an agent.");
            continue;
        }
        if (it->second >= res.plans.size()) {
            ok = false; ++bad;
            m_log.warn("AcceptanceGate: planner result has no matching plan payload.");
            continue;
        }

        const Plan& P = res.plans[it->second];
        if (P.waypoints.empty()) {
            ok = false; ++bad;
            m_log.warn("AcceptanceGate: empty plan.");
            continue;
        }

        const b2Vec2 end  = P.waypoints.back().p;
        const b2Vec2 goal = *meta->requested_goal_m;

        if (grid) {
            if (!endpointInGoalCell(*grid, end, goal)) {
                ok = false; ++bad;
                m_log.warn("AcceptanceGate: endpoint not in goal cell.");
            }
        } else if (opt.nonGridPolicy == NonGridEndpointPolicy::EuclideanTolerance) {
            if (!endpointWithinTol(id, end, goal, opt.tol)) {
                ok = false; ++bad;
                m_log.warn("AcceptanceGate: endpoint deviates beyond tolerance.");
            }
        }
    }

    if (!ok) {
        m_log.warn(fmt::format("AcceptanceGate: rejecting planner result ({} agent{} failed).",
                               bad, bad==1?"":"s"));
    }
    return ok;
}
