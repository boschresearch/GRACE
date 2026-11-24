// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/sim/GoalCheck.h"
#include "../inc/env/ICapabilities.h"
#include <cmath>

bool all_agents_at_goals(const BaseEnvironment& env,
                         const RobotSystem& robots,
                         GoalCheckOptions opt,
                         Logger& log)
{
    const auto& ids = robots.robots();
    const auto* grid = env.gridCapability();

    bool allOK = true;

    for (b2BodyId id : ids) {
        const RobotMeta* meta = robots.meta(id);
        if (!meta || meta->plan.waypoints.empty()) {
            allOK = false;
            log.warn("Goal check: agent has no plan.");
            continue;
        }
        if (!meta->requested_goal_m) {
            allOK = false;
            log.error("Goal check: requested_goal_m not set for agent " +
                      (meta ? meta->name : std::string("<unnamed>")));
            continue;
        }

        const b2Vec2 goal = *meta->requested_goal_m;
        const b2Vec2 pos  = b2Body_GetPosition(id);

        if (grid) {
            const auto [cg, rg] = grid->worldToCell(goal);
            const auto [c, r] = grid->worldToCell(pos);
            if (r != rg || c != cg) {
                allOK = false;
                log.warn("Goal check: agent not in goal cell.");
            }
        } else {
            // Preserve your previous behavior: for non-grid envs, the default policy
            // does NOT enforce a distance check unless explicitly requested.
            if (opt.policy == GoalPolicy::EuclideanTol) {
                if (!within_goal_tolerance(meta, pos, goal, opt.tol)) {
                    allOK = false;
                    log.warn("Goal check: agent outside tolerance from goal.");
                }
            } else {
                // GridCellExact on non-grid → skip (compat with old code).
                // log.debug("Goal check: non-grid env; GridCellExact policy skipped.");
            }
        }
    }

    return allOK;
}
