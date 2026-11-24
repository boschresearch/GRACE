// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "ResetWorkflow.h"

#include "Simulation.h"
#include "env/BaseEnvironment.h"

namespace platform::reset {

void resetRobotsToPlanStarts(Simulation& simulation) {
    for (const b2BodyId id : simulation.robots().robots()) {
        auto* metadata = simulation.robots().meta(id);
        if (!metadata || metadata->plan.waypoints.empty()) {
            continue;
        }

        b2Body_SetTransform(id, metadata->plan.waypoints.front().p, b2Rot_identity);
        b2Body_SetLinearVelocity(id, b2Vec2_zero);
        b2Body_SetAngularVelocity(id, 0.0f);
        metadata->next_wp = 0;
    }
    simulation.resetControllers();
}

void clearRobots(Simulation* simulation, BaseEnvironment* environment) {
    if (simulation) {
        simulation->clearAllRobots();
    } else if (environment) {
        environment->clearRobots();
    }
}

}  // namespace platform::reset
