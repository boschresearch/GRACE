// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <box2d/box2d.h>
#include <string>
#include <vector>
#include <limits>

#include "../env/GridEnvironment.h"  // grid spawn uses grid cells
#include "../Simulation.h"           // your existing Simulation API

namespace grace::sim {

    /** Options for spawning robots into the grid world. */
    struct SpawnOptions {
        std::string baseName = "r";
    };

    /**
     * Spawn robots using world-space StartGoal (meters).
     * Starts are snapped to their containing grid cell; goals are recorded in meters.
     * Note: this does not perform any planning.
     */
    std::vector<b2BodyId>
    spawnGridAgent(Simulation& sim,
                   const std::vector<StartGoal>& specs,
                   const SpawnOptions& opt = {});

} // namespace grace::sim
