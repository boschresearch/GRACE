// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// Sample_MAPF_Grid.cpp — Grid Studio: user-driven map/agents/planner → PLAN → VISUALIZE
#include "platform/PlatformBase.h"
#include "env/GridEnvironment.h"
#include "Simulation.h"
#include "planner/plugin/PlannerFactory.h"

#include <filesystem>
#include <fmt/format.h>

#include "sample/SampleContext.h"

namespace {

    class PlatformSample_Grid final : public PlatformBase {
    public:
        explicit PlatformSample_Grid(SampleContext* ctx) : PlatformBase(ctx)
        {
            // Start in paused state; the Studio guides user to PLAN then VISUALIZE
            m_context->pause = true;

            // We don’t auto-load anything now — the Studio panel handles:
            //   1) Load/create map
            //   2) Load agents
            //   3) Install planner
            //   4) PLAN
            //   5) VISUALIZE
            //
            // Everything else (overlays, editors) is under “Advanced panels”.
            m_instanceInfo = "GRID: Studio (load/create map, then agents, then plan)";
        }
    };

    Sample* CreateGrid(SampleContext* c) { return new PlatformSample_Grid(c); }

    // Rename/describe the entry to reflect studio flow
    const int s_reg_grid = RegisterSample("MAPF", "Grid Studio (guided flow)", &CreateGrid);

} // namespace
