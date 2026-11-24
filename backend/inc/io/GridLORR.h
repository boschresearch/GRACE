// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <vector>
#include <limits>

#include "../env/ICapabilities.h" // for validation against grid bounds/blocks
#include "../planner/Planner.h"    // StartGoal

namespace grace::io {

    /** Options controlling conversion from LORR files to start/goal cells. */
    struct GridLORROptions {
        std::size_t takeK = std::numeric_limits<std::size_t>::max();
        bool validateAgainstGrid = true;
        bool skipBlockedCells    = true;
        bool uniqueGoals         = true;
    };

    // ---------- Internal helpers (reusable, but not part of the main API) ----------
    namespace detail {

        /** Read a LORR integer list: first token N, followed by N integers. */
        [[nodiscard]] std::vector<int>readGridLORRIntList(const std::filesystem::path& path);

    } // namespace detail

    /** Convert LORR (.agents = start idx, .tasks = goal idx) to StartGoal (meters). */
    std::vector<StartGoal>
    readGridLORRAgentsTasks(const IDiscreteGrid& G,
                             const std::filesystem::path& agentsPath,
                             const std::filesystem::path& tasksPath,
                             const GridLORROptions& opt = {});

} // namespace grace::io
