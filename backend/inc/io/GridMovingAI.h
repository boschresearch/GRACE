// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "../env/ICapabilities.h" // IDiscreteGrid (for writing)
#include "../planner/Planner.h"    // StartGoal
#include "GridStartGoal.h"       // detail::StartGoalGrid + converters
#include "GridMapData.h"
#include "ReadResult.h"

namespace grace::io {

    /** MovingAI map (.map) — read dimensions and row-major occupancy. */
    [[nodiscard]] ReadResult<GridMapData>
    readGridMovingAIMap(const std::filesystem::path& path);

    /** MovingAI map (.map) — write from any IDiscreteGrid. */
    [[nodiscard]] bool writeGridMovingAIMap(
        const IDiscreteGrid& G,
        const std::filesystem::path& path);

    /** Parse a MovingAI scenario (.scen) to world-space StartGoals (meters). */
    [[nodiscard]] ReadResult<std::vector<StartGoal>>
    readGridMovingAIScenario(
        const IDiscreteGrid& G,
        const std::filesystem::path& scenPath,
        std::size_t takeK = std::numeric_limits<std::size_t>::max());

    /** Write a simple scenario from world-space starts/goals. */
    [[nodiscard]] bool writeGridMovingAIScenario(
        const IDiscreteGrid& G,
        const std::filesystem::path& path,
        const std::vector<b2Vec2>& starts_m,
        const std::vector<b2Vec2>& goals_m,
        const std::vector<int>& buckets = {},
        const std::vector<double>& dists = {});

    namespace detail {

        /** One row from a MovingAI .scen file (v0 or v1). */
        struct MovingAIScenarioEntry {
            int bucket = 0;
            std::string map;
            int width  = -1;  // -1 for v0
            int height = -1;  // -1 for v0
            int sc = 0; // start col (x)
            int sr = 0; // start row (y)
            int gc = 0; // goal  col (x)
            int gr = 0; // goal  row (y)
            double dist = 0.0;
        };

        /** Parse raw entries in MovingAI v0 or v1 format. */
        [[nodiscard]] ReadResult<std::vector<MovingAIScenarioEntry>>
        parseMovingAIScenario(const std::filesystem::path& scenPath);

        /** Convenience: convert raw entries to detail::StartGoalGrid (take first K). */
        [[nodiscard]] std::vector<GridStartGoal> toGridStartGoals(const std::vector<MovingAIScenarioEntry>& entries,
                         std::size_t takeK = std::numeric_limits<std::size_t>::max());
    } // namespace detail

} // namespace grace::io
