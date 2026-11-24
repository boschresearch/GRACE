// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "../env/ICapabilities.h"
#include "../planner/Planner.h"     // StartGoal

namespace grace::io::detail {

    /** Grid-only representation tucked into details. */
    struct GridStartGoal {
        int sc{0}, sr{0}; // start col/row (cell coordinates)
        int gc{0}, gr{0}; // goal  col/row (cell coordinates)
    };

    /** Convert GridStartGoal (cells) → StartGoal (meters). */
    StartGoal toStartGoal(const IDiscreteGrid& G, const GridStartGoal& gg);

    /** Convert StartGoal (meters) → StartGoalGrid (cells). */
    GridStartGoal toGridStartGoal(const IDiscreteGrid& G, const StartGoal& sg);

    [[nodiscard]] std::vector<StartGoal> toStartGoals(const IDiscreteGrid& G, const std::vector<GridStartGoal>& v);

    [[nodiscard]] std::vector<GridStartGoal> toGridStartGoals(const IDiscreteGrid& G, const std::vector<StartGoal>& v);

} // namespace grace::io::detail
