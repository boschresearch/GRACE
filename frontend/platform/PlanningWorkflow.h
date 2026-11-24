// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "PlannerCatalog.h"

#include <box2d/box2d.h>

#include <cstddef>
#include <string>
#include <vector>

class Simulation;

namespace platform::planning {

struct PlannerSelection {
    std::string name;
    bool usedFallback{false};
};

struct GoalAssignment {
    b2Vec2 snappedGoal{};
    std::size_t updatedRobots{0};
};

[[nodiscard]] PlannerSelection selectPlanner(
    EnvKind kind,
    const std::string& requestedName);

[[nodiscard]] PlannerSelection installPlanner(
    Simulation& simulation,
    EnvKind kind,
    const std::string& requestedName,
    double timeLimitS);

[[nodiscard]] GoalAssignment assignGoal(
    Simulation& simulation,
    const std::vector<b2BodyId>& robotIds,
    b2Vec2 worldGoal);

[[nodiscard]] std::size_t discardPlans(
    Simulation& simulation,
    const std::vector<b2BodyId>& robotIds);

[[nodiscard]] bool planPreservingGoals(
    Simulation& simulation,
    const std::vector<b2BodyId>& robotIds);

}  // namespace platform::planning
