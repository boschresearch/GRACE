// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "PlanningWorkflow.h"

#include "Simulation.h"
#include "planner/plugin/PlannerFactory.h"

#include <utility>

namespace {

void discardPlan(RobotMeta& metadata) {
    metadata.plan.waypoints.clear();
    metadata.next_wp = 0;
}

}  // namespace

namespace platform::planning {

PlannerSelection selectPlanner(
    const EnvKind kind,
    const std::string& requestedName) {
    if (PlannerCatalog::isAllowed(kind, requestedName)) {
        return PlannerSelection{requestedName, false};
    }
    return PlannerSelection{PlannerCatalog::defaultFor(kind), true};
}

PlannerSelection installPlanner(
    Simulation& simulation,
    const EnvKind kind,
    const std::string& requestedName,
    const double timeLimitS) {
    PlannerSelection selection = selectPlanner(kind, requestedName);
    simulation.setPlanner(makePlanner(selection.name));

    PlannerSettings settings;
    settings.set("timelimit_s", timeLimitS);
    simulation.configurePlanner(std::move(settings));
    return selection;
}

GoalAssignment assignGoal(
    Simulation& simulation,
    const std::vector<b2BodyId>& robotIds,
    b2Vec2 worldGoal) {
    if (const auto* grid = simulation.gridEnv()) {
        worldGoal = grid->cellToWorld(grid->worldToCell(worldGoal));
    } else if (const auto* roadmap = simulation.roadmapEnv()) {
        worldGoal = roadmap->nodePos(roadmap->nearestNode(worldGoal));
    }

    std::size_t updatedRobots = 0;
    for (const b2BodyId id : robotIds) {
        if (auto* metadata = simulation.robots().meta(id)) {
            metadata->requested_goal_m = worldGoal;
            discardPlan(*metadata);
            ++updatedRobots;
        }
    }
    return GoalAssignment{worldGoal, updatedRobots};
}

std::size_t discardPlans(
    Simulation& simulation,
    const std::vector<b2BodyId>& robotIds) {
    std::size_t discardedPlans = 0;
    for (const b2BodyId id : robotIds) {
        if (auto* metadata = simulation.robots().meta(id)) {
            discardPlan(*metadata);
            ++discardedPlans;
        }
    }
    return discardedPlans;
}

bool planPreservingGoals(
    Simulation& simulation,
    const std::vector<b2BodyId>& robotIds) {
    if (robotIds.empty()) {
        return false;
    }

    std::vector<StartGoal> startGoals;
    startGoals.reserve(robotIds.size());
    for (const b2BodyId id : robotIds) {
        const b2Vec2 start = b2Body_GetPosition(id);
        b2Vec2 goal = start;
        if (const auto* metadata = simulation.robots().meta(id)) {
            if (metadata->requested_goal_m) {
                goal = *metadata->requested_goal_m;
            } else if (!metadata->plan.waypoints.empty()) {
                goal = metadata->plan.waypoints.back().p;
            }
        }
        startGoals.push_back(StartGoal{start, goal});
    }
    return simulation.plan(robotIds, startGoals);
}

}  // namespace platform::planning
