// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "Simulation.h"
#include "env/ContinuousEnvironment.h"
#include "io/ContinuousYaml.h"
#include "io/SimContinuousYamlOps.h"

namespace {

class RequestSettingsPlanner final : public IPlanner {
public:
    explicit RequestSettingsPlanner(PlannerSettings& observed)
        : observed_(observed) {}

    const char* name() const noexcept override {
        return "request-settings-planner";
    }

    EnvironmentType type() const noexcept override {
        return EnvironmentType::ContinuousEnvironment;
    }

    PlanResult plan(const PlanRequest& request) override {
        observed_ = request.settings;

        PlanResult result;
        result.status = PlanStatus::Success;
        result.ids = request.ids;
        for (const auto& startGoal : request.sgs) {
            result.plans.push_back(Plan{{
                Waypoint{startGoal.start, 0.0f},
                Waypoint{startGoal.goal, 1.0f},
            }});
        }
        return result;
    }

private:
    PlannerSettings& observed_;
};

}  // namespace

TEST(SimContinuousYamlOps, ReusesImmutableSpecAcrossEnvironmentSpawnAndGoals) {
    grace::sim::ProblemYaml problem;
    problem.min = {10.0f, 20.0f};
    problem.max = {15.0f, 25.0f};

    grace::sim::ObstacleYaml obstacle;
    obstacle.kind = grace::sim::ObstacleKind::Circle;
    obstacle.center = {12.0f, 22.0f};
    obstacle.radius = 0.25f;
    problem.obstacles.push_back(std::move(obstacle));

    grace::sim::RobotYamlSpec robot;
    robot.type = "single_integrator_0";
    robot.start = {11.0f, 21.0f};
    robot.goal = {14.0f, 24.0f};
    problem.robots.push_back(std::move(robot));

    const grace::sim::ProblemYaml& immutableProblem = problem;
    ContinuousEnvironment environment;
    ASSERT_TRUE(grace::sim::ops::buildEnvironmentFromProblemSpec(
        environment, immutableProblem, 0.1f));
    EXPECT_EQ(environment.numObstacles(), 1U);

    Simulation simulation(environment);
    const auto ids = grace::sim::ops::spawnFromProblemSpecNoPlan(
        simulation, immutableProblem);
    ASSERT_EQ(ids.size(), 1U);
    EXPECT_FLOAT_EQ(b2Body_GetPosition(ids.front()).x, 1.0f);
    EXPECT_FLOAT_EQ(b2Body_GetPosition(ids.front()).y, 1.0f);

    PlannerSettings observedRequestSettings;
    simulation.setPlanner(
        std::make_unique<RequestSettingsPlanner>(observedRequestSettings));
    ASSERT_TRUE(simulation.plan(
        ids, {{{1.0f, 1.0f}, {4.0f, 4.0f}}}));

    const auto robotMetadata =
        observedRequestSettings.get<std::string>("robot_meta_yaml");
    const auto obstacleMetadata =
        observedRequestSettings.get<std::string>("obstacles_yaml");
    ASSERT_TRUE(robotMetadata.has_value());
    ASSERT_TRUE(obstacleMetadata.has_value());
    EXPECT_EQ(
        *robotMetadata,
        grace::sim::emitRobotsMetaYaml(immutableProblem.robots));
    EXPECT_EQ(
        *obstacleMetadata,
        grace::sim::emitObstaclesYamlLocal(
            immutableProblem, immutableProblem.min));

    ASSERT_TRUE(grace::sim::ops::applyGoalsFromProblemSpec(
        simulation, ids, immutableProblem));
    const auto* metadata = simulation.robots().meta(ids.front());
    ASSERT_NE(metadata, nullptr);
    ASSERT_TRUE(metadata->requested_goal_m.has_value());
    EXPECT_FLOAT_EQ(metadata->requested_goal_m->x, 4.0f);
    EXPECT_FLOAT_EQ(metadata->requested_goal_m->y, 4.0f);
}
