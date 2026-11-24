// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "Simulation.h"
#include "env/ContinuousEnvironment.h"
#include "env/GridEnvironment.h"

#include <cmath>
#include <memory>

namespace {

struct PlannerState {
    int planCalls{0};
    PlannerSettings configured;
    PlannerSettings requestSettings;
    PlanStatus nextStatus{PlanStatus::Success};
    bool omitLastPlan{false};
    bool replaceLastId{false};
    const std::vector<b2BodyId>* requestIds{nullptr};
    const std::vector<StartGoal>* requestStartGoals{nullptr};
};

class RecordingPlanner final : public IPlanner {
public:
    RecordingPlanner(
        std::shared_ptr<PlannerState> state,
        EnvironmentType type = EnvironmentType::ContinuousEnvironment)
        : state_(std::move(state)), type_(type) {}

    const char* name() const noexcept override {
        return "recording-planner";
    }

    EnvironmentType type() const noexcept override {
        return type_;
    }

    void configure(const PlannerSettings& settings) override {
        state_->configured = settings;
    }

    PlanResult plan(const PlanRequest& request) override {
        ++state_->planCalls;
        state_->requestSettings = request.settings;
        state_->requestIds = &request.ids;
        state_->requestStartGoals = &request.sgs;

        PlanResult result;
        result.status = state_->nextStatus;
        result.ids = request.ids;
        result.diag.runtime_s = 0.125;
        if (state_->replaceLastId && !result.ids.empty()) {
            result.ids.back() = b2_nullBodyId;
        }
        if (result.status != PlanStatus::Success) {
            return result;
        }
        for (const auto& startGoal : request.sgs) {
            result.plans.push_back(Plan{{
                Waypoint{startGoal.start, 0.0f},
                Waypoint{startGoal.goal, 1.0f},
            }});
        }
        if (state_->omitLastPlan && !result.plans.empty()) {
            result.plans.pop_back();
        }
        return result;
    }

private:
    std::shared_ptr<PlannerState> state_;
    EnvironmentType type_;
};

ContinuousEnvironment makeEnvironment() {
    ContinuousEnvironment environment;
    environment.setBounds(5.0f, 5.0f);
    return environment;
}

}  // namespace

TEST(Simulation, RejectsPlanningWithoutPlanner) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");

    EXPECT_THROW(
        simulation.plan(
            {robot}, {{{1.0f, 1.0f}, {2.0f, 2.0f}}}),
        std::runtime_error);
}

TEST(Simulation, RejectsPlannerEnvironmentMismatch) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    auto state = std::make_shared<PlannerState>();
    simulation.setPlanner(std::make_unique<RecordingPlanner>(
        state, EnvironmentType::GridEnvironment));

    EXPECT_THROW(
        simulation.plan(
            {robot}, {{{1.0f, 1.0f}, {2.0f, 2.0f}}}),
        std::runtime_error);
    EXPECT_EQ(state->planCalls, 0);
}

TEST(Simulation, ConfiguresPlannerAndInstallsSuccessfulPlan) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    auto state = std::make_shared<PlannerState>();
    simulation.setPlanner(std::make_unique<RecordingPlanner>(state));

    PlannerSettings settings;
    settings.set("fixture", 42);
    simulation.configurePlanner(settings);

    ASSERT_TRUE(simulation.plan(
        {robot}, {{{1.0f, 1.0f}, {2.0f, 2.0f}}}));

    ASSERT_EQ(state->planCalls, 1);
    EXPECT_EQ(state->configured.get<int>("fixture"), 42);
    EXPECT_DOUBLE_EQ(simulation.lastPlannerRuntimeSec(), 0.125);

    const auto* metadata = simulation.robots().meta(robot);
    ASSERT_NE(metadata, nullptr);
    ASSERT_EQ(metadata->plan.waypoints.size(), 2);
    EXPECT_FLOAT_EQ(metadata->plan.waypoints.back().p.x, 2.0f);
    EXPECT_FLOAT_EQ(metadata->plan.waypoints.back().p.y, 2.0f);
}

TEST(Simulation, KeepsConfigurationSeparateFromRequestSettings) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    auto state = std::make_shared<PlannerState>();
    simulation.setPlanner(std::make_unique<RecordingPlanner>(state));

    PlannerSettings configuration;
    configuration.set("timelimit_s", 10);
    simulation.configurePlanner(std::move(configuration));

    PlannerSettings requestSettings;
    requestSettings.set("robot_meta_yaml", "fixture metadata");
    simulation.setPlannerRequestSettings(std::move(requestSettings));

    ASSERT_TRUE(simulation.plan(
        {robot}, {{{1.0f, 1.0f}, {2.0f, 2.0f}}}));

    EXPECT_EQ(state->configured.get<int>("timelimit_s"), 10);
    EXPECT_EQ(
        state->requestSettings.get<std::string>("robot_meta_yaml"),
        "fixture metadata");
}

TEST(Simulation, BorrowsPlannerRequestCollectionsWithoutCopying) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    auto state = std::make_shared<PlannerState>();
    simulation.setPlanner(std::make_unique<RecordingPlanner>(state));
    const std::vector<b2BodyId> ids{robot};
    const std::vector<StartGoal> startGoals{
        {{1.0f, 1.0f}, {2.0f, 2.0f}},
    };

    ASSERT_TRUE(simulation.plan(ids, startGoals));

    EXPECT_EQ(state->requestIds, &ids);
    EXPECT_EQ(state->requestStartGoals, &startGoals);
}

TEST(Simulation, RejectsMismatchedRequestWithoutMutatingGoals) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto first = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    const auto second = simulation.spawnCircleRobot({2.0f, 1.0f}, 0.2f, "r2");
    auto state = std::make_shared<PlannerState>();
    simulation.setPlanner(std::make_unique<RecordingPlanner>(state));

    EXPECT_THROW(
        simulation.plan(
            {first, second},
            {{{1.0f, 1.0f}, {3.0f, 3.0f}}}),
        std::runtime_error);

    EXPECT_EQ(state->planCalls, 0);
    EXPECT_FALSE(simulation.robots().meta(first)->requested_goal_m.has_value());
    EXPECT_FALSE(simulation.robots().meta(second)->requested_goal_m.has_value());
}

TEST(Simulation, RejectsDuplicateRobotIdsWithoutMutatingGoals) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto robot = simulation.spawnCircleRobot(
        {1.0f, 1.0f}, 0.2f, "r1");
    auto state = std::make_shared<PlannerState>();
    simulation.setPlanner(std::make_unique<RecordingPlanner>(state));

    EXPECT_THROW(
        simulation.plan(
            {robot, robot},
            {
                {{1.0f, 1.0f}, {2.0f, 2.0f}},
                {{1.0f, 1.0f}, {3.0f, 3.0f}},
            }),
        std::runtime_error);

    EXPECT_EQ(state->planCalls, 0);
    EXPECT_FALSE(
        simulation.robots().meta(robot)->requested_goal_m.has_value());
}

TEST(Simulation, RejectsPlannerFailureWithoutInstallingPlan) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    auto state = std::make_shared<PlannerState>();
    state->nextStatus = PlanStatus::Infeasible;
    simulation.setPlanner(std::make_unique<RecordingPlanner>(state));

    EXPECT_FALSE(simulation.plan(
        {robot}, {{{1.0f, 1.0f}, {2.0f, 2.0f}}}));
    EXPECT_EQ(state->planCalls, 1);

    const auto* metadata = simulation.robots().meta(robot);
    ASSERT_NE(metadata, nullptr);
    EXPECT_TRUE(metadata->plan.waypoints.empty());
    EXPECT_FALSE(metadata->requested_goal_m.has_value());
}

TEST(Simulation, RejectsUnknownContinuousRobotTypeWithoutCrashing) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);

    EXPECT_THROW(
        simulation.spawnByAlgoType(
            "unknown-robot-model", {1.0f, 1.0f}, "r1"),
        std::runtime_error);
    EXPECT_TRUE(simulation.robots().robots().empty());
}

TEST(Simulation, RejectsAlgorithmSpawnOutsideContinuousEnvironment) {
    GridEnvironment environment;
    environment.makeGridEnvironment(2, 2, 1.0f, std::vector<bool>(4, false));
    Simulation simulation(environment);

    EXPECT_THROW(
        simulation.spawnByAlgoType(
            "single_integrator_0", {0.5f, 0.5f}, "r1"),
        std::runtime_error);
    EXPECT_TRUE(simulation.robots().robots().empty());
}

TEST(Simulation, RejectsMismatchedPlannerResultShape) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto first = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    const auto second = simulation.spawnCircleRobot({2.0f, 1.0f}, 0.2f, "r2");
    auto state = std::make_shared<PlannerState>();
    state->omitLastPlan = true;
    simulation.setPlanner(std::make_unique<RecordingPlanner>(state));

    EXPECT_FALSE(simulation.plan(
        {first, second},
        {
            {{1.0f, 1.0f}, {3.0f, 3.0f}},
            {{2.0f, 1.0f}, {4.0f, 3.0f}},
        }));
    EXPECT_TRUE(simulation.robots().meta(first)->plan.waypoints.empty());
    EXPECT_TRUE(simulation.robots().meta(second)->plan.waypoints.empty());
}

TEST(Simulation, RejectsPlannerResultForDifferentRobotIds) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    auto state = std::make_shared<PlannerState>();
    state->replaceLastId = true;
    simulation.setPlanner(std::make_unique<RecordingPlanner>(state));

    EXPECT_FALSE(simulation.plan(
        {robot}, {{{1.0f, 1.0f}, {2.0f, 2.0f}}}));
    EXPECT_TRUE(simulation.robots().meta(robot)->plan.waypoints.empty());
}

TEST(Simulation, RecordsAvailableGoalsForKnownRobots) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto first = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    const auto second = simulation.spawnCircleRobot({2.0f, 1.0f}, 0.2f, "r2");

    ASSERT_TRUE(simulation.recordGoals({first, second}, {{4.0f, 4.0f}}));

    const auto* firstMetadata = simulation.robots().meta(first);
    const auto* secondMetadata = simulation.robots().meta(second);
    ASSERT_NE(firstMetadata, nullptr);
    ASSERT_NE(secondMetadata, nullptr);
    ASSERT_TRUE(firstMetadata->requested_goal_m.has_value());
    EXPECT_FALSE(secondMetadata->requested_goal_m.has_value());
}

TEST(Simulation, RejectsGridSpawnWithoutGridCapability) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);

    EXPECT_THROW(
        simulation.spawnGridCellRobot(1, 1, "r1"),
        std::runtime_error);
    EXPECT_TRUE(simulation.robots().robots().empty());
}

TEST(Simulation, ExposesGridCapabilityForGridEnvironment) {
    GridEnvironment environment;
    environment.makeGridEnvironment(3, 2, 0.5f, std::vector<bool>(6, false));
    Simulation simulation(environment);

    const auto* grid = simulation.gridEnv();

    ASSERT_NE(grid, nullptr);
    EXPECT_EQ(grid->gridWidthInCells(), 3);
    EXPECT_EQ(grid->gridHeightInCells(), 2);
    EXPECT_EQ(grid->worldToCell(grid->cellToWorld({2, 1})).c, 2);
    EXPECT_EQ(grid->worldToCell(grid->cellToWorld({2, 1})).r, 1);
}

TEST(Simulation, KeepsStableRobotIdCollection) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    const auto* initialCollection = &simulation.robots().robots();

    const auto first = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    const auto second = simulation.spawnCircleRobot({2.0f, 1.0f}, 0.2f, "r2");

    EXPECT_EQ(initialCollection, &simulation.robots().robots());
    ASSERT_EQ(initialCollection->size(), 2);
    EXPECT_TRUE(B2_ID_EQUALS(initialCollection->at(0), first));
    EXPECT_TRUE(B2_ID_EQUALS(initialCollection->at(1), second));
}

TEST(Simulation, ClearsRobotBodiesAndSimulationStateTogether) {
    auto environment = makeEnvironment();
    Simulation simulation(environment);
    simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    simulation.stepOnce(0.1f);

    ASSERT_EQ(environment.numRobots(), 1);
    ASSERT_EQ(simulation.robots().robots().size(), 1);
    ASSERT_GT(simulation.currentSimTime(), 0.0f);

    simulation.clearAllRobots();

    EXPECT_EQ(environment.numRobots(), 0);
    EXPECT_TRUE(simulation.robots().robots().empty());
    EXPECT_FLOAT_EQ(simulation.currentSimTime(), 0.0f);
    EXPECT_NO_THROW(simulation.stepOnce(0.1f));
}
