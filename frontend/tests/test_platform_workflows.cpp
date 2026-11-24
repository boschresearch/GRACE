// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "platform/PlanningWorkflow.h"
#include "platform/PlatformBase.h"
#include "platform/PlatformState.h"
#include "platform/ResetWorkflow.h"

#include "Simulation.h"
#include "env/ContinuousEnvironment.h"
#include "env/GridEnvironment.h"

#include <concepts>
#include <memory>
#include <type_traits>
#include <utility>

namespace {

template <typename Access>
concept ProvidesSelectedRobots = requires(Access access) {
    {
        access.selectedRobots()
    } -> std::same_as<const std::vector<b2BodyId>&>;
};

template <typename Access>
concept ProvidesPlannerSettings = requires(Access access) {
    {
        access.settings()
    } -> std::same_as<platform::PlannerUiState>;
};

template <typename Access>
concept ProvidesBroadStudioState = requires(Access access) {
    access.studio();
};

template <typename Access>
concept ProvidesPanelState = requires(Access access) {
    access.state();
};

template <typename Access>
concept ProvidesMutableContext = requires(Access access) {
    {
        access.context()
    } -> std::same_as<SampleContext&>;
};

template <typename Access>
concept ProvidesSimulationHandle = requires(Access access) {
    {
        access.simulation()
    } -> std::same_as<Simulation*>;
};

template <typename Access>
concept ProvidesVisualizationSettings = requires(Access access) {
    {
        access.visualizationSettings()
    } -> std::same_as<platform::SimulationVisualizationSettings>;
};

template <typename Access>
concept ProvidesEditorSettings = requires(Access access) {
    {
        access.editorSettings()
    } -> std::same_as<platform::EnvironmentEditorSettings>;
};

template <typename Access>
concept ProvidesProblemSpec = requires(
    Access access,
    const std::filesystem::path& source) {
    {
        access.problemSpec(source)
    } -> std::same_as<PlatformBase::ProblemSpecPtr>;
};

template <typename Access>
concept ProvidesPanelRouting = requires(Access access) {
    access.environmentPanel();
    access.agentsPanel();
    access.plannerPanel();
    access.simulationPanel();
};

template <typename Access>
concept ProvidesArbitraryWorkflowNavigation = requires(Access access) {
    access.navigateTo(platform::WorkflowStep::Environment);
};

template <typename Access>
concept ProvidesWorkflowPosition = requires(Access access) {
    {
        access.workflowStep()
    } -> std::same_as<platform::WorkflowStep>;
};

template <typename Access>
concept BraceConstructibleFromPlatform = requires(PlatformBase& platform) {
    Access({}, platform);
};

static_assert(ProvidesPanelRouting<PlatformBase::WorkflowAccess>);
static_assert(!ProvidesPanelRouting<PlatformBase::EnvironmentAccess>);
static_assert(ProvidesWorkflowPosition<PlatformBase::WorkflowAccess>);
static_assert(!ProvidesWorkflowPosition<PlatformBase::EnvironmentAccess>);
static_assert(ProvidesArbitraryWorkflowNavigation<PlatformBase::WorkflowAccess>);
static_assert(
    !ProvidesArbitraryWorkflowNavigation<PlatformBase::EnvironmentAccess>);
static_assert(!ProvidesArbitraryWorkflowNavigation<PlatformBase::AgentsAccess>);
static_assert(!ProvidesArbitraryWorkflowNavigation<PlatformBase::PlannerAccess>);
static_assert(
    !ProvidesArbitraryWorkflowNavigation<PlatformBase::SimulationAccess>);
static_assert(ProvidesSelectedRobots<PlatformBase::AgentsAccess>);
static_assert(!ProvidesSelectedRobots<PlatformBase::PlannerAccess>);
static_assert(ProvidesPlannerSettings<PlatformBase::PlannerAccess>);
static_assert(!ProvidesPlannerSettings<PlatformBase::SimulationAccess>);
static_assert(ProvidesVisualizationSettings<PlatformBase::SimulationAccess>);
static_assert(!ProvidesVisualizationSettings<PlatformBase::EnvironmentAccess>);
static_assert(ProvidesEditorSettings<PlatformBase::EnvironmentAccess>);
static_assert(!ProvidesEditorSettings<PlatformBase::AgentsAccess>);
static_assert(ProvidesProblemSpec<PlatformBase::AgentsAccess>);
static_assert(!ProvidesProblemSpec<PlatformBase::PlannerAccess>);
static_assert(std::is_const_v<
              PlatformBase::ProblemSpecPtr::element_type>);
static_assert(ProvidesPanelState<PlatformBase::WorkflowAccess>);
static_assert(ProvidesPanelState<PlatformBase::EnvironmentAccess>);
static_assert(ProvidesPanelState<PlatformBase::AgentsAccess>);
static_assert(ProvidesPanelState<PlatformBase::PlannerAccess>);
static_assert(ProvidesPanelState<PlatformBase::SimulationAccess>);
static_assert(std::same_as<
              decltype(std::declval<PlatformBase::WorkflowAccess>().state()),
              platform::WorkflowPanelState>);
static_assert(std::same_as<
              decltype(std::declval<PlatformBase::EnvironmentAccess>().state()),
              platform::EnvironmentPanelState>);
static_assert(std::same_as<
              decltype(std::declval<PlatformBase::AgentsAccess>().state()),
              platform::AgentsPanelState>);
static_assert(std::same_as<
              decltype(std::declval<PlatformBase::PlannerAccess>().state()),
              platform::PlannerPanelState>);
static_assert(std::same_as<
              decltype(std::declval<PlatformBase::SimulationAccess>().state()),
              platform::SimulationPanelState>);
static_assert(!ProvidesBroadStudioState<PlatformBase::WorkflowAccess>);
static_assert(!ProvidesBroadStudioState<PlatformBase::EnvironmentAccess>);
static_assert(!ProvidesBroadStudioState<PlatformBase::AgentsAccess>);
static_assert(!ProvidesBroadStudioState<PlatformBase::PlannerAccess>);
static_assert(!ProvidesBroadStudioState<PlatformBase::SimulationAccess>);
static_assert(!ProvidesMutableContext<PlatformBase::WorkflowAccess>);
static_assert(!ProvidesMutableContext<PlatformBase::EnvironmentAccess>);
static_assert(!ProvidesMutableContext<PlatformBase::AgentsAccess>);
static_assert(!ProvidesMutableContext<PlatformBase::PlannerAccess>);
static_assert(!ProvidesMutableContext<PlatformBase::SimulationAccess>);
static_assert(ProvidesSimulationHandle<PlatformBase::AgentsAccess>);
static_assert(ProvidesSimulationHandle<PlatformBase::PlannerAccess>);
static_assert(!ProvidesSimulationHandle<PlatformBase::SimulationAccess>);
static_assert(std::same_as<
              decltype(std::declval<platform::SimulationPanelState>().running),
              const bool&>);
static_assert(!std::is_constructible_v<
              PlatformBase::WorkflowAccess, PlatformBase&>);
static_assert(!std::is_constructible_v<
              PlatformBase::EnvironmentAccess, PlatformBase&>);
static_assert(!std::is_constructible_v<
              PlatformBase::AgentsAccess, PlatformBase&>);
static_assert(!std::is_constructible_v<
              PlatformBase::PlannerAccess, PlatformBase&>);
static_assert(!std::is_constructible_v<
              PlatformBase::SimulationAccess, PlatformBase&>);
static_assert(!BraceConstructibleFromPlatform<PlatformBase::WorkflowAccess>);
static_assert(!BraceConstructibleFromPlatform<PlatformBase::EnvironmentAccess>);
static_assert(!BraceConstructibleFromPlatform<PlatformBase::AgentsAccess>);
static_assert(!BraceConstructibleFromPlatform<PlatformBase::PlannerAccess>);
static_assert(!BraceConstructibleFromPlatform<PlatformBase::SimulationAccess>);

class StraightLinePlanner final : public IPlanner {
public:
    const char* name() const noexcept override {
        return "straight-line";
    }

    EnvironmentType type() const noexcept override {
        return EnvironmentType::ContinuousEnvironment;
    }

    PlanResult plan(const PlanRequest& request) override {
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
};

ContinuousEnvironment makeContinuousEnvironment() {
    ContinuousEnvironment environment;
    environment.setBounds(8.0f, 6.0f);
    return environment;
}

}  // namespace

TEST(PlatformState, PreservesFrontendDefaults) {
    const platform::OverlaySettings overlays;
    const platform::PlaybackState playback;
    const platform::GridVisualizationState grid;
    const platform::ContinuousVisualizationState continuous;
    const platform::PlannerUiState planner;
    const platform::SimulationVisualizationSettings visualization;
    const platform::EnvironmentEditorSettings editorSettings;
    const platform::EditorState editor;

    EXPECT_TRUE(overlays.drawPlanned);
    EXPECT_TRUE(overlays.drawExecuted);
    EXPECT_FALSE(overlays.drawRoadmap);
    EXPECT_TRUE(playback.playGhost);
    EXPECT_FLOAT_EQ(playback.speed, 1.0f);
    EXPECT_TRUE(grid.showReservations);
    EXPECT_TRUE(grid.cacheDirty);
    EXPECT_FALSE(continuous.showClearance);
    EXPECT_EQ(continuous.clearanceSamplesPerMeter, 8);
    EXPECT_DOUBLE_EQ(planner.timeLimitSeconds, 45.0);
    EXPECT_TRUE(visualization.drawPlanned);
    EXPECT_TRUE(visualization.showGridReservations);
    EXPECT_EQ(editorSettings.mode, platform::EditorMode::None);
    EXPECT_EQ(editorSettings.brushRadius, 1);
    EXPECT_EQ(editor.mode, platform::EditorMode::None);
    EXPECT_EQ(editor.brushRadius, 1);
    EXPECT_FLOAT_EQ(editor.roadmapWallThickness, 0.14f);
}

TEST(PlatformState, KeepsWorkflowNavigationPerPlatformInstance) {
    platform::WorkflowNavigationState first;
    platform::WorkflowNavigationState second;

    EXPECT_EQ(first.current(), platform::WorkflowStep::Environment);
    EXPECT_EQ(second.current(), platform::WorkflowStep::Environment);

    first.navigateTo(platform::WorkflowStep::Planner);

    EXPECT_EQ(first.current(), platform::WorkflowStep::Planner);
    EXPECT_EQ(second.current(), platform::WorkflowStep::Environment);
}

TEST(PlatformState, KeepsEditorAndVisualizationStateIndependent) {
    platform::EditorState editor;
    platform::GridVisualizationState grid;
    platform::PlaybackState playback;

    editor.mode = platform::EditorMode::GridPaintBlock;
    editor.painting = true;
    grid.cacheDirty = false;
    playback.visualizationTime = 3.5;

    EXPECT_EQ(editor.mode, platform::EditorMode::GridPaintBlock);
    EXPECT_TRUE(editor.painting);
    EXPECT_FALSE(grid.cacheDirty);
    EXPECT_DOUBLE_EQ(playback.visualizationTime, 3.5);
}

TEST(PlatformState, KeepsRunningAndPauseStateSynchronized) {
    platform::StudioState studio;
    bool paused = true;

    platform::setSimulationRunning(studio, paused, true);
    EXPECT_TRUE(studio.running);
    EXPECT_FALSE(paused);

    platform::setSimulationRunning(studio, paused, false);
    EXPECT_FALSE(studio.running);
    EXPECT_TRUE(paused);
}

TEST(PlatformPlanningWorkflow, SelectsCompatiblePlannerOrFallback) {
    const auto compatible = platform::planning::selectPlanner(
        platform::EnvKind::Grid, "eecbs-bin");
    EXPECT_EQ(compatible.name, "eecbs-bin");
    EXPECT_FALSE(compatible.usedFallback);

    const auto fallback = platform::planning::selectPlanner(
        platform::EnvKind::Continuous, "eecbs-bin");
    EXPECT_EQ(fallback.name, "dbcbs-src");
    EXPECT_TRUE(fallback.usedFallback);
}

TEST(PlatformPlanningWorkflow, AssignsSnappedGoalAndInvalidatesOldPlan) {
    GridEnvironment environment;
    environment.makeGridEnvironment(4, 4, 1.0f, std::vector<bool>(16, false));
    Simulation simulation(environment);
    const b2BodyId robot = simulation.spawnGridCellRobot(0, 0, "r1");
    auto* metadata = simulation.robots().meta(robot);
    ASSERT_NE(metadata, nullptr);
    metadata->plan.waypoints.push_back(Waypoint{{0.5f, 0.5f}, 0.0f});
    metadata->next_wp = 1;

    const b2Vec2 requested{2.2f, 1.8f};
    const auto assignment = platform::planning::assignGoal(
        simulation, std::vector<b2BodyId>{robot}, requested);
    const b2Vec2 expected = environment.cellToWorld(environment.worldToCell(requested));

    EXPECT_EQ(assignment.updatedRobots, 1U);
    EXPECT_FLOAT_EQ(assignment.snappedGoal.x, expected.x);
    EXPECT_FLOAT_EQ(assignment.snappedGoal.y, expected.y);
    ASSERT_TRUE(metadata->requested_goal_m.has_value());
    EXPECT_FLOAT_EQ(metadata->requested_goal_m->x, expected.x);
    EXPECT_FLOAT_EQ(metadata->requested_goal_m->y, expected.y);
    EXPECT_TRUE(metadata->plan.waypoints.empty());
    EXPECT_EQ(metadata->next_wp, 0U);
}

TEST(PlatformPlanningWorkflow, ReplansFromCurrentPositionsToPreservedGoals) {
    auto environment = makeContinuousEnvironment();
    Simulation simulation(environment);
    const b2BodyId robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    simulation.setPlanner(std::make_unique<StraightLinePlanner>());

    auto* metadata = simulation.robots().meta(robot);
    ASSERT_NE(metadata, nullptr);
    metadata->requested_goal_m = b2Vec2{5.0f, 4.0f};

    ASSERT_TRUE(platform::planning::planPreservingGoals(
        simulation, simulation.robots().robots()));
    ASSERT_EQ(metadata->plan.waypoints.size(), 2U);
    EXPECT_FLOAT_EQ(metadata->plan.waypoints.front().p.x, 1.0f);
    EXPECT_FLOAT_EQ(metadata->plan.waypoints.front().p.y, 1.0f);
    EXPECT_FLOAT_EQ(metadata->plan.waypoints.back().p.x, 5.0f);
    EXPECT_FLOAT_EQ(metadata->plan.waypoints.back().p.y, 4.0f);
}

TEST(PlatformPlanningWorkflow, DiscardsPlansForAllRequestedRobots) {
    auto environment = makeContinuousEnvironment();
    Simulation simulation(environment);
    const b2BodyId first = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    const b2BodyId second = simulation.spawnCircleRobot({2.0f, 1.0f}, 0.2f, "r2");
    auto* firstMetadata = simulation.robots().meta(first);
    auto* secondMetadata = simulation.robots().meta(second);
    ASSERT_NE(firstMetadata, nullptr);
    ASSERT_NE(secondMetadata, nullptr);
    firstMetadata->plan.waypoints.push_back(Waypoint{{3.0f, 3.0f}, 1.0f});
    secondMetadata->plan.waypoints.push_back(Waypoint{{4.0f, 3.0f}, 1.0f});
    firstMetadata->next_wp = 1;
    secondMetadata->next_wp = 1;

    EXPECT_EQ(
        platform::planning::discardPlans(
            simulation, simulation.robots().robots()),
        2U);
    EXPECT_TRUE(firstMetadata->plan.waypoints.empty());
    EXPECT_TRUE(secondMetadata->plan.waypoints.empty());
    EXPECT_EQ(firstMetadata->next_wp, 0U);
    EXPECT_EQ(secondMetadata->next_wp, 0U);
}

TEST(PlatformResetWorkflow, RestoresPlanStartsAndControllerProgress) {
    auto environment = makeContinuousEnvironment();
    Simulation simulation(environment);
    const b2BodyId robot = simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1");
    simulation.setPlanner(std::make_unique<StraightLinePlanner>());

    auto* metadata = simulation.robots().meta(robot);
    ASSERT_NE(metadata, nullptr);
    metadata->requested_goal_m = b2Vec2{5.0f, 4.0f};
    ASSERT_TRUE(platform::planning::planPreservingGoals(
        simulation, simulation.robots().robots()));

    b2Body_SetTransform(robot, {3.0f, 3.0f}, b2Rot_identity);
    b2Body_SetLinearVelocity(robot, {2.0f, 1.0f});
    metadata->next_wp = 1;

    platform::reset::resetRobotsToPlanStarts(simulation);

    const b2Vec2 position = b2Body_GetPosition(robot);
    const b2Vec2 velocity = b2Body_GetLinearVelocity(robot);
    EXPECT_FLOAT_EQ(position.x, 1.0f);
    EXPECT_FLOAT_EQ(position.y, 1.0f);
    EXPECT_FLOAT_EQ(velocity.x, 0.0f);
    EXPECT_FLOAT_EQ(velocity.y, 0.0f);
    EXPECT_EQ(metadata->next_wp, 0U);
}

TEST(PlatformResetWorkflow, ClearsSimulationAndEnvironmentRobotState) {
    auto environment = makeContinuousEnvironment();
    Simulation simulation(environment);
    ASSERT_TRUE(B2_IS_NON_NULL(
        simulation.spawnCircleRobot({1.0f, 1.0f}, 0.2f, "r1")));

    platform::reset::clearRobots(&simulation, &environment);

    EXPECT_TRUE(simulation.robots().robots().empty());
    EXPECT_TRUE(environment.robots().empty());
    EXPECT_EQ(environment.numRobots(), 0U);
}

TEST(PlatformResetWorkflow, ClearsEnvironmentWithoutSimulation) {
    auto environment = makeContinuousEnvironment();
    ASSERT_TRUE(B2_IS_NON_NULL(environment.addCircleBody(
        {1.0f, 1.0f}, 0.2f, Type::BodyType::Robot)));

    platform::reset::clearRobots(nullptr, &environment);

    EXPECT_TRUE(environment.robots().empty());
    EXPECT_EQ(environment.numRobots(), 0U);
}
