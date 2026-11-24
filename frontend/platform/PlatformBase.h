// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "box2d/box2d.h"
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "PlatformState.h"

// Forward decls to keep compile times down
struct ImFont;
class BaseEnvironment;
class GridEnvironment;
class RoadmapEnvironment;
class ContinuousEnvironment;
class Simulation;
namespace grace::sim {
struct ProblemYaml;
}

#include "../sample/Sample.h"   // Sample base interface (kept thin)

//------------------------------------------------------------------------------
// PlatformBase
// Superclass for MAPF “platform” samples (Grid, Roadmap, Continuous). Owns an
// environment + simulation and supplies editing, planning and visualization
// helpers callable from UI.
//------------------------------------------------------------------------------
class PlatformBase : public Sample
{
    // Capability constructors require this private lvalue, so UI code cannot
    // manufacture broader access than the workflow router grants it.
    struct AccessKey {};
    [[no_unique_address]] AccessKey m_accessKey;

public:
    using ProblemSpecPtr = std::shared_ptr<const grace::sim::ProblemYaml>;

    class WorkflowAccess;
    class EnvironmentAccess;
    class AgentsAccess;
    class PlannerAccess;
    class SimulationAccess;

    using EditMode = platform::EditorMode;

    explicit PlatformBase(SampleContext* ctx);
    ~PlatformBase() override;
    bool OnEscape() override; // ESC cancels pending edits instead of quitting

    bool hasEnvironment() const noexcept { return static_cast<bool>(m_envOwned); }
    void centerCameraOnEnvironment();

    // Sample virtuals
    void Step() override;
    void MouseDown(b2Vec2 p, int button, int mod) override;
    void MouseUp  (b2Vec2 p, int button) override;
    void MouseMove(b2Vec2 p) override;

protected:
    // ---------- owned platform parts ----------
    std::unique_ptr<BaseEnvironment> m_envOwned;
    std::unique_ptr<Simulation>  m_sim;
    ProblemSpecPtr m_problemSpec;
    std::filesystem::path m_problemSpecSource;
    std::string m_instanceInfo;

    platform::RobotDisplayState m_robotDisplay;
    platform::ExecutionTraceState m_executionTrace;
    platform::OverlaySettings m_overlays;
    platform::PlaybackState m_playback;
    platform::GridVisualizationState m_gridVisualization;
    platform::ContinuousVisualizationState m_continuousVisualization;
    platform::PlannerUiState m_plannerUi;
    platform::EditorState m_editor;
    platform::StudioState m_studio;
    platform::WorkflowNavigationState m_workflowNavigation;

    // ---------- panels ----------
    void drawPanels();

    // ---------- overlays ----------
    void drawPlannedPaths();
    void drawExecutedTraces();
    void drawSelectionBadges();
    void drawGhostFleet();
    void drawHoverCrosshair();
    void drawHoverReadout();     // textual hover info HUD
    void drawRoadmap();
    void drawGridReservations();
    void drawContinuousClearance();
    void drawStartGoalDashes();

    // ---------- HUD composition helpers ----------
    std::string composeHoverLine();           // builds "hover: ..." or returns empty
    std::string composeDetailsLine() const;   // builds one-line env details

    // ---------- sync ----------
    void syncWorldBinding();

    // ---------- editor handlers ----------
    void handleEditorClick(b2Vec2 p);
    void beginGridPaintAt(b2Vec2 p, bool blocked);
    void continueGridPaintAt(b2Vec2 p);
    void endGridPaint();
    void stampBrush(int r, int c, bool blocked);

    void addBoxAt(b2Vec2 p);
    void addCircleAt(b2Vec2 p);
    void addRoadNodeAt(b2Vec2 p);
    void addRoadEdgeAt(b2Vec2 p, bool directed);

    // ---------- planning ----------
    void reconfigurePlannerIfChanged(const std::string& name);
    bool replanPreservingGoals();
    bool replanRobotPreservingGoal(b2BodyId id);
    void spawnAtClick(b2Vec2 p);
    void installPlanner(const std::string& name);
    void assignGoalOnly(b2Vec2 worldGoal);
    bool assignRobotGoalOnly(b2BodyId id, b2Vec2 worldGoal);
    void markPlansStale(const std::vector<b2BodyId>& ids);
    void resetRobotsToPlanStarts();
    void invalidatePlanPresentation();
    void adoptEnvironment(std::unique_ptr<BaseEnvironment> env,
                          std::unique_ptr<Simulation> sim,
                          const char* status,
                          ProblemSpecPtr problemSpec = {},
                          std::filesystem::path problemSpecSource = {});
    [[nodiscard]] ProblemSpecPtr loadProblemSpec(
        const std::filesystem::path& source) const;
    [[nodiscard]] ProblemSpecPtr problemSpec(
        const std::filesystem::path& source);
    void ensureSimulation();
    void setSimulationRunning(bool running) noexcept;
    void showSimulationDetails() noexcept;
    [[nodiscard]] bool simulationPaused() const noexcept;
    [[nodiscard]] float uiScale() const noexcept;
    [[nodiscard]] float overlaySafeTopPx() const noexcept;
    [[nodiscard]] float toolsPanelWidthFrac() const noexcept;
    [[nodiscard]] ImFont* mediumFont() const noexcept;
    [[nodiscard]] std::optional<b2Vec2> worldPositionForPixel(
        int x, int y) const;
    void centerCameraOnPoint(b2Vec2 point) const;

    // ---------- helpers ----------
    GridEnvironment*          grid();
    RoadmapEnvironment*       road();
    ContinuousEnvironment*          freeenv();
    const GridEnvironment*    grid()    const;
    const RoadmapEnvironment* road()    const;
    const ContinuousEnvironment*    freeenv() const;
    bool isGrid()       const;
    bool isRoadmap()    const;
    bool isContinuous() const;
    void clearRobots();

    void adoptWorld(b2WorldId wid);
    void centerOnEnv(const BaseEnvironment& E);

    float pxToWorld(float px) const;
    float pickRadius() const;

    static uint64_t idKey(b2BodyId id);
    void ensurePalette();
    void recordExecutedOnce();
    void drawGridCellRect(int r, int c, b2HexColor col);
    void rebuildGridReservations();

    bool hasAgents() const noexcept;
};

class PlatformBase::WorkflowAccess {
public:
    WorkflowAccess(AccessKey&, PlatformBase& platform) : m_platform(platform) {}

    platform::WorkflowPanelState state() const noexcept {
        auto& studio = m_platform.m_studio;
        return {
            .agentsReady = studio.agentsReady,
            .planReady = studio.planReady,
            .running = studio.running,
            .status = studio.status,
            .leftNavOpen = studio.leftNavOpen,
            .detailsOpen = studio.detailsOpen,
            .autoHideDetailsWhenRunning =
                studio.autoHideDetailsWhenRunning,
            .detailsWidthPx = studio.detailsWidthPx,
        };
    }
    float overlaySafeTopPx() const noexcept {
        return m_platform.overlaySafeTopPx();
    }
    float toolsPanelWidthFrac() const noexcept {
        return m_platform.toolsPanelWidthFrac();
    }
    ImFont* mediumFont() const noexcept { return m_platform.mediumFont(); }
    bool hasEnvironment() const noexcept { return m_platform.hasEnvironment(); }
    bool hasAgents() const noexcept { return m_platform.hasAgents(); }
    platform::WorkflowStep workflowStep() const noexcept {
        return m_platform.m_workflowNavigation.current();
    }
    void navigateTo(platform::WorkflowStep step) const noexcept {
        m_platform.m_workflowNavigation.navigateTo(step);
    }

    EnvironmentAccess environmentPanel() const noexcept;
    AgentsAccess agentsPanel() const noexcept;
    PlannerAccess plannerPanel() const noexcept;
    SimulationAccess simulationPanel() const noexcept;

private:
    PlatformBase& m_platform;
};

class PlatformBase::EnvironmentAccess {
public:
    EnvironmentAccess(AccessKey&, PlatformBase& platform) : m_platform(platform) {}

    platform::EnvironmentPanelState state() const noexcept {
        auto& studio = m_platform.m_studio;
        return {
            .envChoice = studio.envChoice,
            .mapPath = studio.mapPath,
            .cellLength = studio.cellLength,
            .newW = studio.newW,
            .newH = studio.newH,
            .newCell = studio.newCell,
            .roadmapPath = studio.roadmapPath,
            .problemYamlPath = studio.problemYamlPath,
            .freeWallThickness = studio.freeWallThickness,
            .freeNewW = studio.freeNewW,
            .freeNewH = studio.freeNewH,
            .leftNavOpen = studio.leftNavOpen,
            .status = studio.status,
        };
    }
    platform::EnvironmentEditorSettings editorSettings() const noexcept {
        const auto& editor = m_platform.m_editor;
        return {
            .mode = editor.mode,
            .brushRadius = editor.brushRadius,
            .autoReplan = editor.autoReplan,
            .rebuildWallsOnApply = editor.rebuildWallsOnApply,
            .boxWidth = editor.boxWidth,
            .boxHeight = editor.boxHeight,
            .circleRadius = editor.circleRadius,
            .nodePickRadiusPx = editor.nodePickRadiusPx,
            .roadmapWallThickness = editor.roadmapWallThickness,
        };
    }
    void setEditorSettings(
        const platform::EnvironmentEditorSettings& settings) const noexcept {
        auto& editor = m_platform.m_editor;
        editor.mode = settings.mode;
        editor.brushRadius = settings.brushRadius;
        editor.autoReplan = settings.autoReplan;
        editor.rebuildWallsOnApply = settings.rebuildWallsOnApply;
        editor.boxWidth = settings.boxWidth;
        editor.boxHeight = settings.boxHeight;
        editor.circleRadius = settings.circleRadius;
        editor.nodePickRadiusPx = settings.nodePickRadiusPx;
        editor.roadmapWallThickness = settings.roadmapWallThickness;
    }
    void clearRoadmapImport() const {
        m_platform.m_editor.graphImport.reset();
        m_platform.m_editor.hasRoadmapImport = false;
    }
    void setRoadmapImport(const grace::sim::GraphMLImport& import) const {
        m_platform.m_editor.roadmapImport = import;
        m_platform.m_editor.graphImport = import;
        m_platform.m_editor.hasRoadmapImport = true;
    }
    void showEnvironmentStep() const noexcept {
        m_platform.m_workflowNavigation.navigateTo(
            platform::WorkflowStep::Environment);
    }
    void showAgentsStep() const noexcept {
        m_platform.m_workflowNavigation.navigateTo(
            platform::WorkflowStep::Agents);
    }
    bool drawsRoadmap() const noexcept {
        return m_platform.m_overlays.drawRoadmap;
    }
    void setDrawRoadmap(bool enabled) const noexcept {
        m_platform.m_overlays.drawRoadmap = enabled;
    }
    void markGridVisualizationDirty() const noexcept {
        m_platform.m_gridVisualization.cacheDirty = true;
    }
    float uiScale() const noexcept { return m_platform.uiScale(); }

    BaseEnvironment* environment() const noexcept { return m_platform.m_envOwned.get(); }
    GridEnvironment* grid() const { return m_platform.grid(); }

    void adoptEnvironment(std::unique_ptr<BaseEnvironment> environment,
                          std::unique_ptr<Simulation> simulation,
                          const char* status,
                          ProblemSpecPtr problemSpec = {},
                          std::filesystem::path problemSpecSource = {});
    [[nodiscard]] ProblemSpecPtr loadProblemSpec(
        const std::filesystem::path& source) const {
        return m_platform.loadProblemSpec(source);
    }
    void refreshWorldBinding() const { m_platform.syncWorldBinding(); }
    void centerCamera() const { m_platform.centerCameraOnEnvironment(); }
    bool replanPreservingGoals() const { return m_platform.replanPreservingGoals(); }

private:
    PlatformBase& m_platform;
};

class PlatformBase::AgentsAccess {
public:
    AgentsAccess(AccessKey&, PlatformBase& platform) : m_platform(platform) {}

    const std::vector<b2BodyId>& selectedRobots() const noexcept {
        return m_platform.m_robotDisplay.selected;
    }
    void clearSelection() const { m_platform.m_robotDisplay.selected.clear(); }
    const platform::EditorState& editor() const noexcept {
        return m_platform.m_editor;
    }
    platform::AgentsPanelState state() const noexcept {
        auto& studio = m_platform.m_studio;
        return {
            .agentsReady = studio.agentsReady,
            .agentsFromScenario = studio.agentsFromScenario,
            .scenPath = studio.scenPath,
            .agentsPath = studio.agentsPath,
            .tasksPath = studio.tasksPath,
            .taskXmlPath = studio.taskXmlPath,
            .problemYamlPath = studio.problemYamlPath,
            .roadRobotRadius = studio.roadRobotRadius,
            .takeK = studio.takeK,
            .baseName = studio.baseName,
            .planReady = studio.planReady,
            .leftNavOpen = studio.leftNavOpen,
            .status = studio.status,
        };
    }
    float uiScale() const noexcept { return m_platform.uiScale(); }
    std::optional<b2Vec2> worldPositionForPixel(int x, int y) const {
        return m_platform.worldPositionForPixel(x, y);
    }
    void centerCameraOnPoint(b2Vec2 point) const {
        m_platform.centerCameraOnPoint(point);
    }

    BaseEnvironment* environment() const noexcept { return m_platform.m_envOwned.get(); }
    Simulation* simulation() const noexcept { return m_platform.m_sim.get(); }
    GridEnvironment* grid() const { return m_platform.grid(); }
    RoadmapEnvironment* roadmap() const { return m_platform.road(); }
    ContinuousEnvironment* continuous() const { return m_platform.freeenv(); }
    bool isGrid() const { return m_platform.isGrid(); }
    bool isRoadmap() const { return m_platform.isRoadmap(); }

    void ensureSimulation() const { m_platform.ensureSimulation(); }
    [[nodiscard]] ProblemSpecPtr problemSpec(
        const std::filesystem::path& source) const {
        return m_platform.problemSpec(source);
    }
    void clearRobots() const { m_platform.clearRobots(); }
    void ensurePalette() const { m_platform.ensurePalette(); }
    void centerCamera() const { m_platform.centerCameraOnEnvironment(); }
    void invalidatePlanPresentation() const {
        m_platform.invalidatePlanPresentation();
    }
    bool assignRobotGoal(b2BodyId id, b2Vec2 goal) const {
        return m_platform.assignRobotGoalOnly(id, goal);
    }
    bool replanRobotPreservingGoal(b2BodyId id) const {
        return m_platform.replanRobotPreservingGoal(id);
    }
    void markPlansStale(const std::vector<b2BodyId>& ids) const {
        m_platform.markPlansStale(ids);
    }
    void showPlannerStep() const noexcept {
        m_platform.m_workflowNavigation.navigateTo(
            platform::WorkflowStep::Planner);
    }

private:
    PlatformBase& m_platform;
};

class PlatformBase::PlannerAccess {
public:
    PlannerAccess(AccessKey&, PlatformBase& platform) : m_platform(platform) {}

    platform::PlannerUiState settings() const {
        return m_platform.m_plannerUi;
    }
    platform::PlannerPanelState state() const noexcept {
        auto& studio = m_platform.m_studio;
        return {
            .planReady = studio.planReady,
            .plannedTime = studio.plannedTime,
            .plannerRuntime = studio.plannerRuntime,
            .planMetrics = studio.planMetrics,
            .leftNavOpen = studio.leftNavOpen,
            .status = studio.status,
        };
    }
    float uiScale() const noexcept { return m_platform.uiScale(); }

    bool hasEnvironment() const noexcept { return m_platform.hasEnvironment(); }
    Simulation* simulation() const noexcept { return m_platform.m_sim.get(); }
    bool isGrid() const { return m_platform.isGrid(); }
    bool isRoadmap() const { return m_platform.isRoadmap(); }

    void ensureSimulation() const { m_platform.ensureSimulation(); }
    void setPlannerTimeLimit(double seconds) const {
        m_platform.m_plannerUi.timeLimitSeconds = seconds;
        m_platform.installPlanner(m_platform.m_plannerUi.name);
    }
    void reconfigurePlanner(const std::string& name) const {
        m_platform.reconfigurePlannerIfChanged(name);
    }
    bool replanPreservingGoals() const {
        return m_platform.replanPreservingGoals();
    }
    void resetRobotsToPlanStarts() const {
        m_platform.resetRobotsToPlanStarts();
    }
    void showSimulationStep() const noexcept {
        m_platform.m_workflowNavigation.navigateTo(
            platform::WorkflowStep::Simulation);
    }

private:
    PlatformBase& m_platform;
};

class PlatformBase::SimulationAccess {
public:
    SimulationAccess(AccessKey&, PlatformBase& platform) : m_platform(platform) {}

    platform::SimulationVisualizationSettings visualizationSettings() const noexcept {
        const auto& overlays = m_platform.m_overlays;
        const auto& playback = m_platform.m_playback;
        const auto& grid = m_platform.m_gridVisualization;
        const auto& continuous = m_platform.m_continuousVisualization;
        return {
            .drawPlanned = overlays.drawPlanned,
            .drawExecuted = overlays.drawExecuted,
            .drawRoadmap = overlays.drawRoadmap,
            .showStartGoal = overlays.showStartGoal,
            .startGoalDashPx = overlays.startGoalDashPx,
            .startGoalGapPx = overlays.startGoalGapPx,
            .playGhost = playback.playGhost,
            .ghostSpeed = playback.speed,
            .showGridReservations = grid.showReservations,
            .showContinuousClearance = continuous.showClearance,
            .clearanceSamplesPerMeter =
                continuous.clearanceSamplesPerMeter,
            .clearanceMax = continuous.clearanceMax,
        };
    }
    void setVisualizationSettings(
        const platform::SimulationVisualizationSettings& settings) const noexcept {
        auto& overlays = m_platform.m_overlays;
        overlays.drawPlanned = settings.drawPlanned;
        overlays.drawExecuted = settings.drawExecuted;
        overlays.drawRoadmap = settings.drawRoadmap;
        overlays.showStartGoal = settings.showStartGoal;
        overlays.startGoalDashPx = settings.startGoalDashPx;
        overlays.startGoalGapPx = settings.startGoalGapPx;
        m_platform.m_playback.playGhost = settings.playGhost;
        m_platform.m_playback.speed = settings.ghostSpeed;
        m_platform.m_gridVisualization.showReservations =
            settings.showGridReservations;
        m_platform.m_continuousVisualization.showClearance =
            settings.showContinuousClearance;
        m_platform.m_continuousVisualization.clearanceSamplesPerMeter =
            settings.clearanceSamplesPerMeter;
        m_platform.m_continuousVisualization.clearanceMax =
            settings.clearanceMax;
    }
    platform::SimulationPanelState state() const noexcept {
        auto& studio = m_platform.m_studio;
        return {
            .planReady = studio.planReady,
            .running = studio.running,
            .status = studio.status,
        };
    }
    bool hasSimulation() const noexcept {
        return static_cast<bool>(m_platform.m_sim);
    }
    bool isRoadmap() const { return m_platform.isRoadmap(); }
    bool isContinuous() const { return m_platform.isContinuous(); }
    bool paused() const noexcept { return m_platform.simulationPaused(); }
    float overlaySafeTopPx() const noexcept {
        return m_platform.overlaySafeTopPx();
    }
    void setRunning(bool running) const noexcept {
        m_platform.setSimulationRunning(running);
    }
    void showDetails() const noexcept { m_platform.showSimulationDetails(); }

    void resetRobotsToPlanStarts() const {
        m_platform.resetRobotsToPlanStarts();
    }

private:
    PlatformBase& m_platform;
};

inline PlatformBase::EnvironmentAccess
PlatformBase::WorkflowAccess::environmentPanel() const noexcept {
    return EnvironmentAccess(m_platform.m_accessKey, m_platform);
}

inline PlatformBase::AgentsAccess
PlatformBase::WorkflowAccess::agentsPanel() const noexcept {
    return AgentsAccess(m_platform.m_accessKey, m_platform);
}

inline PlatformBase::PlannerAccess
PlatformBase::WorkflowAccess::plannerPanel() const noexcept {
    return PlannerAccess(m_platform.m_accessKey, m_platform);
}

inline PlatformBase::SimulationAccess
PlatformBase::WorkflowAccess::simulationPanel() const noexcept {
    return SimulationAccess(m_platform.m_accessKey, m_platform);
}
