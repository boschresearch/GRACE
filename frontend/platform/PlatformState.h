// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "box2d/box2d.h"
#include "io/RoadmapXML.h"
#include "util/Metrics.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace platform {

enum class WorkflowStep {
    Environment,
    Agents,
    Planner,
    Simulation,
};

class WorkflowNavigationState {
public:
    [[nodiscard]] WorkflowStep current() const noexcept { return m_current; }
    void navigateTo(const WorkflowStep step) noexcept { m_current = step; }

private:
    WorkflowStep m_current{WorkflowStep::Environment};
};

struct RobotDisplayState {
    std::vector<b2BodyId> selected;
    std::unordered_map<uint64_t, b2HexColor> colors;
    uint32_t paletteSeed{0};
};

struct ExecutionTraceState {
    std::unordered_map<uint64_t, std::vector<b2Vec2>> points;
    std::size_t maxPoints{8000};
    int decimate{1};
    int counter{0};
};

struct OverlaySettings {
    bool drawPlanned{true};
    bool drawExecuted{true};
    bool drawRoadmap{false};
    bool themeObstaclesLight{true};
    bool tintRobots{true};
    bool showHover{true};
    bool showStartGoal{false};
    float startGoalDashPx{8.0f};
    float startGoalGapPx{5.0f};
    float goalMarkerPx{4.0f};
    float pathThicknessPx{3.0f};
};

struct PlaybackState {
    bool playGhost{true};
    bool loop{true};
    bool syncToSimulation{false};
    float speed{1.0f};
    double visualizationTime{0.0};
};

struct GridVisualizationState {
    struct CellInterval {
        int row{};
        int column{};
        double startTime{};
        double endTime{};
        uint64_t robotKey{};
    };

    bool showReservations{true};
    std::vector<CellInterval> reservations;
    float reservationWindow{1.0f};
    bool cacheDirty{true};
};

struct ContinuousVisualizationState {
    bool showClearance{false};
    int clearanceSamplesPerMeter{8};
    float clearanceMax{2.0f};
};

struct PlannerUiState {
    std::string name;
    double timeLimitSeconds{45.0};
};

struct SimulationVisualizationSettings {
    bool drawPlanned{true};
    bool drawExecuted{true};
    bool drawRoadmap{false};
    bool showStartGoal{false};
    float startGoalDashPx{8.0f};
    float startGoalGapPx{5.0f};
    bool playGhost{true};
    float ghostSpeed{1.0f};
    bool showGridReservations{true};
    bool showContinuousClearance{false};
    int clearanceSamplesPerMeter{8};
    float clearanceMax{2.0f};
};

enum class EditorMode {
    None,
    GridPaintBlock,
    GridPaintErase,
    RoadmapAddNode,
    RoadmapAddEdgeUndirected,
    RoadmapAddEdgeDirected,
    ContinuousAddBox,
    ContinuousAddCircle
};

struct EnvironmentEditorSettings {
    EditorMode mode{EditorMode::None};
    int brushRadius{1};
    bool autoReplan{false};
    bool rebuildWallsOnApply{false};
    float boxWidth{0.6f};
    float boxHeight{0.6f};
    float circleRadius{0.25f};
    float nodePickRadiusPx{12.0f};
    float roadmapWallThickness{0.14f};
};

struct EditorState {
    EditorMode mode{EditorMode::None};

    bool painting{false};
    int brushRadius{1};
    bool autoReplan{false};
    bool rebuildWallsOnApply{false};
    std::unordered_set<uint64_t> paintedCells;
    bool paintBlocked{true};
    std::vector<std::tuple<int, int, bool>> paintOperations;

    float boxWidth{0.6f};
    float boxHeight{0.6f};
    float circleRadius{0.25f};

    int firstEdgeNode{-1};
    float nodePickRadiusPx{12.0f};
    float roadmapWallThickness{0.14f};
    grace::sim::GraphMLImport roadmapImport{};
    std::optional<grace::sim::GraphMLImport> graphImport;
    bool hasRoadmapImport{false};
};

struct StudioState {
    bool showStudio{true};
    bool showAdvancedPanels{false};

    int envChoice{0};

    bool envReady{false};
    bool newEmpty{false};
    std::string mapPath;
    float cellLength{1.0f};
    int newW{20};
    int newH{20};
    float newCell{1.0f};

    bool agentsReady{false};
    bool agentsFromScenario{true};
    std::string scenPath;
    std::string agentsPath;
    std::string tasksPath;
    std::string roadmapPath;
    std::string taskXmlPath;
    float roadRobotRadius{0.07f};
    int takeK{10};
    std::string baseName{"r"};
    std::string problemYamlPath;
    float freeWallThickness{0.25f};
    float freeRobotRadius{0.08f};
    float freeNewW{20.0f};
    float freeNewH{12.0f};

    bool planReady{false};

    bool running{false};
    std::string status;
    double plannedTime{0.0};
    double plannerRuntime{0.0};
    std::optional<metrics::Aggregate> planMetrics;

    bool leftNavOpen{true};
    bool detailsOpen{true};
    bool autoHideDetailsWhenRunning{true};
    float detailsWidthPx{440.0f};
    bool envDetailsOpen{false};
    bool agentDetailsOpen{false};
};

struct WorkflowPanelState {
    const bool& agentsReady;
    const bool& planReady;
    const bool& running;
    const std::string& status;
    bool& leftNavOpen;
    bool& detailsOpen;
    bool& autoHideDetailsWhenRunning;
    float& detailsWidthPx;
};

struct EnvironmentPanelState {
    int& envChoice;
    std::string& mapPath;
    float& cellLength;
    int& newW;
    int& newH;
    float& newCell;
    std::string& roadmapPath;
    std::string& problemYamlPath;
    float& freeWallThickness;
    float& freeNewW;
    float& freeNewH;
    bool& leftNavOpen;
    std::string& status;
};

struct AgentsPanelState {
    bool& agentsReady;
    bool& agentsFromScenario;
    std::string& scenPath;
    std::string& agentsPath;
    std::string& tasksPath;
    std::string& taskXmlPath;
    std::string& problemYamlPath;
    float& roadRobotRadius;
    int& takeK;
    std::string& baseName;
    bool& planReady;
    bool& leftNavOpen;
    std::string& status;
};

struct PlannerPanelState {
    bool& planReady;
    double& plannedTime;
    double& plannerRuntime;
    std::optional<metrics::Aggregate>& planMetrics;
    bool& leftNavOpen;
    std::string& status;
};

struct SimulationPanelState {
    const bool& planReady;
    const bool& running;
    std::string& status;
};

inline void setSimulationRunning(
    StudioState& studio,
    bool& paused,
    const bool running) noexcept {
    studio.running = running;
    paused = !running;
}

} // namespace platform
