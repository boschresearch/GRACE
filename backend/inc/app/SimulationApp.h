// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstddef>
#include <filesystem>
#include <iosfwd>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

class Simulation;

namespace grace::app {

enum class SimulationMode {
    Grid,
    Roadmap,
    Continuous,
};

enum class SimulationExitCode : int {
    Success = 0,
    RuntimeError = 1,
    InvalidArguments = 2,
    PlanningFailed = 3,
    GoalsNotReached = 4,
};

struct SimulationRunConfig {
    SimulationMode mode{SimulationMode::Grid};

    std::filesystem::path mapPath;
    std::filesystem::path scenarioPath;
    std::filesystem::path tasksPath;
    std::filesystem::path problemPath;
    std::filesystem::path resultsDirectory{"results"};

    std::string planner;
    std::string robotBaseName{"r"};
    std::size_t takeK{std::numeric_limits<std::size_t>::max()};

    float cellLength{1.0f};
    float robotRadius{0.07f};
    float wallThickness{0.25f};
    float updateDt{1.0f / 60.0f};
    int subStepCount{4};

    bool writeMetricsCsv{true};
    bool writeStateCsv{true};
    bool overwriteOutputs{true};
    int stateEveryNSteps{1};
};

struct SimulationCommandLine {
    SimulationRunConfig config;
    bool showHelp{false};
    std::string helpText;
};

[[nodiscard]] SimulationMode parseSimulationMode(std::string_view value);
[[nodiscard]] std::string_view simulationModeName(SimulationMode mode) noexcept;
[[nodiscard]] SimulationRunConfig defaultSimulationRunConfig(
    SimulationMode mode,
    const std::filesystem::path& dataRoot);
void validateSimulationRunConfig(const SimulationRunConfig& config);

[[nodiscard]] SimulationCommandLine parseSimulationCommandLine(
    const std::vector<std::string>& arguments,
    const std::filesystem::path& dataRoot);

void configureSimulation(Simulation& simulation,
                         const SimulationRunConfig& config);
[[nodiscard]] SimulationExitCode runSimulation(
    const SimulationRunConfig& config);
[[nodiscard]] int runSimulationMain(
    int argc,
    char* argv[],
    const std::filesystem::path& dataRoot,
    std::ostream& output,
    std::ostream& error);

} // namespace grace::app
