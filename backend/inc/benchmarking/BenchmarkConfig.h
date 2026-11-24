// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstddef>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace grace::benchmarking {

enum class BenchEnv { Grid, Roadmap, Continuous };

struct RunOpts {
    BenchEnv env{BenchEnv::Grid};

    std::filesystem::path gridMap;
    std::filesystem::path gridScenario;
    std::filesystem::path gridAgents;
    std::filesystem::path gridTasks;
    float gridCellLength{1.0f};
    float gridBoundaryScale{0.5f};
    bool gridSkipBlockedGoals{true};

    std::filesystem::path graphXml;
    std::filesystem::path tasksXml;
    float roadmapWall{1.0f};
    float roadmapRobotRadius{0.25f};

    std::filesystem::path problemYaml;
    float continuousWall{0.25f};

    std::string solverName;
    std::size_t takeK{std::numeric_limits<std::size_t>::max()};
    int repetition{0};
    std::filesystem::path runDirectory;
    std::filesystem::path masterCsv;
    std::string sessionTag;
    bool writeStateCsv{false};
    int stateEveryN{60};
    float updateDt{1.0f / 60.0f};
    int substeps{4};
    std::filesystem::path mapLabel;
    std::filesystem::path scenarioLabel;
};

struct BenchmarkConfig {
    std::vector<std::filesystem::path> gridMaps;
    std::vector<std::filesystem::path> gridScenarios;
    std::vector<std::filesystem::path> gridAgents;
    std::vector<std::filesystem::path> gridTasks;
    std::vector<std::filesystem::path> roadmapGraphs;
    std::vector<std::filesystem::path> roadmapTasks;
    std::vector<std::filesystem::path> continuousProblems;
    std::vector<std::string> solvers;
    std::vector<int> takeK{-1};

    std::filesystem::path outputRoot{"results/benchmarking"};
    std::string sessionTag;
    int repeat{1};
    float updateDt{1.0f / 60.0f};
    int substeps{4};
    bool writeStateCsv{false};
    int stateEveryN{60};
    float gridCellLength{1.0f};
    float gridBoundaryScale{0.5f};
    bool gridSkipBlockedGoals{true};
    float roadmapWall{1.0f};
    float roadmapRobotRadius{0.25f};
    float continuousWall{0.25f};
};

struct BenchmarkCommandLine {
    BenchmarkConfig config;
    bool showHelp{false};
    std::string helpText;
};

[[nodiscard]] std::vector<int> parseTakeKTokens(
    const std::vector<std::string>& tokens);
[[nodiscard]] std::size_t effectiveTakeK(
    int requested, std::size_t available) noexcept;
[[nodiscard]] std::string takeKFolderTag(
    int requested, std::size_t effective);
[[nodiscard]] std::string pathStem(const std::filesystem::path& path);
[[nodiscard]] std::string makeSessionTag();

[[nodiscard]] BenchmarkCommandLine parseBenchmarkCommandLine(
    const std::vector<std::string>& arguments);
void validateBenchmarkConfig(const BenchmarkConfig& config);
[[nodiscard]] std::vector<RunOpts> makeBenchmarkSchedule(
    BenchmarkConfig config);

} // namespace grace::benchmarking
