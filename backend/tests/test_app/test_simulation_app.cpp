// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "app/SimulationApp.h"

#include <filesystem>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
using grace::app::SimulationExitCode;
using grace::app::SimulationMode;

TEST(SimulationApp, ProvidesModeSpecificDefaults)
{
    const fs::path dataRoot = "/tmp/grace-data";

    const auto grid = grace::app::defaultSimulationRunConfig(
        SimulationMode::Grid, dataRoot);
    EXPECT_EQ(grid.planner, "lacam0-src");
    EXPECT_EQ(
        grid.mapPath,
        dataRoot / "maps/movingAI/default/demo.map");
    EXPECT_EQ(
        grid.scenarioPath,
        dataRoot / "maps/movingAI/default/demo.scen");

    const auto roadmap = grace::app::defaultSimulationRunConfig(
        SimulationMode::Roadmap, dataRoot);
    EXPECT_EQ(roadmap.planner, "psippctc-src");
    EXPECT_EQ(
        roadmap.mapPath,
        dataRoot / "maps/xml/default/roadmap/demo_roadmap.xml");
    EXPECT_EQ(
        roadmap.tasksPath,
        dataRoot / "maps/xml/default/roadmap/demo_tasks.xml");
    EXPECT_FLOAT_EQ(roadmap.wallThickness, 1.0f);

    const auto continuous = grace::app::defaultSimulationRunConfig(
        SimulationMode::Continuous, dataRoot);
    EXPECT_EQ(continuous.planner, "dbcbs-src");
    EXPECT_EQ(
        continuous.problemPath,
        dataRoot / "maps/yaml/default/demo.yaml");
}

TEST(SimulationApp, ParsesPositionalModeAndOverrides)
{
    const auto commandLine = grace::app::parseSimulationCommandLine(
        {
            "roadmap",
            "--map", "/tmp/custom-graph.xml",
            "--tasks", "/tmp/custom-tasks.xml",
            "--planner", "ccbs-src",
            "--results-dir", "/tmp/results",
            "--take-k", "7",
            "--robot-radius", "0.12",
            "--wall-thickness", "0.4",
            "--dt", "0.02",
            "--substeps", "6",
            "--state-every", "5",
            "--no-state",
            "--append",
        },
        "/tmp/default-data");

    const auto& config = commandLine.config;
    EXPECT_EQ(config.mode, SimulationMode::Roadmap);
    EXPECT_EQ(config.mapPath, "/tmp/custom-graph.xml");
    EXPECT_EQ(config.tasksPath, "/tmp/custom-tasks.xml");
    EXPECT_EQ(config.planner, "ccbs-src");
    EXPECT_EQ(config.resultsDirectory, "/tmp/results");
    EXPECT_EQ(config.takeK, 7U);
    EXPECT_FLOAT_EQ(config.robotRadius, 0.12f);
    EXPECT_FLOAT_EQ(config.wallThickness, 0.4f);
    EXPECT_FLOAT_EQ(config.updateDt, 0.02f);
    EXPECT_EQ(config.subStepCount, 6);
    EXPECT_EQ(config.stateEveryNSteps, 5);
    EXPECT_TRUE(config.writeMetricsCsv);
    EXPECT_FALSE(config.writeStateCsv);
    EXPECT_FALSE(config.overwriteOutputs);
}

TEST(SimulationApp, UsesAllAgentsAndEmitsHelpByDefault)
{
    const auto commandLine = grace::app::parseSimulationCommandLine(
        {"--help"}, "/tmp/default-data");

    EXPECT_TRUE(commandLine.showHelp);
    EXPECT_EQ(commandLine.config.mode, SimulationMode::Grid);
    EXPECT_EQ(
        commandLine.config.takeK,
        std::numeric_limits<std::size_t>::max());
    EXPECT_NE(commandLine.helpText.find("grid|roadmap|continuous"),
              std::string::npos);
}

TEST(SimulationApp, RejectsInvalidModeAndNumericOptions)
{
    EXPECT_THROW(
        grace::app::parseSimulationCommandLine(
            {"unknown"}, "/tmp/default-data"),
        std::invalid_argument);
    EXPECT_THROW(
        grace::app::parseSimulationCommandLine(
            {"grid", "--take-k", "0"}, "/tmp/default-data"),
        std::invalid_argument);
    EXPECT_THROW(
        grace::app::parseSimulationCommandLine(
            {"grid", "--substeps", "0"}, "/tmp/default-data"),
        std::invalid_argument);
}

TEST(SimulationApp, MainReportsArgumentErrorsWithoutRunning)
{
    char program[] = "grace-backend";
    char mode[] = "invalid-mode";
    char* arguments[] = {program, mode};
    std::ostringstream output;
    std::ostringstream error;

    const int result = grace::app::runSimulationMain(
        2, arguments, "/tmp/default-data", output, error);

    EXPECT_EQ(
        result,
        static_cast<int>(SimulationExitCode::InvalidArguments));
    EXPECT_TRUE(output.str().empty());
    EXPECT_NE(error.str().find("Unknown simulation mode"),
              std::string::npos);
}

} // namespace
