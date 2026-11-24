// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "TestFiles.h"
#include "tools/MapConvert.h"
#include "tools/MapConvertCli.h"
#include "util/RunOutput.h"

#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

using grace::test::TempDirectory;
using namespace tools::mc;

namespace {

struct CliResult {
    int exitCode;
    std::string output;
    std::string error;
};

CliResult runCli(std::initializer_list<std::string_view> arguments) {
    std::vector<std::string> storage;
    storage.reserve(arguments.size() + 1);
    storage.emplace_back("grace_backend_convert");
    for (const auto argument : arguments) {
        storage.emplace_back(argument);
    }

    std::vector<char*> argv;
    argv.reserve(storage.size());
    for (auto& argument : storage) {
        argv.push_back(argument.data());
    }

    std::ostringstream output;
    std::ostringstream error;
    const int exitCode = runMapConvertCli(
        static_cast<int>(argv.size()),
        argv.data(),
        {},
        output,
        error);
    return {exitCode, output.str(), error.str()};
}

}  // namespace

TEST(MapConvertCli, PrintsHelpWithoutRunningAConversion) {
    const auto result = runCli({"--help"});

    EXPECT_EQ(result.exitCode, 0);
    EXPECT_NE(result.output.find("Environment conversion tool"), std::string::npos);
    EXPECT_TRUE(result.error.empty());
}

TEST(MapConvertCli, RejectsUnknownModeBeforeCheckingPaths) {
    const auto result = runCli({"--mode", "unsupported"});

    EXPECT_EQ(result.exitCode, 9);
    EXPECT_NE(result.error.find("Unknown mode: unsupported"), std::string::npos);
}

TEST(MapConvertCli, ReportsModeSpecificRequiredOptions) {
    TempDirectory temp;
    grace::output::RunOutputConfig config;
    config.baseDirectory = temp.path("results");
    config.timestamp = "existing";
    const auto session = grace::output::startRunOutput(config);

    const auto result = runCli({
        "--mode", "y2t",
        "--in", "scenario.yaml",
        "--out", "tasks.xml",
    });

    EXPECT_EQ(result.exitCode, 13);
    EXPECT_NE(result.error.find("--graph required"), std::string::npos);
    EXPECT_EQ(grace::output::currentRunDirectory(), session);
    grace::output::stopRunOutput();
}

TEST(MapConvertCli, PreservesCallerOwnedRunOutputSession) {
    TempDirectory temp;
    grace::output::RunOutputConfig config;
    config.baseDirectory = temp.path("results");
    config.timestamp = "existing";
    const auto session = grace::output::startRunOutput(config);

    const auto result = runCli({
        "--mode", "c2r",
        "--in", temp.path("missing.yaml").string(),
        "--out", temp.path("roadmap.xml").string(),
    });

    EXPECT_EQ(result.exitCode, 10);
    EXPECT_EQ(grace::output::currentRunDirectory(), session);
    grace::output::stopRunOutput();
}

TEST(MapConvert, ContinuousYamlToMovingAIGrid) {
    static constexpr std::string_view problem = R"(
environment:
  min: [10, 20]
  max: [14, 23]
  obstacles:
    - {type: box, center: [11.5, 21.5], size: [0.5, 0.5]}
robots:
  - type: double_integrator_0
    start: [10.5, 20.5, 0, 0]
    goal: [13.5, 22.5, 0, 0]
    footprint:
      shape: circle
      radius: 0.25
)";

    TempDirectory temp;
    const auto input = temp.write("problem.yaml", problem);
    const auto output = temp.path("converted.map");

    RasterOptions options;
    options.cell = 1.0f;
    options.occ_threshold = 0.0f;
    options.supersample_px = 3;

    const auto result = continuousYamlToGrid(
        input, output, GridFormat::MovingAI, options, 0.25f);

    ASSERT_TRUE(result.ok) << result.message;
    EXPECT_EQ(result.output, output);
    ASSERT_TRUE(std::filesystem::is_regular_file(output));

    std::ifstream generated(output);
    const std::string contents{
        std::istreambuf_iterator<char>(generated),
        std::istreambuf_iterator<char>()};
    EXPECT_NE(contents.find("height 3"), std::string::npos);
    EXPECT_NE(contents.find("width 4"), std::string::npos);
    EXPECT_NE(contents.find('@'), std::string::npos);
}

TEST(MapConvert, ConvertsFromPreloadedProblemWithoutSourceFile) {
    static constexpr std::string_view problemYaml = R"(
environment:
  min: [0, 0]
  max: [4, 3]
  obstacles:
    - {type: box, center: [1.5, 1.5], size: [0.5, 0.5]}
robots:
  - type: double_integrator_0
    start: [0.5, 0.5, 0, 0]
    goal: [3.5, 2.5, 0, 0]
)";

    TempDirectory temp;
    const auto input = temp.write("problem.yaml", problemYaml);
    const auto problem = grace::sim::loadProblemYaml(input);
    ASSERT_TRUE(std::filesystem::remove(input));

    RasterOptions options;
    options.cell = 1.0f;
    const auto gridOutput = temp.path("converted.map");
    const auto gridResult = continuousProblemToGrid(
        problem,
        gridOutput,
        GridFormat::MovingAI,
        options,
        0.25f);
    ASSERT_TRUE(gridResult.ok) << gridResult.message;

    const auto scenarioOutput = temp.path("converted.scen");
    const auto scenarioResult = scenarioProblemToGridScen(
        problem, gridOutput, scenarioOutput, options.cell);
    ASSERT_TRUE(scenarioResult.ok) << scenarioResult.message;
    std::ifstream scenario(scenarioOutput);
    const std::string scenarioContents{
        std::istreambuf_iterator<char>(scenario),
        std::istreambuf_iterator<char>()};
    EXPECT_NE(
        scenarioContents.find("\t0\t0\t3\t2\t"),
        std::string::npos);
}

TEST(MapConvert, ReportsGridAndScenarioWriteFailures) {
    grace::sim::ProblemYaml problem;
    problem.max = {2.0f, 2.0f};
    grace::sim::RobotYamlSpec robot;
    robot.type = "double_integrator_0";
    robot.start = {0.5f, 0.5f};
    robot.goal = {1.5f, 1.5f};
    problem.robots.push_back(std::move(robot));

    TempDirectory temp;
    const auto unwritable = temp.path("existing-directory");
    ASSERT_TRUE(std::filesystem::create_directory(unwritable));

    RasterOptions options;
    options.cell = 1.0f;
    const auto gridResult = continuousProblemToGrid(
        problem,
        unwritable,
        GridFormat::MovingAI,
        options,
        0.25f);
    EXPECT_FALSE(gridResult.ok);
    EXPECT_NE(gridResult.message.find("Grid write failed"), std::string::npos);

    const auto scenarioResult = scenarioProblemToGridScen(
        problem, temp.path("map.map"), unwritable, options.cell);
    EXPECT_FALSE(scenarioResult.ok);
    EXPECT_NE(
        scenarioResult.message.find("Scenario write failed"),
        std::string::npos);
}

TEST(MapConvert, RejectsDegeneratePolygonWithoutThrowing) {
    static constexpr std::string_view problem = R"(
environment:
  min: [0, 0]
  max: [4, 3]
  obstacles:
    - type: polygon
      points: [[0, 0], [1, 1], [2, 2]]
robots:
  - type: double_integrator_0
    start: [0.5, 0.5, 0, 0]
    goal: [3.5, 2.5, 0, 0]
)";

    TempDirectory temp;
    const auto input = temp.write("degenerate.yaml", problem);
    const auto output = temp.path("converted.map");

    RasterOptions options;
    options.cell = 1.0f;
    const auto result = continuousYamlToGrid(
        input, output, GridFormat::MovingAI, options, 0.25f);

    EXPECT_FALSE(result.ok);
    EXPECT_NE(
        result.message.find("Invalid polygon obstacle"),
        std::string::npos);
    EXPECT_FALSE(std::filesystem::exists(output));
}
