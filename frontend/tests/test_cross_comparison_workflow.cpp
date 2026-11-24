// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "platform/CrossComparisonWorkflow.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

namespace {

class TestDirectory {
public:
    TestDirectory() {
        const auto suffix =
            std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::current_path() /
            ("cross-comparison-workflow-test-" + std::to_string(suffix));
        std::filesystem::create_directories(path_);
    }

    ~TestDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    [[nodiscard]] const std::filesystem::path& path() const {
        return path_;
    }

private:
    std::filesystem::path path_;
};

grace::sim::ProblemYaml makeProblem() {
    grace::sim::ProblemYaml problem;
    problem.min = {10.0f, 20.0f};
    problem.max = {14.0f, 24.0f};

    grace::sim::RobotYamlSpec robot;
    robot.type = "double_integrator_0";
    robot.start = {10.5f, 20.5f};
    robot.goal = {13.5f, 23.5f};
    grace::sim::RobotFootprintYaml footprint;
    footprint.kind = grace::sim::RobotFootprintKind::Circle;
    footprint.radius = 0.25f;
    robot.footprint = footprint;
    problem.robots.push_back(std::move(robot));
    return problem;
}

}  // namespace

TEST(CrossComparisonWorkflow, CalculatesAutomaticCellFromLargestFootprint) {
    auto problem = makeProblem();
    grace::sim::RobotYamlSpec boxRobot;
    grace::sim::RobotFootprintYaml footprint;
    footprint.kind = grace::sim::RobotFootprintKind::Box;
    footprint.size = {0.6f, 0.8f};
    boxRobot.footprint = footprint;
    problem.robots.push_back(std::move(boxRobot));

    EXPECT_FLOAT_EQ(
        platform::cross_compare::automaticRasterCell(problem, 0.75f),
        1.0f);

    problem.robots.clear();
    EXPECT_FLOAT_EQ(
        platform::cross_compare::automaticRasterCell(problem, 0.75f),
        0.75f);
}

TEST(CrossComparisonWorkflow, GeneratesArtifactsFromPreloadedProblem) {
    TestDirectory output;
    platform::cross_compare::Options options;
    options.outputDirectory = output.path() / "selected-output";
    options.raster.cell = -1.0f;
    options.roadmap.target_samples = 40;
    options.roadmap.time_sec = 0.1;
    options.roadmap.connect_k = 8;

    const auto result =
        platform::cross_compare::generateArtifacts(makeProblem(), options);

    ASSERT_TRUE(result.ok) << result.failureMessage;
    EXPECT_FLOAT_EQ(result.raster.cell, 0.5f);
    EXPECT_EQ(result.artifacts.outputDirectory, options.outputDirectory);
    EXPECT_TRUE(std::filesystem::is_regular_file(result.artifacts.roadmapXml));
    EXPECT_TRUE(std::filesystem::is_regular_file(result.artifacts.gridMap));
    EXPECT_TRUE(std::filesystem::is_regular_file(result.artifacts.roadmapTasksXml));
    EXPECT_TRUE(std::filesystem::is_regular_file(result.artifacts.gridScenario));

    std::ifstream scenario(result.artifacts.gridScenario);
    const std::string scenarioContents{
        std::istreambuf_iterator<char>(scenario),
        std::istreambuf_iterator<char>()};
    EXPECT_NE(scenarioContents.find("\t1\t1\t7\t7\t"), std::string::npos);
}

TEST(CrossComparisonWorkflow, PropagatesInvalidProblemFailure) {
    TestDirectory output;
    platform::cross_compare::Options options;
    options.outputDirectory = output.path();

    const auto result = platform::cross_compare::generateArtifacts({}, options);

    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.failureMessage.find("empty"), std::string::npos);
}

TEST(CrossComparisonWorkflow, RejectsInvalidResolvedRasterCell) {
    TestDirectory output;
    platform::cross_compare::Options options;
    options.outputDirectory = output.path();
    options.raster.cell = std::numeric_limits<float>::quiet_NaN();

    const auto result =
        platform::cross_compare::generateArtifacts(makeProblem(), options);

    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.failureMessage.find("finite and positive"), std::string::npos);
}

TEST(CrossComparisonWorkflow, ExportsComparisonCsv) {
    TestDirectory output;
    metrics::Aggregate continuous;
    continuous.agents = 3;
    continuous.plan_success = true;
    continuous.planner_runtime_s = 1.25;
    continuous.sim_runtime_s = 2.5;
    continuous.makespan_s = 4.0;
    continuous.sum_of_cost_s = 7.0;
    continuous.sum_length_m = 8.0;
    continuous.avg_length_m = 2.0;
    continuous.avg_completion_s = 3.0;

    const auto csv = output.path() / "comparison.csv";
    const auto result = platform::cross_compare::saveCsvComparison(
        csv, &continuous, nullptr, nullptr);

    ASSERT_TRUE(result.ok) << result.failureMessage;
    std::ifstream input(csv);
    const std::string contents{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    EXPECT_NE(contents.find("rep,agents,plan_success"), std::string::npos);
    EXPECT_NE(contents.find("continuous,3,1,1.25,2.5,4,7,8,2,3"), std::string::npos);
    EXPECT_EQ(contents.find("roadmap,"), std::string::npos);
}

TEST(CrossComparisonWorkflow, ReportsCsvOpenFailure) {
    TestDirectory output;

    const auto result = platform::cross_compare::saveCsvComparison(
        output.path(), nullptr, nullptr, nullptr);

    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.failureMessage.find("open"), std::string::npos);
}
