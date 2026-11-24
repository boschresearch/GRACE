// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "CrossComparisonWorkflow.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <system_error>
#include <utility>

namespace platform::cross_compare {
namespace {

float effectiveRadius(const grace::sim::RobotYamlSpec& robot) {
    if (!robot.footprint) {
        return 0.0f;
    }
    if (robot.footprint->kind == grace::sim::RobotFootprintKind::Circle) {
        return robot.footprint->radius;
    }
    if (robot.footprint->kind == grace::sim::RobotFootprintKind::Box) {
        const float width = robot.footprint->size.x;
        const float height = robot.footprint->size.y;
        return 0.5f * std::sqrt(width * width + height * height);
    }
    return std::max(0.0f, robot.footprint->radius);
}

Result failure(Result result, std::string message) {
    result.failureMessage = std::move(message);
    return result;
}

}  // namespace

ArtifactPaths buildArtifactPaths(
    const std::filesystem::path& outputDirectory,
    const tools::mc::GridFormat gridFormat) {
    return {
        outputDirectory,
        outputDirectory / "roadmap.xml",
        outputDirectory /
            (gridFormat == tools::mc::GridFormat::MovingAI ? "grid.map" : "grid.xml"),
        outputDirectory / "tasks.xml",
        outputDirectory / "instance.scen",
    };
}

float automaticRasterCell(
    const grace::sim::ProblemYaml& problem,
    const float fallbackCell) {
    float maximumRadius = 0.0f;
    for (const auto& robot : problem.robots) {
        maximumRadius = std::max(maximumRadius, effectiveRadius(robot));
    }
    return maximumRadius > 0.0f ? 2.0f * maximumRadius : fallbackCell;
}

Result generateArtifacts(
    const grace::sim::ProblemYaml& problem,
    const Options& options) {
    Result result;
    result.artifacts = buildArtifactPaths(options.outputDirectory, options.gridFormat);
    result.raster = options.raster;

    if (problem.obstacles.empty() && problem.robots.empty()) {
        return failure(std::move(result), "Problem YAML is empty or could not be parsed.");
    }
    if (result.artifacts.outputDirectory.empty()) {
        return failure(std::move(result), "Output directory is empty.");
    }
    if (result.raster.cell <= 0.0f) {
        result.raster.cell =
            automaticRasterCell(problem, options.automaticCellFallback);
    }
    if (!std::isfinite(result.raster.cell) ||
        result.raster.cell <= 0.0f) {
        return failure(
            std::move(result),
            "Raster cell length must be finite and positive.");
    }

    std::error_code directoryError;
    std::filesystem::create_directories(
        result.artifacts.outputDirectory, directoryError);
    if (directoryError) {
        return failure(
            std::move(result),
            "Could not create output directory: " + directoryError.message());
    }

    try {
        if (auto conversion = tools::mc::continuousProblemToRoadmap(
                problem,
                result.artifacts.roadmapXml,
                options.roadmap,
                options.wallThickness);
            !conversion.ok) {
            return failure(
                std::move(result),
                "Conversion to roadmap failed: " + conversion.message);
        }
        if (auto conversion = tools::mc::continuousProblemToGrid(
                problem,
                result.artifacts.gridMap,
                options.gridFormat,
                result.raster,
                options.wallThickness);
            !conversion.ok) {
            return failure(
                std::move(result),
                "Conversion to grid failed: " + conversion.message);
        }
        if (auto conversion = tools::mc::scenarioProblemToTasksXml(
                problem,
                result.artifacts.roadmapXml,
                result.artifacts.roadmapTasksXml);
            !conversion.ok) {
            return failure(
                std::move(result),
                "scenarioProblemToTasksXml failed: " + conversion.message);
        }
        if (auto conversion = tools::mc::scenarioProblemToGridScen(
                problem,
                result.artifacts.gridMap,
                result.artifacts.gridScenario,
                result.raster.cell);
            !conversion.ok) {
            return failure(
                std::move(result),
                "scenarioProblemToGridScen failed: " + conversion.message);
        }
    } catch (const std::exception& error) {
        return failure(
            std::move(result),
            "Artifact generation failed: " + std::string(error.what()));
    }

    result.ok = true;
    return result;
}

CsvExportResult saveCsvComparison(
    const std::filesystem::path& outputCsv,
    const metrics::Aggregate* continuous,
    const metrics::Aggregate* roadmap,
    const metrics::Aggregate* grid) {
    std::ofstream output(outputCsv);
    if (!output) {
        return {
            false,
            "Could not open comparison CSV for writing: " + outputCsv.string()};
    }

    output << "rep,agents,plan_success,planner_runtime_s,sim_runtime_s,"
              "makespan_s,sum_of_cost_s,sum_length_m,avg_length_m,"
              "avg_completion_s\n";
    const auto writeAggregate =
        [&output](const char* representation, const metrics::Aggregate* aggregate) {
            if (!aggregate) {
                return;
            }
            const auto finiteOrMissing = [](const double value) {
                return std::isfinite(value) ? value : -1.0;
            };
            output
                << representation << ','
                << aggregate->agents << ','
                << (aggregate->plan_success ? 1 : 0) << ','
                << finiteOrMissing(aggregate->planner_runtime_s) << ','
                << finiteOrMissing(aggregate->sim_runtime_s) << ','
                << finiteOrMissing(aggregate->makespan_s) << ','
                << finiteOrMissing(aggregate->sum_of_cost_s) << ','
                << finiteOrMissing(aggregate->sum_length_m) << ','
                << finiteOrMissing(aggregate->avg_length_m) << ','
                << finiteOrMissing(aggregate->avg_completion_s) << '\n';
        };
    writeAggregate("continuous", continuous);
    writeAggregate("roadmap", roadmap);
    writeAggregate("grid", grid);
    output.flush();
    if (!output) {
        return {
            false,
            "Could not write comparison CSV: " + outputCsv.string()};
    }
    return {true, {}};
}

}  // namespace platform::cross_compare
