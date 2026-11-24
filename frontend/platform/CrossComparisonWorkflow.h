// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "io/ContinuousYaml.h"
#include "tools/MapConvert.h"
#include "util/Metrics.h"

#include <filesystem>
#include <string>

namespace platform::cross_compare {

struct ArtifactPaths {
    std::filesystem::path outputDirectory;
    std::filesystem::path roadmapXml;
    std::filesystem::path gridMap;
    std::filesystem::path roadmapTasksXml;
    std::filesystem::path gridScenario;
};

struct Options {
    std::filesystem::path outputDirectory;
    tools::mc::PRMOptions roadmap;
    tools::mc::RasterOptions raster;
    tools::mc::GridFormat gridFormat{tools::mc::GridFormat::MovingAI};
    float wallThickness{0.25f};
    float automaticCellFallback{0.5f};
};

struct Result {
    bool ok{false};
    ArtifactPaths artifacts;
    tools::mc::RasterOptions raster;
    std::string failureMessage;
};

struct CsvExportResult {
    bool ok{false};
    std::string failureMessage;
};

[[nodiscard]] ArtifactPaths buildArtifactPaths(
    const std::filesystem::path& outputDirectory,
    tools::mc::GridFormat gridFormat);

[[nodiscard]] float automaticRasterCell(
    const grace::sim::ProblemYaml& problem,
    float fallbackCell);

[[nodiscard]] Result generateArtifacts(
    const grace::sim::ProblemYaml& problem,
    const Options& options);

[[nodiscard]] CsvExportResult saveCsvComparison(
    const std::filesystem::path& outputCsv,
    const metrics::Aggregate* continuous,
    const metrics::Aggregate* roadmap,
    const metrics::Aggregate* grid);

}  // namespace platform::cross_compare
