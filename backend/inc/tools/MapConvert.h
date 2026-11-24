// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>

#include "io/ContinuousYaml.h"

namespace tools::mc {

enum class GridFormat { MovingAI, Xml };

struct PRMOptions {
    int    target_samples = 2000;   // target vertex count for PRM*
    double time_sec       = 5.0;    // time budget
    float  robot_radius   = 0.25f;  // local planner clearance (for edge validation)
    bool   pin_starts_goals = true; // insert starts/goals as pinned vertices in V_r
    int    connect_k        = 12;   // k-NN for pin connections
    float  connect_radius   = 2.0f; // max radius for pin connections (meters)
};

struct RasterOptions {
    // Paper: h = 2 * max_i r_eff(S_i) unless caller forces a value.
    // If cell <= 0, we compute it automatically from YAML robots (recommended).
    float cell             = -1.0f; // <= 0 => auto from robots
    float inset            = 0.0f;  // optional inset on AABBs during coverage tests
    // Coverage model (polygon coverage tests). If occ_threshold <= 0, we use "any overlap".
    // If in (0,1], we require at least that fraction of cell area to be covered by obstacles.
    float occ_threshold    = 0.0f;  // 0 = any-overlap, else coverage fraction
    int   supersample_px   = 9;     // N x N quadsampling per cell (fallback for mixed shapes)

    bool  use_shape_overlap = true; // true => accurate shape-vs-rect; false => legacy AABB
    bool  clamp_edge_cells  = true; // clamp last row/col to [0,W]x[0,H] to remove overhang artifacts
};

struct Result {
    bool ok = false;
    std::filesystem::path output;
    std::string message;  // short reason on failure
};

// ------------- Conversions from continuous (problem.yaml) -------------
Result continuousYamlToRoadmap(
    const std::filesystem::path& yamlIn,
    const std::filesystem::path& graphOut,
    const PRMOptions& prm = {},
    float wall_thickness = 0.25f);

Result continuousProblemToRoadmap(
    const grace::sim::ProblemYaml& problem,
    const std::filesystem::path& graphOut,
    const PRMOptions& prm = {},
    float wall_thickness = 0.25f);

Result continuousYamlToGrid(
    const std::filesystem::path& yamlIn,
    const std::filesystem::path& gridOut,
    GridFormat fmt = GridFormat::MovingAI,
    const RasterOptions& raster = {},
    float wall_thickness = 0.25f);

Result continuousProblemToGrid(
    const grace::sim::ProblemYaml& problem,
    const std::filesystem::path& gridOut,
    GridFormat fmt = GridFormat::MovingAI,
    const RasterOptions& raster = {},
    float wall_thickness = 0.25f);

// ------------------------- Conversions from grid ----------------------
Result gridToContinuousYaml(
    const std::filesystem::path& gridIn,
    GridFormat fmt,
    const std::filesystem::path& yamlOut,
    float cell_length);

Result gridToRoadmap(
    const std::filesystem::path& gridIn,
    GridFormat fmt,
    const std::filesystem::path& graphOut,
    const PRMOptions& prm = {},
    float cell_length = 1.0f);

// ------------------------ Conversions from roadmap --------------------
Result roadmapToContinuousYaml(
    const std::filesystem::path& graphIn,
    const std::filesystem::path& yamlOut,
    // optional: a problem.yaml with obstacles to embed and/or to build geometry
    const std::filesystem::path& optionalObstaclesYaml = {},
    float wall_thickness = 0.25f);

Result roadmapToGrid(
    const std::filesystem::path& graphIn,
    const std::filesystem::path& gridOut,
    GridFormat fmt = GridFormat::MovingAI,
    // optional: a problem.yaml providing geometry to rasterize against
    const std::filesystem::path& optionalObstaclesYaml = {},
    const RasterOptions& raster = {},
    float wall_thickness = 0.25f);

// === Scenario conversions =================================================

// problem.yaml (robots) -> MovingAI .scen (grid)
Result scenarioYamlToGridScen(
    const std::filesystem::path& yamlIn,
    const std::filesystem::path& gridMapPath,  // used as the 'map' field written into .scen
    const std::filesystem::path& scenOut,
    float cell_length);

Result scenarioProblemToGridScen(
    const grace::sim::ProblemYaml& problem,
    const std::filesystem::path& gridMapPath,
    const std::filesystem::path& scenOut,
    float cell_length);

// MovingAI .scen (grid) -> problem.yaml (robots)
// 'gridMapIn' is the map file referenced by the .scen (to get dims and scale)
Result scenarioGridScenToYaml(
    const std::filesystem::path& gridMapIn,
    GridFormat fmt,
    const std::filesystem::path& scenIn,
    const std::filesystem::path& yamlOut,
    float cell_length,
    const std::string& default_type = "unicycle_first_order_0");

// problem.yaml (robots) -> tasks.xml (roadmap)
// Map each (start,goal) to nearest roadmap nodes by Euclidean distance
Result scenarioYamlToTasksXml(
    const std::filesystem::path& yamlIn,
    const std::filesystem::path& graphXmlIn,
    const std::filesystem::path& tasksXmlOut);

Result scenarioProblemToTasksXml(
    const grace::sim::ProblemYaml& problem,
    const std::filesystem::path& graphXmlIn,
    const std::filesystem::path& tasksXmlOut);

// tasks.xml (roadmap) -> problem.yaml (robots)
// Rehydrate starts/goals as node positions
Result scenarioTasksXmlToYaml(
    const std::filesystem::path& graphXmlIn,
    const std::filesystem::path& tasksXmlIn,
    const std::filesystem::path& yamlOut,
    const std::string& default_type = "unicycle_first_order_0");

// MovingAI .scen (grid) -> tasks.xml (roadmap)
// Convert each grid cell center to nearest roadmap node
Result scenarioGridScenToTasksXml(
    const std::filesystem::path& gridMapIn,
    GridFormat fmt,
    float cell_length,
    const std::filesystem::path& scenIn,
    const std::filesystem::path& graphXmlIn,
    const std::filesystem::path& tasksXmlOut);

// tasks.xml (roadmap) -> MovingAI .scen (grid)
// Convert each roadmap node to grid cell coords
Result scenarioTasksXmlToGridScen(
    const std::filesystem::path& graphXmlIn,
    const std::filesystem::path& tasksXmlIn,
    const std::filesystem::path& gridMapPath, // used as the 'map' field in .scen
    float cell_length,
    const std::filesystem::path& scenOut);

} // namespace tools::mc
