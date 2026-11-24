// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <vector>
#include <string>

#include "../inc/Simulation.h"
#include "../inc/env/ContinuousEnvironment.h"
#include "../inc/io/ContinuousYaml.h"

namespace grace::sim::ops {

    /// Build only the environment from an already parsed immutable problem.
    bool buildEnvironmentFromProblemSpec(
        ContinuousEnvironment& E,
        const ProblemYaml& spec,
        float wallThickness = 0.25f);

    /// Build only the environment (bounds + obstacles + boundary walls). No agents, no planning.
    bool buildEnvironmentFromProblemYaml(
        ContinuousEnvironment& E,
        const std::filesystem::path& yamlPath,
        float wallThickness = 0.25f);

    /// Spawn agents from an already parsed immutable problem (no planning).
    std::vector<b2BodyId> spawnFromProblemSpecNoPlan(
        Simulation& sim,
        const ProblemYaml& spec,
        std::size_t takeK = std::numeric_limits<std::size_t>::max(),
        const std::string& baseName = "r");

    /// Spawn agents from YAML starts and record goals from YAML (no planning).
    std::vector<b2BodyId> spawnFromProblemYamlNoPlan(
        Simulation& sim,
        ContinuousEnvironment& E,
        const std::filesystem::path& yamlPath,
        std::size_t takeK = std::numeric_limits<std::size_t>::max(),
        const std::string& baseName = "r");

    /// Apply goals and planner metadata from an already parsed immutable problem.
    bool applyGoalsFromProblemSpec(
        Simulation& sim,
        const std::vector<b2BodyId>& ids,
        const ProblemYaml& spec);

    /// Only apply goals from YAML to already-spawned agents (no planning).
    bool applyGoalsFromProblemYaml(
        Simulation& sim,
        ContinuousEnvironment& E,
        const std::vector<b2BodyId>& ids,
        const std::filesystem::path& yamlPath);

    /// Build, spawn, and plan from an already parsed immutable problem.
    bool spawnAndPlanFromProblemSpec(
        Simulation& sim,
        ContinuousEnvironment& E,
        const ProblemYaml& spec,
        float wallThickness = 0.25f,
        std::size_t takeK = std::numeric_limits<std::size_t>::max(),
        const std::string& baseName = "r");

    /// Parse a DBCBS-like problem.yaml, build continuous env, spawn, plan.
    bool spawnAndPlanFromProblemYaml(
        Simulation& sim,
        ContinuousEnvironment& E,
        const std::filesystem::path& yamlPath,
        float wallThickness = 0.25f,
        std::size_t takeK   = std::numeric_limits<std::size_t>::max(),
        const std::string& baseName = "r");

} // namespace grace::sim::ops
