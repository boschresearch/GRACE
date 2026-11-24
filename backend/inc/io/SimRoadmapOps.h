// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <vector>
#include <string>

#include "../inc/Simulation.h"
#include "../inc/env/RoadmapEnvironment.h"
#include "../inc/io/RoadmapXML.h"

namespace grace::sim::ops {

    // ---------- Roadmap convenience ----------

    /// Spawn robots at given roadmap node IDs (all same radius).
    std::vector<b2BodyId> spawnAtNodes(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::vector<IRoadmap::NodeId>& nodes,
        float radius,
        std::size_t takeK = std::numeric_limits<std::size_t>::max(),
        const std::string& baseName = "r");

    /// Plan between roadmap node IDs (sg pairs).
    bool planFromTaskXml(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::vector<b2BodyId>& ids,
        const std::filesystem::path& taskXml,
        const grace::sim::GraphMLImport& import);

    /// Record goals (no planning) from task.xml mapping to node IDs.
    bool assignGoalsFromTaskXml(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::vector<b2BodyId>& ids,
        const std::filesystem::path& taskXml,
        const grace::sim::GraphMLImport& import);

    /// Spawn from task starts, record goals, no planning.
    std::vector<b2BodyId> spawnFromTaskXmlNoPlan(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::filesystem::path& taskXml,
        const grace::sim::GraphMLImport& import,
        float robotRadius,
        std::size_t takeK = std::numeric_limits<std::size_t>::max(),
        const std::string& baseName = "r");

    /// Parse task.xml, spawn, plan.
    bool spawnAndPlanFromTaskXml(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::filesystem::path& taskXml,
        const grace::sim::GraphMLImport& import,
        float robotRadius,
        std::size_t takeK = std::numeric_limits<std::size_t>::max(),
        const std::string& baseName = "r");

} // namespace grace::sim::ops
