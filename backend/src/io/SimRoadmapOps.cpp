// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/io/RoadmapXML.h"
#include "../../inc/util/Logger.h"
#include <fmt/format.h>
#include <algorithm>

#include "../../inc/Simulation.h"

namespace grace::sim::ops {

    using ::Logger;

    std::vector<b2BodyId> spawnAtNodes(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::vector<IRoadmap::NodeId>& nodes,
        float radius,
        std::size_t takeK,
        const std::string& baseName)
    {
        const std::size_t N = std::min<std::size_t>(takeK, nodes.size());
        std::vector<b2BodyId> ids; ids.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            const std::string name = fmt::format("{}{}", baseName, i + 1);
            auto id = sim.spawnRoadmapNodeRobot(R, nodes[i], radius, name);
            if (B2_IS_NULL(id)) {
                Logger::instance().warn(fmt::format("Failed to spawn robot {} at node {}.", name, nodes[i]));
                continue;
            }
            ids.push_back(id);
        }
        Logger::instance().info("All roadmap robots spawned.");
        return ids;
    }

    bool planFromTaskXml(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::vector<b2BodyId>& ids,
        const std::filesystem::path& taskXml,
        const grace::sim::GraphMLImport& import)
    {
        auto sgsNodes = grace::sim::readTasksXml(taskXml, import);
        if (sgsNodes.size() != ids.size()) {
            Logger::instance().error("TaskXmlIO: tasks count (" + std::to_string(sgsNodes.size()) +
                                     ") != agents (" + std::to_string(ids.size()) + ")");
            return false;
        }
        std::vector<StartGoal> sgs; sgs.reserve(sgsNodes.size());
        for (const auto& [s, g] : sgsNodes) {
            sgs.push_back({ R.nodePos(s), R.nodePos(g) });
        }
        return sim.plan(ids, sgs);
    }

    bool assignGoalsFromTaskXml(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::vector<b2BodyId>& ids,
        const std::filesystem::path& taskXml,
        const grace::sim::GraphMLImport& import)
    {
        if (ids.empty()) {
            Logger::instance().error("assignGoalsFromTaskXml: ids empty.");
            return false;
        }
        auto tasks = grace::sim::readTasksXml(taskXml, import);
        if (tasks.empty()) {
            Logger::instance().error("TaskXmlIO: no tasks found in " + taskXml.string());
            return false;
        }
        const std::size_t N = std::min(ids.size(), tasks.size());
        std::vector<b2Vec2> goals; goals.reserve(N);
        for (std::size_t i = 0; i < N; ++i) {
            goals.push_back(R.nodePos(tasks[i].second)); // goals only
        }
        if (tasks.size() < ids.size()) {
            Logger::instance().warn(fmt::format(
                "Goals recorded for {} agent(s) ({} present) due to limited tasks.",
                N, ids.size()));
        }
        return sim.recordGoals(ids, goals);
    }

    std::vector<b2BodyId> spawnFromTaskXmlNoPlan(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::filesystem::path& taskXml,
        const grace::sim::GraphMLImport& import,
        float robotRadius,
        std::size_t takeK,
        const std::string& baseName)
    {
        auto tasks = grace::sim::readTasksXml(taskXml, import);
        if (tasks.empty()) {
            Logger::instance().error("TaskXmlIO: no tasks found in " + taskXml.string());
            return {};
        }
        if (takeK < tasks.size()) tasks.resize(takeK);
        std::vector<IRoadmap::NodeId> starts; starts.reserve(tasks.size());
        for (const auto& p : tasks) starts.push_back(p.first);
        auto ids = spawnAtNodes(sim, R, starts, robotRadius, /*takeK=*/starts.size(), baseName);
        if (ids.empty()) return {};
        const std::size_t N = std::min(ids.size(), tasks.size());
        std::vector<b2Vec2> goals; goals.reserve(N);
        for (std::size_t i = 0; i < N; ++i) goals.push_back(R.nodePos(tasks[i].second));
        (void)sim.recordGoals(ids, goals);
        Logger::instance().info(fmt::format(
            "Spawned {} agent(s) from tasks.xml and recorded goals (no planning).", ids.size()));
        return ids;
    }

    bool spawnAndPlanFromTaskXml(
        Simulation& sim,
        RoadmapEnvironment& R,
        const std::filesystem::path& taskXml,
        const grace::sim::GraphMLImport& import,
        float robotRadius,
        std::size_t takeK,
        const std::string& baseName)
    {
        auto tasks = grace::sim::readTasksXml(taskXml, import);
        if (tasks.empty()) {
            Logger::instance().error("TaskXmlIO: no tasks found in " + taskXml.string());
            return false;
        }
        if (takeK < tasks.size()) tasks.resize(takeK);
        std::vector<IRoadmap::NodeId> starts;
        starts.reserve(tasks.size());
        for (const auto& p : tasks) starts.push_back(p.first);
        auto ids = spawnAtNodes(sim, R, starts, robotRadius, /*takeK=*/starts.size(), baseName);
        if (ids.empty()) {
            Logger::instance().error("No robots were spawned for tasks.");
            return false;
        }
        if (tasks.size() != ids.size()) {
            Logger::instance().warn(fmt::format(
                "Tasks/agents mismatch after spawn (tasks={}, agents={}) — trimming tasks.",
                tasks.size(), ids.size()));
            tasks.resize(ids.size());
        }
        std::vector<StartGoal> sgs; sgs.reserve(tasks.size());
        for (const auto& [s, g] : tasks) {
            sgs.push_back({ R.nodePos(s), R.nodePos(g) });
        }
        return sim.plan(ids, sgs);
    }

} // namespace grace::io::ops
