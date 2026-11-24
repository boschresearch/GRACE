// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/io/SimContinuousYamlOps.h"
#include "../../inc/util/Logger.h"
#include "../../inc/util/AlgoFootprints.h"
#include "../../inc/io/ContinuousYaml.h"
#include <fmt/format.h>
#include <optional>
#include <cmath>
#include <stdexcept>

namespace grace::sim::ops {

    using ::Logger;

    using grace::types::Footprint;
    using grace::types::FootprintKind;

    static std::optional<Footprint>
    toUnifiedFootprint(const std::optional<grace::sim::RobotFootprintYaml>& in)
    {
        if (!in) return std::nullopt;
        switch (in->kind) {
            case grace::sim::RobotFootprintKind::Circle:
                if (in->radius > 0.f) return Footprint::Circle(in->radius);
                return std::nullopt;
            case grace::sim::RobotFootprintKind::Box:
                if (in->size.x > 0.f && in->size.y > 0.f)
                    return Footprint::Box(in->size.x, in->size.y);
                return std::nullopt;
            case grace::sim::RobotFootprintKind::MultiBox: {
                if (in->boxes.empty()) return std::nullopt;
                std::vector<grace::types::BoxPart> parts;
                parts.reserve(in->boxes.size());
                for (const auto& b : in->boxes) {
                    parts.push_back(grace::types::BoxPart{
                        /*center*/ b.center,
                        /*sx*/     b.size.x,
                        /*sy*/     b.size.y,
                        /*angle*/  b.angle
                    });
                }
                return Footprint::MultiBox(std::move(parts));
            }
            default: return std::nullopt;
        }
    }

    static void setProblemPlannerMetadata(
        Simulation& sim,
        const grace::sim::ProblemYaml& spec)
    {
        PlannerSettings metadata;
        const std::string robots =
            grace::sim::emitRobotsMetaYaml(spec.robots);
        const std::string obstacles =
            grace::sim::emitObstaclesYamlLocal(spec, spec.min);
        if (!robots.empty()) {
            metadata.set("robot_meta_yaml", robots);
        }
        if (!obstacles.empty()) {
            metadata.set("obstacles_yaml", obstacles);
        }
        if (!metadata.kv.empty()) {
            sim.setPlannerRequestSettings(std::move(metadata));
        }
    }

    bool buildEnvironmentFromProblemSpec(
        ContinuousEnvironment& E,
        const grace::sim::ProblemYaml& spec,
        float wallThickness)
    {
        const float W = spec.max.x - spec.min.x;
        const float H = spec.max.y - spec.min.y;
        if (!(W > 0.0f && H > 0.0f)) {
            Logger::instance().error("ProblemYamlIO: environment bounds invalid (non-positive).");
            return false;
        }
        E.setBounds(W, H);
        E.addBoundaryWalls(wallThickness);

        auto toLocal = [&](b2Vec2 p) { return b2Vec2{ p.x - spec.min.x, p.y - spec.min.y }; };
        for (const auto& o : spec.obstacles) {
            switch (o.kind) {
                case grace::sim::ObstacleKind::Box: {
                    const b2Vec2 cL = toLocal(o.center);
                    const float  hx = 0.5f * o.size.x;
                    const float  hy = 0.5f * o.size.y;
                    E.addBoxBody(cL, hx, hy, 0, Type::BodyType::Obstacle);
                    break;
                }
                case grace::sim::ObstacleKind::Circle: {
                    const b2Vec2 cL = toLocal(o.center);
                    E.addCircleBody(cL, o.radius, Type::BodyType::Obstacle);
                    break;
                }
                case grace::sim::ObstacleKind::Polygon: {
                    std::vector<b2Vec2> ptsL; ptsL.reserve(o.points.size());
                    for (const auto& p : o.points) ptsL.push_back(toLocal(p));
                    try {
                        E.addConvexBody(
                            ptsL, Type::BodyType::Obstacle);
                    } catch (const std::invalid_argument& error) {
                        Logger::instance().error(fmt::format(
                            "ProblemYamlIO: invalid polygon obstacle: {}",
                            error.what()));
                        return false;
                    }
                    break;
                }
                case grace::sim::ObstacleKind::Capsule: {
                    E.addCapsuleBody(toLocal(o.p0), toLocal(o.p1), o.radius, Type::BodyType::Obstacle);
                    break;
                }
            }
        }
        Logger::instance().info("Environment built from problem.yaml (bounds, walls, obstacles).");
        return true;
    }

    bool buildEnvironmentFromProblemYaml(
        ContinuousEnvironment& E,
        const std::filesystem::path& yamlPath,
        float wallThickness)
    {
        return buildEnvironmentFromProblemSpec(
            E,
            grace::sim::loadProblemYaml(yamlPath),
            wallThickness);
    }

    std::vector<b2BodyId> spawnFromProblemYamlNoPlan(
        Simulation& sim,
        ContinuousEnvironment& E,
        const std::filesystem::path& yamlPath,
        std::size_t takeK,
        const std::string& baseName)
    {
        const grace::sim::ProblemYaml spec = grace::sim::loadProblemYaml(yamlPath);
        (void)E;
        return spawnFromProblemSpecNoPlan(
            sim, spec, takeK, baseName);
    }

    std::vector<b2BodyId> spawnFromProblemSpecNoPlan(
        Simulation& sim,
        const grace::sim::ProblemYaml& spec,
        std::size_t takeK,
        const std::string& baseName)
    {
        if (spec.robots.empty()) {
            Logger::instance().error("ProblemYamlIO: no robots found (or parse failed).");
            return {};
        }
        const std::size_t N = std::min<std::size_t>(takeK, spec.robots.size());
        auto toLocal = [&](b2Vec2 p) { return b2Vec2{ p.x - spec.min.x, p.y - spec.min.y }; };
        std::vector<b2BodyId> ids; ids.reserve(N);
        std::vector<b2Vec2> goals; goals.reserve(N);
        for (std::size_t i = 0; i < N; ++i) {
            const auto& r = spec.robots[i];
            const std::string name = fmt::format("{}{}", baseName, i + 1);
            const b2Vec2 sL = toLocal(r.start);
            const b2Vec2 gL = toLocal(r.goal);
            const auto fp   = toUnifiedFootprint(r.footprint);
            auto id = sim.spawnByAlgoType(r.type, sL, name, fp);
            if (B2_IS_NULL(id)) {
                Logger::instance().warn("Spawn failed for " + name + " (type='" + r.type + "'). Skipping.");
                continue;
            }
            ids.push_back(id);
            goals.push_back(gL);
        }
        if (ids.empty()) {
            Logger::instance().error("No robots were spawned from problem.yaml.");
            return {};
        }
        (void)sim.recordGoals(ids, goals); // record goals only
        setProblemPlannerMetadata(sim, spec);
        Logger::instance().info(fmt::format(
            "Spawned {} agent(s) from problem.yaml and recorded goals (no planning).", ids.size()));
        return ids;
    }

    bool applyGoalsFromProblemYaml(
        Simulation& sim,
        ContinuousEnvironment& E,
        const std::vector<b2BodyId>& ids,
        const std::filesystem::path& yamlPath)
    {
        const grace::sim::ProblemYaml spec = grace::sim::loadProblemYaml(yamlPath);
        (void)E;
        return applyGoalsFromProblemSpec(sim, ids, spec);
    }

    bool applyGoalsFromProblemSpec(
        Simulation& sim,
        const std::vector<b2BodyId>& ids,
        const grace::sim::ProblemYaml& spec)
    {
        if (ids.empty()) {
            Logger::instance().error("applyGoalsFromProblemYaml: ids empty.");
            return false;
        }
        if (spec.robots.empty()) {
            Logger::instance().error("ProblemYamlIO: no robots found (or parse failed).");
            return false;
        }
        auto toLocal = [&](b2Vec2 p) { return b2Vec2{ p.x - spec.min.x, p.y - spec.min.y }; };
        const std::size_t N = std::min<std::size_t>(ids.size(), spec.robots.size());
        std::vector<b2Vec2> goals; goals.reserve(N);
        for (std::size_t i = 0; i < N; ++i) {
            goals.push_back(toLocal(spec.robots[i].goal));
        }
        const bool ok = sim.recordGoals(ids, goals);
        if (ok && N < ids.size()) {
            Logger::instance().warn(fmt::format(
                "Goals recorded for {} agent(s) ({} present) due to limited YAML entries.", N, ids.size()));
        }

        setProblemPlannerMetadata(sim, spec);

        return ok;
    }

    bool spawnAndPlanFromProblemYaml(
        Simulation& sim,
        ContinuousEnvironment& E,
        const std::filesystem::path& yamlPath,
        float wallThickness,
        std::size_t takeK,
        const std::string& baseName)
    {
        const grace::sim::ProblemYaml spec = grace::sim::loadProblemYaml(yamlPath);
        return spawnAndPlanFromProblemSpec(
            sim, E, spec, wallThickness, takeK, baseName);
    }

    bool spawnAndPlanFromProblemSpec(
        Simulation& sim,
        ContinuousEnvironment& E,
        const grace::sim::ProblemYaml& spec,
        float wallThickness,
        std::size_t takeK,
        const std::string& baseName)
    {
        if (!buildEnvironmentFromProblemSpec(E, spec, wallThickness)) {
            return false;
        }
        const b2Vec2 mins = spec.min;
        auto toLocal = [&](b2Vec2 p) { return b2Vec2{ p.x - mins.x, p.y - mins.y }; };

        std::vector<b2BodyId> ids;
        std::vector<StartGoal> sgs;
        ids.reserve(spec.robots.size());
        sgs.reserve(spec.robots.size());

        const std::size_t N = std::min<std::size_t>(takeK, spec.robots.size());
        for (std::size_t i = 0; i < N; ++i) {
            const auto& r = spec.robots[i];
            const std::string name = fmt::format("{}{}", baseName, i + 1);
            const b2Vec2 sL = toLocal(r.start);
            const b2Vec2 gL = toLocal(r.goal);
            const auto fp = toUnifiedFootprint(r.footprint);
            auto id = sim.spawnByAlgoType(r.type, sL, name, fp);
            if (B2_IS_NULL(id)) {
                Logger::instance().warn("Spawn failed for " + name + " (type='" + r.type + "'). Skipping.");
                continue;
            }
            ids.push_back(id);
            sgs.push_back(StartGoal{ sL, gL });
        }
        if (ids.empty()) {
            Logger::instance().error("No robots were spawned from problem.yaml.");
            return false;
        }

        setProblemPlannerMetadata(sim, spec);

        return sim.plan(ids, sgs);
    }

} // namespace grace::io::ops
