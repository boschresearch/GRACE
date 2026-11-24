// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "MapConvertInternal.h"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace tools::mc::internal {

std::optional<std::string> buildFreeEnvFromProblem(
    const grace::sim::ProblemYaml& spec,
    ContinuousEnvironment& environment,
    float wallThickness)
{
    const b2Vec2 mins = spec.min;
    const float width = spec.max.x - spec.min.x;
    const float height = spec.max.y - spec.min.y;

    environment.resetEnvironment();
    environment.setBounds(width, height);
    if (wallThickness > 0.0f) {
        environment.addBoundaryWalls(wallThickness);
    }

    const auto toLocal = [&](b2Vec2 point) {
        return b2Vec2{point.x - mins.x, point.y - mins.y};
    };

    for (const auto& obstacle : spec.obstacles) {
        switch (obstacle.kind) {
            case grace::sim::ObstacleKind::Box:
                environment.addBoxBody(
                    toLocal(obstacle.center),
                    0.5f * obstacle.size.x,
                    0.5f * obstacle.size.y,
                    0,
                    Type::BodyType::Obstacle);
                break;
            case grace::sim::ObstacleKind::Circle:
                environment.addCircleBody(
                    toLocal(obstacle.center),
                    obstacle.radius,
                    Type::BodyType::Obstacle);
                break;
            case grace::sim::ObstacleKind::Polygon: {
                std::vector<b2Vec2> points;
                points.reserve(obstacle.points.size());
                for (const auto point : obstacle.points) {
                    points.push_back(toLocal(point));
                }
                try {
                    environment.addConvexBody(
                        points, Type::BodyType::Obstacle);
                } catch (const std::invalid_argument& error) {
                    return "Invalid polygon obstacle: " +
                           std::string(error.what());
                }
                break;
            }
            case grace::sim::ObstacleKind::Capsule:
                environment.addCapsuleBody(
                    toLocal(obstacle.p0),
                    toLocal(obstacle.p1),
                    obstacle.radius,
                    Type::BodyType::Obstacle);
                break;
        }
    }
    return std::nullopt;
}

namespace {

struct EnvValidityChecker final : ob::StateValidityChecker {
    ContinuousEnvironment* environment{};
    float robotRadius{};

    EnvValidityChecker(
        const ob::SpaceInformationPtr& spaceInformation,
        ContinuousEnvironment* env,
        float radius)
        : ob::StateValidityChecker(spaceInformation),
          environment(env),
          robotRadius(radius)
    {
    }

    bool isValid(const ob::State* state) const override
    {
        const auto* vectorState =
            state->as<ob::RealVectorStateSpace::StateType>();
        const float x = static_cast<float>((*vectorState)[0]);
        const float y = static_cast<float>((*vectorState)[1]);
        b2AABB bounds{};
        bounds.lowerBound = {x - robotRadius, y - robotRadius};
        bounds.upperBound = {x + robotRadius, y + robotRadius};
        return environment->isFreeShape(bounds);
    }
};

struct EnvMotionValidator final : ob::MotionValidator {
    ContinuousEnvironment* environment{};

    EnvMotionValidator(
        const ob::SpaceInformationPtr& spaceInformation,
        ContinuousEnvironment* env)
        : ob::MotionValidator(spaceInformation), environment(env)
    {
    }

    bool checkMotion(
        const ob::State* fromState, const ob::State* toState) const override
    {
        const auto* from =
            fromState->as<ob::RealVectorStateSpace::StateType>();
        const auto* to =
            toState->as<ob::RealVectorStateSpace::StateType>();
        const b2Vec2 start{
            static_cast<float>((*from)[0]),
            static_cast<float>((*from)[1])};
        const b2Vec2 goal{
            static_cast<float>((*to)[0]),
            static_cast<float>((*to)[1])};
        return !environment->raycast(start, goal);
    }

    bool checkMotion(
        const ob::State* fromState,
        const ob::State* toState,
        std::pair<ob::State*, double>&) const override
    {
        return checkMotion(fromState, toState);
    }
};

} // namespace

bool buildPRMStarRoadmap(
    ContinuousEnvironment& environment,
    int samples,
    double timeLimitSec,
    float robotRadius,
    RoadmapEnvironment& roadmap)
{
    const auto [width, height] = environment.boundsInMeter();
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, 0.0);
    bounds.setLow(1, 0.0);
    bounds.setHigh(0, width);
    bounds.setHigh(1, height);
    space->setBounds(bounds);
    og::SimpleSetup setup(space);

    environment.setRobotRadius(robotRadius);
    auto spaceInformation = setup.getSpaceInformation();
    spaceInformation->setStateValidityChecker(
        std::make_shared<EnvValidityChecker>(
            spaceInformation, &environment, robotRadius));
    spaceInformation->setMotionValidator(
        std::make_shared<EnvMotionValidator>(
            spaceInformation, &environment));
    spaceInformation->setup();

    const auto sampleValid = [&](ob::ScopedState<>& state) {
        for (int attempt = 0; attempt < 200; ++attempt) {
            state.random();
            if (spaceInformation->isValid(state.get())) {
                return true;
            }
        }
        return false;
    };
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    if (!sampleValid(start) || !sampleValid(goal)) {
        return false;
    }

    setup.setStartAndGoalStates(start, goal, robotRadius * 0.5);
    auto prm = std::make_shared<og::PRM>(spaceInformation);
    prm->setMaxNearestNeighbors(3);
    setup.setPlanner(prm);

    ob::PlannerData data(spaceInformation);
    const auto startTime = std::chrono::steady_clock::now();
    while (true) {
        setup.solve(0.1);
        setup.getPlanner()->getPlannerData(data);
        if (static_cast<int>(data.numVertices()) >= samples) {
            break;
        }
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsed >= timeLimitSec) {
            break;
        }
    }

    roadmap.resetEnvironment();
    roadmap.setBounds(width, height);

    const std::size_t count = data.numVertices();
    std::vector<IRoadmap::NodeId> ids(count, -1);
    for (std::size_t index = 0; index < count; ++index) {
        const auto* state = data.getVertex(index)
                                .getState()
                                ->as<ob::RealVectorStateSpace::StateType>();
        ids[index] = roadmap.addNode({
            static_cast<float>((*state)[0]),
            static_cast<float>((*state)[1])});
    }
    for (std::size_t index = 0; index < count; ++index) {
        std::vector<unsigned int> edges;
        data.getEdges(index, edges);
        for (const unsigned int neighbor : edges) {
            if (index != neighbor) {
                roadmap.addEdge(ids[index], ids[neighbor]);
            }
        }
    }
    return true;
}

} // namespace tools::mc::internal
