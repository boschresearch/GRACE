// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>
#include <string>
#include <filesystem>
#include <box2d/box2d.h>
#include <box2d/math_functions.h>

#include "ICapabilities.h"
#include "../util/Types.h"

enum class EnvironmentType { GridEnvironment, ContinuousEnvironment, RoadmapEnvironment, UnknownEnvironment };

struct Bounds {
    float width{};
    float height{};
};

struct LocalBox {
    b2Vec2 localCenter{0.0f, 0.0f}; // local (body-frame) center
    float  halfW{0.0f};
    float  halfH{0.0f};
    float  theta{0.0f};        // local rotation (radians)
};

class BaseEnvironment {
public:
    BaseEnvironment(){ resetWorldState(); }
    virtual ~BaseEnvironment(){ destroyWorld(); }

    // environment - specific functions
    [[nodiscard]] virtual Bounds boundsInMeter() const = 0;
    [[nodiscard]] virtual EnvironmentType getEnvironmentType() const=0;
    [[nodiscard]] virtual const IDiscreteGrid* gridCapability() const noexcept { return nullptr; }
    [[nodiscard]] virtual const IRoadmap* roadmapCapability() const noexcept { return nullptr; }
    [[nodiscard]] virtual const IContinuousSpace* continuousCapability() const noexcept { return nullptr; }

    // common functions

    // (1) world management helpers
    // Clears both common Box2D state and environment-specific metadata.
    void resetEnvironment();
    void clearObstacles();    // destroy only obstacle bodies
    void clearBoundingWalls();        // destroy only wall bodies
    void clearRobots();       // destroy only robot bodies
    void clearAllBodies();    // destroy all bodies (obstacles, walls, robots)
    void addBoundaryWalls(float thickness);  // thickness in meters
    [[maybe_unused]] b2BodyId addBoxBody    (b2Vec2 bodyCenter, float halfW, float halfH, float theta, Type::BodyType bt);
    [[maybe_unused]] b2BodyId addCompoundBoxesBody(b2Vec2 bodyCenter, const std::vector<LocalBox>& boxes, Type::BodyType bt);
    [[maybe_unused]] b2BodyId addCircleBody (b2Vec2 bodyCenter, float radius, Type::BodyType bt);
    // Throws std::invalid_argument before mutating the world if no valid hull
    // can be formed from the supplied points.
    [[maybe_unused]] b2BodyId addConvexBody (const std::vector<b2Vec2>& ptsWorld, Type::BodyType bt, float polyRadius = 0.0f);
    [[maybe_unused]] b2BodyId addCapsuleBody(b2Vec2 p0, b2Vec2 p1, float radius, Type::BodyType bt);
    [[maybe_unused]] b2BodyId addCapsuleBody(b2Vec2 bodyCenter, float halfLen, float radius, float theta, Type::BodyType bt);
    [[nodiscard]] b2WorldId getB2World() const noexcept { return m_world; }
    [[nodiscard]] const std::vector<b2BodyId>& obstacles() const noexcept { return m_obstacleBodies; }
    [[nodiscard]] const std::vector<b2BodyId>& boundaryWalls()     const noexcept { return m_wallBodies; }
    [[nodiscard]] const std::vector<b2BodyId>& robots()    const noexcept { return m_robotBodies; }

    // (2) statistics
    [[nodiscard]] float environmentWidthInMeter()  const noexcept { return boundsInMeter().width; }
    [[nodiscard]] float environmentHeightInMeter() const noexcept { return boundsInMeter().height; }
    [[nodiscard]] float environmentAreaInMeter2()  const noexcept {
        auto [w,h] = boundsInMeter(); return w * h;
    }
    [[nodiscard]] float obstacleAreaInMeter2() const noexcept { return m_obstaclesArea; }
    [[nodiscard]] float boundaryWallAreaInMeter2()      const noexcept { return m_wallArea; }
    [[nodiscard]] float robotAreaInMeter2()     const noexcept { return m_robotArea; }
    [[nodiscard]] std::size_t numObstacles() const noexcept { return m_obstacleBodies.size(); }
    [[nodiscard]] std::size_t numBoundingWalls()     const noexcept { return m_wallBodies.size(); }
    [[nodiscard]] std::size_t numRobots()    const noexcept { return m_robotBodies.size(); }

    [[deprecated("Use grace::output::currentRunDirectory()")]]
    static const std::filesystem::path& resultsDir();
    [[deprecated("Use grace::output::startRunOutput()")]]
    static void setResultsDir(const std::filesystem::path& path);

protected:
    virtual void resetDerivedState() noexcept = 0;

private:
    b2WorldId m_world{b2_nullWorldId};

    std::vector<b2BodyId> m_obstacleBodies;
    std::vector<b2BodyId> m_wallBodies;
    std::vector<b2BodyId> m_robotBodies;
    std::vector<b2ShapeId> m_shapes;

    float m_obstaclesArea = 0.0f;
    float m_wallArea      = 0.0f;
    float m_robotArea     = 0.0f;

    void resetWorldState();
    void destroyWorld();

    void addBody (Type::BodyType bt, b2BodyId b, float area);
    static b2BodyDef  makeBodyDef  (Type::BodyType bt, b2Vec2 pos, b2Rot rot=b2MakeRot(0)) noexcept;
    static b2ShapeDef makeShapeDef (Type::BodyType bt) noexcept;

};