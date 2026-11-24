// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/robot/RobotSystem.h"
#include "../../inc/util/Types.h"
#include "../../inc/util/Logger.h"
#include <fmt/format.h>

namespace {
    float effectiveRadiusCircle(const float radius) {return radius;}
    float effectiveRadiusBox(const float hx, const float hy) {return std::sqrt(hx*hx + hy*hy);}

    // TODO: merge this with the same function as in BaseEnvironment.cpp
    void orientedRectVerts(const b2Vec2 c, const float hx, const float hy, const float ang, b2Vec2 out[4]) {
        const float ca = std::cos(ang), sa = std::sin(ang);
        const b2Vec2 ex{hx, 0.0f}, ey{0.0f, hy};
        const b2Vec2 ax{ ca*ex.x - sa*ex.y, sa*ex.x + ca*ex.y };
        const b2Vec2 ay{ ca*ey.x - sa*ey.y, sa*ey.x + ca*ey.y };
        // corners (counter-clockwise)
        out[0] = { c.x - ax.x - ay.x, c.y - ax.y - ay.y };
        out[1] = { c.x + ax.x - ay.x, c.y + ax.y - ay.y };
        out[2] = { c.x + ax.x + ay.x, c.y + ax.y + ay.y };
        out[3] = { c.x - ax.x + ay.x, c.y - ax.y + ay.y };
    }

    float effectiveRadiusCompoundBoxes(const std::vector<LocalBox>& boxes)
    {
        float maxR2 = 0.0f;

        for (const auto& b : boxes)
        {
            // Reuse your existing vertex generator to match shape creation exactly
            b2Vec2 v[4];
            orientedRectVerts(b.localCenter, b.halfW, b.halfH, b.theta, v);

            // Check the 4 corners
            for (int i = 0; i < 4; ++i)
            {
                const float x = v[i].x;
                const float y = v[i].y;
                const float r2 = x*x + y*y;
                if (r2 > maxR2) maxR2 = r2;
            }
        }

        return std::sqrt(maxR2);
    }

}

b2BodyId RobotSystem::spawnBoxRobot(BaseEnvironment& env,
    const b2Vec2 center, const float halfW, const float halfH, std::string name)
{
    // env does the body+shape+bookkeeping
    b2BodyId id = env.addBoxBody(center, halfW, halfH, 0, Type::BodyType::Robot);

    RobotMeta meta;
    meta.name   = std::move(name);
    meta.shape = Type::ShapeType::Box;
    meta.effectiveRadius = effectiveRadiusBox(halfW, halfH);
    m_meta.emplace(id, std::move(meta));
    m_robots.push_back(id);

    Logger::instance().debug(fmt::format("Spawned box robot {}", m_meta.at(id).name));
    return id;
}

b2BodyId RobotSystem::spawnCompoundBoxesRobot(BaseEnvironment& env, const b2Vec2 center,
                                      const std::vector<LocalBox>& boxes,
                                      std::string name)
{
    b2BodyId id = env.addCompoundBoxesBody(center, boxes, Type::BodyType::Robot);

    RobotMeta meta;
    meta.name   = std::move(name);
    meta.shape = Type::ShapeType::ConvexHull;
    meta.effectiveRadius = effectiveRadiusCompoundBoxes(boxes);
    m_meta.emplace(id, std::move(meta));
    m_robots.push_back(id);

    Logger::instance().debug(fmt::format("Spawned multi-box robot {}", m_meta.at(id).name));
    return id;
}

b2BodyId RobotSystem::spawnCircleRobot(BaseEnvironment& env, const b2Vec2 center, const float radius, std::string name) {
    b2BodyId id = env.addCircleBody(center, radius, Type::BodyType::Robot);

    RobotMeta meta;
    meta.name   = std::move(name);
    meta.shape = Type::ShapeType::Circle;
    meta.effectiveRadius = effectiveRadiusCircle(radius);
    m_meta.emplace(id, std::move(meta));
    m_robots.push_back(id);

    Logger::instance().debug(fmt::format("Spawned circle robot {}", m_meta.at(id).name));
    return id;
}

