// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/env/BaseEnvironment.h"
#include "../../inc/util/Logger.h"
#include "../../inc/util/RunOutput.h"
#include <fmt/format.h>
#include <cmath>
#include <stdexcept>

namespace {

    float areaOfBox (const float halfW, const float halfH) noexcept { return 4.0f * halfW * halfH; }

    float areaOfCircle(const float r) noexcept { return B2_PI * r * r; }

    float areaOfConvexHull(
        const b2Hull& hullW,
        const float radius) {
        float area = 0.0f;
        float perimeter = 0.0f;
        for (int i=1;i+1<hullW.count;++i) {
            const auto&[ax, ay]  = hullW.points[0];
            const auto&[bx, by]  = hullW.points[i];
            const auto&[cx, cy]  = hullW.points[i+1];
            area += 0.5f * ((bx-ax)*(cy-ay) - (by-ay)*(cx-ax));
        }
        for (int i = 0; i < hullW.count; ++i) {
            const b2Vec2 a = hullW.points[i];
            const b2Vec2 b = hullW.points[(i + 1) % hullW.count];
            perimeter += std::hypot(b.x - a.x, b.y - a.y);
        }
        return std::fabs(area) +
               perimeter * radius +
               B2_PI * radius * radius;
    }

    float areaOfCapsule(const float halfLen, const float r) noexcept {
        return 2.0f * halfLen * (2.0f * r) + B2_PI * r * r;
    }

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

    // centroid for convex polygon (works with signed A = sum cross, not 0.5*sum)
    b2Vec2 convexCentroid(const b2Vec2* p, const int n) {
        float A = 0.0f, Cx = 0.0f, Cy = 0.0f;
        for (int i=0, j=n-1; i<n; j=i++) {
            const float cross = p[j].x*p[i].y - p[j].y*p[i].x;
            A  += cross;
            Cx += (p[j].x + p[i].x) * cross;
            Cy += (p[j].y + p[i].y) * cross;
        }
        if (std::fabs(A) < 1e-9f) return b2Vec2{0,0};
        const float inv = 1.0f / (3.0f * A);
        return {Cx * inv, Cy * inv};
    }
}

/*** public functions ***/

void BaseEnvironment::resetEnvironment() {
    resetDerivedState();
    resetWorldState();
}

void BaseEnvironment::resetWorldState() {
    destroyWorld();
    b2WorldDef def = b2DefaultWorldDef();
    def.gravity = b2Vec2{0.0f, 0.0f};   // top-down
    m_world = b2CreateWorld(&def);
    clearAllBodies();
}

void BaseEnvironment::clearObstacles() {
    for (const auto b : m_obstacleBodies) b2DestroyBody(b);
    m_obstacleBodies.clear();
    m_obstaclesArea = 0.0f;
}

void BaseEnvironment::clearBoundingWalls() {
    for (const auto b : m_wallBodies) b2DestroyBody(b);
    m_wallBodies.clear();
    m_wallArea = 0.0f;
}

void BaseEnvironment::clearRobots() {
    for (const auto b : m_robotBodies) b2DestroyBody(b);
    m_robotBodies.clear();
    m_robotArea = 0.0f;
}

void BaseEnvironment::clearAllBodies() {
    clearObstacles();
    clearBoundingWalls();
    clearRobots();
    m_shapes.clear();
}

void BaseEnvironment::addBoundaryWalls(const float thickness) {
    const auto [W, H] = boundsInMeter();
    const float t = thickness;

    addBoxBody({-t,     H * 0.5f}, t, H * 0.5f, 0, Type::BodyType::BoundingWall);
    addBoxBody({W + t,  H * 0.5f}, t, H * 0.5f, 0, Type::BodyType::BoundingWall);
    addBoxBody({W * 0.5f, -t    }, W * 0.5f, t, 0, Type::BodyType::BoundingWall);
    addBoxBody({W * 0.5f, H + t }, W * 0.5f, t, 0, Type::BodyType::BoundingWall);

    Logger::instance().info(fmt::format("Boundary walls added around 0,0 to ({:.2f} m, {:.2f} m).", W, H));
}

b2BodyId BaseEnvironment::addBoxBody(const b2Vec2 bodyCenter, const float halfW, const float halfH,
    const float theta, const Type::BodyType bt) {
    // create body
    const b2BodyDef bd = makeBodyDef(bt, bodyCenter, b2MakeRot(theta));
    const b2BodyId b = b2CreateBody(m_world, &bd);
    // create shape attached to the body
    const b2ShapeDef sd = makeShapeDef(bt);
    const b2Polygon  poly = b2MakeBox(halfW, halfH);
    const b2ShapeId  sid  = b2CreatePolygonShape(b, &sd, &poly);
    // add body to the environment
    m_shapes.push_back(sid);
    addBody(bt, b, areaOfBox(halfW, halfH));
    return b;
}

b2BodyId BaseEnvironment::addCompoundBoxesBody(const b2Vec2 bodyCenter,
                                       const std::vector<LocalBox>& boxes,
                                       const Type::BodyType bt){
    // create body
    const b2BodyDef bd = makeBodyDef(bt, bodyCenter);
    const b2BodyId b = b2CreateBody(m_world, &bd);
    // create shape attached to the body
    const b2ShapeDef sd = makeShapeDef(bt);
    float totalArea = 0.0f;
    for (const auto&[localCenter, halfW, halfH, angle] : boxes) {
        const float hx = halfW, hy = halfH;
        b2Vec2 v[4];
        orientedRectVerts(localCenter, hx, hy, angle, v);
        const b2Hull hull = b2ComputeHull(v, 4);
        const b2Polygon poly = b2MakePolygon(&hull, /*polygon radius*/ 0.0f);
        const b2ShapeId sid  = b2CreatePolygonShape(b, &sd, &poly);
        m_shapes.push_back(sid);
        totalArea += areaOfBox(hx, hy);
    }
    // add body to the environment
    addBody(bt, b, totalArea);
    return b;
}

b2BodyId BaseEnvironment::addCircleBody(const b2Vec2 bodyCenter, const float radius, const Type::BodyType bt) {
    // create body
    const b2BodyDef bd = makeBodyDef(bt, bodyCenter);
    const b2BodyId b = b2CreateBody(m_world, &bd);
    // create shape attached to the body
    const b2ShapeDef sd = makeShapeDef(bt);
    b2Circle c{}; c.center = {0,0}; c.radius = radius;
    const b2ShapeId sid = b2CreateCircleShape(b, &sd, &c);
    // add body to the environment
    m_shapes.push_back(sid);
    addBody(bt, b, areaOfCircle(radius));
    return b;
}

b2BodyId BaseEnvironment::addConvexBody(const std::vector<b2Vec2>& ptsWorld, const Type::BodyType bt, const float polyRadius){
    if (ptsWorld.size() < 3 ||
        ptsWorld.size() > B2_MAX_POLYGON_VERTICES) {
        throw std::invalid_argument(fmt::format(
            "Convex polygon requires 3 to {} points; received {}",
            B2_MAX_POLYGON_VERTICES,
            ptsWorld.size()));
    }
    if (!std::isfinite(polyRadius) || polyRadius < 0.0f) {
        throw std::invalid_argument(
            "Convex polygon radius must be finite and non-negative");
    }
    for (const b2Vec2 point : ptsWorld) {
        if (!b2IsValidVec2(point)) {
            throw std::invalid_argument(
                "Convex polygon points must contain finite coordinates");
        }
    }

    const b2Hull hullW = b2ComputeHull(ptsWorld.data(), static_cast<int>(ptsWorld.size()));
    if (hullW.count < 3 || !b2ValidateHull(&hullW)) {
        throw std::invalid_argument(
            "Convex polygon points do not form a valid non-degenerate hull");
    }
    // center the body at the centroid of the world hull
    const b2Vec2 cW = convexCentroid(hullW.points, hullW.count);
    if (!b2IsValidVec2(cW)) {
        throw std::invalid_argument(
            "Convex polygon centroid is outside the supported numeric range");
    }
    // build a local-space hull around the centroid for shape creation
    std::vector<b2Vec2> local; local.reserve(hullW.count);
    for (int i = 0; i < hullW.count; ++i) {
        const b2Vec2 point{
            hullW.points[i].x - cW.x,
            hullW.points[i].y - cW.y};
        if (!b2IsValidVec2(point)) {
            throw std::invalid_argument(
                "Convex polygon local coordinates exceed the supported numeric range");
        }
        local.push_back(point);
    }
    const b2Hull hullL = b2ComputeHull(local.data(), static_cast<int>(local.size()));
    if (hullL.count < 3 || !b2ValidateHull(&hullL)) {
        throw std::invalid_argument(
            "Convex polygon became invalid after local-space conversion");
    }
    // create body
    const b2BodyDef bd = makeBodyDef(bt, cW);
    const b2BodyId b = b2CreateBody(m_world, &bd);
    // create shape attached to the body
    const b2ShapeDef sd = makeShapeDef(bt);
    const b2Polygon  poly = b2MakePolygon(&hullL, polyRadius);
    const b2ShapeId  sid  = b2CreatePolygonShape(b, &sd, &poly);
    // add body to the environment
    m_shapes.push_back(sid);
    addBody(bt, b, areaOfConvexHull(hullW, polyRadius));
    return b;
}

b2BodyId BaseEnvironment::addCapsuleBody(const b2Vec2 p0, const b2Vec2 p1, const float radius, const Type::BodyType bt){
    const b2Vec2 d = { p1.x - p0.x, p1.y - p0.y };
    const float  L = std::sqrt(d.x*d.x + d.y*d.y);
    if (L <= 1e-6f) return addCircleBody(p0, radius, bt);
    const float  angle = std::atan2(d.y, d.x);
    const b2Vec2 c = { 0.5f*(p0.x + p1.x), 0.5f*(p0.y + p1.y) };
    return addCapsuleBody(c, 0.5f*L, radius, angle, bt);
}

b2BodyId BaseEnvironment::addCapsuleBody(const b2Vec2 bodyCenter, const float halfLen,
    const float radius, const float theta, const Type::BodyType bt) {
    // create body
    const b2BodyDef bd = makeBodyDef(bt, bodyCenter, b2MakeRot(theta));
    const b2BodyId b  = b2CreateBody(m_world, &bd);
    // create shape attached to the body
    const b2ShapeDef sd = makeShapeDef(bt);
    b2Capsule cap{};
    // Capsule in local frame: segment along the x-axis
    cap.center1 = { -halfLen, 0.0f };
    cap.center2 = {  +halfLen, 0.0f };
    cap.radius  = radius;
    // Create shape
    const b2ShapeId sid = b2CreateCapsuleShape(b, &sd, &cap);
    // add body to the environment
    m_shapes.push_back(sid);
    addBody(bt, b, areaOfCapsule(halfLen, radius));
    return b;
}

const std::filesystem::path& BaseEnvironment::resultsDir() {
    thread_local std::filesystem::path directory;
    directory = grace::output::currentRunDirectory();
    return directory;
}

void BaseEnvironment::setResultsDir(const std::filesystem::path& path) {
    grace::output::RunOutputConfig config;
    config.baseDirectory = path;
    (void)grace::output::startRunOutput(config);
}

/*** private functions ***/

void BaseEnvironment::destroyWorld() {
    clearAllBodies();  // centralize deletion & accounting resets
    if (b2World_IsValid(m_world)) b2DestroyWorld(m_world);
    m_world = {b2_nullWorldId};
}

// Add a body into the environment
inline void BaseEnvironment::addBody(const Type::BodyType bt, const b2BodyId b, const float area) {
    switch (bt) {
        case Type::BodyType::Obstacle:
            m_obstacleBodies.push_back(b);
            m_obstaclesArea += area;
            break;
        case Type::BodyType::BoundingWall:
            m_wallBodies.push_back(b);
            m_wallArea += area;
            break;
        case Type::BodyType::Robot:
            m_robotBodies.push_back(b);
            m_robotArea += area;
            break;
    }
}

b2BodyDef BaseEnvironment::makeBodyDef(const Type::BodyType bt, const b2Vec2 pos, const b2Rot rot) noexcept {
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type     = Type::getb2BodyTypeFromBodyType(bt);
    bd.position = pos;
    bd.rotation = rot;
    return bd;
}

b2ShapeDef BaseEnvironment::makeShapeDef(const Type::BodyType bt) noexcept {
    b2ShapeDef sd = b2DefaultShapeDef();

    // Filters
    sd.filter.categoryBits = to_underlying(bt);
    sd.filter.maskBits     = Type::getb2maskBitsFromBodyType(bt);

    // Physical defaults: ensure dynamic robots actually have mass.
    // (Some Box2D builds default density to 0; this makes robots reliably dynamic.)
    if (bt == Type::BodyType::Robot) {
        sd.density     = 1.0f;
    } else {
        // static bodies; density ignored but keeps values sane
        sd.density     = 0.0f;
    }
    return sd;
}
