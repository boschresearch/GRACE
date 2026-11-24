// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/env/ContinuousEnvironment.h"
#include "../../inc/util/Types.h"
#include <algorithm>        // std::min, std::max
#include <box2d/box2d.h>    // Box2D 3.x C API

namespace {
    inline b2QueryFilter obsWallFilter() {
        b2QueryFilter q{};
        q.categoryBits = ~0ull;
        q.maskBits = to_underlying(Type::BodyType::Obstacle)
                   | to_underlying(Type::BodyType::BoundingWall);
        return q;
    }
    static bool StopOnFirst(b2ShapeId, void* ctx){ *static_cast<bool*>(ctx)=true; return false; }
}

// ---- Original AABB-based version (unchanged semantics) ----
bool ContinuousEnvironment::isFreeAABB(b2AABB aabb) const {
    bool hit=false;
    auto q = obsWallFilter();
    (void)b2World_OverlapAABB(getB2World(), b2Pos_zero, aabb, q, StopOnFirst, &hit);
    return !hit;
}

// ---- Accurate rectangle-vs-shapes overlap using shape proxy ----
bool ContinuousEnvironment::isFreeShape(b2AABB aabb, float inset) const {
    // Apply optional inset to avoid grazing at borders
    const float x0 = std::min(aabb.lowerBound.x, aabb.upperBound.x) + std::max(0.0f, inset);
    const float y0 = std::min(aabb.lowerBound.y, aabb.upperBound.y) + std::max(0.0f, inset);
    const float x1 = std::max(aabb.lowerBound.x, aabb.upperBound.x) - std::max(0.0f, inset);
    const float y1 = std::max(aabb.lowerBound.y, aabb.upperBound.y) - std::max(0.0f, inset);

    // Degenerate (empty) rectangle is trivially free
    if (x1 <= x0 || y1 <= y0) return true;

    b2Vec2 verts[4] = { {x0,y0}, {x1,y0}, {x1,y1}, {x0,y1} };
    b2ShapeProxy proxy = b2MakeProxy(verts, 4, /*radius*/ 0.0f);

    bool hit=false;
    auto q = obsWallFilter();
    (void)b2World_OverlapShape(getB2World(), b2Pos_zero, &proxy, q, StopOnFirst, &hit);
    return !hit;
}

// Returns true if the inflated ray intersects an obstacle/wall.
bool ContinuousEnvironment::raycast(b2Vec2 p1, b2Vec2 p2) const {
    // coarse check: sample along segment in small steps and test inflated AABB
    const int steps = 48;
    const float r = m_robotRadius;
    for (int i = 1; i <= steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const b2Vec2 p{ p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y) };
        b2AABB a{};
        a.lowerBound = { p.x - r, p.y - r };
        a.upperBound = { p.x + r, p.y + r };
        if (!isFreeAABB(a)) return true; // “hit” = ray intersects something
    }
    return false; // clear
}

std::vector<b2Vec2> ContinuousEnvironment::sampleFree(int n) const {
    std::vector<b2Vec2> out; out.reserve(n);
    std::mt19937 rng(12345u);
    std::uniform_real_distribution<float> Ux(0.0f, m_width);
    std::uniform_real_distribution<float> Uy(0.0f, m_height);

    const float r = m_robotRadius;
    for (int k=0; k<n; ++k) {
        for (int tries=0; tries<200; ++tries) {
            const float x = Ux(rng), y = Uy(rng);
            b2AABB aabb{};
            aabb.lowerBound = {x - r, y - r};
            aabb.upperBound = {x + r, y + r};
            if (isFreeAABB(aabb)) { out.push_back({x,y}); break; }
        }
    }
    return out;
}

void ContinuousEnvironment::resetDerivedState() noexcept {
    m_width = 0.0f;
    m_height = 0.0f;
    m_robotRadius = DefaultRobotRadius;
}
