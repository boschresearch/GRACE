// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

// DebugDrawQueue.cpp

#include "DebugDrawQueue.h"
#include "box2d/math_functions.h"
#include <algorithm>
#include <cmath>

static sf::Color ToSfColor(const b2HexColor c, float alphaMul = 1.0f) {
    const uint8_t r  = (c >> 24) & 0xFF;
    const uint8_t g  = (c >> 16) & 0xFF;
    const uint8_t b  = (c >>  8) & 0xFF;
    const uint8_t ca =  c        & 0xFF;
    const float   am = std::clamp(alphaMul, 0.0f, 1.0f);
    const uint8_t a  = static_cast<uint8_t>(std::lround(am * ca));
    return { r, g, b, a };
}

static b2Vec2 ToVec2(const b2Pos p)
{
    return {static_cast<float>(p.x), static_cast<float>(p.y)};
}

DebugDrawQueue::DebugDrawQueue()
{
    m_dd = b2DefaultDebugDraw();
    m_dd.DrawPolygonFcn        = &DebugDrawQueue::DrawPolygonFcn;
    m_dd.DrawSolidPolygonFcn   = &DebugDrawQueue::DrawSolidPolygonFcn;
    m_dd.DrawCircleFcn         = &DebugDrawQueue::DrawCircleFcn;
    m_dd.DrawSolidCircleFcn    = &DebugDrawQueue::DrawSolidCircleFcn;
    m_dd.DrawSolidCapsuleFcn   = &DebugDrawQueue::DrawSolidCapsuleFcn;
    m_dd.DrawLineFcn           = &DebugDrawQueue::DrawLineFcn;
    m_dd.DrawTransformFcn      = &DebugDrawQueue::DrawTransformFcn;
    m_dd.DrawPointFcn          = &DebugDrawQueue::DrawPointFcn;
    m_dd.DrawStringFcn         = &DebugDrawQueue::DrawStringFcn;
    m_dd.context               = this;

    m_dd.drawShapes            = true;
    m_dd.drawJoints            = true;
}

void DebugDrawQueue::drawLine(const b2Vec2 p1, const b2Vec2 p2, const b2HexColor color)
{
    m_lines[static_cast<int>(m_phase)].push_back({p1, p2, ToSfColor(color)});
}

void DebugDrawQueue::clear()
{
    for (int i = 0; i < (int)Phase::COUNT; ++i) {
        m_lines[i].clear();
        m_points[i].clear();
        m_circles[i].clear();
        m_capsules[i].clear();
        m_solidPolys[i].clear();
        m_texts[i].clear();
    }
}

// --- static callbacks --------------------------------------------------------

void DebugDrawQueue::DrawPolygonFcn(const b2WorldTransform xf, const b2Vec2* vertices,
    const int vertexCount, const b2HexColor color, void* context)
{
    auto* self = static_cast<DebugDrawQueue*>(context);
    const sf::Color col = ToSfColor(color, 1.0f);
    if (vertexCount <= 0) {
        return;
    }
    b2Vec2 p1 = ToVec2(b2TransformWorldPoint(xf, vertices[vertexCount - 1]));
    for (int i = 0; i < vertexCount; ++i) {
        const b2Vec2 p2 = ToVec2(b2TransformWorldPoint(xf, vertices[i]));
        self->m_lines[(int)self->m_phase].push_back({ p1, p2, col });
        p1 = p2;
    }
}

void DebugDrawQueue::DrawSolidPolygonFcn(b2WorldTransform xf, const b2Vec2* vertices,
    int vertexCount, float radius, b2HexColor color, void* context)
{
    auto* self = static_cast<DebugDrawQueue*>(context);
    SolidPolyItem item{};
    item.xf = b2ToRelativeTransform(xf, b2Pos_zero);
    if (vertexCount > 0) {
        item.pts.assign(vertices, vertices + vertexCount);
        item.count = vertexCount;
    }
    item.radius = radius;

    // Only override during WORLD pass (keeps overlays’ colors intact)
    item.col = (self->m_phase == Phase::World && self->m_polyOverride)
        ? *self->m_polyOverride
        : ToSfColor(color, 1.0f);

    self->m_solidPolys[(int)self->m_phase].push_back(item);
}

void DebugDrawQueue::DrawCircleFcn(const b2Pos center, const float radius, const b2HexColor color, void* context)
{
    auto* self = static_cast<DebugDrawQueue*>(context);
    self->m_circles[(int)self->m_phase].push_back(
        {ToVec2(center), radius, ToSfColor(color, 1.0f), false});
}

void DebugDrawQueue::DrawSolidCircleFcn(const b2WorldTransform xf, const b2Vec2 center,
    const float radius, const b2HexColor color, void* context)
{
    auto* self = static_cast<DebugDrawQueue*>(context);
    const b2Vec2 worldCenter = ToVec2(b2TransformWorldPoint(xf, center));
    self->m_circles[(int)self->m_phase].push_back(
        {worldCenter, radius, ToSfColor(color, 1.0f), true});
}

void DebugDrawQueue::DrawSolidCapsuleFcn(b2Pos a, b2Pos b, float radius, b2HexColor color, void* context)
{
    auto* self = static_cast<DebugDrawQueue*>(context);
    CapsuleItem item{};
    item.a = ToVec2(a);
    item.b = ToVec2(b);
    item.r = radius;

    // Only override during WORLD pass
    item.col = (self->m_phase == Phase::World && self->m_capOverride)
        ? *self->m_capOverride
        : ToSfColor(color, 1.0f);

    self->m_capsules[(int)self->m_phase].push_back(item);
}

void DebugDrawQueue::DrawLineFcn(const b2Pos p1, const b2Pos p2, const b2HexColor color, void* context)
{
    auto* self = static_cast<DebugDrawQueue*>(context);
    self->drawLine(ToVec2(p1), ToVec2(p2), color);
}

void DebugDrawQueue::DrawTransformFcn(const b2WorldTransform xf, void* context)
{
    auto* self = static_cast<DebugDrawQueue*>(context);
    constexpr float k_axisScale = 0.2f;
    const b2Vec2 p1 = ToVec2(xf.p);

    b2Vec2 p2 = b2MulAdd(p1, k_axisScale, b2Rot_GetXAxis(xf.q));
    self->m_lines[(int)self->m_phase].push_back({ p1, p2, ToSfColor(b2_colorRed, 1.0f) });

    p2 = b2MulAdd(p1, k_axisScale, b2Rot_GetYAxis(xf.q));
    self->m_lines[(int)self->m_phase].push_back({ p1, p2, ToSfColor(b2_colorGreen, 1.0f) });
}

void DebugDrawQueue::DrawPointFcn(const b2Pos p, float size, const b2HexColor color, void* context)
{
#ifdef __APPLE__
    size *= 2.0f;
#endif
    auto* self = static_cast<DebugDrawQueue*>(context);
    self->m_points[(int)self->m_phase].push_back({ToVec2(p), size, ToSfColor(color, 1.0f)});
}

void DebugDrawQueue::DrawStringFcn(b2Pos p, const char* s, b2HexColor color, void* context)
{
    auto* self = static_cast<DebugDrawQueue*>(context);
    TextItem item;
    item.p = ToVec2(p);
    item.s = s ? std::string(s) : std::string();
    item.c = ToSfColor(color, 1.0f);
    self->m_texts[(int)self->m_phase].push_back(std::move(item));
}