// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

// DebugDrawQueue.h
// Box2D debug-draw that records CPU-side batches; no rendering here.

#pragma once
#include <vector>
#include <array>
#include <SFML/Graphics/Color.hpp>
#include "box2d/types.h"
#include <optional>
#include <string>

struct LineItem   { b2Vec2 a{}, b{}; sf::Color c; };
struct PointItem  { b2Vec2 p{}; float sizePx{}; sf::Color c; };
struct CircleItem { b2Vec2 c{}; float r{}; sf::Color col; bool solid{}; };
struct CapsuleItem{ b2Vec2 a{}, b{}; float r{}; sf::Color col; };
struct TextItem    { b2Vec2 p{}; std::string s; sf::Color c; };

struct SolidPolyItem {
    b2Transform            xf{};
    int                    count = 0;
    std::vector<b2Vec2>    pts;
    float                  radius = 0.0f;
    sf::Color              col{};
};

class DebugDrawQueue {
public:
    enum class Phase : int { World = 0, OverlayBack = 1, OverlayFront = 2, COUNT = 3 };

    DebugDrawQueue();
    ~DebugDrawQueue() = default;

    // b2DebugDraw to pass into b2World_Draw
    b2DebugDraw& getB2DebugDraw() { return m_dd; }

    void setSolidPolyColorOverride(std::optional<sf::Color> c) { m_polyOverride = std::move(c); }
    void setCapsuleColorOverride  (std::optional<sf::Color> c) { m_capOverride  = std::move(c); }

    // Phase selection
    void beginWorldPass()   { m_phase = Phase::World;        }
    void endWorldPass()     { /* no-op */                    }
    void beginOverlayBack() { m_phase = Phase::OverlayBack;  }
    void beginOverlayFront(){ m_phase = Phase::OverlayFront; }

    // Frontend overlays use this instead of calling Box2D callback pointers directly.
    void drawLine(b2Vec2 p1, b2Vec2 p2, b2HexColor color);

    // Batches (render asks per phase)
    [[nodiscard]] const std::vector<LineItem>&      lines(Phase p)    const { return m_lines[(int)p]; }
    [[nodiscard]] const std::vector<PointItem>&     points(Phase p)   const { return m_points[(int)p]; }
    [[nodiscard]] const std::vector<CircleItem>&    circles(Phase p)  const { return m_circles[(int)p]; }
    [[nodiscard]] const std::vector<CapsuleItem>&   capsules(Phase p) const { return m_capsules[(int)p]; }
    [[nodiscard]] const std::vector<SolidPolyItem>& polys(Phase p)    const { return m_solidPolys[(int)p]; }
    [[nodiscard]] const std::vector<TextItem>&      texts(Phase p)    const { return m_texts[(int)p]; }

    void clear();

    // Allow the renderer to query whether AABBs should be drawn.
    bool drawBoundsFlag() const { return m_dd.drawBounds; }

private:
    static void DrawPolygonFcn (b2WorldTransform xf, const b2Vec2* vertices,
        int vertexCount, b2HexColor color, void* context);
    static void DrawSolidPolygonFcn(b2WorldTransform xf, const b2Vec2* vertices,
        int vertexCount, float radius, b2HexColor color, void* context);
    static void DrawCircleFcn   (b2Pos center, float radius, b2HexColor color, void* context);
    static void DrawSolidCircleFcn(b2WorldTransform xf, b2Vec2 center,
        float radius, b2HexColor color, void* context);
    static void DrawSolidCapsuleFcn(b2Pos a, b2Pos b, float radius, b2HexColor color, void* context);
    static void DrawLineFcn     (b2Pos p1, b2Pos p2, b2HexColor color, void* context);
    static void DrawTransformFcn(b2WorldTransform xf, void* context);
    static void DrawPointFcn    (b2Pos p, float size, b2HexColor color, void* context);
    static void DrawStringFcn   (b2Pos p, const char* s, b2HexColor, void* context);

    b2DebugDraw m_dd{};

    // One container per phase
    std::array<std::vector<LineItem>,      (int)Phase::COUNT> m_lines;
    std::array<std::vector<PointItem>,     (int)Phase::COUNT> m_points;
    std::array<std::vector<CircleItem>,    (int)Phase::COUNT> m_circles;
    std::array<std::vector<CapsuleItem>,   (int)Phase::COUNT> m_capsules;
    std::array<std::vector<SolidPolyItem>, (int)Phase::COUNT> m_solidPolys;
    std::array<std::vector<TextItem>,      (int)Phase::COUNT> m_texts;

    std::optional<sf::Color>   m_polyOverride;
    std::optional<sf::Color>   m_capOverride;

    Phase m_phase = Phase::World; // current recording phase
};
