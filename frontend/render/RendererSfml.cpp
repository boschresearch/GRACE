// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#include "RendererSfml.h"
#include <SFML/Graphics.hpp>
#include "box2d/math_functions.h"
#include <array>
#include <vector>
#include <cmath>
#include <limits>  // <-- added

void RendererSfml::draw(const DebugDrawQueue& ddq, sf::RenderTarget& target) {
    const sf::View view = target.getView();
    const sf::Vector2f vsz = view.getSize();
    const sf::Vector2u win = target.getSize();
    const float wppX = win.x ? std::abs(vsz.x) / static_cast<float>(win.x) : 0.0f;
    const float wppY = win.y ? std::abs(vsz.y) / static_cast<float>(win.y) : 0.0f;

    auto drawPhase = [&](DebugDrawQueue::Phase ph)
    {
        // Polygons
        for (const auto& poly : ddq.polys(ph))
        {
            sf::ConvexShape shape;
            shape.setPointCount(poly.count);
            for (int i = 0; i < poly.count; ++i) {
                const auto p = b2TransformPoint(poly.xf, poly.pts[i]);
                shape.setPoint(i, sf::Vector2f(p.x, p.y));
            }
            shape.setFillColor(poly.col);
            shape.setOutlineThickness(0.0f);
            target.draw(shape);
        }

        // Solid capsules
        for (const auto& item : ddq.capsules(ph))
        {
            const b2Vec2 d { item.b.x - item.a.x, item.b.y - item.a.y };
            const float len = std::sqrt(d.x*d.x + d.y*d.y);
            if (len < 1e-6f) continue;

            const float cx = 0.5f * (item.a.x + item.b.x);
            const float cy = 0.5f * (item.a.y + item.b.y);
            const float angleDeg = std::atan2(d.y, d.x) * 180.0f / 3.14159265358979323846f;

            const float R = std::max(0.0f, item.r);
            const float L = len;

            sf::RectangleShape rect(sf::Vector2f(L, 2.0f * R));
#if SFML_VERSION_MAJOR >= 3
            rect.setRotation(sf::degrees(angleDeg));
#else
            rect.setRotation(angleDeg);
#endif
            rect.setFillColor(item.col);
            rect.setOrigin(sf::Vector2f(L * 0.5f, R));
            rect.setPosition(sf::Vector2f(cx, cy));
            target.draw(rect);

            sf::CircleShape disk(R);
            disk.setFillColor(item.col);
            disk.setPosition(sf::Vector2f(item.a.x - R, item.a.y - R));
            target.draw(disk);
            disk.setPosition(sf::Vector2f(item.b.x - R, item.b.y - R));
            target.draw(disk);
        }

        // Circles
        for (const auto& c : ddq.circles(ph))
        {
            const float R = std::max(0.0f, c.r);
            sf::CircleShape circle(R);
            if (c.solid) {
                circle.setFillColor(c.col);
                circle.setOutlineThickness(0.0f);
            } else {
                circle.setFillColor(sf::Color(0,0,0,0));
                circle.setOutlineColor(c.col);
                circle.setOutlineThickness(wppY);
            }
            circle.setPosition(sf::Vector2f(c.c.x - R, c.c.y - R));
            target.draw(circle);
        }

        // Lines
        if (!ddq.lines(ph).empty())
        {
            std::vector<sf::Vertex> verts;
            verts.reserve(ddq.lines(ph).size() * 2);
            for (const auto& L : ddq.lines(ph))
            {
                verts.emplace_back(sf::Vector2f(L.a.x, L.a.y), L.c);
                verts.emplace_back(sf::Vector2f(L.b.x, L.b.y), L.c);
            }
            target.draw(verts.data(), verts.size(), sf::PrimitiveType::Lines);
        }

        // Points (pixel-constant)
        for (const auto& P : ddq.points(ph))
        {
            const float r = 0.5f * P.sizePx * wppY;
            sf::CircleShape dot(r);
            dot.setFillColor(P.c);
            dot.setOutlineThickness(0.0f);
            dot.setPosition(sf::Vector2f(P.p.x - r, P.p.y - r));
            target.draw(dot);
        }

        // Text (world-anchored)
        if (!ddq.texts(ph).empty())
        {
            static sf::Font font;
            static bool fontReady = false;
            if (!fontReady) {
                fontReady = font.openFromFile("data/droid_sans.ttf");
            }
            if (fontReady) {
                for (const auto& T : ddq.texts(ph)) {
                    sf::Text txt(font, T.s);
                    txt.setFillColor(T.c);
                    txt.setPosition(sf::Vector2f(T.p.x, T.p.y));
                    target.draw(txt);
                }
            }
        }

        const auto drawRectOutline = [&](float xmin, float ymin, float xmax, float ymax, const sf::Color col){
            std::array<sf::Vertex, 8> vv = {
                sf::Vertex({xmin,ymin}, col), sf::Vertex({xmax,ymin}, col),
                sf::Vertex({xmax,ymin}, col), sf::Vertex({xmax,ymax}, col),
                sf::Vertex({xmax,ymax}, col), sf::Vertex({xmin,ymax}, col),
                sf::Vertex({xmin,ymax}, col), sf::Vertex({xmin,ymin}, col)
            };
            target.draw(vv.data(), vv.size(), sf::PrimitiveType::Lines);
        };

        // Fallback/assist: draw per-shape AABBs when "Bounds" is enabled.
        if (ph == DebugDrawQueue::Phase::World && ddq.drawBoundsFlag())
        {
            const sf::Color aabbCol(255, 210, 0, 200);

            // Polygons
            for (const auto& poly : ddq.polys(ph)) {
                float xmin =  std::numeric_limits<float>::infinity();
                float ymin =  std::numeric_limits<float>::infinity();
                float xmax = -std::numeric_limits<float>::infinity();
                float ymax = -std::numeric_limits<float>::infinity();
                for (int i = 0; i < poly.count; ++i) {
                    const auto p = b2TransformPoint(poly.xf, poly.pts[i]);
                    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
                    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
                }
                drawRectOutline(xmin, ymin, xmax, ymax, aabbCol);
            }
            // Capsules
            for (const auto& item : ddq.capsules(ph)) {
                const float xmin = std::min(item.a.x, item.b.x) - item.r;
                const float xmax = std::max(item.a.x, item.b.x) + item.r;
                const float ymin = std::min(item.a.y, item.b.y) - item.r;
                const float ymax = std::max(item.a.y, item.b.y) + item.r;
                drawRectOutline(xmin, ymin, xmax, ymax, aabbCol);
            }
            // Circles
            for (const auto& c : ddq.circles(ph)) {
                const float R    = std::max(0.0f, c.r);
                const float xmin = c.c.x - R, xmax = c.c.x + R;
                const float ymin = c.c.y - R, ymax = c.c.y + R;
                drawRectOutline(xmin, ymin, xmax, ymax, aabbCol);
            }
            // Points (render small squares in world space)
            for (const auto& P : ddq.points(ph)) {
                const float r = 0.5f * P.sizePx * wppY;
                const float xmin = P.p.x - r, xmax = P.p.x + r;
                const float ymin = P.p.y - r, ymax = P.p.y + r;
                drawRectOutline(xmin, ymin, xmax, ymax, aabbCol);
            }
        }
    };

    drawPhase(DebugDrawQueue::Phase::OverlayBack);
    drawPhase(DebugDrawQueue::Phase::World);
    drawPhase(DebugDrawQueue::Phase::OverlayFront);
}
