// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

// ViewController.cpp

#include "ViewController.h"
#include <algorithm>
#include <cmath>
#include <SFML/Graphics/View.hpp>

void ViewController::setCenter(b2Vec2 c) const {
    if (!m_window) return;
    sf::View v = m_window->getView();
    v.setCenter({ c.x, c.y });
    m_window->setView(v);
}

b2Vec2 ViewController::getCenter() const {
    if (!m_window) return {0,0};
    const sf::Vector2f c = m_window->getView().getCenter();
    return { c.x, c.y };
}

void ViewController::setZoom(float z) const {
    if (!m_window) return;
    z = std::max(0.0001f, z);
    sf::View v = m_window->getView();
    const auto win = m_window->getSize();
    const float aspect = win.y ? static_cast<float>(win.x) / static_cast<float>(win.y) : 1.0f;
    v.setSize({ 2.0f * z * aspect, 2.0f * z }); // +Y down (positive height)
    m_window->setView(v);
}

float ViewController::getZoom() const {
    if (!m_window) return 1.0f;
    const sf::Vector2f s = m_window->getView().getSize();
    return 0.5f * std::abs(s.y);
}

void ViewController::panWorld(b2Vec2 d) const {
    if (!m_window) return;
    sf::View v = m_window->getView();
    v.move({ d.x, d.y });
    m_window->setView(v);
}

void ViewController::panPixels(const sf::Vector2i dpx) const {
    if (!m_window) return;
    const sf::View v = m_window->getView();
    const auto win   = m_window->getSize();
    const sf::Vector2f vsz = v.getSize();
    const float wppX = win.x ? vsz.x / static_cast<float>(win.x) : 0.f;
    const float wppY = win.y ? vsz.y / static_cast<float>(win.y) : 0.f; // +Y down
    panWorld({ -static_cast<float>(dpx.x) * wppX, -static_cast<float>(dpx.y) * wppY });
}

b2Vec2 ViewController::mapPixelToWorld(const sf::Vector2i px) const {
    if (!m_window) return { static_cast<float>(px.x), static_cast<float>(px.y) };
    const sf::Vector2f w = m_window->mapPixelToCoords(px);
    return { w.x, w.y };
}

sf::Vector2i ViewController::mapWorldToPixel(b2Vec2 w) const {
    if (!m_window) return { static_cast<int>(w.x), static_cast<int>(w.y) };
    return m_window->mapCoordsToPixel({ w.x, w.y });
}

b2AABB ViewController::getViewBounds() const {
    b2AABB aabb{};
    if (!m_window) return aabb;
    const sf::View& v   = m_window->getView();
    const sf::Vector2f c = v.getCenter();
    const sf::Vector2f s = v.getSize();
    const float hx = std::abs(s.x) * 0.5f;
    const float hy = std::abs(s.y) * 0.5f;
    aabb.lowerBound = { c.x - hx, c.y - hy };
    aabb.upperBound = { c.x + hx, c.y + hy };
    return aabb;
}

unsigned ViewController::getWindowWidthPx()  const { return m_window ? m_window->getSize().x : 0u; }
unsigned ViewController::getWindowHeightPx() const { return m_window ? m_window->getSize().y : 0u; }
sf::Vector2u ViewController::getWindowSizePx() const { return m_window ? m_window->getSize() : sf::Vector2u(); }

b2Vec2 ViewController::getViewSizeWorld() const {
    if (!m_window) return {0,0};
    const sf::Vector2f s = m_window->getView().getSize();
    return { std::abs(s.x), std::abs(s.y) };
}
