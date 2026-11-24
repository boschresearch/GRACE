// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

// ViewController.h
// Centralized view control (pan/zoom, mapping). +X right, +Y down.

#pragma once
#include <SFML/Graphics/RenderWindow.hpp>
#include "box2d/types.h"

class ViewController {
public:
    ViewController() = default;

    void attach(sf::RenderWindow* window) { m_window = window; }

    // Zoom semantics: zoom = viewHeight / 2 (world units). +Y is down.
    void  setCenter(b2Vec2 c) const;
    b2Vec2 getCenter() const;

    void  setZoom(float z) const;     // keeps aspect
    float getZoom() const;      // returns viewHeight / 2

    void  panWorld(b2Vec2 d) const;
    void  panPixels(sf::Vector2i dpx) const;

    b2Vec2       mapPixelToWorld(sf::Vector2i px) const;
    sf::Vector2i mapWorldToPixel(b2Vec2 w) const;

    b2AABB getViewBounds() const;

    unsigned     getWindowWidthPx()  const;
    unsigned     getWindowHeightPx() const;
    sf::Vector2u getWindowSizePx()   const;

    b2Vec2 getViewSizeWorld() const; // width,height in world units

private:
    sf::RenderWindow* m_window = nullptr;
};
