// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

// RendererSfml.h
// Consumes DebugDrawQueue batches and renders them with SFML.

#pragma once
#include <SFML/Graphics/RenderTarget.hpp>
#include "DebugDrawQueue.h"
#include "../ui/ViewController.h"

class RendererSfml {
public:
    explicit RendererSfml(ViewController* view) : m_view(view) {}

    static void draw(const DebugDrawQueue& ddq, sf::RenderTarget& target);

private:
    ViewController* m_view{};
};
