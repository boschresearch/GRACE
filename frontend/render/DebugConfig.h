// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#pragma once
class ViewController;
class DebugDrawQueue;
struct SampleContext;

namespace render {
    // Copies visual flags from SampleContext into the b2DebugDraw owned by DebugDrawQueue,
    // and updates drawing bounds using the current view.
    void ApplyDebugConfig(const SampleContext& ctx, DebugDrawQueue& ddq, const ViewController& view);
}
