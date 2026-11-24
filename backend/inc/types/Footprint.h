// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <box2d/box2d.h>
#include <vector>
#include <cstdint>

namespace grace::types {

    enum class FootprintKind : uint8_t { Default=0, Circle, Box, MultiBox };

    struct BoxPart {
        b2Vec2 center{0,0};   // local frame (meters)
        float  sx{0};         // full width  (m)
        float  sy{0};         // full height (m)
        float  angle{0};      // radians
    };

    struct Footprint {
        FootprintKind kind{FootprintKind::Default};
        float radius{0.0f};   // circle
        float sx{0.0f}, sy{0.0f}; // single box (full extents)
        std::vector<BoxPart> parts; // multibox (full extents)

        static Footprint Circle(float r) { Footprint f; f.kind=FootprintKind::Circle; f.radius=r; return f; }
        static Footprint Box(float sx, float sy) { Footprint f; f.kind=FootprintKind::Box; f.sx=sx; f.sy=sy; return f; }
        static Footprint MultiBox(std::vector<BoxPart> ps) { Footprint f; f.kind=FootprintKind::MultiBox; f.parts=std::move(ps); return f; }
    };

} // namespace grace::types
