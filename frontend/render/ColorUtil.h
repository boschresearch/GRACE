// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

// render/ColorUtil.h
#pragma once
#include "box2d/types.h"
#include <algorithm>
#include <cstdint>

inline b2HexColor rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return (b2HexColor)((uint32_t(r) << 24) | (uint32_t(g) << 16) | (uint32_t(b) << 8) | uint32_t(a));
}
inline b2HexColor mix(b2HexColor A, b2HexColor B, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    uint32_t ar=(A>>24)&255, ag=(A>>16)&255, ab=(A>> 8)&255, aa=(A>>0)&255;
    uint32_t br=(B>>24)&255, bg=(B>>16)&255, bb=(B>> 8)&255, ba=(B>>0)&255;
    auto L=[&](uint32_t u,uint32_t v){return uint32_t(std::lround((1.f-t)*u+t*v));};
    return (b2HexColor)((L(ar,br)<<24)|(L(ag,bg)<<16)|(L(ab,bb)<<8)|(L(aa,ba)));
}
