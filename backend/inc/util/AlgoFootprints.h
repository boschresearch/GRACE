// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <optional>
#include <string_view>
#include <vector>
#include <algorithm>
#include <cmath>
#include <box2d/box2d.h>

#include "../types/Footprint.h"

namespace grace::util::algo_fp {

// Canonical sizes used across planners (meters)
inline constexpr float R_SINGLE_INT       = 0.10f;
inline constexpr float R_DOUBLE_INT       = 0.15f;
inline constexpr float R_UNICYCLE_SPHERE  = 0.40f;
inline constexpr float BOX_W              = 0.50f; // full width
inline constexpr float BOX_H              = 0.25f; // full height
inline constexpr float CAR_HEAD_W         = 0.50f, CAR_HEAD_H = 0.25f;
inline constexpr float TRAILER_W          = 0.30f, TRAILER_H  = 0.25f;

// Case-sensitive map (useful internally)
inline std::optional<grace::types::Footprint>
footprintForAlgo(std::string_view type) {
    using grace::types::Footprint;
    using grace::types::BoxPart;

    // spheres
    if (type == "single_integrator_0")            return Footprint::Circle(R_SINGLE_INT);
    if (type == "double_integrator_0")            return Footprint::Circle(R_DOUBLE_INT);
    if (type == "unicycle_first_order_0_sphere")  return Footprint::Circle(R_UNICYCLE_SPHERE);

    // boxes
    if (type == "unicycle_first_order_0")         return Footprint::Box(BOX_W, BOX_H);
    if (type == "unicycle_second_order_0")        return Footprint::Box(BOX_W, BOX_H);
    if (type == "car_first_order_0")              return Footprint::Box(BOX_W, BOX_H);

    // head + 1 trailer (two boxes on one body)
    if (type == "car_first_order_with_1_trailers_0") {
        std::vector<BoxPart> parts;
        parts.push_back(BoxPart{/*center*/{ 0.0f,  0.0f}, /*sx*/CAR_HEAD_W, /*sy*/CAR_HEAD_H, /*a*/0.0f});
        parts.push_back(BoxPart{/*center*/{-0.5f,  0.0f}, /*sx*/TRAILER_W,  /*sy*/TRAILER_H,  /*a*/0.0f});
        return Footprint::MultiBox(parts);
    }

    return std::nullopt;
}

// Case-insensitive convenience (recommended for external use)
inline std::optional<grace::types::Footprint>
footprintForAlgoCI(std::string_view type) {
    std::string t(type);
    std::transform(t.begin(), t.end(), t.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return footprintForAlgo(t);
}

} // namespace grace::util::algo_fp
