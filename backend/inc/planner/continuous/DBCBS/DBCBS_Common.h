// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <string>
#include <vector>
#include <array>

struct DBCBSObstacle {
    // type: "box" | "circle" | "capsule" | "polygon"
    std::string type;

    // Common center for box/circle/capsule
    double cx{0.0}, cy{0.0};

    // Box: full extents (width, height)
    double sx{0.0}, sy{0.0};

    // Circle
    double radius{0.0};

    // Capsule: segment endpoints
    double p0x{0.0}, p0y{0.0};
    double p1x{0.0}, p1y{0.0};

    // Polygon: CCW list of points (x,y) in meters
    std::vector<std::array<double,2>> polygon;
};

struct ContinuousRobotMeta {
    std::string type;
    struct Part { double cx{}, cy{}, sx{}, sy{}, angle{}; };

    struct {
        std::string shape;      // "circle" | "box" | "multibox"
        double radius{-1.0};    // for circle
        double size_x{-1.0};    // for box
        double size_y{-1.0};    // for box
        std::vector<Part> boxes; // multibox parts
        bool   valid{false};
    } fp;
};
