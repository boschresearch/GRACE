// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include <optional>
#include <box2d/box2d.h>
#include "../robot/RobotSystem.h"

namespace grace::sim {

    enum class ObstacleKind : uint8_t { Box, Circle, Polygon, Capsule };

    struct BoxObstacle {
        b2Vec2 center{}; // meters (world coords from YAML)
        b2Vec2 size{};   // full size [w, h] in meters
    };

    // Unified obstacle spec
    struct ObstacleYaml {
        ObstacleKind kind{ObstacleKind::Box};

        // Box / Circle
        b2Vec2 center{};     // box/circle center
        b2Vec2 size{};       // box full extents [w,h]
        float  radius{0.0f}; // circle radius

        // Polygon (convex, no holes)
        std::vector<b2Vec2> points; // world coords

        // Capsule
        b2Vec2 p0{}, p1{}; // world coords (segment ends)
    };

    // Optional per-robot footprint override (local frame of the robot)
    enum class RobotFootprintKind : uint8_t { None = 0, Circle, Box, MultiBox };

    struct RobotFootprintYaml {
        RobotFootprintKind kind{RobotFootprintKind::None};
        // circle
        float radius{0.0f};
        // single box (full extents, meters)
        b2Vec2 size{};   // [sx, sy]
        // multibox parts (full extents, meters), local centers and radians
        struct BoxPart { b2Vec2 center{}; b2Vec2 size{}; float angle{0.0f}; };
        std::vector<BoxPart> boxes;
    };

    struct RobotYamlSpec {
        std::string type; // external algo name, e.g. "unicycle_first_order_0_sphere"
        b2Vec2 start{};   // meters (x,y) — theta ignored for spawning/tracking
        b2Vec2 goal{};    // meters (x,y)
        std::optional<RobotFootprintYaml> footprint; // optional override
    };

    struct ProblemYaml {
        b2Vec2 min{}; // environment min corner (meters)
        b2Vec2 max{}; // environment max corner (meters)
        std::vector<ObstacleYaml> obstacles;  // <— generalized
        std::vector<RobotYamlSpec> robots;
    };

    // Parse a DBCBS-style problem.yaml into ProblemYaml.
    // Returns an empty spec (robots empty) on error.
    ProblemYaml loadProblemYaml(const std::filesystem::path& yamlPath);

    // Build a compact YAML string of robots' meta (type + optional footprint).
    // Overload 1: from parsed ProblemYaml::robots
    std::string emitRobotsMetaYaml(const std::vector<RobotYamlSpec>& robots);

    // Overload 2: from the live RobotRegistry (fallback: circle footprints by radius)
    std::string emitRobotsMetaYaml(const RobotSystem& reg);

    // Build obstacles YAML string from ProblemYaml, converted to a local frame such that
    // 'mins' becomes the origin (same convention used in Simulation).
    std::string emitObstaclesYamlLocal(const ProblemYaml& prob, b2Vec2 mins);
}