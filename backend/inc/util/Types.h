// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <stdexcept>
#include <type_traits>
#include <string>
#include <cstdint>
#include <box2d/box2d.h>

template <class E>
constexpr std::underlying_type_t<E> to_underlying(E e) noexcept {
    static_assert(std::is_enum_v<E>, "to_underlying requires an enum");
    return static_cast<std::underlying_type_t<E>>(e);
}

class Type{
    public:
        enum class BodyType: uint64_t {
            Obstacle = 0x00000001,
            BoundingWall = 0x00000002,
            Robot = 0x00000004,
        };

        enum class ShapeType {
            Circle,
            Capsule,
            Box,
            ConvexHull
        };

        enum class MotionType {
            Static,
            Kinematic,
            Dynamic
        };

        enum class MultiRobotPlannerType {
            PrioritizedSippPlanner
        };

        // Function to convert string to ShapeType
        static ShapeType getShapeTypeFromString(const std::string &type) {
            if (type == "Circle") return ShapeType::Circle;
            if (type == "Capsule") return ShapeType::Capsule;
            if (type == "Box") return ShapeType::Box;
            if (type == "ConvexHull") return ShapeType::ConvexHull;
            throw std::invalid_argument("Unknown RobotType: " + type);
        }

        // Function to convert string to motionType
        static MotionType getMotionTypeFromString(const std::string &type) {
            if (type == "Dynamic") return MotionType::Dynamic;
            if (type == "Kinematic") return MotionType::Kinematic;
            if (type == "Static") return MotionType::Static;
            throw std::invalid_argument("Unknown MotionType: " + type);
        }

        // Function to convert string to MultiRobotPlannerType
        static MultiRobotPlannerType getMultiRobotPlannerTypeFromString(const std::string &type) {
            if (type == "PrioritizedSippPlanner") return MultiRobotPlannerType::PrioritizedSippPlanner;
            throw std::invalid_argument("Unknown MultiRobotPlannerType: " + type);
        }

        static b2BodyType getb2BodyTypeFromBodyType(const BodyType bodyType) {
                switch (bodyType) {
                    case BodyType::Obstacle: return b2_staticBody;
                    case BodyType::BoundingWall: return b2_staticBody;
                    case BodyType::Robot: return b2_dynamicBody;
                }
                throw std::invalid_argument("Unknown BodyType!");
            }

        static uint64_t getb2maskBitsFromBodyType(const BodyType bodyType) {
                switch (bodyType) {
                    case BodyType::Obstacle:
                        return to_underlying(BodyType::Robot);
                    case BodyType::BoundingWall:
                        return to_underlying(BodyType::Robot);
                    case BodyType::Robot:
                        return //to_underlying(BodyType::Robot) |
                                to_underlying(BodyType::Obstacle) |
                                to_underlying(BodyType::BoundingWall);
                }
                throw std::invalid_argument("Unknown BodyType!");
            }

       // Optional: adapter to 16-bit Box2D mask/category (if you wire this into b2Filter)
       static uint16_t mask16FromBodyMask(uint64_t bits) {
           return static_cast<uint16_t>(bits & 0xFFFFu);
       }
       static uint16_t category16FromBody(const BodyType bodyType) {
           return static_cast<uint16_t>(to_underlying(bodyType) & 0xFFFFu);
       }

  };
