// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <unordered_map>
#include <string>
#include <vector>
#include <optional>
#include <box2d/box2d.h>
#include "../env/BaseEnvironment.h"
#include "../util/Box2dIdHash.h"
#include "../planner/Planner.h"

struct RobotMeta {
    std::string name;
    Type::ShapeType shape;
    float effectiveRadius{0.0f};
    Plan plan;
    std::size_t next_wp{0};
    bool active{true};

    // The *commanded* goal in env meters from Simulation::planBatch(sgs).
    std::optional<b2Vec2> requested_goal_m{};
};

class RobotSystem {
    public:

        [[maybe_unused]] b2BodyId spawnBoxRobot(BaseEnvironment& env, b2Vec2 center, float halfW, float halfH, std::string name);
        [[maybe_unused]] b2BodyId spawnCompoundBoxesRobot(BaseEnvironment& env, b2Vec2 center, const std::vector<LocalBox>& boxes, std::string name);
        [[maybe_unused]] b2BodyId spawnCircleRobot(BaseEnvironment& env, b2Vec2 center, float radius, std::string name);

        // TODO: consider other shapes of robot

        RobotMeta* meta(const b2BodyId id) {
            const auto it = m_meta.find(id); return it==m_meta.end()?nullptr:&it->second;
        }
        const RobotMeta* meta(const b2BodyId id) const {
            const auto it = m_meta.find(id); return it==m_meta.end()?nullptr:&it->second;
        }

        [[nodiscard]] const std::vector<b2BodyId>& robots() const noexcept { return m_robots; }

        void clear() { m_meta.clear(); m_robots.clear(); }

    private:
        std::unordered_map<b2BodyId, RobotMeta, BodyIdHash, BodyIdEq> m_meta;
        std::vector<b2BodyId> m_robots;
};