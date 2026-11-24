// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <algorithm>
#include <cmath>
#include <box2d/box2d.h>
#include "../robot/RobotSystem.h"
//
// Shared goal-tolerance helpers for non-grid endpoint checks
//
struct GoalTol {
    float fracOfRadius = 0.05f; // 5% of agent radius
    float minMeters    = 1e-3f; // absolute floor
};

inline bool within_goal_tolerance(const RobotMeta* meta,
                                  b2Vec2 pos,
                                  b2Vec2 goal,
                                  const GoalTol& tol) noexcept
{
    const float rr = (meta && meta->effectiveRadius > 0.f && std::isfinite(meta->effectiveRadius)) ? meta->effectiveRadius : 0.f;
    const float t  = std::max(tol.minMeters, tol.fracOfRadius * rr);
    const float dx = pos.x - goal.x, dy = pos.y - goal.y;
    return std::sqrt(dx*dx + dy*dy) <= t;
}