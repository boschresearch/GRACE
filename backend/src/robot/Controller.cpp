// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/robot/Controller.h"
#include <cmath>

// ---------------- Timed tracker helpers ----------------
static b2Vec2 lerp(const b2Vec2& a, const b2Vec2& b, float s) {
    return b2Vec2{ a.x + s*(b.x - a.x), a.y + s*(b.y - a.y) };
}

// Find segment i such that W[i].t <= t < W[i+1].t.
// Returns 0 if t <= first time, last index if t >= last time.
static std::size_t segmentForTime(const std::vector<Waypoint>& W, float t)
{
    if (W.empty()) return 0;
    if (t <= W.front().t) return 0;
    if (t >= W.back().t)  return W.size() - 1;

    std::size_t lo = 0, hi = W.size() - 1;
    while (lo + 1 < hi) {
        std::size_t mid = (lo + hi) / 2;
        if (W[mid].t <= t) lo = mid;
        else               hi = mid;
    }
    return lo; // W[lo].t <= t < W[lo+1].t
}

// ---------------- Timed, constant-speed controller ----------------
void ConstantSpeedController::update(float dt, BaseEnvironment& /*env*/, RobotSystem& reg, b2BodyId id)
{
    auto* meta = reg.meta(id);
    const auto& W = meta ? meta->plan.waypoints : std::vector<Waypoint>{};

    if (!meta || !meta->active || W.size() < 2) {
        b2Body_SetLinearVelocity(id, {0,0});
        return;
    }

    // advance controller time
    const float t1 = m_time + dt;
    m_time = t1;

    // Compute desired position at t1 on piecewise-linear path
    b2Vec2 target;

    // Snap exactly onto knots if very close
    auto snapToKnot = [&](float t, b2Vec2& out) -> bool {
        for (std::size_t k=0; k<W.size(); ++k) {
            if (std::fabs(t - W[k].t) <= m_snapEps) {
                out = W[k].p;
                meta->next_wp = std::min(k + 1, W.size());
                return true;
            }
        }
        return false;
    };

    if (snapToKnot(t1, target)) {
        // exact knot; velocity computed below will land exactly here in one step
    } else if (t1 <= W.front().t) {
        target = W.front().p;
        meta->next_wp = std::min<std::size_t>(1, W.size());
    } else if (t1 >= W.back().t) {
        target = W.back().p;
        meta->next_wp = W.size();
    } else {
        const std::size_t i = segmentForTime(W, t1); // W[i].t <= t1 < W[i+1].t
        const float tA = W[i].t, tB = W[i+1].t;
        const float s  = (t1 - tA) / std::max(1e-6f, (tB - tA));
        target = lerp(W[i].p, W[i+1].p, s);
        meta->next_wp = i + 1; // “next waypoint” for GUI/logging
    }

    // Set velocity so physics lands exactly at 'target' after this step.
    if (dt > 0.0f) {
        const b2Vec2 pos = b2Body_GetPosition(id);
        const b2Vec2 v   = b2Vec2{ (target.x - pos.x) / dt, (target.y - pos.y) / dt };
        b2Body_SetLinearVelocity(id, v);
    } else {
        b2Body_SetLinearVelocity(id, {0,0});
    }
}