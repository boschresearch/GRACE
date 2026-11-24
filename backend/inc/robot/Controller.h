// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "../env/BaseEnvironment.h"
#include "RobotSystem.h"

class IController {
public:
    virtual ~IController() = default;
    virtual void update(float dt, BaseEnvironment& env, RobotSystem& reg, b2BodyId id) = 0;
    virtual void reset() {}
    virtual const char* name() const noexcept = 0;
};

// time-parametric tracker — follows (p, t) waypoints with constant speed per segment.
class ConstantSpeedController final : public IController {
public:
    explicit ConstantSpeedController(const float snapEps = 1e-6f)
        : m_time(0.0f), m_snapEps(snapEps) {}

    void update(float dt, BaseEnvironment& env, RobotSystem& reg, b2BodyId id) override;

    void reset() override { m_time = 0.0f; }

    const char* name() const noexcept override { return "ConstantSpeedController"; }

private:
    float m_time;     // controller's local time (s) since start
    float m_snapEps;  // snap to knot when |t - t_knot| <= eps
};