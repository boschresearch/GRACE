// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "BaseEnvironment.h"
#include "ICapabilities.h"
#include <random>

class ContinuousEnvironment : public BaseEnvironment, public IContinuousSpace {
public:
    EnvironmentType getEnvironmentType() const override { return EnvironmentType::ContinuousEnvironment; }
    Bounds  boundsInMeter() const override { return {m_width, m_height}; }
    const IContinuousSpace* continuousCapability() const noexcept override { return this; }

    void setBounds(float W, float H) { m_width=W; m_height=H; }

    // IContinuousSpace
    bool raycast(b2Vec2 p1, b2Vec2 p2) const override;
    std::vector<b2Vec2> sampleFree(int n) const override;
    void setRobotRadius(float r) override { m_robotRadius = r; }

    // Fast broad-phase test: AABB vs world (your current behavior)
    bool isFreeAABB(b2AABB aabb) const;

    // Accurate test: axis-aligned rectangle (AABB minus inset) vs world shapes
    // Returns true if the rectangle is FREE of obstacles/walls.
    bool isFreeShape(b2AABB aabb, float inset = 0.0f) const;

private:
    static constexpr float DefaultRobotRadius = 0.25f;

    float m_width{0.0f}, m_height{0.0f};
    float m_robotRadius{DefaultRobotRadius}; // used to inflate sampling AABB

    void resetDerivedState() noexcept override;
};
