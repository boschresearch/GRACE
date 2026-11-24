// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <unordered_map>
#include <utility>
#include <box2d/box2d.h>

#include "../env/ICapabilities.h"
#include "../robot/RobotSystem.h"
#include "../util/Box2dIdHash.h"
#include "../util/Logger.h"

struct PairHash {
    size_t operator()(const std::pair<int,int>& p) const {
        return (static_cast<uint64_t>(p.first) << 32) ^ static_cast<uint32_t>(p.second);
    }
};

class GridConflictAuditor {
public:
    explicit GridConflictAuditor(float tickSec = 1.0f)
      : m_tickSec(tickSec) {}

    void reset();
    void tick(float simTime, const IDiscreteGrid& grid, const RobotSystem& robots, Logger& log);

private:
    int   m_lastTick{-1};
    float m_tickSec{1.0f};

    // Prev tick: id -> rc
    std::unordered_map<b2BodyId, std::pair<int,int>, BodyIdHash, BodyIdEq> m_prevCell;
};
