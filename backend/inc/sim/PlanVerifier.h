// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <unordered_map>
#include <vector>
#include <optional>
#include <box2d/box2d.h>

#include "../env/BaseEnvironment.h"
#include "../env/ICapabilities.h"
#include "../robot/RobotSystem.h"
#include "../planner/Planner.h"      // PlanResult, Plan, Waypoint
#include "../util/Logger.h"
#include "GoalCommon.h"


enum class NonGridEndpointPolicy { Disabled, EuclideanTolerance };

struct PlanVerifyOptions {
    NonGridEndpointPolicy nonGridPolicy = NonGridEndpointPolicy::Disabled;
    GoalTol               tol{};
};

class PlanVerifier {
public:
    PlanVerifier(const BaseEnvironment& env,
                 const RobotSystem& robots,
                 Logger& log)
      : m_env(env), m_robots(robots), m_log(log) {}

    // Returns true if the whole planner result is accepted; logs diagnostics if not.
    bool accept(const std::vector<b2BodyId>& ids,
                const PlanResult& res,
                const PlanVerifyOptions& opt = {});

private:
    const BaseEnvironment&   m_env;
    const RobotSystem& m_robots;
    Logger&              m_log;

    bool endpointInGoalCell(const IDiscreteGrid& grid, b2Vec2 end, b2Vec2 goal) const;
    bool endpointWithinTol(b2BodyId id, b2Vec2 end, b2Vec2 goal, const GoalTol& tol) const;
};
