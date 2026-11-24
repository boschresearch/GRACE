// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <box2d/box2d.h>
#include "../env/BaseEnvironment.h"
#include "../robot/RobotSystem.h"
#include "../util/Logger.h"
#include "GoalCommon.h"   // GoalTol + within_goal_tolerance

enum class GoalPolicy { GridCellExact, EuclideanTol };

struct GoalCheckOptions {
    GoalPolicy policy = GoalPolicy::GridCellExact;
    GoalTol    tol{};
};

// Returns true iff every agent is at its goal (according to policy).
bool all_agents_at_goals(const BaseEnvironment& env,
                         const RobotSystem& robots,
                         GoalCheckOptions opt,
                         Logger& log);
