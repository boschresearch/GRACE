// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <vector>
#include "../../Planner.h"
#include "../../../env/BaseEnvironment.h"
#include "DBCBS_Common.h"

// Writes problem.yaml compatible with external dbcbs tools/optimizer.
void dbcbs_writeProblemYaml(const BaseEnvironment& E,
                            const std::vector<StartGoal>& sgs,
                            const std::vector<ContinuousRobotMeta>& robot_meta,
                            const std::string& robot_type_default,
                            const std::vector<DBCBSObstacle>& env_obstacles,
                            const std::filesystem::path& outYaml);

// Parses agent-wise {result: - states:[ [x,y,...], ... ] }.
// NOTE: No fallbacks; caller decides which file to parse.
bool dbcbs_parseSolutionYaml(const std::filesystem::path& file,
                             double step_dt_s,
                             std::vector<Plan>& outPlans);
