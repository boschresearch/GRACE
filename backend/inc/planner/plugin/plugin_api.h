// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

struct IPlanner;

inline constexpr int kGracePluginAbiVersion = 3;

enum GracePluginEnvironment : int {
    GRACE_PLUGIN_ENVIRONMENT_GRID = 0,
    GRACE_PLUGIN_ENVIRONMENT_CONTINUOUS = 1,
    GRACE_PLUGIN_ENVIRONMENT_ROADMAP = 2,
};

extern "C" {
using create_planner_fn = IPlanner* (*)();
using destroy_planner_fn = void (*)(IPlanner*);
using plugin_abi_fn = int (*)();
using plugin_id_fn = const char* (*)();
using plugin_environment_fn = int (*)();

int grace_plugin_abi();
const char* grace_plugin_id();
int grace_plugin_environment();
IPlanner* create_planner();
void destroy_planner(IPlanner* planner);
}
