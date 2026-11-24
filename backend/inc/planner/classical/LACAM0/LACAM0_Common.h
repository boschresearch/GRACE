// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "../inc/planner/Planner.h"

struct LACAM0_CommonConfig {
    int    verbose     = 3;
    int    seed        = 0;
    bool   log_short   = false;

    bool anytime            = false;
    bool multi_thread_init  = true;
    bool pibt_swap          = true;
    bool pibt_hindrance     = true;

    double step_dt_s = 1.0;
};

inline void applyCommonSettings(const PlannerSettings& s, LACAM0_CommonConfig& cfg) {
    if (auto v = s.get<int>("verbose"))               cfg.verbose = *v;
    if (auto v = s.get<int>("seed"))                  cfg.seed = *v;
    if (auto v = s.get<bool>("log_short"))            cfg.log_short = *v;
    if (auto v = s.get<bool>("anytime"))              cfg.anytime = *v;
    if (auto v = s.get<bool>("multi_thread_init"))    cfg.multi_thread_init = *v;
    if (auto v = s.get<bool>("pibt_swap"))            cfg.pibt_swap = *v;
    if (auto v = s.get<bool>("pibt_hindrance"))       cfg.pibt_hindrance = *v;

    if (auto v = s.get<double>("step_dt_s"))          cfg.step_dt_s = *v;
}
