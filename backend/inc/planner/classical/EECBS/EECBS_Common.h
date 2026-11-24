// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <optional>
#include <string>

struct EECBS_CommonConfig {
    int         seed          = 0;
    int         screen        = 1;              // 0:none, 1:results, 2:all

    std::string highLevelSolver = "EES";        // "A*", "A*eps", "EES", "NEW"
    bool        lowLevelSolver  = true;         // ECBS (true)
    std::string inadmissibleH   = "Global";     // "Zero", "Global", "Path", "Local", "Conflict"
    double      suboptimality   = 1.2;

    std::string heuristics            = "WDG";  // "Zero", "CG", "DG", "WDG"
    bool        rectangleReasoning    = true;
    bool        corridorReasoning     = true;
    bool        bypass                = true;
    bool        prioritizingConflicts = true;
    bool        targetReasoning       = true;
    bool        mutexReasoning        = false;
    bool        disjointSplitting     = false;
    int         restarts      = 1;
    bool        sipp                  = false;  // use SIPP low-level if compiled

    double      step_dt_s     = 1.0;
};

inline void applyCommonSettings(const PlannerSettings& s, EECBS_CommonConfig& cfg) {
    if (auto v = s.get<int>("seed"))                        cfg.seed          = *v;
    if (auto v = s.get<int>("screen"))                      cfg.screen        = *v;

    if (auto v = s.get<std::string>("highLevelSolver"))   cfg.highLevelSolver = *v;
    if (auto v = s.get<bool>("lowLevelSolver"))            cfg.lowLevelSolver  = *v;
    if (auto v = s.get<std::string>("inadmissibleH"))     cfg.inadmissibleH   = *v;
    if (auto v = s.get<double>("suboptimality"))         cfg.suboptimality   = *v;

    if (auto v = s.get<std::string>("heuristics"))        cfg.heuristics = *v;
    if (auto v = s.get<bool>("rectangleReasoning"))        cfg.rectangleReasoning = *v;
    if (auto v = s.get<bool>("corridorReasoning"))         cfg.corridorReasoning = *v;
    if (auto v = s.get<bool>("bypass"))                    cfg.bypass = *v;
    if (auto v = s.get<bool>("prioritizingConflicts"))     cfg.prioritizingConflicts = *v;
    if (auto v = s.get<bool>("targetReasoning"))           cfg.targetReasoning = *v;
    if (auto v = s.get<bool>("mutexReasoning"))            cfg.mutexReasoning = *v;
    if (auto v = s.get<bool>("disjointSplitting"))         cfg.disjointSplitting = *v;
    if (auto v = s.get<int>("restarts"))                    cfg.restarts = *v;
    if (auto v = s.get<bool>("sipp"))                      cfg.sipp = *v;

    if (auto v = s.get<double>("step_dt_s"))             cfg.step_dt_s = *v;
}