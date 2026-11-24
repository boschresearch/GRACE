// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <optional>
#include <string>

struct CBSH2RTC_CommonConfig {
    int         nodeLimit               = INFINITY;
    int         seed                    = 0;
    int         screen                  = 1;          // 0:none, 1:results, 2:all

    std::string heuristics              = "WDG";      // Zero, CG, DG, WDG
    std::string rectangleReasoning      = "GR";       // None, R, RM, GR, Disjoint
    std::string corridorReasoning       = "GC";       // None, C, PC, STC, GC, Disjoint
    bool        bypass                  = true;
    bool        prioritizingConflicts   = true;
    bool        targetReasoning         = true;
    bool        mutexReasoning          = false;
    bool        disjointSplitting       = false;
    int         restarts                = 1;
    bool        sipp                    = false;      // use SIPP for low-level if compiled

    double      step_dt_s               = 1.0;
};

inline void applyCommonSettings(const PlannerSettings& s, CBSH2RTC_CommonConfig& cfg) {
    if (auto v = s.get<int>("nodeLimit"))                      cfg.nodeLimit = *v;
    if (auto v = s.get<int>("seed"))                           cfg.seed = *v;
    if (auto v = s.get<int>("screen"))                         cfg.screen = *v;

    if (auto v = s.get<std::string>("heuristics"))           cfg.heuristics = *v;
    if (auto v = s.get<std::string>("rectangleReasoning"))   cfg.rectangleReasoning = *v;
    if (auto v = s.get<std::string>("corridorReasoning"))    cfg.corridorReasoning = *v;
    if (auto v = s.get<bool>("bypass"))                       cfg.bypass = *v;
    if (auto v = s.get<bool>("prioritizingConflicts"))        cfg.prioritizingConflicts = *v;
    if (auto v = s.get<bool>("targetReasoning"))              cfg.targetReasoning = *v;
    if (auto v = s.get<bool>("mutexReasoning"))               cfg.mutexReasoning = *v;
    if (auto v = s.get<bool>("disjointSplitting"))            cfg.disjointSplitting = *v;
    if (auto v = s.get<int>("restarts"))                       cfg.restarts = *v;
    if (auto v = s.get<bool>("sipp"))                         cfg.sipp = *v;

    if (auto v = s.get<double>("step_dt_s"))               cfg.step_dt_s = *v;
}
