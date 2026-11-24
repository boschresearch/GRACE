// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <optional>
#include <string>

struct MAPFLNS2_CommonConfig {
    int     seed       = 0;
    int     screen     = 1;      // 0:none; 1:LNS results; 2:LNS detailed; 3:MAPF detailed
    bool    sipp       = true;   // use SIPP for single-agent where supported

    // LNS params (naming follows upstream driver.cpp exactly)
    bool        initLNS               = true;
    int         neighborSize          = 8;
    int         maxIterations         = 0;
    std::string initAlgo              = "PP";       // EECBS, PP, PPS, CBS, PIBT, winPIBT
    std::string replanAlgo            = "PP";       // EECBS, CBS, PP
    std::string destoryStrategy       = "Adaptive"; // Random, RandomWalk, Intersection, Adaptive
    int         pibtWindow            = 5;
    bool        winPibtSoftmode       = true;
    std::string initDestoryStrategy   = "Adaptive"; // Target, Collision, Random, Adaptive

    double  step_dt_s  = 1.0;
};

inline void applyCommonSettings(const PlannerSettings& s, MAPFLNS2_CommonConfig& cfg) {
    if (auto v = s.get<int>("seed"))                          cfg.seed = *v;
    if (auto v = s.get<int>("screen"))                        cfg.screen = *v;
    if (auto v = s.get<bool>("sipp"))                        cfg.sipp = *v;

    if (auto v = s.get<bool>("initLNS"))                     cfg.initLNS = *v;
    if (auto v = s.get<int>("neighborSize"))                  cfg.neighborSize = *v;
    if (auto v = s.get<int>("maxIterations"))                 cfg.maxIterations = *v;
    if (auto v = s.get<std::string>("initAlgo"))            cfg.initAlgo = *v;
    if (auto v = s.get<std::string>("replanAlgo"))          cfg.replanAlgo = *v;
    if (auto v = s.get<std::string>("destoryStrategy"))     cfg.destoryStrategy = *v;
    if (auto v = s.get<int>("pibtWindow"))                    cfg.pibtWindow = *v;
    if (auto v = s.get<bool>("winPibtSoftmode"))             cfg.winPibtSoftmode = *v;
    if (auto v = s.get<std::string>("initDestoryStrategy")) cfg.initDestoryStrategy = *v;

    if (auto v = s.get<double>("step_dt_s"))               cfg.step_dt_s = *v;
}
