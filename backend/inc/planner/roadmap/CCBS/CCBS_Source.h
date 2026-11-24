// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>
#include <string>
#include <utility>

#include "../../Planner.h"

struct IRoadmap;

struct CCBS_SourceConfig {
    bool   use_cardinal           = true;
    bool   use_disjoint_splitting = true;
    int    hlh_type               = 2;
    int    connectedness          = 3;       // ignored by roadmap
    double focal_weight           = 1.0;

    double agent_size             = 0.353553; // radius
    double timelimit_s            = 60.0;     // seconds
    double precision              = 1e-7;
};

class CCBS_Source final : public IPlanner {
public:
    explicit CCBS_Source(CCBS_SourceConfig cfg) : m_cfg(std::move(cfg)) {}
    ~CCBS_Source() override = default;

    const char* name() const noexcept override { return "CCBS (src)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::RoadmapEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    CCBS_SourceConfig m_cfg;

    static int findNodeIdByPos(const IRoadmap& R, b2Vec2 p, float tol=1e-5f);
    static void collectRoadmap(const IRoadmap& R,
                               std::vector<std::pair<double,double>>& coords,
                               std::vector<std::vector<int>>& outAdjacency);
};
