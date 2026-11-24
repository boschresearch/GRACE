// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "../../Planner.h"
#include "../../../env/BaseEnvironment.h"
#include "DBCBS_Common.h"

struct DBCBS_SourceConfig {
    // Algorithm knobs (map 1:1 to external cfg)
    double      alpha               = 0.5;
    bool        filter_duplicates   = true;
    std::string heuristic1          = "reverse-search"; // or "none"
    double      heuristic1_delta    = 1.0;
    double      delta0              = 0.5;
    int         num_primitives0     = 1000;
    double      delta_rate          = 0.9;
    double      num_primitives_rate = 1.5;

    // Platform-side
    std::string robot_type  = "unicycle_first_order_0";
    double      step_dt_s   = 0.1;
    double      timelimit_s = 300.0;
    bool        keepTemps   = false;

    // Optimizer controls (if available in external lib)
    bool useOptimizer     = true; // try opt pipeline first
    bool optSumRobotCost  = true; // flowtime (true) vs makespan (false)

    std::optional<int> takeK;
};

class DBCBS_Source final : public IPlanner {
public:
    explicit DBCBS_Source(DBCBS_SourceConfig cfg) : m_cfg(std::move(cfg)) {}
    ~DBCBS_Source() override = default;

    const char* name() const noexcept override { return "DBCBS (src)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::ContinuousEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    DBCBS_SourceConfig m_cfg;

    std::optional<double>              m_agent_radius;
    std::vector<ContinuousRobotMeta>   m_robot_meta; // same order as sgs
    std::vector<DBCBSObstacle>         m_env_obs;    // typed obstacles
};
