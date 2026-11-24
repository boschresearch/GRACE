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

struct DBCBS_BinaryConfig {
    std::string binary = DBCBS_DRIVER_PATH;

    // Execution control
    double timelimit_s = 300.0;
    bool   keepTemps   = false;

    // Planner knobs (mapped to cfg.yaml)
    double      alpha               = 0.5;
    bool        filter_duplicates   = true;
    std::string heuristic1          = "reverse-search"; // or "none"
    double      heuristic1_delta    = 1.0;
    double      delta0              = 0.5;
    int         num_primitives0     = 1000;
    double      delta_rate          = 0.9;
    double      num_primitives_rate = 1.5;

    // Platform-side interpretation
    std::string robot_type = "unicycle_first_order_0";
    double      step_dt_s  = 0.1;  // output parsing dt

    // Cap #agents if requested
    std::optional<int> takeK;
};

class DBCBS_Binary final : public IPlanner {
public:
    explicit DBCBS_Binary(DBCBS_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}
    ~DBCBS_Binary() override = default;

    const char* name() const noexcept override { return "DBCBS (bin)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::ContinuousEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    DBCBS_BinaryConfig m_cfg;

    std::optional<double>              m_agent_radius;
    std::vector<ContinuousRobotMeta>   m_robot_meta;   // same order as sgs
    std::vector<DBCBSObstacle>         m_env_obs;      // typed obstacles [0..W]x[0..H]

    static void writeCfgYaml(const DBCBS_BinaryConfig& cfg,
                             const std::filesystem::path& outYaml);
};
