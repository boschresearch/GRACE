// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "../../Planner.h"
#include "../../../env/BaseEnvironment.h"
#include "../DBCBS/DBCBS_Common.h"

// forward-declare external lib API (header available in external repo)
namespace dbcbs_sst {
    struct ProblemSpec;
    struct SSTConfig;
    struct SSTResult;
    bool sst_solve(const ProblemSpec&, const SSTConfig&, SSTResult&, std::function<void(double,double)> cb);
}

struct SST_SourceConfig {
    // Same knobs as binary
    std::string planner = "sst";
    double timelimit_s  = 300.0;

    double propagation_step_size = 0.1;
    int    control_duration_min  = 1;
    int    control_duration_max  = 10;

    double goal_epsilon          = 0.2;
    double goal_bias             = 0.05;

    double selection_radius      = 0.2;
    double pruning_radius        = 0.1;

    std::string robot_type       = "unicycle_first_order_0";
    double      step_dt_s        = 0.1;

    std::optional<int> takeK;
};

class SST_Source final : public IPlanner {
public:
    explicit SST_Source(SST_SourceConfig cfg) : m_cfg(std::move(cfg)) {}
    ~SST_Source() override = default;

    const char* name() const noexcept override { return "SST (src)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::ContinuousEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    SST_SourceConfig m_cfg;

    // mirrors binary meta ingestion (obstacles optional; kept for parity)
    std::optional<double> m_agent_radius;
    struct Box { double cx,cy,sx,sy; };
    struct Meta {
        std::string type;
        struct Part { double cx,cy,sx,sy,angle; };
        struct { std::string shape; double radius, size_x, size_y; std::vector<Part> boxes; bool valid; } fp;
    };
    std::vector<Meta> m_robot_meta;
    std::vector<DBCBSObstacle> m_env_obs;

    // builders
    void buildProblem(const BaseEnvironment& E,
                      const std::vector<StartGoal>& sgs,
                      dbcbs_sst::ProblemSpec& out) const;

    void buildConfig(dbcbs_sst::SSTConfig& out) const;
};
