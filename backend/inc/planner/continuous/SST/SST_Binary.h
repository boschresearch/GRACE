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

struct SST_RobotMeta {
    std::string type;
    struct Part { double cx{}, cy{}, sx{}, sy{}, angle{}; };
    struct {
        std::string shape;      // "circle" | "box" | "multibox"
        double radius{-1.0};
        double size_x{-1.0}, size_y{-1.0};
        std::vector<Part> boxes;
        bool valid{false};
    } fp;
};

struct SST_BinaryConfig {
    std::string binary = SST_DRIVER_PATH;

    // Execution control
    double timelimit_s = 300.0;
    bool   keepTemps   = false;

    // Planner selection
    std::string planner = "sst"; // "sst" | "rrt"

    // OMPL/SST params
    double propagation_step_size = 0.1;
    int    control_duration_min  = 1;   // multiples of step size
    int    control_duration_max  = 10;  // multiples of step size

    double goal_epsilon          = 0.2;
    double goal_bias             = 0.05;

    // SST only
    double selection_radius      = 0.2; // delta_BN
    double pruning_radius        = 0.1; // delta_s

    // Platform-side interpretation
    std::string robot_type       = "unicycle_first_order_0";
    double      step_dt_s        = 0.1;  // for plan sampling when parsing

    // Optional cap
    std::optional<int> takeK;
};

class SST_Binary final : public IPlanner {
public:
    explicit SST_Binary(SST_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}
    ~SST_Binary() override = default;

    const char* name() const noexcept override { return "SST (bin)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::ContinuousEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    SST_BinaryConfig m_cfg;

    // Per-instance “ingested” meta coming from configure()/request settings.
    std::optional<double>       m_agent_radius; // (reserved for future use)
    std::vector<SST_RobotMeta>  m_robot_meta;   // same order as sgs
    std::vector<DBCBSObstacle>  m_env_obs;      // typed obstacles [0..W]x[0..H]

    // Writers / parsers
    void writeProblemYaml(const BaseEnvironment& E,
                          const std::vector<StartGoal>& sgs,
                          const std::filesystem::path& outYaml);
    void writeCfgYaml(const SST_BinaryConfig& cfg,
                      const std::filesystem::path& outYaml) const;

    static bool parseSolutionYaml(const std::filesystem::path& outYaml,
                                  double step_dt_s,
                                  std::vector<Plan>& outPlans);
};
