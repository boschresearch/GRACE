// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "LACAM0_Common.h"
#include "../../Planner.h"


struct IDiscreteGrid;

struct LACAM0_BinaryConfig: LACAM0_CommonConfig {
    std::string binary = LACAM0_DRIVER_PATH;
    bool keepTemps = false;
};

class LACAM0_Binary final : public IPlanner {
public:
    explicit LACAM0_Binary(LACAM0_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}

    const char* name() const noexcept override { return "LaCAM0 (bin)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }

    PlanResult plan(const PlanRequest& req) override;
    void configure(const PlannerSettings& s) override;

private:
    LACAM0_BinaryConfig m_cfg;
    static void applySettings(const PlannerSettings& s, LACAM0_BinaryConfig& cfg);

    struct RunResult { bool ok{false}; bool timed_out{false}; int exit_code{0}; };

    RunResult runSolver(const std::filesystem::path& workdir,
                        const std::filesystem::path& mapPath,
                        const std::filesystem::path& scenPath,
                        const std::filesystem::path& outPath,
                        int k,
                        double time_limit_s,
                        int seed,
                        int verbose) const;

    // Parse LaCAM result file ("solution=" then lines t:(c,r),...) → Plans
    static std::vector<Plan> parseLaCAMPaths(const IDiscreteGrid& grid,
                                             const std::vector<StartGoal>& sgs,
                                             const std::filesystem::path& outPath,
                                             double step_dt_s);
};
