// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "../../Planner.h"

struct IDiscreteGrid;

struct SCRIMP_BinaryConfig {
    // Python entry
    std::string python_exe = SCRIMP_PYTHON_EXE; // python or wrapper
    std::string driver     = SCRIMP_DRIVER_PATH; // driver script/module

    // Model / runtime
    std::string checkpoint = "final/net_checkpoint.pkl";
    std::string device     = "cuda";  // "cuda", "cpu", "mps"
    int         max_steps  = 256;
    int         seed       = 42;
    // timestamp scale: seconds per discrete step from the CSV
    double      step_dt_s  = 1.0;

    // Integration
    double      cutoff_s   = 60.0;
    bool        keepTemps  = false;
};

class SCRIMP_Binary final : public IPlanner {
public:
    explicit SCRIMP_Binary(SCRIMP_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}

    const char* name() const noexcept override { return "SCRIMP (bin)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    SCRIMP_BinaryConfig m_cfg;
    static void applySettings(const PlannerSettings& s, SCRIMP_BinaryConfig& cfg);

    struct RunResult { bool ok{false}; bool timed_out{false}; int exit_code{0}; };

    RunResult runSolver(const std::filesystem::path& workdir,
                        const std::filesystem::path& gridTxt,
                        const std::filesystem::path& startsCsv,
                        const std::filesystem::path& goalsCsv,
                        const std::filesystem::path& outCsv,
                        int k,
                        double time_limit_s,
                        const SCRIMP_BinaryConfig& cfg) const;

    // Parse driver CSV → platform Plans (meters + t)
    static std::vector<Plan> parseCsvPaths(const IDiscreteGrid& grid,
                                           int numAgents,
                                           const std::filesystem::path& outCsv,
                                           const std::vector<std::pair<int,int>>& goalCell /*(r,c)*/);
};
