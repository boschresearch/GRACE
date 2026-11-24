// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "../../Planner.h"
#include "../../../env/ICapabilities.h"

struct MAPFGPTDDG_BinaryConfig {
    std::string python_exe  = MAPFGPTDDG_PYTHON_EXE;   // wrapper or python3
    std::string driver      = MAPFGPTDDG_DRIVER_PATH;  // driver script/module

    // RL inference options
    std::string model       = "2M-DDG";                // "2M", "6M", "85M", "2M-DDG"
    std::string device      = "cuda";                  // "cuda", "cpu", "mps"
    int         max_steps   = 256;
    int         seed        = 42;
    double      cutoff_s    = 60.0;
    bool        save_svg    = false;
    std::optional<std::filesystem::path> svg_out     = "";

    bool        keepTemps   = false;
    double      step_dt_s   = 1.0;
};

class MAPFGPTDDG_Binary final : public IPlanner {
public:
    explicit MAPFGPTDDG_Binary(MAPFGPTDDG_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}

    [[nodiscard]] const char* name() const noexcept override { return "MAPF-GPT-DDG (bin)"; }
    [[nodiscard]] EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    MAPFGPTDDG_BinaryConfig m_cfg;

    static void applySettings(const PlannerSettings& s, MAPFGPTDDG_BinaryConfig& cfg);

    [[nodiscard]] RunResult runSolver(const std::filesystem::path& workdir,
                        const std::filesystem::path& gridTxt,
                        const std::filesystem::path& startsCsv,
                        const std::filesystem::path& goalsCsv,
                        const std::filesystem::path& outCsv,
                        int k,
                        double time_limit_s,
                        const MAPFGPTDDG_BinaryConfig& cfg) const;

    // Parse driver CSV → platform Plans (meters + t), with trimming
    static std::vector<Plan> parseCsvPaths(const IDiscreteGrid& grid,
        const std::vector<StartGoal>& sgs,
        const std::filesystem::path& outCsv,
        double step_dt_s);
};
