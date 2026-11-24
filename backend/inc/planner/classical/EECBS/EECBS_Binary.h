// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "../../Planner.h"
#include "EECBS_Common.h"

struct IDiscreteGrid;

struct EECBS_BinaryConfig: EECBS_CommonConfig {
    std::string binary = EECBS_DRIVER_PATH;
    bool        keepTemps   = false;
};

class EECBS_Binary final : public IPlanner {
public:
    explicit EECBS_Binary(EECBS_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}

    const char* name() const noexcept override { return "EECBS (bin)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }

    PlanResult plan(const PlanRequest& req) override;
    void configure(const PlannerSettings& s) override { applySettings(s, m_cfg); }

private:
    EECBS_BinaryConfig m_cfg;

    static void applySettings(const PlannerSettings& s, EECBS_BinaryConfig& cfg);

    RunResult runSolver(const std::filesystem::path& dir,
                        const std::filesystem::path& mapPath,
                        const std::filesystem::path& scenPath,
                        const std::filesystem::path& pathsOut,
                        int k,
                        double time_limit_s) const;

    // Parse “paths.txt” produced by driver (Agent i: ... or raw pairs)
    static std::vector<Plan> parsePathsFile(const IDiscreteGrid& grid,
                                            int K,
                                            const std::filesystem::path& file);
};
