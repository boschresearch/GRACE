// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "../../Planner.h"
#include "MAPFLNS2_Common.h"

struct IDiscreteGrid;

struct MAPFLNS2_BinaryConfig: MAPFLNS2_CommonConfig {
    std::string binary = MAPFLNS2_DRIVER_PATH;
    bool        keepTemps = false;
};

class MAPFLNS2_Binary final : public IPlanner {
public:
    explicit MAPFLNS2_Binary(MAPFLNS2_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}

    [[nodiscard]] const char* name() const noexcept override { return "MAPF-LNS2 (bin)"; }
    [[nodiscard]] EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    MAPFLNS2_BinaryConfig m_cfg;
    static void applySettings(const PlannerSettings& s, MAPFLNS2_BinaryConfig& cfg);

    [[nodiscard]] RunResult runSolver(const std::filesystem::path& workdir,
                        const std::filesystem::path& mapPath,
                        const std::filesystem::path& scenPath,
                        const std::filesystem::path& pathsOut,
                        int k,
                        double time_limit_s) const;

    static std::vector<Plan> parsePathsFile(const IDiscreteGrid& grid, int K, const std::filesystem::path& file);
};
