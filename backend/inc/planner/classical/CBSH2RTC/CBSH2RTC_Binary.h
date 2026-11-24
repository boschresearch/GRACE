// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "../../Planner.h"
#include "CBSH2RTC_Common.h"

struct IDiscreteGrid;

struct CBSH2RTC_BinaryConfig: CBSH2RTC_CommonConfig {
    std::string binary    = CBSH2RTC_DRIVER_PATH;
    bool        keepTemps = false;
};

class CBSH2RTC_Binary final : public IPlanner {
public:
    explicit CBSH2RTC_Binary(CBSH2RTC_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}

    [[nodiscard]] const char* name() const noexcept override { return "CBSH2RTC (bin)"; }
    [[nodiscard]] EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }

    PlanResult plan(const PlanRequest& req) override;
    void configure(const PlannerSettings& s) override { applySettings(s, m_cfg); }

private:
    CBSH2RTC_BinaryConfig m_cfg;
    static void applySettings(const PlannerSettings& s, CBSH2RTC_BinaryConfig& cfg);

    [[nodiscard]] RunResult runSolver(const std::filesystem::path& dir,
                        const std::filesystem::path& mapPath,
                        const std::filesystem::path& scenPath,
                        const std::filesystem::path& pathsOut,
                        int k,
                        double time_limit_s) const;

    // Parse typical CBSH2/RT “paths.txt” (Agent i: (x y) ...) into plans on a grid.
    static std::vector<Plan> parsePathsFile(const IDiscreteGrid& grid, int K, const std::filesystem::path& file);
};
