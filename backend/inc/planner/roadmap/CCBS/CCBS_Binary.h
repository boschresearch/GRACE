// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include <optional>

#include "../../Planner.h"
#include "../../../env/ICapabilities.h"
#include "../../../util/Logger.h"

#ifndef CCBS_DRIVER_PATH
#define CCBS_DRIVER_PATH "ccbs_driver"
#endif

struct CCBS_BinaryConfig {
    std::string binary = CCBS_DRIVER_PATH; // external driver path

    // Algorithm toggles (match external XML defaults)
    bool   use_cardinal           = true;
    bool   use_disjoint_splitting = true;
    int    hlh_type               = 2;         // 0,1,2
    int    connectedness          = 3;         // ignored for roadmap
    double focal_weight           = 1.0;

    // Platform-provided
    double agent_size             = 0.353553;  // radius
    double timelimit_s            = 60.0;      // seconds

    // Numeric
    double precision              = 1e-7;

    bool   keepTemps              = false;
};

class CCBS_Binary final : public IPlanner {
public:
    explicit CCBS_Binary(CCBS_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}
    ~CCBS_Binary() override = default;

    const char* name() const noexcept override { return "CCBS (bin)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::RoadmapEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    CCBS_BinaryConfig m_cfg;
    static void applySettings(const PlannerSettings& s, CCBS_BinaryConfig& cfg);

    struct RunResult { bool ok{false}; bool timed_out{false}; int exit_code{0}; };

    static bool writeGraphML(const std::filesystem::path& path, const IRoadmap& R);
    static bool writeTaskXml(const std::filesystem::path& path, const IRoadmap& R,
                             const std::vector<StartGoal>& sgs);
    static bool writeConfigXml(const std::filesystem::path& path, const CCBS_BinaryConfig& c);

    RunResult runSolver(const std::filesystem::path& workdir,
                        const std::filesystem::path& mapPath,
                        const std::filesystem::path& taskPath,
                        double time_limit_s) const;

    static std::vector<Plan> parseLogToPlans(const std::filesystem::path& taskXmlPath);
};
