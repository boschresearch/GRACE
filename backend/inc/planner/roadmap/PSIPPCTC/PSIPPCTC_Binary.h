// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "../../Planner.h"
#include "../../../env/ICapabilities.h"

#ifndef PSIPPCTC_DRIVER_PATH
#  define PSIPPCTC_DRIVER_PATH "psippctc"
#endif

struct PSIPPCTC_BinaryConfig {
    std::string binary = PSIPPCTC_DRIVER_PATH;

    // Planner knobs
    double timelimit_s       = 60.0;
    int    priority_strategy = 0;   // 0: fixed 0..N-1; 1: shuffled (if supported upstream)

    // IO / dev
    bool keepTemps = false;

    // If set, cap #agents (otherwise plan all)
    std::optional<int> takeK;
};

class PSIPPCTC_Binary final : public IPlanner {
public:
    explicit PSIPPCTC_Binary(PSIPPCTC_BinaryConfig cfg) : m_cfg(std::move(cfg)) {}
    ~PSIPPCTC_Binary() override = default;

    const char* name() const noexcept override { return "PSIPP-CTC (bin)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::RoadmapEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    PSIPPCTC_BinaryConfig m_cfg;
    std::optional<double> m_agent_radius; // optional override via settings

    // Helpers
    static void writeRoadmapProblemTxt(const IRoadmap& R,
                                       const std::vector<std::pair<int,int>>& tasks,
                                       double agent_radius,
                                       const std::filesystem::path& outTxt);

    static bool parsePlannerStdoutPlan(const std::filesystem::path& planTxt,
                                       std::vector<Plan>& outPlans);

    static std::vector<std::pair<int,int>>
    nodePairsFromMeters(const IRoadmap& R,
                        const std::vector<StartGoal>& sgs);

    struct RunResult { bool ok{false}; bool timed_out{false}; int exit_code{0}; };
    RunResult run(const std::filesystem::path& workdir,
                  double timelimit_s,
                  const std::filesystem::path& problemTxt,
                  const std::filesystem::path& stdoutTxt,
                  const std::filesystem::path& stderrTxt) const;
};
