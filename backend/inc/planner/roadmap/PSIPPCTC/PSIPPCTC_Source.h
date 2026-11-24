// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <filesystem>

#include "../../Planner.h"
#include "../../../env/ICapabilities.h"

// Upstream headers (PSIPP + problem types)
#include <planners/PrioritizedSIPP.hpp>
#include <base/problem_with_explicit_collisions.hpp>
#include <base/problem_with_geometry.hpp>
#include <base/problem.hpp>
#include <base/geometry_2D.hpp>

struct PSIPPCTC_SourceConfig {
    double timelimit_s       = 60.0;
    int    priority_strategy = 0;     // 0 deterministic; 1 randomize (if supported upstream)

    // If set, cap #agents
    std::optional<int> takeK;
};

class PSIPPCTC_Source final : public IPlanner {
public:
    explicit PSIPPCTC_Source(PSIPPCTC_SourceConfig cfg) : m_cfg(std::move(cfg)) {}
    ~PSIPPCTC_Source() override = default;

    const char* name() const noexcept override { return "PSIPP-CTC (src)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::RoadmapEnvironment; }
    void configure(const PlannerSettings& s) override;
    PlanResult plan(const PlanRequest& req) override;

private:
    PSIPPCTC_SourceConfig m_cfg;
    std::optional<double> m_agent_radius;

    static std::vector<std::pair<int,int>>
    nodePairsFromMeters(const IRoadmap& R,
                        const std::vector<StartGoal>& sgs);

    static bool convertPSIPPPlanToPlatform(
        ProblemInstance& P,
        const ProblemInstance::Plan& planIn,
        std::vector<Plan>& outPlans);
};
