// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>

#include "../../Planner.h"
#include "MAPFLNS2_Common.h"

struct IDiscreteGrid;

struct MAPFLNS2_SourceConfig: MAPFLNS2_CommonConfig {};

class MAPFLNS2_Source final : public IPlanner {
public:
    explicit MAPFLNS2_Source(MAPFLNS2_SourceConfig cfg) : m_cfg(std::move(cfg)) {}
    ~MAPFLNS2_Source() override = default;

    [[nodiscard]] const char* name() const noexcept override { return "MAPF-LNS2 (src)"; }
    [[nodiscard]] EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }

    PlanResult plan(const PlanRequest& req) override;
    void configure(const PlannerSettings& s) override { applySettings(s, m_cfg); }

private:
    MAPFLNS2_SourceConfig m_cfg;

    static void applySettings(const PlannerSettings& s, MAPFLNS2_SourceConfig& cfg);

    static struct Instance makeInstanceFromGrid(const IDiscreteGrid& grid, const std::vector<StartGoal>& sgs);

    static std::vector<Plan> pathsToPlans(const IDiscreteGrid& grid,
                 const std::vector<std::vector<struct PathEntry>>& paths,
                 double step_dt_s);
};
