// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>

#include "../../../../extern/classical/EECBS/inc/common.h"
#include "../../Planner.h"
#include "EECBS_Common.h"

struct IDiscreteGrid;

struct EECBS_SourceConfig: EECBS_CommonConfig {};

class EECBS_Source final : public IPlanner {
public:
    explicit EECBS_Source(EECBS_SourceConfig cfg) : m_cfg(std::move(cfg)) {}
    ~EECBS_Source() override = default;

    const char* name() const noexcept override { return "EECBS (src)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }

    PlanResult plan(const PlanRequest& req) override;
    void configure(const PlannerSettings& s) override;

private:
    EECBS_SourceConfig m_cfg;

    static void applySettings(const PlannerSettings& s, EECBS_SourceConfig& cfg);

    static struct Instance makeInstanceFromGrid(const IDiscreteGrid& grid, const std::vector<StartGoal>& sgs);

    static std::vector<Plan> pathsToPlans(const IDiscreteGrid& grid,
        const std::vector<std::vector<PathEntry>>& paths,
        double step_dt_s);
};
