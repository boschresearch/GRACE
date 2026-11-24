// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>

#include "../../../../extern/classical/CBSH2-RTC/inc/common.h"
#include "../../Planner.h"
#include "CBSH2RTC_Common.h"

struct IDiscreteGrid;

struct CBSH2RTC_SourceConfig: CBSH2RTC_CommonConfig {};

// Direct, in-process CBSH2-RT planner (no external binary; no temp files).
class CBSH2RTC_Source final : public IPlanner {
public:
    explicit CBSH2RTC_Source(CBSH2RTC_SourceConfig cfg) : m_cfg(std::move(cfg)) {}
    ~CBSH2RTC_Source() override = default;

    const char* name() const noexcept override { return "CBSH2RTC (src)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }

    PlanResult plan(const PlanRequest& req) override;
    void configure(const PlannerSettings& s) override { applySettings(s, m_cfg); }

private:
    CBSH2RTC_SourceConfig m_cfg;

    static void applySettings(const PlannerSettings& s, CBSH2RTC_SourceConfig& cfg);

    static struct Instance makeInstanceFromGrid(const IDiscreteGrid& grid, const std::vector<StartGoal>& sgs);

    static std::vector<Plan> pathsToPlans(const IDiscreteGrid& grid,
                 const std::vector<std::vector<PathEntry>>& paths,
                 double step_dt_s);
};
