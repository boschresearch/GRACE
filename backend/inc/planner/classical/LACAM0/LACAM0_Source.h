// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "../../Planner.h"
#include "LACAM0_Common.h"

struct LACAM0_SourceConfig: LACAM0_CommonConfig {};

class LACAM0_Source final : public IPlanner {
public:
    explicit LACAM0_Source(LACAM0_SourceConfig cfg) : m_cfg(std::move(cfg)) {}
    ~LACAM0_Source() override = default;

    const char* name() const noexcept override { return "LaCAM0 (src)"; }
    EnvironmentType type() const noexcept override { return EnvironmentType::GridEnvironment; }

    PlanResult plan(const PlanRequest& req) override;
    void configure(const PlannerSettings& s) override;

private:
    LACAM0_SourceConfig m_cfg;
    static void applySettings(const PlannerSettings& s, LACAM0_SourceConfig& cfg);
};
