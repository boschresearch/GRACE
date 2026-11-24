// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <cstdint>
#include "../env/BaseEnvironment.h"
#include "../robot/RobotSystem.h"
#include "../util/Metrics.h"

namespace grace::sim {

    struct SimStepContext {
        float simTime;                   // current simulation time (s) *after* this step
        float dt;                        // step duration (s)
        std::uint64_t stepCount;         // 1-based counter after this step
        const BaseEnvironment& env;
        const RobotSystem&   robots;
    };

    struct SimRunSummary {
        metrics::Aggregate metrics;      // computed by Simulation::computePlanMetrics()
        double plannerRuntimeSec{0.0};
        double simRuntimeSec{0.0};
        bool   planSucceeded{false};
    };

    struct ISimObserver {
        virtual ~ISimObserver() = default;
        virtual void onRunStart(const BaseEnvironment&, const RobotSystem&) {}
        virtual void onPostStep(const SimStepContext&) {}
        virtual void onRunEnd(const SimRunSummary&) {}
    };

} // namespace grace::sim
