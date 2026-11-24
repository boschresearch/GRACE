// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <limits>
#include <box2d/box2d.h>
#include "../planner/Planner.h"

namespace metrics {

    // ---------- Per-agent & aggregate records ----------
    struct PerAgent {
        b2BodyId     id{};
        std::string  name;
        double       path_length_m{0.0};        // Euclidean polyline length
        double       completion_time_s{std::numeric_limits<double>::quiet_NaN()}; // last waypoint t if timed
        int          waypoints{0};
    };

    struct Aggregate {
        std::size_t              agents{0};
        bool                     plan_success{false};
        double                   sum_length_m{0.0};
        double                   sum_of_cost_s{std::numeric_limits<double>::quiet_NaN()}; // flowtime (sum of completion times)
        double                   makespan_s{std::numeric_limits<double>::quiet_NaN()};
        double                   avg_length_m{std::numeric_limits<double>::quiet_NaN()};
        double                   avg_completion_s{std::numeric_limits<double>::quiet_NaN()};
        double                   planner_runtime_s{std::numeric_limits<double>::quiet_NaN()};
        double                   sim_runtime_s{std::numeric_limits<double>::quiet_NaN()};
        std::vector<PerAgent>    per_agent;
    };

    // ---------- Helpers ----------
    double polylineLength(const std::vector<Waypoint>& wps);

    // Returns NaN if no valid completion times exist
    double lastTimeIfTimed(const std::vector<Waypoint>& wps);

    // ---------- Core: compute per-agent & aggregate ----------
    PerAgent perAgentFromPlan(const Plan& plan, b2BodyId id, std::string name);

    Aggregate summarize(const std::vector<PerAgent>& perAgent,
                        double planner_runtime_s = std::numeric_limits<double>::quiet_NaN(),
                        double sim_runtime_s     = std::numeric_limits<double>::quiet_NaN(),
                        bool   plan_success      = false);

} // namespace metrics
