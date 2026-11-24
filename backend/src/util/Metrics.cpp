// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/util/Metrics.h"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace metrics {

static inline bool isNan(double v) { return std::isnan(v); }

double polylineLength(const std::vector<Waypoint>& wps) {
    if (wps.size() < 2) return 0.0;
    double L = 0.0;
    for (std::size_t i = 1; i < wps.size(); ++i) {
        const double dx = static_cast<double>(wps[i].p.x - wps[i-1].p.x);
        const double dy = static_cast<double>(wps[i].p.y - wps[i-1].p.y);
        L += std::hypot(dx, dy);
    }
    return L;
}

double lastTimeIfTimed(const std::vector<Waypoint>& wps) {
    if (wps.empty()) return std::numeric_limits<double>::quiet_NaN();
    const float t = wps.back().t;
    if (t >= 0.0f) return static_cast<double>(t);
    return std::numeric_limits<double>::quiet_NaN();
}

PerAgent perAgentFromPlan(const Plan& plan, b2BodyId id, std::string name) {
    PerAgent pa;
    pa.id              = id;
    pa.name            = std::move(name);
    pa.waypoints       = static_cast<int>(plan.waypoints.size());
    pa.path_length_m   = polylineLength(plan.waypoints);
    pa.completion_time_s = lastTimeIfTimed(plan.waypoints);
    return pa;
}

static double sumIgnoreNaN(const std::vector<double>& xs) {
    double s = 0.0;
    bool any = false;
    for (double v : xs) { if (!isNan(v)) { s += v; any = true; } }
    return any ? s : std::numeric_limits<double>::quiet_NaN();
}
static double maxIgnoreNaN(const std::vector<double>& xs) {
    double m = -std::numeric_limits<double>::infinity();
    bool any = false;
    for (double v : xs) { if (!isNan(v)) { m = std::max(m, v); any = true; } }
    return any ? m : std::numeric_limits<double>::quiet_NaN();
}
static double avgIgnoreNaN(const std::vector<double>& xs) {
    double s = 0.0; int c = 0;
    for (double v : xs) { if (!isNan(v)) { s += v; ++c; } }
    return c > 0 ? s / static_cast<double>(c) : std::numeric_limits<double>::quiet_NaN();
}

Aggregate summarize(const std::vector<PerAgent>& perAgent,
                    double planner_runtime_s,
                    double sim_runtime_s,
                    bool   plan_success) {
    Aggregate agg;
    agg.per_agent = perAgent;
    agg.agents    = perAgent.size();
    agg.plan_success = plan_success;

    // lengths
    agg.sum_length_m  = std::accumulate(perAgent.begin(), perAgent.end(), 0.0,
        [](double s, const PerAgent& a){ return s + a.path_length_m; });
    agg.avg_length_m  = (agg.agents > 0) ? (agg.sum_length_m / static_cast<double>(agg.agents))
                                         : std::numeric_limits<double>::quiet_NaN();

    // completion-time based metrics (flowtime / makespan)
    std::vector<double> T;
    T.reserve(perAgent.size());
    for (const auto& a : perAgent) T.push_back(a.completion_time_s);

    agg.sum_of_cost_s   = sumIgnoreNaN(T);    // flowtime
    agg.makespan_s      = maxIgnoreNaN(T);
    agg.avg_completion_s= avgIgnoreNaN(T);

    agg.planner_runtime_s = planner_runtime_s;
    agg.sim_runtime_s     = sim_runtime_s;
    return agg;
}

} // namespace metrics
