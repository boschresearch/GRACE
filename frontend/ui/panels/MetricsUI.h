// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <imgui.h>
#include <cmath>
#include "util/Metrics.h"

namespace ui {

    /// Summary table (aggregate metrics). Call with the result of computePlanMetrics().
    void DrawPlanMetricsTable(const metrics::Aggregate& M,
                              const char* table_id = "PlanMetricsSummary");

    /// Per-agent table (ID, Name, Waypoints, Path length, Completion time).
    /// Pass a positive height to make it scrollable; negative lets ImGui size naturally.
    void DrawPerAgentMetricsTable(const metrics::Aggregate& M,
                                  float height = -1.0f,
                                  const char* table_id = "PerAgentMetrics");

} // namespace ui
