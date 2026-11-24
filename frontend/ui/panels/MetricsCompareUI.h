// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <string>
#include <imgui.h>
#include "util/Metrics.h"

namespace ui {

    /// Show a side-by-side comparison table for 3 representations (continuous, roadmap, grid).
    /// Any pointer may be null (column will show “—”).
    void DrawMetricsComparisonTable(const metrics::Aggregate* cont, const char* contLabel,
                                    const metrics::Aggregate* road, const char* roadLabel,
                                    const metrics::Aggregate* grid, const char* gridLabel,
                                    const char* table_id = "CrossRepMetrics");

} // namespace ui
