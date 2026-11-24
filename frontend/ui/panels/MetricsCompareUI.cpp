// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "MetricsCompareUI.h"
#include <cmath>

namespace ui {

static inline void CellNumber(double v, const char* fmt = "%.3f")
{
    if (std::isfinite(v)) ImGui::Text(fmt, v);
    else                  ImGui::TextDisabled("—");
}

static inline void CellText(const char* s)
{
    if (s && *s) ImGui::TextUnformatted(s);
    else         ImGui::TextDisabled("—");
}

static inline void CellBool(bool ok, const char* yes = "success", const char* no = "failed")
{
    ImGui::TextUnformatted(ok ? yes : no);
}

static void RowHeader(const char* label)
{
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted(label);
}

void DrawMetricsComparisonTable(const metrics::Aggregate* cont, const char* lC,
                                const metrics::Aggregate* road, const char* lR,
                                const metrics::Aggregate* grid, const char* lG,
                                const char* table_id)
{
    const ImGuiTableFlags flags =
        ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuter
      | ImGuiTableFlags_RowBg    | ImGuiTableFlags_SizingStretchProp
      | ImGuiTableFlags_Resizable;

    if (ImGui::BeginTable(table_id, 4, flags)) {
        ImGui::TableSetupColumn("Metric");
        ImGui::TableSetupColumn(lC ? lC : "Continuous");
        ImGui::TableSetupColumn(lR ? lR : "Roadmap");
        ImGui::TableSetupColumn(lG ? lG : "Grid");
        ImGui::TableHeadersRow();

        auto col = [&](int i){ ImGui::TableSetColumnIndex(i); };

        // Agents
        RowHeader("Agents (#)");
        col(1); ImGui::Text("%zu", cont ? cont->agents : 0u);
        col(2); ImGui::Text("%zu", road ? road->agents : 0u);
        col(3); ImGui::Text("%zu", grid ? grid->agents : 0u);

        // Plan status
        RowHeader("Plan status");
        col(1); if (cont) CellBool(cont->plan_success); else CellText("");
        col(2); if (road) CellBool(road->plan_success); else CellText("");
        col(3); if (grid) CellBool(grid->plan_success); else CellText("");

        // Runtimes
        RowHeader("Planner runtime (s)");
        col(1); if (cont) CellNumber(cont->planner_runtime_s); else CellText("");
        col(2); if (road) CellNumber(road->planner_runtime_s); else CellText("");
        col(3); if (grid) CellNumber(grid->planner_runtime_s); else CellText("");

        RowHeader("Last sim runtime (s)");
        col(1); if (cont) CellNumber(cont->sim_runtime_s); else CellText("");
        col(2); if (road) CellNumber(road->sim_runtime_s); else CellText("");
        col(3); if (grid) CellNumber(grid->sim_runtime_s); else CellText("");

        // Time-based QoS
        RowHeader("Makespan (s)");
        col(1); if (cont) CellNumber(cont->makespan_s ); else CellText("");
        col(2); if (road) CellNumber(road->makespan_s); else CellText("");
        col(3); if (grid) CellNumber(grid->makespan_s); else CellText("");

        RowHeader("Flowtime / sum of completion (s)");
        col(1); if (cont) CellNumber(cont->sum_of_cost_s); else CellText("");
        col(2); if (road) CellNumber(road->sum_of_cost_s); else CellText("");
        col(3); if (grid) CellNumber(grid->sum_of_cost_s); else CellText("");

        RowHeader("Avg completion time (s)");
        col(1); if (cont) CellNumber(cont->avg_completion_s); else CellText("");
        col(2); if (road) CellNumber(road->avg_completion_s); else CellText("");
        col(3); if (grid) CellNumber(grid->avg_completion_s); else CellText("");

        // Lengths
        RowHeader("Sum path length (m)");
        col(1); if (cont) CellNumber(cont->sum_length_m); else CellText("");
        col(2); if (road) CellNumber(road->sum_length_m); else CellText("");
        col(3); if (grid) CellNumber(grid->sum_length_m); else CellText("");

        RowHeader("Avg path length (m)");
        col(1); if (cont) CellNumber(cont->avg_length_m); else CellText("");
        col(2); if (road) CellNumber(road->avg_length_m); else CellText("");
        col(3); if (grid) CellNumber(grid->avg_length_m); else CellText("");

        ImGui::EndTable();
    }
}

} // namespace ui
