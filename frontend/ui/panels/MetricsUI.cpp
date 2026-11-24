// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "MetricsUI.h"
#include <algorithm>
#include <string>
#include <limits>
#include <imgui.h>
#include "box2d/box2d.h"

namespace ui {

static inline void RowKV(const char* k, const char* fmt, double v, const char* unit,
                         bool showEvenIfNaN = false)
{
    if (!showEvenIfNaN && !std::isfinite(v)) return;

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(k);
    ImGui::TableSetColumnIndex(1);
    if (std::isfinite(v)) ImGui::Text(fmt, v);
    else                  ImGui::TextDisabled("—");
    ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(unit ? unit : "");
}

void DrawPlanMetricsTable(const metrics::Aggregate& M, const char* table_id)
{
    const ImGuiTableFlags flags =
        ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuter |
        ImGuiTableFlags_RowBg    | ImGuiTableFlags_SizingStretchProp |
        ImGuiTableFlags_Resizable;

    if (ImGui::BeginTable(table_id, 3, flags)) {
        ImGui::TableSetupColumn("Metric");
        ImGui::TableSetupColumn("Value");
        ImGui::TableSetupColumn("Unit");
        ImGui::TableHeadersRow();

        // Agents (count) and status
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Agents");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%zu", M.agents);
        ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted("count");

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Plan status");
        ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(M.plan_success ? "success" : "failed");
        ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted("");

        // Runtimes
        RowKV("Planner runtime", "%.3f", M.planner_runtime_s, "s");
        RowKV("Last sim runtime", "%.3f", M.sim_runtime_s,     "s");

        // Time-based QoS
        RowKV("Makespan", "%.3f", M.makespan_s,       "s");
        RowKV("Flowtime (sum of completion)", "%.3f", M.sum_of_cost_s, "s");
        RowKV("Avg completion time", "%.3f", M.avg_completion_s, "s");

        // Lengths (always show sum, even if zero)
        RowKV("Sum path length", "%.3f", M.sum_length_m, "m", /*showEvenIfNaN=*/true);
        RowKV("Avg path length", "%.3f", M.avg_length_m, "m");

        ImGui::EndTable();
    }
}

void DrawPerAgentMetricsTable(const metrics::Aggregate& M, float height, const char* table_id)
{
    // For long lists, make it scrollable by giving a height.
    const ImVec2 outerSize(0.0f, height);

    const ImGuiTableFlags flags =
        ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuter |
        ImGuiTableFlags_RowBg    | ImGuiTableFlags_SizingStretchProp |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;

    if (ImGui::BeginTable(table_id, 5, flags, outerSize)) {
        ImGui::TableSetupScrollFreeze(0, 1); // freeze header row
        ImGui::TableSetupColumn("ID");
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("Waypoints");
        ImGui::TableSetupColumn("Path length (m)");
        ImGui::TableSetupColumn("Completion (s)");
        ImGui::TableHeadersRow();

        for (const auto& a : M.per_agent) {
            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            // Show "index1/gen" for clarity, consistent with your inspector
            ImGui::Text("%u/%u", a.id.index1, a.id.generation);

            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(a.name.c_str());

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%d", a.waypoints);

            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f", a.path_length_m);

            ImGui::TableSetColumnIndex(4);
            if (std::isfinite(a.completion_time_s)) ImGui::Text("%.3f", a.completion_time_s);
            else                                    ImGui::TextDisabled("—");
        }

        ImGui::EndTable();
    }
}

} // namespace ui
