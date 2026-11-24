// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "PlatformPanels.h"
#include "../UiStyle.h"
#include "../../platform/PlannerCatalog.h"
#include "Simulation.h"
#include "../../sample/SampleContext.h"

#include <imgui.h>
#include <cmath>            // std::isfinite
#include <fmt/format.h>     // enrich status line with fmt
#include <filesystem>       // CSV paths

#include "io/MetricsCsvObserver.h"
#include "io/StateCsvObserver.h"

#include "MetricsUI.h"

namespace ui {

void DrawStepPlanner(PlatformBase::PlannerAccess access)
{
    access.ensureSimulation();
    auto planner = access.settings();
    auto studio = access.state();
    auto* simulation = access.simulation();

    if (!access.hasEnvironment()) {
        ImGui::TextDisabled("Load or create an environment first.");
        return;
    }
    const auto kind = access.isGrid() ? platform::EnvKind::Grid
                    : access.isRoadmap() ? platform::EnvKind::Roadmap
                                         : platform::EnvKind::Continuous;
    const auto& itemsVec = platform::PlannerCatalog::allowed(kind);

    ImGui::TextWrapped("Choose a planner compatible with this environment, adjust options, then plan.");
    ImGui::Separator();

    // Planner combo
    int currentIndex = 0;
    for (int i = 0; i < (int)itemsVec.size(); ++i) {
        if (planner.name == itemsVec[i]) {
            currentIndex = i;
            break;
        }
    }

    ImGui::SetNextItemWidth(260);
    if (ImGui::BeginCombo("Planner", planner.name.c_str())) {
        for (int i = 0; i < (int)itemsVec.size(); ++i) {
            const bool selected = (i == currentIndex);
            if (ImGui::Selectable(itemsVec[i].c_str(), selected)) {
                access.reconfigurePlanner(itemsVec[i]);
                planner.name = itemsVec[i];
                currentIndex = i;
            }
            if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    // Time limit (no immediate replan, but options are installed)
    ImGui::SetNextItemWidth(160);
    if (ImGui::InputDouble(
            "Time limit (s)", &planner.timeLimitSeconds, 0.5, 5.0, "%.2f")) {
        access.setPlannerTimeLimit(planner.timeLimitSeconds);
    }

    ImGui::Spacing();
    // Plan-now button
    if (ui::PrimaryButton("Plan now")) {  // uses your themed primary button
        access.replanPreservingGoals();
        const float T = simulation ? simulation->maxPlannedTime() : 0.0f;
        const double rt = simulation ? simulation->lastPlannerRuntimeSec() : 0.0;
        studio.planReady = T > 0.0f;
        studio.plannedTime = T;
        studio.plannerRuntime = rt;
        if (studio.planReady) {
            studio.planMetrics = simulation->computePlanMetrics();
            const auto& M = *studio.planMetrics;
            const double mk = std::isfinite(M.makespan_s)    ? M.makespan_s    : -1.0;
            const double ft = std::isfinite(M.sum_of_cost_s) ? M.sum_of_cost_s : -1.0;
            studio.status = fmt::format(
                "Plan OK with '{}' in {:.3f}s (H={:.3f}s, makespan={:.3f}s, flowtime={:.3f}s, agents={}).",
                planner.name, rt, T, mk, ft, M.agents);
        } else {
            studio.planMetrics.reset();
            studio.status =
                fmt::format("Planning with '{}' produced no plan.", planner.name);
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset to start")) {
        access.resetRobotsToPlanStarts();
        studio.status = "Reset robots to plan starts.";
    }

    ImGui::SeparatorText("Available in this environment");
    for (const auto& s : itemsVec) ImGui::BulletText("%s", s.c_str());

    // --- Metrics (summary + optional per‑agent table) ---
    ImGui::Spacing();
    ImGui::SeparatorText("Plan metrics");
    if (!simulation || !studio.planReady) {
        ImGui::TextDisabled("No plan computed yet.");
    } else {
        if (!studio.planMetrics) {
            studio.planMetrics = simulation->computePlanMetrics();
        }
        const auto& M = *studio.planMetrics;
        ui::DrawPlanMetricsTable(M);

        ImGui::Spacing();
        // --- CSV export (observer-based in new backend) ----------------------
        if (ImGui::CollapsingHeader("CSV export (observers)", ImGuiTreeNodeFlags_DefaultOpen)) {
            // We keep weak_ptrs so we don't duplicate observers if the user mashes the button.
            static std::weak_ptr<grace::sim::MetricsCsvObserver> s_metricsObs;
            static std::weak_ptr<grace::sim::StateCsvObserver>   s_stateObs;

            // Paths and options (persist across frames)
            static char metricsPath[512] = "metrics_summary.csv";
            static bool metricsOverwrite = true;

            static char statePath[512]   = "state_log.csv";
            static bool stateOverwrite   = true;
            static int  stateEveryN      = 1;

            // Ensure parent directory exists before we attach observers.
            auto ensureParentDir = [](const char* path) {
                try {
                    std::filesystem::path p(path);
                    auto dir = p.parent_path();
                    if (!dir.empty()) std::filesystem::create_directories(dir);
                } catch (...) { /* swallow; open will fail and be logged by observer */ }
            };

            // Metrics CSV (one line per run)
            ImGui::SeparatorText("Summary metrics CSV");
            ImGui::InputText("Path##metrics", metricsPath, IM_ARRAYSIZE(metricsPath));
            ImGui::Checkbox("Overwrite on first write##metrics", &metricsOverwrite);
            ImGui::SameLine();
            if (ImGui::Button("Attach metrics CSV observer")) {
                ensureParentDir(metricsPath);
                if (simulation) {
                    // Avoid stacking duplicates if we already attached one.
                    if (s_metricsObs.expired()) {
                        auto obs = std::make_shared<grace::sim::MetricsCsvObserver>(
                            std::filesystem::path(metricsPath), metricsOverwrite);
                        simulation->addObserver(obs);
                        s_metricsObs = obs;
                        studio.status =
                            fmt::format("Metrics CSV observer attached: {}", metricsPath);
                    } else {
                        studio.status = "Metrics CSV observer already attached.";
                    }
                }
            }

            // State CSV (row per agent at an interval)
            ImGui::SeparatorText("State CSV (per step / per agent)");
            ImGui::InputText("Path##state", statePath, IM_ARRAYSIZE(statePath));
            ImGui::Checkbox("Overwrite on first write##state", &stateOverwrite);
            ImGui::InputInt("Every N steps", &stateEveryN);
            if (stateEveryN < 1) stateEveryN = 1;
            if (ImGui::Button("Attach state CSV observer")) {
                ensureParentDir(statePath);
                if (simulation) {
                    if (s_stateObs.expired()) {
                        std::vector<b2BodyId> watchAll; // empty => watch all
                        auto obs = std::make_shared<grace::sim::StateCsvObserver>(
                            std::filesystem::path(statePath), stateOverwrite, stateEveryN, watchAll);
                        simulation->addObserver(obs);
                        s_stateObs = obs;
                        studio.status =
                            fmt::format("State CSV observer attached: {}", statePath);
                    } else {
                        studio.status = "State CSV observer already attached.";
                    }
                }
            }

            // Optional: console logging of robot states (stdout) via SimLogOptions
            ImGui::SeparatorText("Console logging");
            ::SimLogOptions logOpt =
                simulation ? simulation->logOptions() : ::SimLogOptions{};
            bool changed = false;
            changed |= ImGui::Checkbox("Log robot states to console", &logOpt.logRobotStates);
            ImGui::SetNextItemWidth(140.f);
            changed |= ImGui::InputInt("Log every N steps (console)", &logOpt.everyNSteps);
            if (logOpt.everyNSteps < 1) logOpt.everyNSteps = 1;
            if (changed && simulation) simulation->setLogOptions(logOpt);

            ImGui::Spacing();
            ImGui::TextDisabled("Notes:");
            ImGui::BulletText("Metrics CSV: one row per run (written on Run End).");
            ImGui::BulletText("State CSV: a row per agent every N steps (written on Post Step).");
            ImGui::BulletText("Use Simulation step to actually Run and produce rows.");
        }

        // Optional per‑agent breakdown (collapsible + scrollable)
        if (ImGui::CollapsingHeader("Per-agent metrics", ImGuiTreeNodeFlags_DefaultOpen)) {
            const float scale = access.uiScale();
            ui::DrawPerAgentMetricsTable(M, /*height=*/260.0f * scale);
        }
    }


    // Optional navigation
    ImGui::Spacing();
    if (studio.planReady && ui::PrimaryButton("Next: Simulation \xee\x9c\xa8")) {
        access.showSimulationStep();
        studio.leftNavOpen = true;
    }

}

} // namespace ui