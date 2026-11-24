// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "UiWidgets.h"
#include "PlatformPanels.h"
#include "../UiStyle.h"
#include "../../ui/ViewController.h"

#include <imgui.h>

#include "../../sample/SampleContext.h"

namespace W = ui::widgets;

namespace ui {

void DrawPlatformPanels(PlatformBase::WorkflowAccess access)
{
    auto studio = access.state();

    const bool envDone = access.hasEnvironment();
    const bool agDone = envDone && studio.agentsReady && access.hasAgents();
    const bool planDone = agDone && studio.planReady;

    const float topPad = access.overlaySafeTopPx() + 6.0f;
    const float leftNavW = studio.leftNavOpen ? 210.0f : 0.0f;

    if (studio.leftNavOpen) {
        W::LeftDockWindow left("Workflow", leftNavW, topPad);
        if (left) {
            ImGui::PushFont(access.mediumFont());
            ImGui::TextUnformatted("Interactive Workflow");
            ImGui::PopFont();
            ImGui::Separator();

            if (ImGui::Selectable(
                    "Environment",
                    access.workflowStep() == platform::WorkflowStep::Environment)) {
                access.navigateTo(platform::WorkflowStep::Environment);
            }
            W::StepBadge(
                "- Create a new environment", false,
                access.workflowStep() == platform::WorkflowStep::Environment);
            W::StepBadge(
                "- Open an existing environment", envDone,
                access.workflowStep() == platform::WorkflowStep::Environment);
            W::StepBadge(
                "- Environment editor", envDone,
                access.workflowStep() == platform::WorkflowStep::Environment);
            W::StepBadge(
                "Environment done?", envDone,
                access.workflowStep() == platform::WorkflowStep::Environment);
            ImGui::Separator();

            if (ImGui::Selectable(
                    "Agents",
                    access.workflowStep() == platform::WorkflowStep::Agents)) {
                access.navigateTo(platform::WorkflowStep::Agents);
            }
            W::StepBadge(
                "- Create / open agent setting", agDone,
                access.workflowStep() == platform::WorkflowStep::Agents);
            W::StepBadge(
                "- Agent editor", agDone,
                access.workflowStep() == platform::WorkflowStep::Agents);
            W::StepBadge(
                "Agents done?", agDone,
                access.workflowStep() == platform::WorkflowStep::Agents);
            ImGui::Separator();

            if (ImGui::Selectable(
                    "Planner",
                    access.workflowStep() == platform::WorkflowStep::Planner)) {
                access.navigateTo(platform::WorkflowStep::Planner);
            }
            W::StepBadge(
                "- Planner selector", planDone,
                access.workflowStep() == platform::WorkflowStep::Planner);
            W::StepBadge(
                "Planner done?", planDone,
                access.workflowStep() == platform::WorkflowStep::Planner);
            ImGui::Separator();

            if (ImGui::Selectable(
                    "Simulation",
                    access.workflowStep() == platform::WorkflowStep::Simulation)) {
                access.navigateTo(platform::WorkflowStep::Simulation);
            }
            W::StepBadge(
                "- Begin?", studio.running,
                access.workflowStep() == platform::WorkflowStep::Simulation);
            W::StepBadge(
                "- Finished?", !studio.running && planDone,
                access.workflowStep() == platform::WorkflowStep::Simulation);
            W::StepBadge(
                "- Idle", !studio.running && envDone,
                access.workflowStep() == platform::WorkflowStep::Simulation);
            ImGui::Separator();

            if (!studio.status.empty()) {
                ImGui::TextWrapped("%s", studio.status.c_str());
            }
        }
    }

    const float toolsFrac = access.toolsPanelWidthFrac();
    const bool showDetails =
        studio.detailsOpen &&
        (!studio.autoHideDetailsWhenRunning || !studio.running);

    if (showDetails) {
        W::DetailsDockWindow details("Workflow Details",
                                     leftNavW, toolsFrac, studio.detailsWidthPx, topPad,
                                     &studio.detailsOpen);
        if (details) {
            ImGui::Checkbox("Show left navigator", &studio.leftNavOpen);
            ImGui::SameLine();
            ImGui::Checkbox(
                "Auto-hide while running", &studio.autoHideDetailsWhenRunning);
            ImGui::Separator();

            switch (access.workflowStep()) {
                case platform::WorkflowStep::Environment:
                    DrawStepEnvironment(access.environmentPanel());
                    break;
                case platform::WorkflowStep::Agents:
                    DrawStepAgents(access.agentsPanel());
                    break;
                case platform::WorkflowStep::Planner:
                    DrawStepPlanner(access.plannerPanel());
                    break;
                case platform::WorkflowStep::Simulation:
                    DrawStepSimulation(access.simulationPanel());
                    break;
            }
        }
    }

    if (studio.autoHideDetailsWhenRunning && studio.running) {
        DrawRunningHUD(access.simulationPanel());
    }
}

} // namespace ui
