// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../UiStyle.h"
#include "PlatformPanels.h"
#include "../../sample/SampleContext.h"
#include <imgui.h>

namespace ui {

void DrawRunningHUD(PlatformBase::SimulationAccess access)
{
    auto visualization = access.visualizationSettings();
    auto studio = access.state();

    // moved from old UiPlatformPanels.cpp (DrawRunningHUD) unchanged
    // --- begin original body ---
    if (!studio.planReady) return;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float topPad = access.overlaySafeTopPx();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 0.5f*vp->WorkSize.x,
                                   vp->WorkPos.y + topPad + 6.0f),
                            ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.92f);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize
                           | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("SimTransportHUD", nullptr, flags)) {
        const bool running = studio.running && !access.paused();
        ImGui::TextColored(ImColor(200,230,255), running ? "Simulation running..." : "Simulation paused");
        ImGui::SameLine();
        if (ui::PrimaryButton(running ? "Pause" : "Resume")) {
            access.setRunning(!running);
            studio.status = running ? "Paused." : "Simulation running.";
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            access.resetRobotsToPlanStarts();
            studio.status = "Stopped.";
        }
        ImGui::SameLine();
        if (ImGui::Button("Show UI")) {
            access.showDetails();
        }

        ImGui::Separator();
        bool visualizationChanged = false;
        visualizationChanged |= ImGui::Checkbox(
            "Start->Goal (dashed)", &visualization.showStartGoal);
        ImGui::SameLine();
        visualizationChanged |=
            ImGui::Checkbox("Ghost", &visualization.playGhost);
        ImGui::SameLine();
        visualizationChanged |= ImGui::Checkbox(
            "Planned paths", &visualization.drawPlanned);
        if (visualizationChanged) {
            access.setVisualizationSettings(visualization);
        }
    }
    ImGui::End();
    // --- end original body ---
}

void DrawStepSimulation(PlatformBase::SimulationAccess access)
{
    auto visualization = access.visualizationSettings();
    auto studio = access.state();

    // moved from old UiPlatformPanels.cpp (DrawStepSimulation) unchanged
    // --- begin original body ---
    if (!access.hasSimulation() || !studio.planReady) {
        ImGui::TextDisabled("Compute a plan first.");
        return;
    }

    ImGui::TextWrapped("Run the simulation using the current plan.");
    ImGui::Separator();

    if (!studio.running) {
        if (ui::PrimaryButton("Begin")) {
            access.setRunning(true);
            studio.status = "Simulation running.";
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset to t0")) {
            access.resetRobotsToPlanStarts();
            studio.status = "Reset to start.";
        }
    } else {
        if (ui::PrimaryButton("Pause")) {
            access.setRunning(false);
            studio.status = "Paused.";
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            access.resetRobotsToPlanStarts();
            studio.status = "Stopped.";
        }
    }

    ImGui::SeparatorText("Visualize");
    bool visualizationChanged = false;
    visualizationChanged |=
        ImGui::Checkbox("Planned paths", &visualization.drawPlanned);
    visualizationChanged |=
        ImGui::Checkbox("Executed traces", &visualization.drawExecuted);
    visualizationChanged |= ImGui::Checkbox(
        "Start->Goal (dashed)", &visualization.showStartGoal);
    ImGui::SetNextItemWidth(160);
    if (access.isRoadmap()) {
        visualizationChanged |=
            ImGui::Checkbox("Roadmap graph", &visualization.drawRoadmap);
    }
    ImGui::Separator();
    visualizationChanged |= ImGui::SliderFloat(
        "Dash px", &visualization.startGoalDashPx, 3.0f, 20.0f, "%.0f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(160);
    visualizationChanged |= ImGui::SliderFloat(
        "Gap px", &visualization.startGoalGapPx, 2.0f, 20.0f, "%.0f");
    visualizationChanged |=
        ImGui::Checkbox("Ghost playback", &visualization.playGhost);
    visualizationChanged |= ImGui::SliderFloat(
        "Ghost speed", &visualization.ghostSpeed, 0.1f, 5.0f, "%.2fx");
    visualizationChanged |= ImGui::Checkbox(
        "Grid reservations", &visualization.showGridReservations);
    if (access.isContinuous()) {
        visualizationChanged |= ImGui::Checkbox(
            "Clearance field", &visualization.showContinuousClearance);
        visualizationChanged |= ImGui::SliderInt(
            "Samples/meter",
            &visualization.clearanceSamplesPerMeter, 2, 32);
        visualizationChanged |= ImGui::SliderFloat(
            "Max clearance", &visualization.clearanceMax,
            0.5f, 5.0f, "%.2f m");
    }
    if (visualizationChanged) {
        access.setVisualizationSettings(visualization);
    }
    // --- end original body ---
}

} // namespace ui
