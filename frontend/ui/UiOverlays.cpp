// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "UiOverlays.h"
#include "UiStyle.h"
#include "../sample/Sample.h"
#include "ViewController.h"
#include "../platform/PlatformBase.h"
#include "../../sample/SampleContext.h"
#include "panels/UiWidgets.h"
#include <imgui.h>

namespace ui {

static void PreferencesModal(SampleContext& ctx)
{
    if (ImGui::BeginPopupModal("Preferences", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::SeparatorText("Display & UI");
        ImGui::SliderFloat("UI scale", &ctx.uiScale, 0.75f, 2.0f, "%.2f");
        ImGui::SeparatorText("Fonts (px @ scale 1.0)");
        ImGui::SliderFloat("Regular", &ctx.fontRegularPx, 12.0f, 20.0f, "%.0f");
        ImGui::SliderFloat("Medium",  &ctx.fontMediumPx,  18.0f, 28.0f, "%.0f");
        ImGui::SliderFloat("Large",   &ctx.fontLargePx,   26.0f, 40.0f, "%.0f");

        ImGui::SeparatorText("Panels");
        ImGui::SliderFloat("Tools width (window %)", &ctx.toolsPanelWidthFrac, 0.20f, 0.40f, "%.2f");

        ImGui::NewLine();
        if (PrimaryButton("Apply & Rebuild Fonts")) {
            // Re-apply theme and rebuild atlas (no extra ScaleAllSizes).
            ApplyGraceTheme(ctx.uiScale);
            LoadFonts(ctx, ctx.fbScale, "data/droid_sans.ttf");
        }
        ImGui::SameLine();
        if (ImGui::Button("Close")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

static void ShortcutsModal()
{
    if (ImGui::BeginPopupModal("Shortcuts & Tips", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::SeparatorText("Navigation");
        ImGui::TextUnformatted("Z / X : Zoom in/out");
        ImGui::TextUnformatted("Arrow keys : Pan view");
        ImGui::TextUnformatted("Home : Reset camera");

        ImGui::SeparatorText("Simulation");
        ImGui::TextUnformatted("P : Pause/Resume");
        ImGui::TextUnformatted("O : Single Step");
        ImGui::TextUnformatted("R : Restart sample");

        ImGui::SeparatorText("UI");
        ImGui::TextUnformatted("Tab : Toggle UI");
        ImGui::TextUnformatted("Ctrl/Cmd+K : Sample search");
        ImGui::TextUnformatted("Ctrl/Cmd+P : Command palette");

        ImGui::NewLine();
        if (PrimaryButton("Got it")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void DrawTopToolbar(SampleContext& ctx, ViewController& view, bool& requestQuit,
                    const std::function<void()>& restartCallback,
                    Sample* sample)
{
    if (!ctx.showUI) return;

    constexpr ImGuiWindowFlags kToolbarFlags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_AlwaysAutoResize;

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 10.0f, vp->WorkPos.y + 10.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::Begin("Overlay.Toolbar", nullptr, kToolbarFlags);

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12.0f * ctx.uiScale, 8.0f * ctx.uiScale));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,  ImVec2(8.0f  * ctx.uiScale, 0.0f));
    if (ctx.m_mediumFont) ImGui::PushFont(ctx.m_mediumFont);

    const float bigW = 120.0f * ctx.uiScale;
    if (PrimaryButton(ctx.pause ? "Start" : "Pause", ImVec2(bigW, 0))) ctx.pause = !ctx.pause;
    ImGui::SameLine();
    if (ImGui::Button("Step", ImVec2(80.0f * ctx.uiScale, 0))) ctx.singleStep = true;
    ImGui::SameLine();
    if (ImGui::Button("Restart", ImVec2(90.0f * ctx.uiScale, 0))) { if (restartCallback) restartCallback(); }
    ImGui::SameLine();
    if (ImGui::Button("Home", ImVec2(80.0f * ctx.uiScale, 0))) {
        if (auto* ps = dynamic_cast<PlatformBase*>(sample)) {
            if (ps->hasEnvironment()) {
                ps->centerCameraOnEnvironment();
            } else {
                view.setZoom(1.0f);
                view.setCenter({0.0f, 20.0f});
            }
        } else {
            view.setZoom(1.0f);
            view.setCenter({0.0f, 20.0f});
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Quit", ImVec2(70.0f * ctx.uiScale, 0))) requestQuit = true;
    ImGui::SameLine();
    if (ImGui::Button("Prefs")) ImGui::OpenPopup("Preferences");
    ImGui::SameLine();
    if (ImGui::Button("Help")) ImGui::OpenPopup("Shortcuts & Tips");

    if (ctx.m_mediumFont) ImGui::PopFont();
    ImGui::PopStyleVar(2);

    // reserve top-left overlay space under the toolbar ---
    {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        const ImVec2 pos = ImGui::GetWindowPos();
        const ImVec2 sz  = ImGui::GetWindowSize();
        // distance from the viewport working top to the toolbar bottom, plus a small pad
        const float reservedTop = (pos.y - vp->WorkPos.y) + sz.y + (6.0f * ctx.uiScale);
        // write for the *next* frame overlays
        ctx.overlaySafeTopPx = reservedTop;
    }

    ImGui::End();

    PreferencesModal(ctx);
    ShortcutsModal();
}

void DrawCommandPalette(bool& open, const std::vector<Command>& commands)
{
    if (!open) return;

    ImGui::SetNextWindowSize(ImVec2(520, 0), ImGuiCond_Appearing);
    if (ImGui::Begin("Command Palette", &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize))
    {
        static char query[96] = "";
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##cmd", "Type a command…", query, IM_ARRAYSIZE(query));

        auto containsI = [](const char* a, const char* b){
            if (!*b) return true;
            std::string A(a), B(b);
            std::transform(A.begin(), A.end(), A.begin(), ::tolower);
            std::transform(B.begin(), B.end(), B.begin(), ::tolower);
            return A.find(B) != std::string::npos;
        };

        ImGui::Separator();
        int shown = 0;
        for (const auto& cmd : commands) {
            if (!containsI(cmd.name, query)) continue;
            if (ImGui::Selectable(cmd.name)) {
                if (cmd.action) cmd.action();
                open = false;
            }
            if (cmd.hint) {
                ui::widgets::RightJustify(ImGui::CalcTextSize(cmd.hint).x);
                ImGui::TextDisabled("%s", cmd.hint);
            }
            ++shown;
        }
        if (shown == 0) ImGui::TextDisabled("No matches.");
    }
    ImGui::End();
}

} // namespace ui
