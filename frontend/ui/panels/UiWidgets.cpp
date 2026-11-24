// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "UiWidgets.h"

#include <algorithm>
#include <imgui.h>
#include <cmath>

namespace ui::widgets {

void StepBadge(const char* label, bool done, bool active) {
    ImGui::PushStyleColor(ImGuiCol_FrameBg,       active ? ImGui::GetStyle().Colors[ImGuiCol_HeaderActive]
                                                         : ImGui::GetStyle().Colors[ImGuiCol_FrameBg]);
    ImGui::PushStyleColor(ImGuiCol_Text,          done ? (ImVec4)ImColor(153,230,153,255)
                                                       : ImGui::GetStyle().Colors[ImGuiCol_Text]);
    ImGui::PushStyleVar (ImGuiStyleVar_FrameBorderSize, active ? 1.0f : 0.0f);
    ImGui::PushStyleVar (ImGuiStyleVar_FramePadding, ImVec2(8, 6));
    ImGui::Selectable(label, active);
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
}

void Help(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize()*34.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void RightJustify(float itemWidth) {
    float curX   = ImGui::GetCursorPosX();
    float avail  = ImGui::GetContentRegionAvail().x;
    float target = curX + std::max(0.0f, avail - itemWidth);
    ImGui::SameLine();
    ImGui::SetCursorPosX(target);
}

// --------- RAII windows ---------

LeftDockWindow::LeftDockWindow(const char* title, float widthPx, float topPadPx) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos (ImVec2(vp->WorkPos.x + 6.0f, vp->WorkPos.y + topPadPx), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(widthPx, vp->WorkSize.y - topPadPx - 6.0f));
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
    m_visible = ImGui::Begin(title, nullptr, flags);
}

LeftDockWindow::~LeftDockWindow() {
    ImGui::End();
}

DetailsDockWindow::DetailsDockWindow(const char* title,
                                     float leftNavW, float toolsFrac,
                                     float& drawerWidthPx,
                                     float topPadPx, bool* p_open)
{
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float toolsW  = std::clamp(vp->WorkSize.x * toolsFrac, 320.0f, 520.0f);
    const float x       = vp->WorkPos.x + leftNavW + 6.0f;
    const float fullH   = vp->WorkSize.y - topPadPx - 6.0f;
    const float wAvail  = std::max(0.0f, vp->WorkSize.x - toolsW - leftNavW - 12.0f);

    const float kMinW = 360.0f;
    const float kMaxW = std::min(std::max(kMinW, wAvail), 620.0f);
    drawerWidthPx     = std::clamp(drawerWidthPx, kMinW, kMaxW);

    ImGui::SetNextWindowPos(ImVec2(x, vp->WorkPos.y + topPadPx), ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints(ImVec2(kMinW, 200.0f), ImVec2(kMaxW, fullH));
    ImGui::SetNextWindowSize(ImVec2(drawerWidthPx, fullH), ImGuiCond_FirstUseEver);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;
    m_visible = ImGui::Begin(title, p_open, flags);
    if (m_visible) {
        // Snap height to available space every frame without fighting horizontal resizing.
        const float targetH = fullH;
        ImVec2 sz = ImGui::GetWindowSize();
        if (std::abs(sz.y - targetH) > 0.5f) {
            ImGui::SetWindowSize(ImVec2(sz.x, targetH));
            sz = ImGui::GetWindowSize();
        }
        drawerWidthPx = sz.x; // remember the user-chosen width
    }
}

DetailsDockWindow::~DetailsDockWindow() {
    ImGui::End();
}

} // namespace ui::widgets
