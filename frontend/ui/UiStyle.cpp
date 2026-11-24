// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "UiStyle.h"
#include "../sample/Sample.h"
#include "../../sample/SampleContext.h"
#include <filesystem>
#include <cstdio>

#if __has_include(<imgui-SFML.h>)
  #include <imgui-SFML.h>
#elif __has_include(<ImGui-SFML.h>)
  #include <ImGui-SFML.h>
#else
  #error "ImGui-SFML header not found."
#endif

namespace ui {

bool PrimaryButton(const char* label, ImVec2 size)
{
    const ImVec4* c = ImGui::GetStyle().Colors;
    ImGui::PushStyleColor(ImGuiCol_Button,       c[ImGuiCol_HeaderActive]);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,c[ImGuiCol_HeaderHovered]);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, c[ImGuiCol_HeaderActive]);
    const bool pressed = ImGui::Button(label, size);
    ImGui::PopStyleColor(3);
    return pressed;
}

void ApplyGraceTheme(float scale)
{
    ImGui::StyleColorsDark();
    ImGuiStyle& s = ImGui::GetStyle();
    auto S = [&](float x){ return x * scale; };

    s.WindowRounding   = S(12);
    s.ChildRounding    = S(12);
    s.FrameRounding    = S(10);
    s.GrabRounding     = S(12);
    s.TabRounding      = S(10);
    s.WindowPadding    = { S(12), S(12) };
    s.FramePadding     = { S(10), S(8) };
    s.ItemSpacing      = { S(10), S(8) };
    s.ItemInnerSpacing = { S(8),  S(6) };
    s.SeparatorTextPadding = { S(6), S(6) };

    ImVec4* c = s.Colors;
    const ImVec4 accent = ImVec4(0.18f, 0.52f, 0.78f, 1.0f);
    c[ImGuiCol_Text]                  = ImVec4(0.95f, 0.97f, 1.00f, 1.00f);
    c[ImGuiCol_TextDisabled]          = ImVec4(0.60f, 0.65f, 0.70f, 1.00f);
    c[ImGuiCol_WindowBg]              = ImVec4(0.10f, 0.11f, 0.13f, 0.92f);
    c[ImGuiCol_ChildBg]               = ImVec4(0.10f, 0.11f, 0.13f, 0.00f);
    c[ImGuiCol_PopupBg]               = ImVec4(0.10f, 0.11f, 0.13f, 0.98f);
    c[ImGuiCol_Border]                = ImVec4(0.34f, 0.38f, 0.42f, 0.30f);
    c[ImGuiCol_FrameBg]               = ImVec4(0.16f, 0.17f, 0.21f, 1.00f);
    c[ImGuiCol_FrameBgHovered]        = ImVec4(0.22f, 0.24f, 0.29f, 1.00f);
    c[ImGuiCol_FrameBgActive]         = ImVec4(0.24f, 0.26f, 0.32f, 1.00f);
    c[ImGuiCol_TitleBg]               = ImVec4(0.08f, 0.09f, 0.10f, 1.00f);
    c[ImGuiCol_TitleBgActive]         = ImVec4(0.08f, 0.10f, 0.12f, 1.00f);
    c[ImGuiCol_MenuBarBg]             = ImVec4(0.09f, 0.10f, 0.11f, 1.00f);
    c[ImGuiCol_ScrollbarGrab]         = ImVec4(0.26f, 0.28f, 0.33f, 1.00f);
    c[ImGuiCol_ScrollbarGrabHovered]  = ImVec4(0.32f, 0.35f, 0.40f, 1.00f);
    c[ImGuiCol_ScrollbarGrabActive]   = ImVec4(0.36f, 0.38f, 0.44f, 1.00f);
    c[ImGuiCol_CheckMark]             = ImVec4(0.95f, 0.97f, 1.00f, 1.00f);
    c[ImGuiCol_Button]                = accent;
    c[ImGuiCol_ButtonHovered]         = ImVec4(accent.x*1.08f, accent.y*1.08f, accent.z*1.08f, 1.0f);
    c[ImGuiCol_ButtonActive]          = ImVec4(accent.x*0.95f, accent.y*0.95f, accent.z*0.95f, 1.0f);
    c[ImGuiCol_Header]                = ImVec4(accent.x, accent.y, accent.z, 0.70f);
    c[ImGuiCol_HeaderHovered]         = ImVec4(accent.x, accent.y, accent.z, 0.85f);
    c[ImGuiCol_HeaderActive]          = ImVec4(accent.x, accent.y, accent.z, 1.00f);
    c[ImGuiCol_Tab]                   = ImVec4(0.14f, 0.15f, 0.18f, 1.0f);
    c[ImGuiCol_TabHovered]            = c[ImGuiCol_HeaderHovered];
    c[ImGuiCol_TabActive]             = ImVec4(0.20f, 0.21f, 0.24f, 1.0f);
    c[ImGuiCol_TabUnfocused]          = ImVec4(0.14f, 0.15f, 0.18f, 1.0f);
    c[ImGuiCol_TabUnfocusedActive]    = ImVec4(0.18f, 0.19f, 0.22f, 1.0f);
}

void LoadFonts(SampleContext& ctx, float framebufferScale, const std::string& fontPath)
{
    if (!std::filesystem::exists(fontPath)) {
        std::printf("ERROR: font '%s' not found. Run from project root.\n", fontPath.c_str());
        std::fflush(stdout);
        std::abort();
    }
    ImGuiIO& io = ImGui::GetIO();

    ImFontConfig fontConfig;
    fontConfig.RasterizerMultiply = ctx.uiScale * framebufferScale;
    const float regularSize = std::floor(ctx.fontRegularPx * ctx.uiScale);
    const float mediumSize  = std::floor(ctx.fontMediumPx  * ctx.uiScale);
    const float largeSize   = std::floor(ctx.fontLargePx   * ctx.uiScale);

    ctx.m_regularFont = io.Fonts->AddFontFromFileTTF(fontPath.c_str(), regularSize, &fontConfig);
    ctx.m_mediumFont  = io.Fonts->AddFontFromFileTTF(fontPath.c_str(), mediumSize,  &fontConfig);
    ctx.m_largeFont   = io.Fonts->AddFontFromFileTTF(fontPath.c_str(), largeSize,   &fontConfig);
    io.FontDefault = ctx.m_regularFont;

    (void)ImGui::SFML::UpdateFontTexture();
}

} // namespace ui
