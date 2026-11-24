// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "UiTools.h"
#include "UiStyle.h"
#include "../sample/Sample.h"
#include "../../sample/SampleContext.h"
#include "panels/UiWidgets.h"
#include "box2d/box2d.h"
#include <imgui.h>
#include <algorithm>
#include <cstring>

#include "imgui_internal.h"

extern SampleEntry g_sampleEntries[MAX_SAMPLES];
extern int g_sampleCount;

namespace ui {

static bool ContainsI(std::string hay, std::string needle)
{
    std::transform(hay.begin(), hay.end(), hay.begin(), ::tolower);
    std::transform(needle.begin(), needle.end(), needle.begin(), ::tolower);
    return hay.find(needle) != std::string::npos;
}

static bool IsFav(const SamplesPanelState& st, int idx) {
    return std::find(st.favorites.begin(), st.favorites.end(), idx) != st.favorites.end();
}
static void ToggleFav(SamplesPanelState& st, int idx) {
    auto it = std::find(st.favorites.begin(), st.favorites.end(), idx);
    if (it == st.favorites.end()) st.favorites.push_back(idx);
    else st.favorites.erase(it);
}

void DrawToolsPanel(const sf::RenderWindow& window,
                    SampleContext& ctx,
                    Sample* sample,
                    int& selection,
                    SamplesPanelState& st,
                    int maxWorkers)
{
    const auto win = window.getSize();
    const float fontSize  = ImGui::GetFontSize();
    const float menuWidth = std::clamp(float(win.x) * ctx.toolsPanelWidthFrac, 320.0f, 520.0f);
    ImGui::SetNextWindowPos({ float(win.x) - menuWidth - 0.5f * fontSize, 0.5f * fontSize });
    ImGui::SetNextWindowSize({ menuWidth, float(win.y) - fontSize });

    if (!ImGui::Begin("Tools", &ctx.showUI,
                      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse))
    {
        ImGui::End(); return;
    }

    if (ImGui::BeginTabBar("ControlTabs", ImGuiTabBarFlags_None))
    {
        if (ImGui::BeginTabItem("Controls"))
        {
            ImGui::PushItemWidth(100.0f);
            ImGui::SliderInt("Sub-steps", &ctx.subStepCount, 1, 32);
            ImGui::SliderFloat("Hertz", &ctx.hertz, 5.0f, 240.0f, "%.0f hz");

            if (ImGui::SliderInt("Workers", &ctx.workerCount, 1, maxWorkers)) {
                ctx.workerCount = std::clamp(ctx.workerCount, 1, maxWorkers);
                // Restart request should be handled by caller via a callback;
                // here we only hint through a disabled button look:
                ImGui::TextDisabled("Worker change will apply on restart.");
            }
            ImGui::PopItemWidth();

            ImGui::SeparatorText("Physics");
            ImGui::Checkbox("Sleep", &ctx.enableSleep);
            ImGui::Checkbox("Warm Starting", &ctx.enableWarmStarting);
            ImGui::Checkbox("Continuous", &ctx.enableContinuous);

            ImGui::SeparatorText("Visuals");
            ImGui::Checkbox("Shapes", &ctx.drawShapes);
            ImGui::Checkbox("Joints", &ctx.drawJoints);
            ImGui::Checkbox("Joint Extras", &ctx.drawJointExtras);
            ImGui::Checkbox("Bounds", &ctx.drawBounds);
            ImGui::Checkbox("Contact Points", &ctx.drawContactPoints);
            ImGui::Checkbox("Contact Normals", &ctx.drawContactNormals);
            ImGui::Checkbox("Contact Impulses", &ctx.drawContactImpulses);
            ImGui::Checkbox("Contact Features", &ctx.drawContactFeatures);
            ImGui::Checkbox("Friction Impulses", &ctx.drawFrictionImpulses);
            ImGui::Checkbox("Mass", &ctx.drawMass);
            ImGui::Checkbox("Body Names", &ctx.drawBodyNames);
            ImGui::Checkbox("Graph Colors", &ctx.drawGraphColors);
            ImGui::Checkbox("Islands", &ctx.drawIslands);

            ImGui::SeparatorText("Diagnostics");
            ImGui::Checkbox("Counters", &ctx.drawCounters);
            ImGui::Checkbox("Profile", &ctx.drawProfile);
            if (sample && ImGui::Button("Reset Profile", ImVec2(-1, 0))) sample->ResetProfile();
            if (sample && ImGui::Button("Dump Mem Stats", ImVec2(-1, 0))) b2World_DumpMemoryStats(sample->m_worldId);

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Samples"))
        {
            // Search field (Ctrl/Cmd+K to focus)
            ImGui::SetNextItemWidth(-1);
            ImGui::InputTextWithHint("##SampleFilter", "Search samples…  (Ctrl/Cmd+K)", st.filter, IM_ARRAYSIZE(st.filter));
            if (st.focusSearch) { ImGui::SetKeyboardFocusHere(-1); st.focusSearch = false; }
            const bool hasQuery = st.filter[0] != '\0';

            // Favorites (visible when no search)
            if (!hasQuery && !st.favorites.empty()) {
                ImGui::SeparatorText("★ Favorites");
                for (int idx : st.favorites) {
                    if (idx < 0 || idx >= g_sampleCount) continue;
                    const auto& e = g_sampleEntries[idx];
                    const std::string label = std::string(e.category) + " / " + e.name;
                    bool selected = (ctx.sampleIndex == idx);
                    if (ImGui::Selectable(label.c_str(), selected)) selection = idx;

                    // Small inline unfavorite (right-justified)
                    float w = ImGui::CalcTextSize("★").x + ImGui::GetStyle().FramePadding.x * 2.0f;
                    ui::widgets::RightJustify(w);
                    if (ImGui::SmallButton("★")) ToggleFav(st, idx);
                }
                ImGui::Separator();
            }

            if (hasQuery) {
                // Flat filtered list
                for (int i = 0; i < g_sampleCount; ++i) {
                    const auto& e = g_sampleEntries[i];
                    const std::string full = std::string(e.category) + " / " + e.name;
                    if (!ContainsI(full, st.filter)) continue;

                    bool selected = (ctx.sampleIndex == i);
                    if (ImGui::Selectable(full.c_str(), selected)) selection = i;

                    const char* star = IsFav(st,i) ? "★" : "☆";
                    float w = ImGui::CalcTextSize(star).x + ImGui::GetStyle().FramePadding.x * 2.0f;
                    ui::widgets::RightJustify(w);
                    if (ImGui::SmallButton(star)) ToggleFav(st, i);
                }
            } else {
                // Categorized tree
                int i = 0;
                while (i < g_sampleCount) {
                    const char* category = g_sampleEntries[i].category;
                    const bool catSelected =
                        std::strcmp(category, g_sampleEntries[ctx.sampleIndex].category) == 0;

                    ImGuiTreeNodeFlags nodeFlags = ImGuiTreeNodeFlags_OpenOnArrow
                                                | ImGuiTreeNodeFlags_OpenOnDoubleClick;
                    if (catSelected) nodeFlags |= ImGuiTreeNodeFlags_Selected;

                    if (ImGui::TreeNodeEx(category, nodeFlags)) {
                        while (i < g_sampleCount && std::strcmp(category, g_sampleEntries[i].category) == 0) {
                            const bool sel = (ctx.sampleIndex == i);
                            ImGui::Selectable(g_sampleEntries[i].name, sel, ImGuiSelectableFlags_SpanAvailWidth);
                            if (ImGui::IsItemClicked()) selection = i;

                            // Star on right
                            const char* star2 = IsFav(st,i) ? "★" : "☆";
                            float w2 = ImGui::CalcTextSize(star2).x + ImGui::GetStyle().FramePadding.x * 2.0f;
                            ui::widgets::RightJustify(w2);
                            if (ImGui::SmallButton(star2)) ToggleFav(st, i);

                            ++i;
                        }
                        ImGui::TreePop();
                    } else {
                        while (i < g_sampleCount && std::strcmp(category, g_sampleEntries[i].category) == 0) ++i;
                    }
                }
            }

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End(); // Tools

    if (sample) sample->UpdateGui();
}

} // namespace ui
