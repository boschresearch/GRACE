// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// ui/widgets/FilePicker.cpp
#include "FilePicker.h"
#include <imgui.h>
#include <filesystem>
#include <unordered_map>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

namespace ui::widgets {

// Small per-dialog state stored by modal id
struct PickerState {
    fs::path cwd;
    std::string selection;
    bool initialized = false;
};

static std::unordered_map<std::string, PickerState> g_states;

static bool extMatches(const fs::directory_entry& de, const std::vector<std::string>& exts) {
    if (exts.empty()) return true;
    if (!de.is_regular_file()) return false;
    std::string e = de.path().extension().string();
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    for (auto f : exts) {
        std::string fl = f; std::transform(fl.begin(), fl.end(), fl.begin(), ::tolower);
        if (e == fl) return true;
    }
    return false;
}

static void ensureInit(PickerState& st, const std::string& currentPathFallback) {
    if (st.initialized) return;
    try {
        if (!currentPathFallback.empty()) {
            fs::path p(currentPathFallback);
            st.cwd = (fs::is_directory(p) ? p : (p.has_parent_path() ? p.parent_path() : fs::current_path()));
        } else {
            st.cwd = fs::current_path();
        }
    } catch (...) {
        st.cwd = fs::path(".");
    }
    st.initialized = true;
}

bool FilePickerModal(const char* modalId,
                     std::string& ioPath,
                     const std::vector<std::string>& extensions)
{
    PickerState& st = g_states[modalId];
    ensureInit(st, ioPath);

    bool picked = false;
    if (ImGui::BeginPopupModal(modalId, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::TextDisabled("%s", st.cwd.string().c_str());
        ImGui::Separator();

        if (ImGui::Button("Up")) {
            if (st.cwd.has_parent_path()) st.cwd = st.cwd.parent_path();
            st.selection.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button("Home")) {
            try { st.cwd = fs::path(std::getenv("HOME") ? std::getenv("HOME") : "."); }
            catch (...) { st.cwd = fs::path("."); }
            st.selection.clear();
        }

        ImGui::Separator();
        ImGui::BeginChild("##filelist", ImVec2(520, 360), true);

        std::vector<fs::directory_entry> dirs, files;
        try {
            for (const auto& de : fs::directory_iterator(st.cwd)) {
                if (de.is_directory()) dirs.push_back(de);
                else if (extMatches(de, extensions)) files.push_back(de);
            }
        } catch (...) {
            // ignore permission errors etc.
        }

        auto byName = [](const fs::directory_entry& a, const fs::directory_entry& b){
            return a.path().filename().string() < b.path().filename().string();
        };
        std::sort(dirs.begin(), dirs.end(), byName);
        std::sort(files.begin(), files.end(), byName);

        for (const auto& d : dirs) {
            const std::string label = "[Dir] " + d.path().filename().string();
            if (ImGui::Selectable(label.c_str(), false)) {
                st.cwd = d.path();
                st.selection.clear();
                break;
            }
        }
        for (const auto& f : files) {
            const bool sel = (st.selection == f.path().filename().string());
            if (ImGui::Selectable(f.path().filename().string().c_str(), sel)) {
                st.selection = f.path().filename().string();
            }
            if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
                ioPath = fs::absolute(f.path()).string();
                picked = true;
            }
        }

        ImGui::EndChild();
        ImGui::Separator();

        const bool canOpen = !st.selection.empty();
        ImGui::BeginDisabled(!canOpen);
        if (ImGui::Button("Open", ImVec2(120, 0))) {
            ioPath = fs::absolute(st.cwd / st.selection).string();
            picked = true;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }

        if (picked) {
            ImGui::CloseCurrentPopup();
            st.selection.clear();
        }
        ImGui::EndPopup();
    }
    return picked;
}

} // namespace ui::widgets
