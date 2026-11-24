// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// ui/widgets/FilePicker.h
#pragma once
#include <string>
#include <vector>

namespace ui::widgets {

    // Open a modal file picker.
    // Usage:
    //   if (ImGui::Button("Browse…")) ImGui::OpenPopup("PickMap##env");
    //   if (ui::widgets::FilePickerModal("PickMap##env", mapPath, {".map"})) { /* mapPath updated */ }
    //
    // Returns true in the frame the user confirms a file; the selected absolute path is written to ioPath.
    // 'extensions' may be empty (no filter) or a list like {".map",".agents"} (case-insensitive).
    bool FilePickerModal(const char* modalId,
                         std::string& ioPath,
                         const std::vector<std::string>& extensions);

} // namespace ui::widgets
