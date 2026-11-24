// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <string>
#include <imgui.h>

struct SampleContext;

namespace ui {

    // Apple-ish calm theme with blue accent. Safe to call every run.
    void ApplyGraceTheme(float scale);

    // Primary button (accented) helper.
    bool PrimaryButton(const char* label, ImVec2 size = ImVec2(0, 0));

    // Load fonts into SampleContext (regular/medium/large) and update atlas.
    // `framebufferScale` helps keep glyphs crisp on HiDPI.
    void LoadFonts(SampleContext& ctx, float framebufferScale,
                   const std::string& fontPath = "data/droid_sans.ttf");

} // namespace ui
