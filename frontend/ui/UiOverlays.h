// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <functional>
#include <vector>

class ViewController;
class Sample;
struct SampleContext;

namespace ui {

     void DrawTopToolbar(SampleContext& ctx, ViewController& view, bool& requestQuit,
                         const std::function<void()>& restartCallback,
                         class Sample* sample = nullptr);

    // Bottom status bar overlay
    void DrawBottomStatus(const SampleContext& ctx, const Sample* sample,
                          const ViewController& view, float frameTime);

    // Command palette (Ctrl/Cmd+P)
    struct Command {
        const char* name;
        const char* hint;          // optional right-aligned hint/shortcut
        std::function<void()> action;
    };
    void DrawCommandPalette(bool& open, const std::vector<Command>& commands);

} // namespace ui
