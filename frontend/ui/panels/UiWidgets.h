// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <imgui.h>

namespace ui::widgets {

    // Badges & misc
    void StepBadge(const char* label, bool done, bool active);
    void Help(const char* text);
    void RightJustify(float itemWidth);

    // RAII dock windows (always call ImGui::End())
    class LeftDockWindow {
    public:
        LeftDockWindow(const char* title, float widthPx, float topPadPx = 6.0f);
        ~LeftDockWindow();
        explicit operator bool() const { return m_visible; }
        bool visible() const { return m_visible; }
    private:
        bool m_visible = false;
    };

    class DetailsDockWindow {
    public:
        DetailsDockWindow(const char* title,
                          float leftNavW, float toolsFrac,
                          float& drawerWidthPx,
                          float topPadPx = 6.0f,
                          bool* p_open = nullptr);
        ~DetailsDockWindow();
        explicit operator bool() const { return m_visible; }
        bool visible() const { return m_visible; }
    private:
        bool m_visible = false;
    };

} // namespace ui::widgets
