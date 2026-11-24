// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <SFML/Graphics/RenderWindow.hpp>

struct SampleContext;
class Sample;

namespace ui {

    struct SamplesPanelState {
        bool focusSearch = false;
        char filter[96] = "";
        // ephemeral favorites (indices)
        std::vector<int> favorites;
    };

    void DrawToolsPanel(const sf::RenderWindow& window,
                        SampleContext& ctx,
                        Sample* sample,
                        int& selection,
                        SamplesPanelState& state,
                        int maxWorkers);

} // namespace ui
