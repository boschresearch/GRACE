// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#pragma once
#include <SFML/Window/Event.hpp>

class ViewController;
class Sample;
struct SampleContext;

namespace app {

    // Per-application input tracker. Keeps any across-event state out of file-scope statics.
    struct InputTracker {
        bool          panActive    = false;          // "hand tool" active while dragging
        bool          spaceDown    = false;          // spacebar pressed => temporarily pan with LMB
        sf::Vector2i  lastMousePx  = {0, 0};         // last mouse position in pixels

        void reset() {
            panActive   = false;
            spaceDown   = false;
            lastMousePx = {0,0};
        }
    };

    struct InputState {
        SampleContext* ctx{};
        Sample* sample{};
        ViewController* view{};
        int sampleCount{};
        int* selection{};
        bool* requestQuit{};
        bool* focusSamplesSearch{};
        bool* commandPaletteOpen{}; // toggled by Ctrl/Cmd+P
        // explicit cross-event input state holder. Owned by the app; we only reference it.
        InputTracker* tracker{};
    };

    void HandleKeyPressed(const sf::Event::KeyPressed& e, InputState&);
    void HandleKeyReleased(const sf::Event::KeyReleased& keyEvt, InputState& S);
    void HandleMousePressed(const sf::Event::MouseButtonPressed& e, InputState&);
    void HandleMouseReleased(const sf::Event::MouseButtonReleased& e, InputState&);
    void HandleMouseMove(const sf::Event::MouseMoved& e, InputState&);
    void HandleMouseWheel(const sf::Event::MouseWheelScrolled& e, InputState&);

} // namespace app
