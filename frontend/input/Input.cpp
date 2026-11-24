// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#include "Input.h"
#include "../ui/ViewController.h"
#include "../sample/Sample.h"
#include <imgui.h>
#include <cmath>
#include "box2d/box2d.h"
#include "../sample/SampleContext.h"

namespace app {

namespace {
    inline bool isPanClick(sf::Mouse::Button b, bool spaceHeld) {
        if (b == sf::Mouse::Button::Right || b == sf::Mouse::Button::Middle) return true;
        return b == sf::Mouse::Button::Left && spaceHeld;
    }
}

void HandleKeyPressed(const sf::Event::KeyPressed& keyEvt, InputState& S)
{
    if (ImGui::GetIO().WantCaptureKeyboard) return;
    const auto key  = keyEvt.code;
    const bool ctrl = keyEvt.control || keyEvt.system;

    switch (key) {
        case sf::Keyboard::Key::Space: if (S.tracker) S.tracker->spaceDown = true; break;
        case sf::Keyboard::Key::Escape:
            // Give the active sample a chance to consume ESC (e.g., cancel an edit)
            if (S.sample && S.sample->OnEscape()) {
                // handled by sample; do not quit
            } else {
                if (S.requestQuit) *S.requestQuit = true;
            }
            break;

        case sf::Keyboard::Key::Left:
            if (ctrl) { if (S.sample) Sample::ShiftOrigin({  2.0f,  0.0f }); }
            else if (S.view) { S.view->panWorld({ -0.5f,  0.0f }); }
            break;
        case sf::Keyboard::Key::Right:
            if (ctrl) { if (S.sample) Sample::ShiftOrigin({ -2.0f,  0.0f }); }
            else if (S.view) { S.view->panWorld({  0.5f,  0.0f }); }
            break;
        case sf::Keyboard::Key::Up:
            if (ctrl) { if (S.sample) Sample::ShiftOrigin({  0.0f, -2.0f }); }
            else if (S.view) { S.view->panWorld({  0.0f, -0.5f }); }
            break;
        case sf::Keyboard::Key::Down:
            if (ctrl) { if (S.sample) Sample::ShiftOrigin({  0.0f,  2.0f }); }
            else if (S.view) { S.view->panWorld({  0.0f,  0.5f }); }
            break;
        case sf::Keyboard::Key::Home:
            if (S.view) { S.view->setZoom(1.0f); S.view->setCenter({0.0f, 20.0f}); }
            break;

        case sf::Keyboard::Key::R:     // restart requested by caller
                                       // (handled via toolbar; here we keep legacy binding as no-op)
                                       break;
        case sf::Keyboard::Key::O:     if (S.ctx) S.ctx->singleStep = true; break;
        case sf::Keyboard::Key::P:
            if (S.ctx && !ctrl) S.ctx->pause = !S.ctx->pause;
            else if (S.commandPaletteOpen) *S.commandPaletteOpen = !*S.commandPaletteOpen;
            break;

        case sf::Keyboard::Key::LBracket:
            if (S.selection && S.sampleCount > 0) {
                --(*S.selection);
                if (*S.selection < 0) *S.selection = S.sampleCount - 1;
            }
            break;
        case sf::Keyboard::Key::RBracket:
            if (S.selection && S.sampleCount > 0) {
                ++(*S.selection);
                if (*S.selection == S.sampleCount) *S.selection = 0;
            }
            break;

        case sf::Keyboard::Key::Tab:   if (S.ctx) S.ctx->showUI = !S.ctx->showUI; break;
        case sf::Keyboard::Key::K:     if (ctrl && S.focusSamplesSearch) *S.focusSamplesSearch = true; break;
        // case sf::Keyboard::Key::P:     if (ctrl && S.commandPaletteOpen) *S.commandPaletteOpen = true; break;

        default:
            if (S.sample) {
                int forwarded = -1;
                if (key >= sf::Keyboard::Key::A && key <= sf::Keyboard::Key::Z)
                    forwarded = 'A' + (static_cast<int>(key) - static_cast<int>(sf::Keyboard::Key::A));
                else if (key >= sf::Keyboard::Key::Num0 && key <= sf::Keyboard::Key::Num9)
                    forwarded = '0' + (static_cast<int>(key) - static_cast<int>(sf::Keyboard::Key::Num0));
                if (forwarded != -1) S.sample->Keyboard(forwarded);
            }
            break;
    }
}

void HandleKeyReleased(const sf::Event::KeyReleased& keyEvt, InputState& S)
{
    if (ImGui::GetIO().WantCaptureKeyboard) return;
    if (keyEvt.code == sf::Keyboard::Key::Space) {
        if (S.tracker) {
            S.tracker->spaceDown = false;
            S.tracker->panActive = false; // release hand tool
        }
    }
}

void HandleMousePressed(const sf::Event::MouseButtonPressed& mb, InputState& S) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (!S.view) return;
    if (!S.sample) return;

    const sf::Vector2i pixel(mb.position.x, mb.position.y);
    const auto [x, y] = S.view->mapPixelToWorld(pixel);
    const b2Vec2 pw{ x, y };

    if (mb.button == sf::Mouse::Button::Left && !(S.tracker && S.tracker->spaceDown)) {
        S.sample->MouseDown(pw, /*button*/0, /*modifiers*/0);
    } else if (S.tracker && isPanClick(mb.button, S.tracker->spaceDown)) {
        S.tracker->panActive   = true;
        S.tracker->lastMousePx = pixel;
    }
}

void HandleMouseReleased(const sf::Event::MouseButtonReleased& mb, InputState& S)
{
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (!S.view) return;
    if (!S.sample) return;

    const sf::Vector2i pixel(mb.position.x, mb.position.y);
    const auto [x, y] = S.view->mapPixelToWorld(pixel);
    const b2Vec2 pw{ x, y };

    if (mb.button == sf::Mouse::Button::Left && !(S.tracker && S.tracker->spaceDown)) {
        S.sample->MouseUp(pw, /*button*/0);
    } else if (S.tracker && isPanClick(mb.button, S.tracker->spaceDown)) {
        S.tracker->panActive = false;
    }
}

void HandleMouseMove(const sf::Event::MouseMoved& mm, InputState& S)
{
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (!S.view) return;
    const sf::Vector2i pixel(mm.position.x, mm.position.y);
    const auto [x, y] = S.view->mapPixelToWorld(pixel);
    const b2Vec2 pw{ x, y };

    if (S.sample) S.sample->MouseMove(pw);

    if (S.tracker && S.tracker->panActive) {
        S.view->panPixels(pixel - S.tracker->lastMousePx);
        S.tracker->lastMousePx = pixel;
    }
}

void HandleMouseWheel(const sf::Event::MouseWheelScrolled& ws, InputState& S)
{
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (!S.view) return;
    constexpr float kStep = 1.1f;
    S.view->setZoom( S.view->getZoom() * std::pow(kStep, -ws.delta) );
}

} // namespace app
