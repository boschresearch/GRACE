// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

// main.cpp

#include "extern/enkiTS/src/TaskScheduler.h"
#include "sample/Sample.h"
#include "render/DebugDrawQueue.h"
#include "render/RendererSfml.h"
#include "ui/ViewController.h"

#include "box2d/base.h"
#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

#include <imgui.h>

#include "sample/SampleContext.h"
#if __has_include(<imgui-SFML.h>)
  #include <imgui-SFML.h>
#elif __has_include(<ImGui-SFML.h>)
  #include <ImGui-SFML.h>
#else
  #error "ImGui-SFML header not found. Install ImGui-SFML and adjust include paths."
#endif

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>
#include <memory>
#include <vector>
#include <fmt/format.h>

// NEW modules
#include "ui/UiStyle.h"
#include "ui/UiOverlays.h"
#include "ui/UiTools.h"
#include "input/Input.h"
#include "util/RunOutput.h"


namespace {
    SampleContext s_context;
    int32_t       s_selection        = 0;
    std::unique_ptr<Sample> s_sample = nullptr;

    bool         g_requestQuit      = false;
    bool         g_requestRestart   = false;

    ui::SamplesPanelState g_samplesPanel;

    bool  g_focusSamplesSearch = false;
    bool  g_commandPaletteOpen = false;

    ViewController  g_view;
    DebugDrawQueue  g_debugDraw;

    // shared tracker for pan/space/last mouse position
    app::InputTracker g_input;

    constexpr unsigned initW = 1280, initH = 720;
    constexpr float kZoomInFactor  = 1.005f;
    constexpr float kZoomOutFactor = 0.995f;

    void SortSamples() {
        auto less = [](const SampleEntry& a, const SampleEntry& b) {
            if (const int r = std::strcmp(a.category, b.category); r != 0) return r < 0;
            return std::strcmp(a.name, b.name) < 0;
        };
        std::sort(g_sampleEntries, g_sampleEntries + g_sampleCount, less);
    }

    static int FindSampleIndexByKey(std::string_view category, std::string_view name) {
        for (int i = 0; i < g_sampleCount; ++i) {
            if (std::string_view(g_sampleEntries[i].category) == category &&
                std::string_view(g_sampleEntries[i].name)     == name) {
                return i;
                }
        }
        return -1;
    }

    void RestartSample() {
        // Pause to ensure nothing is stepping while we tear down & rebuild.
        s_context.pause = true;
        s_sample.reset();
        s_context.restart = true;
        s_sample = std::unique_ptr<Sample>(g_sampleEntries[s_context.sampleIndex].createFcn(&s_context));
        s_context.restart = false;
        // No stale primitives carried to next frame
        g_debugDraw.clear();
    }

    void CreateUI(sf::RenderWindow& window) {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        if (!ImGui::SFML::Init(window, /*loadDefaultFont=*/false)) {
            std::printf("ImGui::SFML::Init failed");
            assert(false);
        }
        // QoL flags
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.ConfigWindowsResizeFromEdges = true;

        ui::ApplyGraceTheme(s_context.uiScale);
        // Theme already encodes uiScale; no extra ScaleAllSizes here.
        // Query framebuffer scale from ImGui (use X as representative):
        s_context.fbScale = std::max(1.0f, ImGui::GetIO().DisplayFramebufferScale.x);
        ui::LoadFonts(s_context, s_context.fbScale, "data/droid_sans.ttf");
    }

    void DestroyUI() {
        ImGui::SFML::Shutdown();
        ImGui::DestroyContext();
    }
} // namespace

int main(int, char**) {
    (void)grace::output::startRunOutput();
    s_context.Load();
    {
        int threads = static_cast<int>(enki::GetNumHardwareThreads());
        if (threads <= 0) threads = 1;
        // Cap for responsiveness; half the HW threads is typically a good default here.
        s_context.workerCount = std::max(1, std::min(8, threads / 2));
    }
    SortSamples();

    auto [major, minor, revision] = b2GetVersion();
    const std::string windowTitle = fmt::format("GRACE: A unified 2D simulator and benchmark for multi‑robot path planning");

    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(initW, initH)), windowTitle, sf::Style::Default);
    window.setVerticalSyncEnabled(false);
    window.setFramerateLimit(60);

    // Attach components
    g_view.attach(&window);
    s_context.view      = &g_view;
    s_context.debugDraw = &g_debugDraw;

    // ImGui
    CreateUI(window);

    // Initial view
    g_view.setZoom(1.0f);
    g_view.setCenter({ 0.0f, 20.0f });

    // Choose startup sample by stable key; fallback to MAPF → Grid Studio
    int idx = -1;
    if (!s_context.sampleCategory.empty() && !s_context.sampleName.empty()) {
        idx = FindSampleIndexByKey(s_context.sampleCategory, s_context.sampleName);
    }
    if (idx < 0) {
        idx = FindSampleIndexByKey("MAPF", "Grid Studio (guided flow)");
    }
    if (idx < 0) idx = 0; // ultimate fallback
    s_context.sampleIndex = idx;
    s_selection           = idx;

    float     frameTime  = 0.0f;
    sf::Clock deltaClock;
    sf::Clock frameClock;

    while (window.isOpen()) {
        // Keyboard zoom (smooth)
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Z)) {
            g_view.setZoom(g_view.getZoom() * kZoomInFactor);
        } else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::X)) {
            g_view.setZoom(g_view.getZoom() * kZoomOutFactor);
        }

        // Events
        while (const auto event = window.pollEvent()) {
            ImGui::SFML::ProcessEvent(window, *event);

            app::InputState S{
                .ctx = &s_context,
                .sample = s_sample.get(),
                .view = &g_view,
                .sampleCount = g_sampleCount,
                .selection = &s_selection,
                .requestQuit = &g_requestQuit,
                .focusSamplesSearch = &g_focusSamplesSearch,
                .commandPaletteOpen = &g_commandPaletteOpen,
                .tracker = &g_input
            };

            if (event->is<sf::Event::Closed>()) { g_requestQuit = true; continue; }
            if (const auto* resized = event->getIf<sf::Event::Resized>()){
                // Preserve world-height zoom across resizes
                g_view.setZoom(g_view.getZoom());
                continue;
            }
            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()){
                app::HandleKeyPressed(*keyPressed, S);
                continue;
            }
            if (const auto* keyReleased = event->getIf<sf::Event::KeyReleased>()){
                app::HandleKeyReleased(*keyReleased, S);
                continue;
            }
            if (const auto* mouseButtonPressed = event->getIf<sf::Event::MouseButtonPressed>()) {
                app::HandleMousePressed(*mouseButtonPressed, S);
                continue;
            }
            if (const auto* mouseButtonReleased = event->getIf<sf::Event::MouseButtonReleased>()) {
                app::HandleMouseReleased(*mouseButtonReleased, S);
                continue;
            }
            if (const auto* mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
                app::HandleMouseMove(*mouseMoved, S);
                continue;
            }
            if (const auto* mouseWheelScrolled = event->getIf<sf::Event::MouseWheelScrolled>()) {
                app::HandleMouseWheel(*mouseWheelScrolled, S);
                continue;
            }
        }

        if (g_requestQuit) {
            window.close();
            break;
        }

        // Frame begin
        window.clear(sf::Color(51, 51, 51));
        ImGui::SFML::Update(window, deltaClock.restart());

        if (!s_sample) {
            s_sample = std::unique_ptr<Sample>(g_sampleEntries[s_context.sampleIndex].createFcn(&s_context));
        }

        // Title
        if (s_sample && s_context.showUI) {
            const SampleEntry& entry = g_sampleEntries[s_context.sampleIndex];
            const std::string title  = fmt::format("{} : {}", entry.category, entry.name);
            s_sample->DrawTitle(title.c_str());
        }

        // Step & render
        if (s_sample) {
            s_sample->Step();
            RendererSfml::draw(g_debugDraw, window); // <-- static call
            g_debugDraw.clear();
        }

        // --- UI ---
        // Toolbar
        ui::DrawTopToolbar(s_context, g_view, g_requestQuit, [](){
            // Defer restart; do NOT destroy & recreate sample mid-frame.
            g_requestRestart = true;
        }, s_sample.get());

        // Tools panel (controls + samples) — width now uses s_context.toolsPanelWidthFrac
        g_samplesPanel.focusSearch |= g_focusSamplesSearch; // consume focus flag
        ui::DrawToolsPanel(window, s_context, s_sample.get(), s_selection, g_samplesPanel,
                           static_cast<int>(enki::GetNumHardwareThreads()));

        // Command palette
        std::vector<ui::Command> commands = {
            {"Start/Pause", "P", [&]{ s_context.pause = !s_context.pause; }},
            {"Step once", "O",  [&]{ s_context.singleStep = true; }},
            {"Restart sample", "R", [&]{ g_requestRestart = true; }},
            {"Home view", "Home", [&]{ g_view.setZoom(1.0f); g_view.setCenter({0.0f, 20.0f}); }},
            {"Toggle UI", "Tab", [&]{ s_context.showUI = !s_context.showUI; }},
            {"Quit", "Esc", [&]{ g_requestQuit = true; }}
        };
        ui::DrawCommandPalette(g_commandPaletteOpen, commands);

        // ImGui::ShowMetricsWindow();

        ImGui::SFML::Render(window);
        window.display();

        // Switch sample?
        if (s_selection != s_context.sampleIndex) {
            g_view.setZoom(1.0f);
            g_view.setCenter({ 0.0f, 20.0f });

            s_context.sampleIndex = s_selection;

            s_sample.reset();
            s_sample = std::unique_ptr<Sample>(g_sampleEntries[s_context.sampleIndex].createFcn(&s_context));
        }

        // Perform any deferred restart at the very end of the frame.
        if (g_requestRestart) {
            RestartSample();
            g_requestRestart = false;
        }

        frameTime = frameClock.restart().asSeconds();
        g_focusSamplesSearch = false; // reset one-shot focus request
    }

    s_sample.reset();
    DestroyUI();
    s_context.Save();
    grace::output::stopRunOutput();
    return 0;
}
