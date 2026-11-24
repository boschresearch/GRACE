// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#pragma once

#include "box2d/types.h"
#include <memory>
#include <string>

class ViewController;
class DebugDrawQueue;

struct SampleContext
{
    // Persistence
    void Save() const;
    void Load();

    // Persisted identity of the last sample by key (stable across registry changes)
    std::string sampleCategory;
    std::string sampleName;

    // Non-owning components (live in main)
    ViewController* view      = nullptr;
    DebugDrawQueue* debugDraw = nullptr;

    // UI & sim settings
    float uiScale      = 1.0f;

    // Font sizes in pixels (pre-scale). Tuned for 1080p defaults; adjusted in Preferences.
    float fontRegularPx = 15.0f;
    float fontMediumPx  = 22.0f;
    float fontLargePx   = 30.0f;

    // Right-side Tools panel width as a fraction of window width.
    float toolsPanelWidthFrac = 0.28f;

    // Runtime hint for font rasterization on HiDPI (not persisted).
    float fbScale = 1.0f;

    // Runtime: pixels to offset top-left overlays (reserved by toolbar). Not persisted.
    float overlaySafeTopPx = 0.0f;

    // Simulation
    float hertz        = 60.0f;
    int   subStepCount = 4;
    int   workerCount  = 1;
    b2Vec2 gravity { 0.0f, -9.81f };

    bool  restart    = false;
    bool  pause      = false;
    bool  singleStep = false;
    bool  showUI     = true;

    // Debug draw toggles
    bool  drawJointExtras      = false;
    bool  drawBounds           = false;
    bool  drawMass             = false;
    bool  drawBodyNames        = false;
    bool  drawContactPoints    = false;
    bool  drawContactNormals   = false;
    bool  drawContactImpulses  = false;
    bool  drawContactFeatures  = false;
    bool  drawFrictionImpulses = false;
    bool  drawIslands          = false;
    bool  drawGraphColors      = false;
    bool  drawCounters         = false;
    bool  drawProfile          = false;

    bool  enableWarmStarting = true;
    bool  enableContinuous   = true;
    bool  enableSleep        = true;

    // Persisted (legacy)
    int   sampleIndex = 0;
    bool  drawShapes  = true;
    bool  drawJoints  = true;

    // Fonts (for ImGui text)
    struct ImFont* m_regularFont = nullptr;
    ImFont* m_mediumFont  = nullptr;
    ImFont* m_largeFont   = nullptr;
};
