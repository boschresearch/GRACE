// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#pragma once

#include "box2d/id.h"
#include "box2d/types.h"
#include "SampleRegistry.h"

#include <memory>

class ViewController;
class DebugDrawQueue;

namespace enki { class TaskScheduler; }
class SampleTask;

//------------------------------------------------------------------------------
// Sample (base for all samples, Box2D world owner)
//------------------------------------------------------------------------------
class Sample
{
public:
    explicit Sample(struct SampleContext* context);
    virtual ~Sample();

    Sample(const Sample&)            = delete;
    Sample& operator=(const Sample&) = delete;
    Sample(Sample&&)                 = delete;
    Sample& operator=(Sample&&)      = delete;

    void CreateWorld();

    void DrawTitle(const char* string);
    virtual void Step();
    virtual void UpdateGui() {}
    virtual void Keyboard(int key) {}
    virtual bool OnEscape() { return false; } // allow scene-specific ESC handling

    // Mouse buttons are integers supplied by the app. Convention: 0 = left.
    virtual void MouseDown(b2Vec2 p, int button, int mod);
    virtual void MouseUp  (b2Vec2 p, int button);
    virtual void MouseMove(b2Vec2 p);

    void DrawTextLine(const char* text, ...);
    void ResetProfile();

    static void ShiftOrigin(b2Vec2 newOrigin);

    [[nodiscard]] static int ParsePath(const char* svgPath, b2Vec2 offset, b2Vec2* points,
                                       int capacity, float scale, bool reverseOrder);

    static constexpr int m_maxTasks   = 64;
    static constexpr int m_maxThreads = 64;

#ifdef NDEBUG
    static constexpr bool m_isDebug = false;
#else
    static constexpr bool m_isDebug = true;
#endif

    // Non-owning
    struct SampleContext* m_context = nullptr;

    // Owned resources
    std::unique_ptr<enki::TaskScheduler> m_scheduler;
    std::unique_ptr<SampleTask[]>        m_tasks;

    int         m_taskCount    = 0;
    int         m_threadCount  = 0;

    b2BodyId    m_mouseBodyId  = b2_nullBodyId;

    b2WorldId   m_worldId      = b2_nullWorldId;
    b2JointId   m_mouseJointId = b2_nullJointId;
    b2Vec2      m_mousePoint   = {};
    int         m_stepCount    = 0;
    b2Profile   m_maxProfile   = {};
    b2Profile   m_totalProfile = {};

private:
    int m_textLine      = 0;
    int m_textIncrement = 0;
};
