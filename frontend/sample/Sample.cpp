// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#include "Sample.h"
#include "SampleTasks.h"
#include "SampleContext.h"

#include "../extern/enkiTS/src/TaskScheduler.h"
#include "../extern/shared/random.h"

#include "../render/DebugDrawQueue.h"
#include "../ui/ViewController.h"
#include "../render/DebugConfig.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <imgui.h>
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

Sample::Sample(SampleContext* context)
{
    m_context = context;

    m_scheduler = std::make_unique<enki::TaskScheduler>();
    m_scheduler->Initialize(m_context->workerCount);

    m_tasks       = std::make_unique<SampleTask[]>(m_maxTasks);
    m_taskCount   = 0;
    m_threadCount = 1 + m_context->workerCount;

    m_worldId       = b2_nullWorldId;
    m_textLine      = 30;
    m_textIncrement = 22;
    m_mouseJointId  = b2_nullJointId;
    m_stepCount     = 0;
    m_mouseBodyId   = b2_nullBodyId;
    m_mousePoint    = {};
    m_maxProfile    = {};
    m_totalProfile  = {};

    g_randomSeed = RAND_SEED;

    CreateWorld();
}

Sample::~Sample()
{
    // Only destroy if we own a valid world. PlatformBase sets m_worldId
    // to b2_nullWorldId in its destructor to signal "do not destroy here".
    if (B2_IS_NON_NULL(m_worldId)) {
        b2DestroyWorld(m_worldId);
        m_worldId = b2_nullWorldId;
    }
}

void Sample::CreateWorld()
{
    if (B2_IS_NON_NULL(m_worldId)) {
        b2DestroyWorld(m_worldId);
        m_worldId = b2_nullWorldId;
    }

    b2WorldDef worldDef       = b2DefaultWorldDef();
    worldDef.gravity          = m_context->gravity;
    worldDef.workerCount      = m_context->workerCount;
    worldDef.enqueueTask      = EnqueueTask;
    worldDef.finishTask       = FinishTask;
    worldDef.userTaskContext  = this;
    worldDef.enableSleep      = m_context->enableSleep;

    m_worldId = b2CreateWorld(&worldDef);
}

void Sample::DrawTitle(const char* string)
{
    if (!m_context->showUI) return;

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 5.0f,
                                   vp->WorkPos.y + 5.0f + m_context->overlaySafeTopPx),
                            ImGuiCond_Always);

    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav;

    ImGui::Begin("Overlay.Title", nullptr, flags);
    ImGui::PushFont(m_context->m_regularFont);
    ImGui::TextColored(ImColor(230, 153, 153, 255), "%s", string);
    ImGui::PopFont();
    ImGui::End();

    m_textLine = 26;
}

void Sample::MouseDown(b2Vec2 p, int button, int /*mod*/)
{
    if (B2_IS_NON_NULL(m_mouseJointId)) return;

    if (button == 0) // left mouse
    {
        b2AABB box;
        constexpr b2Vec2 d = { 0.001f, 0.001f };
        box.lowerBound = b2Sub(p, d);
        box.upperBound = b2Add(p, d);

        m_mousePoint = p;

        struct QueryContext { b2Vec2 point{}; b2BodyId bodyId = b2_nullBodyId; } queryContext{ p, b2_nullBodyId };

        auto QueryCallback = [](b2ShapeId shapeId, void* context)->bool {
            auto* qc = static_cast<QueryContext*>(context);
            const b2BodyId bodyId = b2Shape_GetBody(shapeId);
            if (const b2BodyType bodyType = b2Body_GetType(bodyId); bodyType != b2_dynamicBody)
                return true; // continue
            if (b2Shape_TestPoint(shapeId, qc->point)) { qc->bodyId = bodyId; return false; }
            return true;
        };

        b2World_OverlapAABB(
            m_worldId, b2Pos_zero, box, b2DefaultQueryFilter(), QueryCallback, &queryContext);

        if (B2_IS_NON_NULL(queryContext.bodyId))
        {
            b2BodyDef bodyDef = b2DefaultBodyDef();
            bodyDef.type        = b2_kinematicBody;
            bodyDef.position    = p;
            bodyDef.enableSleep = false;
            m_mouseBodyId = b2CreateBody(m_worldId, &bodyDef);

            b2MotorJointDef jointDef = b2DefaultMotorJointDef();
            jointDef.base.bodyIdA      = m_mouseBodyId;
            jointDef.base.bodyIdB      = queryContext.bodyId;
            jointDef.base.localFrameB.p= b2Body_GetLocalPoint(queryContext.bodyId, p);
            jointDef.linearHertz       = 7.5f;
            jointDef.linearDampingRatio= 0.7f;

            const b2MassData massData = b2Body_GetMassData(queryContext.bodyId);
            const float g  = b2Length(b2World_GetGravity(m_worldId));
            const float mg = massData.mass * g;
            jointDef.maxSpringForce    = 100.0f * mg;

            if (massData.mass > 0.0f)
            {
                const float lever = std::sqrt(massData.rotationalInertia / massData.mass);
                jointDef.maxVelocityTorque = 1.0f * lever * mg;
            }

            m_mouseJointId = b2CreateMotorJoint(m_worldId, &jointDef);
        }
    }
}

void Sample::MouseUp(b2Vec2 /*p*/, const int button)
{
    if (B2_IS_NON_NULL(m_mouseJointId) && button == 0)
    {
        b2DestroyJoint(m_mouseJointId, true);
        m_mouseJointId = b2_nullJointId;

        b2DestroyBody(m_mouseBodyId);
        m_mouseBodyId = b2_nullBodyId;
    }
}

void Sample::MouseMove(const b2Vec2 p)
{
    if (b2Joint_IsValid(m_mouseJointId) == false)
    {
        m_mouseJointId = b2_nullJointId;
    }
    m_mousePoint = p;
}

void Sample::DrawTextLine(const char* text, ...)
{
    if (!m_context->showUI) return;
    va_list arg;
    va_start(arg, text);

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 5.0f,
                                   vp->WorkPos.y + 10.0f + m_context->overlaySafeTopPx),
                            ImGuiCond_Always);

    ImGui::Begin("Overlay.Profile", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs |
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
    ImGui::PushFont(m_context->m_regularFont);
    ImGui::SetCursorPos(ImVec2(5.0f, static_cast<float>(m_textLine)));
    ImGui::TextColoredV(ImColor(230, 153, 153, 255), text, arg);
    ImGui::PopFont();
    ImGui::End();
    va_end(arg);

    m_textLine += m_textIncrement;
}

void Sample::ResetProfile()
{
    m_totalProfile = {};
    m_maxProfile   = {};
    m_stepCount    = 0;
}

void Sample::Step()
{
    float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;

    if (m_context->pause)
    {
        if (m_context->singleStep) {
            m_context->singleStep = false;
        } else {
            timeStep = 0.0f;
        }

        if (m_context->showUI)
        {
            DrawTextLine("****PAUSED****");
            m_textLine += m_textIncrement;
        }
    }

    if (B2_IS_NON_NULL(m_mouseJointId) && b2Joint_IsValid(m_mouseJointId) == false)
    {
        m_mouseJointId = b2_nullJointId;
        if (B2_IS_NON_NULL(m_mouseBodyId))
        {
            b2DestroyBody(m_mouseBodyId);
            m_mouseBodyId = b2_nullBodyId;
        }
    }

    if (B2_IS_NON_NULL(m_mouseBodyId) && timeStep > 0.0f)
    {
        const b2Transform target{m_mousePoint, b2Rot_identity};
        b2Body_SetTargetTransform(m_mouseBodyId, b2MakeWorldTransform(target), timeStep, true);
    }

    render::ApplyDebugConfig(*m_context, *m_context->debugDraw, *m_context->view);

    b2World_EnableSleeping    (m_worldId, m_context->enableSleep);
    b2World_EnableWarmStarting(m_worldId, m_context->enableWarmStarting);
    b2World_EnableContinuous  (m_worldId, m_context->enableContinuous);

    // Step Box2D
    b2World_Step(m_worldId, timeStep, m_context->subStepCount);
    m_taskCount = 0;

    // Collect debug draw batches
    b2World_Draw(m_worldId, &m_context->debugDraw->getB2DebugDraw());

    if (timeStep > 0.0f) ++m_stepCount;

    if (m_context->drawCounters)
    {
        const b2Counters counters = b2World_GetCounters(m_worldId);

        DrawTextLine("bodies/shapes/contacts/joints = %d/%d/%d/%d",
                     counters.bodyCount, counters.shapeCount,
                     counters.contactCount, counters.jointCount);
        DrawTextLine("islands/tasks = %d/%d", counters.islandCount, counters.taskCount);
        DrawTextLine("tree height static/movable = %d/%d",
                     counters.staticTreeHeight, counters.treeHeight);

        int totalCount = 0;
        char buffer[256] = {};
        const int colorCount = std::size(counters.colorCounts);

        int offset = std::snprintf(buffer, 256, "colors: ");
        for (int i = 0; i < colorCount; ++i)
        {
            offset += std::snprintf(
                buffer + offset, 256 - offset, "%d/", counters.colorCounts[i]);
            totalCount += counters.colorCounts[i];
        }
        std::snprintf(buffer + offset, 256 - offset, "[%d]", totalCount);
        DrawTextLine(buffer);
        DrawTextLine("stack allocator size = %d K", counters.stackUsed / 1024);
        DrawTextLine("total allocation = %lld K",
                     static_cast<long long>(counters.byteCount / 1024));
    }

    // Profile display
    {
        b2Profile p = b2World_GetProfile(m_worldId);
        m_maxProfile.step                = b2MaxFloat(m_maxProfile.step, p.step);
        m_maxProfile.pairs               = b2MaxFloat(m_maxProfile.pairs, p.pairs);
        m_maxProfile.collide             = b2MaxFloat(m_maxProfile.collide, p.collide);
        m_maxProfile.solve               = b2MaxFloat(m_maxProfile.solve, p.solve);
        m_maxProfile.solverSetup         = b2MaxFloat(m_maxProfile.solverSetup, p.solverSetup);
        m_maxProfile.constraints         = b2MaxFloat(m_maxProfile.constraints, p.constraints);
        m_maxProfile.prepareConstraints  = b2MaxFloat(m_maxProfile.prepareConstraints, p.prepareConstraints);
        m_maxProfile.integrateVelocities = b2MaxFloat(m_maxProfile.integrateVelocities, p.integrateVelocities);
        m_maxProfile.warmStart           = b2MaxFloat(m_maxProfile.warmStart, p.warmStart);
        m_maxProfile.solveImpulses       = b2MaxFloat(m_maxProfile.solveImpulses, p.solveImpulses);
        m_maxProfile.integratePositions  = b2MaxFloat(m_maxProfile.integratePositions, p.integratePositions);
        m_maxProfile.relaxImpulses       = b2MaxFloat(m_maxProfile.relaxImpulses, p.relaxImpulses);
        m_maxProfile.applyRestitution    = b2MaxFloat(m_maxProfile.applyRestitution, p.applyRestitution);
        m_maxProfile.storeImpulses       = b2MaxFloat(m_maxProfile.storeImpulses, p.storeImpulses);
        m_maxProfile.transforms          = b2MaxFloat(m_maxProfile.transforms, p.transforms);
        m_maxProfile.splitIslands        = b2MaxFloat(m_maxProfile.splitIslands, p.splitIslands);
        m_maxProfile.jointEvents         = b2MaxFloat(m_maxProfile.jointEvents, p.jointEvents);
        m_maxProfile.hitEvents           = b2MaxFloat(m_maxProfile.hitEvents, p.hitEvents);
        m_maxProfile.refit               = b2MaxFloat(m_maxProfile.refit, p.refit);
        m_maxProfile.bullets             = b2MaxFloat(m_maxProfile.bullets, p.bullets);
        m_maxProfile.sleepIslands        = b2MaxFloat(m_maxProfile.sleepIslands, p.sleepIslands);
        m_maxProfile.sensors             = b2MaxFloat(m_maxProfile.sensors, p.sensors);

        m_totalProfile.step                += p.step;
        m_totalProfile.pairs               += p.pairs;
        m_totalProfile.collide             += p.collide;
        m_totalProfile.solve               += p.solve;
        m_totalProfile.solverSetup         += p.solverSetup;
        m_totalProfile.constraints         += p.constraints;
        m_totalProfile.prepareConstraints  += p.prepareConstraints;
        m_totalProfile.integrateVelocities += p.integrateVelocities;
        m_totalProfile.warmStart           += p.warmStart;
        m_totalProfile.solveImpulses       += p.solveImpulses;
        m_totalProfile.integratePositions  += p.integratePositions;
        m_totalProfile.relaxImpulses       += p.relaxImpulses;
        m_totalProfile.applyRestitution    += p.applyRestitution;
        m_totalProfile.storeImpulses       += p.storeImpulses;
        m_totalProfile.transforms          += p.transforms;
        m_totalProfile.splitIslands        += p.splitIslands;
        m_totalProfile.jointEvents         += p.jointEvents;
        m_totalProfile.hitEvents           += p.hitEvents;
        m_totalProfile.refit               += p.refit;
        m_totalProfile.bullets             += p.bullets;
        m_totalProfile.sleepIslands        += p.sleepIslands;
        m_totalProfile.sensors             += p.sensors;
    }

    if (m_context->drawProfile)
    {
        b2Profile p = b2World_GetProfile(m_worldId);

        b2Profile aveProfile = {};
        if (m_stepCount > 0)
        {
            const float scale = 1.0f / static_cast<float>(m_stepCount);
            aveProfile.step                = scale * m_totalProfile.step;
            aveProfile.pairs               = scale * m_totalProfile.pairs;
            aveProfile.collide             = scale * m_totalProfile.collide;
            aveProfile.solve               = scale * m_totalProfile.solve;
            aveProfile.solverSetup         = scale * m_totalProfile.solverSetup;
            aveProfile.constraints         = scale * m_totalProfile.constraints;
            aveProfile.prepareConstraints  = scale * m_totalProfile.prepareConstraints;
            aveProfile.integrateVelocities = scale * m_totalProfile.integrateVelocities;
            aveProfile.warmStart           = scale * m_totalProfile.warmStart;
            aveProfile.solveImpulses       = scale * m_totalProfile.solveImpulses;
            aveProfile.integratePositions  = scale * m_totalProfile.integratePositions;
            aveProfile.relaxImpulses       = scale * m_totalProfile.relaxImpulses;
            aveProfile.applyRestitution    = scale * m_totalProfile.applyRestitution;
            aveProfile.storeImpulses       = scale * m_totalProfile.storeImpulses;
            aveProfile.transforms          = scale * m_totalProfile.transforms;
            aveProfile.splitIslands        = scale * m_totalProfile.splitIslands;
            aveProfile.jointEvents         = scale * m_totalProfile.jointEvents;
            aveProfile.hitEvents           = scale * m_totalProfile.hitEvents;
            aveProfile.refit               = scale * m_totalProfile.refit;
            aveProfile.bullets             = scale * m_totalProfile.bullets;
            aveProfile.sleepIslands        = scale * m_totalProfile.sleepIslands;
            aveProfile.sensors             = scale * m_totalProfile.sensors;
        }

        DrawTextLine("step [ave] (max) = %5.2f [%6.2f] (%6.2f)",  p.step,  aveProfile.step,  m_maxProfile.step);
        DrawTextLine("pairs [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.pairs, aveProfile.pairs, m_maxProfile.pairs);
        DrawTextLine("collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide);
        DrawTextLine("solve [ave] (max) = %5.2f [%6.2f] (%6.2f)",  p.solve, aveProfile.solve,  m_maxProfile.solve);

        DrawTextLine("> solver setup [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.solverSetup, aveProfile.solverSetup, m_maxProfile.solverSetup);
        DrawTextLine("> solve constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.constraints, aveProfile.constraints, m_maxProfile.constraints);

        DrawTextLine(">> prepare constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.prepareConstraints,  aveProfile.prepareConstraints,  m_maxProfile.prepareConstraints);
        DrawTextLine(">> integrate velocities [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.integrateVelocities, aveProfile.integrateVelocities, m_maxProfile.integrateVelocities);
        DrawTextLine(">> warm start [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.warmStart, aveProfile.warmStart, m_maxProfile.warmStart);
        DrawTextLine(">> solve impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.solveImpulses, aveProfile.solveImpulses, m_maxProfile.solveImpulses);
        DrawTextLine(">> integrate positions [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.integratePositions, aveProfile.integratePositions, m_maxProfile.integratePositions);
        DrawTextLine(">> relax impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.relaxImpulses, aveProfile.relaxImpulses, m_maxProfile.relaxImpulses);
        DrawTextLine(">> apply restitution [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.applyRestitution, aveProfile.applyRestitution, m_maxProfile.applyRestitution);
        DrawTextLine(">> store impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.storeImpulses, aveProfile.storeImpulses, m_maxProfile.storeImpulses);

        DrawTextLine(">> split islands [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.splitIslands, aveProfile.splitIslands, m_maxProfile.splitIslands);
        DrawTextLine("> update transforms [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.transforms, aveProfile.transforms, m_maxProfile.transforms);
        DrawTextLine("> joint events [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.jointEvents, aveProfile.jointEvents, m_maxProfile.jointEvents);
        DrawTextLine("> hit events [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.hitEvents, aveProfile.hitEvents, m_maxProfile.hitEvents);
        DrawTextLine("> refit BVH [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.refit, aveProfile.refit, m_maxProfile.refit);
        DrawTextLine("> sleep islands [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.sleepIslands, aveProfile.sleepIslands, m_maxProfile.sleepIslands);
        DrawTextLine("> bullets [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.bullets, aveProfile.bullets, m_maxProfile.bullets);
        DrawTextLine("sensors [ave] (max) = %5.2f [%6.2f] (%6.2f)",
            p.sensors, aveProfile.sensors, m_maxProfile.sensors);
    }
}

void Sample::ShiftOrigin(b2Vec2 /*newOrigin*/) { }

int Sample::ParsePath(const char* svgPath, const b2Vec2 offset, b2Vec2* points,
                      const int capacity, const float scale, const bool reverseOrder)
{
    int pointCount = 0;
    b2Vec2 currentPoint = {};
    const char* ptr = svgPath;
    char command = *ptr;

    auto isSpace = [](unsigned char c){ return std::isspace(c) != 0; };
    auto isDigitOrMinus = [](unsigned char c){ return std::isdigit(c) != 0 || c == '-'; };

    while (*ptr != '\0')
    {
        if (!isDigitOrMinus((unsigned char)*ptr))
        {
            command = *ptr;

            if (command == 'M' || command == 'L' || command == 'H' || command == 'V' ||
                command == 'm' || command == 'l' || command == 'h' || command == 'v')
            {
                ptr += 2;
            }

            if (command == 'z')
            {
                break;
            }
        }

        assert(isDigitOrMinus((unsigned char)*ptr));

        float x = 0.0f, y = 0.0f;
        switch (command)
        {
            case 'M':
            case 'L':
                if (std::sscanf(ptr, "%f,%f", &x, &y) == 2) {
                    currentPoint.x = x; currentPoint.y = y;
                } else { assert(false); }
                break;
            case 'H':
                if (std::sscanf(ptr, "%f", &x) == 1) {
                    currentPoint.x = x;
                } else { assert(false); }
                break;
            case 'V':
                if (std::sscanf(ptr, "%f", &y) == 1) {
                    currentPoint.y = y;
                } else { assert(false); }
                break;
            case 'm':
            case 'l':
                if (std::sscanf(ptr, "%f,%f", &x, &y) == 2) {
                    currentPoint.x += x; currentPoint.y += y;
                } else { assert(false); }
                break;
            case 'h':
                if (std::sscanf(ptr, "%f", &x) == 1) {
                    currentPoint.x += x;
                } else { assert(false); }
                break;
            case 'v':
                if (std::sscanf(ptr, "%f", &y) == 1) {
                    currentPoint.y += y;
                } else { assert(false); }
                break;
            default:
                assert(false);
                break;
        }

        points[pointCount] = { scale * (currentPoint.x + offset.x),
                               -scale * (currentPoint.y + offset.y) };
        ++pointCount;
        if (pointCount == capacity) break;

        while (*ptr != '\0' && !isSpace((unsigned char)*ptr)) { ++ptr; }
        while (isSpace((unsigned char)*ptr)) { ++ptr; }
    }

    if (pointCount == 0) return 0;

    if (reverseOrder)
    {
        for (int i = 0, j = pointCount - 1; i < j; ++i, --j)
            std::swap(points[i], points[j]);
    }
    return pointCount;
}
