// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// Sample_MAPF_Roadmap.cpp — Roadmap sample registration & ctor
#include "platform/PlatformBase.h"
#include "platform/PlatformDefaults.h"
#include "env/RoadmapEnvironment.h"
#include "Simulation.h"
#include "planner/plugin/PlannerFactory.h"
#include "io/RoadmapXML.h"
#include "sample/SampleContext.h"

namespace {
    const auto kXmlPath = platform::defaults::mapRoadmapDemo();

    class PlatformSample_Roadmap final : public PlatformBase {
    public:
        explicit PlatformSample_Roadmap(SampleContext* ctx) : PlatformBase(ctx)
        {
            // Enter guided workflow, paused.
            m_context->pause = true;

            // Pre-select "Roadmap" representation in the Environment panel
            m_studio.envChoice = 1;

            // Prefill a friendly default path (purely as a convenience in the UI)
            m_studio.mapPath = kXmlPath;

            // Show roadmap overlay once there is an environment, but do NOT load here.
            m_overlays.drawRoadmap = false;

            // Message in title/HUD
            m_instanceInfo = "ROADMAP: Studio (open/create roadmap, edit graph, then agents, then plan)";
        }
    };

    Sample* CreateRoadmap(SampleContext* c) { return new PlatformSample_Roadmap(c); }

    const int s_reg_roadmap = RegisterSample("MAPF", "Roadmap: roadmap.xml + editor", &CreateRoadmap);
} // namespace
