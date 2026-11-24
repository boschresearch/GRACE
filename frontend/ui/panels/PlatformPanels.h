// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "../../platform/PlatformBase.h"

namespace ui {

void DrawPlatformPanels(PlatformBase::WorkflowAccess platform);
void DrawStepEnvironment(PlatformBase::EnvironmentAccess platform);
void DrawStepAgents(PlatformBase::AgentsAccess platform);
void DrawStepPlanner(PlatformBase::PlannerAccess platform);
void DrawStepSimulation(PlatformBase::SimulationAccess platform);
void DrawRunningHUD(PlatformBase::SimulationAccess platform);

void DrawAgentInspector(PlatformBase::AgentsAccess platform);
void DrawGridAgentsUI(PlatformBase::AgentsAccess platform);
void DrawRoadmapAgentsUI(PlatformBase::AgentsAccess platform);
void DrawFreeAgentsUI(PlatformBase::AgentsAccess platform);

void Adopt(PlatformBase::EnvironmentAccess platform,
           std::unique_ptr<BaseEnvironment> environment,
           std::unique_ptr<Simulation> simulation,
           const char* status,
           PlatformBase::ProblemSpecPtr problemSpec = {},
           std::filesystem::path problemSpecSource = {});

} // namespace ui
