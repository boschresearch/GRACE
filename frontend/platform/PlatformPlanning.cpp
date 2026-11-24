// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "PlatformBase.h"
#include "PlanningWorkflow.h"
#include "PlannerCatalog.h"
#include "ResetWorkflow.h"
#include "Simulation.h"
#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "env/ContinuousEnvironment.h"

#include <fmt/format.h>

void PlatformBase::reconfigurePlannerIfChanged(const std::string& name)
{
    installPlanner(name);
    (void)replanPreservingGoals();
}

void PlatformBase::installPlanner(const std::string& name)
{
    if (!m_sim) return;

    const auto kind = isGrid() ? platform::EnvKind::Grid
                    : isRoadmap() ? platform::EnvKind::Roadmap
                                  : platform::EnvKind::Continuous;
    const auto selection = platform::planning::installPlanner(
        *m_sim, kind, name, m_plannerUi.timeLimitSeconds);
    m_plannerUi.name = selection.name;
    if (selection.usedFallback) {
        m_studio.status = fmt::format(
            "Requested planner '{}' is not valid for this environment — using '{}' instead.",
            name, selection.name);
    }
}

void PlatformBase::assignGoalOnly(b2Vec2 worldGoal)
{
    if (!m_sim || m_robotDisplay.selected.empty()) return;

    const auto assignment = platform::planning::assignGoal(
        *m_sim, m_robotDisplay.selected, worldGoal);
    invalidatePlanPresentation();
    m_studio.planReady = false;
    m_studio.status = (assignment.updatedRobots > 0)
        ? fmt::format(
            "Assigned goal to {} robot(s). No planning done — use Planner → Plan now.",
            assignment.updatedRobots)
        : "No robots selected to assign a goal.";
}

bool PlatformBase::assignRobotGoalOnly(
    const b2BodyId id,
    const b2Vec2 worldGoal) {
    if (!m_sim) return false;

    const std::vector<b2BodyId> ids{id};
    const auto assignment = platform::planning::assignGoal(
        *m_sim, ids, worldGoal);
    if (assignment.updatedRobots == 0) return false;

    invalidatePlanPresentation();
    m_studio.planReady = false;
    return true;
}

void PlatformBase::markPlansStale(const std::vector<b2BodyId>& ids)
{
    if (!m_sim) return;
    (void)platform::planning::discardPlans(*m_sim, ids);
    invalidatePlanPresentation();
    m_studio.planReady = false;
}

bool PlatformBase::replanPreservingGoals()
{
    if (!m_sim) return false;
    if (m_sim->robots().robots().empty()) return false;
    const bool planned = platform::planning::planPreservingGoals(
        *m_sim, m_sim->robots().robots());
    invalidatePlanPresentation();
    return planned;
}

bool PlatformBase::replanRobotPreservingGoal(const b2BodyId id)
{
    if (!m_sim) return false;
    const std::vector<b2BodyId> ids{id};
    const bool planned = platform::planning::planPreservingGoals(*m_sim, ids);
    invalidatePlanPresentation();
    return planned;
}

void PlatformBase::spawnAtClick(b2Vec2 p)
{
    if (!m_sim) return;
    b2BodyId id = b2_nullBodyId;
    std::string name = fmt::format("u{}", (unsigned)m_sim->robots().robots().size()+1);

    if (isGrid()) {
        const auto* G = grid();
        const auto [c, r] = G->worldToCell(p);
        id = m_sim->spawnGridCellRobot(c, r, name);
    } else if (isRoadmap()) {
        auto* R = road(); id = m_sim->spawnRoadmapNodeRobot(*R, R->nearestNode(p), /*r=*/0.07f, name);
    } else {
        id = m_sim->spawnCircleRobot(p, 0.08f, name);
    }
    if (B2_IS_NON_NULL(id)) {
        m_robotDisplay.colors[idKey(id)] =
            (b2HexColor)((52u<<24)|(138u<<16)|(189u<<8)|230u);
        m_robotDisplay.selected = { id };
        markPlansStale(m_sim->robots().robots());
        m_studio.agentsReady = true;
    }
}

void PlatformBase::resetRobotsToPlanStarts()
{
    setSimulationRunning(false);
    if (!m_sim) return;
    platform::reset::resetRobotsToPlanStarts(*m_sim);
    invalidatePlanPresentation();
}

void PlatformBase::invalidatePlanPresentation()
{
    m_executionTrace.points.clear();
    m_gridVisualization.reservations.clear();
    m_playback.visualizationTime = 0.0;
    m_gridVisualization.cacheDirty = true;
    m_studio.planMetrics.reset();
}
