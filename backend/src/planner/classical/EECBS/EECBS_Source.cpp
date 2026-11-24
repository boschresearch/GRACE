// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/classical/EECBS/EECBS_Source.h"
#include "../inc/env/ICapabilities.h"
#include <fmt/format.h>
#include <stdexcept>
#include <chrono>

// === External EECBS headers ======================================
#include <../extern/classical/EECBS/inc/ECBS.h>
#include <../extern/classical/EECBS/inc/CBS.h>
#include <../extern/classical/EECBS/inc/Instance.h>

#include "inc/util/Logger.h"

namespace {

    heuristics_type parseHeuristic(const std::string& s) {
        if (s == "Zero") return ZERO;
        if (s == "CG")   return CG;
        if (s == "DG")   return DG;
        if (s == "WDG")  return WDG;
        throw std::invalid_argument("EECBSSource: invalid heuristics=" + s);
    }

    heuristics_type parseHeuristicType(const high_level_solver_type& ss, const std::string& s) {
        if (ss == ASTAR || ss == ASTAREPS) return ZERO;
        if (s == "Zero") return ZERO;
        if (s == "Global") return GLOBAL;
        if (s == "Path") return PATH;
        if (s == "Local") return LOCAL;
        if (s == "Conflict") return CONFLICT;
        throw std::invalid_argument("EECBSSource: invalid heuristicsType=" + s);
    }

    high_level_solver_type parseHighLevel(const std::string& s) {
        if (s == "A*")    return ASTAR;
        if (s == "A*eps") return ASTAREPS;
        if (s == "EES")   return EES;
        if (s == "NEW")   return NEW;
        throw std::invalid_argument("EECBSSource: invalid highLevelSolver=" + s);
    }

} // namespace

void EECBS_Source::configure(const PlannerSettings& s) {
    applyCommonSettings(s, m_cfg);
}

void EECBS_Source::applySettings(const PlannerSettings& s, EECBS_SourceConfig& cfg) {
    applyCommonSettings(s,cfg);
}

Instance EECBS_Source::makeInstanceFromGrid(const IDiscreteGrid& grid, const std::vector<StartGoal>& sgs){
    std::vector<int> starts; starts.reserve(sgs.size());
    std::vector<int> goals;  goals.reserve(sgs.size());
    for (const auto&[start, goal] : sgs) {
        const auto startCell = grid.worldToCell(start);
        const auto goalCell = grid.worldToCell(goal);
        starts.push_back(grid.gridCellToOccupancyIndex(startCell));
        goals.push_back(grid.gridCellToOccupancyIndex(goalCell));
    }
    Instance ins(grid.gridHeightInCells(), grid.gridWidthInCells(),
        grid.occupancy(), starts, goals, /*warehouseWidth*/0);
    return ins;
}

std::vector<Plan> EECBS_Source::pathsToPlans(const IDiscreteGrid& grid,
                           const std::vector<std::vector<PathEntry>>& paths,
                           const double step_dt_s){
    std::vector<Plan> out;
    out.reserve(paths.size());

    const int cols = grid.gridWidthInCells();

    for (const auto& p : paths) {
        Plan P;
        P.waypoints.reserve(p.size());
        for (size_t i = 0; i < p.size(); ++i) {
            const auto& e = p[i];
            const int loc = e.location;
            const int r   = loc / cols;
            const int c   = loc % cols;
            const b2Vec2 pos = grid.cellToWorld({c,r});
            const float t = static_cast<float>(i * step_dt_s);
            P.waypoints.push_back(Waypoint{pos, t});
        }
        out.push_back(std::move(P));
    }
    return out;
}

// ---------- Planner entry -----------------------------------------------------
PlanResult EECBS_Source::plan(const PlanRequest& req){
    PlanResult out;
    out.ids = req.ids;
    out.plans.clear();
    out.diag = {};
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("[EECBS(src)] ids.size() != sgs.size().");
        out.status = PlanStatus::Error;
        return out;
    }

    // allow per-call overrides
    EECBS_SourceConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    if (req.rng_seed)         cfg.seed     = static_cast<int>(*req.rng_seed);
    const IDiscreteGrid* grid = req.env.gridCapability();
    if (!grid) {
        Logger::instance().error("[EECBSSource] Environment does not support IDiscreteGrid.");
        out.status = PlanStatus::Error;
        return out;
    }

    const auto t0 = std::chrono::steady_clock::now();

    auto insOpt = makeInstanceFromGrid(*grid, req.sgs);
    auto instance = makeInstanceFromGrid(*grid, req.sgs);
    srand(cfg.seed);

    double runtime_sum = 0.0;
    int    lowerbound  = 0;
    bool   solved      = false;
    const  int R       = std::max(1, cfg.restarts);
    const auto s     = parseHighLevel(cfg.highLevelSolver);

    ECBS ecbs(instance, cfg.sipp, cfg.screen);
    ecbs.setPrioritizeConflicts(cfg.prioritizingConflicts);
    ecbs.setDisjointSplitting(cfg.disjointSplitting);
    ecbs.setBypass(cfg.bypass);
    ecbs.setRectangleReasoning(cfg.rectangleReasoning);
    ecbs.setCorridorReasoning(cfg.corridorReasoning);
    ecbs.setHeuristicType(parseHeuristic(cfg.heuristics), parseHeuristicType(s, cfg.inadmissibleH));
    ecbs.setTargetReasoning(cfg.targetReasoning);
    ecbs.setMutexReasoning(cfg.mutexReasoning);
    ecbs.setConflictSelectionRule(EARLIEST);
    ecbs.setNodeSelectionRule(NODE_CONFLICTPAIRS);
    ecbs.setSavingStats(false);
    ecbs.setHighLevelSolver(s, cfg.suboptimality);

    for (int i = 0; i < R; ++i) {
        ecbs.clear();
        ecbs.solve(req.time_limit_s, lowerbound);
        runtime_sum += ecbs.runtime;
        if (ecbs.solution_found) { solved = true; break; }
        lowerbound = ecbs.getLowerBound();
        ecbs.randomRoot = true;
    }
    ecbs.runtime = runtime_sum;

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!solved) {
        Logger::instance().warn(fmt::format(
            "[EECBSSource] No solution (timeout/budget). runtime={:.3f}s", runtime_sum));
        out.status = PlanStatus::Timeout;
        return out;
    }

    const auto paths = ecbs.getPaths(); // std::vector<std::vector<PathEntry>>
    out.plans = pathsToPlans(*grid, paths, cfg.step_dt_s);
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;
    return out;
}
