// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/classical/CBSH2RTC/CBSH2RTC_Source.h"
#include "../inc/env/ICapabilities.h"
#include <fmt/format.h>
#include <stdexcept>
#include <chrono>

// === External CBSH2-RT headers ==============================================
#include <../extern/classical/CBSH2-RTC/inc/CBS.h>
#include <../extern/classical/CBSH2-RTC/inc/Instance.h>

#include "inc/util/Logger.h"

namespace {

    heuristics_type parseHeuristic(const std::string& s) {
        if (s == "Zero") return ZERO;
        if (s == "CG")   return CG;
        if (s == "DG")   return DG;
        if (s == "WDG")  return WDG;
        throw std::invalid_argument("CBSH2RTC(src): invalid heuristics=" + s);
    }

    rectangle_strategy parseRect(const std::string& s) {
        if (s == "None")     return NR;
        if (s == "R")        return R;
        if (s == "RM")       return RM;
        if (s == "GR")       return GR;
        if (s == "Disjoint") return DR;
        throw std::invalid_argument("CBSH2RTC(src): invalid rectangleReasoning=" + s);
    }

    corridor_strategy parseCorr(const std::string& s) {
        if (s == "None")     return NC;
        if (s == "C")        return C;
        if (s == "PC")       return PC;
        if (s == "STC")      return STC;
        if (s == "GC")       return GC;
        if (s == "Disjoint") return DC;
        throw std::invalid_argument("CBSH2RTC(src): invalid corridorReasoning=" + s);
    }

} // namespace

void CBSH2RTC_Source::applySettings(const PlannerSettings& s, CBSH2RTC_SourceConfig& cfg) {
    applyCommonSettings(s,cfg);
}

Instance CBSH2RTC_Source::makeInstanceFromGrid(const IDiscreteGrid& grid, const std::vector<StartGoal>& sgs){
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

std::vector<Plan> CBSH2RTC_Source::pathsToPlans(const IDiscreteGrid& grid,
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
            const int r = loc / cols;
            const int c = loc % cols;
            const b2Vec2 pos = grid.cellToWorld({c,r});
            const float t = static_cast<float>(i * step_dt_s);
            P.waypoints.push_back(Waypoint{pos, t});
        }
        out.push_back(std::move(P));
    }
    return out;
}

// ---------- Planner entry -----------------------------------------------------
PlanResult CBSH2RTC_Source::plan(const PlanRequest& req){
    PlanResult out;
    out.ids = req.ids;
    out.plans.clear();
    out.diag = {};
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("[CBSH2RTC(src)] ids.size() != sgs.size().");
        out.status = PlanStatus::Error;
        return out;
    }

    // allow per-call overrides
    CBSH2RTC_SourceConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    if (req.rng_seed) cfg.seed = static_cast<int>(*req.rng_seed);
    const IDiscreteGrid* grid = req.env.gridCapability();
    if (!grid) {
        Logger::instance().error("[CBSH2RTC(src)] Environment does not support IDiscreteGrid.");
        out.status = PlanStatus::Error;
        return out;
    }

    const auto t0 = std::chrono::steady_clock::now();

    auto instance = makeInstanceFromGrid(*grid, req.sgs);
    srand(cfg.seed);
    CBS cbs(instance, cfg.sipp, cfg.screen);
    cbs.setPrioritizeConflicts(cfg.prioritizingConflicts);
    cbs.setDisjointSplitting(cfg.disjointSplitting);
    cbs.setBypass(cfg.bypass);
    cbs.setRectangleReasoning(parseRect(cfg.rectangleReasoning));
    cbs.setCorridorReasoning(parseCorr(cfg.corridorReasoning));
    cbs.setHeuristicType(parseHeuristic(cfg.heuristics));
    cbs.setTargetReasoning(cfg.targetReasoning);
    cbs.setMutexReasoning(cfg.mutexReasoning);
    cbs.setSavingStats(false);
    cbs.setNodeLimit(cfg.nodeLimit);
    double runtime_sum = 0.0;
    int    min_f_val   = 0;
    bool   solved      = false;
    const int R = std::max(1, cfg.restarts);

    for (int i = 0; i < R; ++i) {
        cbs.clear();
        cbs.solve(req.time_limit_s, min_f_val);
        runtime_sum += cbs.runtime;
        if (cbs.solution_found) { solved = true; break; }
        min_f_val = static_cast<int>(cbs.min_f_val);
        cbs.randomRoot = true;
    }
    cbs.runtime = runtime_sum;

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!solved) {
        Logger::instance().warn(
            fmt::format("[CBSH2RTC(src)] No solution (timeout/node-limit). runtime={:.3f}s", runtime_sum));
        out.status = PlanStatus::Timeout; // best guess; could also be Infeasible depending on solver flags
        return out;
    }

    const auto paths = cbs.getPaths(); // std::vector<Path>
    out.plans = pathsToPlans(*grid, paths, cfg.step_dt_s);
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;
    return out;
}
