// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/classical/MAPFLNS2/MAPFLNS2_Source.h"
#include "../inc/env/ICapabilities.h"
#include "../inc/util/Logger.h"

#include <fmt/format.h>
#include <chrono>

// ===== External MAPFLNS2 headers (keep only in this .cpp) ====================
#include <../extern/classical/MAPF-LNS2/inc/LNS.h>
#include <../extern/classical/MAPF-LNS2/inc/Instance.h>

void MAPFLNS2_Source::applySettings(const PlannerSettings& s, MAPFLNS2_SourceConfig& cfg) {
    applyCommonSettings(s,cfg);
}

// ---------- Instance builder (in-memory, no files) ----------------------------
Instance MAPFLNS2_Source::makeInstanceFromGrid(const IDiscreteGrid& grid, const std::vector<StartGoal>& sgs){
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

std::vector<Plan> MAPFLNS2_Source::pathsToPlans(const IDiscreteGrid& grid,
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

PlanResult MAPFLNS2_Source::plan(const PlanRequest& req){
    PlanResult out;
    out.ids = req.ids;
    out.plans.clear();
    out.diag = {};
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("[MAPFLNS2(src)] ids.size() != sgs.size().");
        out.status = PlanStatus::Error;
        return out;
    }

    // Merge per-call overrides
    MAPFLNS2_SourceConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    if (req.rng_seed) cfg.seed = static_cast<int>(*req.rng_seed);
    const IDiscreteGrid* grid = req.env.gridCapability();
    if (!grid) {
        Logger::instance().error("[MAPFLNS2(src)] Environment does not support IDiscreteGrid.");
        out.status = PlanStatus::Error;
        return out;
    }

    const auto t0 = std::chrono::steady_clock::now();

    // Build instance in-memory
    auto instance = makeInstanceFromGrid(*grid, req.sgs);
    srand(cfg.seed);
    PIBTPPS_option pipp_opt;
    pipp_opt.windowSize  = cfg.pibtWindow;
    pipp_opt.winPIBTSoft = cfg.winPibtSoftmode;
    LNS lns(instance,
            req.time_limit_s,
            cfg.initAlgo,
            cfg.replanAlgo,
            cfg.destoryStrategy,
            cfg.neighborSize,
            cfg.maxIterations,
            cfg.initLNS,
            cfg.initDestoryStrategy,
            cfg.sipp,
            cfg.screen,
            pipp_opt);
    const bool success = lns.run();

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    const auto& paths = lns.getPaths(); // std::vector<std::vector<PathEntry>>

    // Trim trailing idle at goal (keep first arrival)
    std::vector goal_idx(req.sgs.size(), -1);
    for (const auto&[start, goal] : req.sgs) {
        goal_idx.push_back(grid->gridCellToOccupancyIndex(grid->worldToCell(goal)));
    }

    std::vector trimmed(paths);
    for (size_t a = 0; a < trimmed.size() && a < goal_idx.size(); ++a) {
        auto& v = trimmed[a];
        if (v.empty() || goal_idx[a] < 0) continue;
        int cut = static_cast<int>(v.size());
        for (int i = 0; i < static_cast<int>(v.size()); ++i) {
            bool allGoal = true;
            for (int j = i; j < static_cast<int>(v.size()); ++j) {
                if (v[static_cast<size_t>(j)].location != goal_idx[a]) { allGoal = false; break; }
            }
            if (allGoal) { cut = i; break; }
        }
        if (cut < static_cast<int>(v.size())) v.resize(static_cast<size_t>(cut) + 1);
    }

    out.plans  = pathsToPlans(*grid, trimmed, cfg.step_dt_s);
    out.status = out.plans.empty() || !success ? PlanStatus::Error : PlanStatus::Success;
    return out;

}
