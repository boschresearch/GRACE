// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/classical/LACAM0/LACAM0_Source.h"
#include "../inc/env/ICapabilities.h"
#include "../inc/util/Logger.h"

#include <algorithm>
#include <cmath>
#include <vector>
#include <chrono>

// LaCAM public API
#include <planner.hpp>   // Instance, Solution, Deadline, DistTable, LaCAM, PIBT, solve, is_feasible_solution
#include <instance.hpp>  // in-memory ctor

void LACAM0_Source::configure(const PlannerSettings& s) { applySettings(s, m_cfg); }

void LACAM0_Source::applySettings(const PlannerSettings& s, LACAM0_SourceConfig& cfg) {
    applyCommonSettings(s,cfg);
}

PlanResult LACAM0_Source::plan(const PlanRequest& req){
    PlanResult out;
    out.ids = req.ids;
    out.plans.clear();
    out.diag = {};
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("[LACAM0(src)] ids.size() != sgs.size().");
        out.status = PlanStatus::Error;
        return out;
    }

    // Per-call overrides
    LACAM0_SourceConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    if (req.rng_seed)         cfg.seed     = static_cast<int>(*req.rng_seed);
    const IDiscreteGrid* grid = req.env.gridCapability();
    if (!grid) {
        Logger::instance().error("[LACAM0-src] Environment does not support IDiscreteGrid.");
        out.status = PlanStatus::Error;
        return out;
    }

    const auto t0 = std::chrono::steady_clock::now();

    // Configure LaCAM globals (in-process)
    DistTable::MULTI_THREAD_INIT = cfg.multi_thread_init;
    LaCAM::ANYTIME               = cfg.anytime;
    PIBT::SWAP                   = cfg.pibt_swap;
    PIBT::HINDRANCE              = cfg.pibt_hindrance;

    // Convert starts/goals to linear indices
    const int K = static_cast<int>(req.ids.size());
    std::vector<int> starts_idx; starts_idx.reserve(K);
    std::vector<int> goals_idx;  goals_idx.reserve(K);

    for (const auto&[start, goal] : req.sgs) {
        starts_idx.push_back(grid->gridCellToOccupancyIndex(grid->worldToCell(start)));
        goals_idx.push_back(grid->gridCellToOccupancyIndex(grid->worldToCell(goal)));
    }

    Instance ins(grid->gridWidthInCells(), grid->gridHeightInCells(),
        grid->occupancy(), starts_idx, goals_idx);
    if (!ins.is_valid(1)) {
        Logger::instance().error("[LACAM0-src] Instance invalid.");
        return out;
    }
    Deadline deadline(req.time_limit_s * 1000.0);
    const int verbose = std::max(0, cfg.verbose);
    const auto solution = solve(ins, std::max(0, verbose - 1), &deadline, cfg.seed);

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (solution.empty()) {
        Logger::instance().warn("[LACAM0-src] solver returned empty solution (timeout/failure).");
        out.status = PlanStatus::Timeout;
        return out;
    }
    if (!is_feasible_solution(ins, solution, verbose)) {
        Logger::instance().warn("[LACAM0-src] solution not feasible.");
        out.status = PlanStatus::Error;
        return out;
    }

    // Determine trim-length per agent (drop trailing goal-idle, keep first goal hit)
    const int T = static_cast<int>(solution.size());
    std::vector keep_len(static_cast<size_t>(K), T);
    for (int a = 0; a < K; ++a) {
        Vertex* goalA = ins.goals[static_cast<size_t>(a)];
        int j = T - 1;
        while (j >= 0 && solution[static_cast<size_t>(j)][static_cast<size_t>(a)] == goalA) --j;
        keep_len[static_cast<size_t>(a)] = std::min(T, j + 2); // include first at-goal step
    }

    // Build plans (time-stamped)
    out.plans.resize(req.ids.size());
    const double dt = cfg.step_dt_s;

    for (int a = 0; a < K; ++a) {
        const int len = std::min<int>(keep_len[static_cast<size_t>(a)], T);
        auto&[waypoints] = out.plans[static_cast<size_t>(a)];
        waypoints.reserve(static_cast<size_t>(len));

        for (int t = 0; t < len; ++t) {
            const auto* v = solution[static_cast<size_t>(t)][static_cast<size_t>(a)]; // Vertex*
            const int rr = v->y;
            const int cc = v->x;
            waypoints.push_back(Waypoint{grid->cellToWorld({cc, rr}), static_cast<float>(t * dt)});
        }
    }

    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;
    return out;
}
