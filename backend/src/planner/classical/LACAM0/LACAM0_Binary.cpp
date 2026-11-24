// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <chrono>
#include <cctype>
#include <cmath>

#include "../inc/planner/classical/LACAM0/LACAM0_Binary.h"
#include "../inc/planner/classical/LACAM0/LACAM0_Common.h"
#include "../inc/env/ICapabilities.h"
#include "../inc/io/GridMovingAI.h"
#include "../inc/util/Logger.h"
#include "../inc/util/ProcRunner.h"
#include "../inc/util/TempDirGuard.h"

namespace fs = std::filesystem;

void LACAM0_Binary::configure(const PlannerSettings& s) { applySettings(s, m_cfg); }

void LACAM0_Binary::applySettings(const PlannerSettings& s, LACAM0_BinaryConfig& cfg) {
    if (const auto v = s.get<std::string>("binary")) cfg.binary  = *v;
    if (const auto v = s.get<bool>("keepTemps"))    cfg.keepTemps = *v;
    applyCommonSettings(s,cfg);
}

LACAM0_Binary::RunResult LACAM0_Binary::runSolver(const fs::path& workdir,
                         const fs::path& mapPath,
                         const fs::path& scenPath,
                         const fs::path& outPath,
                         const int k,
                         const double time_limit_s,
                         const int seed,
                         const int verbose) const {
    std::vector<std::string> args{
        "-m", mapPath.string(),
        "-i", scenPath.string(),
        "-N", std::to_string(k),
        "-s", std::to_string(seed),
        "-v", std::to_string(verbose),
        "-t", std::to_string(time_limit_s),
        "-o", outPath.string()
    };

    if (m_cfg.log_short)            args.emplace_back("-l");
    if (m_cfg.anytime)              args.emplace_back("--anytime");
    if (!m_cfg.multi_thread_init)   args.emplace_back("--no_dist_table_init");
    if (!m_cfg.pibt_swap)           args.emplace_back("--no_pibt_swap");
    if (!m_cfg.pibt_hindrance)      args.emplace_back("--no_pibt_hindrance");

    const fs::path stdoutPath = workdir / "lacam_stdout.txt";
    const fs::path stderrPath = workdir / "lacam_stderr.txt";
    const auto timeout_ms = std::chrono::milliseconds(static_cast<long>(time_limit_s * 1000.0) + 500);

    const auto res = util::runWithTimeout(m_cfg.binary, args, workdir, stdoutPath, stderrPath, timeout_ms);
    return RunResult{ res.exit_code == 0 && !res.timed_out, res.timed_out, res.exit_code };
}

std::vector<Plan> LACAM0_Binary::parseLaCAMPaths(const IDiscreteGrid& grid,const std::vector<StartGoal>& sgs,
    const fs::path& outPath, const double step_dt_s) {
    std::ifstream fin(outPath);
    if (!fin) {
        Logger::instance().error(std::string("[LACAM0] cannot open output: ") + outPath.string());
        return {};
    }

    std::string line;
    bool inSolution = false;
    while (std::getline(fin, line)) {
        if (line.rfind("solution=", 0) == 0) { inSolution = true; break; }
    }
    if (!inSolution) {
        Logger::instance().warn("[LACAM0] 'solution=' section not found.");
        return {};
    }

    const int K = static_cast<int>(sgs.size());

    std::vector<std::vector<Waypoint>> perAgent(static_cast<size_t>(K));

    // parse each "t:(c,r),(c,r),...,"
    long timesteps = 0;
    while (std::getline(fin, line)) {
        size_t p = 0;
        while (p < line.size() && std::isspace(static_cast<unsigned char>(line[p]))) ++p;
        if (p >= line.size()) continue;

        double t_val = 0;
        {
            char* endptr = nullptr;
            t_val = static_cast<int>(std::strtol(line.c_str() + p, &endptr, 10));
            if (!endptr) continue;
            const char* q = endptr;
            while (*q && std::isspace(static_cast<unsigned char>(*q))) ++q;
            if (*q != ':') continue;
            p = static_cast<size_t>(q - line.c_str() + 1);
        }

        std::vector<std::pair<int,int>> pairs;
        pairs.reserve(static_cast<size_t>(K));

        while (p < line.size()) {
            while (p < line.size() && line[p] != '(') ++p;
            if (p >= line.size()) break;
            ++p; // after '('

            while (p < line.size() && std::isspace(static_cast<unsigned char>(line[p]))) ++p;
            char* endC = nullptr;
            int c = static_cast<int>(std::strtol(line.c_str() + p, &endC, 10));
            if (endC == line.c_str() + p) break;
            p = static_cast<size_t>(endC - line.c_str());
            while (p < line.size() && (std::isspace(static_cast<unsigned char>(line[p])) || line[p] == ',')) ++p;

            char* endR = nullptr;
            int r = static_cast<int>(std::strtol(line.c_str() + p, &endR, 10));
            if (endR == line.c_str() + p) break;
            p = static_cast<size_t>(endR - line.c_str());
            while (p < line.size() && line[p] != ')') ++p;
            if (p < line.size()) ++p;

            if (grid.checkIfCellOutOfBound({c,r})) {
                Logger::instance().warn(std::string("[LACAM0] pair out of bounds in line: ") + line);
                continue;
            }
            pairs.emplace_back(static_cast<int>(c), static_cast<int>(r));
        }

        if (!pairs.empty()) {
            const int M = std::min<int>(K, static_cast<int>(pairs.size()));
            const auto t_sec = static_cast<float>(t_val * step_dt_s);
            for (int a = 0; a < M; ++a) {
                const auto [c, r] = pairs[static_cast<size_t>(a)];
                perAgent[static_cast<size_t>(a)].push_back(Waypoint{grid.cellToWorld({c, r}), t_sec});
            }
            ++timesteps;
        }
    }

    if (timesteps == 0) {
        Logger::instance().warn("[LACAM0] no timesteps parsed from 'solution=' section.");
        return {};
    }
    const auto sgsGrid = grace::io::detail::toGridStartGoals(grid, sgs);
    auto isGoal = [&](const int a, const IDiscreteGrid::GridCell cell)->bool{
        const auto [sc, sr, gc, gr] = sgsGrid[static_cast<size_t>(a)];
        return cell.r == gr && cell.c == gc;
    };

    // Trim trailing idle at goal; keep first arrival index inclusive
    for (int a = 0; a < K; ++a) {
        auto& v = perAgent[static_cast<size_t>(a)];
        if (v.empty()) continue;
        int cut = static_cast<int>(v.size());
        for (int i = 0; i < static_cast<int>(v.size()); ++i) {
            bool allGoal = true;
            for (int j = i; j < static_cast<int>(v.size()); ++j) {
                if (!isGoal(a, {grid.worldToCell(v[static_cast<size_t>(j)].p)})) { allGoal = false; break; }
            }
            if (allGoal) { cut = i; break; }
        }
        if (cut < static_cast<int>(v.size())) v.resize(static_cast<size_t>(cut) + 1);
    }

    // Build Plans
    std::vector<Plan> plans(static_cast<size_t>(K));
    for (int a = 0; a < K; ++a) {
        auto& steps = perAgent[static_cast<size_t>(a)];
        auto&[waypoints] = plans[static_cast<size_t>(a)];
        waypoints.reserve(steps.size());
        for (const auto& waypoint: steps) {
            waypoints.push_back(waypoint);
        }
    }
    return plans;
}

PlanResult LACAM0_Binary::plan(const PlanRequest& req){
    PlanResult out;
    out.ids = req.ids;
    out.plans.clear();
    out.status = PlanStatus::Error;
    out.diag = {};
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("[LACAM0(bin)] ids.size() != sgs.size().");
        return out;
    }

    // Per-call overrides (don’t mutate m_cfg; merge into a local copy)
    LACAM0_BinaryConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    const double time_limit_s = resolvePlannerTimeLimit(req.time_limit_s, 60.0);
    const IDiscreteGrid* grid = req.env.gridCapability();
    if (!grid) {
        Logger::instance().error("[LACAM0(bin)] Environment does not support IDiscreteGrid.");
        out.status = PlanStatus::Error;
        return out;
    }
    const int    seed         = (req.rng_seed ? static_cast<int>(*req.rng_seed) : cfg.seed);
    const int    verbose      = cfg.verbose;

    const auto t0 = std::chrono::steady_clock::now();

    util::TempDirGuard tmp(fs::temp_directory_path(), "lacam0_", cfg.keepTemps);
    const fs::path base = tmp.path();

    // Write map as MovingAI .map using new IO
    const fs::path mapPath = base / "map.map";
    if (!grace::io::writeGridMovingAIMap(*grid, mapPath)) {
        out.status = PlanStatus::Error;
        return out;
    }

    // Emit scenario from req.sgs (meters) with new IO
    std::vector<b2Vec2> starts_m, goals_m;
    starts_m.reserve(req.sgs.size()); goals_m.reserve(req.sgs.size());
    for (const auto&[start, goal] : req.sgs) { starts_m.push_back(start); goals_m.push_back(goal); }

    const fs::path scenPath = base / "instance.scen";
    if (!grace::io::writeGridMovingAIScenario(
            *grid, scenPath, starts_m, goals_m)) {
        out.status = PlanStatus::Error;
        return out;
    }

    const fs::path outPaths = base / "result.txt";
    const int k = static_cast<int>(req.ids.size());

    const auto rr = runSolver(base, mapPath, scenPath, outPaths, k, time_limit_s, seed, verbose);

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!rr.ok) {
        out.status = rr.timed_out ? PlanStatus::Timeout : PlanStatus::Error;
        if (cfg.keepTemps) { Logger::instance().info(std::string("[LACAM0] temp kept at ") + base.string()); tmp.set_keep(true); }
        return out;
    }

    out.plans  = parseLaCAMPaths(*grid, req.sgs, outPaths, cfg.step_dt_s);
    if (cfg.keepTemps) {
        Logger::instance().info(std::string("[LACAM0] temp kept at ") + base.string());
        tmp.set_keep(true);
    }
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;

    return out;
}
