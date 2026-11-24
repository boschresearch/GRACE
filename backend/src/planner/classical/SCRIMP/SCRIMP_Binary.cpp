// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "../inc/planner/classical/SCRIMP/SCRIMP_Binary.h"
#include "../inc/env/ICapabilities.h"
#include "../inc/io/GridMovingAI.h"
#include "../inc/util/Logger.h"
#include "../inc/util/ProcRunner.h"
#include "../inc/util/TempDirGuard.h"

namespace fs = std::filesystem;

namespace {

    struct SampleRC {
        int r;
        int c;
        int t;
    };

    std::vector<std::string> buildArgs(const fs::path& gridTxt,
                                       const fs::path& startsCsv,
                                       const fs::path& goalsCsv,
                                       const fs::path& outCsv,
                                       const int k,
                                       const SCRIMP_BinaryConfig& cfg)
    {
        return {
            cfg.driver,
            "--grid_file",   gridTxt.string(),
            "--starts_xy",   startsCsv.string(),
            "--targets_xy",  goalsCsv.string(),
            "--k",           std::to_string(k),
            "--outputPaths", outCsv.string(),
            "--checkpoint",  cfg.checkpoint,
            "--device",      cfg.device,
            "--max_steps",   std::to_string(cfg.max_steps),
            "--seed",        std::to_string(cfg.seed)
        };
    }

    bool writeGridTxt(const IDiscreteGrid& grid, const fs::path& path) {
        std::ofstream out(path);
        if (!out) return false;
        const int rows = grid.gridHeightInCells();
        const int cols = grid.gridWidthInCells();
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                out << (grid.checkIfCellBlocked({c, r}) ? '#' : '.'); // '.' free, '#' obstacle
            }
            out << '\n';
        }
        return static_cast<bool>(out);
    }

    bool writeXYCsv(const std::vector<std::pair<int,int>>& xy, const fs::path& path) {
        std::ofstream out(path);
        if (!out) return false;
        for (auto [x, y] : xy) out << x << ',' << y << '\n'; // x=col, y=row
        return static_cast<bool>(out);
    }

}


void SCRIMP_Binary::configure(const PlannerSettings& s) {
    applySettings(s, m_cfg);
}

void SCRIMP_Binary::applySettings(const PlannerSettings& s, SCRIMP_BinaryConfig& cfg) {
    if (const auto v = s.get<std::string>("python_exe")) cfg.python_exe = *v;
    if (const auto v = s.get<std::string>("driver"))     cfg.driver     = *v;

    if (const auto v = s.get<std::string>("checkpoint")) cfg.checkpoint = *v;
    if (const auto v = s.get<std::string>("device"))     cfg.device     = *v;
    if (const auto v = s.get<int>("max_steps"))          cfg.max_steps  = *v;
    if (const auto v = s.get<int>("seed"))               cfg.seed       = *v;
    if (const auto v = s.get<double>("cutoff_s"))        cfg.cutoff_s   = *v;

    if (const auto v = s.get<bool>("keepTemps"))         cfg.keepTemps  = *v;
    if (const auto v = s.get<double>("step_dt_s"))       cfg.step_dt_s  = *v;
}

SCRIMP_Binary::RunResult SCRIMP_Binary::runSolver(const fs::path& workdir,
                         const fs::path& gridTxt,
                         const fs::path& startsCsv,
                         const fs::path& goalsCsv,
                         const fs::path& outCsv,
                         const int k,
                         const double time_limit_s,
                         const SCRIMP_BinaryConfig& cfg) const {
    const fs::path outPath = workdir / "scrimp_stdout.txt";
    const fs::path errPath = workdir / "scrimp_stderr.txt";

    const auto args = buildArgs(gridTxt, startsCsv, goalsCsv, outCsv, k, cfg);
    const auto timeout_ms = std::chrono::milliseconds(static_cast<long>(time_limit_s * 1000.0) + 500);

    const auto res = util::runWithTimeout(
        cfg.python_exe, args, workdir, outPath, errPath, timeout_ms);
    return RunResult{ res.exit_code == 0 && !res.timed_out, res.timed_out, res.exit_code };
}

std::vector<Plan> SCRIMP_Binary::parseCsvPaths(const IDiscreteGrid& grid, const int numAgents,
    const fs::path& outCsv, const std::vector<std::pair<int,int>>& goalCell){
    std::ifstream in(outCsv);
    if (!in) {
        Logger::instance().error("SCRIMP: paths CSV missing.");
        return {};
    }

    const int K = numAgents;
    std::vector<std::vector<SampleRC>> perAgent(static_cast<size_t>(K));

    std::string line;
    const int maxAgent = numAgents - 1;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        for (char& ch : line) if (ch == ',') ch = ' ';
        std::istringstream ss(line);
        int a=-1, t=0, x=0, y=0;
        if (!(ss >> a >> t >> x >> y)) {
            Logger::instance().warn("SCRIMP: bad CSV line: " + line);
            continue;
        }
        if (a < 0 || a > maxAgent) continue;
        const int r = y, c = x;
        if (r < 0 || r >= grid.gridHeightInCells() || c < 0 || c >= grid.gridWidthInCells()) {
            Logger::instance().warn("SCRIMP: out-of-bounds cell (" + std::to_string(r)+","+std::to_string(c)+")");
            continue;
        }
        perAgent[static_cast<size_t>(a)].push_back(SampleRC{r, c, t});
    }

    auto isGoal = [&](int a, int r, int c)->bool {
        if (a < 0 || a >= numAgents) return false;
        if (static_cast<size_t>(a) >= goalCell.size()) return false;
        const auto [rg, cg] = goalCell[static_cast<size_t>(a)];
        return (rg >= 0 && cg >= 0 && r == rg && c == cg);
    };

    for (int a = 0; a < numAgents; ++a) {
        auto& v = perAgent[static_cast<size_t>(a)];
        if (v.empty()) continue;
        std::stable_sort(v.begin(), v.end(),
                         [](const SampleRC& u, const SampleRC& w){ return u.t < w.t; });

        int cut = static_cast<int>(v.size());
        for (int i = 0; i < static_cast<int>(v.size()); ++i) {
            bool allGoal = true;
            for (int j = i; j < static_cast<int>(v.size()); ++j) {
                if (!isGoal(a, v[static_cast<size_t>(j)].r, v[static_cast<size_t>(j)].c)) {
                    allGoal = false; break;
                }
            }
            if (allGoal) { cut = i; break; }
        }
        if (cut < static_cast<int>(v.size())) v.resize(static_cast<size_t>(cut + 1));
    }

    std::vector<Plan> plans(static_cast<size_t>(numAgents));
    for (int a = 0; a < numAgents; ++a) {
        auto& P = plans[static_cast<size_t>(a)];
        auto& v = perAgent[static_cast<size_t>(a)];
        P.waypoints.reserve(v.size());
        for (const auto& s : v) {
            P.waypoints.push_back(Waypoint{ grid.cellToWorld({s.c, s.r}),
                                            static_cast<float>(s.t) });
        }
    }
    return plans;
}

PlanResult SCRIMP_Binary::plan(const PlanRequest& req){
    PlanResult out;
    out.ids = req.ids;
    out.status = PlanStatus::Error;
    out.plans.clear();
    out.diag = {};

    const IDiscreteGrid* grid = req.env.gridCapability();
    if (!grid) {
        Logger::instance().error("SCRIMP: env is not IDiscreteGrid.");
        return out;
    }

    // Per-call overrides (don’t mutate defaults)
    SCRIMP_BinaryConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    const double time_limit_s = resolvePlannerTimeLimit(
        req.time_limit_s, cfg.cutoff_s);
    const int    seed         = (req.rng_seed ? static_cast<int>(*req.rng_seed) : cfg.seed);

    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("SCRIMP: ids/sgs size mismatch.");
        return out;
    }

    const auto t0 = std::chrono::steady_clock::now();

    util::TempDirGuard tmp(fs::temp_directory_path(), "scrimp_", cfg.keepTemps);
    const fs::path base = tmp.path();

    // -------- Files for python driver -------------------------------------
    const fs::path gridTxt  = base / "grid.txt";
    if (!writeGridTxt(*grid, gridTxt)) {
        Logger::instance().error("SCRIMP: failed to write grid.txt");
        return out;
    }

    std::vector<std::pair<int,int>> starts_xy, goals_xy; // (x,y) = (col,row)
    starts_xy.reserve(req.sgs.size());
    goals_xy.reserve(req.sgs.size());

    for (const auto& sg : req.sgs) {
        const auto start = grid->worldToCell(sg.start);
        const auto goal = grid->worldToCell(sg.goal);
        if (grid->checkIfCellOutOfBound(start) || grid->checkIfCellOutOfBound(goal)) {
            Logger::instance().error("SCRIMP: start/goal out of grid bounds.");
            return out;
        }
        starts_xy.emplace_back(start.c, start.r);
        goals_xy.emplace_back(goal.c, goal.r);
    }

    const fs::path startsCsv = base / "starts_xy.csv";
    const fs::path goalsCsv  = base / "targets_xy.csv";
    if (!writeXYCsv(starts_xy, startsCsv) || !writeXYCsv(goals_xy, goalsCsv)) {
        Logger::instance().error("SCRIMP: failed to write XY CSVs");
        return out;
    }

    const fs::path outCsv = base / "paths.csv";

    // goal cells (r,c) for trimming
    std::vector<std::pair<int,int>> goalCell;
    goalCell.reserve(goals_xy.size());
    for (const auto& gxy : goals_xy) goalCell.emplace_back(gxy.second, gxy.first); // (y,x)->(r,c)

    const int k = static_cast<int>(req.ids.size());
    cfg.seed = seed;
    const auto rr = runSolver(
        base, gridTxt, startsCsv, goalsCsv, outCsv, k, time_limit_s, cfg);

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!rr.ok) {
        out.status = rr.timed_out ? PlanStatus::Timeout : PlanStatus::Error;
        if (cfg.keepTemps) { Logger::instance().info("SCRIMP: temp kept at " + base.string()); tmp.set_keep(true); }
        return out;
    }

    out.plans  = parseCsvPaths(*grid, k, outCsv, goalCell);

    // Scale timestamps from steps → seconds if requested
    if (!out.plans.empty() && cfg.step_dt_s != 1.0) {
        for (auto& P : out.plans)
            for (auto& wp : P.waypoints)
                wp.t = static_cast<float>(wp.t * cfg.step_dt_s);
    }

    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;

    if (cfg.keepTemps) {
        Logger::instance().info("SCRIMP: temp kept at " + base.string());
        tmp.set_keep(true);
    }
    return out;
}
