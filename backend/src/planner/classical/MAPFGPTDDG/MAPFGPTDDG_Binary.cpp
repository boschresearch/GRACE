// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <chrono>
#include <fstream>
#include <sstream>

#include "../inc/planner/classical/MAPFGPTDDG/MAPFGPTDDG_Binary.h"
#include "../inc/env/ICapabilities.h"
#include "../inc/io/GridMovingAI.h"
#include "../inc/util/Logger.h"
#include "../inc/util/ProcRunner.h"
#include "../inc/util/TempDirGuard.h"

namespace fs = std::filesystem;

// ---------- helpers to talk to the python driver ------------------------------
namespace {

    std::vector<std::string> buildArgs(const fs::path& gridTxt,
                                       const fs::path& startsCsv,
                                       const fs::path& goalsCsv,
                                       const fs::path& outCsv,
                                       const int k,
                                       const MAPFGPTDDG_BinaryConfig& cfg){
        std::vector<std::string> a{
            cfg.driver,
            "--grid_file",   gridTxt.string(),
            "--starts_xy",   startsCsv.string(),
            "--targets_xy",  goalsCsv.string(),
            "--k",           std::to_string(k),
            "--outputPaths", outCsv.string(),
            "--model",       cfg.model,
            "--device",      cfg.device,
            "--max_steps",   std::to_string(cfg.max_steps),
            "--seed",        std::to_string(cfg.seed)
        };
        if (cfg.svg_out && cfg.save_svg) {
            a.emplace_back("--svg");
            a.push_back(cfg.svg_out.value());
        }
        return a;
    }

    bool writeGridTxt(const IDiscreteGrid& grid, const fs::path& path) {
        std::ofstream out(path);
        if (!out) return false;
        const int rows = grid.gridHeightInCells();
        const int cols = grid.gridWidthInCells();
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) out << (grid.checkIfCellBlocked({c,r})? '#' : '.');
            out << '\n';
        }
        return static_cast<bool>(out);
    }

    bool writeXYCsv(const std::vector<IDiscreteGrid::GridCell>& xy, const fs::path& path) {
        std::ofstream out(path);
        if (!out) return false;
        for (auto [x, y] : xy) out << x << ',' << y << '\n'; // x=col, y=row
        return static_cast<bool>(out);
    }

} // namespace

// ---------- class statics -----------------------------------------------------
void MAPFGPTDDG_Binary::configure(const PlannerSettings& s) {
    applySettings(s, m_cfg);
}

void MAPFGPTDDG_Binary::applySettings(const PlannerSettings& s, MAPFGPTDDG_BinaryConfig& cfg) {
    if (const auto v = s.get<std::string>("python_exe"))   cfg.python_exe = *v;
    if (const auto v = s.get<std::string>("driver"))       cfg.driver = *v;

    if (const auto v = s.get<std::string>("model"))        cfg.model = *v;
    if (const auto v = s.get<std::string>("device"))       cfg.device = *v;
    if (const auto v = s.get<int>("max_steps"))            cfg.max_steps = *v;
    if (const auto v = s.get<int>("seed"))                 cfg.seed = *v;
    if (const auto v = s.get<double>("cutoff_s"))           cfg.cutoff_s = *v;
    if (const auto v = s.get<bool>("save_svg"))            cfg.save_svg = *v;

    if (const auto v = s.get<bool>("keepTemps"))           cfg.keepTemps = *v;
    if (const auto v = s.get<double>("step_dt_s"))         cfg.step_dt_s = *v;
}

// ---------- run python driver -------------------------------------------------
MAPFGPTDDG_Binary::RunResult MAPFGPTDDG_Binary::runSolver(const fs::path& workdir,
                             const fs::path& gridTxt,
                             const fs::path& startsCsv,
                             const fs::path& goalsCsv,
                             const fs::path& outCsv,
                             const int k,
                             const double time_limit_s,
                             const MAPFGPTDDG_BinaryConfig& cfg) const{
    const fs::path outPath  = workdir / "mapfgptddg_stdout.txt";
    const fs::path errPath  = workdir / "mapfgptddg_stderr.txt";

    const auto args = buildArgs(gridTxt, startsCsv, goalsCsv, outCsv, k, cfg);
    const auto timeout_ms = std::chrono::milliseconds(static_cast<long>(time_limit_s * 1000.0) + 500);

    const auto res = util::runWithTimeout(cfg.python_exe, args, workdir, outPath, errPath, timeout_ms);
    return RunResult{ res.exit_code == 0 && !res.timed_out, res.timed_out, res.exit_code };
}

// ---------- parse CSV -> Plans ------------------------------------------------
std::vector<Plan> MAPFGPTDDG_Binary::parseCsvPaths(const IDiscreteGrid& grid,
    const std::vector<StartGoal>& sgs, const fs::path& outCsv, const double step_dt_s){
    std::ifstream in(outCsv);
    if (!in) {
        Logger::instance().error("MAPFGPTDDG: paths CSV missing.");
        return {};
    }

    const int K = static_cast<int>(sgs.size());
    std::vector<std::vector<Waypoint>> perAgent(static_cast<size_t>(K));

    std::string line;
    const int maxAgent = K - 1;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        // accept both "a,t,x,y" and any whitespace-separated
        for (char& ch : line) if (ch == ',') ch = ' ';
        std::istringstream ss(line);
        int a=-1, t_val=0, x=0, y=0;
        if (!(ss >> a >> t_val >> x >> y)) {
            Logger::instance().warn("MAPFGPTDDG: bad CSV line: " + line);
            continue;
        }
        if (a < 0 || a > maxAgent) continue;
        const int r = y, c = x; // CSV writes (x,y); grid uses (r=y, c=x)
        if (grid.checkIfCellOutOfBound({c,r})) {
            Logger::instance().warn("MAPFGPTDDG: out-of-bounds cell (" +
                                    std::to_string(r) + "," + std::to_string(c) + ")");
            continue;
        }
        const auto t_sec = static_cast<float>(t_val * step_dt_s);
        perAgent[static_cast<size_t>(a)].push_back(Waypoint{grid.cellToWorld({c, r}), t_sec});
    }

    const auto sgsGrid = grace::io::detail::toGridStartGoals(grid, sgs);
    auto isGoal = [&](const int a, const IDiscreteGrid::GridCell cell)->bool{
        const auto [sc, sr, gc, gr] = sgsGrid[static_cast<size_t>(a)];
        return cell.r == gr && cell.c == gc;
    };

    // sort by time and trim trailing idle-at-goal runs (keep first index of final run)
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

// ---------- main entry --------------------------------------------------------
PlanResult MAPFGPTDDG_Binary::plan(const PlanRequest& req)
{
    PlanResult out;
    out.ids = req.ids;
    out.plans.clear();
    out.diag = {};
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("MAPFGPTDDG: ids.size() != sgs.size().");
        out.status = PlanStatus::Error;
        return out;
    }

    // Per-call overrides
    MAPFGPTDDG_BinaryConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    const IDiscreteGrid* grid = req.env.gridCapability();
    if (!grid) {
        Logger::instance().error("MAPFGPTDDG: Environment does not support IDiscreteGrid.");
        out.status = PlanStatus::Error;
        return out;
    }

    const auto t0 = std::chrono::steady_clock::now();

    util::TempDirGuard tmp(fs::temp_directory_path(), "mapfgptddg_", cfg.keepTemps);
    const fs::path base = tmp.path();

    // Build grid + XY CSVs
    const fs::path gridTxt   = base / "grid.txt";
    if (!writeGridTxt(*grid, gridTxt)) {
        Logger::instance().error("MAPFGPTDDG: failed to write grid.txt");
        return out;
    }

    std::vector<IDiscreteGrid::GridCell> starts_cell, goals_cell;
    starts_cell.reserve(req.sgs.size());
    goals_cell.reserve(req.sgs.size());
    for (const auto&[start, goal] : req.sgs) {
        starts_cell.push_back(grid->worldToCell(start));
        goals_cell.push_back(grid->worldToCell(goal));
    }

    const fs::path startsCsv = base / "starts_xy.csv";
    const fs::path goalsCsv  = base / "targets_xy.csv";
    if (!writeXYCsv(starts_cell, startsCsv) || !writeXYCsv(goals_cell, goalsCsv)) {
        Logger::instance().error("MAPFGPTDDG: failed to write XY CSVs");
        return out;
    }

    const fs::path outCsv    = base / "paths.csv";
    cfg.svg_out = cfg.save_svg ? std::optional(base / "episode.svg") : std::nullopt;

    const int k = static_cast<int>(req.ids.size());
    const double time_limit_s = resolvePlannerTimeLimit(
        req.time_limit_s, cfg.cutoff_s);
    const auto rr = runSolver(
        base, gridTxt, startsCsv, goalsCsv, outCsv, k, time_limit_s, cfg);

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!rr.ok) {
        out.status = rr.timed_out ? PlanStatus::Timeout : PlanStatus::Error;
        if (cfg.keepTemps) { Logger::instance().info("MAPFGPTDDG: temp kept at " + base.string()); tmp.set_keep(true); }
        return out;
    }

    out.plans  = parseCsvPaths(*grid, req.sgs, outCsv, cfg.step_dt_s);
    if (cfg.keepTemps) {
        Logger::instance().info("MAPFGPTDDG: temp kept at " + base.string());
        tmp.set_keep(true);
    }
    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;

    return out;
}
