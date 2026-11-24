// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <chrono>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <regex>

#include "../inc/planner/classical/CBSH2RTC/CBSH2RTC_Binary.h"
#include "../inc/env/ICapabilities.h"
#include "../inc/io/GridMovingAI.h"
#include "../inc/util/Logger.h"
#include "../inc/util/ProcRunner.h"
#include "../inc/util/TempDirGuard.h"

namespace fs = std::filesystem;

namespace {

    // command-line args builder
    std::vector<std::string> buildArgs(const fs::path& mapPath,
                                       const fs::path& scenPath,
                                       const fs::path& statsCsv,
                                       const fs::path& pathsOut,
                                       const int k,
                                       const CBSH2RTC_BinaryConfig& cfg,
                                       const double time_limit_s){
        std::vector<std::string> a{
            "-m", mapPath.string(),
            "-a", scenPath.string(),
            "-o", statsCsv.string(),
            "--outputPaths", pathsOut.string(),
            "-k", std::to_string(k),
            "-t", std::to_string(time_limit_s),
            "--screen", std::to_string(cfg.screen),
            "--heuristics", cfg.heuristics,
            "--rectangleReasoning", cfg.rectangleReasoning,
            "--corridorReasoning",  cfg.corridorReasoning,
            "--bypass", cfg.bypass ? "1" : "0",
            "--prioritizingConflicts", cfg.prioritizingConflicts ? "1" : "0",
            "--targetReasoning", cfg.targetReasoning ? "1" : "0",
            "--mutexReasoning",  cfg.mutexReasoning ? "1" : "0",
            "--restart", std::to_string(cfg.restarts)
        };
        return a;
    }

} // namespace

void CBSH2RTC_Binary::applySettings(const PlannerSettings& s, CBSH2RTC_BinaryConfig& cfg) {
    if (const auto v = s.get<std::string>("binary")) cfg.binary  = *v;
    if (const auto v = s.get<bool>("keepTemps"))    cfg.keepTemps= *v;
    applyCommonSettings(s,cfg);
}

CBSH2RTC_Binary::RunResult CBSH2RTC_Binary::runSolver(const fs::path& dir,
                           const fs::path& mapPath,
                           const fs::path& scenPath,
                           const fs::path& pathsOut,
                           const int k,
                           const double time_limit_s) const{
    const fs::path outPath  = dir / "cbs_stdout.txt";
    const fs::path errPath  = dir / "cbs_stderr.txt";
    const fs::path statsCsv = dir / "stats.csv";

    const auto args = buildArgs(mapPath, scenPath, statsCsv, pathsOut, k, m_cfg, time_limit_s);
    const auto timeout_ms = std::chrono::milliseconds(static_cast<long>(time_limit_s * 1000.0) + 500);

    const auto res = util::runWithTimeout(m_cfg.binary, args, dir, outPath, errPath, timeout_ms);
    return RunResult{ res.exit_code == 0 && !res.timed_out, res.timed_out, res.exit_code };
}

std::vector<Plan> CBSH2RTC_Binary::parsePathsFile(const IDiscreteGrid& grid, int K, const fs::path& file){
    std::ifstream in(file);
    if (!in) {
        Logger::instance().warn("CBSH2RTC(bin): cannot open paths file: " + file.string());
        return {};
    }

    std::vector<std::vector<std::pair<int,int>>> raw(K);
    std::size_t cur = 0;
    std::string line;
    std::regex pairAny(R"(\(\s*(-?\d+)\s*[,;\s]\s*(-?\d+)\s*\)|(-?\d+)\s*[,;\s]\s*(-?\d+))");

    auto ensure = [&](const std::size_t i){ if (raw.size() <= i) raw.resize(i+1); };

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line.rfind("Agent", 0) == 0 || (line.rfind("agent", 0) == 0)) {
            std::size_t colon = line.find(':');
            if (std::size_t posNum = line.find_first_of("0123456789"); posNum != std::string::npos) {
                try { cur = std::stoul(line.substr(posNum)); }
                catch(...) {}
            }
            ensure(cur);
            std::string rest = (colon == std::string::npos) ? std::string{} : line.substr(colon + 1);
            for (std::sregex_iterator it(rest.begin(), rest.end(), pairAny), end; it != end; ++it) {
                const std::smatch& m = *it;
                int a=0,b=0;
                if (m[1].matched) { a = std::stoi(m[1]); b = std::stoi(m[2]); }
                else               { a = std::stoi(m[3]); b = std::stoi(m[4]); }
                raw[cur].emplace_back(a,b);
            }
            continue;
        }
        ensure(cur);
        for (std::sregex_iterator it(line.begin(), line.end(), pairAny), end; it != end; ++it) {
            const std::smatch& m = *it;
            int a=0,b=0;
            if (m[1].matched) { a = std::stoi(m[1]); b = std::stoi(m[2]); }
            else               { a = std::stoi(m[3]); b = std::stoi(m[4]); }
            raw[cur].emplace_back(a,b);
        }
    }

    std::vector<Plan> plans(std::max(0, K));
    for (int i=0; i<K; ++i) {
        const auto& seq = (i < static_cast<int>(raw.size())) ? raw[i] : std::vector<std::pair<int,int>>{};
        Plan P;
        P.waypoints.reserve(seq.size());
        for (std::size_t t=0; t<seq.size(); ++t) {
            const auto [r, c] = seq[t];
            const b2Vec2 pos = grid.cellToWorld({c, r});
            P.waypoints.push_back(Waypoint{pos, static_cast<float>(t)}); // 1s per step default
        }
        plans[i] = std::move(P);
    }
    return plans;
}

PlanResult CBSH2RTC_Binary::plan(const PlanRequest& req){
    PlanResult out;
    out.ids = req.ids;
    out.plans.clear();
    out.diag = {};
    if (req.ids.size() != req.sgs.size()) {
        Logger::instance().error("[CBSH2RTC(bin)] ids.size() != sgs.size().");
        out.status = PlanStatus::Error;
        return out;
    }

    // Per-call overrides
    CBSH2RTC_BinaryConfig cfg = m_cfg;
    applySettings(req.settings, cfg);
    const double time_limit_s = resolvePlannerTimeLimit(req.time_limit_s, 60.0);
    const IDiscreteGrid* grid = req.env.gridCapability();
    if (!grid) {
        Logger::instance().error("[CBSH2RTC(bin)] Environment does not support IDiscreteGrid.");
        out.status = PlanStatus::Error;
        return out;
    }

    const auto t0 = std::chrono::steady_clock::now();

    util::TempDirGuard tmp(fs::temp_directory_path(), "cbsh2rtc_", cfg.keepTemps);
    const fs::path base = tmp.path();

    // Write map (.map) next to the .scen we’ll emit
    const fs::path mapPath = base / "map.map";
    if (!grace::io::writeGridMovingAIMap(*grid, mapPath)) {
        out.status = PlanStatus::Error;
        return out;
    }

    // Write a scenario
    std::vector<b2Vec2> starts_m, goals_m;
    starts_m.reserve(req.sgs.size());
    goals_m.reserve(req.sgs.size());
    for (const auto&[start, goal] : req.sgs) { starts_m.push_back(start); goals_m.push_back(goal); }

    const fs::path scenPath = base / "instance.scen";
    if (!grace::io::writeGridMovingAIScenario(
            *grid, scenPath, starts_m, goals_m)) {
        out.status = PlanStatus::Error;
        return out;
    }

    const fs::path outPaths = base / "paths.txt";

    const int k = static_cast<int>(req.ids.size());
    const auto rr = runSolver(base, mapPath, scenPath, outPaths, k, time_limit_s);

    const auto t1 = std::chrono::steady_clock::now();
    out.diag.runtime_s = std::chrono::duration<double>(t1 - t0).count();

    if (!rr.ok) {
        out.status = rr.timed_out ? PlanStatus::Timeout : PlanStatus::Error;
        if (cfg.keepTemps) {
            Logger::instance().info("CBSH2RTC(bin): temp kept at " + base.string());
            tmp.set_keep(true);
        }
        return out;
    }

    // Parse paths
    out.plans = parsePathsFile(*grid, k, outPaths);
    if (static_cast<int>(out.plans.size()) != k) {
        Logger::instance().warn("CBSH2RTC(bin): parsed plans count mismatch.");
    }

    if (cfg.keepTemps) {
        Logger::instance().info("CBSH2RTC(bin): temp kept at " + base.string());
        tmp.set_keep(true);
    }

    out.status = out.plans.empty() ? PlanStatus::Error : PlanStatus::Success;
    return out;
}
