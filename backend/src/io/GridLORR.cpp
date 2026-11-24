// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/io/GridLORR.h"
#include "../../inc/util/Logger.h"
#include "../../inc/io/GridStartGoal.h"
#include <fmt/format.h>
#include <fstream>
#include <unordered_set>
#include <algorithm>

namespace grace::io {

    namespace detail {

        std::vector<int> readGridLORRIntList(const std::filesystem::path& path)
        {
            std::ifstream fin(path);
            std::vector<int> out;
            if (!fin) {
                Logger::instance().error("LORR: cannot open " + path.string());
                return out;
            }

            long long n = -1;
            if (!(fin >> n) || n < 0) {
                Logger::instance().error("LORR: first token invalid in " + path.string());
                return out;
            }

            out.reserve(static_cast<std::size_t>(n));
            for (long long i = 0; i < n; ++i) {
                int v = 0;
                if (!(fin >> v)) {
                    Logger::instance().warn("LORR: count mismatch in " + path.string());
                    break;
                }
                out.push_back(v);
            }

            // If there are trailing tokens, we ignore them (LORR lists are length-prefixed).
            return out;
        }

    } // namespace detail

    std::vector<StartGoal>
    readGridLORRAgentsTasks(const IDiscreteGrid& G,
                             const std::filesystem::path& agentsPath,
                             const std::filesystem::path& tasksPath,
                             const GridLORROptions& opt)
    {
        const int W = G.gridWidthInCells(), H = G.gridHeightInCells(), Ncells = W * H;

        const auto startsIdx = detail::readGridLORRIntList(agentsPath);
        const auto tasksIdx  = detail::readGridLORRIntList(tasksPath);

        if (startsIdx.empty() || tasksIdx.empty()) return {};

        const std::size_t K = std::min(opt.takeK, startsIdx.size());

        // Choose unique (or just first valid) task indices
        std::vector<int> chosenTasks; chosenTasks.reserve(K);
        std::unordered_set<int> used;
        if (opt.uniqueGoals) used.reserve(K * 2);

        for (int idx : tasksIdx) {
            if (chosenTasks.size() >= K) break;
            if (idx < 0 || idx >= Ncells) continue;
            if (opt.uniqueGoals && used.contains(idx)) continue;

            const int gr = idx / W, gc = idx % W;
            if (opt.validateAgainstGrid) {
                if (G.checkIfCellOutOfBound({gc, gr})) continue;
                if (opt.skipBlockedCells && G.checkIfCellBlocked({gc, gr})) continue;
            }
            chosenTasks.push_back(idx);
            if (opt.uniqueGoals) used.insert(idx);
        }

        if (chosenTasks.size() < K) {
            Logger::instance().warn(fmt::format("LORR: only {} unique goals available ({} requested).",
                                                chosenTasks.size(), K));
        }

        const std::size_t Kuse = std::min(K, chosenTasks.size());
        std::vector<StartGoal> out; out.reserve(Kuse);

        std::size_t added = 0;
        for (std::size_t i = 0; i < startsIdx.size() && added < Kuse; ++i) {
            const int s = startsIdx[i];
            if (s < 0 || s >= Ncells) continue;
            const int sr = s / W, sc = s % W;

            if (opt.validateAgainstGrid) {
                if (G.checkIfCellOutOfBound({sc, sr})) continue;
                if (opt.skipBlockedCells && G.checkIfCellBlocked({sc, sr})) continue;
            }

            const int gIdx = chosenTasks[added];
            const int gr = gIdx / W, gc = gIdx % W;

            out.push_back(detail::toStartGoal(G, detail::GridStartGoal{sc, sr, gc, gr}));
            ++added;
        }

        if (out.empty()) {
            Logger::instance().error("LORR: no valid agent/task pairs after filtering.");
        } else {
            Logger::instance().info(fmt::format("LORR: loaded {} agents.", out.size()));
        }
        return out;
    }

} // namespace grace::io
