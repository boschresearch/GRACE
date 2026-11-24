// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/sim/SpawnRobot.h"
#include "../../inc/util/Logger.h"
#include <fmt/format.h>

namespace grace::sim {

    std::vector<b2BodyId> spawnGridAgent(Simulation& sim,
                   const std::vector<StartGoal>& specs,
                   const SpawnOptions& opt){
        const auto* grid = sim.gridEnv();
        if (!grid) {
            Logger::instance().error("spawnGridCellRobot: only valid for Grid environments.");
            return {};
        }
        std::vector<b2BodyId> ids;
        if (specs.empty()) {
            Logger::instance().error("spawnGridAgent: no specs provided.");
        }

        const std::size_t N = specs.size();
        ids.reserve(N);

        for (std::size_t i = 0; i < N; ++i) {
            const auto& sg = specs[i];
            // Name generation (no names in StartGoal)
            const std::string name = fmt::format("{}{}", opt.baseName, ids.size() + 1);
            // Map start (meters) to grid cell
            const auto [sc, sr] = grid->worldToCell(sg.start);
            if (sr < 0 || sr >= grid->gridHeightInCells() || sc < 0 || sc >= grid->gridWidthInCells()) {
                Logger::instance().error(fmt::format(
                    "spawnGridAgent: start out of bounds for {} at (r={},c={})", name, sr, sc));
                continue;
            }
            if (grid->checkIfCellBlocked({sc, sr})) {
                Logger::instance().error(fmt::format(
                    "spawnGridAgent: start on blocked cell for {} at (r={},c={})", name, sr, sc));
                continue;
            }
            auto id = sim.spawnGridCellRobot(sc, sr, name);
                if (B2_IS_NULL(id)) {
                    Logger::instance().error(fmt::format(
                    "spawnGridAgent: Simulation refused to spawn {} at (r={},c={})", name, sr, sc));
                    continue;
                }
                ids.push_back(id);
            }

        if (!ids.empty()) {
            std::vector<b2Vec2> goals; goals.reserve(ids.size());
            for (std::size_t i = 0; i < ids.size(); ++i) {
                goals.push_back(specs[i].goal); // already meters
            }
            (void)sim.recordGoals(ids, goals);
        }

        Logger::instance().info(fmt::format("spawnGridAgent: spawned {} robot(s).", ids.size()));
        return ids;
    }

} // namespace grace::sim
