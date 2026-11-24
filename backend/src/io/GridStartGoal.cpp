// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/io/GridStartGoal.h"
#include "../../inc/env/ICapabilities.h"

namespace grace::io::detail {

    StartGoal toStartGoal(const IDiscreteGrid& G, const GridStartGoal& gg) {
        return StartGoal{G.cellToWorld({gg.sc, gg.sr}),G.cellToWorld({gg.gc, gg.gr})};
    }

    GridStartGoal toGridStartGoal(const IDiscreteGrid& G, const StartGoal& sg) {

        const auto [sc, sr] = G.worldToCell(sg.start);
        const auto [gc, gr] = G.worldToCell(sg.goal);
        return GridStartGoal{sc, sr, gc, gr};
    }

    [[nodiscard]] std::vector<StartGoal> toStartGoals(const IDiscreteGrid& G, const std::vector<GridStartGoal>& v){
        std::vector<StartGoal> out;
        out.reserve(v.size());
        for (const auto& gg : v) out.push_back(toStartGoal(G, gg));
        return out;
    }

    [[nodiscard]] std::vector<GridStartGoal> toGridStartGoals(const IDiscreteGrid& G, const std::vector<StartGoal>& v){
        std::vector<GridStartGoal> out;
        out.reserve(v.size());
        for (const auto& sg : v) out.push_back(toGridStartGoal(G, sg));
        return out;
    }

}