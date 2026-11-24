// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/sim/GridConflictAuditor.h"
#include <cmath>

void GridConflictAuditor::reset() {
    m_prevCell.clear();
    m_lastTick = -1;
}

void GridConflictAuditor::tick(const float simTime, const IDiscreteGrid& grid, const RobotSystem& robots, Logger& log) {
    const int k = static_cast<int>(std::floor(simTime / m_tickSec));
    if (k == m_lastTick) return;

    // Build occNow (rc -> id) and idNowCell (id -> rc) in one pass
    std::unordered_map<std::pair<int,int>, b2BodyId, PairHash> occNow;
    std::unordered_map<b2BodyId, std::pair<int,int>, BodyIdHash, BodyIdEq> idNowCell;
    bool anyVertex=false, anyEdgeSwap=false;

    for (b2BodyId id : robots.robots()) {
        const b2Vec2 p = b2Body_GetPosition(id);
        const auto [c, r] = grid.worldToCell(p);
        if (r<0 || r>=grid.gridHeightInCells() || c<0 || c>=grid.gridWidthInCells()) continue;

        const auto rc = std::make_pair(r,c);
        idNowCell[id] = rc;

        if (auto it = occNow.find(rc); it != occNow.end() && !BodyIdEq{}(it->second, id)) {
            anyVertex = true; // two different ids mapped to same cell now
        } else {
            occNow.emplace(rc, id);
        }
    }

    // Build prevOcc (rc -> id) to detect swaps in O(N)
    std::unordered_map<std::pair<int,int>, b2BodyId, PairHash> prevOcc;
    prevOcc.reserve(m_prevCell.size());
    for (const auto&[fst, snd] : m_prevCell) prevOcc.emplace(snd, fst);

    // Edge swaps: A u->v AND B v->u
    for (const auto&[fst, snd] : m_prevCell) {
        const b2BodyId id   = fst;
        const auto&    rc0  = snd;              // prev cell of id
        auto itNow = idNowCell.find(id);
        if (itNow == idNowCell.end()) continue;
        const auto& rc1 = itNow->second;              // now cell of id

        // Who was at my current cell previously?
        auto itPartnerPrev = prevOcc.find(rc1);
        if (itPartnerPrev == prevOcc.end()) continue;
        const b2BodyId partner = itPartnerPrev->second;
        if (BodyIdEq{}(partner, id)) continue;

        // Is partner now at my previous cell?
        if (auto itPartnerNow = idNowCell.find(partner); itPartnerNow != idNowCell.end() && itPartnerNow->second == rc0) {
            anyEdgeSwap = true;
        }
    }

    if (anyVertex)  log.error("MAPF auditor: vertex conflict at tick " + std::to_string(k));
    if (anyEdgeSwap) log.error("MAPF auditor: edge swap at tick " + std::to_string(k));

    // rollover
    m_prevCell = std::move(idNowCell);
    m_lastTick = k;
}
