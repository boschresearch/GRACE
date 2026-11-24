// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/env/GridEnvironment.h"
#include "../inc/util/Logger.h"
#include <fmt/format.h>
#include <cmath>

/*** public functions ***/

void GridEnvironment::makeGridEnvironment(
    const int NumWidthCells,
    const int NumHeightCells,
    const float cellLength,
    const std::vector<bool>& occ,
    const float boundaryScale)
{
    if (NumWidthCells <= 0 || NumHeightCells <= 0) {
        Logger::instance().error("setOccupancyGrid: non-positive grid size.");
        return;
    }
    if (!std::isfinite(cellLength) || cellLength <= 0.0f ||
        !std::isfinite(boundaryScale) || boundaryScale <= 0.0f) {
        Logger::instance().error(
            "setOccupancyGrid: cell length and boundary scale must be finite and positive.");
        return;
    }
    if (const std::size_t need = static_cast<std::size_t>(NumWidthCells) * static_cast<std::size_t>(NumHeightCells);
        occ.size() != need) {
        Logger::instance().error("setOccupancyGrid: occ.size() mismatch with W*H.");
        return;
    }
    resetEnvironment();
    numWidthCells = NumWidthCells;
    numHeightCells = NumHeightCells;
    cellLengthInMeter = cellLength;
    m_occupancy = occ;

    addBoundaryWalls(boundaryScale * cellLengthInMeter);
    addBodyFromOccupancy();
    Logger::instance().info(fmt::format(
        "Grid applied: {}x{} cells @ {:.3f} m ({:.3f}×{:.3f} m).",
        numWidthCells, numHeightCells, cellLengthInMeter, gridWidthInMeter(), gridHeightInMeter()
        ));
}

void GridEnvironment::setOccupancyRect(int c0, int r0, int c1, int r1, const bool blocked)
{
    if (r0 > r1) std::swap(r0, r1);
    if (c0 > c1) std::swap(c0, c1);
    // clamp to grid bounds
    c0 = std::max(0, std::min(c0, numWidthCells - 1));
    c1 = std::max(0, std::min(c1, numWidthCells - 1));
    r0 = std::max(0, std::min(r0, numHeightCells - 1));
    r1 = std::max(0, std::min(r1, numHeightCells - 1));

    for (int r = r0; r <= r1; ++r) for (int c = c0; c <= c1; ++c) setOccupancy(c, r, blocked);

}

void GridEnvironment::bulkSetBlocked(const std::vector<std::tuple<int,int,bool>>& ops) {
    bool changed = false;
    for (const auto& t : ops) {
        const int r = std::get<0>(t);
        const int c = std::get<1>(t);
        const bool blk = std::get<2>(t);
        if (checkIfCellOutOfBound({c,r})) continue;
        const size_t k = gridCellToOccupancyIndex({c,r});
        if (m_occupancy[k] != blk) {setOccupancy(k, blk); changed = true;}
    }
    if (!changed) return;

    // Rebuild obstacle bodies from occupancy
    clearObstacles();
    addBodyFromOccupancy();
}

/*** private functions ***/

void GridEnvironment::resetDerivedState() noexcept {
    numWidthCells = 0;
    numHeightCells = 0;
    cellLengthInMeter = 1.0f;
    m_occupancy.clear();
}

void GridEnvironment::addBodyFromOccupancy() {
    // Re-create obstacles from m_occ.
    const float half = 0.5f * cellLengthInMeter;
    for (int r = 0; r < numHeightCells; ++r) {
        for (int c = 0; c < numWidthCells; ++c) {
            if (m_occupancy[gridCellToOccupancyIndex({c,r})]) {
                addBoxBody(cellToWorld({c,r}), half, half, 0, Type::BodyType::Obstacle);
            }
        }
    }
}
