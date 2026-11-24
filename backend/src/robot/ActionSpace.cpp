// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/robot/ActionSpace.h"
#include <cstdlib>
#include <algorithm>
#include <cmath>

// --------- Bresenham line-of-sight on the occupancy grid ----------
static inline int sgn(int x) { return (x>0) - (x<0); }

// Returns true if *all* cells strictly between (r0,c0) and (r1,c1) are free,
// and the end cell (r1,c1) is free. Start cell is assumed free by caller.
bool GridAnyActionSpace::losFree(const IDiscreteGrid& G, int r0, int c0, int r1, int c1, bool allowDiag)
{
    if (r1<0 || r1>=G.gridHeightInCells() || c1<0 || c1>=G.gridWidthInCells()) return false;
    if (G.checkIfCellBlocked({c1,r1})) return false;
    auto blockedOrOOB = [&](int rr, int cc){
    return (rr<0 || rr>=G.gridHeightInCells() || cc<0 || cc>=G.gridWidthInCells()) || G.checkIfCellBlocked({cc,rr});
    };

    int dr = std::abs(r1 - r0);
    int dc = std::abs(c1 - c0);
    int sr = sgn(r1 - r0);
    int sc = sgn(c1 - c0);

    int r = r0, c = c0;

    // Bresenham variant: step through cells touching the discrete segment.
    if (dc >= dr) {
        int err = 2*dr - dc;
        for (int i=0; i<dc; ++i) {
            c += sc;
            if (err > 0) { r += sr; err -= 2*dc; }
            err += 2*dr;

            if (r == r1 && c == c1) break;
            if (r<0 || r>=G.gridHeightInCells() || c<0 || c>=G.gridWidthInCells()) return false;

            if (G.checkIfCellBlocked({c,r})) return false;

            if (!allowDiag && sr!=0 && sc!=0) {
                // forbid "cutting corners" through diagonal adjacency:
                // ensure at least one of the orthogonal neighbors is free
                if (blockedOrOOB(r - sr, c) && blockedOrOOB(r, c - sc)) return false;
            }
        }
    } else {
        int err = 2*dc - dr;
        for (int i=0; i<dr; ++i) {
            r += sr;
            if (err > 0) { c += sc; err -= 2*dr; }
            err += 2*dc;

            if (r == r1 && c == c1) break;
            if (r<0 || r>=G.gridHeightInCells() || c<0 || c>=G.gridWidthInCells()) return false;

            if (G.checkIfCellBlocked({c,r})) return false;

            if (!allowDiag && sr!=0 && sc!=0) {
                if (blockedOrOOB(r - sr, c) && blockedOrOOB(r, c - sc)) return false;
            }
        }
    }

    // end cell was already checked free at the top
    return true;
}

void GridAnyActionSpace::enumerate(const IDiscreteGrid& grid, int r, int c,
                                   std::vector<GridMove>& out) const
{
    out.clear();
    const float L = grid.cellLength();
    if (m_p.includeWait) out.push_back(GridMove{r, c, m_p.waitCost, true});

    // Enumerate candidates. Default: all free cells (watch branching!).
    const int H = grid.gridHeightInCells();
    const int W = grid.gridWidthInCells();

    // Optional hard cap to keep branching manageable.
    int emitted = 0;
    const bool capEnabled = (m_p.hardCapDestinations > 0);

    // Range bound for “any tile”
    const int rMin = (m_p.maxRangeCells < 0) ? 0        : std::max(0, r - m_p.maxRangeCells);
    const int rMax = (m_p.maxRangeCells < 0) ? (H - 1)  : std::min(H - 1, r + m_p.maxRangeCells);
    const int cMin = (m_p.maxRangeCells < 0) ? 0        : std::max(0, c - m_p.maxRangeCells);
    const int cMax = (m_p.maxRangeCells < 0) ? (W - 1)  : std::min(W - 1, c + m_p.maxRangeCells);

    for (int rr = rMin; rr <= rMax; ++rr) {
        for (int cc = cMin; cc <= cMax; ++cc) {
            if (rr == r && cc == c) continue;
            if (grid.checkIfCellBlocked({cc,rr})) continue;

            if (m_p.requireLineOfSight) {
                if (!losFree(grid, r, c, rr, cc, m_p.includeDiagonalsInLoS)) continue;
            }
            const float dx = float(cc - c) * L;
            const float dy = float(rr - r) * L;
            const float cost = std::sqrt(dx*dx + dy*dy);

            out.push_back(GridMove{rr, cc, cost, false});
            if (capEnabled && ++emitted >= m_p.hardCapDestinations) return;
        }
    }
}

// --------- Free-space sampled rays ----------
void FreeSampledRaysActionSpace::enumerate(const IContinuousSpace& space, b2Vec2 p,
                                           std::vector<FreeMove>& out) const
{
    out.clear();
    if (m_p.includeWait) out.push_back(FreeMove{p, m_p.waitCost, true});

    const int   K = std::max(1, m_p.headings);
    const float L = std::max(1e-6f, m_p.step);

    for (int k=0; k<K; ++k) {
        const float theta = (2.0f * 3.14159265358979323846f) * (float(k) / float(K));
        b2Vec2 q { p.x + L * std::cos(theta), p.y + L * std::sin(theta) };

        // Validate by raycast; if hit, skip
        if (space.raycast(p, q)) continue;

        out.push_back(FreeMove{q, L, false});
    }
}