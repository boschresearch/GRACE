// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>
#include <cmath>
#include <box2d/box2d.h>
#include "../env/ICapabilities.h"

// ---------- Common action result types ----------
struct GridMove {
    int   r{}, c{};          // destination cell
    float cost{0.0f};        // meters (typically)
    bool  isWait{false};
};

struct RoadmapMove {
    IRoadmap::NodeId to{-1};
    float cost{0.0f};        // edge length (meters); wait uses the action space's waitCost
    bool  isWait{false};
};

struct FreeMove {
    b2Vec2 to{0.0f, 0.0f};   // destination in meters
    float  cost{0.0f};       // typically path length, wait=0
    bool   isWait{false};
};

// ---------- Grid Action Space ----------
class IGridActionSpace {
public:
    virtual ~IGridActionSpace() = default;
    virtual void enumerate(const IDiscreteGrid& grid, int r, int c,
                           std::vector<GridMove>& out) const = 0;
};

class Grid4ActionSpace final : public IGridActionSpace {
public:
    explicit Grid4ActionSpace(bool includeWait=true, float waitCost=1.0f)
        : m_includeWait(includeWait), m_waitCost(waitCost) {}

    void enumerate(const IDiscreteGrid& grid, int r, int c,
                   std::vector<GridMove>& out) const override
    {
        out.clear();
        const float d = grid.cellLength();
        if (m_includeWait) out.push_back(GridMove{r, c, m_waitCost, true});

        static const int dr[4] = {-1, +1,  0,  0};
        static const int dc[4] = { 0,  0, -1, +1};
        for (int k=0; k<4; ++k) {
            const int rr = r + dr[k], cc = c + dc[k];
            if (rr<0 || rr>=grid.gridHeightInCells() || cc<0 || cc>=grid.gridWidthInCells()) continue;
            if (grid.checkIfCellBlocked({cc,rr})) continue;
            out.push_back(GridMove{rr, cc, d, false});
        }
    }
private:
    bool  m_includeWait{true};
    float m_waitCost{1.0f};
};

class Grid8ActionSpace final : public IGridActionSpace {
public:
    explicit Grid8ActionSpace(bool includeWait=true, float waitCost=1.0f)
        : m_includeWait(includeWait), m_waitCost(waitCost) {}

    void enumerate(const IDiscreteGrid& grid, int r, int c,
                   std::vector<GridMove>& out) const override
    {
        out.clear();
        const float d  = grid.cellLength();
        const float dd = d * std::sqrt(2.0f);
        if (m_includeWait) out.push_back(GridMove{r, c, m_waitCost, true});

        static const int dr[8] = {-1,+1, 0, 0, -1,-1,+1,+1};
        static const int dc[8] = { 0, 0,-1,+1, -1,+1,-1,+1};
        for (int k=0; k<8; ++k) {
            const int rr = r + dr[k], cc = c + dc[k];
            if (rr<0 || rr>=grid.gridHeightInCells() || cc<0 || cc>=grid.gridWidthInCells()) continue;
            if (grid.checkIfCellBlocked({cc,rr})) continue;
            const bool diag = (dr[k] != 0 && dc[k] != 0);
            out.push_back(GridMove{rr, cc, diag ? dd : d, false});
        }
    }
private:
    bool  m_includeWait{true};
    float m_waitCost{1.0f};
};

// "Any tile" action space.
// By default it enumerates all free destinations within optional maxRangeCells
// and only if there is line-of-sight through free cells (Bresenham).
class GridAnyActionSpace final : public IGridActionSpace {
public:
    struct Params {
        bool  includeWait        = true;
        float waitCost           = 1.0f;
        bool  requireLineOfSight = true; // Bresenham across free cells
        int   maxRangeCells      = -1;   // -1 => unlimited
        bool  includeDiagonalsInLoS = true; // affects how you consider adjacency in LOS (see impl)
        // If range is unlimited on large grids, consider setting a hard cap to keep branching sane.
        int   hardCapDestinations = 4096; // 0 => no cap
    };

    explicit GridAnyActionSpace(Params p) : m_p(p) {}

    void enumerate(const IDiscreteGrid& grid, int r, int c,
                   std::vector<GridMove>& out) const override;

private:
    Params m_p;
    static bool losFree(const IDiscreteGrid& grid, int r0, int c0, int r1, int c1, bool allowDiag);
};

// ---------- Roadmap Action Space ----------
class IRoadmapActionSpace {
public:
    virtual ~IRoadmapActionSpace() = default;
    virtual void enumerate(const IRoadmap& rm, IRoadmap::NodeId node,
                           std::vector<RoadmapMove>& out) const = 0;
};

class RoadmapAdjacencyActionSpace final : public IRoadmapActionSpace {
public:
    explicit RoadmapAdjacencyActionSpace(bool includeWait=true, float waitCost=1.0f)
        : m_includeWait(includeWait), m_waitCost(waitCost) {}

    void enumerate(const IRoadmap& rm, IRoadmap::NodeId node,
                   std::vector<RoadmapMove>& out) const override
    {
        out.clear();
        if (m_includeWait) out.push_back(RoadmapMove{node, m_waitCost, true});
        for (auto v : rm.adj(node)) {
            out.push_back(RoadmapMove{v, rm.edgeLength(node, v), false});
        }
    }
private:
    bool  m_includeWait{true};
    float m_waitCost{1.0f};
};

// ---------- Free-space Action Space ----------
class IFreeActionSpace {
public:
    virtual ~IFreeActionSpace() = default;
    virtual void enumerate(const IContinuousSpace& space, b2Vec2 p,
                           std::vector<FreeMove>& out) const = 0;
};

// Sampled steering primitives: K headings × step length L (meters).
// Each ray is validated using IContinuousSpace::raycast.
class FreeSampledRaysActionSpace final : public IFreeActionSpace {
public:
    struct Params {
        int   headings = 16;      // number of angular samples in [0, 2π)
        float step     = 0.5f;    // meters to advance
        bool  includeWait = true;
        float waitCost = 1.0f;
        // Optional: restrict to a view cone around preferred heading later
    };

    explicit FreeSampledRaysActionSpace(Params p) : m_p(p) {}

    void enumerate(const IContinuousSpace& space, b2Vec2 p,
                   std::vector<FreeMove>& out) const override;

private:
    Params m_p;
};