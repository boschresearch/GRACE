// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>
#include <box2d/math_functions.h>

// A grid in world meters with (c,r) cells and uniform cellLength
struct IDiscreteGrid {
    struct GridCell {int c; int r;};
    virtual ~IDiscreteGrid() = default;
    [[nodiscard]] virtual int   gridWidthInCells()   const = 0;
    [[nodiscard]] virtual int   gridHeightInCells()  const = 0;
    [[nodiscard]] virtual float   gridWidthInMeter()   const = 0;
    [[nodiscard]] virtual float   gridHeightInMeter()  const = 0;
    [[nodiscard]] virtual float cellLength()   const = 0;
    [[nodiscard]] virtual bool  checkIfCellOutOfBound(GridCell cell) const = 0;
    [[nodiscard]] virtual bool  checkIfCellBlocked(GridCell cell) const = 0;
    [[nodiscard]] virtual b2Vec2 cellToWorld(GridCell cell) const = 0;
    [[nodiscard]] virtual GridCell worldToCell(b2Vec2 p) const = 0;
    [[nodiscard]] virtual int gridCellToOccupancyIndex(GridCell cell) const = 0;
    [[nodiscard]] virtual GridCell occupancyIndexToGridCell(int i) const = 0;
    [[nodiscard]] virtual double euclideanDistance(GridCell c1, GridCell c2) const = 0;
    [[nodiscard]] virtual const std::vector<bool>& occupancy() const = 0;
};

struct IRoadmap {
    using NodeId = int;
    virtual ~IRoadmap() = default;
    [[nodiscard]] virtual int    numNodes() const = 0;
    [[nodiscard]] virtual b2Vec2 nodePos(NodeId n) const = 0;
    [[nodiscard]] virtual const std::vector<NodeId>& adj(NodeId n) const = 0;
    [[nodiscard]] virtual float  edgeLength(NodeId u, NodeId v) const = 0;

    // convenience
    [[nodiscard]] virtual NodeId nearestNode(b2Vec2 p) const = 0;
};

struct IContinuousSpace {
    virtual ~IContinuousSpace() = default;

    // true if segment hits an obstacle/wall
    [[nodiscard]] virtual bool raycast(b2Vec2 p1, b2Vec2 p2) const = 0;

    // quick uniform sampling in bounds, rejected if colliding (simple baseline)
    [[nodiscard]] virtual std::vector<b2Vec2> sampleFree(int n) const = 0;

    // for configuration-space inflation (optional)
    virtual void setRobotRadius(float r) = 0;
};