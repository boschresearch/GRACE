// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "BaseEnvironment.h"
#include "ICapabilities.h"
#include <vector>

class GridEnvironment final : public BaseEnvironment, public IDiscreteGrid {
public:
    // IDiscreteGrid
    [[nodiscard]] int gridWidthInCells() const override { return numWidthCells; }
    [[nodiscard]] int gridHeightInCells() const override { return numHeightCells; }
    [[nodiscard]] float gridWidthInMeter() const override {return static_cast<float>(numWidthCells) * cellLengthInMeter;}
    [[nodiscard]] float gridHeightInMeter() const override {return static_cast<float>(numHeightCells) * cellLengthInMeter;}
    [[nodiscard]] float cellLength() const override {return cellLengthInMeter;}
    [[nodiscard]] bool checkIfCellOutOfBound(const GridCell cell) const override {
        if (cell.r < 0 || cell.r >= numHeightCells || cell.c < 0 || cell.c >= numWidthCells) return true;
        return false;
    }
    [[nodiscard]] bool checkIfCellBlocked(const GridCell cell) const override {
        if (checkIfCellOutOfBound(cell)) return true;
        return m_occupancy[gridCellToOccupancyIndex(cell)];
    }
    [[nodiscard]] b2Vec2 cellToWorld(const GridCell cell) const override {
        // 0.5f is not a magic number -- the origin is the top left corner of the cell, so 0.5 cell shift
        return b2Vec2{(static_cast<float>(cell.c) + 0.5f) * cellLengthInMeter,
            (static_cast<float>(cell.r) + 0.5f) * cellLengthInMeter};
    }
    [[nodiscard]] GridCell worldToCell(const b2Vec2 p) const override {
        return GridCell{static_cast<int>(p.x / cellLengthInMeter), static_cast<int>(p.y / cellLengthInMeter)};
    }
    [[nodiscard]] int gridCellToOccupancyIndex(const GridCell cell) const override {
        return cell.r * numWidthCells + cell.c;
    }
    [[nodiscard]] GridCell occupancyIndexToGridCell(const int i) const override {
        return {i % numWidthCells, i / numWidthCells};
    }
    [[nodiscard]] double euclideanDistance(const GridCell c1, const GridCell c2) const override {
        return std::hypot(static_cast<double>(c1.c - c2.c), static_cast<double>(c1.r - c2.r));
    }
    [[nodiscard]] const std::vector<bool>& occupancy() const override { return m_occupancy; }

    // BaseEnvironment - specific functions
    [[nodiscard]] Bounds boundsInMeter() const override { return {gridWidthInMeter(), gridHeightInMeter()}; }
    [[nodiscard]] EnvironmentType getEnvironmentType() const override { return EnvironmentType::GridEnvironment; }
    [[nodiscard]] const IDiscreteGrid* gridCapability() const noexcept override { return this; }

    // GridEnvironment - specific functions
    void makeGridEnvironment(
        int NumWidthCells,
        int NumHeightCells,
        float cellLength,
        const std::vector<bool>& occ,
        float boundaryScale = 0.5f);

    [[nodiscard]] const std::vector<bool>& getOccupancy() const { return m_occupancy; }
    void setOccupancy(const int occupancyIndex, const bool blocked) {m_occupancy[occupancyIndex] = blocked; }
    void setOccupancy(const int c, const int r, const bool blocked) {m_occupancy[gridCellToOccupancyIndex({c,r})] = blocked; }
    void setOccupancyRect(int c0, int r0, int c1, int r1, bool blocked);

    void bulkSetBlocked(const std::vector<std::tuple<int, int, bool>> &ops);

private:
    int numWidthCells{0}, numHeightCells{0};
    float cellLengthInMeter{1.0f};
    std::vector<bool> m_occupancy;

    void resetDerivedState() noexcept override;
    void addBodyFromOccupancy();
};
