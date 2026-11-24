// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "PlatformBase.h"

#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "env/ContinuousEnvironment.h"

#include <cmath>

bool PlatformBase::OnEscape()
{
    // Cancel any pending paint/edge operation instead of quitting the app
    bool consumed = false;
    if (m_editor.mode == EditMode::GridPaintBlock ||
        m_editor.mode == EditMode::GridPaintErase) {
        if (m_editor.painting) {
            endGridPaint();
            consumed = true;
        }
    }
    if (m_editor.mode == EditMode::RoadmapAddEdgeUndirected ||
        m_editor.mode == EditMode::RoadmapAddEdgeDirected) {
        if (m_editor.firstEdgeNode >= 0) {
            m_editor.firstEdgeNode = -1;
            consumed = true;
        }
    }
    return consumed;
}

void PlatformBase::handleEditorClick(b2Vec2 p)
{
    if (m_editor.mode == EditMode::GridPaintBlock ||
        m_editor.mode == EditMode::GridPaintErase) {
        beginGridPaintAt(p, /*blocked*/ m_editor.mode == EditMode::GridPaintBlock);
        return;
    }
    if (m_editor.mode == EditMode::ContinuousAddBox) {
        addBoxAt(p);
        if (m_editor.autoReplan) replanPreservingGoals();
        return;
    }
    if (m_editor.mode == EditMode::ContinuousAddCircle) {
        addCircleAt(p);
        if (m_editor.autoReplan) replanPreservingGoals();
        return;
    }
    if (m_editor.mode == EditMode::RoadmapAddNode) {
        addRoadNodeAt(p);
        return;
    }
    if (m_editor.mode == EditMode::RoadmapAddEdgeUndirected) {
        addRoadEdgeAt(p, /*directed=*/false);
        return;
    }
    if (m_editor.mode == EditMode::RoadmapAddEdgeDirected) {
        addRoadEdgeAt(p, /*directed=*/true);
    }
}

void PlatformBase::beginGridPaintAt(b2Vec2 p, bool blocked)
{
    auto* G = grid(); if (!G) return;
    const auto [c,r] = G->worldToCell(p);
    m_editor.painting = true;
    m_editor.paintBlocked = blocked;
    m_editor.paintedCells.clear();
    m_editor.paintOperations.clear();
    stampBrush(r,c,blocked);
}

void PlatformBase::continueGridPaintAt(b2Vec2 p)
{
    auto* G = grid(); if (!G) return;
    const auto [c,r] = G->worldToCell(p);
    stampBrush(r,c, m_editor.paintBlocked);
}

void PlatformBase::endGridPaint()
{
    auto* G = grid();
    if (!G) {
        m_editor.painting = false;
        return;
    }
    if (!m_editor.paintOperations.empty()) {
        G->bulkSetBlocked(m_editor.paintOperations);
        m_gridVisualization.cacheDirty = true;
        if (m_editor.autoReplan) replanPreservingGoals();
    }

    // Re-bind world handle in case editor actions rebuilt internals.
    syncWorldBinding();
    m_editor.paintOperations.clear();
    m_editor.paintedCells.clear();
    m_editor.painting = false;
}

void PlatformBase::stampBrush(int r, int c, bool blocked)
{
    auto* G = grid(); if (!G) return;
    const int rad = std::max(0, m_editor.brushRadius); // 0 => 1x1
    for (int rr = r - rad; rr <= r + rad; ++rr)
    for (int cc = c - rad; cc <= c + rad; ++cc) {
        if (rr < 0 || cc < 0 || rr >= G->gridHeightInCells() || cc >= G->gridWidthInCells()) continue;
        const uint64_t key = (uint64_t(uint32_t(rr)) << 32) | uint64_t(uint32_t(cc));
        if (m_editor.paintedCells.insert(key).second) {
            m_editor.paintOperations.emplace_back(rr, cc, blocked);
        }
    }
}

void PlatformBase::addBoxAt(b2Vec2 p)
{
    if (auto* E = freeenv()) {
        (void)E->addBoxBody(
            p, 0.5f * m_editor.boxWidth, 0.5f * m_editor.boxHeight,
            0, Type::BodyType::Obstacle);
    }
}

void PlatformBase::addCircleAt(b2Vec2 p)
{
    if (auto* E = freeenv()) {
        (void)E->addCircleBody(
            p, m_editor.circleRadius, Type::BodyType::Obstacle);
    }
}

void PlatformBase::addRoadNodeAt(b2Vec2 p)
{
    auto* R = road(); if (!R) return;
    (void)R->addNode(p);
    R->autoBounds(0.0f);
    // refresh walls to new bounds
    m_envOwned->clearBoundingWalls();
    m_envOwned->addBoundaryWalls(m_editor.roadmapWallThickness);
}

void PlatformBase::addRoadEdgeAt(b2Vec2 p, bool directed)
{
    auto* R = road(); if (!R) return;
    const int pick = R->nearestNode(p);
    const float radW = pxToWorld(m_editor.nodePickRadiusPx);
    if (b2Distance(R->nodePos(pick), p) > radW) return; // click too far

    if (m_editor.firstEdgeNode < 0) {
        m_editor.firstEdgeNode = pick;
        return;
    }
    // Guard: avoid self-edge if second click picks the same node
    if (pick == m_editor.firstEdgeNode) {
        m_editor.firstEdgeNode = -1;
        return;
    }
    if (directed) R->addEdgeDirected(m_editor.firstEdgeNode, pick);
    else          R->addEdge(m_editor.firstEdgeNode, pick);
    m_editor.firstEdgeNode = -1;
}
