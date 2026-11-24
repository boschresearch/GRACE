// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "PlatformBase.h"
#include "../render/DebugDrawQueue.h"
#include "../ui/ViewController.h"
#include "../render/ColorUtil.h"
#include "Simulation.h"
#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "env/ContinuousEnvironment.h"
#include "../sample/SampleContext.h"

#include <fmt/format.h>
#include <algorithm>
#include <cmath>
#include <imgui.h>
#include <filesystem>

// --- small helpers for dashed segments ---
static inline b2Vec2 add(b2Vec2 a, b2Vec2 b){ return {a.x+b.x, a.y+b.y}; }
static inline b2Vec2 mul(b2Vec2 a, float s){ return {a.x*s, a.y*s}; }

void PlatformBase::drawPlannedPaths()
{
    if (!m_overlays.drawPlanned || !m_sim) return;
    ensurePalette();
    auto& dd = m_context->debugDraw->getB2DebugDraw();
    const float r = pxToWorld(std::max(1.f, m_overlays.pathThicknessPx) * 0.5f);
    const auto& ids = m_sim->robots().robots();
    for (b2BodyId id : ids) {
        const auto* meta = m_sim->robots().meta(id);
        if (!meta || meta->plan.waypoints.size() < 2) continue;
        const b2HexColor col = m_robotDisplay.colors[idKey(id)];
        for (size_t i = 1; i < meta->plan.waypoints.size(); ++i) {
            dd.DrawSolidCapsuleFcn(meta->plan.waypoints[i-1].p, meta->plan.waypoints[i].p, r, col, dd.context);
        }
    }
}

void PlatformBase::drawExecutedTraces()
{
    if (!m_overlays.drawExecuted) return;
    ensurePalette();
    auto& ddq = *m_context->debugDraw;
    auto& dd = ddq.getB2DebugDraw();
    const float r = pxToWorld(std::max(1.f, m_overlays.pathThicknessPx) * 0.5f);
    for (const auto& kv : m_executionTrace.points) {
        const auto& poly = kv.second; if (poly.size() < 2) continue;
        const b2HexColor col = m_robotDisplay.colors.count(kv.first)
            ? m_robotDisplay.colors.at(kv.first)
            : b2_colorWhite;
        for (size_t i = 1; i < poly.size(); ++i) dd.DrawSolidCapsuleFcn(poly[i-1], poly[i], r, col, dd.context);
    }
}

void PlatformBase::drawStartGoalDashes()
{
    if (!m_overlays.showStartGoal || !m_sim) return;
    ensurePalette();
    auto& dd = m_context->debugDraw->getB2DebugDraw();

    const float dashW = pxToWorld(std::max(1.f, m_overlays.pathThicknessPx) * 0.5f);
    const float dash  = pxToWorld(std::max(1.f, m_overlays.startGoalDashPx));
    const float gap   = pxToWorld(std::max(0.5f, m_overlays.startGoalGapPx));
    const float step  = std::max(1e-6f, dash + gap);
    const float rGoal = pxToWorld(std::max(1.f, m_overlays.goalMarkerPx));

    for (b2BodyId id : m_sim->robots().robots()) {
        const b2Vec2 p0 = b2Body_GetPosition(id);
        const RobotMeta* meta = m_sim->robots().meta(id);
        if (!meta) continue;
        // Prefer the explicitly requested goal (works even before planning).
        b2Vec2 goal = p0;
        bool hasGoal = false;
        if (meta->requested_goal_m) { goal = *meta->requested_goal_m; hasGoal = true; }
        else if (!meta->plan.waypoints.empty()) { goal = meta->plan.waypoints.back().p; hasGoal = true; }
        if (!hasGoal) continue;

        b2Vec2 d { goal.x - p0.x, goal.y - p0.y };
        const float L = std::sqrt(d.x*d.x + d.y*d.y);
        if (L <= 1e-6f) continue;
        d.x /= L; d.y /= L; // unit dir

        const b2HexColor col = m_robotDisplay.colors.count(idKey(id))
            ? m_robotDisplay.colors.at(idKey(id))
            : b2_colorWhite;

        // Dashed from p0 to goal
        for (float t = 0.0f; t < L; t += step) {
            const float a0 = t;
            const float a1 = std::min(L, t + dash);
            const b2Vec2 q0 = add(p0, mul(d, a0));
            const b2Vec2 q1 = add(p0, mul(d, a1));
            dd.DrawSolidCapsuleFcn(q0, q1, dashW, col, dd.context);
        }
        // Goal endpoint marker
        dd.DrawPointFcn(goal, rGoal, col, dd.context);
    }
}

// Now returns a string to be embedded in the unified HUD (no window here).
std::string PlatformBase::composeHoverLine()
{
    if (!m_overlays.showHover) return {};
    const ImVec2 mp = ImGui::GetMousePos();
    const auto   win = m_context->view->getWindowSizePx();
    const int mx = int(std::lround(mp.x)), my = int(std::lround(mp.y));
    if (mx < 0 || my < 0 || mx >= (int)win.x || my >= (int)win.y) return {};

    const b2Vec2 pw = m_context->view->mapPixelToWorld({mx, my});
    std::string line = fmt::format("hover: (x={:.3f}, y={:.3f})", pw.x, pw.y);
    if (isGrid()) {
        if (auto* G = grid()) {
            auto [c, r] = G->worldToCell(pw);
            const b2Vec2 cc = G->cellToWorld({c, r});
            line += fmt::format("  |  cell (r={}, c={})  center=({:.3f},{:.3f})",
                                r, c, cc.x, cc.y);

            // lightweight “traffic weight” from reservation cache near tVis
            if (m_gridVisualization.cacheDirty) rebuildGridReservations();
            const double t0 =
                m_playback.visualizationTime - double(m_gridVisualization.reservationWindow);
            const double t1 =
                m_playback.visualizationTime + double(m_gridVisualization.reservationWindow);
            int hits = 0;
            for (const auto& interval : m_gridVisualization.reservations) {
                if (interval.row == r && interval.column == c &&
                    std::max(t0, interval.startTime) <
                        std::min(t1, interval.endTime)) {
                    ++hits;
                }
            }
            line += fmt::format("  resv≈{}", hits);
        }
    }
    return line;
}

std::string PlatformBase::composeDetailsLine() const
{
    if (!m_envOwned) return "No environment loaded.";

    const std::size_t nObs = m_envOwned->numObstacles();
    const std::size_t nWal = m_envOwned->numBoundingWalls();
    const std::size_t nRob = m_envOwned->numRobots();

    if (const auto* G = grid()) {
        std::string mapName;
        if (!m_studio.mapPath.empty()) {
            try { mapName = std::filesystem::path(m_studio.mapPath).filename().string(); }
            catch (...) { mapName = m_studio.mapPath; }
        }
        if (!mapName.empty()) {
            return fmt::format(
                "Path: {}  |  Grid size: {} × {} cells (cell={:.3f} m)  |  Obstacles={}  Walls={}  Robots={}",
                mapName, G->gridWidthInCells(), G->gridHeightInCells(), G->cellLength(), nObs, nWal, nRob);
        } else {
            return fmt::format(
                "Grid size: {} × {} cells (cell={:.3f} m)  |  Obstacles={}  Walls={}  Robots={}",
                G->gridWidthInCells(), G->gridHeightInCells(), G->cellLength(), nObs, nWal, nRob);
        }
    }

    // Non-grid envs: keep it short and consistent
    return fmt::format("Obstacles={}  Walls={}  Robots={}", nObs, nWal, nRob);
}

void PlatformBase::drawSelectionBadges()
{
    if (m_robotDisplay.selected.empty()) return;
    auto& dd = m_context->debugDraw->getB2DebugDraw();
    const float ring = pxToWorld(8.0f);
    for (auto id : m_robotDisplay.selected) {
        dd.DrawCircleFcn(
            b2Body_GetPosition(id), ring, rgba(255,215,0,255), dd.context);
    }
}

static inline b2Vec2 lerp(const b2Vec2& a, const b2Vec2& b, float t) {
    return { a.x + t*(b.x-a.x), a.y + t*(b.y-a.y) };
}

template <class WP>
static b2Vec2 samplePlanAt(const std::vector<WP>& wps, double t)
{
    if (wps.empty()) return {0,0};
    if (t <= wps.front().t) return wps.front().p;
    if (t >= wps.back().t)  return wps.back().p;
    auto it = std::upper_bound(wps.begin(), wps.end(), t, [](double tt, const WP& s){ return tt < s.t; });
    size_t i = size_t(std::distance(wps.begin(), it));
    const auto& a = wps[i-1]; const auto& b = wps[i];
    const float u = float((t - a.t) / std::max<double>(1e-9, b.t - a.t));
    return lerp(a.p, b.p, u);
}

void PlatformBase::drawGhostFleet()
{
    if (!m_sim) return;
    auto& dd = m_context->debugDraw->getB2DebugDraw();
    const float rRing = pxToWorld(5.0f);
    for (b2BodyId id : m_sim->robots().robots()) {
        const auto* m = m_sim->robots().meta(id);
        if (!m || m->plan.waypoints.empty()) continue;
        dd.DrawCircleFcn(
            samplePlanAt(m->plan.waypoints, m_playback.visualizationTime),
            rRing, rgba(255,255,255,220), dd.context);
    }
}

void PlatformBase::drawHoverCrosshair()
{
    if (!m_overlays.showHover) return;
    const ImVec2 mp = ImGui::GetMousePos();
    const auto win  = m_context->view->getWindowSizePx();
    const int mx = int(std::lround(mp.x)), my = int(std::lround(mp.y));
    if (mx < 0 || my < 0 || mx >= (int)win.x || my >= (int)win.y) return;

    const b2Vec2 pw = m_context->view->mapPixelToWorld({mx,my});
    auto& ddq = *m_context->debugDraw;
    auto& dd = ddq.getB2DebugDraw();
    const b2HexColor col = rgba(255,215,0,255);
    const float s = pxToWorld(8.0f);
    ddq.drawLine({pw.x-s,pw.y}, {pw.x+s,pw.y}, col);
    ddq.drawLine({pw.x,pw.y-s}, {pw.x,pw.y+s}, col);
    dd.DrawPointFcn(pw, 3.0f, col, dd.context);
}

void PlatformBase::drawRoadmap()
{
    if (!m_overlays.drawRoadmap) return;
    auto* R = road();
    if (!R) return;
    auto& ddq = *m_context->debugDraw;
    auto& dd = ddq.getB2DebugDraw();
    const b2HexColor colUndir=rgba(90,90,90,220), colDir=rgba(0,160,170,230), colNode=rgba(30,144,255,220);
    const float rNode=pxToWorld(2.0f), ah=pxToWorld(5.0f), aw=pxToWorld(3.0f);
    const int N = R->numNodes();
    for (int u=0; u<N; ++u) {
        const b2Vec2 pu = R->nodePos(u);
        const auto& Au  = R->adj(u);
        for (auto v : Au) {
            if (v == u) continue;
            const b2Vec2 pv = R->nodePos(v);
            const auto& Av  = R->adj(v);
            const bool rev  = (std::find(Av.begin(), Av.end(), u) != Av.end());
            if (rev) {
                if (u < v) ddq.drawLine(pu, pv, colUndir);
            } else {
                ddq.drawLine(pu, pv, colDir);
                b2Vec2 d = { pv.x - pu.x, pv.y - pu.y };
                const float L = std::sqrt(d.x*d.x + d.y*d.y);
                if (L > 1e-6f) {
                    d.x /= L; d.y /= L;
                    const b2Vec2 n = { -d.y, d.x };
                    const b2Vec2 tip  = pv;
                    const b2Vec2 base = { pv.x - d.x*ah, pv.y - d.y*ah };
                    ddq.drawLine(tip, { base.x + n.x*aw, base.y + n.y*aw }, colDir);
                    ddq.drawLine(tip, { base.x - n.x*aw, base.y - n.y*aw }, colDir);
                }
            }
        }
    }
    for (int u=0; u<N; ++u) dd.DrawCircleFcn(R->nodePos(u), rNode, colNode, dd.context);
}

void PlatformBase::rebuildGridReservations()
{
    auto& reservations = m_gridVisualization.reservations;
    reservations.clear();
    auto* G = grid();
    if (!G || !m_sim) return;
    for (b2BodyId id : m_sim->robots().robots()) {
        const auto* m = m_sim->robots().meta(id);
        if (!m || m->plan.waypoints.size() < 2) continue;
        const uint64_t key = idKey(id);
        for (size_t i = 1; i < m->plan.waypoints.size(); ++i) {
            const auto& a = m->plan.waypoints[i-1];
            const auto& b = m->plan.waypoints[i];
            const float len = b2Distance(a.p, b.p);
            const float step = std::max(1.0f, G->cellLength());  // ~1 sample per cell
            const int   n    = std::max(1, int(std::ceil(len / step)));
            for (int k = 0; k < n; ++k) {
                const float t0 = float(k)   / float(n);
                const float t1 = float(k+1) / float(n);
                const b2Vec2 q0{ a.p.x + (b.p.x - a.p.x) * t0, a.p.y + (b.p.y - a.p.y) * t0 };
                const b2Vec2 q1{ a.p.x + (b.p.x - a.p.x) * t1, a.p.y + (b.p.y - a.p.y) * t1 };

                const auto [c0, r0] = G->worldToCell(q0);
                const double u0 = (1.0 - t0) * a.t + t0 * b.t;
                const double u1 = (1.0 - t1) * a.t + t1 * b.t;
                reservations.push_back({ r0, c0, u0, u1, key });
            }
        }
    }
    m_gridVisualization.cacheDirty = false;
}

void PlatformBase::drawGridCellRect(int r, int c, b2HexColor col)
{
    auto* G = grid(); if (!G) return;
    const float a = G->cellLength();
    b2Transform xf{}; xf.p=G->cellToWorld({c,r}); xf.q.c=1.0f; xf.q.s=0.0f;
    const b2Vec2 v[4]={{-0.5f*a,-0.5f*a},{+0.5f*a,-0.5f*a},{+0.5f*a,+0.5f*a},{-0.5f*a,+0.5f*a}};
    auto& dd = m_context->debugDraw->getB2DebugDraw();
    dd.DrawSolidPolygonFcn(xf, v, 4, 0.0f, col, dd.context);
}

void PlatformBase::drawGridReservations()
{
    if (!isGrid() || !m_sim) return;
    if (m_gridVisualization.cacheDirty) rebuildGridReservations();

    const double t0 =
        m_playback.visualizationTime - double(m_gridVisualization.reservationWindow);
    const double t1 =
        m_playback.visualizationTime + double(m_gridVisualization.reservationWindow);
    if (t1 <= t0) return;

    for (const auto& interval : m_gridVisualization.reservations) {
        const double u0 = std::max(t0, interval.startTime);
        const double u1 = std::min(t1, interval.endTime);
        if (u1 <= u0) continue;
        const float w = float((u1 - u0) / std::max(1e-6, t1 - t0));
        const b2HexColor base = m_robotDisplay.colors.count(interval.robotKey)
            ? m_robotDisplay.colors.at(interval.robotKey)
            : rgba(200,200,200,220);
        drawGridCellRect(
            interval.row, interval.column,
            mix(base, rgba(255,255,255,30), 1.f - w));
    }
}

void PlatformBase::drawContinuousClearance()
{
    auto* E = freeenv();
    if (!E) return;

    auto& dd = m_context->debugDraw->getB2DebugDraw();
    const auto B  = E->boundsInMeter();
    const auto VB = m_context->view->getViewBounds();
    const float x0 = std::max(0.0f, VB.lowerBound.x);
    const float y0 = std::max(0.0f, VB.lowerBound.y);
    const float x1 = std::min(B.width,  VB.upperBound.x);
    const float y1 = std::min(B.height, VB.upperBound.y);
    if (x1 <= x0 || y1 <= y0) return;

    // LOD: coarser when zoomed out
    const float base =
        1.0f / std::max(1, m_continuousVisualization.clearanceSamplesPerMeter);
    const float zoom = m_context->view->getZoom();
    const float lod  = std::clamp(zoom / 10.0f, 0.5f, 4.0f); // tune: bigger = coarser
    const float cell = base * lod;

    const float dx = cell, dy = cell;
    const float maxD = std::max(1e-3f, m_continuousVisualization.clearanceMax);
    const b2HexColor c0 = rgba(255, 60, 60, 140);  // red
    const b2HexColor c1 = rgba( 30,255, 30, 140);  // green

    // Probe radius tied to step
    const float rProbe = 0.5f * cell;

    auto marchDir = [&](b2Vec2 p, b2Vec2 d)->float
    {
        const float len = std::sqrt(d.x*d.x + d.y*d.y);
        if (len <= 1e-6f) return maxD;
        d.x /= len; d.y /= len;

        const float ds = cell;
        float s = 0.0f;

        for (; s <= maxD; s += ds) {
            const b2Vec2 q{ p.x + d.x * s, p.y + d.y * s };
            b2AABB a{};
            a.lowerBound = { q.x - rProbe, q.y - rProbe };
            a.upperBound = { q.x + rProbe, q.y + rProbe };
            if (!E->isFreeAABB(a)) break;
        }
        return std::min(s, maxD);
    };

    for (float y = y0; y < y1; y += dy) {
        for (float x = x0; x < x1; x += dx) {
            const b2Vec2 p{x, y};

            float dmin = maxD;
            dmin = std::min(dmin, marchDir(p, {+1, 0}));
            dmin = std::min(dmin, marchDir(p, {-1, 0}));
            dmin = std::min(dmin, marchDir(p, { 0,+1}));
            dmin = std::min(dmin, marchDir(p, { 0,-1}));

            const float t = std::clamp(dmin / maxD, 0.0f, 1.0f);
            const b2HexColor col = mix(c0, c1, t);

            b2Transform xf{}; xf.p = p; xf.q.c = 1.0f; xf.q.s = 0.0f;
            const b2Vec2 v[4] = {
                {-0.5f*dx, -0.5f*dy}, {+0.5f*dx, -0.5f*dy},
                {+0.5f*dx, +0.5f*dy}, {-0.5f*dx, +0.5f*dy}
            };
            dd.DrawSolidPolygonFcn(xf, v, 4, 0.0f, col, dd.context);
        }
    }
}
