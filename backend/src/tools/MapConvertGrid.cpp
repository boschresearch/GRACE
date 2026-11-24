// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "tools/MapConvert.h"

#include "MapConvertInternal.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

#include "env/GridEnvironment.h"
#include "io/GridMovingAI.h"
#include "io/GridXML.h"
#include "io/RoadmapXML.h"

namespace fs = std::filesystem;

namespace {

static bool writeEnvironmentYamlFromGrid(const GridEnvironment& G, const fs::path& outPath)
{
    std::ofstream out(outPath);
    if (!out) return false;
    const auto [Wm, Hm] = G.boundsInMeter();
    const float cell = G.cellLength();

    out << "environment:\n";
    out << "  min: [0.0, 0.0]\n";
    out << "  max: [" << Wm << ", " << Hm << "]\n";
    out << "  obstacles:\n";

    for (int r=0;r<G.gridHeightInCells();++r) for (int c=0;c<G.gridWidthInCells();++c) {
        if (!G.checkIfCellBlocked({c,r})) continue;
        const float cx = (c + 0.5f) * cell;
        const float cy = (r + 0.5f) * cell;
        out << "    - {type: box, center: ["<<cx<<","<<cy<<"], size: ["<<cell<<","<<cell<<"]}\n";
    }
    return true;
}

static float effRadiusForRobot(const grace::sim::RobotYamlSpec& r)
{
    if (r.footprint && r.footprint->kind == grace::sim::RobotFootprintKind::Circle)
        return r.footprint->radius;
    if (r.footprint && r.footprint->kind == grace::sim::RobotFootprintKind::Box) {
        const float w = r.footprint->size.x, h = r.footprint->size.y;
        return 0.5f * std::sqrt(w*w + h*h); // circumscribed
    }
    // Fallback (treat as circle if unknown/missing)
    return (r.footprint ? std::max(0.0f, r.footprint->radius) : 0.0f);
}

static float computeCellFromRobotsOrFallback(const grace::sim::ProblemYaml& spec, float fallbackCell /*meters*/)
{
    float rmax = 0.0f;
    for (const auto& rb : spec.robots)
        rmax = std::max(rmax, effRadiusForRobot(rb));
    if (rmax > 0.0f) return 2.0f * rmax; // paper: h = 2 * max r_eff
    return fallbackCell;
}

struct CoverageTester {
    // Supersampled area coverage in a cell against mixed shapes.
    const ContinuousEnvironment* E{nullptr};
    float occ_thresh{};   // 0 => any-overlap; else fraction in (0,1]
    int ss{};             // supersamples per axis
    float inset{};        // optional
    float cell{};         // cell length
    std::pair<float,float> WH;
    bool  use_shape{true};      // true => E->isFreeShape, false => E->isFreeAABB
    bool  clamp_edges{true};    // clamp last cell to [0,W]x[0,H]

    bool coveredCell(float cx0, float cy0) const {
        float x0=cx0, y0=cy0, x1=cx0+cell, y1=cy0+cell;
        if (clamp_edges) {
            x1 = std::min(x1, WH.first);
            y1 = std::min(y1, WH.second);
        }

        if (occ_thresh <= 0.0f) {
            // --- any-overlap (single test) ---
            b2AABB a{}; a.lowerBound={x0, y0}; a.upperBound={x1, y1};
            if (use_shape) {
                return !E->isFreeShape(a, inset);
            } else {
                // legacy AABB: shrink by inset, as before
                a.lowerBound.x += inset; a.lowerBound.y += inset;
                a.upperBound.x -= inset; a.upperBound.y -= inset;
                return !E->isFreeAABB(a);
            }
        }

        // --- fractional coverage ---
        const int   N   = std::max(1, ss);
        const float dx  = (x1 - x0) / N;
        const float dy  = (y1 - y0) / N;

        int hits = 0, tot = N * N;
        for (int sy=0; sy<N; ++sy) {
            const float ry0 = y0 + sy*dy;
            const float ry1 = ry0 + dy;
            for (int sx=0; sx<N; ++sx) {
                const float rx0 = x0 + sx*dx;
                const float rx1 = rx0 + dx;
                if (use_shape) {
                    b2AABB sub{}; sub.lowerBound={rx0, ry0}; sub.upperBound={rx1, ry1};
                    if (!E->isFreeShape(sub, inset)) ++hits;
                } else {
                    // legacy point sampling (tiny AABB)
                    const float px = rx0 + 0.5f*dx, py = ry0 + 0.5f*dy;
                    b2AABB q{}; q.lowerBound={px, py}; q.upperBound={px, py};
                    if (!E->isFreeAABB(q)) ++hits;
                }
            }
        }
        return (float)hits / (float)tot >= occ_thresh;
    }
};

} // namespace

namespace tools::mc {

Result continuousYamlToGrid(const fs::path& yamlIn,
                            const fs::path& gridOut,
                            GridFormat fmt,
                            const RasterOptions& raster,
                            float wall_thickness)
{
    const auto problem = grace::sim::loadProblemYaml(yamlIn);
    if (problem.obstacles.empty() && problem.robots.empty()) {
        Result result;
        result.message =
            "YAML parse failed or empty env: " + yamlIn.string();
        return result;
    }
    return continuousProblemToGrid(
        problem, gridOut, fmt, raster, wall_thickness);
}

Result continuousProblemToGrid(
    const grace::sim::ProblemYaml& prob,
    const fs::path& gridOut,
    GridFormat fmt,
    const RasterOptions& raster,
    float wall_thickness)
{
    Result R;
    if (prob.obstacles.empty() && prob.robots.empty()) {
        R.message = "Problem specification is empty"; return R;
    }

    ContinuousEnvironment E;
    if (const auto error =
            internal::buildFreeEnvFromProblem(prob, E, wall_thickness)) {
        R.message = *error;
        return R;
    }

    // Paper: h = 2*max r_eff(S_i) unless caller forces a cell length.
    RasterOptions ro = raster;
    if (ro.cell <= 0.0f) ro.cell = computeCellFromRobotsOrFallback(prob, /*fallback*/0.5f);
    const float h = ro.cell;

    // Coverage tests (any-overlap or fractional).
    const auto [Wm,Hm] = E.boundsInMeter();
    const int Wc = std::max(1, (int)std::ceil(Wm / h));
    const int Hc = std::max(1, (int)std::ceil(Hm / h));
    GridEnvironment G;
    G.makeGridEnvironment(Wc, Hc, h, std::vector(Wc*Hc,false));

    CoverageTester cov{
        &E,
        ro.occ_threshold,
        std::max(1, ro.supersample_px),
        ro.inset,
        h,
        {Wm, Hm},
        ro.use_shape_overlap,
        ro.clamp_edge_cells
    };

    for (int r=0;r<Hc;++r) for (int c=0;c<Wc;++c) {
        const float x0 = c*h, y0 = r*h;
        G.setOccupancy(c,r, cov.coveredCell(x0,y0));

    }

    const bool written = fmt == GridFormat::MovingAI
        ? grace::io::writeGridMovingAIMap(G, gridOut)
        : grace::io::writeGridXmlMap(G, gridOut);
    if (!written) {
        R.message = "Grid write failed: " + gridOut.string();
        return R;
    }

    R.ok = true; R.output = gridOut; return R;
}

Result gridToContinuousYaml(const fs::path& gridIn,
                            GridFormat fmt,
                            const fs::path& yamlOut,
                            float cell_length)
{
    Result R;
    GridEnvironment G;

    const auto map = fmt == GridFormat::MovingAI
        ? grace::io::readGridMovingAIMap(gridIn)
        : grace::io::readGridXmlMap(gridIn);
    if (!map) { R.message = map.error; return R; }

    G.makeGridEnvironment(
        map.value->width, map.value->height, cell_length,
        map.value->occupancy);

    if (!writeEnvironmentYamlFromGrid(G, yamlOut)) {
        R.message = "YAML write failed: " + yamlOut.string(); return R;
    }
    R.ok = true; R.output = yamlOut; return R;
}

Result gridToRoadmap(const fs::path& gridIn,
                     GridFormat fmt,
                     const fs::path& graphOut,
                     const PRMOptions& prm,
                     float cell_length)
{
    Result R;

    // Build a continuous env mirroring grid occupancy
    GridEnvironment G;
    const auto map = fmt == GridFormat::MovingAI
        ? grace::io::readGridMovingAIMap(gridIn)
        : grace::io::readGridXmlMap(gridIn);
    if (!map) { R.message = map.error; return R; }

    G.makeGridEnvironment(
        map.value->width, map.value->height, cell_length,
        map.value->occupancy);

    ContinuousEnvironment E;
    const auto [Wm,Hm] = G.boundsInMeter();
    E.setBounds(Wm,Hm);
    const float half = 0.5f * float(G.cellLength());
    for (int r=0;r<G.gridHeightInCells();++r)
        for (int c=0;c<G.gridWidthInCells();++c)
            if (G.checkIfCellBlocked({c,r}))
                E.addBoxBody({(c+0.5f)*G.cellLength(), (r+0.5f)*G.cellLength()}, half, half, 0, Type::BodyType::Obstacle);
    E.addBoundaryWalls(0.5f * float(G.cellLength()));

    RoadmapEnvironment Rd;
    if (!internal::buildPRMStarRoadmap(E, prm.target_samples, prm.time_sec, prm.robot_radius, Rd)) {
        R.message = "PRM* build failed"; return R;
    }
    const auto writeWeights = false;
    const auto coordScale   = 1.0f;
    if (!grace::sim::writeGraphML(Rd, graphOut, writeWeights, coordScale)) {
        R.message = "GraphML write failed: " + graphOut.string(); return R;
    }
    R.ok = true; R.output = graphOut; return R;
}

} // namespace tools::mc
