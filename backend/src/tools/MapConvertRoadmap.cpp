// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "tools/MapConvert.h"

#include "MapConvertInternal.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "io/ContinuousYaml.h"
#include "io/GridMovingAI.h"
#include "io/GridXML.h"
#include "io/RoadmapXML.h"

namespace fs = std::filesystem;

namespace tools::mc {

Result continuousYamlToRoadmap(const fs::path& yamlIn,
                               const fs::path& graphOut,
                               const PRMOptions& prm,
                               float wall_thickness)
{
    const auto problem = grace::sim::loadProblemYaml(yamlIn);
    if (problem.obstacles.empty() && problem.robots.empty()) {
        Result result;
        result.message =
            "YAML parse failed or empty env: " + yamlIn.string();
        return result;
    }
    return continuousProblemToRoadmap(
        problem, graphOut, prm, wall_thickness);
}

Result continuousProblemToRoadmap(
    const grace::sim::ProblemYaml& prob,
    const fs::path& graphOut,
    const PRMOptions& prm,
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

    RoadmapEnvironment Rd;
    if (!internal::buildPRMStarRoadmap(E, prm.target_samples, prm.time_sec, prm.robot_radius, Rd)) {
        R.message = "PRM* build failed"; return R;
    }

    // Paper: insert starts/goals as pinned vertices and connect with a local planner.
    if (prm.pin_starts_goals && !prob.robots.empty()) {
        const auto toLocal = [&](const b2Vec2 point) {
            return b2Vec2{
                point.x - prob.min.x,
                point.y - prob.min.y};
        };
        // k-NN connect each pin to roadmap with clearance prm.robot_radius
        const int K = std::max(1, prm.connect_k);
        const float Rmax = std::max(0.0f, prm.connect_radius);
        auto tryConnect = [&](const b2Vec2& p){
            const int N = Rd.numNodes();
            // collect (i,dist) pairs
            std::vector<std::pair<int,float>> cand; cand.reserve(N);
            for (int i=0;i<N;++i){
                const b2Vec2 q = Rd.nodePos(i);
                const float d = std::hypot(q.x - p.x, q.y - p.y);
                if (Rmax <= 0.0f || d <= Rmax) cand.emplace_back(i,d);
            }
            std::partial_sort(cand.begin(), cand.begin()+std::min(K,(int)cand.size()), cand.end(),
                              [](auto& a, auto& b){return a.second<b.second;});
            const int pin = Rd.addNode(p);  // pinned vertex
            // connect both directions if raycast is free
            for (int c=0; c<std::min(K,(int)cand.size()); ++c) {
                const int j = cand[c].first;
                if (!E.raycast(p, Rd.nodePos(j))) {
                    Rd.addEdge(pin, j);
                    Rd.addEdge(j, pin);
                }
            }
        };
        for (const auto& rb : prob.robots) {
            tryConnect(toLocal(rb.start));
            tryConnect(toLocal(rb.goal));
        }
    }

    // Paper: directed edges; add weights = Euclidean length (for time = length/v).
    const auto writeWeights = true;
    const auto coordScale   = 1.0f;
    if (!grace::sim::writeGraphML(Rd, graphOut, writeWeights, coordScale)) {
        R.message = "GraphML write failed: " + graphOut.string();
        return R;
    }
    R.ok = true; R.output = graphOut; return R;
}

Result roadmapToContinuousYaml(const fs::path& graphIn,
                               const fs::path& yamlOut,
                               const fs::path& optionalObstaclesYaml,
                               float wall_thickness)
{
    Result R;

    RoadmapEnvironment Rd;
    grace::sim::GraphMLImport imp;
    if (!grace::sim::readGraphML(graphIn, Rd, &imp)) {
        R.message = "GraphML load failed: " + graphIn.string(); return R;
    }
    Rd.autoBounds(0.0f);

    std::optional<grace::sim::ProblemYaml> problem;
    if (!optionalObstaclesYaml.empty() && fs::exists(optionalObstaclesYaml)) {
        problem = grace::sim::loadProblemYaml(optionalObstaclesYaml);
        if (!problem->obstacles.empty() || !problem->robots.empty()) {
            ContinuousEnvironment E;
            if (const auto error =
                    internal::buildFreeEnvFromProblem(*problem, E, wall_thickness)) {
                R.message = *error;
                return R;
            }
        }
    }

    // If we have an obstacles YAML, pass-through its obstacles into the emitted YAML.
    std::ofstream out(yamlOut);
    if (!out) { R.message = "Cannot write: " + yamlOut.string(); return R; }

    const auto [W,H] = Rd.boundsInMeter();
    out << "environment:\n";
    out << "  min: [0.0, 0.0]\n";
    out << "  max: ["<<W<<", "<<H<<"]\n";
    out << "  obstacles:\n";

    if (problem) {
        const auto toLocal = [&](const b2Vec2 point) {
            return b2Vec2{
                point.x - problem->min.x,
                point.y - problem->min.y};
        };
        for (const auto& o : problem->obstacles) {
            switch (o.kind) {
                case grace::sim::ObstacleKind::Box: {
                    const b2Vec2 center = toLocal(o.center);
                    out << "    - {type: box, center: ["<<center.x<<","<<center.y
                        <<"], size: ["<<o.size.x<<","<<o.size.y<<"]}\n"; break;
                }
                case grace::sim::ObstacleKind::Circle: {
                    const b2Vec2 center = toLocal(o.center);
                    out << "    - {type: circle, center: ["<<center.x<<","<<center.y
                        <<"], radius: "<<o.radius<<"}\n"; break;
                }
                case grace::sim::ObstacleKind::Polygon: {
                    out << "    - {type: polygon, points: [";
                    for (size_t i=0;i<o.points.size();++i) {
                        const b2Vec2 p = toLocal(o.points[i]);
                        out<<"["<<p.x<<","<<p.y<<"]"<<(i+1<o.points.size()? ", ":"");
                    }
                    out << "]}\n"; break;
                }
                case grace::sim::ObstacleKind::Capsule: {
                    const b2Vec2 p0 = toLocal(o.p0);
                    const b2Vec2 p1 = toLocal(o.p1);
                    out << "    - {type: capsule, p0: ["<<p0.x<<","<<p0.y
                        <<"], p1: ["<<p1.x<<","<<p1.y<<"], radius: "<<o.radius<<"}\n"; break;
                }
            }
        }
    }

    R.ok = true; R.output = yamlOut; return R;
}

Result roadmapToGrid(const fs::path& graphIn,
                     const fs::path& gridOut,
                     GridFormat fmt,
                     const fs::path& optionalObstaclesYaml,
                     const RasterOptions& raster,
                     float wall_thickness)
{
    Result R;

    RoadmapEnvironment Rd;
    grace::sim::GraphMLImport imp;
    if (!grace::sim::readGraphML(graphIn, Rd, &imp)) { R.message = "GraphML load failed: " + graphIn.string(); return R; }
    Rd.autoBounds(0.0f);

    // Grid bounds and resolution
    const auto [Wm,Hm] = Rd.boundsInMeter();
    RasterOptions ro = raster;
    if (ro.cell <= 0.0f) ro.cell = /* default */ 0.5f;
    const float h = ro.cell;
    const int Wc = std::max(1, (int)std::ceil(Wm / h));
    const int Hc = std::max(1, (int)std::ceil(Hm / h));
    GridEnvironment G; G.makeGridEnvironment(Wc, Hc, h, std::vector(Wc*Hc,true));

    // If an obstacles YAML is provided, we rasterize it first (blocked),
    // then carve free cells along roadmap edges to *preserve topology*.
    std::unique_ptr<ContinuousEnvironment> E;
    if (!optionalObstaclesYaml.empty() && fs::exists(optionalObstaclesYaml)) {
        grace::sim::ProblemYaml prob = grace::sim::loadProblemYaml(optionalObstaclesYaml);
        E = std::make_unique<ContinuousEnvironment>();
        if (const auto error =
                internal::buildFreeEnvFromProblem(prob, *E, wall_thickness)) {
            R.message = *error;
            return R;
        }
    }

    auto mark_free_cell = [&](int r,int c){
        if (r>=0 && r<Hc && c>=0 && c<Wc) G.setOccupancy(c,r,false);
    };
    auto world_to_rc = [&](float x, float y, int& r, int& c){
        c = (int)std::floor(x / h); r = (int)std::floor(y / h);
    };

    // Carve nodes (ensure at least one free cell at each vertex).
    const int N = Rd.numNodes();
    for (int i=0;i<N;++i) {
        int r,c; const auto p = Rd.nodePos(i); world_to_rc(p.x, p.y, r, c);
        for (int dr=-1; dr<=1; ++dr) for (int dc=-1; dc<=1; ++dc) mark_free_cell(r+dr,c+dc);
    }

    // Carve edges by walking along each adjacency (u -> v) with dedupe.
    auto keyUndir = [](int u,int v)->uint64_t{
        uint32_t a = static_cast<uint32_t>(std::min(u,v));
        uint32_t b = static_cast<uint32_t>(std::max(u,v));
        return (uint64_t(a) << 32) | uint64_t(b);
    };
    std::unordered_set<uint64_t> seen;

    for (int u=0; u<N; ++u) {
        const auto& nbrs = Rd.adj(u);
        for (int v : nbrs) {
            const uint64_t k = keyUndir(u,v);
            if (seen.find(k) != seen.end()) continue;
            seen.insert(k);

            const auto p = Rd.nodePos(u);
            const auto q = Rd.nodePos(v);

            // If an obstacle YAML was supplied and the edge is invalid against it, skip carving.
            if (E && E->raycast(p,q)) continue;

            // Sample along the edge at ~quarter-cell spacing; mark intersected cells free.
            const float L = std::hypot(q.x - p.x, q.y - p.y);
            const int steps = std::max(1, static_cast<int>(std::ceil(L / (0.25f * h))));
            for (int s=0; s<=steps; ++s) {
                const float t = static_cast<float>(s) / static_cast<float>(steps);
                const float x = p.x + t * (q.x - p.x);
                const float y = p.y + t * (q.y - p.y);
                int r,c; world_to_rc(x,y,r,c);
                mark_free_cell(r,c);
            }
        }
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

} // namespace tools::mc
