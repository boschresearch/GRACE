// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "tools/MapConvert.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <utility>
#include <vector>

#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "io/ContinuousYaml.h"
#include "io/GridMovingAI.h"
#include "io/GridXML.h"
#include "io/RoadmapXML.h"

namespace fs = std::filesystem;

namespace {

// YAML writer: environment + robots list
static bool writeProblemYamlEnvAndRobots(
    const std::pair<float,float>& WH,               // {W,H}
    const std::vector<grace::sim::ObstacleYaml>* gridBoxes,     // optional: boxes for blocked cells
    const std::vector<std::pair<b2Vec2,b2Vec2>>& robots, // start,goal in meters
    const std::string& robotType,
    const fs::path& out)
{
    std::ofstream o(out);
    if (!o) return false;

    const auto [W,H] = WH;
    o << "environment:\n";
    o << "  min: [0.0, 0.0]\n";
    o << "  max: [" << W << ", " << H << "]\n";
    o << "  obstacles:\n";

    if (gridBoxes) {
        for (const auto& ob : *gridBoxes) {
            if (ob.kind != grace::sim::ObstacleKind::Box) continue;
            o << "    - {type: box, center: [" << ob.center.x << "," << ob.center.y
              << "], size: [" << ob.size.x << "," << ob.size.y << "]}\n";
        }
    }

    o << "robots:\n";
    for (const auto& rg : robots) {
        const auto& s = rg.first;
        const auto& g = rg.second;
        o << "  - type: " << robotType << "\n"
          << "    start: [" << s.x << ", " << s.y << "]\n"
          << "    goal:  [" << g.x << ", " << g.y << "]\n";
    }
    return true;
}

// Build list of box obstacles from a grid
static void collectBoxesFromGrid(const GridEnvironment& G, std::vector<grace::sim::ObstacleYaml>& outObs)
{
    outObs.clear();
    const float L = G.cellLength();
    for (int r=0;r<G.gridHeightInCells();++r) {
        for (int c=0;c<G.gridWidthInCells();++c) {
            if (!G.checkIfCellBlocked({c,r})) continue;
            grace::sim::ObstacleYaml ob;
            ob.kind   = grace::sim::ObstacleKind::Box;
            ob.center = { (c + 0.5f)*L, (r + 0.5f)*L };
            ob.size   = { L, L };
            outObs.push_back(std::move(ob));
        }
    }
}

static int nearestNodeByXY(const RoadmapEnvironment& R, const b2Vec2& p)
{
    const int N = R.numNodes();
    int best = (N>0 ? 0 : -1);
    float bestD2 = std::numeric_limits<float>::infinity();
    for (int i=0;i<N;++i) {
        b2Vec2 q = R.nodePos(i);
        const float dx = q.x - p.x, dy = q.y - p.y;
        const float d2 = dx*dx + dy*dy;
        if (d2 < bestD2) { bestD2 = d2; best = i; }
    }
    return best;
}

// Minimal tasks.xml writer compatible with grace::sim::readTasksXml()
static bool writeTasksXml(const std::vector<std::pair<int,int>>& nodePairs, const fs::path& out)
{
    std::ofstream o(out);
    if (!o) return false;
    o << "<root>\n";
    for (auto [s,g] : nodePairs) {
        // IMPORTANT: write raw ints (no 'n' prefix); loader adds 'n'
        o << "  <agent start_id=\"" << s << "\" goal_id=\"" << g << "\"/>\n";
    }
    o << "</root>\n";
    return true;
}

} // namespace

namespace tools::mc {

// ---------- problem.yaml (robots) -> .scen ----------
Result scenarioYamlToGridScen(const fs::path& yamlIn,
                              const fs::path& gridMapPath,
                              const fs::path& scenOut,
                              float cell_length)
{
    return scenarioProblemToGridScen(
        grace::sim::loadProblemYaml(yamlIn),
        gridMapPath,
        scenOut,
        cell_length);
}

Result scenarioProblemToGridScen(
    const grace::sim::ProblemYaml& P,
    const fs::path& gridMapPath,
    const fs::path& scenOut,
    float cell_length)
{
    Result R;
    if (P.robots.empty()) { R.message = "No robots in YAML"; return R; }

    // Compose starts/goals
    std::vector<b2Vec2> S, G;
    S.reserve(P.robots.size()); G.reserve(P.robots.size());
    for (const auto& r : P.robots) {
        S.push_back({r.start.x - P.min.x, r.start.y - P.min.y});
        G.push_back({r.goal.x - P.min.x, r.goal.y - P.min.y});
    }

    const float Wm = P.max.x - P.min.x;
    const float Hm = P.max.y - P.min.y;
    const int Wc = std::max(1, (int)std::ceil(Wm / cell_length));
    const int Hc = std::max(1, (int)std::ceil(Hm / cell_length));

    GridEnvironment Gd;
    Gd.makeGridEnvironment(Wc, Hc, cell_length, std::vector(Wc*Hc,false));

    // Write scenario directly to requested path
    (void)gridMapPath; // map path is no longer embedded by the writer; we emit a neutral name
    if (!grace::io::writeGridMovingAIScenario(Gd, scenOut, S, G)) {
        R.message = "Scenario write failed: " + scenOut.string();
        return R;
    }

    R.ok = true; R.output = scenOut; return R;
}

// ---------- .scen (grid) -> problem.yaml (robots) ----------
Result scenarioGridScenToYaml(const fs::path& gridMapIn,
                              GridFormat fmt,
                              const fs::path& scenIn,
                              const fs::path& yamlOut,
                              float cell_length,
                              const std::string& default_type)
{
    Result R;
    // Load grid to get bounds + scale
    GridEnvironment G;

    const auto map = fmt == GridFormat::MovingAI
        ? grace::io::readGridMovingAIMap(gridMapIn)
        : grace::io::readGridXmlMap(gridMapIn);
    if (!map) { R.message = map.error; return R; }

    G.makeGridEnvironment(
        map.value->width, map.value->height, cell_length,
        map.value->occupancy);

    // Load scenario entries (cell pairs)
    auto scenario = grace::io::readGridMovingAIScenario(G, scenIn);
    if (!scenario) { R.message = scenario.error; return R; }
    const auto& cells = *scenario.value;
    if (cells.empty()) { R.message = "Scenario file contains no agents"; return R; }

    // Compose robots (meters)
    std::vector<std::pair<b2Vec2,b2Vec2>> robots;
    robots.reserve(cells.size());
    for (const auto& e : cells) {
        robots.emplace_back(e.start, e.goal);
    }

    const auto [Wm,Hm] = G.boundsInMeter();

    // Optional: embed obstacles mirroring the grid
    std::vector<grace::sim::ObstacleYaml> boxes;
    collectBoxesFromGrid(G, boxes);

    if (!writeProblemYamlEnvAndRobots({Wm,Hm}, &boxes, robots, default_type, yamlOut)) {
        R.message = "Failed to write YAML"; return R;
    }
    R.ok = true; R.output = yamlOut; return R;
}

// ---------- problem.yaml (robots) -> tasks.xml ----------
Result scenarioYamlToTasksXml(const fs::path& yamlIn,
                              const fs::path& graphXmlIn,
                              const fs::path& tasksXmlOut)
{
    return scenarioProblemToTasksXml(
        grace::sim::loadProblemYaml(yamlIn),
        graphXmlIn,
        tasksXmlOut);
}

Result scenarioProblemToTasksXml(
    const grace::sim::ProblemYaml& P,
    const fs::path& graphXmlIn,
    const fs::path& tasksXmlOut)
{
    Result R;
    if (P.robots.empty()) { R.message = "No robots in YAML"; return R; }

    RoadmapEnvironment Rd; grace::sim::GraphMLImport imp;
    if (!grace::sim::readGraphML(graphXmlIn, Rd, &imp)) { R.message = "GraphML load failed"; return R; }

    std::vector<std::pair<int,int>> nodePairs;
    nodePairs.reserve(P.robots.size());
    for (const auto& rb : P.robots) {
        const b2Vec2 start{
            rb.start.x - P.min.x,
            rb.start.y - P.min.y};
        const b2Vec2 goal{
            rb.goal.x - P.min.x,
            rb.goal.y - P.min.y};
        int s = nearestNodeByXY(Rd, start);
        int g = nearestNodeByXY(Rd, goal);
        if (s < 0 || g < 0) { R.message = "No nodes in roadmap"; return R; }
        nodePairs.emplace_back(s, g);
    }

    if (!writeTasksXml(nodePairs, tasksXmlOut)) { R.message = "Failed to write tasks.xml"; return R; }
    R.ok = true; R.output = tasksXmlOut; return R;
}

// ---------- tasks.xml -> problem.yaml (robots) ----------
Result scenarioTasksXmlToYaml(const fs::path& graphXmlIn,
                              const fs::path& tasksXmlIn,
                              const fs::path& yamlOut,
                              const std::string& default_type)
{
    Result R;

    RoadmapEnvironment Rd; grace::sim::GraphMLImport imp;
    if (!grace::sim::readGraphML(graphXmlIn, Rd, &imp)) { R.message = "GraphML load failed"; return R; }

    auto pairs = grace::sim::readTasksXml(tasksXmlIn, imp);
    if (pairs.empty()) { R.message = "No agents in tasks.xml"; return R; }

    std::vector<std::pair<b2Vec2,b2Vec2>> robots;
    robots.reserve(pairs.size());
    for (auto [sId, gId] : pairs) {
        robots.emplace_back(Rd.nodePos(sId), Rd.nodePos(gId));
    }

    const auto [W,H] = Rd.boundsInMeter();
    if (!writeProblemYamlEnvAndRobots({W,H}, nullptr, robots, default_type, yamlOut)) {
        R.message = "Failed to write YAML"; return R;
    }
    R.ok = true; R.output = yamlOut; return R;
}

// ---------- .scen (grid) -> tasks.xml ----------
Result scenarioGridScenToTasksXml(const fs::path& gridMapIn,
                                  GridFormat fmt,
                                  float cell_length,
                                  const fs::path& scenIn,
                                  const fs::path& graphXmlIn,
                                  const fs::path& tasksXmlOut)
{
    Result R;

    GridEnvironment G;

    const auto map = fmt == GridFormat::MovingAI
        ? grace::io::readGridMovingAIMap(gridMapIn)
        : grace::io::readGridXmlMap(gridMapIn);
    if (!map) { R.message = map.error; return R; }

    G.makeGridEnvironment(
        map.value->width, map.value->height, cell_length,
        map.value->occupancy);

    auto scenario = grace::io::readGridMovingAIScenario(G, scenIn);
    if (!scenario) { R.message = scenario.error; return R; }
    const auto& cells = *scenario.value;
    if (cells.empty()) { R.message = "Scenario file contains no agents"; return R; }

    RoadmapEnvironment Rd; grace::sim::GraphMLImport imp;
    if (!grace::sim::readGraphML(graphXmlIn, Rd, &imp)) { R.message = "GraphML load failed"; return R; }

    std::vector<std::pair<int,int>> nodePairs;
    nodePairs.reserve(cells.size());
    for (const auto& e : cells) {
        int ns = nearestNodeByXY(Rd, e.start);
        int ng = nearestNodeByXY(Rd, e.goal);
        if (ns < 0 || ng < 0) { R.message = "No nodes in roadmap"; return R; }
        nodePairs.emplace_back(ns, ng);
    }

    if (!writeTasksXml(nodePairs, tasksXmlOut)) { R.message = "Failed to write tasks.xml"; return R; }
    R.ok = true; R.output = tasksXmlOut; return R;
}

// ---------- tasks.xml -> .scen (grid) ----------
Result scenarioTasksXmlToGridScen(const fs::path& graphXmlIn,
                                  const fs::path& tasksXmlIn,
                                  const fs::path& gridMapPath,
                                  float cell_length,
                                  const fs::path& scenOut)
{
    Result R;

    RoadmapEnvironment Rd; grace::sim::GraphMLImport imp;
    if (!grace::sim::readGraphML(graphXmlIn, Rd, &imp)) { R.message = "GraphML load failed"; return R; }

    auto pairs = grace::sim::readTasksXml(tasksXmlIn, imp);
    if (pairs.empty()) { R.message = "No agents in tasks.xml"; return R; }

    // We need a "grid" for writing .scen (only for row/col conversion and dims).
    // Infer bounds from roadmap and convert to cells.
    const auto [Wm,Hm] = Rd.boundsInMeter();
    const int Wc = std::max(1, (int)std::ceil(Wm / cell_length));
    const int Hc = std::max(1, (int)std::ceil(Hm / cell_length));

    GridEnvironment Gd;
    Gd.makeGridEnvironment(
        Wc, Hc, cell_length, std::vector<bool>(Wc * Hc, false));

    std::vector<b2Vec2> S, G;
    S.reserve(pairs.size()); G.reserve(pairs.size());
    for (auto [sId,gId] : pairs) { S.push_back(Rd.nodePos(sId)); G.push_back(Rd.nodePos(gId)); }

    (void)gridMapPath; // map path is no longer embedded by the writer; we emit a neutral name
    if (!grace::io::writeGridMovingAIScenario(Gd, scenOut, S, G)) {
        R.message = "Scenario write failed: " + scenOut.string();
        return R;
    }

    R.ok = true; R.output = scenOut; return R;
}

} // namespace tools::mc
