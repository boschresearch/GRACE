// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// tests/test_planners_roadmap.cpp
#include <gtest/gtest.h>

#include "env/RoadmapEnvironment.h"
#include "Simulation.h"
#include "io/SimRoadmapOps.h"
#include "planner/plugin/PlannerFactory.h"
#include "io/RoadmapXML.h"
#include <pugixml.hpp>                  // task parsing
#include <algorithm>
#include <string>
#include <vector>

#ifndef ROADMAP_MAP_PATH
#  error "ROADMAP_MAP_PATH must be provided by the test target"
#endif
#ifndef ROADMAP_SCEN_PATH
#  error "ROADMAP_SCEN_PATH must be provided by the test target"
#endif

namespace {

static std::vector<std::string> enabledRoadmapPlanners() {
  std::vector<std::string> planners;
#if GRACE_BUILD_PLANNER_CCBS
  planners.emplace_back("ccbs-bin");
  planners.emplace_back("ccbs-src");
#endif
#if GRACE_BUILD_PLANNER_PSIPPCTC
  planners.emplace_back("psippctc-bin");
  planners.emplace_back("psippctc-src");
#endif
  return planners;
}

// -------- helpers ------------------------------------------------------------

static void initRoadmap(
    RoadmapEnvironment& R,
    grace::sim::GraphMLImport& import) {
  grace::sim::GraphMLOptions opts{};
  ASSERT_TRUE(grace::sim::readGraphML(ROADMAP_MAP_PATH, R, &import, opts))
      << "Failed to load roadmap GraphML at " << ROADMAP_MAP_PATH;

  // Add a thin boundary so robots are contained; thickness in meters.
  R.addBoundaryWalls(0.25f);
}

// Minimal task loader: <root><agent start_id="..." goal_id="..."/></root>
static std::vector<std::pair<int,int>>
loadTaskPairs(const std::string& path, int takeK) {
  std::vector<std::pair<int,int>> pairs;
  pairs.reserve(takeK > 0 ? takeK : 16);

  pugi::xml_document doc;
  pugi::xml_parse_result ok = doc.load_file(path.c_str());
  if (!ok) {
    ADD_FAILURE() << "Failed to open task XML at " << path
                  << " : " << ok.description();
    return pairs; // empty -> caller can decide what to do
  }

  pugi::xml_node root = doc.child("root");
  if (!root) {
    ADD_FAILURE() << "Task XML missing <root> element";
    return pairs;
  }

  int count = 0;
  for (pugi::xml_node agent : root.children("agent")) {
    const int s = agent.attribute("start_id").as_int(-1);
    const int g = agent.attribute("goal_id").as_int(-1);
    if (s >= 0 && g >= 0) {
      pairs.emplace_back(s, g);
      if (takeK > 0 && ++count >= takeK) break;
    } else {
      ADD_FAILURE() << "Agent entry missing start_id/goal_id";
    }
  }

  return pairs;
}

// Spawn robots at start nodes, plan to their goal nodes using a named planner.
static bool plan_once_roadmap(const std::string& plannerName,
                              int takeK = 20,
                              float robotRadius = 0.25f)
{
  try {
    RoadmapEnvironment R;
    grace::sim::GraphMLImport import;
    initRoadmap(R, import);

    // Load tasks
    auto sg_pairs = loadTaskPairs(ROADMAP_SCEN_PATH, takeK);
    if (sg_pairs.empty()) {
      ADD_FAILURE() << "No tasks parsed from " << ROADMAP_SCEN_PATH;
      return false;
    }

    // Prepare starts & (start, goal) tuples for the planner call
    std::vector<IRoadmap::NodeId> starts;
    std::vector<std::pair<IRoadmap::NodeId, IRoadmap::NodeId>> sgs;
    starts.reserve(sg_pairs.size());
    sgs.reserve(sg_pairs.size());
    for (const auto& [s, g] : sg_pairs) {
      starts.push_back(s);
      sgs.emplace_back(s, g);
    }

    Simulation sim(R);
    auto ids = grace::sim::ops::spawnAtNodes(
        sim, R, starts, robotRadius, starts.size(), "r");
    if (ids.empty()) {
      ADD_FAILURE() << "No robots spawned for tasks in " << ROADMAP_SCEN_PATH;
      return false;
    }

    auto planner = makePlanner(plannerName);
    sim.setPlanner(std::move(planner));

    std::vector<StartGoal> plans;
    plans.reserve(sgs.size());
    for (const auto& [start, goal] : sgs) {
      plans.push_back({R.nodePos(start), R.nodePos(goal)});
    }
    return sim.plan(ids, plans);
  } catch (const std::exception& e) {
    ADD_FAILURE() << "Roadmap planner '" << plannerName << "' threw: " << e.what();
    return false;
  } catch (...) {
    ADD_FAILURE() << "Roadmap planner '" << plannerName << "' threw unknown exception";
    return false;
  }
}

} // namespace

// ------------------------
// 1) Independent runs
// ------------------------
#if GRACE_BUILD_PLANNER_CCBS
TEST(RoadmapPlannersIndependent, CCBS_Binary) {
  EXPECT_TRUE(plan_once_roadmap("ccbs-bin"));
}

TEST(RoadmapPlannersIndependent, CCBS_Source) {
  EXPECT_TRUE(plan_once_roadmap("ccbs-src"));
}
#endif

#if GRACE_BUILD_PLANNER_PSIPPCTC
TEST(RoadmapPlannersIndependent, PSIPPCTC_Binary) {
  EXPECT_TRUE(plan_once_roadmap("psippctc-bin"));
}

TEST(RoadmapPlannersIndependent, PSIPPCTC_Source) {
  EXPECT_TRUE(plan_once_roadmap("psippctc-src"));
}
#endif

// ------------------------
// 2) Sequential switching in one process
//    (fresh Simulation per call; still same process to exercise dlopen/dlclose)
// ------------------------
TEST(RoadmapPlannersSequential, SwitchAll) {
  for (const auto& planner : enabledRoadmapPlanners()) {
    EXPECT_TRUE(plan_once_roadmap(planner)) << planner;
  }
}

// ------------------------
// 3) Optional: one Simulation instance, swapping planners in-place.
// ------------------------
TEST(RoadmapPlannersSequential, SwapSameSimulationInstance) {
  RoadmapEnvironment R;
  grace::sim::GraphMLImport import;
  initRoadmap(R, import);

  // Load tasks
  auto sg_pairs = loadTaskPairs(ROADMAP_SCEN_PATH, /*takeK=*/2);
  ASSERT_FALSE(sg_pairs.empty());

  std::vector<IRoadmap::NodeId> starts;
  std::vector<std::pair<IRoadmap::NodeId, IRoadmap::NodeId>> sgs;
  for (const auto& [s,g] : sg_pairs) {
    starts.push_back(s);
    sgs.emplace_back(s,g);
  }

  Simulation sim(R);
  // Use the 3-arg form, letting defaults pick takeK=all and baseName="r"
  auto ids = grace::sim::ops::spawnAtNodes(sim, R, starts, 0.25f);
  ASSERT_FALSE(ids.empty());

  std::vector<StartGoal> plans;
  plans.reserve(sgs.size());
  for (const auto& [start, goal] : sgs) {
    plans.push_back({R.nodePos(start), R.nodePos(goal)});
  }

  auto runWith = [&](const std::string& name) {
    auto p = makePlanner(name);
    sim.setPlanner(std::move(p));
    ASSERT_TRUE(sim.plan(ids, plans));
  };

  for (const auto& planner : enabledRoadmapPlanners()) {
    runWith(planner);
  }
}
