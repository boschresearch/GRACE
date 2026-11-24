// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// tests/test_planners.cpp
#include <gtest/gtest.h>

#include <cstdlib>
#include <string_view>

#include "env/GridEnvironment.h"
#include "io/GridMovingAI.h"
#include "Simulation.h"
#include "planner/plugin/PlannerFactory.h"
#include "sim/SpawnRobot.h"

#ifndef GRID_MAP_PATH
#  error "GRID_MAP_PATH must be provided by the test target"
#endif
#ifndef GRID_SCEN_PATH
#  error "GRID_SCEN_PATH must be provided by the test target"
#endif

namespace {

static std::vector<std::string> enabledGridPlanners() {
  std::vector<std::string> planners;
#if GRACE_BUILD_PLANNER_EECBS
  planners.emplace_back("eecbs-bin");
  planners.emplace_back("eecbs-src");
#endif
#if GRACE_BUILD_PLANNER_CBSH2RTC
  planners.emplace_back("cbsh2rtc-bin");
  planners.emplace_back("cbsh2rtc-src");
#endif
#if GRACE_BUILD_PLANNER_LACAM0
  planners.emplace_back("lacam0-bin");
  planners.emplace_back("lacam0-src");
#endif
#if GRACE_BUILD_PLANNER_MAPFLNS2
  planners.emplace_back("mapflns2-bin");
  planners.emplace_back("mapflns2-src");
#endif
#if GRACE_BUILD_PLANNER_MAPFGPTDDG
  planners.emplace_back("mapfgptddg-bin");
#endif
#if GRACE_BUILD_PLANNER_SCRIMP
  planners.emplace_back("scrimp-bin");
#endif
  return planners;
}

static void initGrid(GridEnvironment& grid) {
  const auto map = grace::io::readGridMovingAIMap(GRID_MAP_PATH);
  ASSERT_TRUE(map) << map.error;
  grid.makeGridEnvironment(
      map.value->width, map.value->height, 1.0f, map.value->occupancy);
}

static bool plan_once(
    const std::string& plannerName,
    int takeK = 20,
    std::string_view device = "cpu") {
  try {
    GridEnvironment grid;
    initGrid(grid);
    Simulation sim(grid);

    auto planner = makePlanner(plannerName);
    sim.setPlanner(std::move(planner));
    if (plannerName == "mapfgptddg-bin" || plannerName == "scrimp-bin") {
      PlannerSettings settings;
      settings.kv["device"] = std::string(device);
      sim.configurePlanner(std::move(settings));
    }

    auto scenario =
        grace::io::readGridMovingAIScenario(grid, GRID_SCEN_PATH, takeK);
    if (!scenario) {
      ADD_FAILURE() << scenario.error;
      return false;
    }
    const auto& scenarios = *scenario.value;
    auto ids = grace::sim::spawnGridAgent(sim, scenarios);
    if (ids.empty()) {
      ADD_FAILURE() << "No agents spawned for scenario " << GRID_SCEN_PATH;
      return false;
    }

    return sim.plan(ids, scenarios);
  } catch (const std::exception& e) {
    ADD_FAILURE() << "Planner '" << plannerName << "' threw: " << e.what();
    return false;
  } catch (...) {
    ADD_FAILURE() << "Planner '" << plannerName << "' threw unknown exception";
    return false;
  }
}

static bool gpuTestsEnabled() {
  const char* value = std::getenv("GRACE_RUN_GPU_TESTS");
  return value != nullptr && std::string_view(value) == "1";
}

} // namespace

// ------------------------
// 1) Independent runs
// ------------------------
#if GRACE_BUILD_PLANNER_EECBS
TEST(PlannersIndependent, EECBS_Binary) { EXPECT_TRUE(plan_once("eecbs-bin")); }
TEST(PlannersIndependent, EECBS_Source) { EXPECT_TRUE(plan_once("eecbs-src")); }
#endif
#if GRACE_BUILD_PLANNER_CBSH2RTC
TEST(PlannersIndependent, CBSH2RTC_Binary) { EXPECT_TRUE(plan_once("cbsh2rtc-bin")); }
TEST(PlannersIndependent, CBSH2RTC_Source) { EXPECT_TRUE(plan_once("cbsh2rtc-src")); }
#endif
#if GRACE_BUILD_PLANNER_LACAM0
TEST(PlannersIndependent, LACAM0_Binary) { EXPECT_TRUE(plan_once("lacam0-bin")); }
TEST(PlannersIndependent, LACAM0_Source) { EXPECT_TRUE(plan_once("lacam0-src")); }
#endif
#if GRACE_BUILD_PLANNER_MAPFLNS2
TEST(PlannersIndependent, MAPFLNS2_Binary) { EXPECT_TRUE(plan_once("mapflns2-bin")); }
TEST(PlannersIndependent, MAPFLNS2_Source) { EXPECT_TRUE(plan_once("mapflns2-src")); }
#endif
#if GRACE_BUILD_PLANNER_MAPFGPTDDG
TEST(PlannersIndependent, MAPFGPTDDG_Binary) { EXPECT_TRUE(plan_once("mapfgptddg-bin")); }
#endif
#if GRACE_BUILD_PLANNER_SCRIMP
TEST(PlannersIndependent, SCRIMP_Binary) { EXPECT_TRUE(plan_once("scrimp-bin")); }
#endif

// ------------------------
// 2) Opt-in CUDA functionality
// ------------------------
#if GRACE_BUILD_PLANNER_MAPFGPTDDG
TEST(PlannersGPU, MAPFGPTDDG_CUDA) {
  if (!gpuTestsEnabled()) {
    GTEST_SKIP() << "Set GRACE_RUN_GPU_TESTS=1 to run CUDA planner tests";
  }
  EXPECT_TRUE(plan_once("mapfgptddg-bin", 2, "cuda"));
}
#endif
#if GRACE_BUILD_PLANNER_SCRIMP
TEST(PlannersGPU, SCRIMP_CUDA) {
  if (!gpuTestsEnabled()) {
    GTEST_SKIP() << "Set GRACE_RUN_GPU_TESTS=1 to run CUDA planner tests";
  }
  EXPECT_TRUE(plan_once("scrimp-bin", 2, "cuda"));
}
#endif

// ------------------------
// 3) Sequential switching in one process
//    (fresh Simulation to avoid GUI/state; still same process to exercise dlopen/dlclose)
// ------------------------
TEST(PlannersSequential, SwitchAll) {
  for (const auto& planner : enabledGridPlanners()) {
    EXPECT_TRUE(plan_once(planner)) << planner;
  }
}

// Optional: one Simulation instance, swapping planners in-place.
// This assumes your Simulation allows re-planning on the same agents.
TEST(PlannersSequential, SwapSameSimulationInstance) {
  GridEnvironment grid;
  initGrid(grid);
  Simulation sim(grid);

  auto scenario =
      grace::io::readGridMovingAIScenario(grid, GRID_SCEN_PATH, 20);
  ASSERT_TRUE(scenario) << scenario.error;
  const auto& scenarios = *scenario.value;
  auto ids = grace::sim::spawnGridAgent(sim, scenarios);
  ASSERT_FALSE(ids.empty());

  auto runWith = [&](const std::string& name) {
    auto p = makePlanner(name);
    sim.setPlanner(std::move(p));
    ASSERT_TRUE(sim.plan(ids, scenarios));
  };

  for (const auto& planner : enabledGridPlanners()) {
    runWith(planner);
  }
}
