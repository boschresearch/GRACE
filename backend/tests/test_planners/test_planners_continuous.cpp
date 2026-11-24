// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// tests/test_planners_continuous.cpp
#include <gtest/gtest.h>

#include "env/ContinuousEnvironment.h"
#include "Simulation.h"
#include "io/SimContinuousYamlOps.h"
#include "planner/plugin/PlannerFactory.h"
#include "planner/Planner.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <algorithm>

namespace {

static std::vector<std::string> enabledContinuousPlanners() {
  std::vector<std::string> planners;
#if GRACE_BUILD_PLANNER_DBCBS
  planners.emplace_back("dbcbs-bin");
  planners.emplace_back("dbcbs-src");
#endif
#if GRACE_BUILD_PLANNER_SST
  planners.emplace_back("sst-bin");
  planners.emplace_back("sst-src");
#endif
  return planners;
}

// -------- helpers ------------------------------------------------------------

// Parse the YAML to collect robot types (to configure db_cbs properly)
static std::vector<std::string> getRobotTypesFromYaml(const std::string& yamlPath) {
  std::vector<std::string> types;
  try {
    YAML::Node root = YAML::LoadFile(yamlPath);
    if (!root || !root["robots"]) return types;
    for (const auto& rn : root["robots"]) {
      if (rn["type"]) types.push_back(rn["type"].as<std::string>());
    }
  } catch (...) {
    // leave empty; caller will assert or handle
  }
  return types;
}

// Quickly verify plans exist for all robots and time is monotone
static void expectPlansExist(Simulation& sim) {
  auto& reg = sim.robots();
  const auto& ids = reg.robots();
  ASSERT_FALSE(ids.empty());
  for (auto id : ids) {
    const auto* m = reg.meta(id);
    ASSERT_NE(m, nullptr);
    EXPECT_FALSE(m->plan.waypoints.empty());
    for (size_t k = 1; k < m->plan.waypoints.size(); ++k) {
      EXPECT_LE(m->plan.waypoints[k-1].t, m->plan.waypoints[k].t);
    }
  }
}

// Spawn & plan once from a YAML problem file, using a named planner.
// Returns true on success, false on failure (and adds gtest failures).
static bool plan_once_continuous(const std::string& plannerName,
                                 const std::string& yamlPath = CONT_PROBLEM_PATH,
                                 float wallThickness = 0.25f)
{
  try {
    // Try to pass the robot type to db-CBS (helps choose state dimension/footprints)
    const auto types = getRobotTypesFromYaml(yamlPath);
    if (types.empty()) {
      ADD_FAILURE() << "No robots found in problem YAML at " << yamlPath;
      return false;
    }
    const std::string chosenType = types.front();

    ContinuousEnvironment E;              // bounds will be set by the helper
    Simulation sim(E);

    auto planner = makePlanner(plannerName);

    // Provide a minimal setting that both dbcbs-bin/src understand
    // (they accept "robot_type" in applySettings).
    PlannerSettings S;
    S.kv["robot_type"] = chosenType;
    S.kv["timelimit_s"] = "10";
    // Optional: keep temp files for debugging
    // S.kv["keeptemps"] = "1";

    sim.setPlanner(std::move(planner));
    sim.configurePlanner(std::move(S));

    if (!grace::sim::ops::spawnAndPlanFromProblemYaml(
            sim, E, yamlPath, wallThickness)) {
      ADD_FAILURE() << "spawnAndPlanFromProblemYaml failed for planner '" << plannerName
                    << "' on " << yamlPath;
      return false;
    }

    expectPlansExist(sim);
    return true;

  } catch (const std::exception& e) {
    ADD_FAILURE() << "Continuous planner '" << plannerName << "' threw: " << e.what();
    return false;
  } catch (...) {
    ADD_FAILURE() << "Continuous planner '" << plannerName << "' threw unknown exception";
    return false;
  }
}

} // namespace

// ----------------------------------------------------------------------------
// 1) Independent runs (fresh Simulation per test)
// ----------------------------------------------------------------------------
#if GRACE_BUILD_PLANNER_DBCBS
TEST(ContinuousPlannersIndependent, DBCBS_Binary) {
  EXPECT_TRUE(plan_once_continuous("dbcbs-bin"));
}

TEST(ContinuousPlannersIndependent, DBCBS_Source) {
  testing::internal::CaptureStderr();
  const bool planned = plan_once_continuous("dbcbs-src");
  const std::string logs = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(planned);
  EXPECT_NE(logs.find("DBCBS (src): using opt.yaml"), std::string::npos)
      << logs;
  EXPECT_EQ(logs.find("DBCBS (src): optimizer unavailable/failed"),
            std::string::npos)
      << logs;
}
#endif

#if GRACE_BUILD_PLANNER_SST
TEST(ContinuousPlannersIndependent, SST_Binary) {
  EXPECT_TRUE(plan_once_continuous("sst-bin"));
}

TEST(ContinuousPlannersIndependent, SST_Source) {
  EXPECT_TRUE(plan_once_continuous("sst-src"));
}
#endif

// ----------------------------------------------------------------------------
// 2) Sequential switching in one process (fresh Simulation for each call)
//    Exercises dlopen/dlclose & state isolation between planners.
// ----------------------------------------------------------------------------
TEST(ContinuousPlannersSequential, SwitchBoth) {
  for (const auto& planner : enabledContinuousPlanners()) {
    EXPECT_TRUE(plan_once_continuous(planner)) << planner;
  }
}
