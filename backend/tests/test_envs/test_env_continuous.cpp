// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// tests/test_env_continuous.cpp
#include <gtest/gtest.h>

#include "env/ContinuousEnvironment.h"
#include "util/Types.h"
#include <yaml-cpp/yaml.h>
#include <box2d/box2d.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

inline float dist(b2Vec2 a, b2Vec2 b) {
  const float dx = a.x - b.x, dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
}

typedef bool b2OverlapResultFcn(b2ShapeId, void*);
static bool StopOnFirst(b2ShapeId, void* ctx) { *static_cast<bool*>(ctx) = true; return false; }

inline b2QueryFilter makeFilter(uint64_t catBits, uint64_t maskBits) {
  b2QueryFilter q{}; q.categoryBits = catBits; q.maskBits = maskBits; return q;
}

// Convenience: add all YAML boxes into E as obstacles
static void addYamlBoxesAsObstacles(ContinuousEnvironment& E, const YAML::Node& envNode) {
  if (!envNode["obstacles"]) return;
  for (const auto& obs : envNode["obstacles"]) {
    const std::string typ = obs["type"].as<std::string>("");
    if (typ != "box") continue; // ignore unknowns
    const auto& c = obs["center"];
    const auto& s = obs["size"];
    const b2Vec2 center{ c[0].as<float>(), c[1].as<float>() };
    const float halfW = 0.5f * s[0].as<float>();
    const float halfH = 0.5f * s[1].as<float>();
    (void)E.addBoxBody(center, halfW, halfH, 0.0f, Type::BodyType::Obstacle);
  }
}

} // namespace

// ----------------------------------------------------------------------------
// 1) End-to-end: load YAML, build env, add walls+obstacles, sanity-check queries
// ----------------------------------------------------------------------------
TEST(FreeEnv, YamlBoundsWallsObstaclesAndQueries)
{
  // Parse YAML
  YAML::Node root = YAML::LoadFile(CONT_PROBLEM_PATH);
  ASSERT_TRUE(root) << "Failed to load problem YAML at " << CONT_PROBLEM_PATH;

  const auto& Eenv = root["environment"];
  ASSERT_TRUE(Eenv) << "YAML missing 'environment'";

  const auto minN = Eenv["min"];
  const auto maxN = Eenv["max"];
  ASSERT_TRUE(minN && maxN) << "YAML environment missing min/max";

  const float xmin = minN[0].as<float>();
  const float ymin = minN[1].as<float>();
  const float xmax = maxN[0].as<float>();
  const float ymax = maxN[1].as<float>();
  ASSERT_FLOAT_EQ(xmin, 0.0f);
  ASSERT_FLOAT_EQ(ymin, 0.0f);

  // Build env
  ContinuousEnvironment E;
  E.setBounds(xmax - xmin, ymax - ymin);
  E.addBoundaryWalls(/*thickness=*/0.25f);

  // Add obstacles from YAML
  addYamlBoxesAsObstacles(E, Eenv);

  // Bounds sanity
  const auto [W,H] = E.boundsInMeter();
  EXPECT_FLOAT_EQ(W, xmax - xmin);
  EXPECT_FLOAT_EQ(H, ymax - ymin);

  // Walls present and classified: query left wall (x in [-0.25,0])
  {
    b2AABB aabb{};
    aabb.lowerBound = {-0.2f, 0.5f};
    aabb.upperBound = { 0.0f, H - 0.5f};

    bool hit = false;
    auto fWall = makeFilter(~0ull, to_underlying(Type::BodyType::BoundingWall));
    (void)b2World_OverlapAABB(E.getB2World(), b2Pos_zero, aabb, fWall, StopOnFirst, &hit);
    EXPECT_TRUE(hit);

    hit = false;
    auto fObst = makeFilter(~0ull, to_underlying(Type::BodyType::Obstacle));
    (void)b2World_OverlapAABB(E.getB2World(), b2Pos_zero, aabb, fObst, StopOnFirst, &hit);
    EXPECT_FALSE(hit);
  }

  // If obstacles exist, test isFree/raycast against the first one
  if (Eenv["obstacles"] && Eenv["obstacles"].size() > 0) {
    const auto& obs = Eenv["obstacles"][0];
    if (obs["type"].as<std::string>("") == "box") {
      const auto& c = obs["center"];
      const auto& s = obs["size"];
      const b2Vec2 center{ c[0].as<float>(), c[1].as<float>() };
      const float rx = 0.05f, ry = 0.05f;

      // isFree -> false at obstacle center
      {
        b2AABB aabb{};
        aabb.lowerBound = {center.x - rx, center.y - ry};
        aabb.upperBound = {center.x + rx, center.y + ry};
        EXPECT_FALSE(E.isFreeAABB(aabb));
      }

      // raycast across obstacle -> "hit" (true)
      {
        const b2Vec2 pL{ center.x - 2.0f, center.y };
        const b2Vec2 pR{ center.x + 2.0f, center.y };
        E.setRobotRadius(std::max(0.05f, 0.25f)); // be a bit conservative
        EXPECT_TRUE(E.raycast(pL, pR));
      }
    }

  }

  // sampleFree returns points that are free (under current radius inflation)
  {
    E.setRobotRadius(0.10f);
    const auto pts = E.sampleFree(32);
    EXPECT_EQ(pts.size(), 32u);
    for (const auto& p : pts) {
      b2AABB a{};
      const float r = 0.10f;
      a.lowerBound = {p.x - r, p.y - r};
      a.upperBound = {p.x + r, p.y + r};
      EXPECT_TRUE(E.isFreeAABB(a));
    }
  }
}

TEST(FreeEnv, RejectsDegenerateConvexPolygonsWithoutMutation)
{
    ContinuousEnvironment environment;
    environment.setBounds(10.0f, 10.0f);

    const auto expectRejected = [&](const std::vector<b2Vec2>& points) {
      EXPECT_THROW(
          environment.addConvexBody(
              points, Type::BodyType::Obstacle),
          std::invalid_argument);
      EXPECT_EQ(environment.numObstacles(), 0U);
      EXPECT_FLOAT_EQ(environment.obstacleAreaInMeter2(), 0.0f);
    };

    expectRejected({});
    expectRejected({{0.0f, 0.0f}, {1.0f, 1.0f}});
    expectRejected({
        {0.0f, 0.0f},
        {1.0f, 1.0f},
        {2.0f, 2.0f}});
    expectRejected({
        {0.0f, 0.0f},
        {0.0f, 0.0f},
        {1.0f, 1.0f}});
    expectRejected({
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {std::numeric_limits<float>::infinity(), 1.0f}});

    std::vector<b2Vec2> tooMany(
        B2_MAX_POLYGON_VERTICES + 1, b2Vec2{});
    expectRejected(tooMany);

    const std::vector<b2Vec2> validTriangle{
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {0.0f, 1.0f}};
    EXPECT_THROW(
        environment.addConvexBody(
            validTriangle, Type::BodyType::Obstacle, -0.1f),
        std::invalid_argument);
    EXPECT_THROW(
        environment.addConvexBody(
            validTriangle,
            Type::BodyType::Obstacle,
            std::numeric_limits<float>::quiet_NaN()),
        std::invalid_argument);
    EXPECT_EQ(environment.numObstacles(), 0U);
}

TEST(FreeEnv, AcceptsValidConvexPolygon)
{
    ContinuousEnvironment environment;
    environment.setBounds(10.0f, 10.0f);

    const b2BodyId body = environment.addConvexBody(
        {{1.0f, 1.0f},
         {3.0f, 1.0f},
         {3.0f, 3.0f},
         {1.0f, 3.0f}},
        Type::BodyType::Obstacle);

    EXPECT_TRUE(B2_IS_NON_NULL(body));
    EXPECT_EQ(environment.numObstacles(), 1U);
    EXPECT_FLOAT_EQ(environment.obstacleAreaInMeter2(), 4.0f);
}

TEST(FreeEnv, IncludesRoundedRadiusInConvexPolygonArea)
{
    ContinuousEnvironment environment;
    environment.setBounds(10.0f, 10.0f);

    environment.addConvexBody(
        {{1.0f, 1.0f},
         {3.0f, 1.0f},
         {3.0f, 3.0f},
         {1.0f, 3.0f}},
        Type::BodyType::Obstacle,
        0.5f);

    const float expected =
        4.0f + 8.0f * 0.5f + B2_PI * 0.25f;
    EXPECT_NEAR(
        environment.obstacleAreaInMeter2(),
        expected,
        1.0e-5f);
}

TEST(FreeEnv, PublicResetClearsContinuousMetadataPolymorphically)
{
    ContinuousEnvironment environment;
    environment.setBounds(10.0f, 8.0f);
    environment.setRobotRadius(1.0f);
    environment.addBoundaryWalls(0.25f);
    environment.addCircleBody(
        {2.0f, 2.0f}, 0.5f, Type::BodyType::Obstacle);

    BaseEnvironment& base = environment;
    base.resetEnvironment();

    EXPECT_FLOAT_EQ(environment.boundsInMeter().width, 0.0f);
    EXPECT_FLOAT_EQ(environment.boundsInMeter().height, 0.0f);
    EXPECT_EQ(environment.numObstacles(), 0U);
    EXPECT_EQ(environment.numBoundingWalls(), 0U);
    EXPECT_EQ(environment.numRobots(), 0U);

    environment.setBounds(5.0f, 3.0f);
    environment.addBoxBody(
        {2.5f, 1.8f}, 0.1f, 0.1f, 0.0f,
        Type::BodyType::Obstacle);
    EXPECT_FALSE(environment.raycast({0.5f, 1.0f}, {4.5f, 1.0f}));
}
