// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>
#include "TestFiles.h"
#include "env/GridEnvironment.h"
#include "io/GridMovingAI.h"
#include "io/GridXML.h"
#include "util/Types.h"
#include <box2d/box2d.h>
#include "util/Logger.h"
#include "util/RunOutput.h"
#include <limits>

using grace::test::TempDirectory;

namespace {
    static bool StopOnFirst(b2ShapeId, void* ctx) { *static_cast<bool*>(ctx) = true; return false; }

    void loadMovingAI(
        GridEnvironment& env,
        const std::filesystem::path& path,
        float cellLength) {
        const auto map = grace::io::readGridMovingAIMap(path);
        ASSERT_TRUE(map) << map.error;
        env.makeGridEnvironment(
            map.value->width, map.value->height, cellLength,
            map.value->occupancy);
    }

    TEST(MAPFEnv, ResetDoesNotRotateRunOutputSession) {
        const auto runDirectory = grace::output::currentRunDirectory();
        ASSERT_FALSE(runDirectory.empty());

        GridEnvironment environment;
        environment.makeGridEnvironment(2, 2, 1.0f, std::vector<bool>(4, false));
        environment.resetEnvironment();

        EXPECT_EQ(grace::output::currentRunDirectory(), runDirectory);
    }

    void loadXml(
        GridEnvironment& env,
        const std::filesystem::path& path,
        float cellLength) {
        const auto map = grace::io::readGridXmlMap(path);
        ASSERT_TRUE(map) << map.error;
        env.makeGridEnvironment(
            map.value->width, map.value->height, cellLength,
            map.value->occupancy);
    }

    inline b2QueryFilter MakeObstacleQueryFilter(uint64_t obstacleBit) {
        b2QueryFilter q{};
        q.categoryBits = ~0ull;         // query belongs to all categories
        q.maskBits     = obstacleBit;   // accept shapes that include Obstacle in their category
        return q;
    }

    inline b2QueryFilter makeFilter(uint64_t catBits, uint64_t maskBits) {
        b2QueryFilter q{}; q.categoryBits = catBits; q.maskBits = maskBits; return q;
    }
}

// ------------------------- 1) cell length scaling ----------------------------
TEST(MAPFEnv, loadMovingAI_CellLengthScaling)
{
    GridEnvironment env;
    TempDirectory temp;

    const char* mapTxt =
        "type octile\n"
        "height 3\n"
        "width  4\n"
        "map\n"
        ".T..\n"
        "..@.\n"
        "...T\n";

    const auto path = temp.write("tiny_scale.map", mapTxt);

    // Use half-meter cells
    const float cell = 0.5f;
    ASSERT_NO_THROW(loadMovingAI(env, path, cell));

    // 4x3 cells -> 2.0m x 1.5m
    EXPECT_FLOAT_EQ(env.gridWidthInMeter(),  4.0f * cell);
    EXPECT_FLOAT_EQ(env.gridHeightInMeter(), 3.0f * cell);

    // At least area of 3 blocked cells (3 * 0.5^2 = 0.75).
    // Boundary walls (if counted) only increase this.
    EXPECT_GE(env.obstacleAreaInMeter2(), 3.0f * cell * cell);

}

// ------------------------- 2) XML loader parity ------------------------------
TEST(MAPFEnv, loadXmlGrid_Parity)
{
    GridEnvironment env;
    TempDirectory temp;

    // Build a tiny 2x3 grid and write as XML
    const int W = 4, H = 3;
    const float cell = 0.25f;
    // occupancy row-major: X=blocked
    // . X . .
    // . . X .
    // . . . X
    const char* xml =
        R"(<?xml version="1.0"?>
            <root>
              <map>
                <width>4</width>
                <height>3</height>
                <grid>
                  <row>0 1 0 0</row>
                  <row>0 0 1 0</row>
                  <row>0 0 0 1</row>
                </grid>
              </map>
            </root>
        )";
    const auto p = temp.write("grid.xml", xml);

    ASSERT_NO_THROW(loadXml(env, p, cell));

    EXPECT_FLOAT_EQ(env.gridWidthInMeter(),  W * cell);
    EXPECT_FLOAT_EQ(env.gridHeightInMeter(), H * cell);

    std::vector<bool> expected {
        false,true ,false,false,
        false,false,true ,false,
        false,false,false,true
    };
    EXPECT_EQ(env.getOccupancy(), expected);

    // Minimum area = 3 blocked cells * cell^2
    EXPECT_GE(env.obstacleAreaInMeter2(), 3.0f * cell * cell);

}

// --------------------- 3) Rebuild from occupancy is stable -------------------
TEST(MAPFEnv, rebuildBox2DFromOccupancy_IdempotentArea)
{
    GridEnvironment env;
    TempDirectory temp;

    const char* mapTxt =
        "type octile\n"
        "height 2\n"
        "width  3\n"
        "map\n"
        "@@.\n"
        "..@\n";

    const auto path = temp.write("tiny_rebuild.map", mapTxt);

    ASSERT_NO_THROW(loadMovingAI(env, path, 1.0f));

    const float area_before = env.obstacleAreaInMeter2();

    loadMovingAI(env, path, 1.0f);

    const float area_after = env.obstacleAreaInMeter2();

    // Should match (same blocked cells, same boundaries)
    EXPECT_FLOAT_EQ(area_before, area_after);

}

// --------------------- 4) Query a known cell with OverlapAABB ----------------

TEST(MAPFEnv, queryOverlapAABB_KnownCells)
{
    GridEnvironment env;
    TempDirectory temp;

    const char* mapTxt =
        "type octile\n"
        "height 2\n"
        "width  3\n"
        "map\n"
        "@..\n"  // (r=0,c=0) is blocked
        "...\n";

    const auto path = temp.write("tiny_query.map", mapTxt);

    ASSERT_NO_THROW(loadMovingAI(env, path, 1.0f));

    // Build AABB for cell (0,0) in meters (1m cells)
    const float cs = 1.0f;
    auto cellAABB = [&](int r, int c, float inset = 0.0f){
        b2AABB aabb{};
        aabb.lowerBound = b2Vec2{c*cs + inset,     r*cs + inset};
        aabb.upperBound = b2Vec2{(c+1)*cs - inset, (r+1)*cs - inset};
        return aabb;
    };

    const uint64_t OBST = (uint64_t)to_underlying(Type::BodyType::Obstacle);
    b2QueryFilter q = MakeObstacleQueryFilter(OBST);

    // Blocked cell should hit
    bool hit = false;
    (void)b2World_OverlapAABB(env.getB2World(), b2Pos_zero, cellAABB(0,0), q, StopOnFirst, &hit);
    EXPECT_TRUE(hit);

    // Free cell should not hit — use an inset to avoid touching the neighbor’s fat AABB.
    // Pick an inset comfortably larger than the broad-phase padding (e.g. 0.2 m for 1 m cells).
    hit = false;
    (void)b2World_OverlapAABB(
        env.getB2World(), b2Pos_zero, cellAABB(0,1, /*inset=*/0.2f), q, StopOnFirst, &hit);
    EXPECT_FALSE(hit);

}

// ------------------------ 5) Invalid inputs are handled ----------------------
TEST(MAPFEnv, loadMovingAI_InvalidHeaders)
{
    TempDirectory temp;

    // Missing 'map' section
    const char* bad1 =
        "type octile\n"
        "height 2\n"
        "width  2\n";
    const auto p1 = temp.write("bad_no_map.map", bad1);
    const auto missingMap = grace::io::readGridMovingAIMap(p1);
    EXPECT_FALSE(missingMap);
    EXPECT_FALSE(missingMap.value.has_value());
    EXPECT_FALSE(missingMap.error.empty());

    // Short row
    const char* bad2 =
        "type octile\n"
        "height 2\n"
        "width  3\n"
        "map\n"
        "@.\n"     // too short
        "...\n";
    const auto p2 = temp.write("bad_short_row.map", bad2);

    const auto shortRow = grace::io::readGridMovingAIMap(p2);
    EXPECT_FALSE(shortRow);
    EXPECT_FALSE(shortRow.value.has_value());
    EXPECT_FALSE(shortRow.error.empty());
}

TEST(MAPFEnv, rejectsMismatchedOccupancy)
{
    GridEnvironment env;
    env.makeGridEnvironment(2, 2, 1.0f, std::vector<bool>(4, false));

    Logger::instance().setThrowOnLevel(LogLevel::ERROR);
    EXPECT_THROW(
        env.makeGridEnvironment(3, 2, 1.0f, {}),
        std::runtime_error);
    Logger::instance().setThrowOnLevel(LogLevel::FATAL);

    EXPECT_EQ(env.gridWidthInCells(), 2);
    EXPECT_EQ(env.gridHeightInCells(), 2);
    EXPECT_EQ(env.getOccupancy(), std::vector<bool>(4, false));
}

TEST(MAPFEnv, rejectsInvalidGridScaleWithoutMutation)
{
    GridEnvironment env;
    env.makeGridEnvironment(2, 2, 1.0f, std::vector<bool>(4, false));

    Logger::instance().setThrowOnLevel(LogLevel::ERROR);
    EXPECT_THROW(
        env.makeGridEnvironment(3, 3, 0.0f, std::vector<bool>(9, false)),
        std::runtime_error);
    EXPECT_THROW(
        env.makeGridEnvironment(
            3,
            3,
            1.0f,
            std::vector<bool>(9, false),
            std::numeric_limits<float>::quiet_NaN()),
        std::runtime_error);
    Logger::instance().setThrowOnLevel(LogLevel::FATAL);

    EXPECT_EQ(env.gridWidthInCells(), 2);
    EXPECT_EQ(env.gridHeightInCells(), 2);
    EXPECT_FLOAT_EQ(env.cellLength(), 1.0f);
}

TEST(MAPFEnv, publicResetClearsGridMetadataPolymorphically)
{
    GridEnvironment env;
    env.makeGridEnvironment(
        3, 2, 0.5f,
        {true, false, false, false, true, false});
    ASSERT_EQ(env.numObstacles(), 2U);

    BaseEnvironment& base = env;
    base.resetEnvironment();

    EXPECT_EQ(env.gridWidthInCells(), 0);
    EXPECT_EQ(env.gridHeightInCells(), 0);
    EXPECT_FLOAT_EQ(env.cellLength(), 1.0f);
    EXPECT_TRUE(env.occupancy().empty());
    EXPECT_FLOAT_EQ(env.boundsInMeter().width, 0.0f);
    EXPECT_FLOAT_EQ(env.boundsInMeter().height, 0.0f);
    EXPECT_EQ(env.numObstacles(), 0U);
    EXPECT_EQ(env.numBoundingWalls(), 0U);
    EXPECT_EQ(env.numRobots(), 0U);
}

// ------------------------ 6) Boundary walls don’t change obstacle area ----------------------
TEST(MAPFEnv, boundaryWallsDontAffectObstacleArea)
{
    GridEnvironment env;
    TempDirectory temp;

    const char* mapTxt =
        "type octile\n"
        "height 2\n"
        "width  3\n"
        "map\n"
        "@..\n"
        "...\n";

    const auto path = temp.write("tiny_walls.map", mapTxt);
    loadMovingAI(env, path, 1.0f);

    // Exactly 1 blocked cell of 1m×1m
    EXPECT_FLOAT_EQ(env.obstacleAreaInMeter2(), 1.0f);

    // Rebuild walls again (no area change expected)
    env.addBoundaryWalls(0.5f);
    EXPECT_FLOAT_EQ(env.obstacleAreaInMeter2(), 1.0f);

}

// ------------------------ 7) Walls are present and queryable, obstacles filter won’t pick them ----------------------

TEST(MAPFEnv, wallsExistButNotCountedAndNotQueriedAsObstacles)
{
    GridEnvironment env;
    TempDirectory temp;

    const char* mapTxt =
        "type octile\n"
        "height 2\n"
        "width  3\n"
        "map\n"
        "...\n"
        "...\n";
    const auto path = temp.write("tiny_empty.map", mapTxt);
    loadMovingAI(env, path, 1.0f);

    // No obstacles area
    EXPECT_FLOAT_EQ(env.obstacleAreaInMeter2(), 0.0f);

    // AABB overlapping the left wall (x in [-0.5, 0]) at mid-height
    b2AABB aabb{};
    aabb.lowerBound = { -0.5f, 0.5f };
    aabb.upperBound = {  0.0f, 1.5f };

    // Query as "Obstacle" — should NOT hit (walls are BoundingWall)
    bool hit = false;
    auto fObstacle = makeFilter(~0ull, to_underlying(Type::BodyType::Obstacle));
    (void)b2World_OverlapAABB(env.getB2World(), b2Pos_zero, aabb, fObstacle, StopOnFirst, &hit);
    EXPECT_FALSE(hit);

    // Query as "BoundingWall" — SHOULD hit
    hit = false;
    auto fWall = makeFilter(~0ull, to_underlying(Type::BodyType::BoundingWall));
    (void)b2World_OverlapAABB(env.getB2World(), b2Pos_zero, aabb, fWall, StopOnFirst, &hit);
    EXPECT_TRUE(hit);

}
