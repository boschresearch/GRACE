// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>
#include "env/RoadmapEnvironment.h"
#include "util/Logger.h"
#include <box2d/box2d.h>
#include <fstream>
#include <cmath>

namespace {

inline float dist(b2Vec2 a, b2Vec2 b) {
    const float dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy);
}

} // namespace

// ------------------------- 1) Node/edge basics -------------------------------
TEST(RoadmapEnv, NodeEdgeBasics)
{
    RoadmapEnvironment R;
    R.setBounds(10.f, 10.f);
    R.addBoundaryWalls(0.25f);

    auto n0 = R.addNode({1.f, 1.f});
    auto n1 = R.addNode({5.f, 1.f});
    auto n2 = R.addNode({5.f, 4.f});

    EXPECT_EQ(R.numNodes(), 3);
    EXPECT_FLOAT_EQ(R.nodePos(n0).x, 1.f);
    EXPECT_FLOAT_EQ(R.nodePos(n0).y, 1.f);

    // Undirected edge
    R.addEdge(n0, n1);
    ASSERT_FALSE(R.adj(n0).empty());
    ASSERT_FALSE(R.adj(n1).empty());
    // Directed edge
    R.addEdgeDirected(n1, n2);
    bool n2_in_adj_n1 = false;
    for (auto v : R.adj(n1)) if (v == n2) n2_in_adj_n1 = true;
    EXPECT_TRUE(n2_in_adj_n1);

    bool n1_in_adj_n2 = false;
    for (auto v : R.adj(n2)) if (v == n1) n1_in_adj_n2 = true;
    EXPECT_FALSE(n1_in_adj_n2); // directed, so no back edge

    // Edge length = Euclidean by default
    EXPECT_NEAR(R.edgeLength(n0, n1), dist(R.nodePos(n0), R.nodePos(n1)), 1e-6);

    // Override weight (undirected)
    R.setEdgeWeight(n0, n1, /*w=*/42.f, /*bidir=*/true);
    EXPECT_FLOAT_EQ(R.edgeLength(n0, n1), 42.f);
    EXPECT_FLOAT_EQ(R.edgeLength(n1, n0), 42.f);

    // Override directed weight
    R.setEdgeWeight(n1, n2, /*w=*/3.14f, /*bidir=*/false);
    EXPECT_FLOAT_EQ(R.edgeLength(n1, n2), 3.14f);
    // Reverse still Euclidean (no undirected weight set)
    EXPECT_NEAR(R.edgeLength(n2, n1), dist(R.nodePos(n1), R.nodePos(n2)), 1e-6);
}

// ------------------------- 2) nearestNode sanity -----------------------------
TEST(RoadmapEnv, NearestNode)
{
    RoadmapEnvironment R;
    R.setBounds(20.f, 20.f);
    auto a = R.addNode({2.f, 2.f});
    auto b = R.addNode({10.f, 2.f});
    auto c = R.addNode({2.f, 10.f});

    auto n = R.nearestNode({1.9f, 1.8f});
    EXPECT_EQ(n, a);

    n = R.nearestNode({8.0f, 1.9f});
    EXPECT_EQ(n, b);

    n = R.nearestNode({1.8f, 9.9f});
    EXPECT_EQ(n, c);
}

// ------------------------- 3) Walls exist & are queryable --------------------
namespace {
    typedef bool b2OverlapResultFcn(b2ShapeId, void*);
    static bool StopOnFirst(b2ShapeId, void* ctx) { *static_cast<bool*>(ctx) = true; return false; }

    inline b2QueryFilter makeFilter(uint64_t catBits, uint64_t maskBits) {
        b2QueryFilter q{}; q.categoryBits = catBits; q.maskBits = maskBits; return q;
    }
}

TEST(RoadmapEnv, BoundaryWallsPresentAndClassified)
{
    RoadmapEnvironment R;
    R.setBounds(4.f, 2.f);
    R.addBoundaryWalls(0.5f);

    // Query a left-wall AABB: x in [-0.5, 0]
    b2AABB aabb{};
    aabb.lowerBound = {-0.4f, 0.5f};
    aabb.upperBound = { 0.0f, 1.5f};

    bool hit = false;
    auto fWall = makeFilter(~0ull, to_underlying(Type::BodyType::BoundingWall));
    (void)b2World_OverlapAABB(R.getB2World(), b2Pos_zero, aabb, fWall, StopOnFirst, &hit);
    EXPECT_TRUE(hit);

    hit = false;
    auto fObst = makeFilter(~0ull, to_underlying(Type::BodyType::Obstacle));
    (void)b2World_OverlapAABB(R.getB2World(), b2Pos_zero, aabb, fObst, StopOnFirst, &hit);
    EXPECT_FALSE(hit);
}

TEST(RoadmapEnv, PublicResetClearsGraphMetadataPolymorphically)
{
    RoadmapEnvironment environment;
    environment.setBounds(8.0f, 6.0f);
    const auto first = environment.addNode({0.0f, 0.0f});
    const auto second = environment.addNode({3.0f, 4.0f});
    environment.addEdge(first, second);
    environment.setEdgeWeight(first, second, 42.0f, true);
    environment.addBoundaryWalls(0.25f);

    BaseEnvironment& base = environment;
    base.resetEnvironment();

    EXPECT_EQ(environment.numNodes(), 0);
    EXPECT_EQ(environment.nearestNode({0.0f, 0.0f}), -1);
    EXPECT_FLOAT_EQ(environment.boundsInMeter().width, 0.0f);
    EXPECT_FLOAT_EQ(environment.boundsInMeter().height, 0.0f);
    EXPECT_EQ(environment.numBoundingWalls(), 0U);

    const auto resetFirst = environment.addNode({0.0f, 0.0f});
    const auto resetSecond = environment.addNode({3.0f, 4.0f});
    environment.addEdge(resetFirst, resetSecond);
    EXPECT_FLOAT_EQ(
        environment.edgeLength(resetFirst, resetSecond),
        5.0f);
}
