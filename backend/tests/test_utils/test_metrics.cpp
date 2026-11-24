// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// tests/test_metrics.cpp
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <box2d/box2d.h>
#include "util/Metrics.h"
#include "planner/Planner.h"

using namespace metrics;

static inline bool isNaN(double v) { return std::isnan(v); }

TEST(PolylineLength, EmptyAndSingletonZero)
{
    std::vector<Waypoint> wps0;
    std::vector<Waypoint> wps1 = { { b2Vec2{0.0f, 0.0f}, 0.0f } };
    EXPECT_DOUBLE_EQ(polylineLength(wps0), 0.0);
    EXPECT_DOUBLE_EQ(polylineLength(wps1), 0.0);
}

TEST(PolylineLength, SimpleDistances)
{
    // (0,0) -> (3,4) -> (3,8) : lengths 5 and 4, total = 9
    std::vector<Waypoint> wps = {
        { b2Vec2{0.0f, 0.0f}, 0.0f },
        { b2Vec2{3.0f, 4.0f}, 1.0f },
        { b2Vec2{3.0f, 8.0f}, 2.0f }
    };
    EXPECT_NEAR(polylineLength(wps), 9.0, 1e-9);
}

TEST(LastTimeIfTimed, TimedVsUntimed)
{
    // Timed plan
    std::vector<Waypoint> timed = {
        { b2Vec2{0.0f, 0.0f}, 0.0f },
        { b2Vec2{1.0f, 0.0f}, 3.5f }
    };
    EXPECT_DOUBLE_EQ(lastTimeIfTimed(timed), 3.5);

    // Untimed: last t < 0 or empty
    std::vector<Waypoint> untimed = {
        { b2Vec2{0.0f, 0.0f}, -1.0f },
        { b2Vec2{1.0f, 0.0f}, -1.0f }
    };
    EXPECT_TRUE(isNaN(lastTimeIfTimed(untimed)));

    std::vector<Waypoint> empty;
    EXPECT_TRUE(isNaN(lastTimeIfTimed(empty)));
}

TEST(PerAgentFromPlan, BasicFields)
{
    Plan p;
    p.waypoints = {
        { b2Vec2{0.0f, 0.0f}, 0.0f },
        { b2Vec2{3.0f, 4.0f}, 5.0f } // length 5, completion 5
    };

    b2BodyId id{}; // default; we only propagate it through the struct
    PerAgent pa = perAgentFromPlan(p, id, "r1");

    EXPECT_EQ(pa.name, "r1");
    EXPECT_EQ(pa.waypoints, 2);
    EXPECT_NEAR(pa.path_length_m, 5.0, 1e-9);
    EXPECT_DOUBLE_EQ(pa.completion_time_s, 5.0);
}

TEST(Summarize, TimedTwoAgents)
{
    // Agent 1: length 5, completion 5
    Plan p1; p1.waypoints = {
        { b2Vec2{0.0f, 0.0f}, 0.0f },
        { b2Vec2{3.0f, 4.0f}, 5.0f }
    };
    // Agent 2: two segments: (0,0)->(0,3)->(4,3) length 7, completion 7
    Plan p2; p2.waypoints = {
        { b2Vec2{0.0f, 0.0f}, 0.0f },
        { b2Vec2{0.0f, 3.0f}, 3.0f },
        { b2Vec2{4.0f, 3.0f}, 7.0f }
    };

    b2BodyId idA{}, idB{};
    PerAgent a = perAgentFromPlan(p1, idA, "a");
    PerAgent b = perAgentFromPlan(p2, idB, "b");

    auto agg = summarize({a, b}, /*planner_runtime_s=*/0.123, /*sim_runtime_s=*/4.5);

    EXPECT_EQ(agg.agents, 2u);
    EXPECT_NEAR(agg.sum_length_m, 12.0, 1e-9);
    EXPECT_NEAR(agg.avg_length_m, 6.0, 1e-9);

    // flowtime = 5 + 7 = 12, makespan = 7, avg_completion = 6
    EXPECT_NEAR(agg.sum_of_cost_s, 12.0, 1e-9);
    EXPECT_NEAR(agg.makespan_s, 7.0, 1e-9);
    EXPECT_NEAR(agg.avg_completion_s, 6.0, 1e-9);

    EXPECT_NEAR(agg.planner_runtime_s, 0.123, 1e-12);
    EXPECT_NEAR(agg.sim_runtime_s, 4.5, 1e-12);
}

TEST(Summarize, UntimedAllNaN)
{
    Plan p; p.waypoints = {
        { b2Vec2{0.0f, 0.0f}, -1.0f },
        { b2Vec2{1.0f, 0.0f}, -1.0f }
    };
    PerAgent a = perAgentFromPlan(p, b2BodyId{}, "untimed");

    auto agg = summarize({a});

    EXPECT_EQ(agg.agents, 1u);
    EXPECT_FALSE(isNaN(agg.sum_length_m)); // length is geometric, should be valid
    EXPECT_TRUE(isNaN(agg.sum_of_cost_s));
    EXPECT_TRUE(isNaN(agg.makespan_s));
    EXPECT_TRUE(isNaN(agg.avg_completion_s));
}

TEST(Summarize, MixedNaNHandling)
{
    // A1 timed completion = 4
    Plan p1; p1.waypoints = {
        { b2Vec2{0.0f, 0.0f}, 0.0f },
        { b2Vec2{4.0f, 0.0f}, 4.0f }
    };
    // A2 untimed completion = NaN
    Plan p2; p2.waypoints = {
        { b2Vec2{0.0f, 0.0f}, -1.0f },
        { b2Vec2{0.0f, 3.0f}, -1.0f }
    };

    PerAgent a = perAgentFromPlan(p1, b2BodyId{}, "a");
    PerAgent b = perAgentFromPlan(p2, b2BodyId{}, "b");

    auto agg = summarize({a, b});

    // sum_of_cost should sum only valid completion times (=4)
    EXPECT_NEAR(agg.sum_of_cost_s, 4.0, 1e-9);
    // makespan should be max of valid values (=4)
    EXPECT_NEAR(agg.makespan_s, 4.0, 1e-9);
    // avg_completion over valid entries only (1 value)
    EXPECT_NEAR(agg.avg_completion_s, 4.0, 1e-9);
}
