// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>
#include "TestFiles.h"
#include "env/RoadmapEnvironment.h"
#include "io/RoadmapXML.h"

using grace::test::TempDirectory;

// Tiny GraphML with coords & weight keys; undirected edges by option
static const char* kGraphML = R"(<?xml version="1.0" encoding="UTF-8"?>
<graphml xmlns="http://graphml.graphdrawing.org/xmlns"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
  <key id="key0" for="node" attr.name="coords" attr.type="string"/>
  <key id="key1" for="edge" attr.name="weight" attr.type="double"/>
  <graph id="G" edgedefault="directed">
    <node id="n0"><data key="key0">0,0</data></node>
    <node id="n1"><data key="key0">10,0</data></node>
    <node id="n2"><data key="key0">10,10</data></node>
    <node id="n3"><data key="key0">0,10</data></node>
    <edge id="e0" source="n0" target="n1"><data key="key1">10</data></edge>
    <edge id="e1" source="n1" target="n2"><data key="key1">10</data></edge>
    <edge id="e2" source="n2" target="n3"><data key="key1">10</data></edge>
    <edge id="e3" source="n3" target="n0"><data key="key1">10</data></edge>
  </graph>
</graphml>
)";

static const char* kTasks = R"(<?xml version="1.0"?>
<root>
  <agent start_id="0" goal_id="2"/>
  <agent start_id="1" goal_id="3"/>
</root>
)";

TEST(RoadmapIO, LoadGraphMLAndTasks)
{
    TempDirectory temp;
    const auto gPath = temp.write("tiny.graphml", kGraphML);
    const auto tPath = temp.write("tiny_tasks.xml", kTasks);

    RoadmapEnvironment R;
    grace::sim::GraphMLImport import;
    grace::sim::GraphMLOptions opt;
    opt.scale = 0.1f;        // 10 units -> 1.0 m
    opt.undirected = true;   // add both directions even though graph says "directed"
    opt.useEdgeWeight = true;

    ASSERT_TRUE(grace::sim::readGraphML(gPath, R, &import, opt));
    EXPECT_EQ(R.numNodes(), 4);

    // Coordinates should be scaled to meters
    EXPECT_FLOAT_EQ(R.nodePos(0).x, 0.0f);
    EXPECT_FLOAT_EQ(R.nodePos(1).x, 1.0f);
    EXPECT_FLOAT_EQ(R.nodePos(2).y, 1.0f);
    EXPECT_FLOAT_EQ(R.nodePos(3).x, 0.0f);

    // Weight honored (10 units * 0.1 scale -> still 10 weight by design; weight is absolute here)
    EXPECT_FLOAT_EQ(R.edgeLength(0,1), 10.f);
    EXPECT_FLOAT_EQ(R.edgeLength(1,0), 10.f); // undirected added

    const auto tasks = grace::sim::readTasksXml(tPath, import);
    ASSERT_EQ(tasks.size(), 2u);
    EXPECT_EQ(tasks[0].first, 0);
    EXPECT_EQ(tasks[0].second, 2);
    EXPECT_EQ(tasks[1].first, 1);
    EXPECT_EQ(tasks[1].second, 3);
}
