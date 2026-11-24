// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "TestFiles.h"
#include "env/GridEnvironment.h"
#include "io/GridMovingAI.h"
#include "io/GridXML.h"

using grace::test::TempDirectory;

namespace {

template <typename T>
void expectFailure(const grace::io::ReadResult<T>& result)
{
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.value.has_value());
    EXPECT_FALSE(result.error.empty());
}

void makeGrid(GridEnvironment& grid)
{
    grid.makeGridEnvironment(3, 2, 1.0f, std::vector<bool>(6, false));
}

}  // namespace

TEST(GridMovingAIIO, ReadsValidMap)
{
    TempDirectory temp;
    const auto path = temp.write(
        "valid.map",
        "type octile\nheight 2\nwidth 3\nmap\n.@.\nTG.\n");

    const auto result = grace::io::readGridMovingAIMap(path);

    ASSERT_TRUE(result.success) << result.error;
    ASSERT_TRUE(result.value);
    EXPECT_EQ(result.value->width, 3);
    EXPECT_EQ(result.value->height, 2);
    EXPECT_EQ(
        result.value->occupancy,
        (std::vector<bool>{false, true, false, true, false, false}));
}

TEST(GridMovingAIIO, RejectsMissingFileAndMalformedHeader)
{
    TempDirectory temp;
    expectFailure(
        grace::io::readGridMovingAIMap(temp.path("missing.map")));

    const auto path = temp.write(
        "bad-header.map",
        "type octile\nheight nope\nwidth 3\nmap\n...\n...\n");
    expectFailure(grace::io::readGridMovingAIMap(path));
}

TEST(GridMovingAIIO, RejectsTruncatedShortAndLongRows)
{
    TempDirectory temp;
    const auto truncated = temp.write(
        "truncated.map",
        "type octile\nheight 2\nwidth 3\nmap\n...\n");
    const auto shortRow = temp.write(
        "short.map",
        "type octile\nheight 2\nwidth 3\nmap\n..\n...\n");
    const auto longRow = temp.write(
        "long.map",
        "type octile\nheight 2\nwidth 3\nmap\n....\n...\n");

    expectFailure(grace::io::readGridMovingAIMap(truncated));
    expectFailure(grace::io::readGridMovingAIMap(shortRow));
    expectFailure(grace::io::readGridMovingAIMap(longRow));
}

TEST(GridMovingAIIO, RejectsCellCountsBeyondEnvironmentIndexRange)
{
    TempDirectory temp;
    const auto path = temp.write(
        "oversized.map",
        "type octile\nheight 46341\nwidth 46341\nmap\n");

    expectFailure(grace::io::readGridMovingAIMap(path));
}

TEST(GridXmlIO, ReadsValidMap)
{
    TempDirectory temp;
    const auto path = temp.write(
        "valid.xml",
        "<root><map><width>3</width><height>2</height><grid>"
        "<row>0 1 0</row><row>1 0 0</row>"
        "</grid></map></root>");

    const auto result = grace::io::readGridXmlMap(path);

    ASSERT_TRUE(result.success) << result.error;
    ASSERT_TRUE(result.value);
    EXPECT_EQ(result.value->width, 3);
    EXPECT_EQ(result.value->height, 2);
    EXPECT_EQ(
        result.value->occupancy,
        (std::vector<bool>{false, true, false, true, false, false}));
}

TEST(GridXmlIO, RejectsMissingFileDimensionsAndGridRows)
{
    TempDirectory temp;
    expectFailure(grace::io::readGridXmlMap(temp.path("missing.xml")));

    const auto badDimensions = temp.write(
        "bad-dimensions.xml",
        "<root><map><width>-3</width><height>2</height><grid>"
        "<row>0 0 0</row><row>0 0 0</row>"
        "</grid></map></root>");
    const auto shortRow = temp.write(
        "short-row.xml",
        "<root><map><width>3</width><height>2</height><grid>"
        "<row>0 0</row><row>0 0 0</row>"
        "</grid></map></root>");
    const auto missingRow = temp.write(
        "missing-row.xml",
        "<root><map><width>3</width><height>2</height><grid>"
        "<row>0 0 0</row>"
        "</grid></map></root>");

    expectFailure(grace::io::readGridXmlMap(badDimensions));
    expectFailure(grace::io::readGridXmlMap(shortRow));
    expectFailure(grace::io::readGridXmlMap(missingRow));
}

TEST(GridXmlIO, RejectsCellCountsBeyondEnvironmentIndexRange)
{
    TempDirectory temp;
    const auto path = temp.write(
        "oversized.xml",
        "<root><map><width>46341</width><height>46341</height><grid/>"
        "</map></root>");

    expectFailure(grace::io::readGridXmlMap(path));
}

TEST(GridMovingAIIO, ReadsValidScenario)
{
    TempDirectory temp;
    GridEnvironment grid;
    makeGrid(grid);
    const auto path = temp.write(
        "valid.scen",
        "version 1\n"
        "0 map.map 3 2 0 0 2 1 3.0\n");

    const auto result =
        grace::io::readGridMovingAIScenario(grid, path);

    ASSERT_TRUE(result.success) << result.error;
    ASSERT_TRUE(result.value);
    ASSERT_EQ(result.value->size(), 1u);
}

TEST(GridMovingAIIO, RejectsMissingMalformedVersionAndRecords)
{
    TempDirectory temp;
    expectFailure(grace::io::detail::parseMovingAIScenario(
        temp.path("missing.scen")));

    const auto badVersion = temp.write(
        "bad-version.scen",
        "version unknown\n"
        "0 map.map 3 2 0 0 2 1 3.0\n");
    const auto badRecord = temp.write(
        "bad-record.scen",
        "version 1\n"
        "0 map.map 3 2 0 0 2\n");

    expectFailure(
        grace::io::detail::parseMovingAIScenario(badVersion));
    expectFailure(
        grace::io::detail::parseMovingAIScenario(badRecord));
}

TEST(GridMovingAIIO, RejectsScenarioThatDoesNotMatchGrid)
{
    TempDirectory temp;
    GridEnvironment grid;
    makeGrid(grid);
    const auto path = temp.write(
        "wrong-grid.scen",
        "version 1\n"
        "0 map.map 4 2 0 0 3 1 3.0\n");

    expectFailure(grace::io::readGridMovingAIScenario(grid, path));
}

TEST(GridXmlIO, ReadsValidTasks)
{
    TempDirectory temp;
    GridEnvironment grid;
    makeGrid(grid);
    const auto path = temp.write(
        "valid-tasks.xml",
        "<root><agents>"
        "<agent name=\"r1\" sr=\"0\" sc=\"0\" gr=\"1\" gc=\"2\"/>"
        "</agents></root>");

    const auto result = grace::io::readGridXmlTask(grid, path);

    ASSERT_TRUE(result.success) << result.error;
    ASSERT_TRUE(result.value);
    EXPECT_EQ(result.value->size(), 1u);
}

TEST(GridXmlIO, RejectsMalformedTaskXmlAndAttributes)
{
    TempDirectory temp;
    GridEnvironment grid;
    makeGrid(grid);
    expectFailure(
        grace::io::readGridXmlTask(grid, temp.path("missing-tasks.xml")));

    const auto malformedXml = temp.write(
        "malformed-tasks.xml",
        "<root><agents><agent sr=\"0\"></agents></root>");
    const auto badAttribute = temp.write(
        "bad-attribute.xml",
        "<root><agents>"
        "<agent sr=\"zero\" sc=\"0\" gr=\"1\" gc=\"2\"/>"
        "</agents></root>");

    expectFailure(grace::io::readGridXmlTask(grid, malformedXml));
    expectFailure(grace::io::readGridXmlTask(grid, badAttribute));
}
