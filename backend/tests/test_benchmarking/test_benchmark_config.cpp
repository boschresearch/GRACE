// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "benchmarking/BenchmarkConfig.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

using grace::benchmarking::effectiveTakeK;
using grace::benchmarking::parseTakeKTokens;
using grace::benchmarking::pathStem;
using grace::benchmarking::takeKFolderTag;

TEST(BenchmarkConfig, ParsesAndDeduplicatesTakeK)
{
    EXPECT_EQ(
        parseTakeKTokens({"ALL", "4", "4", "2"}),
        (std::vector<int>{-1, 4, 2}));
    EXPECT_EQ(parseTakeKTokens({}), (std::vector<int>{-1}));
}

TEST(BenchmarkConfig, RejectsInvalidTakeK)
{
    EXPECT_THROW(parseTakeKTokens({"0"}), std::invalid_argument);
    EXPECT_THROW(parseTakeKTokens({"-2"}), std::invalid_argument);
    EXPECT_THROW(parseTakeKTokens({"3robots"}), std::invalid_argument);
}

TEST(BenchmarkConfig, ComputesEffectiveTakeKAndFolder)
{
    EXPECT_EQ(effectiveTakeK(8, 5), 5U);
    EXPECT_EQ(effectiveTakeK(3, 5), 3U);
    EXPECT_EQ(
        effectiveTakeK(-1, 5), std::numeric_limits<std::size_t>::max());
    EXPECT_EQ(takeKFolderTag(-1, 5), "all");
    EXPECT_EQ(takeKFolderTag(8, 5), "5");
}

TEST(BenchmarkConfig, SanitizesFolderStem)
{
    EXPECT_EQ(pathStem("/maps/my map,1.scen"), "my_map_1");
}

TEST(BenchmarkConfig, RejectsInvalidBoundaryScale)
{
    grace::benchmarking::BenchmarkConfig config;
    config.gridMaps = {"map.map"};
    config.gridScenarios = {"agents.scen"};
    config.solvers = {"planner"};
    config.gridBoundaryScale =
        std::numeric_limits<float>::quiet_NaN();

    EXPECT_THROW(
        grace::benchmarking::validateBenchmarkConfig(config),
        std::invalid_argument);
}

} // namespace
