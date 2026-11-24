// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "TestFiles.h"
#include "util/Logger.h"
#include "util/RunOutput.h"

#include <fstream>
#include <iterator>

TEST(RunOutput, CreatesSessionDirectoryAndLog) {
    grace::test::TempDirectory temp;
    grace::output::RunOutputConfig config;
    config.baseDirectory = temp.path("results");
    config.timestamp = "fixed-session";
    config.throwLevel = LogLevel::FATAL;
    const auto directory = grace::output::startRunOutput(config);

    EXPECT_EQ(directory, grace::output::currentRunDirectory());
    EXPECT_TRUE(std::filesystem::is_directory(directory));
    Logger::instance().info("run-output-test-message");
    grace::output::stopRunOutput();

    std::ifstream log(directory / "logging.log");
    const std::string contents{
        std::istreambuf_iterator<char>{log},
        std::istreambuf_iterator<char>{}};
    EXPECT_NE(contents.find("run-output-test-message"), std::string::npos);
    EXPECT_TRUE(grace::output::currentRunDirectory().empty());
}

TEST(RunOutput, RejectsEmptyBaseDirectory) {
    grace::output::RunOutputConfig config;
    config.baseDirectory.clear();
    EXPECT_THROW(
        (void)grace::output::startRunOutput(config),
        std::invalid_argument);
}

TEST(RunOutput, FailedRotationPreservesCurrentSession) {
    grace::test::TempDirectory temp;
    grace::output::RunOutputConfig config;
    config.baseDirectory = temp.path("results");
    config.timestamp = "working-session";
    config.throwLevel = LogLevel::FATAL;
    const auto original = grace::output::startRunOutput(config);

    const auto blockedBase = temp.path("blocked");
    std::filesystem::create_directories(
        blockedBase / "unavailable" / "logging.log");
    config.baseDirectory = blockedBase;
    config.timestamp = "unavailable";
    config.logLevel = LogLevel::TRACE;
    config.throwLevel = LogLevel::ERROR;

    EXPECT_THROW(
        (void)grace::output::startRunOutput(config),
        std::runtime_error);
    EXPECT_EQ(grace::output::currentRunDirectory(), original);
    EXPECT_FALSE(Logger::instance().isEnabled(LogLevel::DEBUG));
    EXPECT_NO_THROW(Logger::instance().error("preserved-throw-threshold"));

    Logger::instance().info("original-session-remains-active");
    grace::output::stopRunOutput();
    std::ifstream log(original / "logging.log");
    const std::string contents{
        std::istreambuf_iterator<char>{log},
        std::istreambuf_iterator<char>{}};
    EXPECT_NE(
        contents.find("original-session-remains-active"),
        std::string::npos);
}

TEST(RunOutput, AnnouncementDoesNotTripConfiguredThrowLevel) {
    grace::test::TempDirectory temp;
    grace::output::RunOutputConfig config;
    config.baseDirectory = temp.path("results");
    config.timestamp = "strict-session";
    config.throwLevel = LogLevel::INFO;

    EXPECT_NO_THROW((void)grace::output::startRunOutput(config));
    EXPECT_THROW(
        Logger::instance().info("ordinary-info-still-throws"),
        std::runtime_error);
    grace::output::stopRunOutput();
}

TEST(RunOutput, DefaultSessionsReserveDistinctDirectories) {
    grace::test::TempDirectory temp;
    grace::output::RunOutputConfig config;
    config.baseDirectory = temp.path("results");

    const auto first = grace::output::startRunOutput(config);
    const auto second = grace::output::startRunOutput(config);

    EXPECT_NE(first, second);
    EXPECT_TRUE(std::filesystem::is_directory(first));
    EXPECT_TRUE(std::filesystem::is_directory(second));
    grace::output::stopRunOutput();
}
