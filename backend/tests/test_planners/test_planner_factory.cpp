// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "env/GridEnvironment.h"
#include "planner/plugin/PlannerFactory.h"
#include "planner/plugin/PluginLoader.h"
#include "planner/plugin/PluginOwnedPlanner.h"
#include "planner/plugin/PlannerRegistry.h"

#include <cstdlib>
#include <filesystem>
#include <limits>
#include <set>
#include <string>

namespace {

class ScopedPluginDirectory {
public:
    explicit ScopedPluginDirectory(const std::string& value) {
        if (const char* current = std::getenv("GRACE_PLUGIN_DIR")) {
            previous_ = current;
            hadPrevious_ = true;
        }
        set(value);
    }

    ~ScopedPluginDirectory() {
        if (hadPrevious_) {
            set(previous_);
        } else {
#if defined(_WIN32)
            _putenv_s("GRACE_PLUGIN_DIR", "");
#else
            unsetenv("GRACE_PLUGIN_DIR");
#endif
        }
    }

private:
    static void set(const std::string& value) {
#if defined(_WIN32)
        _putenv_s("GRACE_PLUGIN_DIR", value.c_str());
#else
        setenv("GRACE_PLUGIN_DIR", value.c_str(), 1);
#endif
    }

    std::string previous_;
    bool hadPrevious_{false};
};

}  // namespace

TEST(PluginPathResolution, ExplicitDirectoryOverridesEnvironment) {
    ScopedPluginDirectory environment("/environment/plugins");
    PlannerFactoryConfig config;
    config.plugin_dir = "/explicit/plugins";

    EXPECT_EQ(
        resolvePluginDirectory(config),
        std::filesystem::path("/explicit/plugins"));
}

TEST(PluginPathResolution, EnvironmentOverridesBuildDefault) {
    ScopedPluginDirectory environment("/environment/plugins");
    EXPECT_EQ(
        resolvePluginDirectory(),
        std::filesystem::path("/environment/plugins"));
}

TEST(PluginPathResolution, EmptyEnvironmentUsesBuildDefault) {
    ScopedPluginDirectory environment("");
    EXPECT_EQ(
        resolvePluginDirectory(),
        std::filesystem::path(BUILD_PATH));
}

TEST(PluginPathResolution, UsesPlatformLibraryName) {
    const auto path = pluginLibraryPath("/plugins", "planner_fixture");
#if defined(_WIN32)
    EXPECT_EQ(path.filename(), "planner_fixture.dll");
#elif defined(__APPLE__)
    EXPECT_EQ(path.filename(), "libplanner_fixture.dylib");
#else
    EXPECT_EQ(path.filename(), "libplanner_fixture.so");
#endif
}

TEST(PluginLoader, ReportsMissingLibrary) {
    std::string error;
    const auto plugin = PluginHandle::load(
        "/path/that/does/not/exist/grace-plugin", &error);

    EXPECT_EQ(plugin, nullptr);
    EXPECT_FALSE(error.empty());
}

TEST(PluginLoader, ExposesMetadataAndRetainsLibraryForPlannerLifetime) {
    std::string error;
    auto plugin = PluginHandle::load(
        pluginLibraryPath(TEST_PLUGIN_DIR, "planner_eecbs_src").string(),
        &error);

    ASSERT_NE(plugin, nullptr) << error;
    EXPECT_EQ(plugin->plannerId(), "eecbs-src");
    EXPECT_EQ(plugin->environment(), GRACE_PLUGIN_ENVIRONMENT_GRID);

    auto planner = plugin->createPlanner();
    ASSERT_NE(planner, nullptr);
    plugin.reset();

    EXPECT_STREQ(planner->name(), "fixture-plugin");
    EXPECT_EQ(planner->type(), EnvironmentType::GridEnvironment);
}

TEST(PluginLoader, RejectsMissingMetadata) {
    std::string error;
    const auto plugin = PluginHandle::load(
        pluginLibraryPath(
            TEST_PLUGIN_DIR, "planner_missing_metadata").string(),
        &error);

    EXPECT_EQ(plugin, nullptr);
    EXPECT_NE(error.find("Missing required plugin metadata"), std::string::npos);
}

TEST(PluginLoader, RejectsInvalidEnvironmentValue) {
    std::string error;
    const auto plugin = PluginHandle::load(
        pluginLibraryPath(
            TEST_PLUGIN_DIR, "planner_psippctc_src").string(),
        &error);

    EXPECT_EQ(plugin, nullptr);
    EXPECT_NE(error.find("invalid environment"), std::string::npos);
}

TEST(PluginLoader, RejectsEmptyPlannerId) {
    std::string error;
    const auto plugin = PluginHandle::load(
        pluginLibraryPath(TEST_PLUGIN_DIR, "planner_dbcbs_src").string(),
        &error);

    EXPECT_EQ(plugin, nullptr);
    EXPECT_NE(error.find("empty planner ID"), std::string::npos);
}

TEST(PluginLoader, TranslatesMetadataExceptions) {
    std::string error;
    const auto plugin = PluginHandle::load(
        pluginLibraryPath(
            TEST_PLUGIN_DIR, "planner_throwing_metadata").string(),
        &error);

    EXPECT_EQ(plugin, nullptr);
    EXPECT_NE(error.find("grace_plugin_id threw"), std::string::npos);
}

TEST(PluginLoader, TranslatesPlannerConstructionExceptions) {
    std::string error;
    const auto plugin = PluginHandle::load(
        pluginLibraryPath(
            TEST_PLUGIN_DIR, "planner_throwing_create").string(),
        &error);

    ASSERT_NE(plugin, nullptr) << error;
    EXPECT_THROW(plugin->createPlanner(), std::runtime_error);
}

TEST(PluginLoader, ContainsPlannerDestructionExceptions) {
    std::string error;
    const auto plugin = PluginHandle::load(
        pluginLibraryPath(
            TEST_PLUGIN_DIR, "planner_throwing_destroy").string(),
        &error);

    ASSERT_NE(plugin, nullptr) << error;
    auto planner = plugin->createPlanner();
    ASSERT_NE(planner, nullptr);
    planner.reset();
}

TEST(PluginOwnedPlanner, TranslatesPlanExceptionBeforeLibraryUnload) {
    std::string message;
    try {
        std::string error;
        auto plugin = PluginHandle::load(
            pluginLibraryPath(
                TEST_PLUGIN_DIR, "planner_throwing_operations").string(),
            &error);
        ASSERT_NE(plugin, nullptr) << error;

        auto planner = std::make_unique<PluginOwnedPlanner>(
            plugin->createPlanner());
        plugin.reset();

        GridEnvironment environment;
        const std::vector<b2BodyId> ids;
        const std::vector<StartGoal> startsAndGoals;
        planner->plan(PlanRequest{
            environment,
            ids,
            startsAndGoals,
            PlannerSettings{},
            std::numeric_limits<double>::infinity(),
            std::uint64_t{0}});
    } catch (const std::runtime_error& exception) {
        message = exception.what();
    }

    EXPECT_NE(message.find("Plugin plan threw"), std::string::npos);
}

TEST(PluginOwnedPlanner, TranslatesConfigureException) {
    std::string error;
    auto plugin = PluginHandle::load(
        pluginLibraryPath(
            TEST_PLUGIN_DIR, "planner_throwing_operations").string(),
        &error);
    ASSERT_NE(plugin, nullptr) << error;

    PluginOwnedPlanner planner(plugin->createPlanner());
    EXPECT_THROW(
        planner.configure(PlannerSettings{}),
        std::runtime_error);
}

TEST(PlannerFactory, RejectsUnknownPlanner) {
    EXPECT_THROW(makePlanner("not-a-planner"), std::runtime_error);
}

TEST(PlannerRegistry, HasUniqueIdsAndOneDefaultPerEnvironment) {
    std::set<std::string_view> ids;
    std::set<EnvironmentType> defaults;

    for (const auto& planner : grace::planner::registeredPlanners()) {
        EXPECT_TRUE(ids.insert(planner.id).second) << planner.id;
        EXPECT_EQ(grace::planner::findPlanner(planner.id), &planner);
        if (planner.isDefault) {
            EXPECT_TRUE(defaults.insert(planner.environment).second);
        }
        if (planner.implementation ==
            grace::planner::PlannerImplementation::Plugin) {
            EXPECT_FALSE(planner.pluginLibrary.empty());
            EXPECT_EQ(planner.createBuiltIn, nullptr);
        } else {
            EXPECT_TRUE(planner.pluginLibrary.empty());
            EXPECT_EQ(
                planner.createBuiltIn != nullptr,
                planner.isBundled);
        }
    }

    EXPECT_EQ(defaults.size(), 3U);
    EXPECT_EQ(
        grace::planner::defaultPlanner(EnvironmentType::GridEnvironment).id,
        "lacam0-src");
    EXPECT_EQ(
        grace::planner::defaultPlanner(EnvironmentType::RoadmapEnvironment).id,
        "psippctc-src");
    EXPECT_EQ(
        grace::planner::defaultPlanner(
            EnvironmentType::ContinuousEnvironment).id,
        "dbcbs-src");
}

TEST(PlannerFactory, LoadsPluginFromExplicitDirectory) {
    PlannerFactoryConfig config;
    config.plugin_dir = TEST_PLUGIN_DIR;

    auto planner = makePlanner("eecbs-src", config);

    ASSERT_NE(planner, nullptr);
    EXPECT_STREQ(planner->name(), "fixture-plugin");
    EXPECT_EQ(planner->type(), EnvironmentType::GridEnvironment);
}

TEST(PlannerFactory, RejectsPluginWithMismatchedIdentity) {
    PlannerFactoryConfig config;
    config.plugin_dir = TEST_PLUGIN_DIR;

    EXPECT_THROW(
        makePlanner("lacam0-src", config),
        std::runtime_error);
}

TEST(PlannerFactory, RejectsPluginWithMismatchedAdvertisedEnvironment) {
    PlannerFactoryConfig config;
    config.plugin_dir = TEST_PLUGIN_DIR;

    EXPECT_THROW(
        makePlanner("mapflns2-src", config),
        std::runtime_error);
}

TEST(PlannerFactory, RejectsPluginWithMismatchedImplementationEnvironment) {
    PlannerFactoryConfig config;
    config.plugin_dir = TEST_PLUGIN_DIR;

    EXPECT_THROW(
        makePlanner("cbsh2rtc-src", config),
        std::runtime_error);
}

TEST(PlannerFactory, RejectsPluginWithLegacyAbi) {
    PlannerFactoryConfig config;
    config.plugin_dir = TEST_PLUGIN_DIR;

    EXPECT_THROW(
        makePlanner("ccbs-src", config),
        std::runtime_error);
}

TEST(PlannerFactory, RejectsPluginReturningNullPlanner) {
    PlannerFactoryConfig config;
    config.plugin_dir = TEST_PLUGIN_DIR;

    EXPECT_THROW(
        makePlanner("sst-src", config),
        std::runtime_error);
}
