// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/plugin/PlannerFactory.h"
#include "../inc/planner/plugin/PluginLoader.h"
#include "../inc/planner/plugin/PluginOwnedPlanner.h"
#include "../inc/planner/plugin/PlannerRegistry.h"
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string_view>

namespace fs = std::filesystem;

namespace {

GracePluginEnvironment pluginEnvironment(
    const EnvironmentType environment) {
    switch (environment) {
        case EnvironmentType::GridEnvironment:
            return GRACE_PLUGIN_ENVIRONMENT_GRID;
        case EnvironmentType::ContinuousEnvironment:
            return GRACE_PLUGIN_ENVIRONMENT_CONTINUOUS;
        case EnvironmentType::RoadmapEnvironment:
            return GRACE_PLUGIN_ENVIRONMENT_ROADMAP;
        case EnvironmentType::UnknownEnvironment:
            break;
    }
    throw std::invalid_argument(
        "Cannot load a planner plugin for an unknown environment");
}

}  // namespace

fs::path resolvePluginDirectory(const PlannerFactoryConfig& cfg) {
    if (!cfg.plugin_dir.empty()) {
        return cfg.plugin_dir;
    }
    if (const char* environment = std::getenv("GRACE_PLUGIN_DIR");
        environment && *environment != '\0') {
        return environment;
    }
    return BUILD_PATH;
}

fs::path pluginLibraryPath(
    const fs::path& directory,
    const std::string_view baseName) {
#if defined(_WIN32)
    return directory / (std::string(baseName) + ".dll");
#elif defined(__APPLE__)
    return directory / ("lib" + std::string(baseName) + ".dylib");
#else
    return directory / ("lib" + std::string(baseName) + ".so");
#endif
}

std::unique_ptr<IPlanner> makePlanner(const std::string& which,
                                      const PlannerFactoryConfig& cfg) {
    const auto* descriptor = grace::planner::findPlanner(which);
    if (!descriptor) {
        throw std::runtime_error("Unknown planner: " + which);
    }

    if (descriptor->createBuiltIn) {
        return descriptor->createBuiltIn();
    }

    if (descriptor->implementation ==
        grace::planner::PlannerImplementation::Plugin) {
        std::string err;
        auto handle = PluginHandle::load(
            pluginLibraryPath(
                resolvePluginDirectory(cfg),
                descriptor->pluginLibrary).string(),
            &err);
        if (!handle) throw std::runtime_error("Failed to load plugin " + which + ": " + err);
        if (handle->plannerId() != descriptor->id) {
            throw std::runtime_error(
                "Plugin identity mismatch for " + which +
                ": library advertises " + handle->plannerId());
        }
        if (handle->environment() !=
            pluginEnvironment(descriptor->environment)) {
            throw std::runtime_error(
                "Plugin environment capability mismatch for " + which);
        }

        auto implementation = handle->createPlanner();
        if (!implementation) {
            throw std::runtime_error(
                "Plugin " + which +
                " returned null from create_planner()");
        }
        if (implementation->type() != descriptor->environment) {
            throw std::runtime_error(
                "Plugin implementation capability mismatch for " + which);
        }
        return std::make_unique<PluginOwnedPlanner>(
            std::move(implementation));
    }
    throw std::runtime_error(
        "Planner is unavailable in this build: " + which);
}
