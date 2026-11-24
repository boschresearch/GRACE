// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "../Planner.h"
#include <filesystem>
#include <memory>
#include <string>

struct PlannerFactoryConfig {
    // Directory containing src plugins (overrides build-time path if set)
    std::string plugin_dir; // e.g., from env GRACE_PLUGIN_DIR
};

[[nodiscard]] std::filesystem::path resolvePluginDirectory(
    const PlannerFactoryConfig& cfg = {});
[[nodiscard]] std::filesystem::path pluginLibraryPath(
    const std::filesystem::path& directory,
    std::string_view baseName);

std::unique_ptr<IPlanner> makePlanner(const std::string& which,
                                      const PlannerFactoryConfig& cfg = {});