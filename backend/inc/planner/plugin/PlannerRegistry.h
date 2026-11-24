// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "../Planner.h"

#include <memory>
#include <span>
#include <string_view>

namespace grace::planner {

enum class PlannerImplementation {
    Binary,
    Plugin,
};

using BuiltInPlannerFactory = std::unique_ptr<IPlanner> (*)();

struct PlannerDescriptor {
    std::string_view id;
    EnvironmentType environment;
    PlannerImplementation implementation;
    std::string_view pluginLibrary;
    bool isDefault;
    bool isBundled;
    BuiltInPlannerFactory createBuiltIn;
};

[[nodiscard]] std::span<const PlannerDescriptor> registeredPlanners() noexcept;
[[nodiscard]] const PlannerDescriptor* findPlanner(
    std::string_view id) noexcept;
[[nodiscard]] const PlannerDescriptor& defaultPlanner(
    EnvironmentType environment);

}  // namespace grace::planner
