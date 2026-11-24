// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "PlannerCatalog.h"

#include "planner/plugin/PlannerRegistry.h"

namespace platform {
namespace {

EnvironmentType environmentType(const EnvKind kind) {
    switch (kind) {
        case EnvKind::Grid:
            return EnvironmentType::GridEnvironment;
        case EnvKind::Roadmap:
            return EnvironmentType::RoadmapEnvironment;
        case EnvKind::Continuous:
            return EnvironmentType::ContinuousEnvironment;
    }
    return EnvironmentType::UnknownEnvironment;
}

std::vector<std::string> plannersFor(const EnvironmentType environment) {
    std::vector<std::string> result;
    for (const auto& planner : grace::planner::registeredPlanners()) {
        if (planner.environment == environment) {
            result.emplace_back(planner.id);
        }
    }
    return result;
}

}  // namespace

const std::vector<std::string>& PlannerCatalog::allowed(const EnvKind kind) {
    static const auto grid =
        plannersFor(EnvironmentType::GridEnvironment);
    static const auto roadmap =
        plannersFor(EnvironmentType::RoadmapEnvironment);
    static const auto continuous =
        plannersFor(EnvironmentType::ContinuousEnvironment);

    switch (kind) {
        case EnvKind::Grid:
            return grid;
        case EnvKind::Roadmap:
            return roadmap;
        case EnvKind::Continuous:
            return continuous;
    }
    return grid;
}

const char* PlannerCatalog::defaultFor(const EnvKind kind) {
    return grace::planner::defaultPlanner(environmentType(kind)).id.data();
}

bool PlannerCatalog::isAllowed(
    const EnvKind kind,
    const std::string& planner) {
    const auto* descriptor = grace::planner::findPlanner(planner);
    return descriptor &&
           descriptor->environment == environmentType(kind);
}

}  // namespace platform
