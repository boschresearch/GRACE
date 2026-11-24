// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// platform/PlannerCatalog.h
#pragma once
#include <string>
#include <vector>

namespace platform {

    enum class EnvKind { Grid, Roadmap, Continuous };

    struct PlannerCatalog {
        // Allowed list for the given environment kind
        static const std::vector<std::string>& allowed(EnvKind k);

        // Default (fallback) planner name for the environment kind
        static const char* defaultFor(EnvKind k);

        // Convenience: is name valid for environment kind?
        static bool isAllowed(EnvKind k, const std::string& planner);
    };

} // namespace platform
