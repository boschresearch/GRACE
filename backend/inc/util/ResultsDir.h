// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>

namespace results {

    // Format current time like "20250901_123456"
    std::string makeTimestamp();

    // Ensure results/<timestamp>/ exists and return the absolute path.
    std::filesystem::path makeRunDir(const std::filesystem::path& base = "results",
                                     const std::string& timestamp = "");

} // namespace results
