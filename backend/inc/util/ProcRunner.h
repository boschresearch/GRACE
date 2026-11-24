// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include <chrono>

namespace util {
    namespace fs = std::filesystem;

    struct ProcessResult {
        int exit_code = -1;
        bool timed_out = false;
        std::chrono::milliseconds wall{0};
    };

    ProcessResult runWithTimeout(const std::string& exe,
                                 const std::vector<std::string>& args,
                                 const fs::path& start_dir,
                                 const fs::path& stdout_file,
                                 const fs::path& stderr_file,
                                 std::chrono::milliseconds timeout_ms);
} // namespace util