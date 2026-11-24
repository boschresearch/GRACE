// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Logger.h"

#include <filesystem>
#include <string>

namespace grace::output {

struct RunOutputConfig {
    std::filesystem::path baseDirectory{"results"};
    std::string timestamp;
    LogLevel logLevel{LogLevel::INFO};
    LogLevel throwLevel{LogLevel::ERROR};
    bool overwriteLog{true};
};

[[nodiscard]] std::filesystem::path startRunOutput(
    const RunOutputConfig& config = {});
[[nodiscard]] std::filesystem::path currentRunDirectory();
void stopRunOutput();

}  // namespace grace::output
