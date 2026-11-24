// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/util/ResultsDir.h"
#include "../inc/util/Timer.h"   // assumes you already have get_current_time_second()
#include <fmt/format.h>
#include <filesystem>
#include <system_error>
#include <fstream>

namespace fs = std::filesystem;

namespace results {

    std::string makeTimestamp() {
        // Reuse your chrono helper so fmt can do {:%Y%m%d_%H%M%S}
        return fmt::format("{:%Y%m%d_%H%M%S}", get_current_time_second());
    }

    std::filesystem::path makeRunDir(const std::filesystem::path& base,
                                     const std::string& timestamp) {
        const std::string ts = timestamp.empty() ? makeTimestamp() : timestamp;
        fs::path dir = base / ts;
        std::error_code ec;
        fs::create_directories(dir, ec);
        fs::path canon = fs::weakly_canonical(dir, ec);
        return canon.empty() ? dir : canon;
    }

} // namespace results
