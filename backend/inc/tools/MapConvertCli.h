// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <filesystem>
#include <iosfwd>

namespace tools::mc {

int runMapConvertCli(
    int argc,
    char* argv[],
    const std::filesystem::path& dataRoot,
    std::ostream& output,
    std::ostream& error);

}  // namespace tools::mc
