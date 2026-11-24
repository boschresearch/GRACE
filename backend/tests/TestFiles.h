// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string_view>

namespace grace::test {

class TempDirectory {
public:
    TempDirectory() {
        std::random_device random;
        const auto root = std::filesystem::temp_directory_path();

        for (int attempt = 0; attempt < 10; ++attempt) {
            path_ = root / ("grace-test-" + std::to_string(random()));
            if (std::filesystem::create_directory(path_)) {
                return;
            }
        }

        throw std::runtime_error("Unable to create a temporary test directory");
    }

    ~TempDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    TempDirectory(const TempDirectory&) = delete;
    TempDirectory& operator=(const TempDirectory&) = delete;

    [[nodiscard]] std::filesystem::path write(
        std::string_view name,
        std::string_view contents) const {
        const auto file = path_ / name;
        std::ofstream output(file);
        if (!output || !(output << contents)) {
            throw std::runtime_error("Unable to write test fixture: " + file.string());
        }
        return file;
    }

    [[nodiscard]] std::filesystem::path path(std::string_view name) const {
        return path_ / name;
    }

private:
    std::filesystem::path path_;
};

}  // namespace grace::test
