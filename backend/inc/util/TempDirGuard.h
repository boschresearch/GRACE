// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <chrono>
#include <system_error>

namespace util {
    namespace fs = std::filesystem;

    class TempDirGuard {
        fs::path dir_;
        bool keep_{false};

    public:
        TempDirGuard(const fs::path& base, const std::string& prefix, bool keep)
            : keep_(keep)
        {
            const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
            dir_ = base / (prefix + std::to_string(tick));
            std::error_code ec;
            fs::create_directories(dir_, ec);
        }

        ~TempDirGuard() {
            if (!keep_) {
                std::error_code ec;
                fs::remove_all(dir_, ec);
            }
        }

        const fs::path& path() const noexcept { return dir_; }
        void set_keep(bool k) noexcept { keep_ = k; }
    };
} // namespace util