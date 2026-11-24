// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <chrono>
#include <fmt/chrono.h>

inline std::chrono::time_point<std::chrono::system_clock, std::chrono::seconds>
get_current_time_second() {
    return std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
}