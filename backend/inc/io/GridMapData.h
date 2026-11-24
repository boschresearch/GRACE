// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <vector>

namespace grace::io {

struct GridMapData {
    int width = 0;
    int height = 0;
    std::vector<bool> occupancy;
};

}  // namespace grace::io
