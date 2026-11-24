// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <box2d/box2d.h>
#include <cstdint>
#include <functional>

struct BodyIdHash {
    size_t operator()(const b2BodyId id) const noexcept {
        // Combine index and revision; any stable mix is fine
        const uint64_t v = (uint64_t{static_cast<unsigned long long>(id.index1)} << 32) ^ uint64_t{id.generation};
        return std::hash<uint64_t>{}(v);
    }
};
struct BodyIdEq {
    bool operator()(const b2BodyId a, const b2BodyId b) const noexcept {
        return a.index1 == b.index1 && a.generation == b.generation;
    }
};