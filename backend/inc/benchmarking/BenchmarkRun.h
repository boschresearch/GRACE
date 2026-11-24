// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "benchmarking/BenchmarkConfig.h"

#include <memory>

namespace grace::benchmarking {

class ResourceMonitor {
public:
    ResourceMonitor();
    ~ResourceMonitor();
    ResourceMonitor(const ResourceMonitor&) = delete;
    ResourceMonitor& operator=(const ResourceMonitor&) = delete;
    ResourceMonitor(ResourceMonitor&&) = delete;
    ResourceMonitor& operator=(ResourceMonitor&&) = delete;

    void start();
    void stop();
    [[nodiscard]] double peakCpuPercent() const noexcept;
    [[nodiscard]] double peakMemoryGb() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

[[nodiscard]] bool runBenchmark(const RunOpts& options);

} // namespace grace::benchmarking
