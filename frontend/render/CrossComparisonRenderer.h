// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <memory>

class ContinuousEnvironment;
class GridEnvironment;
class RoadmapEnvironment;
class Simulation;

namespace frontend::render {

struct PreviewOptions {
    int widthPixels{0};
    int heightPixels{0};
    bool drawPaths{true};
    bool drawRoadmap{false};
    bool drawWalls{true};
    float wallThickness{0.0f};
    float marginFraction{0.06f};
};

class CrossComparisonRenderer {
public:
    CrossComparisonRenderer();
    ~CrossComparisonRenderer();

    CrossComparisonRenderer(const CrossComparisonRenderer&) = delete;
    CrossComparisonRenderer& operator=(const CrossComparisonRenderer&) = delete;

    [[nodiscard]] bool renderContinuousPreview(
        ContinuousEnvironment& environment,
        const Simulation* simulation,
        const PreviewOptions& options);
    [[nodiscard]] bool renderRoadmapPreview(
        RoadmapEnvironment& environment,
        const Simulation* simulation,
        const PreviewOptions& options);
    [[nodiscard]] bool renderGridPreview(
        GridEnvironment& environment,
        const Simulation* simulation,
        const PreviewOptions& options);

    void drawContinuousPreview(float width, float height) const;
    void drawRoadmapPreview(float width, float height) const;
    void drawGridPreview(float width, float height) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace frontend::render
