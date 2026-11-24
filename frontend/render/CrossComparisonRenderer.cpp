// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "CrossComparisonRenderer.h"

#include "Simulation.h"
#include "env/ContinuousEnvironment.h"
#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "render/DebugDrawQueue.h"
#include "render/RendererSfml.h"

#include <SFML/Graphics/RenderTexture.hpp>
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <imgui.h>
#include <type_traits>
#if __has_include(<imgui-SFML.h>)
#include <imgui-SFML.h>
#elif __has_include(<ImGui-SFML.h>)
#include <ImGui-SFML.h>
#else
#error "ImGui-SFML header not found."
#endif

namespace frontend::render {
namespace {

b2HexColor pack(
    const std::uint8_t red,
    const std::uint8_t green,
    const std::uint8_t blue,
    const std::uint8_t alpha = 230) {
    return static_cast<b2HexColor>(
        (std::uint32_t(red) << 24U) |
        (std::uint32_t(green) << 16U) |
        (std::uint32_t(blue) << 8U) |
        std::uint32_t(alpha));
}

void drawPlannedPaths(
    const Simulation& simulation,
    DebugDrawQueue& queue,
    const float strokeWorld) {
    queue.beginOverlayFront();
    auto& debugDraw = queue.getB2DebugDraw();
    static const b2HexColor colors[] = {
        pack(226, 74, 51), pack(52, 138, 189), pack(152, 142, 213),
        pack(119, 119, 119), pack(251, 193, 94), pack(142, 186, 66),
        pack(255, 157, 167), pack(78, 205, 196),
    };

    const float radius = std::max(0.0001f, 0.5f * strokeWorld);
    std::size_t index = 0;
    for (const b2BodyId id : simulation.robots().robots()) {
        const auto* metadata = simulation.robots().meta(id);
        if (metadata && metadata->plan.waypoints.size() >= 2) {
            const b2HexColor color =
                colors[index % (sizeof(colors) / sizeof(colors[0]))];
            for (std::size_t waypoint = 1;
                 waypoint < metadata->plan.waypoints.size();
                 ++waypoint) {
                debugDraw.DrawSolidCapsuleFcn(
                    metadata->plan.waypoints[waypoint - 1].p,
                    metadata->plan.waypoints[waypoint].p,
                    radius,
                    color,
                    debugDraw.context);
            }
        }
        ++index;
    }
}

void drawRoadmap(
    const RoadmapEnvironment& environment,
    DebugDrawQueue& queue,
    const float nodeRadius) {
    queue.beginOverlayFront();
    auto& debugDraw = queue.getB2DebugDraw();
    const b2HexColor edgeColor = pack(90, 90, 90, 220);
    const b2HexColor nodeColor = pack(30, 144, 255, 220);

    for (int node = 0; node < environment.numNodes(); ++node) {
        const b2Vec2 from = environment.nodePos(node);
        for (const int adjacent : environment.adj(node)) {
            if (adjacent != node) {
                queue.drawLine(
                    from, environment.nodePos(adjacent), edgeColor);
            }
        }
    }
    for (int node = 0; node < environment.numNodes(); ++node) {
        debugDraw.DrawCircleFcn(
            environment.nodePos(node),
            nodeRadius,
            nodeColor,
            debugDraw.context);
    }
}

template <class Environment>
void drawBoundaryWalls(
    const Environment& environment,
    DebugDrawQueue& queue,
    const float thicknessHint) {
    const auto bounds = environment.boundsInMeter();
    const float diagonal = std::sqrt(std::max(
        1e-6f,
        bounds.width * bounds.width + bounds.height * bounds.height));
    const float thickness =
        thicknessHint > 0.0f ? thicknessHint : 0.012f * diagonal;
    const float radius = 0.5f * std::max(1e-4f, thickness);
    const b2HexColor color = pack(235, 235, 242, 220);

    queue.beginOverlayFront();
    auto& debugDraw = queue.getB2DebugDraw();
    debugDraw.DrawSolidCapsuleFcn(
        {0.0f, 0.0f}, {bounds.width, 0.0f}, radius, color, debugDraw.context);
    debugDraw.DrawSolidCapsuleFcn(
        {0.0f, bounds.height},
        {bounds.width, bounds.height},
        radius,
        color,
        debugDraw.context);
    debugDraw.DrawSolidCapsuleFcn(
        {0.0f, 0.0f}, {0.0f, bounds.height}, radius, color, debugDraw.context);
    debugDraw.DrawSolidCapsuleFcn(
        {bounds.width, 0.0f},
        {bounds.width, bounds.height},
        radius,
        color,
        debugDraw.context);
}

sf::View makeFitView(
    float worldWidth,
    float worldHeight,
    const int pixelWidth,
    const int pixelHeight,
    const float marginFraction) {
    worldWidth = std::max(1e-6f, worldWidth);
    worldHeight = std::max(1e-6f, worldHeight);
    const float targetAspect =
        pixelWidth > 0 && pixelHeight > 0
        ? static_cast<float>(pixelWidth) / static_cast<float>(pixelHeight)
        : worldWidth / worldHeight;
    float width = worldWidth * (1.0f + marginFraction);
    float height = worldHeight * (1.0f + marginFraction);
    if (width / height > targetAspect) {
        height = width / targetAspect;
    } else {
        width = height * targetAspect;
    }
    sf::View view(sf::FloatRect({0.0f, 0.0f}, {width, height}));
    view.setCenter({0.5f * worldWidth, 0.5f * worldHeight});
    return view;
}

template <class Environment>
bool renderPreview(
    Environment& environment,
    const Simulation* simulation,
    sf::RenderTexture& target,
    const PreviewOptions& options,
    const bool isRoadmap) {
    if (options.widthPixels <= 0 || options.heightPixels <= 0) {
        return false;
    }

    const sf::Vector2u requested{
        static_cast<unsigned>(options.widthPixels),
        static_cast<unsigned>(options.heightPixels)};
    if (target.getTexture().getSize() != requested && !target.resize(requested)) {
        return false;
    }

    DebugDrawQueue queue;
    queue.beginWorldPass();
    auto& debugDraw = queue.getB2DebugDraw();
    b2World_Draw(environment.getB2World(), &debugDraw);

    const auto bounds = environment.boundsInMeter();
    const float diagonal = std::sqrt(std::max(
        1e-6f,
        bounds.width * bounds.width + bounds.height * bounds.height));
    if (options.drawWalls) {
        drawBoundaryWalls(environment, queue, options.wallThickness);
    }
    if (options.drawPaths && simulation) {
        drawPlannedPaths(*simulation, queue, 0.003f * diagonal);
    }
    if constexpr (std::is_same_v<Environment, RoadmapEnvironment>) {
        if (isRoadmap && options.drawRoadmap) {
            drawRoadmap(environment, queue, 0.006f * diagonal);
        }
    }

    target.setView(makeFitView(
        bounds.width,
        bounds.height,
        options.widthPixels,
        options.heightPixels,
        options.marginFraction));
    target.clear(sf::Color(0, 0, 0, 0));
    RendererSfml::draw(queue, target);
    target.display();
    return true;
}

void drawPreview(
    const sf::RenderTexture& target,
    const float width,
    const float height) {
    ImGui::Image(target.getTexture(), ImVec2(width, height));
}

}  // namespace

struct CrossComparisonRenderer::Impl {
    sf::RenderTexture continuous;
    sf::RenderTexture roadmap;
    sf::RenderTexture grid;
};

CrossComparisonRenderer::CrossComparisonRenderer()
    : impl_(std::make_unique<Impl>()) {}

CrossComparisonRenderer::~CrossComparisonRenderer() = default;

bool CrossComparisonRenderer::renderContinuousPreview(
    ContinuousEnvironment& environment,
    const Simulation* simulation,
    const PreviewOptions& options) {
    return renderPreview(
        environment, simulation, impl_->continuous, options, false);
}

bool CrossComparisonRenderer::renderRoadmapPreview(
    RoadmapEnvironment& environment,
    const Simulation* simulation,
    const PreviewOptions& options) {
    return renderPreview(
        environment, simulation, impl_->roadmap, options, true);
}

bool CrossComparisonRenderer::renderGridPreview(
    GridEnvironment& environment,
    const Simulation* simulation,
    const PreviewOptions& options) {
    return renderPreview(environment, simulation, impl_->grid, options, false);
}

void CrossComparisonRenderer::drawContinuousPreview(
    const float width,
    const float height) const {
    drawPreview(impl_->continuous, width, height);
}

void CrossComparisonRenderer::drawRoadmapPreview(
    const float width,
    const float height) const {
    drawPreview(impl_->roadmap, width, height);
}

void CrossComparisonRenderer::drawGridPreview(
    const float width,
    const float height) const {
    drawPreview(impl_->grid, width, height);
}

}  // namespace frontend::render
