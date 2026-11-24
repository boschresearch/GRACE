// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "PlatformBase.h"

// platform & env
#include "Simulation.h"
#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "env/ContinuousEnvironment.h"
#include "io/ContinuousYaml.h"
#include "../sample/SampleContext.h"

// debug draw
#include "../render/DebugDrawQueue.h"
#include "../ui/ViewController.h"
#include "../render/DebugConfig.h"
#include "../ui/panels/PlatformPanels.h"

// defaults
#include "PlatformDefaults.h"
#include "PlannerCatalog.h"
#include "ResetWorkflow.h"

// imgui
#include <imgui.h>

// std
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <fmt/format.h>


//------------------------------------------------------------------------------
// ctor / dtor
//------------------------------------------------------------------------------
PlatformBase::PlatformBase(SampleContext* ctx)
: Sample(ctx)
{
    // Base Sample created a temporary Box2D world; we will adopt platform's world, so free it now.
    if (B2_IS_NON_NULL(m_worldId)) {
        b2DestroyWorld(m_worldId);
        m_worldId = b2_nullWorldId;
    }

    // Studio defaults (grid-first)
    m_studio.mapPath    = platform::defaults::mapMovingAIDemo();
    m_studio.scenPath   = platform::defaults::scenMovingAIDemo();
    m_studio.agentsPath = platform::defaults::agentsWarehouseLarge();
    m_studio.tasksPath  = platform::defaults::tasksWarehouseLarge();
    // roadmap
    m_studio.roadmapPath = platform::defaults::mapRoadmapDemo();
    m_studio.taskXmlPath = platform::defaults::taskRoadmapDemo();
    // continuous
    m_studio.problemYamlPath = platform::defaults::problemYamlDemo();

    // Start paused if the Studio is the entrypoint.
    m_context->pause = true;
}

PlatformBase::~PlatformBase()
{
    // Do not let base destructor destroy the platform-owned world.
    m_worldId = b2_nullWorldId;
}

//------------------------------------------------------------------------------
// step / input
//------------------------------------------------------------------------------
void PlatformBase::Step()
{
    auto& playback = m_playback;
    auto& overlays = m_overlays;

    // Compute dt honoring pause/single-step
    float dt_phys = 0.0f;
    if (!m_context->pause) {
        dt_phys = (m_context->hertz > 0.0f) ? 1.0f / m_context->hertz : 0.0f;
    } else if (m_context->singleStep) {
        m_context->singleStep = false;
        dt_phys = (m_context->hertz > 0.0f) ? 1.0f / m_context->hertz : 0.0f;
    }

    if (m_sim) m_sim->setSubStepCount(m_context->subStepCount);

    // Advance platform sim one frame
    if (m_sim && dt_phys > 0.0f) {
        m_sim->stepOnce(dt_phys);
        ++m_stepCount;
        recordExecutedOnce();
        syncWorldBinding(); // keep renderer bound if env rebuilt internals
    }

    // Ghost playback (ticks only while physics is stepping)
    if (playback.playGhost && dt_phys > 0.0f) {
        const double T = std::max(1e-6, double(m_sim ? m_sim->maxPlannedTime() : 0.0f));
        if (T > 0.0) {
            if (playback.syncToSimulation && m_sim) {
                playback.visualizationTime =
                    std::clamp<double>(m_sim->currentSimTime(), 0.0, T);
            } else {
                playback.visualizationTime += double(playback.speed) * double(dt_phys);
                if (playback.loop) {
                    // wrap into [0, T)
                    playback.visualizationTime =
                        std::fmod(std::fmod(playback.visualizationTime, T) + T, T);
                } else {
                    // clamp into [0, T]
                    if (playback.visualizationTime >= T) {
                        playback.visualizationTime = T;
                    } else if (playback.visualizationTime < 0.0) {
                        playback.visualizationTime = 0.0;
                    }
                }
            }
        }
    }

    // Debug draw settings (world pass)
    render::ApplyDebugConfig(*m_context, *m_context->debugDraw, *m_context->view);

    // Theme: recolor SOLID polygons & capsules during world pass
    if (overlays.themeObstaclesLight) {
        m_context->debugDraw->setSolidPolyColorOverride(sf::Color(127,191,127,220));
        m_context->debugDraw->setCapsuleColorOverride  (sf::Color(127,191,127,220));
    } else {
        m_context->debugDraw->setSolidPolyColorOverride(std::nullopt);
        m_context->debugDraw->setCapsuleColorOverride  (std::nullopt);
    }

    // --------- DRAW WORLD ---------
    if (B2_IS_NON_NULL(m_worldId)) {
        auto& dd = m_context->debugDraw->getB2DebugDraw();
        m_context->debugDraw->beginWorldPass();
        b2World_Draw(m_worldId, &dd);
        m_context->debugDraw->endWorldPass();
    }

    // --------- BACK OVERLAYS (under everything) ---------
    m_context->debugDraw->beginOverlayBack();
    if (m_gridVisualization.showReservations) drawGridReservations();
    if (m_continuousVisualization.showClearance && isContinuous()) {
        drawContinuousClearance();
    }

    // --------- FRONT OVERLAYS (above everything) ---------
    m_context->debugDraw->beginOverlayFront();
    drawRoadmap();
    drawPlannedPaths();
    drawExecutedTraces();
    drawStartGoalDashes();
    drawSelectionBadges();
    drawGhostFleet();
    drawHoverCrosshair();

    // --- Unified top-center HUD (MiniHUD + Hover + Details) ---
    if (m_context->showUI) {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        // Hide this compact HUD when the Running HUD is visible.
        const bool runningHUDVisible =
            m_studio.autoHideDetailsWhenRunning && m_studio.running && !m_context->pause;
        if (!runningHUDVisible)
        {
            ImGui::SetNextWindowPos(
                ImVec2(vp->WorkPos.x + 0.5f * vp->WorkSize.x,
                       vp->WorkPos.y + m_context->overlaySafeTopPx),
                ImGuiCond_Always, ImVec2(0.5f, 0.0f));
            ImGui::SetNextWindowBgAlpha(0.20f);
            ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar
                                   | ImGuiWindowFlags_NoInputs
                                   | ImGuiWindowFlags_AlwaysAutoResize
                                   | ImGuiWindowFlags_NoSavedSettings;
            if (ImGui::Begin("Overlay.HUD", nullptr, flags)) {
                // line 1: mini status
                const auto [cx, cy] = m_context->view->getCenter();
                ImGui::Text("sim=%d  cam=(%.2f, %.2f, z=%.3f)  t_vis=%.2f  %s",
                            m_stepCount, cx, cy, m_context->view->getZoom(),
                            playback.visualizationTime,
                            m_instanceInfo.c_str());
                // line 2: hover (optional)
                if (const std::string hover = composeHoverLine(); !hover.empty()) {
                    ImGui::TextUnformatted(hover.c_str());
                }
                // line 3: details
                const std::string details = composeDetailsLine();
                ImGui::TextUnformatted(details.c_str());
            }
            ImGui::End();
        }
    }

    // Panels & workflow
    drawPanels();
}

void PlatformBase::MouseDown(b2Vec2 p, int button, int /*mod*/)
{
    if (!m_sim || button != 0) return;

    // Editors first
    if (m_editor.mode != EditMode::None) {
        handleEditorClick(p);
        return;
    }

    // Interaction: Shift = assign goal; Ctrl = spawn; else pick robot
    if (ImGui::GetIO().KeyShift) { assignGoalOnly(p); return; }
    if (ImGui::GetIO().KeyCtrl)  { spawnAtClick(p);   return; }

    // Toggle selection unless Alt is held
    const bool toggle = !ImGui::GetIO().KeyAlt;
    float bestD = std::numeric_limits<float>::infinity();
    b2BodyId best = b2_nullBodyId;
    for (b2BodyId id : m_sim->robots().robots()) {
        float d = b2Distance(b2Body_GetPosition(id), p);
        if (d < bestD && d < pickRadius()) { bestD = d; best = id; }
    }
    if (B2_IS_NULL(best)) return;

    auto& selected = m_robotDisplay.selected;
    auto it = std::find_if(selected.begin(), selected.end(),
                           [&](b2BodyId x){ return x.index1==best.index1 && x.generation==best.generation; });
    const bool already = (it != selected.end());
    if (toggle) {
        if (already) selected.erase(it);
        else selected.push_back(best);
    } else {
        selected.clear();
        selected.push_back(best);
    }
}

void PlatformBase::MouseUp(b2Vec2 /*p*/, int button)
{
    if (button != 0) return;
    if (m_editor.mode == EditMode::GridPaintBlock ||
        m_editor.mode == EditMode::GridPaintErase) {
        endGridPaint();
    }
}

void PlatformBase::MouseMove(b2Vec2 p)
{
    if (m_editor.mode == EditMode::GridPaintBlock ||
        m_editor.mode == EditMode::GridPaintErase) {
        if (m_editor.painting) continueGridPaintAt(p);
    }
}

//------------------------------------------------------------------------------
// helpers
//------------------------------------------------------------------------------
GridEnvironment*    PlatformBase::grid()    { return dynamic_cast<GridEnvironment*>(m_envOwned.get()); }
RoadmapEnvironment* PlatformBase::road()    { return dynamic_cast<RoadmapEnvironment*>(m_envOwned.get()); }
ContinuousEnvironment*    PlatformBase::freeenv() { return dynamic_cast<ContinuousEnvironment*>(m_envOwned.get()); }
const GridEnvironment*    PlatformBase::grid()    const { return dynamic_cast<const GridEnvironment*>(m_envOwned.get()); }
const RoadmapEnvironment* PlatformBase::road()    const { return dynamic_cast<const RoadmapEnvironment*>(m_envOwned.get()); }
const ContinuousEnvironment*    PlatformBase::freeenv() const { return dynamic_cast<const ContinuousEnvironment*>(m_envOwned.get()); }
bool PlatformBase::isGrid() const {
    return grid() != nullptr;
}

bool PlatformBase::isRoadmap() const {
    return road() != nullptr;
}

bool PlatformBase::isContinuous() const {
    return freeenv() != nullptr;
}

bool PlatformBase::hasAgents() const noexcept
{
    return m_sim && !m_sim->robots().robots().empty();
}

void PlatformBase::clearRobots()
{
    setSimulationRunning(false);
    platform::reset::clearRobots(m_sim.get(), m_envOwned.get());
    m_robotDisplay.selected.clear();
    m_robotDisplay.colors.clear();
    invalidatePlanPresentation();
    m_studio.agentsReady = false;
    m_studio.planReady = false;
}

void PlatformBase::adoptWorld(b2WorldId wid) { m_worldId = wid; }

void PlatformBase::centerCameraOnEnvironment()
{
    if (m_envOwned) centerOnEnv(*m_envOwned);
}

void PlatformBase::centerOnEnv(const BaseEnvironment& E)
{
    // Environment extents in world meters (origin assumed at (0,0) → (W,H))
    const auto b = E.boundsInMeter();
    const float Wenv = std::max(1e-6f, b.width);
    const float Henv = std::max(1e-6f, b.height);

    // Center the camera on the environment
    m_context->view->setCenter({ 0.5f * Wenv, 0.5f * Henv });

    // Compute a zoom that fits the whole env in the current window, with a small margin
    const auto winPx = m_context->view->getWindowSizePx();
    const float vpW  = std::max(1.0f, float(winPx.x));
    const float vpH  = std::max(1.0f, float(winPx.y));
    const float vpAR = vpW / vpH;

    // Add ~6% guard band so edges aren’t tight against the viewport
    const float margin = 0.06f;
    float W = Wenv * (1.0f + margin);
    float H = Henv * (1.0f + margin);

    // Expand the shorter side so the target box matches the viewport AR
    if ((W / H) > vpAR) { H = W / vpAR; } else { W = H * vpAR; }

    // Use zoom=1 as a baseline to read the world size, then set absolute zoom to fit W×H
    m_context->view->setZoom(0.8f);
    const b2Vec2 baseWorld = m_context->view->getViewSizeWorld();
    const float zx = W / std::max(1e-6f, baseWorld.x);
    const float zy = H / std::max(1e-6f, baseWorld.y);
    m_context->view->setZoom(std::max(zx, zy));
}

float PlatformBase::pxToWorld(float px) const
{
    const float winH  = float(std::max(1u, m_context->view->getWindowHeightPx()));
    const float viewH = m_context->view->getViewSizeWorld().y;
    return (px * viewH) / winH;
}

float PlatformBase::pickRadius() const { return pxToWorld(10.0f); }

uint64_t PlatformBase::idKey(b2BodyId id)
{
    return (uint64_t(id.index1) << 32) | uint64_t(id.generation);
}

void PlatformBase::ensurePalette()
{
    if (!m_sim) return;
    // Basic repeating palette (consistent & friendly)
    static const uint32_t base[] = {
        0xE24A33, 0x348ABD, 0x988ED5, 0x777777, 0xFBC15E, 0x8EBA42, 0xFF9DA7, 0x4ECDC4
    };
    // Add missing colors (don’t bail early when map is non-empty).
    uint32_t i = m_robotDisplay.paletteSeed;
    const auto& robots = m_sim->robots().robots();
    // If we already colored some, advance i to keep sequence consistent, but only
    // for reproducibility across add/remove. This is simple and deterministic.
    auto& colors = m_robotDisplay.colors;
    i += static_cast<uint32_t>(colors.size());

    for (b2BodyId id : robots) {
        const uint64_t key = idKey(id);
        if (colors.find(key) != colors.end()) continue;
        const uint32_t rgb = base[i % (sizeof(base)/sizeof(base[0]))];
        colors[key] = (b2HexColor)((uint32_t((rgb>>16)&255) << 24) |
                                    (uint32_t((rgb>> 8)&255) << 16) |
                                    (uint32_t( rgb     &255) <<  8) |
                                    230u);
        ++i;
    }
    // Optional: prune colors for robots that no longer exist (keeps map tidy).
    if (!colors.empty()) {
        std::unordered_set<uint64_t> live;
        live.reserve(robots.size());
        for (b2BodyId id : robots) live.insert(idKey(id));
        for (auto it = colors.begin(); it != colors.end(); ) {
            it = (live.count(it->first) ? std::next(it) : colors.erase(it));
        }
    }
}

void PlatformBase::recordExecutedOnce()
{
    if (!m_overlays.drawExecuted || !m_sim) return;
    auto& traces = m_executionTrace;
    if ((traces.counter++ % std::max(1, traces.decimate)) != 0) return;
    for (b2BodyId id : m_sim->robots().robots()) {
        auto& poly = traces.points[idKey(id)];
        poly.push_back(b2Body_GetPosition(id));
        if (traces.maxPoints && poly.size() > traces.maxPoints) {
            const size_t drop = poly.size() - traces.maxPoints;
            poly.erase(poly.begin(), poly.begin() + drop);
        }
    }
}

void PlatformBase::adoptEnvironment(
    std::unique_ptr<BaseEnvironment> env,
    std::unique_ptr<Simulation> sim,
    const char* status,
    ProblemSpecPtr problemSpec,
    std::filesystem::path problemSpecSource) {
    if (!env || !sim) {
        throw std::invalid_argument("Platform environment adoption requires an environment and simulation");
    }

    centerOnEnv(*env);
    m_envOwned = std::move(env);
    m_sim = std::move(sim);
    m_problemSpec = std::move(problemSpec);
    m_problemSpecSource = m_problemSpec
        ? std::move(problemSpecSource)
        : std::filesystem::path{};
    m_robotDisplay.selected.clear();
    m_robotDisplay.colors.clear();
    invalidatePlanPresentation();

    m_studio.planReady = false;
    m_studio.agentsReady = false;
    setSimulationRunning(false);
    m_studio.envReady = true;
    syncWorldBinding();
    m_studio.leftNavOpen = true;

    const auto kind = isGrid() ? platform::EnvKind::Grid
                    : isRoadmap() ? platform::EnvKind::Roadmap
                                  : platform::EnvKind::Continuous;
    const auto& allowed = platform::PlannerCatalog::allowed(kind);
    if (std::find(allowed.begin(), allowed.end(), m_plannerUi.name) == allowed.end()) {
        m_plannerUi.name = platform::PlannerCatalog::defaultFor(kind);
    }
    installPlanner(m_plannerUi.name);

    m_studio.status = status
        ? std::string(status)
        : "Environment ready. Edit as needed, then continue to Agents.";
}

PlatformBase::ProblemSpecPtr PlatformBase::loadProblemSpec(
    const std::filesystem::path& source) const {
    return std::make_shared<const grace::sim::ProblemYaml>(
        grace::sim::loadProblemYaml(source));
}

PlatformBase::ProblemSpecPtr PlatformBase::problemSpec(
    const std::filesystem::path& source) {
    if (!m_problemSpec || m_problemSpecSource != source) {
        m_problemSpec = loadProblemSpec(source);
        m_problemSpecSource = source;
    }
    return m_problemSpec;
}

void PlatformBase::ensureSimulation()
{
    if (!m_envOwned || m_sim) return;
    m_sim = std::make_unique<Simulation>(*m_envOwned);
}

void PlatformBase::setSimulationRunning(const bool running) noexcept
{
    platform::setSimulationRunning(
        m_studio, m_context->pause, running);
}

void PlatformBase::showSimulationDetails() noexcept
{
    m_studio.detailsOpen = true;
    m_studio.autoHideDetailsWhenRunning = false;
}

bool PlatformBase::simulationPaused() const noexcept
{
    return m_context->pause;
}

float PlatformBase::uiScale() const noexcept
{
    return m_context->uiScale;
}

float PlatformBase::overlaySafeTopPx() const noexcept
{
    return m_context->overlaySafeTopPx;
}

float PlatformBase::toolsPanelWidthFrac() const noexcept
{
    return m_context->toolsPanelWidthFrac;
}

ImFont* PlatformBase::mediumFont() const noexcept
{
    return m_context->m_mediumFont;
}

std::optional<b2Vec2> PlatformBase::worldPositionForPixel(
    const int x,
    const int y) const
{
    const auto window = m_context->view->getWindowSizePx();
    if (x < 0 || y < 0 ||
        static_cast<unsigned>(x) >= window.x ||
        static_cast<unsigned>(y) >= window.y) {
        return std::nullopt;
    }
    return m_context->view->mapPixelToWorld({x, y});
}

void PlatformBase::centerCameraOnPoint(const b2Vec2 point) const
{
    m_context->view->setCenter(point);
}

void PlatformBase::EnvironmentAccess::adoptEnvironment(
    std::unique_ptr<BaseEnvironment> env,
    std::unique_ptr<Simulation> sim,
    const char* status,
    ProblemSpecPtr problemSpec,
    std::filesystem::path problemSpecSource) {
    m_platform.adoptEnvironment(
        std::move(env),
        std::move(sim),
        status,
        std::move(problemSpec),
        std::move(problemSpecSource));
}

void PlatformBase::syncWorldBinding()
{
    if (m_envOwned) adoptWorld(m_envOwned->getB2World());
}

void PlatformBase::drawPanels()
{
    ui::DrawPlatformPanels(WorkflowAccess(m_accessKey, *this));
}
