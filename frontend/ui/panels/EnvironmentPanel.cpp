// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "UiWidgets.h"
#include "PlatformPanels.h"
#include "../UiStyle.h"           // PrimaryButton

#include "Simulation.h"
#include "io/SimContinuousYamlOps.h"

#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "env/ContinuousEnvironment.h"
#include "io/RoadmapXML.h"

#include "../widgets/FilePicker.h"
#include "../../sample/SampleContext.h"

#include <imgui.h>

#include "io/GridMovingAI.h"
#if __has_include(<misc/cpp/imgui_stdlib.h>)
  #include <misc/cpp/imgui_stdlib.h>
#elif __has_include(<imgui_stdlib.h>)
  #include <imgui_stdlib.h>
#endif
#include <algorithm>

namespace ui {

void Adopt(PlatformBase::EnvironmentAccess access,
           std::unique_ptr<BaseEnvironment> env,
           std::unique_ptr<Simulation> sim,
           const char* info,
           PlatformBase::ProblemSpecPtr problemSpec,
           std::filesystem::path problemSpecSource)
{
    access.adoptEnvironment(
        std::move(env),
        std::move(sim),
        info,
        std::move(problemSpec),
        std::move(problemSpecSource));
    access.showEnvironmentStep();
}

void DrawStepEnvironment(PlatformBase::EnvironmentAccess access)
{
    auto studio = access.state();
    auto editor = access.editorSettings();
    bool drawRoadmap = access.drawsRoadmap();

    // --- begin original body (with minor adaptions) ---
    ImGui::TextWrapped("Create or open an environment, then optionally edit it.");
    ImGui::Separator();

    // Representation selector
    ImGui::RadioButton("Grid", &studio.envChoice, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Roadmap", &studio.envChoice, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Continuous", &studio.envChoice, 2);

    // ----------------------------
    // Branch per representation
    // ----------------------------
    if (studio.envChoice == 0)
    {
        ImGui::SeparatorText("Open an existing environment");
        {
            const float scale = access.uiScale();
            const float minW    = 200.0f * scale;
            const float maxW    = 420.0f * scale;
            ImGui::SetNextItemWidth(std::clamp(ImGui::GetContentRegionAvail().x * 0.55f, minW, maxW));
            ImGui::InputText("MovingAI .map", &studio.mapPath);
        }
        ImGui::SameLine();
        if (ImGui::Button("Browse…")) ImGui::OpenPopup("PickMap##env-grid");
        if (ui::widgets::FilePickerModal("PickMap##env-grid", studio.mapPath, {".map"})) { /* field updated */ }
        ImGui::InputFloat("Cell length (m)", &studio.cellLength);
        if (ui::PrimaryButton("Load .map")) {
            const auto map =
                grace::io::readGridMovingAIMap(studio.mapPath);
            if (!map) {
                studio.status = map.error;
                return;
            }
            auto env = std::make_unique<GridEnvironment>();
            env->makeGridEnvironment(
                map.value->width, map.value->height,
                studio.cellLength, map.value->occupancy);
            env->addBoundaryWalls(0.14f);
            auto sim = std::make_unique<Simulation>(*env);
            Adopt(access, std::move(env), std::move(sim),
                  "Loaded map. Edit as needed, then click “Done — Next: Agents >>”.");
            access.clearRoadmapImport();
        }
        ImGui::Spacing();

        ImGui::SeparatorText("Create a new environment");
        ImGui::InputInt ("Width (cells)",  &studio.newW);
        ImGui::InputInt ("Height (cells)", &studio.newH);
        ImGui::InputFloat("Cell length",   &studio.newCell);
        ui::widgets::Help("Creates an empty grid. You can paint obstacles with the editor below.");
        if (ui::PrimaryButton("Create empty grid")) {
            auto env = std::make_unique<GridEnvironment>();
            const std::vector<bool> occ;
            env->makeGridEnvironment(studio.newW, studio.newH, studio.newCell, occ);
            env->addBoundaryWalls(0.14f);
            auto sim = std::make_unique<Simulation>(*env);
            Adopt(access, std::move(env), std::move(sim),
                  "New empty grid created. Edit as needed, then click “Done — Next: Agents >>”.");
            access.clearRoadmapImport();
        }

        if (!access.environment()) return;

        ImGui::SeparatorText("Environment editor");
        ImGui::TextDisabled("Tip: Click & drag in the scene. Release mouse to apply a stroke.");
        ImGui::TextDisabled("Hotkeys: [Left-click] paint/erase, [Mouse-wheel] zoom, [Space] pan.");
        ImGui::Spacing();

        bool isBlock = editor.mode == platform::EditorMode::GridPaintBlock;
        bool isErase = editor.mode == platform::EditorMode::GridPaintErase;
        if (ImGui::RadioButton("Paint blocked", isBlock)) {
            editor.mode = platform::EditorMode::GridPaintBlock;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Erase", isErase)) {
            editor.mode = platform::EditorMode::GridPaintErase;
        }

        ImGui::SliderInt("Brush radius (cells)", &editor.brushRadius, 0, 6);
        ImGui::TextDisabled("0 = 1×1, 1 = 3×3, ...");

        ImGui::Checkbox("Auto‑replan after edits", &editor.autoReplan);
        ImGui::SameLine();
        ImGui::Checkbox("Rebuild walls on apply", &editor.rebuildWallsOnApply);

        if (ImGui::Button("Clear all obstacles")) {
            if (auto* G = access.grid()) {
                const int H = G->gridHeightInCells(), W = G->gridWidthInCells();
                G->setOccupancyRect(0, 0, W-1, H-1, /*blocked=*/false);
                access.markGridVisualizationDirty();
                studio.status = "All obstacles cleared.";
                access.refreshWorldBinding();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Center camera")) {
            access.centerCamera();
        }

        ImGui::Spacing();

        ImGui::SeparatorText("Live status");
        if (access.environment()) {
            if (auto* G = access.grid()) {
                ImGui::Text("Grid: %d × %d cells @ %.3f m",
                            G->gridWidthInCells(), G->gridHeightInCells(), G->cellLength());
            }
            ImGui::Text("Bodies  | obstacles=%zu, walls=%zu, robots=%zu",
                        access.environment()->numObstacles(),
                        access.environment()->numBoundingWalls(),
                        access.environment()->numRobots());
        }
    }
    else if (studio.envChoice == 1)
    {
        // ---------------- ROADMAP ----------------
        ImGui::SeparatorText("Open an existing roadmap");
        {
            const float scale = access.uiScale();
            const float minW    = 200.0f * scale;
            const float maxW    = 420.0f * scale;
            ImGui::SetNextItemWidth(std::clamp(ImGui::GetContentRegionAvail().x * 0.55f, minW, maxW));
            ImGui::InputText("GraphML (.xml)", &studio.roadmapPath);
        }
        ImGui::SameLine();
        if (ImGui::Button("Browse…")) ImGui::OpenPopup("PickMap##env-roadmap");
        if (ui::widgets::FilePickerModal("PickMap##env-roadmap", studio.roadmapPath, {".xml", ".graphml"})) { /* updated */ }
        if (ui::PrimaryButton("Load roadmap")) {
            auto env = std::make_unique<RoadmapEnvironment>();
            grace::sim::GraphMLImport import;
            if (!grace::sim::readGraphML(studio.roadmapPath, *env, &import)) {
                studio.status = "Failed to load GraphML.";
            } else {
                env->addBoundaryWalls(editor.roadmapWallThickness);
                auto sim = std::make_unique<Simulation>(*env);

                Adopt(access, std::move(env), std::move(sim),
                      "Roadmap loaded. You can add nodes/edges, then proceed to Agents.");
                access.setDrawRoadmap(true);
                access.setRoadmapImport(import);
            }
        }
        ImGui::Spacing();

        ImGui::SeparatorText("Create a new roadmap");
        ui::widgets::Help("Creates an empty roadmap; use the editor below to add nodes and edges.");
        if (ui::PrimaryButton("Create empty roadmap")) {
            auto env = std::make_unique<RoadmapEnvironment>();
            env->addBoundaryWalls(editor.roadmapWallThickness);
            auto sim = std::make_unique<Simulation>(*env);

            Adopt(access, std::move(env), std::move(sim),
                  "Empty roadmap created. Add nodes/edges, then proceed to Agents.");
            access.setDrawRoadmap(true);
            access.clearRoadmapImport();
        }

        if (!access.environment()) return;

        ImGui::SeparatorText("Roadmap editor");
        ImGui::TextDisabled("Click in the scene to add nodes/edges. Mouse wheel zooms, Space pans.");
        ImGui::TextDisabled("Edges snap between picked nodes. Esc cancels a pending edge.");
        ImGui::Spacing();

        if (ImGui::Checkbox("Show roadmap graph", &drawRoadmap)) {
            access.setDrawRoadmap(drawRoadmap);
        }

        const bool rmNode =
            editor.mode == platform::EditorMode::RoadmapAddNode;
        const bool rmEU =
            editor.mode == platform::EditorMode::RoadmapAddEdgeUndirected;
        const bool rmED =
            editor.mode == platform::EditorMode::RoadmapAddEdgeDirected;
        const bool rmNone = editor.mode == platform::EditorMode::None;
        if (ImGui::RadioButton("None", rmNone)) {
            editor.mode = platform::EditorMode::None;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Add node", rmNode)) {
            editor.mode = platform::EditorMode::RoadmapAddNode;
        }

        if (ImGui::RadioButton("Add edge (undirected)", rmEU)) {
            editor.mode = platform::EditorMode::RoadmapAddEdgeUndirected;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Add edge (directed)", rmED)) {
            editor.mode = platform::EditorMode::RoadmapAddEdgeDirected;
        }

        {
            const float scale = access.uiScale();
            const float minW    = 200.0f * scale;
            const float maxW    = 420.0f * scale;
            ImGui::SetNextItemWidth(std::clamp(ImGui::GetContentRegionAvail().x * 0.55f, minW, maxW));
            ImGui::SliderFloat(
                "Pick radius (px)", &editor.nodePickRadiusPx, 6.0f, 30.0f, "%.0f px");
            ImGui::SetNextItemWidth(std::clamp(ImGui::GetContentRegionAvail().x * 0.55f, minW, maxW));
            ImGui::SliderFloat(
                "Wall thickness (m)", &editor.roadmapWallThickness, 0.05f, 0.60f, "%.2f");
        }

        ui::widgets::Help("Boundary walls are rebuilt automatically after adding nodes.\n"
                          "Click 'Rebuild now' to re-apply the current thickness.");

        if (ImGui::Button("Rebuild now")) {
            if (auto* environment = access.environment()) {
                environment->clearBoundingWalls();
                environment->addBoundaryWalls(editor.roadmapWallThickness);
                access.refreshWorldBinding();
            }
        }

        ImGui::Checkbox("Auto‑replan after edits", &editor.autoReplan);
        ImGui::SameLine();
        if (ImGui::Button("Replan now")) {
            (void)access.replanPreservingGoals();
        }

        if (ImGui::Button("Center camera")) {
            access.centerCamera();
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Live status");
        ImGui::Text("Bodies  | obstacles=%zu, walls=%zu, robots=%zu",
                    access.environment()->numObstacles(),
                    access.environment()->numBoundingWalls(),
                    access.environment()->numRobots());
    }
    else // Continuous
    {
        // -------------- CONTINUOUS --------------
        ImGui::SeparatorText("Open an existing problem (YAML) — ENVIRONMENT ONLY");
        {
            const float scale = access.uiScale();
            const float minW    = 200.0f * scale;
            const float maxW    = 420.0f * scale;
            ImGui::SetNextItemWidth(std::clamp(ImGui::GetContentRegionAvail().x * 0.55f, minW, maxW));
            ImGui::InputText("problem.yaml", &studio.problemYamlPath);
        }
        ImGui::SameLine();
        if (ImGui::Button("Browse…")) ImGui::OpenPopup("PickMap##env-cont");
        if (ui::widgets::FilePickerModal("PickMap##env-cont", studio.problemYamlPath, {".yaml", ".yml"})) { /* updated */ }

        ImGui::SetNextItemWidth(140.0f * access.uiScale());
        ImGui::InputFloat(
            "Boundary wall thickness (m)", &studio.freeWallThickness, 0.01f, 0.10f, "%.2f");
        ui::widgets::Help("Applied when loading YAML.");
        if (ui::PrimaryButton("Load problem.yaml (environment only — no agents, no plan)")) {
            const auto problem =
                access.loadProblemSpec(studio.problemYamlPath);
            auto env = std::make_unique<ContinuousEnvironment>();
            auto sim = std::make_unique<Simulation>(*env);

            const bool ok = grace::sim::ops::buildEnvironmentFromProblemSpec(
                *env, *problem, studio.freeWallThickness);

            if (!ok) {
                studio.status = "Failed to build environment from problem.yaml (see log).";
            } else {
                Adopt(access, std::move(env), std::move(sim),
                      "Environment built from problem.yaml (obstacles & walls). Next: Agents.",
                      problem,
                      studio.problemYamlPath);
                access.clearRoadmapImport();
            }
        }
        ImGui::Spacing();
        ImGui::SeparatorText("Or: create an empty continuous world");
        ImGui::InputFloat("Width  (m)", &studio.freeNewW, 0.5f, 2.0f, "%.2f");
        ImGui::InputFloat("Height (m)", &studio.freeNewH, 0.5f, 2.0f, "%.2f");
        if (ui::PrimaryButton("Create empty continuous env")) {
            if (studio.freeNewW <= 0.0f || studio.freeNewH <= 0.0f) {
                studio.status = "Width/Height must be > 0.";
            } else {
                auto env = std::make_unique<ContinuousEnvironment>();
                env->setBounds(studio.freeNewW, studio.freeNewH);
                env->addBoundaryWalls(studio.freeWallThickness);
                auto sim = std::make_unique<Simulation>(*env);
                Adopt(access, std::move(env), std::move(sim),
                      "Empty continuous env created. Next: Agents.");
                access.clearRoadmapImport();
            }
        }
        if (!access.environment()) return;

        ImGui::SeparatorText("Continuous-space editor (no auto planning)");
        ImGui::TextDisabled("Click in the scene to add obstacles. Mouse wheel zooms, Space pans.");
        ImGui::Spacing();

        // Modes
        const bool cBox =
            editor.mode == platform::EditorMode::ContinuousAddBox;
        const bool cCircle =
            editor.mode == platform::EditorMode::ContinuousAddCircle;
        const bool cNone = editor.mode == platform::EditorMode::None;
        if (ImGui::RadioButton("None", cNone)) {
            editor.mode = platform::EditorMode::None;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Add box", cBox)) {
            editor.mode = platform::EditorMode::ContinuousAddBox;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Add circle", cCircle)) {
            editor.mode = platform::EditorMode::ContinuousAddCircle;
        }

        ImGui::SliderFloat("Box W (m)", &editor.boxWidth, 0.20f, 3.0f, "%.2f");
        ImGui::SliderFloat("Box H (m)", &editor.boxHeight, 0.20f, 3.0f, "%.2f");
        ImGui::SliderFloat(
            "Circle R (m)", &editor.circleRadius, 0.05f, 1.50f, "%.2f");

        if (ImGui::Button("Center camera")) {
            access.centerCamera();
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Live status");
        ImGui::Text("Bodies  | obstacles=%zu, walls=%zu, robots=%zu",
                    access.environment()->numObstacles(),
                    access.environment()->numBoundingWalls(),
                    access.environment()->numRobots());
    }

    // ----------- common footer (all env kinds) -----------
    if (ui::PrimaryButton("Done — Next: Agents →")) {
        editor.mode = platform::EditorMode::None;
        access.showAgentsStep();
        studio.leftNavOpen = true;
        studio.status = "Environment edited. Proceed to Agents.";
        access.refreshWorldBinding();
    }
    access.setEditorSettings(editor);
    // --- end original body ---
}

} // namespace ui
