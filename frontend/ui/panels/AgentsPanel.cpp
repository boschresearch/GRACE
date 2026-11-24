// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "PlatformPanels.h"
#include "../UiStyle.h"

#include "Simulation.h"
#include "sim/SpawnRobot.h"
#include "io/GridMovingAI.h"
#include "io/GridLORR.h"
#include "io/SimContinuousYamlOps.h"

#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "env/ContinuousEnvironment.h"
#include "io/RoadmapXML.h"

#include "../../sample/SampleContext.h"
#include "../ViewController.h"
#include "../widgets/FilePicker.h"

#include <imgui.h>

#include "UiWidgets.h"
#include "io/SimRoadmapOps.h"
#if __has_include(<misc/cpp/imgui_stdlib.h>)
  #include <misc/cpp/imgui_stdlib.h>
#elif __has_include(<imgui_stdlib.h>)
  #include <imgui_stdlib.h>
#endif
#include <fmt/format.h>
#include <limits>
#include <random>
#include <algorithm>

namespace ops = grace::sim::ops;   // kept for roadmap/continuous paths

namespace ui {

void DrawAgentInspector(PlatformBase::AgentsAccess access)
{
    const auto& selectedRobots = access.selectedRobots();
    auto studio = access.state();
    auto* simulation = access.simulation();

    // moved from old UiPlatformPanels.cpp (DrawAgentInspector) unchanged
    // --- begin original body ---
    if (!simulation) return;
    ImGui::SeparatorText("Agent inspector");
    if (selectedRobots.empty()) {
        ImGui::TextDisabled("Select a robot to inspect.");
        ImGui::TextDisabled("Tip: click a robot, Shift+Click assigns a goal.");
        return;
    }
    const b2BodyId id = selectedRobots.back();
    RobotMeta* meta = simulation->robots().meta(id);
    if (!meta) { ImGui::TextDisabled("No metadata for selected robot."); return; }

    const b2Vec2 p  = b2Body_GetPosition(id);
    const b2Vec2 v  = b2Body_GetLinearVelocity(id);
    ImGui::InputText("Name", &meta->name);
    ImGui::BeginDisabled();
    ImGui::Text("ID: (%u, gen=%u)", id.index1, id.generation);
    ImGui::EndDisabled();

    if (auto* G = access.grid()) {
        const auto [c, r] = G->worldToCell(p);
        ImGui::Text("Pos: (%.3f, %.3f)  Cell: (r=%d, c=%d)", p.x, p.y, r, c);
    } else {
        ImGui::Text("Pos: (%.3f, %.3f)", p.x, p.y);
    }
    ImGui::Text("Vel: (%.3f, %.3f)", v.x, v.y);
    ImGui::Text("Shape: %s  radius=%.3f", (meta->shape==Type::ShapeType::Circle?"Circle":
                       meta->shape==Type::ShapeType::Box?"Box":"Hull"), meta->effectiveRadius);
    ImGui::Text("Plan waypoints: %zu  next=%zu", meta->plan.waypoints.size(), meta->next_wp);

    if (ImGui::CollapsingHeader("Limits", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Active", &meta->active);
        ImGui::TextDisabled("Note: Changing radius/shape would require rebuilding the body;");
        ImGui::TextDisabled("we keep geometry as-is and use 'radius' for planner hints.");
    }

    ImGui::SeparatorText("Goal tools");
    if (meta->requested_goal_m) {
        const b2Vec2 g = *meta->requested_goal_m;
        if (access.grid()) {
            const auto [cg, rg] = access.grid()->worldToCell(p);
            ImGui::Text("Goal: (%.3f, %.3f)  Cell: (r=%d, c=%d)", g.x, g.y, rg, cg);
        } else {
            ImGui::Text("Goal: (%.3f, %.3f)", g.x, g.y);
        }
    } else {
        ImGui::TextDisabled("No goal recorded for this robot.");
    }
    if (ImGui::Button("Clear goal")) {
        meta->requested_goal_m.reset();
        studio.status = "Goal cleared for selected robot.";
    }
    ImGui::SameLine();
    if (ImGui::Button("Set goal = mouse")) {
        const ImVec2 mp = ImGui::GetMousePos();
        const int mx = int(std::lround(mp.x)), my = int(std::lround(mp.y));
        if (const auto goal = access.worldPositionForPixel(mx, my)) {
            studio.status = access.assignRobotGoal(id, *goal)
                ? "Goal set from mouse."
                : "Failed to set goal for selected robot.";
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Replan selected")) {
        const bool planned = access.replanRobotPreservingGoal(id);
        studio.planReady = simulation->maxPlannedTime() > 0.0f;
        studio.status = planned
            ? "Plan updated for selection."
            : "Planning failed for selected robot.";
    }

    ImGui::SeparatorText("Utilities");
    if (ImGui::Button("Center camera on agent")) {
        access.centerCameraOnPoint(p);
    }
    ImGui::SameLine();
    if (ImGui::Button("Teleport to goal")) {
        if (meta->requested_goal_m) {
            const b2Vec2 g = *meta->requested_goal_m;
            b2Body_SetTransform(id, g, b2Rot_identity);
            b2Body_SetLinearVelocity(id, {0,0});
        }
    }
    // --- end original body ---
}

void DrawStepAgents(PlatformBase::AgentsAccess access)
{
    access.ensureSimulation();
    if (!access.environment()) {
        ImGui::TextDisabled("Load or create an environment first.");
        return;
    }

    if (access.isGrid()) {
        DrawGridAgentsUI(access);
    } else if (access.isRoadmap()) {
        DrawRoadmapAgentsUI(access);
    } else {
        DrawFreeAgentsUI(access);
    }
}

// ----------------- GRID (adapted to grace::io + grace::sim only) -----------------
void DrawGridAgentsUI(PlatformBase::AgentsAccess access)
{
    auto studio = access.state();
    auto* simulation = access.simulation();

    ImGui::TextWrapped("Load agents from a scenario, or spawn from agents/tasks files.");
    ImGui::Separator();

    ImGui::Checkbox("Agents from .scen (MovingAI)", &studio.agentsFromScenario);
    if (studio.agentsFromScenario) {
        ImGui::InputText("Scenario (.scen)", &studio.scenPath);
        ImGui::InputInt("Take K robots", &studio.takeK);
        ImGui::InputText("Base name", &studio.baseName);
        if (ui::PrimaryButton("Load agents (spawn from .scen)")) {
            if (!access.grid() || !simulation) {
                studio.status = "Load a grid first.";
            } else {
                const std::size_t K = (studio.takeK > 0)
                    ? static_cast<std::size_t>(studio.takeK)
                    : std::numeric_limits<std::size_t>::max();

                // Parse .scen -> grid cell pairs
                auto scenario = grace::io::readGridMovingAIScenario(
                    *access.grid(), studio.scenPath, K);

                if (!scenario) {
                    studio.status = scenario.error;
                } else if (scenario.value->empty()) {
                    studio.status = "Scenario contains no agents.";
                } else {
                    access.clearRobots();
                    const auto& specs = *scenario.value;
                    // Spawn (records goals automatically)
                    grace::sim::SpawnOptions opt;
                    opt.baseName = studio.baseName.empty() ? "r" : studio.baseName;
                    auto ids = grace::sim::spawnGridAgent(*simulation, specs, opt);

                    if (ids.empty()) {
                        studio.agentsReady = false;
                        studio.status = "Spawn failed (check starts on obstacles?).";
                    } else {
                        studio.agentsReady = true;
                        studio.planReady = false;
                        access.ensurePalette();
                        studio.status = fmt::format(
                            "Spawned {} robot(s). Goals loaded from .scen file.", ids.size());
                    }
                }
            }
        }
    } else {
        ImGui::InputText("Agents (.agents)", &studio.agentsPath);
        ImGui::InputText("Tasks (.tasks)",   &studio.tasksPath);
        ImGui::InputInt("Take K robots", &studio.takeK);
        ImGui::InputText("Base name", &studio.baseName);
        if (ui::PrimaryButton("Load agents (spawn from .agents/.tasks)")) {
            if (!access.grid() || !simulation) {
                studio.status = "Load map first.";
            } else {
                // reset state
                access.clearRobots();

                const std::size_t K = (studio.takeK > 0)
                    ? static_cast<std::size_t>(studio.takeK)
                    : std::numeric_limits<std::size_t>::max();

                const std::string base = studio.baseName.empty() ? "r" : studio.baseName;

                // Enforce presence of BOTH .agents and .tasks
                const bool hasAgents =
                    !studio.agentsPath.empty() && std::filesystem::exists(studio.agentsPath);
                const bool hasTasks =
                    !studio.tasksPath.empty() && std::filesystem::exists(studio.tasksPath);

                if (!hasAgents || !hasTasks) {
                    studio.agentsReady = false;
                    studio.status =
                        "Provide BOTH .agents and .tasks files (agents = starts, tasks = goals).";
                } else {
                    // Unified LORR read
                    grace::io::GridLORROptions lopt;
                    lopt.takeK               = K;
                    lopt.validateAgainstGrid = true;
                    lopt.skipBlockedCells    = true;
                    lopt.uniqueGoals         = true;

                    auto specs = grace::io::readGridLORRAgentsTasks(
                        *access.grid(), studio.agentsPath, studio.tasksPath, lopt);

                    if (specs.empty()) {
                        studio.agentsReady = false;
                        studio.status = "Spawn failed or empty/invalid .agents/.tasks.";
                    } else {
                        grace::sim::SpawnOptions opt;
                        opt.baseName    = base;

                        auto ids = grace::sim::spawnGridAgent(*simulation, specs, opt);

                        if (ids.empty()) {
                            studio.agentsReady = false;
                            studio.status = "Spawn failed (starts may be blocked?).";
                        } else {
                            studio.agentsReady = true;
                            studio.planReady = false;
                            access.ensurePalette();
                            studio.status = fmt::format(
                                "Spawned {} robot(s) from .agents and loaded goals from .tasks.", ids.size());
                        }
                    }
                }
            }
        }
    }

    if (!(access.environment() && simulation &&
          !simulation->robots().robots().empty())) {
        return;
    }

    ImGui::SeparatorText("Agent editor");
    ImGui::TextDisabled("Tips:");
    ImGui::BulletText("Click robot to select. Click again to toggle selection.");
    ImGui::BulletText("Shift+Click anywhere: assign goal to selected. (Snaps to grid on Grid envs.)");
    ImGui::BulletText("Ctrl+Click anywhere: spawn a robot at the click position.");
    ImGui::BulletText("Alt+Click: pick without toggling.");
    ImGui::Spacing();

    ImGui::SeparatorText("Live status");
    {
        const std::size_t n = simulation->robots().robots().size();
        ImGui::Text("Agents in simulation: %zu", n);
    }

    if (ImGui::Button("Select none")) {
        access.clearSelection();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear agents")) {
        access.clearRobots();
        studio.status = "Agents cleared.";
    }
    ImGui::SameLine();
    if (ImGui::Button("Center camera")) {
        access.centerCamera();
    }

    ImGui::Spacing();
    DrawAgentInspector(access);
    if (ui::PrimaryButton("Next: Planner >>")) {
        access.showPlannerStep();
        studio.leftNavOpen = true;
        studio.status = "Agents ready. Choose a planner next.";
    }
}

// ----------------- ROADMAP (kept using existing ops) -----------------
void DrawRoadmapAgentsUI(PlatformBase::AgentsAccess access)
{
    const auto& editor = access.editor();
    auto studio = access.state();
    auto* simulation = access.simulation();

    ImGui::TextWrapped("Spawn robots on roadmap nodes, then set goals. You can also Ctrl+Click to spawn and Shift+Click to set a goal.");
    ImGui::Separator();

    auto* R = access.roadmap();
    if (!R || !simulation) {
        ImGui::TextDisabled("Load or create a roadmap first.");
        return;
    }

    // Inputs
    ImGui::InputInt("Robots to spawn (N)", &studio.takeK);
    if (studio.takeK < 0) studio.takeK = 0;
    ImGui::InputText("Base name", &studio.baseName);
    ImGui::InputFloat(
        "Robot radius (m)", &studio.roadRobotRadius, 0.01f, 0.10f, "%.3f");

    // --------- Load tasks.xml (Roadmap) – spawn &/or set goals (NO planning) ----------
    ImGui::SeparatorText("Load tasks.xml (Roadmap)");
    {
        const float scale = access.uiScale();
        const float minW  = 200.0f * scale;
        const float maxW  = 420.0f * scale;
        ImGui::SetNextItemWidth(std::clamp(ImGui::GetContentRegionAvail().x * 0.55f, minW, maxW));
        ImGui::InputText("tasks.xml", &studio.taskXmlPath);
        ImGui::SameLine();
        if (ImGui::Button("Browse…")) ImGui::OpenPopup("PickTasks##agents-roadmap");
        if (ui::widgets::FilePickerModal(
                "PickTasks##agents-roadmap", studio.taskXmlPath, {".xml"})) {
            /* updated */
        }
    }

    const bool haveImport =
        editor.graphImport.has_value() || editor.hasRoadmapImport;
    if (!haveImport) {
        ImGui::TextDisabled("Note: tasks.xml needs a GraphML import mapping. Load a GraphML roadmap in Environment first.");
    }

    const grace::sim::GraphMLImport* importPtr =
        editor.graphImport
            ? &*editor.graphImport
            : (editor.hasRoadmapImport ? &editor.roadmapImport : nullptr);

    // Spawn from tasks.xml (no planning)
    ImGui::BeginDisabled(!haveImport || studio.taskXmlPath.empty());
    if (ui::PrimaryButton("Spawn from tasks.xml (no plan)")) {
        access.clearRobots();

        const std::size_t K = (studio.takeK > 0)
            ? static_cast<std::size_t>(studio.takeK)
            : std::numeric_limits<std::size_t>::max();

        auto ids = ops::spawnFromTaskXmlNoPlan(
            *simulation, *R, studio.taskXmlPath, *importPtr,
            /*robotRadius=*/studio.roadRobotRadius,
            /*takeK=*/K,
            /*baseName=*/studio.baseName.empty() ? std::string("r") : studio.baseName);

        if (ids.empty()) {
            studio.agentsReady = false;
            studio.status = "Spawn from tasks.xml failed or no tasks found.";
        } else {
            access.ensurePalette();
            studio.agentsReady = true;
            studio.planReady = false;
            access.invalidatePlanPresentation();
            studio.status = fmt::format(
                "Spawned {} robot(s) from tasks.xml and recorded goals (no planning).",
                ids.size());
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();

    // Apply goals (no planning) for already-present agents
    ImGui::BeginDisabled(
        !haveImport || studio.taskXmlPath.empty() ||
        simulation->robots().robots().empty());
    if (ImGui::Button("Apply goals to current agents (no plan)")) {
        const auto& ids = simulation->robots().robots();
        const bool ok = ops::assignGoalsFromTaskXml(
            *simulation, *R, ids, studio.taskXmlPath, *importPtr);
        access.markPlansStale(ids);
        studio.status = ok
            ? fmt::format("Recorded goals from tasks.xml for {} agent(s).", ids.size())
            : "Failed to record goals from tasks.xml (see log).";
    }
    ImGui::EndDisabled();

    // Actions
    if (ui::PrimaryButton("Spawn N robots at random nodes")) {
        const int N = studio.takeK;
        const int M = R->numNodes();
        if (N <= 0) {
            studio.status = "N must be > 0.";
        } else if (M <= 0) {
            studio.status = "Roadmap has no nodes to spawn on.";
        } else {
            std::mt19937 rng{ std::random_device{}() };
            std::uniform_int_distribution<int> pick(0, M - 1);

            int added = 0;
            for (int i = 0; i < N; ++i) {
                const int u = pick(rng);
                const std::string name = fmt::format(
                    "{}{}", studio.baseName.empty() ? "r" : studio.baseName, i+1);
                b2BodyId id = simulation->spawnRoadmapNodeRobot(
                    *R, u, studio.roadRobotRadius, name);
                if (B2_IS_NON_NULL(id)) ++added;
            }
            if (added > 0) {
                access.ensurePalette();
                studio.agentsReady = true;
                access.markPlansStale(simulation->robots().robots());
                studio.status =
                    fmt::format("Spawned {} robot(s) at random nodes.", added);
            } else {
                studio.status = "No robots added (check radius?).";
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Assign random goals to all robots")) {
        const auto& ids = simulation->robots().robots();
        const int M = R->numNodes();
        if (ids.empty() || M <= 0) {
            studio.status = "No robots or nodes available.";
        } else {
            std::mt19937 rng{ std::random_device{}() };
            std::uniform_int_distribution<int> pick(0, M - 1);
            int updated = 0;
            for (b2BodyId id : ids) {
                if (auto* meta = simulation->robots().meta(id)) {
                    const int u = pick(rng);
                    meta->requested_goal_m = R->nodePos(u);
                    ++updated;
                }
            }
            access.markPlansStale(ids);
            studio.status =
                fmt::format("Assigned random goals to {} robot(s).", updated);
        }
    }

    ImGui::SeparatorText("Agent editor");
    ImGui::TextDisabled("Tips:");
    ImGui::BulletText("Click robot to select. Click again to toggle selection.");
    ImGui::BulletText("Shift+Click anywhere: assign goal to selected (snaps to nearest node).");
    ImGui::BulletText("Ctrl+Click anywhere: spawn a robot at the nearest node.");
    ImGui::BulletText("Alt+Click: pick without toggling.");
    ImGui::Spacing();

    ImGui::SeparatorText("Live status");
    {
        const std::size_t n = simulation->robots().robots().size();
        ImGui::Text("Agents in simulation: %zu", n);
    }

    if (ImGui::Button("Select none")) {
        access.clearSelection();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear agents")) {
        access.clearRobots();
        studio.status = "Agents cleared.";
    }
    ImGui::SameLine();
    if (ImGui::Button("Center camera")) {
        access.centerCamera();
    }

    ImGui::Spacing();
    DrawAgentInspector(access);
    if (ui::PrimaryButton("Next: Planner >>")) {
        access.showPlannerStep();
        studio.leftNavOpen = true;
        studio.status = "Agents ready. Choose a planner next.";
    }
}

// ----------------- CONTINUOUS (kept using existing ops) -----------------
void DrawFreeAgentsUI(PlatformBase::AgentsAccess access)
{
    auto studio = access.state();
    auto* simulation = access.simulation();

    ImGui::TextWrapped("Spawn robots directly in free space. Ctrl+Click also spawns at cursor in the scene.");
    ImGui::Separator();

    auto* E = access.continuous();
    if (!E || !simulation) {
        ImGui::TextDisabled("Load or create a continuous environment first.");
        return;
    }

    // --- YAML-based spawn/goals (no planning) ---
    ImGui::SeparatorText("Load from problem.yaml (no planning)");
    {
        const float scale = access.uiScale();
        const float minW    = 200.0f * scale;
        const float maxW    = 420.0f * scale;
        ImGui::SetNextItemWidth(std::clamp(ImGui::GetContentRegionAvail().x * 0.55f, minW, maxW));
        ImGui::InputText("problem.yaml", &studio.problemYamlPath);
    }
    ImGui::SameLine();
    if (ImGui::Button("Browse…")) ImGui::OpenPopup("PickYaml##agents-free");
    if (ui::widgets::FilePickerModal(
            "PickYaml##agents-free", studio.problemYamlPath, {".yaml", ".yml"})) {
        /* updated */
    }

    ImGui::InputInt("Take K robots", &studio.takeK);
    ImGui::InputText("Base name", &studio.baseName);
    ui::widgets::Help("Spawns at YAML start positions and records YAML goals. No planning.");
    if (ui::PrimaryButton("Spawn from YAML (no plan)")) {
        access.clearRobots();
        const auto problem =
            access.problemSpec(studio.problemYamlPath);

        const std::size_t K = (studio.takeK > 0)
            ? static_cast<std::size_t>(studio.takeK)
            : std::numeric_limits<std::size_t>::max();

        auto ids = ops::spawnFromProblemSpecNoPlan(
            *simulation, *problem, K,
            studio.baseName.empty() ? std::string("r") : studio.baseName);

        if (ids.empty()) {
            studio.agentsReady = false;
            studio.status = "Spawn from YAML failed (see log).";
        } else {
            access.ensurePalette();
            studio.agentsReady = true;
            studio.planReady = false;
            access.invalidatePlanPresentation();
            studio.status = fmt::format(
                "Spawned {} robot(s) from YAML and recorded goals (no planning).",
                ids.size());
        }
    }
    ImGui::SameLine();
    {
        const bool disabled =
            studio.problemYamlPath.empty() ||
            simulation->robots().robots().empty();
        ImGui::BeginDisabled(disabled);
        if (ImGui::Button("Apply goals from YAML (no plan)")) {
            const auto& ids = simulation->robots().robots();
            const auto problem =
                access.problemSpec(studio.problemYamlPath);
            const bool ok = ops::applyGoalsFromProblemSpec(
                *simulation, ids, *problem);
            access.markPlansStale(ids);
            studio.status = ok
                ? fmt::format("Recorded goals from YAML for {} agent(s).", ids.size())
                : "Failed to record goals from YAML (see log).";
        }
        ImGui::EndDisabled();
    }

    ImGui::Separator();

    ImGui::InputInt("Robots to spawn (N)", &studio.takeK);
    if (studio.takeK < 0) studio.takeK = 0;
    ImGui::InputText("Base name", &studio.baseName);
    static float rad = 0.08f;
    ImGui::InputFloat("Robot radius (m)", &rad, 0.01f, 0.1f, "%.3f");

    if (ui::PrimaryButton("Spawn N robots in a grid near center")) {
        const int N = studio.takeK;
        if (N <= 0) {
            studio.status = "N must be > 0.";
        }
        else {
            const auto B = E->boundsInMeter();
            const b2Vec2 c{ 0.5f*B.width, 0.5f*B.height };
            int k = std::max(1, (int)std::ceil(std::sqrt(float(N))));
            const float gap = 2.5f * rad;
            int added = 0;
            int n = 0;
            for (int r = 0; r < k && n < N; ++r) {
                for (int q = 0; q < k && n < N; ++q) {
                    const float x = c.x + (q - k/2) * gap;
                    const float y = c.y + (r - k/2) * gap;
                    const std::string name = fmt::format(
                        "{}{}", studio.baseName.empty() ? "u" : studio.baseName, n+1);
                    b2BodyId id = simulation->spawnCircleRobot({x,y}, rad, name);
                    if (B2_IS_NON_NULL(id)) { ++added; ++n; }
                }
            }
            if (added > 0) {
                access.ensurePalette();
                studio.agentsReady = true;
                access.markPlansStale(simulation->robots().robots());
                studio.status = fmt::format("Spawned {} robot(s).", added);
            }
        }
    }

    ImGui::SeparatorText("Live status");
    {
        const std::size_t n = simulation->robots().robots().size();
        ImGui::Text("Agents in simulation: %zu", n);
    }

    if (ImGui::Button("Select none")) {
        access.clearSelection();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear agents")) {
        access.clearRobots();
        studio.status = "Agents cleared.";
    }
    ImGui::SameLine();
    if (ImGui::Button("Center camera")) {
        access.centerCamera();
    }

    ImGui::Spacing();
    DrawAgentInspector(access);
    if (ui::PrimaryButton("Next: Planner >>")) {
        access.showPlannerStep();
        studio.leftNavOpen = true;
        studio.status = "Agents ready. Choose a planner next.";
    }
}

} // namespace ui
