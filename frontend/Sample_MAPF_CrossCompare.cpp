// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// Sample_MAPF_CrossCompare.cpp — Compare continuous vs roadmap vs grid on the same YAML
#include "sample/Sample.h"
#include "sample/SampleContext.h"
#include "platform/PlatformDefaults.h"
#include "platform/PlannerCatalog.h"
#include "platform/CrossComparisonWorkflow.h"
#include "platform/PlanningWorkflow.h"
#include "render/CrossComparisonRenderer.h"

#include "ui/UiStyle.h"                       // PrimaryButton, theme
#include "ui/panels/MetricsUI.h"              // existing single-aggregate tables
#include "ui/panels/MetricsCompareUI.h"       // new comparison table
#include "ui/widgets/FilePicker.h"            // optional file picker modal

//  sim + helpers
#include "Simulation.h"
#include "io/SimContinuousYamlOps.h"

// Env + planners + IO
#include "planner/plugin/PlannerFactory.h"
#include "env/ContinuousEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "env/GridEnvironment.h"
#include "io/RoadmapXML.h"
#include "io/ContinuousYaml.h"

#include <imgui.h>
#include <fmt/format.h>
#include <filesystem>
#include <chrono>
#include <string>
#include <vector>
#include <optional>
#include <iomanip>
#include <sstream>

#include <imgui.h>
#include <imgui_internal.h>

#include "io/GridMovingAI.h"
#include "io/GridXML.h"
#include "io/SimRoadmapOps.h"
#include "sim/SpawnRobot.h"

#if __has_include(<misc/cpp/imgui_stdlib.h>)
  #include <misc/cpp/imgui_stdlib.h>
#elif __has_include(<imgui_stdlib.h>)
  #include <imgui_stdlib.h>
#else
  #error "ImGui stdlib helpers not found (imgui_stdlib.h). Required for InputText with std::string."
#endif

#include <algorithm> // std::clamp

namespace fs = std::filesystem;
namespace ops = grace::sim::ops; // ⬅️ helper namespace

namespace {

    const auto kProblemYaml = platform::defaults::problemYamlDemo();
static std::string NowStamp()
{
    using clock = std::chrono::system_clock;
    const auto t  = clock::to_time_t(clock::now());
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream timestamp;
    timestamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return timestamp.str();
}

} // namespace

// -----------------------------------------------------------------------------
// Sample
// -----------------------------------------------------------------------------
class Sample_CrossCompare final : public Sample
{
public:
    explicit Sample_CrossCompare(SampleContext* ctx) : Sample(ctx)
    {
        m_context->pause = true;
        m_yamlPath = kProblemYaml;
        // Results go under a timestamped folder by default.
        m_outDir = fs::path("results") / fs::path("cross_compare") / NowStamp();

        m_plannerCont = platform::PlannerCatalog::defaultFor(
            platform::EnvKind::Continuous);
        m_plannerRoad = platform::PlannerCatalog::defaultFor(
            platform::EnvKind::Roadmap);
        m_plannerGrid = platform::PlannerCatalog::defaultFor(
            platform::EnvKind::Grid);
        m_timeLimitS  = 45.0;

        // PRM* defaults
        m_prm.target_samples = 1000;
        m_prm.time_sec       = 1.0;
        m_prm.robot_radius   = 0.08f;
        m_prm.pin_starts_goals = true;
        m_prm.connect_k        = 0;
        m_prm.connect_radius   = 1.0f;

        // Raster defaults (leave cell <= 0 to auto from YAML robots, per paper)
        m_ras.cell          = 0.15625;
        m_ras.inset         = 0.0f;
        m_ras.occ_threshold = 0.0f;
        m_ras.supersample_px= 9;

        m_gridFmt = tools::mc::GridFormat::MovingAI;
        m_wallThickness = 0.0f;
        m_roadRobotRadius = 0.08f;  // spawn radius for roadmap robots

        m_status = "Pick a problem.yaml, then click \"Run comparison\".";
    }

    void Step() override
    {
        DrawTitle("MAPF vs. MRMP: Cross representation comparison");

        if (!m_context->showUI) return;

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 10.0f,
                                       vp->WorkPos.y + 10.0f + m_context->overlaySafeTopPx),
                                ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(2000.0f, std::min(1500.0f, vp->WorkSize.y - 20.0f)));

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse;
        if (ImGui::Begin("Cross representation comparison", nullptr, flags)) {
            // --- begin stable 2-column split (table + children) ---
            const float windowW  = ImGui::GetContentRegionAvail().x;
            const float minLeft  = 280.0f;                                 // never smaller than this
            const float maxLeft  = ImMax(minLeft, windowW - m_minRightPanelPx);
            // Clamp once per frame, based only on window width (not current content)
            m_leftPanelWidthPx = ImClamp(m_leftPanelWidthPx, minLeft, maxLeft);

            // Table: left = fixed pixels, right = stretch remainder
            const ImGuiTableFlags splitFlags =
                ImGuiTableFlags_SizingStretchProp |
                ImGuiTableFlags_BordersInnerV     |
                ImGuiTableFlags_NoSavedSettings;   // avoid .ini side-effects

            if (ImGui::BeginTable("cc_split", 2, splitFlags, ImVec2(-FLT_MIN, 0.0f))) {
                ImGui::TableSetupColumn("Controls", ImGuiTableColumnFlags_WidthFixed,  m_leftPanelWidthPx);
                ImGui::TableSetupColumn("Preview",  ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableNextRow();

                // -------------------------- LEFT COLUMN (controls/results) --------------------------
                ImGui::TableSetColumnIndex(0);
                // Child isolates vertical scrolling so the parent width never shrinks when content grows
                ImGui::BeginChild("left_panel", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);

                ImGui::SeparatorText("Input");
                {
                    // Keep path inputs compact and independent of content width
                    const float labelPadding = ImGui::GetStyle().ItemInnerSpacing.x * 2.0f;
                    const float widestLabel  = ImGui::CalcTextSize("Output directory").x; // the longer of the two
                    const float maxPathW     = 420.0f;
                    const float minPathW     = 220.0f;
                    const float budget       = m_leftPanelWidthPx - widestLabel - labelPadding - 20.0f;
                    const float pathW        = ImClamp(m_pathFieldTargetPx, minPathW, ImMin(maxPathW, budget));

                    ImGui::SetNextItemWidth(pathW);
                    ImGui::InputText("problem.yaml", &m_yamlPath);
                    ImGui::SameLine();
                    if (ImGui::Button("Browse")) ImGui::OpenPopup("PickYaml##cmp");
                    ui::widgets::FilePickerModal("PickYaml##cmp", m_yamlPath, {".yaml", ".yml"});

                    ImGui::SetNextItemWidth(pathW);
                    ImGui::InputText("Output directory", &m_outDirStr);
                    if (m_outDirStr.empty()) m_outDirStr = m_outDir.string();
                }

                ImGui::SeparatorText("Conversion options");
                ImGui::TextDisabled("Continuous to Roadmap (PRM*)");
                ImGui::InputInt   ("PRM* target samples", &m_prm.target_samples);
                ImGui::InputDouble("PRM* time budget (s)", &m_prm.time_sec);
                ImGui::InputFloat ("PRM* robot radius (m)", &m_prm.robot_radius);
                ImGui::Checkbox   ("Pin starts/goals", &m_prm.pin_starts_goals);
                ImGui::InputInt   ("Connect k", &m_prm.connect_k);
                ImGui::InputFloat ("Connect radius (m)", &m_prm.connect_radius);

                ImGui::TextDisabled("Continuous to Grid (raster)");
                ImGui::InputFloat ("Cell length h (m)  (<=0 to auto)", &m_ras.cell);
                ImGui::InputFloat ("Occ. threshold [0..1] (0=any-overlap)", &m_ras.occ_threshold);
                ImGui::InputInt   ("Supersample N (NxN)", &m_ras.supersample_px);
                ImGui::TextDisabled("Boundary wall thickness");
                ImGui::InputFloat ("Wall thickness (m)", &m_wallThickness);

                ImGui::SeparatorText("Planners");
                PlannerCombo(
                    "Continuous",
                    m_plannerCont,
                    platform::PlannerCatalog::allowed(
                        platform::EnvKind::Continuous));
                PlannerCombo(
                    "Roadmap",
                    m_plannerRoad,
                    platform::PlannerCatalog::allowed(
                        platform::EnvKind::Roadmap));
                PlannerCombo(
                    "Grid",
                    m_plannerGrid,
                    platform::PlannerCatalog::allowed(
                        platform::EnvKind::Grid));

                ImGui::SetNextItemWidth(160); ImGui::InputDouble("Time limit (s)", &m_timeLimitS);
                ImGui::SetNextItemWidth(160); ImGui::InputFloat ("Roadmap spawn radius (m)", &m_roadRobotRadius);

                ImGui::Spacing();
                if (ui::PrimaryButton("Run comparison")) { m_status.clear(); DoRun(); }
                ImGui::SameLine();
                if (ImGui::Button("Open results folder in log")) {
                    m_status = fmt::format(
                        "Results folder: {}", m_last.outputDirectory.string());
                }

                ImGui::Spacing();
                ImGui::SeparatorText("Status");
                ImGui::TextWrapped("%s", m_status.c_str());

                if (m_haveResults) {
                    ImGui::SeparatorText("Comparison");
                    ui::DrawMetricsComparisonTable(m_cont ? &*m_cont : nullptr, "Continuous",
                                                   m_road ? &*m_road : nullptr, "Roadmap",
                                                   m_grid ? &*m_grid : nullptr, "Grid");
                    ImGui::Spacing();
                    if (ImGui::Button("Export combined CSV")) {
                        const fs::path outDir = m_last.outputDirectory.empty()
                            ? fs::path(m_outDirStr)
                            : m_last.outputDirectory;
                        const fs::path csv    = outDir / "comparison_metrics.csv";
                        const auto exported =
                            platform::cross_compare::saveCsvComparison(
                                csv, m_cont ? &*m_cont : nullptr,
                                m_road ? &*m_road : nullptr,
                                m_grid ? &*m_grid : nullptr);
                        m_status = exported.ok
                            ? fmt::format("Wrote {}", csv.string())
                            : exported.failureMessage;
                    }
                    ImGui::Spacing();
                    if (ImGui::CollapsingHeader("Per agent (Continuous)")) ui::DrawPerAgentMetricsTable(*m_cont, 260.0f);
                    if (ImGui::CollapsingHeader("Per agent (Roadmap)"))    ui::DrawPerAgentMetricsTable(*m_road, 260.0f);
                    if (ImGui::CollapsingHeader("Per agent (Grid)"))       ui::DrawPerAgentMetricsTable(*m_grid, 260.0f);
                }

                ImGui::EndChild(); // left_panel

                // -------------------------- RIGHT COLUMN (stable triptych panel) -------------------
                ImGui::TableSetColumnIndex(1);
                ImGui::BeginChild("right_panel", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);

                ImGui::SeparatorText("Side by side visualization");
                ImGui::Checkbox("Show triptych", &m_showTriptych);
                ImGui::SameLine(); ImGui::Checkbox("Planned paths", &m_triptychDrawPaths);
                ImGui::SameLine(); ImGui::Checkbox("Roadmap graph", &m_triptychDrawRoad);
                ImGui::SameLine(); ImGui::Checkbox("Walls", &m_triptychDrawWalls);

                ImGui::SetNextItemWidth(200);
                ImGui::SliderInt("Preview height (px)", &m_triptychHeightPx, 120, 640);

                // Left panel width slider: we DO NOT call SetColumnWidth/Offset anymore.
                // Next frame the table will pick the new fixed size from m_leftPanelWidthPx.
                ImGui::SetNextItemWidth(200);
                ImGui::SliderFloat("Left panel width (px)", &m_leftPanelWidthPx, minLeft, maxLeft, "%.0f");

                if (m_showTriptych) {
                    // Compute stable tile width from the right child’s width (already excludes its scrollbar)
                    const float availW = ImGui::GetContentRegionAvail().x;
                    const float pad    = ImGui::GetStyle().ItemSpacing.x;
                    const float colW   = ImFloor((availW - 2.0f * pad) / 3.0f);
                    const int   Wpx    = (int)colW;
                    const int   Hpx    = m_triptychHeightPx;

                    frontend::render::PreviewOptions preview;
                    preview.widthPixels = Wpx;
                    preview.heightPixels = Hpx;
                    preview.drawPaths = m_triptychDrawPaths;
                    preview.drawWalls = m_triptychDrawWalls;
                    preview.wallThickness = m_wallThickness;
                    preview.marginFraction = m_triptychMarginFrac;
                    const ImGuiTableFlags tFlags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_BordersInnerV;
                    if (ImGui::BeginTable("Triptych3", 3, tFlags)) {
                        ImGui::TableSetupColumn("Continuous");
                        ImGui::TableSetupColumn("Roadmap");
                        ImGui::TableSetupColumn("Grid");
                        ImGui::TableHeadersRow();

                        ImGui::TableNextRow();

                        // Continuous
                        ImGui::TableSetColumnIndex(0);
                        if (m_envCont && m_simCont) {
                            (void)m_previewRenderer.renderContinuousPreview(
                                *m_envCont, m_simCont.get(), preview);
                            m_previewRenderer.drawContinuousPreview(
                                static_cast<float>(Wpx), static_cast<float>(Hpx));
                            ImGui::TextDisabled("%s", m_plannerCont.c_str());
                        } else { ImGui::Dummy(ImVec2((float)Wpx,(float)Hpx)); ImGui::TextDisabled("—"); }

                        // Roadmap
                        ImGui::TableSetColumnIndex(1);
                        if (m_envRoad && m_simRoad) {
                            preview.drawRoadmap = m_triptychDrawRoad;
                            (void)m_previewRenderer.renderRoadmapPreview(
                                *m_envRoad, m_simRoad.get(), preview);
                            m_previewRenderer.drawRoadmapPreview(
                                static_cast<float>(Wpx), static_cast<float>(Hpx));
                            ImGui::TextDisabled("%s", m_plannerRoad.c_str());
                        } else {
                            ImGui::Dummy(ImVec2((float)Wpx, (float)Hpx));
                            ImGui::TextDisabled("—");
                        }

                        // Grid
                        ImGui::TableSetColumnIndex(2);
                        if (m_envGrid && m_simGrid) {
                            preview.drawRoadmap = false;
                            (void)m_previewRenderer.renderGridPreview(
                                *m_envGrid, m_simGrid.get(), preview);
                            m_previewRenderer.drawGridPreview(
                                static_cast<float>(Wpx), static_cast<float>(Hpx));
                            ImGui::TextDisabled("%s", m_plannerGrid.c_str());
                        } else { ImGui::Dummy(ImVec2((float)Wpx,(float)Hpx)); ImGui::TextDisabled("—"); }

                        ImGui::EndTable();
                    }
                }

                ImGui::EndChild(); // right_panel
                ImGui::EndTable();
            }
            // --- end stable 2-column split ---

        }
        ImGui::End();
    }

private:
    static void PlannerCombo(
        const char* label,
        std::string& pick,
        const std::vector<std::string>& items)
    {
        std::size_t current = 0;
        for (std::size_t index = 0; index < items.size(); ++index) {
            if (pick == items[index]) {
                current = index;
                break;
            }
        }
        if (ImGui::BeginCombo(label, pick.c_str())) {
            for (std::size_t index = 0; index < items.size(); ++index) {
                const bool selected = index == current;
                if (ImGui::Selectable(items[index].c_str(), selected)) {
                    pick = items[index];
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    void DoRun()
    {
        using namespace tools::mc;

        m_haveResults = false;
        m_cont.reset(); m_road.reset(); m_grid.reset();
        m_last = {};

        // Reset previews
        m_envCont.reset(); m_envRoad.reset(); m_envGrid.reset();
        m_simCont.reset(); m_simRoad.reset(); m_simGrid.reset();

        // One immutable snapshot drives every conversion and continuous run.
        const auto problem = grace::sim::loadProblemYaml(m_yamlPath);
        platform::cross_compare::Options workflowOptions;
        workflowOptions.outputDirectory =
            m_outDirStr.empty() ? m_outDir : fs::path(m_outDirStr);
        workflowOptions.roadmap = m_prm;
        workflowOptions.raster = m_ras;
        workflowOptions.gridFormat = m_gridFmt;
        workflowOptions.wallThickness = m_wallThickness;
        const auto generated =
            platform::cross_compare::generateArtifacts(problem, workflowOptions);
        m_last = generated.artifacts;
        if (!generated.ok) {
            m_status = generated.failureMessage;
            return;
        }
        const auto& ras = generated.raster;

        // ===== Run planning on three reps =====
        // A) Continuous
        {
            m_envCont = std::make_unique<ContinuousEnvironment>();
            m_simCont = std::make_unique<Simulation>(*m_envCont);
            m_simCont->setPlanner(makePlanner(m_plannerCont));
            PlannerSettings opt; opt.set("timelimit_s", m_timeLimitS);
            m_simCont->configurePlanner(std::move(opt));

            if (!ops::spawnAndPlanFromProblemSpec(
                    *m_simCont,
                    *m_envCont,
                    problem,
                    m_wallThickness)) {
                m_status = "Continuous planning failed.";
                return;
            }
            m_cont = m_simCont->computePlanMetrics();
        }

        // B) Roadmap
        {
            m_envRoad = std::make_unique<RoadmapEnvironment>();
            grace::sim::GraphMLImport imp{};
            if (!grace::sim::readGraphML(
                    m_last.roadmapXml, *m_envRoad, &imp)) {
                m_status = "GraphML load failed for roadmap.xml"; return;
            }
            m_envRoad->addBoundaryWalls(m_wallThickness);

            m_simRoad = std::make_unique<Simulation>(*m_envRoad);
            m_simRoad->setPlanner(makePlanner(m_plannerRoad));
            PlannerSettings opt; opt.set("timelimit_s", m_timeLimitS);
            m_simRoad->configurePlanner(std::move(opt));

            // ⬇️ spawn goals only (no plan) from tasks.xml via ops::
            auto ids = ops::spawnFromTaskXmlNoPlan(
                                                   *m_simRoad, *m_envRoad, m_last.roadmapTasksXml, imp,
                                                   /*robotRadius=*/m_roadRobotRadius,
                                                   /*takeK=*/std::numeric_limits<std::size_t>::max(),
                                                   /*baseName=*/"r");
            if (ids.empty()) { m_status = "Roadmap spawn from tasks.xml produced no robots."; return; }
            (void)platform::planning::planPreservingGoals(
                *m_simRoad, m_simRoad->robots().robots());
            m_road = m_simRoad->computePlanMetrics();
        }

        // C) Grid
        {
            const auto map = m_gridFmt == GridFormat::MovingAI
                ? grace::io::readGridMovingAIMap(m_last.gridMap)
                : grace::io::readGridXmlMap(m_last.gridMap);
            if (!map) {
                m_status = map.error;
                return;
            }
            auto envGrid = std::make_unique<GridEnvironment>();
            envGrid->makeGridEnvironment(
                map.value->width, map.value->height, ras.cell,
                map.value->occupancy);
            envGrid->addBoundaryWalls(m_wallThickness);

            auto simGrid = std::make_unique<Simulation>(*envGrid);
            simGrid->setPlanner(makePlanner(m_plannerGrid));
            PlannerSettings opt; opt.set("timelimit_s", m_timeLimitS);
            simGrid->configurePlanner(std::move(opt));

            const auto scenario = grace::io::readGridMovingAIScenario(
                *envGrid, m_last.gridScenario);
            if (!scenario) {
                m_status = scenario.error;
                return;
            }
            const auto& specs = *scenario.value;

            grace::sim::SpawnOptions sopt;
            sopt.baseName    = "r";

            const auto ids = grace::sim::spawnGridAgent(*simGrid, specs, sopt);
            if (ids.empty()) { m_status = "Grid spawn from .scen produced no robots."; return; }
            (void)platform::planning::planPreservingGoals(
                *simGrid, simGrid->robots().robots());
            m_grid = simGrid->computePlanMetrics();
            m_envGrid = std::move(envGrid);
            m_simGrid = std::move(simGrid);
        }

        m_haveResults = true;
        m_status = fmt::format(
            "Done. Outputs under {}", m_last.outputDirectory.string());
    }

private:
    // Inputs
    std::string m_yamlPath;
    fs::path    m_outDir;
    std::string m_outDirStr;
    tools::mc::PRMOptions    m_prm{};
    tools::mc::RasterOptions m_ras{};
    tools::mc::GridFormat    m_gridFmt{};
    float  m_wallThickness = 0.25f;
    float  m_roadRobotRadius = 0.07f;

    // Planners
    std::string m_plannerCont, m_plannerRoad, m_plannerGrid;
    double      m_timeLimitS{45.0};

    // Results
    bool m_haveResults{false};
    std::optional<metrics::Aggregate> m_cont, m_road, m_grid;
    platform::cross_compare::ArtifactPaths m_last{};
    std::string  m_status;

    // Disable copy
    Sample_CrossCompare(const Sample_CrossCompare&) = delete;
    Sample_CrossCompare& operator=(const Sample_CrossCompare&) = delete;

    // --- Triptych UI state ---
    bool  m_showTriptych       = true;
    bool  m_triptychDrawPaths  = true;
    bool  m_triptychDrawRoad   = true;
    bool  m_triptychDrawWalls  = true;
    int   m_triptychHeightPx   = 260;
    float m_triptychMarginFrac = 0.06f;
    // Width controls
    float m_leftPanelWidthPx   = 700.0f; // width of the left column (right takes the rest)
    float m_minRightPanelPx    = 600.0f;  // guarantee enough room for triptych
    float m_pathFieldTargetPx  = 360.0f;  // compact path field target width

    // --- Keep env/sim alive for previews ---
    std::unique_ptr<ContinuousEnvironment>    m_envCont;
    std::unique_ptr<RoadmapEnvironment> m_envRoad;
    std::unique_ptr<GridEnvironment>    m_envGrid;
    std::unique_ptr<Simulation>     m_simCont, m_simRoad, m_simGrid;
    frontend::render::CrossComparisonRenderer m_previewRenderer;
};

// Registry
static Sample* CreateCrossCompare(SampleContext* ctx) { return new Sample_CrossCompare(ctx); }
static const int s_reg_cross = RegisterSample("MRMP", "Cross representation comparison", &CreateCrossCompare);
