// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "benchmarking/BenchmarkConfig.h"

#include "io/ContinuousYaml.h"
#include "io/GridLORR.h"
#include "io/GridMovingAI.h"
#include "io/GridXML.h"
#include "io/RoadmapXML.h"

#include <boost/program_options.hpp>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <pugixml.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace grace::benchmarking {
namespace {

namespace fs = std::filesystem;
namespace po = boost::program_options;
using json = nlohmann::json;

template <typename T>
void appendPaths(const T& values, std::vector<fs::path>& output)
{
    for (const auto& value : values) {
        output.emplace_back(value);
    }
}

void appendJsonPaths(
    const json& object, const char* key, std::vector<fs::path>& output)
{
    if (!object.contains(key)) {
        return;
    }
    const auto& values = object.at(key);
    if (!values.is_array()) {
        throw std::invalid_argument(
            fmt::format("Config field '{}' must be an array", key));
    }
    for (const auto& value : values) {
        output.emplace_back(value.get<std::string>());
    }
}

void appendJsonStrings(
    const json& object, const char* key, std::vector<std::string>& output)
{
    if (!object.contains(key)) {
        return;
    }
    const auto& values = object.at(key);
    if (!values.is_array()) {
        throw std::invalid_argument(
            fmt::format("Config field '{}' must be an array", key));
    }
    for (const auto& value : values) {
        output.push_back(value.get<std::string>());
    }
}

json loadJson(const fs::path& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open config: " + path.string());
    }
    json result;
    input >> result;
    return result;
}

std::vector<int> parseTakeKJson(const json& root)
{
    if (!root.contains("takeK")) {
        return {-1};
    }
    const auto& values = root.at("takeK");
    if (!values.is_array()) {
        throw std::invalid_argument(
            "Config field 'takeK' must be an array, such as [1, 2, \"all\"]");
    }
    std::vector<std::string> tokens;
    tokens.reserve(values.size());
    for (const auto& value : values) {
        if (value.is_number_integer()) {
            tokens.push_back(std::to_string(value.get<int>()));
        } else if (value.is_string()) {
            tokens.push_back(value.get<std::string>());
        } else {
            throw std::invalid_argument(
                "Every takeK value must be a positive integer or \"all\"");
        }
    }
    return parseTakeKTokens(tokens);
}

void applyJsonConfig(const json& root, BenchmarkConfig& config)
{
    if (root.contains("grid")) {
        const auto& grid = root.at("grid");
        if (!grid.is_object()) {
            throw std::invalid_argument("Config field 'grid' must be an object");
        }
        appendJsonPaths(grid, "maps", config.gridMaps);
        appendJsonPaths(grid, "scens", config.gridScenarios);
        appendJsonPaths(grid, "agents", config.gridAgents);
        appendJsonPaths(grid, "tasks", config.gridTasks);
        config.gridCellLength =
            grid.value("grid_cell", config.gridCellLength);
        config.gridBoundaryScale =
            grid.value("grid_boundary_scale", config.gridBoundaryScale);
        config.gridSkipBlockedGoals =
            grid.value("skip_blocked_goals", config.gridSkipBlockedGoals);
    }
    if (root.contains("roadmap")) {
        const auto& roadmap = root.at("roadmap");
        if (!roadmap.is_object()) {
            throw std::invalid_argument(
                "Config field 'roadmap' must be an object");
        }
        appendJsonPaths(roadmap, "graphs", config.roadmapGraphs);
        appendJsonPaths(roadmap, "tasks", config.roadmapTasks);
        config.roadmapWall =
            roadmap.value("roadmap_wall", config.roadmapWall);
        config.roadmapRobotRadius =
            roadmap.value("roadmap_rr", config.roadmapRobotRadius);
    }
    if (root.contains("continuous")) {
        const auto& continuous = root.at("continuous");
        if (!continuous.is_object()) {
            throw std::invalid_argument(
                "Config field 'continuous' must be an object");
        }
        appendJsonPaths(
            continuous, "problems", config.continuousProblems);
        config.continuousWall =
            continuous.value("cont_wall", config.continuousWall);
    }

    appendJsonStrings(root, "solvers", config.solvers);
    config.takeK = parseTakeKJson(root);
    config.outputRoot = root.value("out_dir", config.outputRoot.string());
    config.repeat = root.value("repeat", config.repeat);
    config.updateDt = root.value("dt", config.updateDt);
    config.substeps = root.value("substeps", config.substeps);
    config.writeStateCsv = root.value("state_csv", config.writeStateCsv);
    config.stateEveryN = root.value("state_every", config.stateEveryN);
    config.sessionTag = root.value("session", config.sessionTag);
}

std::size_t availableAgents(BenchEnv environment, const fs::path& path)
{
    std::error_code error;
    if (!fs::is_regular_file(path, error) || error) {
        throw std::runtime_error(
            "Benchmark input does not exist or is not a file: " +
            path.string());
    }

    std::size_t count = 0;
    switch (environment) {
        case BenchEnv::Grid:
            if (path.extension() == ".scen") {
                const auto scenario =
                    grace::io::detail::parseMovingAIScenario(path);
                if (!scenario) {
                    throw std::runtime_error(scenario.error);
                }
                count = scenario.value->size();
            } else if (path.extension() == ".xml") {
                const auto tasks =
                    grace::io::detail::parseGridXmlTask(path);
                if (!tasks) {
                    throw std::runtime_error(tasks.error);
                }
                count = tasks.value->size();
            } else {
                count =
                    grace::io::detail::readGridLORRIntList(path).size();
            }
            break;
        case BenchEnv::Roadmap:
        {
            pugi::xml_document document;
            const auto result = document.load_file(path.string().c_str());
            if (!result) {
                throw std::runtime_error(
                    "Failed to read roadmap tasks '" + path.string() +
                    "': " + result.description());
            }
            auto root = document.child("root");
            if (!root) root = document.first_child();
            for ([[maybe_unused]] const auto agent :
                 root.children("agent")) {
                ++count;
            }
            break;
        }
        case BenchEnv::Continuous:
            count = grace::sim::loadProblemYaml(path).robots.size();
            break;
    }
    if (count == 0) {
        throw std::runtime_error(
            "Benchmark input contains no agents: " + path.string());
    }
    return count;
}

RunOpts commonRun(
    const BenchmarkConfig& config,
    BenchEnv environment,
    const std::string& solver,
    std::size_t takeK,
    int repetition,
    fs::path runDirectory,
    const fs::path& masterCsv)
{
    RunOpts options;
    options.env = environment;
    options.solverName = solver;
    options.takeK = takeK;
    options.repetition = repetition;
    options.runDirectory = std::move(runDirectory);
    options.masterCsv = masterCsv;
    options.sessionTag = config.sessionTag;
    options.writeStateCsv = config.writeStateCsv;
    options.stateEveryN = config.stateEveryN;
    options.updateDt = config.updateDt;
    options.substeps = config.substeps;
    return options;
}

} // namespace

std::vector<int> parseTakeKTokens(const std::vector<std::string>& tokens)
{
    std::vector<int> result;
    for (std::string token : tokens) {
        std::transform(
            token.begin(), token.end(), token.begin(),
            [](unsigned char value) {
                return static_cast<char>(std::tolower(value));
            });
        int value = -1;
        if (token != "all") {
            std::size_t parsed = 0;
            value = std::stoi(token, &parsed);
            if (parsed != token.size() || value <= 0) {
                throw std::invalid_argument(
                    "takeK values must be positive integers or \"all\"");
            }
        }
        if (std::find(result.begin(), result.end(), value) == result.end()) {
            result.push_back(value);
        }
    }
    if (result.empty()) {
        result.push_back(-1);
    }
    return result;
}

std::size_t effectiveTakeK(
    const int requested, const std::size_t available) noexcept
{
    return requested < 0
        ? std::numeric_limits<std::size_t>::max()
        : std::min<std::size_t>(static_cast<std::size_t>(requested), available);
}

std::string takeKFolderTag(
    const int requested, const std::size_t effective)
{
    return requested < 0 ? "all" : std::to_string(effective);
}

std::string pathStem(const fs::path& path)
{
    std::string stem = path.stem().string();
    std::replace(stem.begin(), stem.end(), ',', '_');
    std::replace(stem.begin(), stem.end(), ' ', '_');
    return stem;
}

std::string makeSessionTag()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &time);
#else
    localtime_r(&time, &local);
#endif
    std::ostringstream output;
    output << std::put_time(&local, "%Y%m%d_%H%M%S");
    return output.str();
}

BenchmarkCommandLine parseBenchmarkCommandLine(
    const std::vector<std::string>& arguments)
{
    BenchmarkConfig config;
    std::string configPath;
    std::vector<std::string> maps;
    std::vector<std::string> scenarios;
    std::vector<std::string> agents;
    std::vector<std::string> gridTasks;
    std::vector<std::string> graphs;
    std::vector<std::string> roadmapTasks;
    std::vector<std::string> problems;
    std::vector<std::string> solvers;
    std::vector<std::string> takeK;
    std::string outputRoot = config.outputRoot.string();
    std::string session;
    int repeat = config.repeat;
    float dt = config.updateDt;
    int substeps = config.substeps;
    bool stateCsv = config.writeStateCsv;
    int stateEvery = config.stateEveryN;
    float gridCell = config.gridCellLength;
    float gridBoundaryScale = config.gridBoundaryScale;
    bool skipBlocked = config.gridSkipBlockedGoals;
    float roadmapWall = config.roadmapWall;
    float roadmapRadius = config.roadmapRobotRadius;
    float continuousWall = config.continuousWall;

    po::options_description options(
        "Experiment options (grouped JSON configuration)");
    options.add_options()
        ("help,h", "Show help")
        ("config", po::value<std::string>(&configPath), "JSON config file")
        ("map", po::value<std::vector<std::string>>(&maps)->multitoken(),
         "Grid maps (.map or .xml)")
        ("scen", po::value<std::vector<std::string>>(&scenarios)->multitoken(),
         "MovingAI scenarios paired with maps")
        ("agents", po::value<std::vector<std::string>>(&agents)->multitoken(),
         "LORR .agents files paired with maps")
        ("grid_tasks",
         po::value<std::vector<std::string>>(&gridTasks)->multitoken(),
         "LORR .tasks files paired with maps and agents")
        ("grid_cell", po::value<float>(&gridCell)->default_value(gridCell),
         "Grid cell length in meters")
        ("grid_boundary_scale",
         po::value<float>(&gridBoundaryScale)->default_value(gridBoundaryScale),
         "Boundary wall scale relative to a grid cell")
        ("grid_skip_blocked_goals",
         po::value<bool>(&skipBlocked)->default_value(skipBlocked)
             ->implicit_value(true),
         "Skip blocked goals in LORR tasks")
        ("graph", po::value<std::vector<std::string>>(&graphs)->multitoken(),
         "Roadmap GraphML files")
        ("tasks",
         po::value<std::vector<std::string>>(&roadmapTasks)->multitoken(),
         "Roadmap task XML files")
        ("roadmap_wall",
         po::value<float>(&roadmapWall)->default_value(roadmapWall),
         "Roadmap boundary wall thickness")
        ("roadmap_rr",
         po::value<float>(&roadmapRadius)->default_value(roadmapRadius),
         "Roadmap robot radius")
        ("problem",
         po::value<std::vector<std::string>>(&problems)->multitoken(),
         "Continuous problem YAML files")
        ("cont_wall",
         po::value<float>(&continuousWall)->default_value(continuousWall),
         "Continuous boundary wall thickness")
        ("solver", po::value<std::vector<std::string>>(&solvers)->multitoken(),
         "Planner names")
        ("out_dir", po::value<std::string>(&outputRoot)
             ->default_value(outputRoot), "Output root")
        ("takeK", po::value<std::vector<std::string>>(&takeK)->multitoken(),
         "Agent counts; use all for all agents")
        ("repeat", po::value<int>(&repeat)->default_value(repeat), "Repeats")
        ("dt", po::value<float>(&dt)->default_value(dt), "Simulation dt")
        ("substeps", po::value<int>(&substeps)->default_value(substeps),
         "Box2D substeps")
        ("state_csv", po::value<bool>(&stateCsv)->default_value(stateCsv)
             ->implicit_value(true), "Write state CSV")
        ("state_every", po::value<int>(&stateEvery)->default_value(stateEvery),
         "State CSV interval")
        ("session", po::value<std::string>(&session)->default_value(session),
         "Session tag");

    po::variables_map values;
    po::store(po::command_line_parser(arguments).options(options).run(), values);
    po::notify(values);

    std::ostringstream help;
    help << options;
    if (values.count("help") != 0U) {
        return {std::move(config), true, help.str()};
    }

    if (!configPath.empty()) {
        applyJsonConfig(loadJson(configPath), config);
    }

    appendPaths(maps, config.gridMaps);
    appendPaths(scenarios, config.gridScenarios);
    appendPaths(agents, config.gridAgents);
    appendPaths(gridTasks, config.gridTasks);
    appendPaths(graphs, config.roadmapGraphs);
    appendPaths(roadmapTasks, config.roadmapTasks);
    appendPaths(problems, config.continuousProblems);
    config.solvers.insert(config.solvers.end(), solvers.begin(), solvers.end());

    if (!takeK.empty()) config.takeK = parseTakeKTokens(takeK);
    if (!values["out_dir"].defaulted()) config.outputRoot = outputRoot;
    if (!values["repeat"].defaulted()) config.repeat = repeat;
    if (!values["dt"].defaulted()) config.updateDt = dt;
    if (!values["substeps"].defaulted()) config.substeps = substeps;
    if (!values["state_csv"].defaulted()) config.writeStateCsv = stateCsv;
    if (!values["state_every"].defaulted()) config.stateEveryN = stateEvery;
    if (!values["session"].defaulted()) config.sessionTag = session;
    if (!values["grid_cell"].defaulted()) config.gridCellLength = gridCell;
    if (!values["grid_boundary_scale"].defaulted()) {
        config.gridBoundaryScale = gridBoundaryScale;
    }
    if (!values["grid_skip_blocked_goals"].defaulted()) {
        config.gridSkipBlockedGoals = skipBlocked;
    }
    if (!values["roadmap_wall"].defaulted()) config.roadmapWall = roadmapWall;
    if (!values["roadmap_rr"].defaulted()) {
        config.roadmapRobotRadius = roadmapRadius;
    }
    if (!values["cont_wall"].defaulted()) {
        config.continuousWall = continuousWall;
    }

    return {std::move(config), false, help.str()};
}

void validateBenchmarkConfig(const BenchmarkConfig& config)
{
    const bool gridScenario = !config.gridScenarios.empty();
    if (gridScenario &&
        config.gridMaps.size() != config.gridScenarios.size()) {
        throw std::invalid_argument(
            "Grid maps and scenarios must have the same length");
    }
    const bool gridLorr =
        !config.gridAgents.empty() || !config.gridTasks.empty();
    if (gridLorr &&
        (config.gridMaps.size() != config.gridAgents.size() ||
         config.gridMaps.size() != config.gridTasks.size())) {
        throw std::invalid_argument(
            "Grid maps, agents, and grid tasks must have the same length");
    }
    if (!config.gridMaps.empty() && !gridScenario && !gridLorr) {
        throw std::invalid_argument(
            "Grid maps require scenarios or agents/tasks");
    }
    if (config.roadmapGraphs.size() != config.roadmapTasks.size()) {
        throw std::invalid_argument(
            "Roadmap graphs and tasks must have the same length");
    }
    if (config.gridMaps.empty() && config.roadmapGraphs.empty() &&
        config.continuousProblems.empty()) {
        throw std::invalid_argument("No benchmark instances were provided");
    }
    if (config.solvers.empty()) {
        throw std::invalid_argument("No benchmark solvers were provided");
    }
    if (config.repeat < 1 || config.substeps < 1 ||
        config.stateEveryN < 1 ||
        !std::isfinite(config.updateDt) || config.updateDt <= 0.0f ||
        !std::isfinite(config.gridCellLength) ||
        config.gridCellLength <= 0.0f ||
        !std::isfinite(config.gridBoundaryScale) ||
        config.gridBoundaryScale <= 0.0f ||
        !std::isfinite(config.roadmapRobotRadius) ||
        config.roadmapRobotRadius <= 0.0f) {
        throw std::invalid_argument("Benchmark numeric settings are invalid");
    }
}

std::vector<RunOpts> makeBenchmarkSchedule(BenchmarkConfig config)
{
    validateBenchmarkConfig(config);
    if (config.sessionTag.empty()) {
        config.sessionTag = makeSessionTag();
    }
    const fs::path sessionRoot = config.outputRoot / config.sessionTag;
    const fs::path masterCsv = sessionRoot / "all_runs.csv";
    std::vector<RunOpts> schedule;

    for (const int requested : config.takeK) {
        if (!config.gridScenarios.empty()) {
            for (std::size_t i = 0; i < config.gridMaps.size(); ++i) {
                const std::size_t effective = effectiveTakeK(
                    requested,
                    availableAgents(BenchEnv::Grid, config.gridScenarios[i]));
                for (const auto& solver : config.solvers) {
                    for (int repetition = 0;
                         repetition < config.repeat; ++repetition) {
                        const fs::path runDir = sessionRoot / "grid" /
                            pathStem(config.gridMaps[i]) /
                            pathStem(config.gridScenarios[i]) / solver /
                            ("k" + takeKFolderTag(requested, effective)) /
                            fmt::format("rep{}", repetition);
                        auto run = commonRun(
                            config, BenchEnv::Grid, solver, effective,
                            repetition, runDir, masterCsv);
                        run.gridMap = config.gridMaps[i];
                        run.gridScenario = config.gridScenarios[i];
                        run.gridCellLength = config.gridCellLength;
                        run.gridBoundaryScale = config.gridBoundaryScale;
                        run.mapLabel = run.gridMap;
                        run.scenarioLabel = run.gridScenario;
                        schedule.push_back(std::move(run));
                    }
                }
            }
        }

        if (!config.gridAgents.empty()) {
            for (std::size_t i = 0; i < config.gridMaps.size(); ++i) {
                const std::size_t effective = effectiveTakeK(
                    requested,
                    availableAgents(BenchEnv::Grid, config.gridAgents[i]));
                for (const auto& solver : config.solvers) {
                    for (int repetition = 0;
                         repetition < config.repeat; ++repetition) {
                        const fs::path runDir = sessionRoot / "grid" /
                            pathStem(config.gridMaps[i]) /
                            (pathStem(config.gridAgents[i]) + "__" +
                             pathStem(config.gridTasks[i])) / solver /
                            ("k" + takeKFolderTag(requested, effective)) /
                            fmt::format("rep{}", repetition);
                        auto run = commonRun(
                            config, BenchEnv::Grid, solver, effective,
                            repetition, runDir, masterCsv);
                        run.gridMap = config.gridMaps[i];
                        run.gridAgents = config.gridAgents[i];
                        run.gridTasks = config.gridTasks[i];
                        run.gridCellLength = config.gridCellLength;
                        run.gridBoundaryScale = config.gridBoundaryScale;
                        run.gridSkipBlockedGoals =
                            config.gridSkipBlockedGoals;
                        run.mapLabel = run.gridMap;
                        run.scenarioLabel = run.gridTasks;
                        schedule.push_back(std::move(run));
                    }
                }
            }
        }

        for (std::size_t i = 0; i < config.roadmapGraphs.size(); ++i) {
            const std::size_t effective = effectiveTakeK(
                requested,
                availableAgents(BenchEnv::Roadmap, config.roadmapTasks[i]));
            for (const auto& solver : config.solvers) {
                for (int repetition = 0;
                     repetition < config.repeat; ++repetition) {
                    const fs::path runDir = sessionRoot / "roadmap" /
                        pathStem(config.roadmapGraphs[i]) /
                        pathStem(config.roadmapTasks[i]) / solver /
                        ("k" + takeKFolderTag(requested, effective)) /
                        fmt::format("rep{}", repetition);
                    auto run = commonRun(
                        config, BenchEnv::Roadmap, solver, effective,
                        repetition, runDir, masterCsv);
                    run.graphXml = config.roadmapGraphs[i];
                    run.tasksXml = config.roadmapTasks[i];
                    run.roadmapWall = config.roadmapWall;
                    run.roadmapRobotRadius = config.roadmapRobotRadius;
                    run.mapLabel = run.graphXml;
                    run.scenarioLabel = run.tasksXml;
                    schedule.push_back(std::move(run));
                }
            }
        }

        for (const auto& problem : config.continuousProblems) {
            const std::size_t effective = effectiveTakeK(
                requested,
                availableAgents(BenchEnv::Continuous, problem));
            for (const auto& solver : config.solvers) {
                for (int repetition = 0;
                     repetition < config.repeat; ++repetition) {
                    const fs::path runDir = sessionRoot / "continuous" /
                        pathStem(problem) / solver /
                        ("k" + takeKFolderTag(requested, effective)) /
                        fmt::format("rep{}", repetition);
                    auto run = commonRun(
                        config, BenchEnv::Continuous, solver, effective,
                        repetition, runDir, masterCsv);
                    run.problemYaml = problem;
                    run.continuousWall = config.continuousWall;
                    run.mapLabel = problem;
                    run.scenarioLabel = "-";
                    schedule.push_back(std::move(run));
                }
            }
        }
    }
    return schedule;
}

} // namespace grace::benchmarking
