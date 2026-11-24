// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "app/SimulationApp.h"
#include "util/RunOutput.h"

#include "Simulation.h"
#include "env/BaseEnvironment.h"
#include "env/ContinuousEnvironment.h"
#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "io/GridMovingAI.h"
#include "io/MetricsCsvObserver.h"
#include "io/RoadmapXML.h"
#include "io/SimContinuousYamlOps.h"
#include "io/SimRoadmapOps.h"
#include "io/StateCsvObserver.h"
#include "planner/plugin/PlannerFactory.h"
#include "sim/SpawnRobot.h"

#include <boost/program_options.hpp>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace grace::app {
namespace {

namespace fs = std::filesystem;
namespace po = boost::program_options;

std::string lowerCopy(const std::string_view value)
{
    std::string lowered(value);
    std::transform(
        lowered.begin(), lowered.end(), lowered.begin(),
        [](const unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return lowered;
}

void requireInputFile(const fs::path& path, const std::string_view label)
{
    std::error_code error;
    const bool isFile = fs::is_regular_file(path, error);
    if (error || !isFile) {
        throw std::runtime_error(
            std::string(label) + " does not exist or is not a file: " +
            path.string());
    }
}

SimulationExitCode finishSimulation(Simulation& simulation, const bool planned)
{
    if (!planned) {
        return SimulationExitCode::PlanningFailed;
    }

    simulation.run();
    return simulation.allAgentsReachedGoals()
        ? SimulationExitCode::Success
        : SimulationExitCode::GoalsNotReached;
}

SimulationExitCode runGrid(const SimulationRunConfig& config)
{
    requireInputFile(config.mapPath, "Grid map");
    requireInputFile(config.scenarioPath, "Grid scenario");

    GridEnvironment environment;
    const auto map = grace::io::readGridMovingAIMap(config.mapPath);
    if (!map) {
        throw std::runtime_error(map.error);
    }
    environment.makeGridEnvironment(
        map.value->width, map.value->height, config.cellLength,
        map.value->occupancy);

    Simulation simulation(environment);
    configureSimulation(simulation, config);

    const auto scenario = grace::io::readGridMovingAIScenario(
        environment, config.scenarioPath, config.takeK);
    if (!scenario) {
        throw std::runtime_error(scenario.error);
    }
    const auto& startGoals = *scenario.value;
    if (startGoals.empty()) {
        throw std::runtime_error(
            "Grid scenario contains no usable start/goal entries: " +
            config.scenarioPath.string());
    }

    grace::sim::SpawnOptions spawnOptions;
    spawnOptions.baseName = config.robotBaseName;
    const auto ids =
        grace::sim::spawnGridAgent(simulation, startGoals, spawnOptions);
    return finishSimulation(
        simulation, simulation.plan(ids, startGoals));
}

SimulationExitCode runRoadmap(const SimulationRunConfig& config)
{
    requireInputFile(config.mapPath, "Roadmap graph");
    requireInputFile(config.tasksPath, "Roadmap tasks");

    RoadmapEnvironment environment;
    grace::sim::GraphMLImport import;
    if (!grace::sim::readGraphML(config.mapPath, environment, &import)) {
        throw std::runtime_error(
            "Failed to read roadmap graph: " + config.mapPath.string());
    }
    environment.addBoundaryWalls(config.wallThickness);

    Simulation simulation(environment);
    configureSimulation(simulation, config);
    const bool planned = grace::sim::ops::spawnAndPlanFromTaskXml(
        simulation, environment, config.tasksPath, import,
        config.robotRadius, config.takeK, config.robotBaseName);
    return finishSimulation(simulation, planned);
}

SimulationExitCode runContinuous(const SimulationRunConfig& config)
{
    requireInputFile(config.problemPath, "Continuous problem");

    ContinuousEnvironment environment;
    Simulation simulation(environment);
    configureSimulation(simulation, config);
    const bool planned = grace::sim::ops::spawnAndPlanFromProblemYaml(
        simulation, environment, config.problemPath, config.wallThickness,
        config.takeK, config.robotBaseName);
    return finishSimulation(simulation, planned);
}

} // namespace

SimulationMode parseSimulationMode(const std::string_view value)
{
    const std::string lowered = lowerCopy(value);
    if (lowered == "grid") {
        return SimulationMode::Grid;
    }
    if (lowered == "roadmap" || lowered == "road") {
        return SimulationMode::Roadmap;
    }
    if (lowered == "continuous" || lowered == "cont") {
        return SimulationMode::Continuous;
    }
    throw std::invalid_argument(
        "Unknown simulation mode '" + std::string(value) +
        "' (expected grid, roadmap, or continuous)");
}

std::string_view simulationModeName(const SimulationMode mode) noexcept
{
    switch (mode) {
        case SimulationMode::Grid:
            return "grid";
        case SimulationMode::Roadmap:
            return "roadmap";
        case SimulationMode::Continuous:
            return "continuous";
    }
    return "unknown";
}

SimulationRunConfig defaultSimulationRunConfig(
    const SimulationMode mode,
    const fs::path& dataRoot)
{
    SimulationRunConfig config;
    config.mode = mode;

    switch (mode) {
        case SimulationMode::Grid:
            config.mapPath =
                dataRoot / "maps/movingAI/default/demo.map";
            config.scenarioPath =
                dataRoot / "maps/movingAI/default/demo.scen";
            config.planner = "lacam0-src";
            break;
        case SimulationMode::Roadmap:
            config.mapPath =
                dataRoot / "maps/xml/default/roadmap/demo_roadmap.xml";
            config.tasksPath =
                dataRoot / "maps/xml/default/roadmap/demo_tasks.xml";
            config.planner = "psippctc-src";
            config.wallThickness = 1.0f;
            break;
        case SimulationMode::Continuous:
            config.problemPath =
                dataRoot / "maps/yaml/default/demo.yaml";
            config.planner = "dbcbs-src";
            break;
    }

    return config;
}

void validateSimulationRunConfig(const SimulationRunConfig& config)
{
    if (config.planner.empty()) {
        throw std::invalid_argument("Planner name must not be empty");
    }
    if (config.resultsDirectory.empty()) {
        throw std::invalid_argument("Results directory must not be empty");
    }
    if (config.robotBaseName.empty()) {
        throw std::invalid_argument("Robot base name must not be empty");
    }
    if (!(config.cellLength > 0.0f)) {
        throw std::invalid_argument("Cell length must be greater than zero");
    }
    if (!(config.robotRadius > 0.0f)) {
        throw std::invalid_argument("Robot radius must be greater than zero");
    }
    if (!(config.wallThickness > 0.0f)) {
        throw std::invalid_argument(
            "Wall thickness must be greater than zero");
    }
    if (!(config.updateDt > 0.0f)) {
        throw std::invalid_argument("Simulation dt must be greater than zero");
    }
    if (config.subStepCount < 1) {
        throw std::invalid_argument(
            "Simulation substep count must be at least one");
    }
    if (config.stateEveryNSteps < 1) {
        throw std::invalid_argument(
            "State CSV interval must be at least one");
    }
}

SimulationCommandLine parseSimulationCommandLine(
    const std::vector<std::string>& arguments,
    const fs::path& dataRoot)
{
    po::options_description options("Options");
    options.add_options()
        ("help,h", "Show this help")
        ("mode,m", po::value<std::string>()->default_value("grid"),
         "Simulation mode: grid | roadmap | continuous")
        ("map", po::value<std::string>(),
         "Grid .map or roadmap GraphML input")
        ("scenario", po::value<std::string>(),
         "MovingAI .scen input for grid mode")
        ("tasks", po::value<std::string>(),
         "tasks.xml input for roadmap mode")
        ("problem", po::value<std::string>(),
         "problem YAML input for continuous mode")
        ("planner,p", po::value<std::string>(), "Planner name")
        ("results-dir", po::value<std::string>(),
         "Directory for metrics and state CSV files")
        ("take-k", po::value<int>()->default_value(-1),
         "Number of agents to use; -1 means all")
        ("robot-base", po::value<std::string>(), "Robot name prefix")
        ("cell-length", po::value<float>(), "Grid cell length in meters")
        ("robot-radius", po::value<float>(),
         "Roadmap robot radius in meters")
        ("wall-thickness", po::value<float>(),
         "Boundary wall thickness in meters")
        ("dt", po::value<float>(), "Simulation time step in seconds")
        ("substeps", po::value<int>(), "Box2D substeps per simulation step")
        ("state-every", po::value<int>(),
         "Write state CSV every N simulation steps")
        ("no-metrics", po::bool_switch(),
         "Disable metrics_summary.csv")
        ("no-state", po::bool_switch(), "Disable state_trace.csv")
        ("append", po::bool_switch(),
         "Append to existing CSV files instead of overwriting");

    po::positional_options_description positional;
    positional.add("mode", 1);

    po::variables_map values;
    try {
        po::store(
            po::command_line_parser(arguments)
                .options(options)
                .positional(positional)
                .run(),
            values);
        po::notify(values);
    } catch (const po::error& error) {
        throw std::invalid_argument(
            "Invalid backend arguments: " + std::string(error.what()));
    }

    SimulationCommandLine result;
    result.showHelp = values.count("help") != 0;

    std::ostringstream help;
    help << "Usage: grace-backend [grid|roadmap|continuous] [options]\n\n"
         << options;
    result.helpText = help.str();

    const SimulationMode mode =
        parseSimulationMode(values["mode"].as<std::string>());
    result.config = defaultSimulationRunConfig(mode, dataRoot);

    auto setPath = [&](const char* name, fs::path& destination) {
        if (values.count(name) != 0) {
            destination = values[name].as<std::string>();
        }
    };
    setPath("map", result.config.mapPath);
    setPath("scenario", result.config.scenarioPath);
    setPath("tasks", result.config.tasksPath);
    setPath("problem", result.config.problemPath);
    setPath("results-dir", result.config.resultsDirectory);

    if (values.count("planner") != 0) {
        result.config.planner = values["planner"].as<std::string>();
    }
    if (values.count("robot-base") != 0) {
        result.config.robotBaseName =
            values["robot-base"].as<std::string>();
    }
    if (values.count("cell-length") != 0) {
        result.config.cellLength = values["cell-length"].as<float>();
    }
    if (values.count("robot-radius") != 0) {
        result.config.robotRadius = values["robot-radius"].as<float>();
    }
    if (values.count("wall-thickness") != 0) {
        result.config.wallThickness =
            values["wall-thickness"].as<float>();
    }
    if (values.count("dt") != 0) {
        result.config.updateDt = values["dt"].as<float>();
    }
    if (values.count("substeps") != 0) {
        result.config.subStepCount = values["substeps"].as<int>();
    }
    if (values.count("state-every") != 0) {
        result.config.stateEveryNSteps =
            values["state-every"].as<int>();
    }

    const int requestedAgents = values["take-k"].as<int>();
    if (requestedAgents == -1) {
        result.config.takeK = std::numeric_limits<std::size_t>::max();
    } else if (requestedAgents > 0) {
        result.config.takeK =
            static_cast<std::size_t>(requestedAgents);
    } else {
        throw std::invalid_argument(
            "--take-k must be -1 or a positive integer");
    }

    result.config.writeMetricsCsv = !values["no-metrics"].as<bool>();
    result.config.writeStateCsv = !values["no-state"].as<bool>();
    result.config.overwriteOutputs = !values["append"].as<bool>();

    validateSimulationRunConfig(result.config);
    return result;
}

void configureSimulation(
    Simulation& simulation,
    const SimulationRunConfig& config)
{
    validateSimulationRunConfig(config);
    fs::create_directories(config.resultsDirectory);

    simulation.setDt(config.updateDt);
    simulation.setSubStepCount(config.subStepCount);

    if (config.writeMetricsCsv) {
        simulation.addObserver(
            std::make_shared<grace::sim::MetricsCsvObserver>(
                config.resultsDirectory / "metrics_summary.csv",
                config.overwriteOutputs));
    }
    if (config.writeStateCsv) {
        simulation.addObserver(
            std::make_shared<grace::sim::StateCsvObserver>(
                config.resultsDirectory / "state_trace.csv",
                config.overwriteOutputs,
                config.stateEveryNSteps));
    }

    simulation.setPlanner(makePlanner(config.planner));
}

SimulationExitCode runSimulation(const SimulationRunConfig& config)
{
    validateSimulationRunConfig(config);
    grace::output::RunOutputConfig outputConfig;
    outputConfig.baseDirectory = config.resultsDirectory;
    (void)grace::output::startRunOutput(outputConfig);

    switch (config.mode) {
        case SimulationMode::Grid:
            return runGrid(config);
        case SimulationMode::Roadmap:
            return runRoadmap(config);
        case SimulationMode::Continuous:
            return runContinuous(config);
    }
    throw std::logic_error("Unhandled simulation mode");
}

int runSimulationMain(
    const int argc,
    char* argv[],
    const fs::path& dataRoot,
    std::ostream& output,
    std::ostream& error)
{
    std::vector<std::string> arguments;
    arguments.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0U);
    for (int index = 1; index < argc; ++index) {
        arguments.emplace_back(argv[index]);
    }

    try {
        const auto commandLine =
            parseSimulationCommandLine(arguments, dataRoot);
        if (commandLine.showHelp) {
            output << commandLine.helpText;
            return static_cast<int>(SimulationExitCode::Success);
        }

        return static_cast<int>(runSimulation(commandLine.config));
    } catch (const std::invalid_argument& exception) {
        error << exception.what() << '\n';
        return static_cast<int>(SimulationExitCode::InvalidArguments);
    } catch (const std::exception& exception) {
        error << "Backend simulation failed: " << exception.what() << '\n';
        return static_cast<int>(SimulationExitCode::RuntimeError);
    }
}

} // namespace grace::app
