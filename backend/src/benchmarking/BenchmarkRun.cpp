// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "benchmarking/BenchmarkRun.h"
#include "util/RunOutput.h"

#include "Simulation.h"
#include "env/BaseEnvironment.h"
#include "env/ContinuousEnvironment.h"
#include "env/GridEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "io/GridLORR.h"
#include "io/GridMovingAI.h"
#include "io/GridXML.h"
#include "io/RoadmapXML.h"
#include "io/SimContinuousYamlOps.h"
#include "io/SimRoadmapOps.h"
#include "io/StateCsvObserver.h"
#include "planner/plugin/PlannerFactory.h"
#include "sim/SpawnRobot.h"
#include "util/Logger.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <unistd.h>

namespace grace::benchmarking {
namespace {

namespace fs = std::filesystem;

void ensureParent(const fs::path& path)
{
    std::error_code error;
    fs::create_directories(path.parent_path(), error);
    if (error) {
        throw std::runtime_error(
            "Failed to create directory '" + path.parent_path().string() +
            "': " + error.message());
    }
}

double finiteOrZero(const double value)
{
    return std::isnan(value) ? 0.0 : value;
}

void writeMetrics(const fs::path& path, const metrics::Aggregate& metrics)
{
    ensureParent(path);
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        throw std::runtime_error("Failed to write " + path.string());
    }
    output << "agents,plan_success,planner_runtime_s,sim_runtime_s,"
              "makespan_s,sum_of_cost_s,sum_length_m,avg_completion_s,"
              "avg_length_m\n"
           << std::fixed << std::setprecision(6)
           << metrics.agents << ','
           << (metrics.plan_success ? 1 : 0) << ','
           << finiteOrZero(metrics.planner_runtime_s) << ','
           << finiteOrZero(metrics.sim_runtime_s) << ','
           << finiteOrZero(metrics.makespan_s) << ','
           << finiteOrZero(metrics.sum_of_cost_s) << ','
           << metrics.sum_length_m << ','
           << finiteOrZero(metrics.avg_completion_s) << ','
           << finiteOrZero(metrics.avg_length_m) << '\n';
}

void writeMetadata(
    const RunOpts& options,
    const metrics::Aggregate& metrics,
    const std::string_view environment,
    const double peakCpu,
    const double peakMemory)
{
    const fs::path path = options.runDirectory / "run_meta.json";
    ensureParent(path);
    nlohmann::json metadata{
        {"env", environment},
        {"map", options.mapLabel.string()},
        {"scen", options.scenarioLabel.string()},
        {"solver", options.solverName},
        {"takeK", options.takeK},
        {"rep", options.repetition},
        {"dt", options.updateDt},
        {"substeps", options.substeps},
        {"agents", metrics.agents},
        {"plan_success", metrics.plan_success},
        {"planner_runtime_s", finiteOrZero(metrics.planner_runtime_s)},
        {"sim_runtime_s", finiteOrZero(metrics.sim_runtime_s)},
        {"makespan_s", finiteOrZero(metrics.makespan_s)},
        {"sum_of_cost_s", finiteOrZero(metrics.sum_of_cost_s)},
        {"sum_length_m", metrics.sum_length_m},
        {"avg_length_m", finiteOrZero(metrics.avg_length_m)},
        {"avg_completion_s", finiteOrZero(metrics.avg_completion_s)},
        {"cpu_peak_pct", peakCpu},
        {"mem_peak_gb", peakMemory},
    };
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Failed to write " + path.string());
    }
    output << std::setw(2) << metadata << '\n';
}

void appendMaster(
    const RunOpts& options,
    const metrics::Aggregate& metrics,
    const std::string_view environment,
    const double peakCpu,
    const double peakMemory)
{
    std::error_code error;
    const bool exists = fs::exists(options.masterCsv, error);
    if (error) {
        throw std::runtime_error(
            "Failed to inspect " + options.masterCsv.string());
    }
    ensureParent(options.masterCsv);
    std::ofstream output(options.masterCsv, std::ios::app);
    if (!output) {
        throw std::runtime_error(
            "Failed to append " + options.masterCsv.string());
    }
    if (!exists) {
        output << "session,env,sim_dt,sim_substeps,map,scen,solver,takeK,"
                  "rep,agents,plan_success,planner_runtime_s,sim_runtime_s,"
                  "makespan_s,sum_of_cost_s,sum_length_m,avg_length_m,"
                  "avg_completion_s,cpu_peak_pct,mem_peak_gb,run_dir\n";
    }
    output << options.sessionTag << ','
           << environment << ','
           << options.updateDt << ','
           << options.substeps << ','
           << pathStem(options.mapLabel) << ','
           << pathStem(options.scenarioLabel) << ','
           << options.solverName << ','
           << options.takeK << ','
           << options.repetition << ','
           << metrics.agents << ','
           << (metrics.plan_success ? 1 : 0) << ','
           << finiteOrZero(metrics.planner_runtime_s) << ','
           << finiteOrZero(metrics.sim_runtime_s) << ','
           << finiteOrZero(metrics.makespan_s) << ','
           << finiteOrZero(metrics.sum_of_cost_s) << ','
           << metrics.sum_length_m << ','
           << finiteOrZero(metrics.avg_length_m) << ','
           << finiteOrZero(metrics.avg_completion_s) << ','
           << peakCpu << ','
           << peakMemory << ','
           << options.runDirectory.string() << '\n';
}

bool recordRun(
    const RunOpts& options,
    Simulation& simulation,
    const std::string_view environment,
    const bool planned,
    const ResourceMonitor& monitor)
{
    auto metrics = simulation.computePlanMetrics();
    const bool reachedAll = planned && simulation.allAgentsReachedGoals();
    metrics.plan_success = metrics.plan_success && reachedAll;
    if (planned && !reachedAll) {
        Logger::instance().warn(
            "[BENCH] Goal check failed; marking run unsuccessful.");
    }
    writeMetrics(options.runDirectory / "metrics_summary.csv", metrics);
    writeMetadata(
        options, metrics, environment, monitor.peakCpuPercent(),
        monitor.peakMemoryGb());
    appendMaster(
        options, metrics, environment, monitor.peakCpuPercent(),
        monitor.peakMemoryGb());
    return reachedAll;
}

void configureSimulation(Simulation& simulation, const RunOpts& options)
{
    simulation.setDt(options.updateDt);
    simulation.setSubStepCount(options.substeps);
    simulation.setPlanner(makePlanner(options.solverName));
    if (options.writeStateCsv) {
        simulation.addObserver(
            std::make_shared<grace::sim::StateCsvObserver>(
                options.runDirectory / "state.csv", true,
                std::max(1, options.stateEveryN)));
        SimLogOptions logging;
        logging.logRobotStates = true;
        logging.everyNSteps = std::max(1, options.stateEveryN);
        simulation.setLogOptions(std::move(logging));
    }
}

bool isDigits(const char* value)
{
    if (*value == '\0') return false;
    for (const char* character = value; *character != '\0'; ++character) {
        if (!std::isdigit(static_cast<unsigned char>(*character))) {
            return false;
        }
    }
    return true;
}

bool readCpuTicks(const fs::path& path, unsigned long long& ticks)
{
    std::ifstream input(path);
    std::string line;
    if (!input || !std::getline(input, line)) return false;
    const auto closingParenthesis = line.rfind(')');
    if (closingParenthesis == std::string::npos) return false;
    std::istringstream fields(line.substr(closingParenthesis + 2));
    char state;
    long parent;
    unsigned long pgrp, session, tty, tpgid, flags;
    unsigned long minflt, cminflt, majflt, cmajflt;
    unsigned long long user, system;
    if (!(fields >> state >> parent >> pgrp >> session >> tty >> tpgid >>
          flags >> minflt >> cminflt >> majflt >> cmajflt >> user >>
          system)) {
        return false;
    }
    ticks = user + system;
    return true;
}

} // namespace

struct ResourceMonitor::Impl {
    std::atomic<bool> running{false};
    std::thread thread;
    std::atomic<double> peakCpu{0.0};
    std::atomic<double> peakMemory{0.0};
    std::unordered_map<pid_t, unsigned long long> previousTicks;
    const long ticksPerSecond{std::max<long>(1, sysconf(_SC_CLK_TCK))};

    static std::unordered_map<pid_t, std::vector<pid_t>> children()
    {
        std::unordered_map<pid_t, std::vector<pid_t>> result;
        DIR* directory = opendir("/proc");
        if (directory == nullptr) return result;
        while (const dirent* entry = readdir(directory)) {
            if (!isDigits(entry->d_name)) continue;
            std::ifstream input(
                std::string("/proc/") + entry->d_name + "/stat");
            std::string line;
            if (!input || !std::getline(input, line)) continue;
            const auto closingParenthesis = line.rfind(')');
            if (closingParenthesis == std::string::npos) continue;
            std::istringstream fields(line.substr(closingParenthesis + 2));
            char state;
            long parent;
            if (!(fields >> state >> parent)) continue;
            result[static_cast<pid_t>(parent)].push_back(
                static_cast<pid_t>(std::stol(entry->d_name)));
        }
        closedir(directory);
        return result;
    }

    static std::unordered_set<pid_t> processTree(const pid_t root)
    {
        const auto childMap = children();
        std::unordered_set<pid_t> result{root};
        std::vector<pid_t> queue{root};
        for (std::size_t index = 0; index < queue.size(); ++index) {
            const auto found = childMap.find(queue[index]);
            if (found == childMap.end()) continue;
            for (const pid_t child : found->second) {
                if (result.insert(child).second) queue.push_back(child);
            }
        }
        return result;
    }

    static bool allThreadTicks(
        const pid_t process, unsigned long long& ticks)
    {
        const fs::path taskDirectory =
            fs::path("/proc") / std::to_string(process) / "task";
        DIR* directory = opendir(taskDirectory.c_str());
        if (directory == nullptr) {
            return readCpuTicks(
                fs::path("/proc") / std::to_string(process) / "stat", ticks);
        }
        ticks = 0;
        while (const dirent* entry = readdir(directory)) {
            if (!isDigits(entry->d_name)) continue;
            unsigned long long threadTicks = 0;
            if (readCpuTicks(
                    taskDirectory / entry->d_name / "stat", threadTicks)) {
                ticks += threadTicks;
            }
        }
        closedir(directory);
        return true;
    }

    static long residentMemoryKb(const pid_t process)
    {
        std::ifstream input(
            fs::path("/proc") / std::to_string(process) / "status");
        std::string line;
        while (std::getline(input, line)) {
            if (line.rfind("VmRSS:", 0) == 0) {
                std::istringstream fields(line.substr(6));
                long result = 0;
                fields >> result;
                return result;
            }
        }
        return 0;
    }

    void sampleLoop()
    {
        using clock = std::chrono::steady_clock;
        auto previousTime = clock::now();
        while (running.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            const auto now = clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - previousTime).count();
            previousTime = now;
            const auto processes = processTree(getpid());
            unsigned long long deltaTicks = 0;
            long memoryKb = 0;

            for (auto iterator = previousTicks.begin();
                 iterator != previousTicks.end();) {
                if (!processes.contains(iterator->first)) {
                    iterator = previousTicks.erase(iterator);
                } else {
                    ++iterator;
                }
            }
            for (const pid_t process : processes) {
                unsigned long long current = 0;
                if (allThreadTicks(process, current)) {
                    const auto previous = previousTicks.find(process);
                    if (previous != previousTicks.end() &&
                        current >= previous->second) {
                        deltaTicks += current - previous->second;
                    }
                    previousTicks[process] = current;
                }
                memoryKb += residentMemoryKb(process);
            }

            const double cpu = 100.0 *
                (static_cast<double>(deltaTicks) /
                 static_cast<double>(ticksPerSecond)) / elapsed;
            peakCpu.store(
                std::max(peakCpu.load(std::memory_order_relaxed), cpu),
                std::memory_order_relaxed);
            const double memory =
                static_cast<double>(memoryKb) * 1024.0 / 1.0e9;
            peakMemory.store(
                std::max(peakMemory.load(std::memory_order_relaxed), memory),
                std::memory_order_relaxed);
        }
    }
};

ResourceMonitor::ResourceMonitor() : impl_(std::make_unique<Impl>()) {}
ResourceMonitor::~ResourceMonitor() { stop(); }

void ResourceMonitor::start()
{
    if (impl_->running.exchange(true)) return;
    impl_->thread = std::thread([this] { impl_->sampleLoop(); });
}

void ResourceMonitor::stop()
{
    if (!impl_ || !impl_->running.exchange(false)) return;
    if (impl_->thread.joinable()) impl_->thread.join();
}

double ResourceMonitor::peakCpuPercent() const noexcept
{
    return impl_->peakCpu.load();
}

double ResourceMonitor::peakMemoryGb() const noexcept
{
    return impl_->peakMemory.load();
}

bool runBenchmark(const RunOpts& options)
{
    grace::output::RunOutputConfig outputConfig;
    outputConfig.baseDirectory = options.runDirectory;
    (void)grace::output::startRunOutput(outputConfig);
    std::error_code error;
    fs::create_directories(options.runDirectory, error);
    if (error) {
        throw std::runtime_error(
            "Failed to create run directory: " + error.message());
    }

    ResourceMonitor monitor;
    monitor.start();
    switch (options.env) {
        case BenchEnv::Grid: {
            GridEnvironment environment;
            const auto map = [&options] {
                if (options.gridMap.extension() == ".map") {
                    return grace::io::readGridMovingAIMap(options.gridMap);
                }
                if (options.gridMap.extension() == ".xml") {
                    return grace::io::readGridXmlMap(options.gridMap);
                }
                throw std::invalid_argument(
                    "Unsupported grid map extension: " +
                    options.gridMap.extension().string());
            }();
            if (!map) {
                throw std::runtime_error(map.error);
            }
            environment.makeGridEnvironment(
                map.value->width, map.value->height,
                options.gridCellLength, map.value->occupancy,
                options.gridBoundaryScale);
            Simulation simulation(environment);
            configureSimulation(simulation, options);
            std::vector<StartGoal> startGoals;
            if (!options.gridAgents.empty()) {
                grace::io::GridLORROptions readOptions;
                readOptions.takeK = options.takeK;
                readOptions.skipBlockedCells =
                    options.gridSkipBlockedGoals;
                startGoals = grace::io::readGridLORRAgentsTasks(
                    environment, options.gridAgents, options.gridTasks,
                    readOptions);
            } else if (options.gridScenario.extension() == ".xml") {
                auto tasks = grace::io::readGridXmlTask(
                    environment, options.gridScenario, options.takeK);
                if (!tasks) throw std::runtime_error(tasks.error);
                startGoals = std::move(*tasks.value);
            } else {
                auto scenario = grace::io::readGridMovingAIScenario(
                    environment, options.gridScenario, options.takeK);
                if (!scenario) throw std::runtime_error(scenario.error);
                startGoals = std::move(*scenario.value);
            }
            if (startGoals.empty()) {
                throw std::runtime_error(
                    "Grid input contains no usable agents");
            }
            grace::sim::SpawnOptions spawnOptions;
            spawnOptions.baseName = "r";
            const auto ids =
                grace::sim::spawnGridAgent(
                    simulation, startGoals, spawnOptions);
            const bool planned = simulation.plan(ids, startGoals);
            if (planned) simulation.run();
            monitor.stop();
            return recordRun(
                options, simulation, "grid", planned, monitor);
        }
        case BenchEnv::Roadmap: {
            RoadmapEnvironment environment;
            grace::sim::GraphMLImport import;
            if (!grace::sim::readGraphML(
                    options.graphXml, environment, &import)) {
                throw std::runtime_error(
                    "Failed to read roadmap graph: " +
                    options.graphXml.string());
            }
            if (options.roadmapWall > 0.0f) {
                environment.addBoundaryWalls(options.roadmapWall);
            }
            Simulation simulation(environment);
            configureSimulation(simulation, options);
            const bool planned =
                grace::sim::ops::spawnAndPlanFromTaskXml(
                    simulation, environment, options.tasksXml, import,
                    options.roadmapRobotRadius, options.takeK, "r");
            if (planned) simulation.run();
            monitor.stop();
            return recordRun(
                options, simulation, "roadmap", planned, monitor);
        }
        case BenchEnv::Continuous: {
            ContinuousEnvironment environment;
            Simulation simulation(environment);
            configureSimulation(simulation, options);
            const bool planned =
                grace::sim::ops::spawnAndPlanFromProblemYaml(
                    simulation, environment, options.problemYaml,
                    options.continuousWall, options.takeK, "r");
            if (planned) simulation.run();
            monitor.stop();
            return recordRun(
                options, simulation, "continuous", planned, monitor);
        }
    }
    return false;
}

} // namespace grace::benchmarking
