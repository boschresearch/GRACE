// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <memory>
#include <unordered_map>
#include <vector>
#include <optional>
#include <functional>
#include <limits>
#include <filesystem>
#include <box2d/box2d.h>
#include "env/BaseEnvironment.h"
#include "env/ContinuousEnvironment.h"
#include "env/ICapabilities.h"
#include "env/RoadmapEnvironment.h"
#include "robot/RobotSystem.h"
#include "robot/Controller.h"
#include "planner/Planner.h"
#include "sim/GoalCheck.h"
#include "sim/GridConflictAuditor.h"
#include "sim/ISimObserver.h"
#include "types/Footprint.h"
#include "util/Box2dIdHash.h"
#include "util/Logger.h"
#include "util/Metrics.h"

struct SimLogOptions {
    int  everyNSteps    = 60;               // log every N sim steps
    bool logRobotStates = true;             // emit state lines
    std::vector<b2BodyId> watch;            // empty = log all
    LogLevel stateLevel = LogLevel::DEBUG;  // level for state lines
};

struct SimConfig {
    float updateDt     = 1.0f / 60.0f;
    int   subStepCount = 4;
    SimLogOptions log{};
};

using grace::types::Footprint;
using grace::types::FootprintKind;

class Simulation {
public:
    explicit Simulation(BaseEnvironment& env, const SimConfig &cfg = {});

    // Access environment
    EnvironmentType getEnvType() const {return m_env.getEnvironmentType();}
    const IDiscreteGrid* gridEnv() const noexcept { return m_env.gridCapability(); }
    const IRoadmap* roadmapEnv() const noexcept { return m_env.roadmapCapability(); }
    const IContinuousSpace* continuousEnv() const noexcept { return m_env.continuousCapability(); }

    // Spawning Robots
    [[maybe_unused]] b2BodyId spawnGridCellRobot(int c, int r, const std::string& name);
    [[maybe_unused]] b2BodyId spawnCircleRobot(b2Vec2 center, float radius, const std::string& name);
    [[maybe_unused]] b2BodyId spawnRoadmapNodeRobot(const IRoadmap& R, IRoadmap::NodeId n,
        float radius, std::string name);
    [[maybe_unused]] b2BodyId spawnByAlgoType(const std::string& algoType, b2Vec2 center, std::string name,
        std::optional<Footprint> fp = std::nullopt);

    // Access Robots
    RobotSystem& robots()             { return m_robots; }
    const RobotSystem& robots() const { return m_robots; }
    void clearAllRobots();

    // Planning & control
    void setPlanner(std::unique_ptr<IPlanner> planner) {m_planner = std::move(planner);}
    void configurePlanner(PlannerSettings opts) {m_plannerUserOpts = std::move(opts);}
    void setPlannerRequestSettings(PlannerSettings opts) {m_plannerRequestOpts = std::move(opts);}
    void setController(const b2BodyId id, std::unique_ptr<IController> ctrl) {m_ctrl[id] = std::move(ctrl);}
    bool plan(const std::vector<b2BodyId>& ids, const std::vector<StartGoal>& sgs);
    bool recordGoals(const std::vector<b2BodyId>& ids, const std::vector<b2Vec2>& goals);

    // Simulation loop
    void run() { run(m_updateDt); }
    void run(const float dt) {const float T = maxPlannedTime(); run(dt, T + dt);}
    void run(float dt, float T);

    void stepOnce() { step(m_updateDt); }
    void stepOnce(const float dt) { step(dt); }

    // Timing/counters
    void  setDt(const float dt)  { m_updateDt = dt; }
    void  setSubStepCount(const int n)  { m_subStepCount = (n > 0 ? n : 4); }
    float currentSimTime() const { return m_simTime; }

    // Controllers
    void resetController(b2BodyId id);
    void resetControllers() const;

    // Plan state & metrics
    bool  allAgentsReachedGoals(float tolMeters = -1.0f) const;
    void  AssertIfAllAgentsReachedTheirGoals() const;
    float maxPlannedTime() const;
    metrics::Aggregate computePlanMetrics() const;

    // Diagnostics / logging
    double lastPlannerRuntimeSec() const { return m_lastPlannerRuntimeSec; }
    double lastSimRuntimeSec() const     { return m_lastSimRuntimeSec; }
    void setLogOptions(SimLogOptions opt) { m_log = std::move(opt); }
    const SimLogOptions& logOptions() const { return m_log; }
    void logRobotStatesNow(LogLevel level = LogLevel::INFO);

    // Observers
    void addObserver(std::shared_ptr<grace::sim::ISimObserver> obs);
    void clearObservers();

private:
    BaseEnvironment& m_env;
    RobotSystem    m_robots;
    std::unique_ptr<IPlanner> m_planner;
    PlannerSettings           m_plannerUserOpts;
    PlannerSettings           m_plannerRequestOpts;

    std::unordered_map<b2BodyId, std::unique_ptr<IController>, BodyIdHash, BodyIdEq> m_ctrl;
    float    m_updateDt{1.0f/60.0f};
    int      m_subStepCount{4};
    uint64_t m_stepCount{0};
    float    m_simTime{0.0f};
    SimLogOptions m_log{};

    bool   m_lastPlanSucceeded{false};
    double m_lastPlannerRuntimeSec{std::numeric_limits<double>::quiet_NaN()};
    double m_lastSimRuntimeSec{std::numeric_limits<double>::quiet_NaN()};

    float               m_gridTickSec{1.0f};
    GridConflictAuditor m_auditor{m_gridTickSec};

    std::vector<std::shared_ptr<grace::sim::ISimObserver>> m_observers;

    void singleIntegratorController(b2BodyId id);
    void applyAutoPlannerHints(const std::vector<b2BodyId>& ids) const;
    void preparePlannerFor(const std::vector<b2BodyId>& ids);
    float inferMinAgentRadius(const std::vector<b2BodyId>& ids) const;
    static bool isWatched(const std::vector<b2BodyId>& watch, b2BodyId id);
    void step(float dt);
};
