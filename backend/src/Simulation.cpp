// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/Simulation.h"
#include "../inc/env/GridEnvironment.h"
#include "../inc/env/RoadmapEnvironment.h"
#include "../inc/env/ContinuousEnvironment.h"
#include "../inc/util/Metrics.h"
#include "../inc/util/AlgoFootprints.h"
#include "../inc/types/Footprint.h"
#include <fmt/format.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <unordered_set>

#include "../inc/sim/PlanVerifier.h"

using grace::types::Footprint;
using grace::types::FootprintKind;

Simulation::Simulation(BaseEnvironment& env, const SimConfig &cfg)
    : m_env(env)
{
    m_updateDt    = cfg.updateDt;
    m_subStepCount= cfg.subStepCount;
    m_log         = cfg.log;
    m_auditor = GridConflictAuditor{m_gridTickSec};
}

b2BodyId Simulation::spawnGridCellRobot(const int c, const int r, const std::string& name) {
    const auto* grid = gridEnv();
    if (!grid) {
        Logger::instance().error("spawnGridCellRobot: environment has no grid capability.");
        return b2_nullBodyId;
    }
    const float R = 0.35f * grid->cellLength();
    const b2Vec2 p = grid->cellToWorld({c, r});
    return spawnCircleRobot(p, R, name);
}

b2BodyId Simulation::spawnRoadmapNodeRobot(const IRoadmap& R, const IRoadmap::NodeId n,
    const float radius, std::string name) {
    const b2Vec2 p = R.nodePos(n);
    return spawnCircleRobot(p, radius, std::move(name));
}

b2BodyId Simulation::spawnCircleRobot(const b2Vec2 center, const float radius, const std::string& name) {
    const auto id = m_robots.spawnCircleRobot(m_env, center, radius, name);
    singleIntegratorController(id);
    return id;
}

b2BodyId Simulation::spawnByAlgoType(const std::string& algoType,
                                     const b2Vec2 center,
                                     std::string name,
                                     std::optional<Footprint> fp){
    if (getEnvType() != EnvironmentType::ContinuousEnvironment) {
        Logger::instance().error("spawnByAlgoType: only valid for Continuous environments.");
        return b2_nullBodyId;
    }
    if (!fp) {
        fp = grace::util::algo_fp::footprintForAlgoCI(algoType);
        if (!fp) {
            Logger::instance().error("spawnByAlgoType: unsupported type '" + algoType + "'");
            return b2_nullBodyId;
        }
    }
    switch (fp->kind) {
        case FootprintKind::Circle: {
            const float r = (fp->radius > 0.f) ? fp->radius : 0.001f;
            return m_robots.spawnCircleRobot(m_env, center, r, std::move(name));
        }
        case FootprintKind::Box: {
            const float hx   = std::max(0.f, fp->sx * 0.5f);
            const float hy   = std::max(0.f, fp->sy * 0.5f);
            return m_robots.spawnBoxRobot(m_env, center, hx, hy, std::move(name));
        }
        case FootprintKind::MultiBox: {
            std::vector<LocalBox> boxes; boxes.reserve(fp->parts.size());
            for (const auto& p : fp->parts) {
                boxes.push_back(LocalBox{
                    /*localCenter*/ {p.center.x, p.center.y},
                    /*halfW */ std::max(0.f, p.sx * 0.5f),
                    /*halfH */ std::max(0.f, p.sy * 0.5f),
                    /*angle */ p.angle
                });
            }
            return m_robots.spawnCompoundBoxesRobot(m_env, center, boxes, std::move(name));
        }
        default: break;
    }
    Logger::instance().error("spawnByAlgoType: invalid footprint data for '" + algoType + "'");
    return b2_nullBodyId;
}

bool Simulation::plan(const std::vector<b2BodyId>& ids, const std::vector<StartGoal>& sgs)
{
    if (!m_planner) {
        Logger::instance().error("plan: no planner set.");
        return false;
    }
    if (m_planner->type() != m_env.getEnvironmentType()) {
        Logger::instance().error("plan: planner and environment mismatch.");
        return false;
    }
    if (ids.empty() || sgs.empty()) {
        Logger::instance().error("plan: empty ids/sgs.");
        return false;
    }
    if (ids.size() != sgs.size()) {
        Logger::instance().error(fmt::format(
            "plan: size mismatch: ids={}, sgs={}.", ids.size(), sgs.size()));
        return false;
    }
    std::unordered_set<b2BodyId, BodyIdHash, BodyIdEq> uniqueIds;
    uniqueIds.reserve(ids.size());
    for (const b2BodyId id : ids) {
        if (!uniqueIds.insert(id).second) {
            Logger::instance().error(
                "plan: duplicate robot IDs are not allowed.");
            return false;
        }
    }

    struct RequestedGoalRollback {
        std::vector<std::pair<RobotMeta*, std::optional<b2Vec2>>> previous;
        bool committed{false};

        ~RequestedGoalRollback() {
            if (committed) {
                return;
            }
            for (auto& [metadata, goal] : previous) {
                metadata->requested_goal_m = goal;
            }
        }
    } requestedGoals;
    requestedGoals.previous.reserve(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        auto* metadata = m_robots.meta(ids[i]);
        if (!metadata) {
            Logger::instance().error(
                "plan: missing RobotMeta for a requested robot.");
            return false;
        }
        requestedGoals.previous.emplace_back(
            metadata, metadata->requested_goal_m);
        metadata->requested_goal_m = sgs[i].goal;
    }

    preparePlannerFor(ids);
    Logger::instance().info(fmt::format("Calling planner: {}", m_planner->name()));
    PlanRequest req{
        .env = m_env,
        .ids = ids,
        .sgs = sgs,
        .settings = m_plannerRequestOpts,
        .time_limit_s = INFINITY,
        .rng_seed = 0,
    };
    const auto t0 = std::chrono::steady_clock::now();
    PlanResult res = m_planner->plan(req);
    const auto t1 = std::chrono::steady_clock::now();
    const double measured = std::chrono::duration<double>(t1 - t0).count();
    m_lastPlannerRuntimeSec =
        (std::isfinite(res.diag.runtime_s) ? res.diag.runtime_s : measured);
    if (res.status != PlanStatus::Success || res.plans.empty() || res.ids.empty()) {
        m_lastPlanSucceeded = false;
        Logger::instance().warn("Planning failed or returned an empty result.");
        return false;
    }
    if (res.ids.size() != ids.size() || res.ids.size() != res.plans.size()) {
        m_lastPlanSucceeded = false;
        Logger::instance().warn(fmt::format(
            "Planning returned mismatched result sizes: requested={}, ids={}, plans={}.",
            ids.size(), res.ids.size(), res.plans.size()));
        return false;
    }
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (!B2_ID_EQUALS(res.ids[i], ids[i])) {
            m_lastPlanSucceeded = false;
            Logger::instance().warn(
                "Planning returned robot IDs that do not match the request order.");
            return false;
        }
    }
    {
        PlanVerifier verifier(m_env, m_robots, Logger::instance());
        PlanVerifyOptions vopt;
        vopt.nonGridPolicy = NonGridEndpointPolicy::Disabled; // legacy compat on non-grid
        if (!verifier.accept(ids, res, vopt)) {
            m_lastPlanSucceeded = false;
            return false;
        }
    }
    m_lastPlanSucceeded = true;
    for (std::size_t i = 0; i < res.ids.size(); ++i) {
        b2BodyId id = res.ids[i];
        if (auto* m = m_robots.meta(id)) {
            m->plan    = std::move(res.plans[i]);
            m->next_wp = 0;
            Logger::instance().debug(fmt::format("Plan for {}: waypoints={}", m->name, m->plan.waypoints.size()));
        }
        singleIntegratorController(id);
        resetController(id);
    }
    requestedGoals.committed = true;
    Logger::instance().info("Plan generated and installed for all robots.");
    return true;
}

bool Simulation::recordGoals(const std::vector<b2BodyId>& ids, const std::vector<b2Vec2>& goals)
{
    const std::size_t N = std::min(ids.size(), goals.size());
    bool any = false;
    for (std::size_t i = 0; i < N; ++i) {
        if (auto* m = m_robots.meta(ids[i])) {
            m->requested_goal_m = goals[i];
            any = true;
        } else {
            Logger::instance().warn("recordGoals: missing RobotMeta for an id; goal not recorded.");
        }
    }
    if (!any) {
        Logger::instance().error("recordGoals: no goals were recorded — check sizes/contents.");
    } else if (N < ids.size()) {
        Logger::instance().warn(fmt::format(
            "recordGoals: {} goal(s) recorded, {} robot(s) present — extra robots have no goals.",
            N, ids.size()));
    }
    return any;
}

void Simulation::run(float dt, float T)
{
    if (!(dt > 0.0f)) {
        Logger::instance().error("Run aborted: dt <= 0.");
        return;
    }
    if (!(T > 0.0f)) {
        Logger::instance().warn("Run aborted: non-positive horizon.");
        return;
    }
    const int N = std::max(0, static_cast<int>(std::ceil(T / dt)));
    Logger::instance().info(fmt::format("Run start: dt={:.4f}s steps={} horizon={:.3f}s", dt, N, T));

    // Notify observers run start
    for (auto& obs : m_observers) if (obs) obs->onRunStart(m_env, m_robots);

    if (m_log.logRobotStates) {
        logRobotStatesNow(m_log.stateLevel);
    }
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < N; ++i) step(dt);
    auto t1 = std::chrono::steady_clock::now();
    m_lastSimRuntimeSec = std::chrono::duration<double>(t1 - t0).count();
    Logger::instance().info(fmt::format("Run done: sim_t={:.3f}s steps={}", m_simTime, m_stepCount));

    const auto M = computePlanMetrics();
    Logger::instance().info(fmt::format(
        "METRICS | agents={} | makespan={:.3f}s | flowtime={:.3f}s | sum_path_len={:.3f}m | "
        "avg_path_len={:.3f}m | planner_runtime={:.3f}s | sim_runtime={:.3f}s",
        M.agents,
        (std::isnan(M.makespan_s) ? -1.0 : M.makespan_s),
        (std::isnan(M.sum_of_cost_s) ? -1.0 : M.sum_of_cost_s),
        M.sum_length_m,
        (std::isnan(M.avg_length_m) ? -1.0 : M.avg_length_m),
        (std::isnan(M.planner_runtime_s) ? 0.0 : M.planner_runtime_s),
        (std::isnan(M.sim_runtime_s) ? 0.0 : M.sim_runtime_s)
    ));

    // Notify observers run end
    grace::sim::SimRunSummary summary{
        /*metrics*/ M,
        /*plannerRuntimeSec*/ m_lastPlannerRuntimeSec,
        /*simRuntimeSec*/     m_lastSimRuntimeSec,
        /*planSucceeded*/     m_lastPlanSucceeded
    };
    for (auto& obs : m_observers) if (obs) obs->onRunEnd(summary);
}

void Simulation::resetController(const b2BodyId id) {
    auto it = m_ctrl.find(id);
    if (it != m_ctrl.end() && it->second) {
        it->second->reset();
    }
}

void Simulation::resetControllers() const {
    for (auto& kv : m_ctrl) {
        if (kv.second) kv.second->reset();
    }
}

void Simulation::clearAllRobots() {
    m_env.clearRobots();
    m_robots.clear();
    m_ctrl.clear();
    m_stepCount = 0;
    m_simTime   = 0.0f;
    m_lastPlanSucceeded     = false;
    m_lastPlannerRuntimeSec = std::numeric_limits<double>::quiet_NaN();
    m_lastSimRuntimeSec     = std::numeric_limits<double>::quiet_NaN();
    m_auditor.reset();
}

bool Simulation::allAgentsReachedGoals(const float tolMeters) const
{
    GoalCheckOptions opt;
    if (getEnvType() == EnvironmentType::GridEnvironment) {
        opt.policy = GoalPolicy::GridCellExact;
    } else {
        if (tolMeters >= 0.0f) {
            opt.policy = GoalPolicy::EuclideanTol;
            opt.tol.minMeters    = tolMeters;
            opt.tol.fracOfRadius = 0.0f;
        } else {
            opt.policy = GoalPolicy::GridCellExact; // ignored outside grid
        }
    }
    return all_agents_at_goals(m_env, m_robots, opt, Logger::instance());
}

void Simulation::AssertIfAllAgentsReachedTheirGoals() const {
    if (!allAgentsReachedGoals()) {
        Logger::instance().fatal("Goal check: one or more agents are NOT at their goals.");
    } else {
        Logger::instance().info("Goal check: all agents are at their goals.");
    }
}

float Simulation::maxPlannedTime() const
{
    float T = 0.0f;
    const auto& ids = m_robots.robots();
    for (b2BodyId id : ids) {
        if (const RobotMeta* meta = m_robots.meta(id)) {
            if (!meta->plan.waypoints.empty()) {
                T = std::max(T, meta->plan.waypoints.back().t);
            }
        }
    }
    return T;
}

metrics::Aggregate Simulation::computePlanMetrics() const
{
    std::vector<metrics::PerAgent> rows;
    const auto& ids = m_robots.robots();
    rows.reserve(ids.size());
    for (b2BodyId id : ids) {
        const RobotMeta* meta = m_robots.meta(id);
        if (!meta) continue;
        const auto& plan = meta->plan;
        if (plan.waypoints.empty()) continue;
        rows.emplace_back(metrics::perAgentFromPlan(plan, id, meta->name));
    }
    return metrics::summarize(
        rows, m_lastPlannerRuntimeSec, m_lastSimRuntimeSec, m_lastPlanSucceeded);
}

void Simulation::logRobotStatesNow(LogLevel level)
{
    const auto& ids = m_robots.robots();
    const auto* grid = gridEnv();
    for (b2BodyId id : ids) {
        if (!isWatched(m_log.watch, id)) continue;
        const b2Vec2 pos = b2Body_GetPosition(id);
        const float  ang = b2Rot_GetAngle(b2Body_GetRotation(id));
        const b2Vec2 vel = b2Body_GetLinearVelocity(id);
        const RobotMeta* meta = m_robots.meta(id);
        const char* name = (meta ? meta->name.c_str() : "<unnamed>");
        const std::size_t k = (meta ? meta->next_wp : 0);
        const std::size_t n = (meta ? meta->plan.waypoints.size() : 0);
        std::string cellStr;
        if (grid) {
            const auto [c, r] = grid->worldToCell(pos);
            cellStr = fmt::format(" cell=(x={},y={})", c, r);
        }
        std::string goalStr;
        if (meta && meta->requested_goal_m) {
            if (grid) {
                const auto [c, r] = grid->worldToCell(meta->requested_goal_m.value());
                goalStr = fmt::format(" goal=(x={},y={})", c, r);
            } else {
                goalStr = fmt::format(" goal=({:.3f},{:.3f})",
                    meta->requested_goal_m->x, meta->requested_goal_m->y);
            }
        }
        Logger::instance().log(
            level, std::source_location::current(),
            fmt::format("t={:.2f}s | {} pos=({:.3f},{:.3f}){}{} θ={:.2f} vel=({:.3f},{:.3f}) wp={}/{}",
                        m_simTime, name, pos.x, pos.y, cellStr, goalStr, ang, vel.x, vel.y, k, n));
    }
}

float Simulation::inferMinAgentRadius(const std::vector<b2BodyId>& ids) const
{
    float r_min = std::numeric_limits<float>::infinity();
    for (auto id : ids) {
        if (const auto* m = m_robots.meta(id)) {
            if (m->effectiveRadius > 0.0f && std::isfinite(m->effectiveRadius)) {
                if (std::isfinite(r_min) && r_min != m->effectiveRadius) {
                    Logger::instance().warn("Robots have different radius. You may want to check it.");
                }
                r_min = std::min(r_min, m->effectiveRadius);
            }
        }
    }
    return std::isfinite(r_min) ? r_min : 0.0f;
}

bool Simulation::isWatched(const std::vector<b2BodyId>& watch, b2BodyId id) {
    if (watch.empty()) return true;
    for (auto w : watch) if (w.index1 == id.index1 && w.generation == id.generation) return true;
    return false;
}

void Simulation::singleIntegratorController(b2BodyId id) {
    m_ctrl[id] = std::make_unique<ConstantSpeedController>();
    Logger::instance().debug(fmt::format("Single Integrator for Controller {}: {}",
        m_robots.meta(id)->name, m_ctrl[id]->name()));
}

void Simulation::applyAutoPlannerHints(const std::vector<b2BodyId>& ids) const {
    PlannerSettings autoHints;
    const float r = inferMinAgentRadius(ids);
    if (r > 0.0f && std::isfinite(r)) autoHints.set("agent_size", r);
    m_planner->configure(autoHints);
}

void Simulation::preparePlannerFor(const std::vector<b2BodyId>& ids) {
    applyAutoPlannerHints(ids);
    if (!m_plannerUserOpts.kv.empty()) {
        m_planner->configure(m_plannerUserOpts);
    }
}

void Simulation::addObserver(std::shared_ptr<grace::sim::ISimObserver> obs) {
    if (obs) m_observers.push_back(std::move(obs));
}

void Simulation::clearObservers() {
    m_observers.clear();
}

/*** private functions ***/

void Simulation::step(float dt)
{
    const auto& ids = m_robots.robots();
    for (b2BodyId id : ids) {
        auto it = m_ctrl.find(id);
        if (it != m_ctrl.end() && it->second) {
            it->second->update(dt, m_env, m_robots, id);
        }
    }
    b2World_Step(m_env.getB2World(), dt, m_subStepCount);
    m_simTime += dt;

    if (const auto* grid = gridEnv()) {
        m_auditor.tick(m_simTime, *grid, m_robots, Logger::instance());
    }
    ++m_stepCount;

    // Observer tick
    grace::sim::SimStepContext ctx{ m_simTime, dt, m_stepCount, m_env, m_robots };
    for (auto& obs : m_observers) if (obs) obs->onPostStep(ctx);

    if (m_log.logRobotStates && m_log.everyNSteps > 0 && (m_stepCount % m_log.everyNSteps == 0)) {
        logRobotStatesNow(m_log.stateLevel);
    }
}
