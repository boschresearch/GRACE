// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

// Sample_MAPF_Continuous.cpp — Continuous sample registration & ctor
#include "platform/PlatformBase.h"
#include "platform/PlannerCatalog.h"
#include "platform/PlatformDefaults.h"
#include "env/ContinuousEnvironment.h"
#include "Simulation.h"
#include "io/SimContinuousYamlOps.h"
#include "planner/plugin/PlannerFactory.h"

namespace {
    const auto kProblemYaml = platform::defaults::problemYamlDemo();

    class PlatformSample_Continuous final : public PlatformBase {
    public:
        explicit PlatformSample_Continuous(SampleContext* ctx) : PlatformBase(ctx)
        {
            auto env = std::make_unique<ContinuousEnvironment>();
            auto sim = std::make_unique<Simulation>(*env);

            m_plannerUi.name = platform::PlannerCatalog::defaultFor(
                platform::EnvKind::Continuous);
            sim->setPlanner(makePlanner(m_plannerUi.name));
            PlannerSettings user; user.set("timelimit_s", 45.0);
            sim->configurePlanner(std::move(user));     // ⬅️ new API

            // ⬇️ convenience flow (build env + spawn + plan)
            (void)grace::sim::ops::spawnAndPlanFromProblemYaml(*sim, *env, kProblemYaml, /*wallThickness=*/0.25f);

            centerOnEnv(*env);
            adoptWorld(env->getB2World());
            m_envOwned = std::move(env);
            m_sim      = std::move(sim);
            m_playback.visualizationTime = 0.0;
        }
    };

    Sample* CreateContinuous(SampleContext* c) { return new PlatformSample_Continuous(c); }

    const int s_reg_continuous = RegisterSample("MRMP", "Continuous: problem.yaml + editor", &CreateContinuous);
} // namespace
