// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "planner/Planner.h"
#include "planner/plugin/plugin_api.h"

#include <stdexcept>

#ifndef TEST_PLUGIN_ABI
#define TEST_PLUGIN_ABI kGracePluginAbiVersion
#endif

#ifndef TEST_PLUGIN_ID
#define TEST_PLUGIN_ID "eecbs-src"
#endif

#ifndef TEST_PLUGIN_ENVIRONMENT
#define TEST_PLUGIN_ENVIRONMENT GRACE_PLUGIN_ENVIRONMENT_GRID
#endif

#ifndef TEST_PLANNER_ENVIRONMENT
#define TEST_PLANNER_ENVIRONMENT EnvironmentType::GridEnvironment
#endif

namespace {

class FakePluginPlanner final : public IPlanner {
public:
    const char* name() const noexcept override {
        return "fixture-plugin";
    }

    EnvironmentType type() const noexcept override {
        return TEST_PLANNER_ENVIRONMENT;
    }

    PlanResult plan(const PlanRequest&) override {
#if defined(TEST_PLUGIN_THROW_PLAN)
        throw std::runtime_error("fixture planning failure");
#else
        return {};
#endif
    }

    void configure(const PlannerSettings&) override {
#if defined(TEST_PLUGIN_THROW_CONFIGURE)
        throw std::runtime_error("fixture configuration failure");
#endif
    }
};

}  // namespace

extern "C" int grace_plugin_abi() {
    return TEST_PLUGIN_ABI;
}

#if !defined(TEST_PLUGIN_OMIT_ID)
extern "C" const char* grace_plugin_id() {
#if defined(TEST_PLUGIN_THROW_ID)
    throw std::runtime_error("fixture metadata failure");
#else
    return TEST_PLUGIN_ID;
#endif
}
#endif

extern "C" int grace_plugin_environment() {
    return TEST_PLUGIN_ENVIRONMENT;
}

extern "C" IPlanner* create_planner() {
#if defined(TEST_PLUGIN_CREATE_NULL)
    return nullptr;
#elif defined(TEST_PLUGIN_THROW_CREATE)
    throw std::runtime_error("fixture construction failure");
#else
    return new FakePluginPlanner;
#endif
}

extern "C" void destroy_planner(IPlanner* planner) {
    delete planner;
#if defined(TEST_PLUGIN_THROW_DESTROY)
    throw std::runtime_error("fixture destruction failure");
#endif
}
