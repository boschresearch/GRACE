// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/planner/plugin/plugin_api.h"
#include "../inc/planner/Planner.h"

// You must define these via compile definitions:
//
//   PLUGIN_PLANNER_HEADER  -> string literal path to the planner's header
//   PLUGIN_PLANNER_CLASS   -> the concrete planner class (symbol)
//   PLUGIN_CONFIG_TYPE     -> the config struct/class type (symbol)
//   (optional) PLUGIN_CONFIG_INIT -> initializer for the config (defaults to {})
//
#ifndef PLUGIN_PLANNER_HEADER
#  error "PLUGIN_PLANNER_HEADER must be defined (e.g., \"planner/classical/EECBS/EECBS_Source.h\")"
#endif
#ifndef PLUGIN_PLANNER_CLASS
#  error "PLUGIN_PLANNER_CLASS must be defined (e.g., EECBS_Source)"
#endif
#ifndef PLUGIN_CONFIG_TYPE
#  error "PLUGIN_CONFIG_TYPE must be defined (e.g., EECBS_SourceConfig)"
#endif
#ifndef PLUGIN_ID_STR
#  error "PLUGIN_ID_STR must be defined (e.g., \"eecbs-src\")"
#endif
#ifndef PLUGIN_ENVIRONMENT
#  error "PLUGIN_ENVIRONMENT must be defined"
#endif

// Safe default for config initialization
#ifndef PLUGIN_CONFIG_INIT
#  define PLUGIN_CONFIG_INIT {}
#endif

// Include the solver's header provided by the macro:
#include PLUGIN_PLANNER_HEADER

#if defined(_WIN32)
#  define GRACE_PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#  define GRACE_PLUGIN_EXPORT \
    extern "C" __attribute__((visibility("default")))
#endif

GRACE_PLUGIN_EXPORT IPlanner* create_planner() {
    try {
        PLUGIN_CONFIG_TYPE cfg = PLUGIN_CONFIG_INIT;
        return new PLUGIN_PLANNER_CLASS(cfg);
    } catch (...) {
        return nullptr;
    }
}

GRACE_PLUGIN_EXPORT void destroy_planner(IPlanner* p) {
    delete p;
}

GRACE_PLUGIN_EXPORT int grace_plugin_abi() {
    return kGracePluginAbiVersion;
}

GRACE_PLUGIN_EXPORT const char* grace_plugin_id() {
    return PLUGIN_ID_STR;
}

GRACE_PLUGIN_EXPORT int grace_plugin_environment() {
    return PLUGIN_ENVIRONMENT;
}

#undef GRACE_PLUGIN_EXPORT