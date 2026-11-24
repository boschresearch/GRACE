// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "planner/plugin/PluginLoader.h"

#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace {

#if defined(_WIN32)
void* openLibrary(const char* path) {
    return reinterpret_cast<void*>(LoadLibraryA(path));
}

void* loadSymbol(void* handle, const char* name) {
    return reinterpret_cast<void*>(
        GetProcAddress(static_cast<HMODULE>(handle), name));
}

std::string libraryError() {
    return "LoadLibrary/GetProcAddress failed";
}

void closeLibrary(void* handle) {
    if (handle) {
        FreeLibrary(static_cast<HMODULE>(handle));
    }
}
#else
void* openLibrary(const char* path) {
    return dlopen(path, RTLD_NOW | RTLD_LOCAL);
}

void* loadSymbol(void* handle, const char* name) {
    return dlsym(handle, name);
}

std::string libraryError() {
    const char* error = dlerror();
    return error ? error : "dynamic library operation failed";
}

void closeLibrary(void* handle) {
    if (handle) {
        dlclose(handle);
    }
}
#endif

void setError(std::string* destination, std::string message) {
    if (destination) {
        *destination = std::move(message);
    }
}

bool validEnvironment(const int environment) {
    return environment == GRACE_PLUGIN_ENVIRONMENT_GRID ||
           environment == GRACE_PLUGIN_ENVIRONMENT_CONTINUOUS ||
           environment == GRACE_PLUGIN_ENVIRONMENT_ROADMAP;
}

template <typename Callback, typename Result>
bool invokePluginCallback(
    const char* callbackName,
    Callback callback,
    Result& result,
    std::string& error) {
    try {
        result = callback();
        return true;
    } catch (const std::exception& exception) {
        error = std::string(callbackName) +
                " threw an exception: " + exception.what();
    } catch (...) {
        error = std::string(callbackName) +
                " threw a non-standard exception";
    }
    return false;
}

}  // namespace

void PluginPlannerDeleter::operator()(IPlanner* planner) const noexcept {
    if (planner && destroy) {
        try {
            destroy(planner);
        } catch (const std::exception& exception) {
            std::fprintf(
                stderr,
                "GRACE plugin destroy_planner threw an exception: %s\n",
                exception.what());
        } catch (...) {
            std::fputs(
                "GRACE plugin destroy_planner threw a non-standard exception\n",
                stderr);
        }
    }
}

std::shared_ptr<PluginHandle> PluginHandle::load(
    const std::string& libraryPath,
    std::string* error) {
    if (error) {
        error->clear();
    }

    std::unique_ptr<void, decltype(&closeLibrary)> library(
        openLibrary(libraryPath.c_str()),
        &closeLibrary);
    if (!library) {
        setError(error, libraryError());
        return {};
    }

    const auto fail = [&](std::string message) {
        setError(error, std::move(message));
        return std::shared_ptr<PluginHandle>{};
    };

    const auto abi = reinterpret_cast<plugin_abi_fn>(
        loadSymbol(library.get(), "grace_plugin_abi"));
    if (!abi) {
        return fail("Missing grace_plugin_abi");
    }
    std::string callbackError;
    int abiVersion = 0;
    if (!invokePluginCallback(
            "grace_plugin_abi",
            abi,
            abiVersion,
            callbackError)) {
        return fail(std::move(callbackError));
    }
    if (abiVersion != kGracePluginAbiVersion) {
        return fail(
            "Plugin ABI mismatch: expected " +
            std::to_string(kGracePluginAbiVersion) + ", received " +
            std::to_string(abiVersion));
    }

    const auto id = reinterpret_cast<plugin_id_fn>(
        loadSymbol(library.get(), "grace_plugin_id"));
    const auto environment = reinterpret_cast<plugin_environment_fn>(
        loadSymbol(library.get(), "grace_plugin_environment"));
    const auto create = reinterpret_cast<create_planner_fn>(
        loadSymbol(library.get(), "create_planner"));
    const auto destroy = reinterpret_cast<destroy_planner_fn>(
        loadSymbol(library.get(), "destroy_planner"));
    if (!id || !environment || !create || !destroy) {
        return fail(
            "Missing required plugin metadata or lifecycle symbol");
    }

    const char* plannerId = nullptr;
    if (!invokePluginCallback(
            "grace_plugin_id",
            id,
            plannerId,
            callbackError)) {
        return fail(std::move(callbackError));
    }
    if (!plannerId || *plannerId == '\0') {
        return fail("Plugin returned an empty planner ID");
    }
    int advertisedEnvironment = 0;
    if (!invokePluginCallback(
            "grace_plugin_environment",
            environment,
            advertisedEnvironment,
            callbackError)) {
        return fail(std::move(callbackError));
    }
    if (!validEnvironment(advertisedEnvironment)) {
        return fail("Plugin returned an invalid environment capability");
    }

    std::string validatedPath = libraryPath;
    std::string validatedPlannerId = plannerId;
    auto plugin = std::unique_ptr<PluginHandle>(new PluginHandle(
        library.get(),
        create,
        destroy,
        std::move(validatedPath),
        std::move(validatedPlannerId),
        static_cast<GracePluginEnvironment>(advertisedEnvironment)));
    library.release();
    return std::shared_ptr<PluginHandle>(std::move(plugin));
}

PluginPlannerPtr PluginHandle::createPlanner() const {
    auto library = shared_from_this();
    std::string callbackError;
    IPlanner* planner = nullptr;
    if (!invokePluginCallback(
            "create_planner",
            create_,
            planner,
            callbackError)) {
        throw std::runtime_error(std::move(callbackError));
    }
    return PluginPlannerPtr{
        planner,
        PluginPlannerDeleter{destroy_, std::move(library)},
    };
}

PluginHandle::~PluginHandle() {
    closeLibrary(handle_);
}
