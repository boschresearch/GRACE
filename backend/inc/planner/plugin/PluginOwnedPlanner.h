// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "../Planner.h"      // IPlanner / PlanResultPtr
#include "PluginLoader.h"
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

class PluginOwnedPlanner final : public IPlanner {
public:
    explicit PluginOwnedPlanner(PluginPlannerPtr implementation)
        : implementation_(std::move(implementation)) {
        if (!implementation_) {
            throw std::invalid_argument(
                "PluginOwnedPlanner requires an implementation");
        }
    }

    ~PluginOwnedPlanner() override = default;

    const char* name() const noexcept override {
        return implementation_->name();
    }

    EnvironmentType type() const noexcept override {
        return implementation_->type();
    }

    PlanResult plan(const PlanRequest& req) override {
        return invokePluginOperation(
            "plan", [&] { return implementation_->plan(req); });
    }

    void configure(const PlannerSettings& s) override {
        invokePluginOperation(
            "configure", [&] { implementation_->configure(s); });
    }

private:
    template <typename Operation>
    static std::invoke_result_t<Operation> invokePluginOperation(
        const char* operationName,
        Operation&& operation) {
        using Result = std::invoke_result_t<Operation>;
        std::string error;
        try {
            if constexpr (std::is_void_v<Result>) {
                std::forward<Operation>(operation)();
                return;
            } else {
                return std::forward<Operation>(operation)();
            }
        } catch (const std::exception& exception) {
            error = std::string("Plugin ") + operationName +
                    " threw an exception: " + exception.what();
        } catch (...) {
            error = std::string("Plugin ") + operationName +
                    " threw a non-standard exception";
        }
        throw std::runtime_error(std::move(error));
    }

    PluginPlannerPtr implementation_;
};
