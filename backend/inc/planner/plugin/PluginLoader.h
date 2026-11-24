// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "plugin_api.h"

#include <memory>
#include <string>
#include <utility>

class PluginHandle;

struct PluginPlannerDeleter {
    destroy_planner_fn destroy{nullptr};
    std::shared_ptr<const PluginHandle> library;

    void operator()(IPlanner* planner) const noexcept;
};

using PluginPlannerPtr =
    std::unique_ptr<IPlanner, PluginPlannerDeleter>;

class PluginHandle
    : public std::enable_shared_from_this<PluginHandle> {
public:
    static std::shared_ptr<PluginHandle> load(
        const std::string& libraryPath,
        std::string* error = nullptr);
    ~PluginHandle();

    [[nodiscard]] PluginPlannerPtr createPlanner() const;
    [[nodiscard]] const std::string& path() const noexcept { return path_; }
    [[nodiscard]] const std::string& plannerId() const noexcept {
        return plannerId_;
    }
    [[nodiscard]] GracePluginEnvironment environment() const noexcept {
        return environment_;
    }

private:
    void* handle_{nullptr};
    create_planner_fn create_{nullptr};
    destroy_planner_fn destroy_{nullptr};
    std::string path_;
    std::string plannerId_;
    GracePluginEnvironment environment_;

    explicit PluginHandle(
        void* handle,
        create_planner_fn create,
        destroy_planner_fn destroy,
        std::string path,
        std::string plannerId,
        GracePluginEnvironment environment)
        : handle_(handle),
          create_(create),
          destroy_(destroy),
          path_(std::move(path)),
          plannerId_(std::move(plannerId)),
          environment_(environment) {}

    PluginHandle(const PluginHandle&) = delete;
    PluginHandle& operator=(const PluginHandle&) = delete;
};
