// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <vector>
#include <memory>
#include <optional>
#include <unordered_map>
#include <string>
#include <limits>
#include <cmath>
#include <cctype>
#include <fmt/format.h>
#include "../env/BaseEnvironment.h"

struct Waypoint { b2Vec2 p{}; float t{-1.0f}; };
struct StartGoal { b2Vec2 start; b2Vec2 goal; };

struct Plan {
    std::vector<Waypoint> waypoints;     // you already defined Waypoint{ b2Vec2 p; float t=-1 }
    bool timed() const { return !waypoints.empty() && waypoints.front().t >= 0.0f; }
};

enum class PlanStatus : uint8_t { Success, Infeasible, Timeout, Error };

struct PlanDiagnostics {
    double runtime_s       = std::numeric_limits<double>::quiet_NaN();
    std::size_t expansions = 0;
    double sum_of_cost_s   = std::numeric_limits<double>::quiet_NaN();
    double makespan_s      = std::numeric_limits<double>::quiet_NaN();
    std::string detail; // free-form notes from the planner
};

struct PlanResult {
    PlanStatus status{PlanStatus::Error};
    std::vector<b2BodyId> ids;   // same order as input request.ids
    std::vector<Plan>     plans; // plans[i] corresponds to ids[i]
    PlanDiagnostics       diag{};
};

// Lightweight, generic dict for per-planner options
struct PlannerSettings {
    std::unordered_map<std::string, std::string> kv;

    template <class T>
    void set(const std::string& key, const T& value) {
           kv[key] = fmt::format("{}", value);
       }
    // returns empty optional if missing or parse fails
    template <class T>
    std::optional<T> get(const std::string& key) const;

    template <class T>
    T getOr(const std::string& key, const T& def) const {
        if (auto v = get<T>(key)) return *v;
        return def;
    }

    bool contains(const std::string& key) const {
        return kv.contains(key);
    }
};

// string specialization passthrough
template<> inline std::optional<std::string>
PlannerSettings::get<std::string>(const std::string& key) const {
      const auto it = kv.find(key);
      if (it == kv.end()) return std::nullopt;
      return it->second;
  }


// bool specialization: accepts 1/0, true/false, on/off, yes/no (case-insensitive)
template<> inline std::optional<bool>
PlannerSettings::get<bool>(const std::string& key) const {
    const auto it = kv.find(key);
    if (it == kv.end()) return std::nullopt;
    std::string s = it->second;
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (s == "1" || s == "true" || s == "on"  || s == "yes") return true;
    if (s == "0" || s == "false"|| s == "off" || s == "no")  return false;
    return std::nullopt;
}

template<> inline std::optional<int>
PlannerSettings::get<int>(const std::string& key) const {
    const auto it = kv.find(key); if (it == kv.end()) return std::nullopt;
    try { return std::stoi(it->second); } catch (...) { return std::nullopt; }
}
template<> inline std::optional<unsigned>
PlannerSettings::get<unsigned>(const std::string& key) const {
    const auto it = kv.find(key); if (it == kv.end()) return std::nullopt;
    try { return static_cast<unsigned>(std::stoul(it->second)); } catch (...) { return std::nullopt; }
}
template<> inline std::optional<std::size_t>
PlannerSettings::get<std::size_t>(const std::string& key) const {
    const auto it = kv.find(key); if (it == kv.end()) return std::nullopt;
    try { return std::stoull(it->second); } catch (...) { return std::nullopt; }
}
template<> inline std::optional<float>
PlannerSettings::get<float>(const std::string& key) const {
    const auto it = kv.find(key); if (it == kv.end()) return std::nullopt;
    try { return std::stof(it->second); } catch (...) { return std::nullopt; }
}
template<> inline std::optional<double>
PlannerSettings::get<double>(const std::string& key) const {
    const auto it = kv.find(key); if (it == kv.end()) return std::nullopt;
    try { return std::stod(it->second); } catch (...) { return std::nullopt; }
}

struct PlanRequest {
    const BaseEnvironment&       env;  // planners must treat env as read-only
    const std::vector<b2BodyId>& ids;
    const std::vector<StartGoal>& sgs;
    PlannerSettings              settings;  // planner-specific knobs
    double                       time_limit_s{INFINITY};
    std::optional<std::uint64_t> rng_seed{0};         // optional seed for determinism
};

inline double resolvePlannerTimeLimit(
    const double requested,
    const double configured) noexcept {
    return requested > 0.0 && std::isfinite(requested) ? requested : configured;
}

class IPlanner {
public:
    virtual ~IPlanner() = default;
    virtual const char* name() const noexcept = 0;
    virtual EnvironmentType type() const noexcept = 0;
    virtual PlanResult plan(const PlanRequest& req) = 0;
    virtual void configure(const PlannerSettings&) {} // optional preconfig

    struct RunResult {bool ok{false}; bool timed_out{false}; int  exit_code{0};};
};
