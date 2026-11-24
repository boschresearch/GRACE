// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <fstream>
#include <iomanip>
#include "../sim/ISimObserver.h"

namespace grace::sim {

    class MetricsCsvObserver : public grace::sim::ISimObserver {
    public:
        explicit MetricsCsvObserver(std::filesystem::path path,
                                    bool overwrite = true)
          : m_path(std::move(path)),
            m_overwrite(overwrite) {}

        void onRunStart(const BaseEnvironment&, const RobotSystem&) override {
            openIfNeeded();
        }

        void onRunEnd(const grace::sim::SimRunSummary& summary) override {
            if (!m_open) return;
            const auto& m = summary.metrics;
            m_out << std::fixed << std::setprecision(6)
                  << m.agents << ','
                  << (m.plan_success ? 1 : 0) << ','
                  << (std::isnan(m.planner_runtime_s) ? 0.0 : m.planner_runtime_s) << ','
                  << (std::isnan(m.sim_runtime_s) ? 0.0 : m.sim_runtime_s) << ','
                  << (std::isnan(m.makespan_s) ? 0.0 : m.makespan_s) << ','
                  << (std::isnan(m.sum_of_cost_s) ? 0.0 : m.sum_of_cost_s) << ','
                  << m.sum_length_m << ','
                  << (std::isnan(m.avg_completion_s) ? 0.0 : m.avg_completion_s) << ','
                  << (std::isnan(m.avg_length_m) ? 0.0 : m.avg_length_m)
                  << '\n';
            m_out.flush();
        }

    private:
        std::filesystem::path m_path;
        bool m_overwrite{true};

        std::ofstream m_out;
        bool          m_open{false};

        void openIfNeeded() {
            if (m_open) return;
            std::ios::openmode mode = m_overwrite ? std::ios::trunc : std::ios::app;
            m_out.open(m_path, mode);
            if (!m_out) { m_open = false; return; }
            m_out << "agents,plan_success,planner_runtime_s,sim_runtime_s,"
                     "makespan_s,sum_of_cost_s,sum_length_m,avg_completion_s,avg_length_m\n";
            m_out.flush();
            m_open = true;
        }
    };

} // namespace grace::io
