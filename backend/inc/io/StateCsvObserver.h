// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <fstream>
#include <vector>
#include <iomanip>
#include <optional>
#include <box2d/box2d.h>
#include "../sim/ISimObserver.h"
#include "../robot/RobotSystem.h"

namespace grace::sim {

    class StateCsvObserver : public grace::sim::ISimObserver {
    public:
        StateCsvObserver(std::filesystem::path path,
                         bool overwrite = true,
                         int everyNSteps = 1,
                         std::vector<b2BodyId> watch = {})
          : m_path(std::move(path)),
            m_overwrite(overwrite),
            m_everyNSteps(everyNSteps),
            m_watch(std::move(watch)) {}

        void onRunStart(const BaseEnvironment& env, const RobotSystem& robots) override {
            (void)env; (void)robots;
            openIfNeeded();
        }

        void onPostStep(const grace::sim::SimStepContext& ctx) override {
            if (!m_open) return;
            if (m_everyNSteps > 1 && (ctx.stepCount % m_everyNSteps) != 0) return;

            for (b2BodyId id : ctx.robots.robots()) {
                if (!isWatched(id)) continue;
                const RobotMeta* meta = ctx.robots.meta(id);
                if (!meta) continue;
                const b2Vec2 pos = b2Body_GetPosition(id);
                const float  ang = b2Rot_GetAngle(b2Body_GetRotation(id));
                const b2Vec2 vel = b2Body_GetLinearVelocity(id);

                m_out << std::fixed << std::setprecision(6)
                      << ctx.simTime << ','
                      << '"' << meta->name << '"' << ','
                      << pos.x << ',' << pos.y << ','
                      << ang   << ','
                      << vel.x << ',' << vel.y << ','
                      << meta->next_wp << ',' << meta->plan.waypoints.size()
                      << '\n';
            }
            m_out.flush();
        }

    private:
        std::filesystem::path m_path;
        bool  m_overwrite{true};
        int   m_everyNSteps{1};
        std::vector<b2BodyId> m_watch;

        std::ofstream m_out;
        bool          m_open{false};

        void openIfNeeded() {
            if (m_open) return;
            std::ios::openmode mode = m_overwrite ? std::ios::trunc : std::ios::app;
            m_out.open(m_path, mode);
            if (!m_out) { m_open = false; return; }
            m_out << "time,name,x,y,angle,vx,vy,next_wp,total_wp\n";
            m_out.flush();
            m_open = true;
        }

        bool isWatched(b2BodyId id) const {
            if (m_watch.empty()) return true;
            for (auto w : m_watch) if (w.index1 == id.index1 && w.generation == id.generation) return true;
            return false;
        }
    };

} // namespace grace::io
