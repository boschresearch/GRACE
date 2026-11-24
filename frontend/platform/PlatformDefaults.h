// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace platform::defaults {

    inline const std::filesystem::path& root()
    {
        static const std::filesystem::path kRoot{GRACE_REPO_ROOT};
        return kRoot;
    }

    inline std::string dataPath(std::string_view relativePath)
    {
        return (root() / "backend/data" / relativePath).string();
    }

    inline std::string mapMovingAIDemo()
    {
        return dataPath("maps/movingAI/default/demo.map");
    }

    inline std::string scenMovingAIDemo()
    {
        return dataPath("maps/movingAI/default/demo.scen");
    }

    inline std::string agentsWarehouseLarge()
    {
        return dataPath("maps/movingAI/large/warehouse_large_8000.agents");
    }

    inline std::string tasksWarehouseLarge()
    {
        return dataPath("maps/movingAI/large/warehouse_large.tasks");
    }

    inline std::string mapRoadmapDemo()
    {
        return dataPath("maps/xml/default/roadmap/demo_roadmap.xml");
    }

    inline std::string taskRoadmapDemo()
    {
        return dataPath("maps/xml/default/roadmap/demo_tasks.xml");
    }

    inline std::string problemYamlDemo()
    {
        return dataPath("maps/yaml/default/demo.yaml");
    }

} // namespace platform::defaults
