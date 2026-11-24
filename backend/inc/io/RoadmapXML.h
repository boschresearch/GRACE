// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>
#include "../env/RoadmapEnvironment.h"

namespace grace::sim {

    struct GraphMLOptions {
        float scale = 1.0f;        // coords scale (e.g. px->m)
        bool  undirected = false;  // if true, add both directions
        bool  useEdgeWeight = false; // read <data key="weight">
    };

    struct GraphMLImport {
        std::unordered_map<std::string, IRoadmap::NodeId> idToNode; // "n136" -> NodeId
        std::vector<std::string> nodeToId;                          // inverse index -> "n<i>"
        std::string nodeCoordsKey; // e.g., "key0"
        std::string edgeWeightKey; // e.g., "key1"
    };

    bool readGraphML(const std::filesystem::path& path,
                     RoadmapEnvironment& out,
                     GraphMLImport* import,
                     const GraphMLOptions& opt = {});

    bool writeGraphML(const RoadmapEnvironment& R,
                      const std::filesystem::path& outPath,
                      bool writeWeights = false,
                      float coordScale = 1.0f);

    // Parse <root><agent start_id="136" goal_id="50"/>…</root>
    // start_id/goal_id refer to GraphML ids WITHOUT the 'n' prefix.
    std::vector<std::pair<IRoadmap::NodeId, IRoadmap::NodeId>>
    readTasksXml(const std::filesystem::path& path, const GraphMLImport& import);

} // namespace grace::io
