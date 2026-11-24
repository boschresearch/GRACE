// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include "BaseEnvironment.h"
#include "ICapabilities.h"
#include <vector>
#include <unordered_map>

class RoadmapEnvironment : public BaseEnvironment, public IRoadmap {
public:
    EnvironmentType getEnvironmentType() const override { return EnvironmentType::RoadmapEnvironment; }
    Bounds  boundsInMeter() const override { return {m_width, m_height}; }
    const IRoadmap* roadmapCapability() const noexcept override { return this; }

    // Build API
    void setBounds(float W, float H) { m_width=W; m_height=H; }

    // Compute bounds from current node coordinates. Does not shift nodes; origin stays at (0,0).
    // If some nodes are negative, we clamp to [0, max] in each axis.
    void autoBounds(float margin = 0.0f);

    NodeId addNode(b2Vec2 p);
    void   addEdge(NodeId u, NodeId v);

    // IRoadmap
    int    numNodes() const override { return static_cast<int>(m_nodes.size()); }
    b2Vec2 nodePos(NodeId n) const override { return m_nodes[static_cast<size_t>(n)]; }
    const std::vector<NodeId>& adj(NodeId n) const override { return m_adj[static_cast<size_t>(n)]; }
    float  edgeLength(NodeId u, NodeId v) const override;
    NodeId nearestNode(b2Vec2 p) const override;

    // Directed edge (u -> v). Use addEdge(u,v) for undirected.
    void   addEdgeDirected(NodeId u, NodeId v);
    // Optional edge weight override (used by edgeLength if present).
    void   setEdgeWeight(NodeId u, NodeId v, float w, bool bidir);
private:
    float m_width{0.0f}, m_height{0.0f};
    std::vector<b2Vec2> m_nodes;
    std::vector<std::vector<NodeId>> m_adj;

    // packed ids; keep separate maps to avoid collisions
    // Undirected: keyUndir(min(u,v), max(u,v))  -> weight
    // Directed:   keyDir(u, v)                  -> weight
    std::unordered_map<uint64_t, float> m_w_dir;
    std::unordered_map<uint64_t, float> m_w_undir;

    void resetDerivedState() noexcept override;

    static uint64_t keyUndir(NodeId u, NodeId v) {
            uint32_t a = static_cast<uint32_t>(std::min(u,v));
            uint32_t b = static_cast<uint32_t>(std::max(u,v));
            return (uint64_t(a) << 32) | uint64_t(b);
        }
    static uint64_t keyDir(NodeId u, NodeId v) {
            return (uint64_t(static_cast<uint32_t>(u)) << 32) | uint64_t(static_cast<uint32_t>(v));
        }
};