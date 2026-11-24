// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/env/RoadmapEnvironment.h"
#include "../inc/util/Logger.h"
#include <limits>
#include <cmath>
#include <algorithm>


void RoadmapEnvironment::autoBounds(float margin)
{
    if (m_nodes.empty()) { m_width = m_height = 0.0f; return; }

    float minx =  std::numeric_limits<float>::infinity();
    float miny =  std::numeric_limits<float>::infinity();
    float maxx = -std::numeric_limits<float>::infinity();
    float maxy = -std::numeric_limits<float>::infinity();
    for (const auto& p : m_nodes) {
        minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
    }
    // Platform origin is (0,0) top-left, x→right, y→down. We don't translate nodes,
    // so we clamp to [0, max] to keep the walls at x∈[0,W], y∈[0,H].
    if (!std::isfinite(maxx) || !std::isfinite(maxy)) { m_width = m_height = 0.0f; return; }
    const float W = std::max(0.0f, maxx) + margin;
    const float H = std::max(0.0f, maxy) + margin;
    setBounds(W, H);
}

IRoadmap::NodeId RoadmapEnvironment::addNode(b2Vec2 p) {
    m_nodes.push_back(p);
    m_adj.emplace_back();
    return static_cast<NodeId>(m_nodes.size() - 1);
}

void RoadmapEnvironment::addEdge(NodeId u, NodeId v) {
    if (u<0 || v<0 || u>=numNodes() || v>=numNodes() || u==v) {
        Logger::instance().error("MAPFRoadmapEnv::addEdge: invalid node id");
        return;
    }
    auto &Au = m_adj[static_cast<size_t>(u)];
    auto &Av = m_adj[static_cast<size_t>(v)];
    if (std::find(Au.begin(), Au.end(), v) == Au.end()) Au.push_back(v);
    if (std::find(Av.begin(), Av.end(), u) == Av.end()) Av.push_back(u);
}

float RoadmapEnvironment::edgeLength(NodeId u, NodeId v) const {
    // Prefer directed weight, then undirected, else Euclidean.
    if (auto itD = m_w_dir.find(keyDir(u,v)); itD != m_w_dir.end()) return itD->second;
    if (auto itU = m_w_undir.find(keyUndir(u,v)); itU != m_w_undir.end()) return itU->second;

    const auto a = nodePos(u), b = nodePos(v);
    const float dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy);
}

IRoadmap::NodeId RoadmapEnvironment::nearestNode(b2Vec2 p) const {
    float best = std::numeric_limits<float>::infinity();
    NodeId bestId = -1;
    for (NodeId i=0; i<numNodes(); ++i) {
        const auto q = nodePos(i);
        const float dx=p.x-q.x, dy=p.y-q.y;
        const float d2 = dx*dx+dy*dy;
        if (d2 < best) { best = d2; bestId = i; }
    }
    return bestId;
}

void RoadmapEnvironment::addEdgeDirected(NodeId u, NodeId v) {
    if (u<0 || v<0 || u>=numNodes() || v>=numNodes() || u==v) {
            Logger::instance().error("MAPFRoadmapEnv::addEdgeDirected: invalid node id");
            return;
        }
    auto &Au = m_adj[static_cast<size_t>(u)];
    if (std::find(Au.begin(), Au.end(), v) == Au.end()) Au.push_back(v);
    }

void RoadmapEnvironment::setEdgeWeight(NodeId u, NodeId v, float w, bool bidir) {
    if (u<0 || v<0 || u>=numNodes() || v>=numNodes() || u==v) {
        Logger::instance().error("MAPFRoadmapEnv::setEdgeWeight: invalid node id");
        return;
    }
    if (bidir) {
        m_w_undir[keyUndir(u,v)] = w;
        }
    else {
        m_w_dir[keyDir(u,v)] = w;
        }
    }

void RoadmapEnvironment::resetDerivedState() noexcept {
    m_width = 0.0f;
    m_height = 0.0f;
    m_nodes.clear();
    m_adj.clear();
    m_w_dir.clear();
    m_w_undir.clear();
}