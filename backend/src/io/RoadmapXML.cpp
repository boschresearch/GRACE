// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/io/RoadmapXML.h"
#include "../../inc/util/Logger.h"
#include <pugixml.hpp>
#include <sstream>
#include <charconv>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <fstream>

namespace {

  inline bool parseXY(const std::string& s, float& x, float& y) {
    const char* p = s.c_str();
    char* end=nullptr;
    x = std::strtof(p, &end);
    if (!end) return false;
    while (*end==',' || std::isspace(static_cast<unsigned char>(*end))) ++end;
    char* end2=nullptr;
    y = std::strtof(end, &end2);
    return end2 != nullptr;
  }

} // anon

namespace grace::sim {

  bool readGraphML(const std::filesystem::path& path,
                   RoadmapEnvironment& out,
                   GraphMLImport* import,
                   const GraphMLOptions& opt)
  {
    pugi::xml_document doc;
    pugi::xml_parse_result ok = doc.load_file(path.string().c_str());
    if (!ok) {
      Logger::instance().error("GraphML: parse fail " + path.string() + ": " + ok.description());
      return false;
    }

    auto graphml = doc.child("graphml"); if (!graphml) graphml = doc.first_child();
    if (!graphml) { Logger::instance().error("GraphML: no <graphml>"); return false; }

    std::string coordsKey, weightKey;
    for (auto key : graphml.children("key")) {
      const std::string forAttr = key.attribute("for").as_string();
      const std::string name    = key.attribute("attr.name").as_string();
      const std::string id      = key.attribute("id").as_string();
      if (forAttr == "node" && name == "coords")  coordsKey = id;
      if (forAttr == "edge" && name == "weight")  weightKey = id;
    }

    auto graph = graphml.child("graph");
    if (!graph) {
      for (auto child : graphml.children())
        if (std::string(child.name()) == "graph") { graph = child; break; }
    }
    if (!graph) { Logger::instance().error("GraphML: no <graph>"); return false; }

    std::unordered_map<std::string, IRoadmap::NodeId> idMap;
    std::vector<std::string> inverse;

    for (auto node : graph.children("node")) {
      const char* gid = node.attribute("id").as_string(); // e.g., "n136"
      if (!gid || !*gid) continue;

      float x=0.f, y=0.f; bool haveCoords=false;
      for (auto data : node.children("data")) {
        const char* key = data.attribute("key").as_string();
        if (coordsKey.empty() || (key && coordsKey == key)) {
          std::string text = data.text().as_string();
          if (parseXY(text, x, y)) { haveCoords = true; break; }
        }
      }
      if (!haveCoords) {
        Logger::instance().warn(std::string("GraphML: node ") + gid + " missing coords; defaulting (0,0)");
      }
      b2Vec2 p{ x*opt.scale, y*opt.scale };
      auto nid = out.addNode(p);
      idMap.emplace(gid, nid);
      inverse.emplace_back(gid);
    }

    const bool makeUndirected = opt.undirected;
    for (auto edge : graph.children("edge")) {
      const char* su = edge.attribute("source").as_string();
      const char* sv = edge.attribute("target").as_string();
      if (!su || !sv) continue;
      auto itU = idMap.find(su), itV = idMap.find(sv);
      if (itU == idMap.end() || itV == idMap.end()) {
        Logger::instance().warn("GraphML: edge references unknown node(s): " +
          std::string(su?su:"?") + " -> " + std::string(sv?sv:"?"));
        continue;
      }
      auto u = itU->second, v = itV->second;

      float w = -1.0f; bool haveW=false;
      if (opt.useEdgeWeight && !weightKey.empty()) {
        for (auto data : edge.children("data")) {
          const char* key = data.attribute("key").as_string();
          if (key && weightKey == key) {
            w = std::strtof(data.text().as_string(), nullptr);
            haveW = true; break;
          }
        }
      }

      if (makeUndirected) {
        out.addEdge(u, v);
        if (haveW) out.setEdgeWeight(u, v, w, /*bidir=*/true);
      } else {
        out.addEdgeDirected(u, v);
        if (haveW) out.setEdgeWeight(u, v, w, /*bidir=*/false);
      }
    }

    if (import) {
      import->idToNode = std::move(idMap);
      import->nodeToId = std::move(inverse);
      import->nodeCoordsKey = coordsKey;
      import->edgeWeightKey = weightKey;
    }
    out.autoBounds(0.0f);
    return true;
  }

  bool writeGraphML(const RoadmapEnvironment& R,
                    const std::filesystem::path& outPath,
                    bool writeWeights,
                    float coordScale)
  {
    std::ofstream out(outPath);
    if (!out) {
      Logger::instance().error("GraphML: cannot write " + outPath.string());
      return false;
    }
    out.setf(std::ios::fmtflags(0), std::ios::floatfield);
    out << std::setprecision(6);

    out <<
  R"(<?xml version="1.0" encoding="UTF-8"?>
  <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
    <key id="key0" for="node" attr.name="coords" attr.type="string" />
    <key id="key1" for="edge" attr.name="weight" attr.type="double" />
    <graph id="G" edgedefault="directed" parse.nodeids="free" parse.edgeids="canonical" parse.order="nodesfirst">
  )";

    const int N = R.numNodes();
    for (int i = 0; i < N; ++i) {
      const auto p = R.nodePos(i);
      const double x = double(p.x) * double(coordScale);
      const double y = double(p.y) * double(coordScale);
      out << "    <node id=\"n" << i << "\">\n"
          << "      <data key=\"key0\">" << x << "," << y << "</data>\n"
          << "    </node>\n";
    }

    std::size_t eid = 0;
    for (int u = 0; u < N; ++u) {
      for (int v : R.adj(u)) {
        if (u == v) continue;
        const double w = writeWeights ? R.edgeLength(u, v) : 1.0;
        out << "    <edge id=\"e" << eid++ << "\" source=\"n" << u << "\" target=\"n" << v << "\">\n"
            << "      <data key=\"key1\">" << w << "</data>\n"
            << "    </edge>\n";
      }
    }

    out << "  </graph>\n</graphml>\n";
    return true;
  }

  std::vector<std::pair<IRoadmap::NodeId, IRoadmap::NodeId>>
  readTasksXml(const std::filesystem::path& path, const GraphMLImport& import)
  {
    std::vector<std::pair<IRoadmap::NodeId, IRoadmap::NodeId>> res;

    pugi::xml_document doc;
    pugi::xml_parse_result ok = doc.load_file(path.string().c_str());
    if (!ok) {
      Logger::instance().error("TaskXml: parse fail " + path.string() + ": " + ok.description());
      return res;
    }

    auto root = doc.child("root"); if (!root) root = doc.first_child();
    if (!root) { Logger::instance().error("TaskXml: no <root>"); return res; }

    for (auto ag : root.children("agent")) {
      int s = ag.attribute("start_id").as_int(-1);
      int g = ag.attribute("goal_id").as_int(-1);
      if (s < 0 || g < 0) { Logger::instance().warn("TaskXml: missing start_id/goal_id"); continue; }
      std::string sId = "n" + std::to_string(s);
      std::string gId = "n" + std::to_string(g);

      auto itS = import.idToNode.find(sId);
      auto itG = import.idToNode.find(gId);
      if (itS == import.idToNode.end() || itG == import.idToNode.end()) {
        Logger::instance().warn("TaskXml: unknown node id(s): " + sId + " -> " + gId);
        continue;
      }
      res.emplace_back(itS->second, itG->second);
    }
    return res;
  }

} // namespace grace::io
