// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/io/ContinuousYaml.h"
#include "../../inc/util/Logger.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>

namespace {
    template <typename T>
    static bool asVec2(const YAML::Node& n, T& x, T& y) {
        if (!n || !n.IsSequence() || n.size() < 2) return false;
        try {
            x = static_cast<T>(n[0].as<double>());
            y = static_cast<T>(n[1].as<double>());
        } catch (...) { return false; }
        return true;
    }

    static std::string lower(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return char(std::tolower(c)); });
        return s;
    }
}

namespace grace::sim {

    ProblemYaml loadProblemYaml(const std::filesystem::path& yamlPath)
    {
        ProblemYaml out;

        YAML::Node root;
        try {
            root = YAML::LoadFile(yamlPath.string());
        } catch (const std::exception& e) {
            Logger::instance().error(std::string("ProblemYamlIO: failed to load '")
                                     + yamlPath.string() + "': " + e.what());
            return out;
        }

        auto env = root["environment"];
        if (!env) {
            Logger::instance().error("ProblemYamlIO: missing 'environment' node.");
            return out;
        }

        double minx=0, miny=0, maxx=0, maxy=0;
        if (!asVec2(env["min"], minx, miny) || !asVec2(env["max"], maxx, maxy)) {
            Logger::instance().error("ProblemYamlIO: environment.min/environment.max invalid.");
            return out;
        }
        out.min = {static_cast<float>(minx), static_cast<float>(miny)};
        out.max = {static_cast<float>(maxx), static_cast<float>(maxy)};

        // -------- obstacles (box | circle | polygon | capsule) --------
        if (auto obs = env["obstacles"]; obs && obs.IsSequence()) {
            for (const auto& o : obs) {
                const auto ty = (o["type"] ? lower(o["type"].as<std::string>()) : "");
                if (ty.empty()) {
                    Logger::instance().warn("ProblemYamlIO: obstacle missing 'type'. Skipped.");
                    continue;
                }

                if (ty == "box") {
                    double cx=0, cy=0, w=0, h=0;
                    if (!asVec2(o["center"], cx, cy) || !asVec2(o["size"], w, h)) {
                        Logger::instance().warn("ProblemYamlIO: malformed box obstacle. Skipped.");
                        continue;
                    }
                    ObstacleYaml ob;
                    ob.kind   = ObstacleKind::Box;
                    ob.center = {float(cx), float(cy)};
                    ob.size   = {float(w),  float(h)};
                    out.obstacles.push_back(std::move(ob));
                }
                else if (ty == "circle") {
                    double cx=0, cy=0; double r=0;
                    if (!asVec2(o["center"], cx, cy) || !o["radius"]) {
                        Logger::instance().warn("ProblemYamlIO: malformed circle obstacle. Skipped.");
                        continue;
                    }
                    try { r = o["radius"].as<double>(); } catch(...) { r = 0; }
                    if (r <= 0) {
                        Logger::instance().warn("ProblemYamlIO: circle radius <= 0. Skipped.");
                        continue;
                    }
                    ObstacleYaml ob;
                    ob.kind   = ObstacleKind::Circle;
                    ob.center = {float(cx), float(cy)};
                    ob.radius = float(r);
                    out.obstacles.push_back(std::move(ob));
                }
                else if (ty == "polygon") {
                    auto pts = o["points"];
                    if (!pts || !pts.IsSequence() || pts.size() < 3) {
                        Logger::instance().warn("ProblemYamlIO: polygon needs >= 3 points. Skipped.");
                        continue;
                    }
                    ObstacleYaml ob;
                    ob.kind = ObstacleKind::Polygon;
                    ob.points.reserve(pts.size());
                    for (const auto& p : pts) {
                        double px=0, py=0;
                        if (!asVec2(p, px, py)) { ob.points.clear(); break; }
                        ob.points.push_back({float(px), float(py)});
                    }
                    if (ob.points.size() < 3) {
                        Logger::instance().warn("ProblemYamlIO: polygon contains invalid points. Skipped.");
                        continue;
                    }
                    out.obstacles.push_back(std::move(ob));
                }
                else if (ty == "capsule") {
                    double x0=0,y0=0,x1=0,y1=0; double r=0;
                    if (!asVec2(o["p0"], x0, y0) || !asVec2(o["p1"], x1, y1) || !o["radius"]) {
                        Logger::instance().warn("ProblemYamlIO: capsule missing p0/p1/radius. Skipped.");
                        continue;
                    }
                    try { r = o["radius"].as<double>(); } catch(...) { r = 0; }
                    if (r <= 0) {
                        Logger::instance().warn("ProblemYamlIO: capsule radius <= 0. Skipped.");
                        continue;
                    }
                    ObstacleYaml ob;
                    ob.kind  = ObstacleKind::Capsule;
                    ob.p0    = {float(x0), float(y0)};
                    ob.p1    = {float(x1), float(y1)};
                    ob.radius= float(r);
                    out.obstacles.push_back(std::move(ob));
                }
                else {
                    Logger::instance().warn("ProblemYamlIO: unknown obstacle type '" + ty + "'. Skipped.");
                }
            }
        }

        // robots
        auto bots = root["robots"];
        if (!bots || !bots.IsSequence()) {
            Logger::instance().warn("ProblemYamlIO: 'robots' missing or not a list.");
            return out; // environment is still valid; caller can decide
        }

        for (const auto& r : bots) {
            RobotYamlSpec rs{};
            if (!r["type"]) {
                Logger::instance().warn("ProblemYamlIO: robot missing 'type'. Skipped.");
                continue;
            }
            rs.type = r["type"].as<std::string>();

            double sx=0, sy=0, gx=0, gy=0;
            if (!asVec2(r["start"], sx, sy) || !asVec2(r["goal"], gx, gy)) {
                Logger::instance().warn("ProblemYamlIO: robot missing/invalid start/goal (need x,y). Skipped.");
                continue;
            }
            rs.start = {float(sx), float(sy)};
            rs.goal  = {float(gx), float(gy)};

            // Optional footprint override
            if (auto f = r["footprint"]) {
                RobotFootprintYaml fp;
                const std::string shape = f["shape"] ? lower(f["shape"].as<std::string>()) : "";
                if (shape == "circle") {
                    if (!f["radius"]) {
                        Logger::instance().warn("ProblemYamlIO: 'footprint: circle' missing 'radius'. Ignored.");
                    } else {
                        fp.kind   = RobotFootprintKind::Circle;
                        fp.radius = static_cast<float>(f["radius"].as<double>());
                        rs.footprint = fp;
                    }
                } else if (shape == "box") {
                    double sx=0, sy=0;
                    if (!asVec2(f["size"], sx, sy)) {
                        Logger::instance().warn("ProblemYamlIO: 'footprint: box' missing/invalid 'size'. Ignored.");
                    } else {
                        fp.kind = RobotFootprintKind::Box;
                        fp.size = b2Vec2{float(sx), float(sy)};
                        rs.footprint = fp;
                    }
                } else if (shape == "multibox") {
                    if (!f["boxes"] || !f["boxes"].IsSequence()) {
                        Logger::instance().warn("ProblemYamlIO: 'footprint: multibox' missing 'boxes'. Ignored.");
                    } else {
                        fp.kind = RobotFootprintKind::MultiBox;
                        for (const auto& b : f["boxes"]) {
                            double cx=0, cy=0, sx=0, sy=0;
                            float ang = 0.0f;
                            if (!asVec2(b["center"], cx, cy) || !asVec2(b["size"], sx, sy)) {
                                Logger::instance().warn("ProblemYamlIO: multibox part missing 'center' or 'size'. Skipped part.");
                                continue;
                            }
                            if (b["angle"]) ang = static_cast<float>(b["angle"].as<double>());
                            fp.boxes.push_back(RobotFootprintYaml::BoxPart{
                                b2Vec2{float(cx), float(cy)}, b2Vec2{float(sx), float(sy)}, ang
                            });
                        }
                        if (!fp.boxes.empty()) rs.footprint = fp;
                    }
                } else if (!shape.empty()) {
                    Logger::instance().warn("ProblemYamlIO: unknown footprint shape '" + shape + "'. Ignored.");
                }
            }

            out.robots.push_back(std::move(rs));
        }

        return out;
    }

    static YAML::Node footprintYaml(const std::optional<RobotFootprintYaml>& f) {
        YAML::Node fp;
        if (!f) return fp;
        switch (f->kind) {
            case RobotFootprintKind::Circle:
                fp["shape"]  = "circle";
                fp["radius"] = f->radius;
                break;
            case RobotFootprintKind::Box: {
                fp["shape"] = "box";
                YAML::Node sz(YAML::NodeType::Sequence);
                sz.SetStyle(YAML::EmitterStyle::Flow);
                sz.push_back(f->size.x);
                sz.push_back(f->size.y);
                fp["size"] = sz;
                break;
            }
            case RobotFootprintKind::MultiBox: {
                fp["shape"] = "multibox";
                YAML::Node boxes(YAML::NodeType::Sequence);
                for (const auto& b : f->boxes) {
                    YAML::Node one;
                    YAML::Node c(YAML::NodeType::Sequence); c.SetStyle(YAML::EmitterStyle::Flow);
                    c.push_back(b.center.x); c.push_back(b.center.y);
                    YAML::Node s(YAML::NodeType::Sequence); s.SetStyle(YAML::EmitterStyle::Flow);
                    s.push_back(b.size.x); s.push_back(b.size.y);
                    one["center"] = c;
                    one["size"]   = s;
                    one["angle"]  = b.angle;
                    boxes.push_back(one);
                }
                fp["boxes"] = boxes;
                break;
            }
            default: break;
        }
        return fp;
    }

    std::string emitRobotsMetaYaml(const std::vector<RobotYamlSpec>& robots) {
        YAML::Node meta(YAML::NodeType::Sequence);
        meta.SetStyle(YAML::EmitterStyle::Block);
        for (const auto& r : robots) {
            YAML::Node one;
            one["type"] = r.type;
            if (r.footprint) {
                auto fp = footprintYaml(r.footprint);
                if (fp) one["footprint"] = fp;
            }
            meta.push_back(one);
        }
        YAML::Emitter out; out << meta;
        return std::string(out.c_str());
    }

    std::string emitRobotsMetaYaml(const RobotSystem& reg) {
        YAML::Node meta(YAML::NodeType::Sequence);
        meta.SetStyle(YAML::EmitterStyle::Block);
        for (b2BodyId id : reg.robots()) {
            const RobotMeta* m = reg.meta(id);
            if (!m) continue;
            YAML::Node one;
            // Backwards-compat: if we don't know the type, write only footprint.
            YAML::Node fp;
            fp["shape"]  = "circle";
            fp["radius"] = std::max(0.0f, m->effectiveRadius);
            one["footprint"] = fp;
            meta.push_back(one);
        }
        YAML::Emitter out; out << meta;
        return std::string(out.c_str());
    }

    static YAML::Node seq2(double a, double b) {
        YAML::Node n(YAML::NodeType::Sequence);
        n.SetStyle(YAML::EmitterStyle::Flow);
        n.push_back(a); n.push_back(b);
        return n;
    }

    std::string emitObstaclesYamlLocal(const ProblemYaml& spec, b2Vec2 mins) {
        auto toLocal = [&](b2Vec2 p) { return b2Vec2{ p.x - mins.x, p.y - mins.y }; };
        YAML::Node obsNode(YAML::NodeType::Sequence);
        for (const auto& o : spec.obstacles) {
            YAML::Node n;
            switch (o.kind) {
                case ObstacleKind::Box:
                    n["type"]   = "box";
                    n["center"] = seq2(toLocal(o.center).x, toLocal(o.center).y);
                    n["size"]   = seq2(o.size.x, o.size.y);
                    break;
                case ObstacleKind::Circle:
                    n["type"]   = "circle";
                    n["center"] = seq2(toLocal(o.center).x, toLocal(o.center).y);
                    n["radius"] = o.radius;
                    break;
                case ObstacleKind::Polygon: {
                    n["type"] = "polygon";
                    YAML::Node pts(YAML::NodeType::Sequence);
                    pts.SetStyle(YAML::EmitterStyle::Block);
                    for (const auto& pW : o.points) {
                        auto pL = toLocal(pW);
                        pts.push_back(seq2(pL.x, pL.y));
                    }
                    n["points"] = pts;
                    break;
                }
                case ObstacleKind::Capsule:
                    n["type"]  = "capsule";
                    n["p0"]     = seq2(toLocal(o.p0).x, toLocal(o.p0).y);
                    n["p1"]     = seq2(toLocal(o.p1).x, toLocal(o.p1).y);
                    n["radius"] = o.radius;
                    break;
            }
            obsNode.push_back(n);
        }
        YAML::Emitter out; out << obsNode;
        return std::string(out.c_str());
    }

}