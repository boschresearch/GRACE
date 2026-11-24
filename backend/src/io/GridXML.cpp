// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/io/GridXML.h"
#include "../../inc/io/GridStartGoal.h"
#include <fstream>
#include <sstream>
#include <limits>
#include <new>
#include <stdexcept>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>


namespace grace::io {

    ReadResult<GridMapData> readGridXmlMap(const std::filesystem::path& path)
    {
        boost::property_tree::ptree pt;
        try {
            read_xml(
                path.string(), pt,
                boost::property_tree::xml_parser::trim_whitespace);
        } catch (const boost::property_tree::xml_parser::xml_parser_error& error) {
            return ReadResult<GridMapData>::failed(
                "XML grid: parse failed for '" + path.string() +
                "': " + error.what());
        }

        const auto map = pt.get_child_optional("root.map");
        if (!map) {
            return ReadResult<GridMapData>::failed(
                "XML grid: missing <root><map>");
        }
        const auto width = map->get_optional<long long>("width");
        const auto height = map->get_optional<long long>("height");
        const auto grid = map->get_child_optional("grid");
        if (!width || !height || !grid ||
            *width <= 0 || *height <= 0 ||
            *width > std::numeric_limits<int>::max() ||
            *height > std::numeric_limits<int>::max()) {
            return ReadResult<GridMapData>::failed(
                "XML grid: missing or invalid dimensions/grid");
        }
        const auto widthSize = static_cast<std::size_t>(*width);
        const auto heightSize = static_cast<std::size_t>(*height);
        if (heightSize > std::numeric_limits<std::size_t>::max() / widthSize ||
            widthSize * heightSize >
                static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            return ReadResult<GridMapData>::failed(
                "XML grid: dimensions exceed the supported cell count");
        }

        GridMapData data;
        data.width = static_cast<int>(*width);
        data.height = static_cast<int>(*height);
        try {
            data.occupancy.assign(widthSize * heightSize, false);
        } catch (const std::bad_alloc&) {
            return ReadResult<GridMapData>::failed(
                "XML grid: dimensions are too large");
        } catch (const std::length_error&) {
            return ReadResult<GridMapData>::failed(
                "XML grid: dimensions are too large");
        }
        int row = 0;
        for (const auto&[fst, snd] : *grid) {
            const auto& tag  = fst;
            const auto& node = snd;
            if (tag == "<xmlattr>") continue;
            if (tag != "row") {
                return ReadResult<GridMapData>::failed(
                    "XML grid: unexpected <" + tag + "> in <grid>");
            }
            if (row >= data.height) {
                return ReadResult<GridMapData>::failed(
                    "XML grid: too many <row> entries");
            }
            std::istringstream iss(node.get_value<std::string>());
            int col = 0, cell = 0;
            while (iss >> cell) {
                if (col >= data.width) {
                    return ReadResult<GridMapData>::failed(
                        "XML grid: row " + std::to_string(row) +
                        " is wider than <width>");
                }
                if (cell != 0 && cell != 1) {
                    return ReadResult<GridMapData>::failed(
                        "XML grid: invalid cell token in row " +
                        std::to_string(row));
                }
                data.occupancy[static_cast<std::size_t>(row) * widthSize +
                               static_cast<std::size_t>(col)] = cell == 1;
                ++col;
            }
            if (!iss.eof()) {
                return ReadResult<GridMapData>::failed(
                    "XML grid: invalid cell token in row " +
                    std::to_string(row));
            }
            if (col != data.width) {
                return ReadResult<GridMapData>::failed(
                    "XML grid: row " + std::to_string(row) +
                    " is shorter than <width>");
            }
            ++row;
        }
        if (row != data.height) {
            return ReadResult<GridMapData>::failed(
                "XML grid: expected " + std::to_string(data.height) +
                " rows, found " + std::to_string(row));
        }
        return ReadResult<GridMapData>::succeeded(std::move(data));
    }

    bool writeGridXmlMap(const IDiscreteGrid& G, const std::filesystem::path& path)
    {
        std::error_code error;
        std::filesystem::create_directories(path.parent_path(), error);
        std::ofstream out(path);
        if (!out) return false;
        out << "<root>\n  <map>\n    <width>"<<G.gridWidthInCells()<<"</width>\n    <height>"<<G.gridHeightInCells()
            <<"</height>\n    <grid>\n";
        for (int r=0; r<G.gridHeightInCells(); ++r) {
            out << "      <row>";
            for (int c=0; c<G.gridWidthInCells(); ++c) out << (G.checkIfCellBlocked({c,r}) ? 1 : 0) << " ";
            out << "</row>\n";
        }
        out << "    </grid>\n  </map>\n</root>\n";
        out.close();
        return !out.fail();
    }

    ReadResult<std::vector<detail::GridStartGoal>>
    detail::parseGridXmlTask(const std::filesystem::path& path)
    {
        using detail::GridStartGoal;
        using boost::property_tree::ptree;

        ptree pt;
        try {
            read_xml(
                path.string(), pt,
                boost::property_tree::xml_parser::trim_whitespace);
        } catch (const boost::property_tree::xml_parser::xml_parser_error& error) {
            return ReadResult<std::vector<GridStartGoal>>::failed(
                "XML agents: parse failed for '" + path.string() +
                "': " + error.what());
        }

        const auto agents = pt.get_child_optional("root.agents");
        if (!agents) {
            return ReadResult<std::vector<GridStartGoal>>::failed(
                "XML agents: missing <root><agents>");
        }

        std::vector<GridStartGoal> tmp;
        for (const auto&[fst, snd] : *agents) {
            if (fst == "<xmlattr>") continue;
            if (fst != "agent") {
                return ReadResult<std::vector<GridStartGoal>>::failed(
                    "XML agents: unexpected <" + fst + "> in <agents>");
            }
            const auto& node = snd;

            const auto sr = node.get_optional<int>("<xmlattr>.sr");
            const auto sc = node.get_optional<int>("<xmlattr>.sc");
            const auto gr = node.get_optional<int>("<xmlattr>.gr");
            const auto gc = node.get_optional<int>("<xmlattr>.gc");
            if (!sr || !sc || !gr || !gc ||
                *sr < 0 || *sc < 0 || *gr < 0 || *gc < 0) {
                return ReadResult<std::vector<GridStartGoal>>::failed(
                    "XML agents: missing or invalid agent attributes");
            }
            tmp.push_back(GridStartGoal{*sc, *sr, *gc, *gr});
        }

        return ReadResult<std::vector<GridStartGoal>>::succeeded(
            std::move(tmp));
    }

    ReadResult<std::vector<StartGoal>> readGridXmlTask(
        const IDiscreteGrid& G,
        const std::filesystem::path& path,
        std::size_t takeK)
    {
        auto parsed = detail::parseGridXmlTask(path);
        if (!parsed) {
            return ReadResult<std::vector<StartGoal>>::failed(parsed.error);
        }
        auto& tmp = *parsed.value;
        for (const auto& entry : tmp) {
            if (entry.sr >= G.gridHeightInCells() ||
                entry.gr >= G.gridHeightInCells() ||
                entry.sc >= G.gridWidthInCells() ||
                entry.gc >= G.gridWidthInCells()) {
                return ReadResult<std::vector<StartGoal>>::failed(
                    "XML agents: agent coordinates are outside the grid");
            }
        }
        if (takeK < tmp.size()) tmp.resize(takeK);
        return ReadResult<std::vector<StartGoal>>::succeeded(
            detail::toStartGoals(G, tmp));
    }

} // namespace grace::io
