// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../../inc/io/GridMovingAI.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>

namespace fs = std::filesystem;

namespace grace::io {

    // ------------------ MovingAI .map ------------------

    ReadResult<GridMapData> readGridMovingAIMap(const fs::path& path)
    {
        std::ifstream fin(path);
        if (!fin) {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: cannot open '" + path.string() + "'");
        }

        auto readHeader = [&fin](const std::string& expected, auto& value) {
            std::string line;
            if (!std::getline(fin, line)) return false;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            std::istringstream input(line);
            std::string key;
            std::string extra;
            return (input >> key >> value) && key == expected &&
                   !(input >> extra);
        };

        std::string type;
        long long height = 0;
        long long width = 0;
        if (!readHeader("type", type) || type.empty() ||
            !readHeader("height", height) ||
            !readHeader("width", width)) {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: malformed header in '" + path.string() + "'");
        }

        std::string line;
        if (!std::getline(fin, line)) {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: missing map header in '" + path.string() + "'");
        }
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line != "map") {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: expected 'map' header in '" + path.string() + "'");
        }
        if (width <= 0 || height <= 0 ||
            width > std::numeric_limits<int>::max() ||
            height > std::numeric_limits<int>::max()) {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: invalid dimensions in '" + path.string() + "'");
        }
        const auto widthSize = static_cast<std::size_t>(width);
        const auto heightSize = static_cast<std::size_t>(height);
        if (heightSize > std::numeric_limits<std::size_t>::max() / widthSize ||
            widthSize * heightSize >
                static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: dimensions exceed the supported cell count in '" +
                path.string() + "'");
        }

        GridMapData data;
        data.width = static_cast<int>(width);
        data.height = static_cast<int>(height);
        try {
            data.occupancy.assign(widthSize * heightSize, false);
        } catch (const std::bad_alloc&) {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: dimensions are too large");
        } catch (const std::length_error&) {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: dimensions are too large");
        }
        for (int row = 0; row < data.height; ++row) {
            if (!std::getline(fin, line)) {
                return ReadResult<GridMapData>::failed(
                    "MovingAI map: truncated at row " + std::to_string(row));
            }
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.size() != widthSize) {
                return ReadResult<GridMapData>::failed(
                    "MovingAI map: row " + std::to_string(row) +
                    " has length " + std::to_string(line.size()) +
                    ", expected " + std::to_string(width));
            }
            for (int column = 0; column < data.width; ++column) {
                const char cell = line[static_cast<std::size_t>(column)];
                if (cell != '.' && cell != 'G' && cell != 'S' &&
                    cell != '@' && cell != 'O' && cell != 'T' && cell != 'W') {
                    return ReadResult<GridMapData>::failed(
                        "MovingAI map: invalid cell token at row " +
                        std::to_string(row) + ", column " +
                        std::to_string(column));
                }
                data.occupancy[static_cast<std::size_t>(row) * widthSize +
                               static_cast<std::size_t>(column)] =
                    cell == '@' || cell == 'O' || cell == 'T' || cell == 'W';
            }
        }
        if (std::getline(fin, line)) {
            return ReadResult<GridMapData>::failed(
                "MovingAI map: too many rows in '" + path.string() + "'");
        }
        return ReadResult<GridMapData>::succeeded(std::move(data));
    }

    bool writeGridMovingAIMap(const IDiscreteGrid& G, const fs::path& path){

        std::error_code ec; fs::create_directories(path.parent_path(), ec);
        std::ofstream out(path);
        if (!out) return false;
        out << "type octile\n";
        out << "height " << G.gridHeightInCells() << "\n";
        out << "width "  << G.gridWidthInCells()  << "\n";
        out << "map\n";
        for (int r=0; r<G.gridHeightInCells(); ++r) {
            for (int c=0; c<G.gridWidthInCells(); ++c) out << (G.checkIfCellBlocked({c, r}) ? '@' : '.');
            out << '\n';
        }
        out.close();
        return !out.fail();
    }

    // ------------------ MovingAI .scen ------------------

    namespace detail {

        ReadResult<std::vector<MovingAIScenarioEntry>>
        parseMovingAIScenario(const fs::path& scenPath)
        {
            std::ifstream sfile(scenPath);
            if (!sfile) {
                return ReadResult<std::vector<MovingAIScenarioEntry>>::failed(
                    "MovingAI scenario: cannot open '" + scenPath.string() + "'");
            }

            std::vector<std::pair<std::size_t, std::string>> lines;
            std::string line;
            std::size_t lineNumber = 0;
            while (std::getline(sfile, line)) {
                ++lineNumber;
                if (!line.empty() && line.back() == '\r') line.pop_back();
                if (line.find_first_not_of(" \t") != std::string::npos) {
                    lines.emplace_back(lineNumber, std::move(line));
                }
            }
            if (sfile.bad()) {
                return ReadResult<std::vector<MovingAIScenarioEntry>>::failed(
                    "MovingAI scenario: read failure in '" + scenPath.string() + "'");
            }
            if (lines.empty()) {
                return ReadResult<std::vector<MovingAIScenarioEntry>>::succeeded({});
            }

            bool versionOne = false;
            std::size_t firstRecord = 0;
            {
                std::istringstream header(lines.front().second);
                std::string keyword;
                header >> keyword;
                if (keyword == "version") {
                    double version = 0.0;
                    std::string extra;
                    if (!(header >> version) || (header >> extra) ||
                        !std::isfinite(version) ||
                        (version != 0.0 && version != 1.0)) {
                        return ReadResult<std::vector<MovingAIScenarioEntry>>::failed(
                            "MovingAI scenario: malformed or unsupported version header");
                    }
                    versionOne = version == 1.0;
                    firstRecord = 1;
                }
            }

            std::vector<MovingAIScenarioEntry> entries;
            entries.reserve(lines.size() - firstRecord);
            for (std::size_t index = firstRecord; index < lines.size(); ++index) {
                const auto& [recordLine, text] = lines[index];
                MovingAIScenarioEntry entry;
                std::istringstream record(text);
                bool parsed = false;
                if (versionOne) {
                    parsed = static_cast<bool>(
                        record >> entry.bucket >> entry.map >>
                        entry.width >> entry.height >>
                        entry.sc >> entry.sr >> entry.gc >> entry.gr >>
                        entry.dist);
                } else {
                    parsed = static_cast<bool>(
                        record >> entry.bucket >> entry.map >>
                        entry.sc >> entry.sr >> entry.gc >> entry.gr >>
                        entry.dist);
                }
                std::string extra;
                if (!parsed || (record >> extra) || entry.map.empty() ||
                    !std::isfinite(entry.dist) || entry.dist < 0.0 ||
                    entry.sc < 0 || entry.sr < 0 ||
                    entry.gc < 0 || entry.gr < 0 ||
                    (versionOne &&
                     (entry.width <= 0 || entry.height <= 0 ||
                      entry.sc >= entry.width || entry.gc >= entry.width ||
                      entry.sr >= entry.height || entry.gr >= entry.height))) {
                    return ReadResult<std::vector<MovingAIScenarioEntry>>::failed(
                        "MovingAI scenario: malformed record at line " +
                        std::to_string(recordLine));
                }
                entries.push_back(std::move(entry));
            }
            return ReadResult<std::vector<MovingAIScenarioEntry>>::succeeded(
                std::move(entries));
        }

        [[nodiscard]] std::vector<GridStartGoal> toGridStartGoals(
            const std::vector<MovingAIScenarioEntry>& entries, const std::size_t takeK){

            std::vector<GridStartGoal> out;
            std::size_t N;
            if (takeK <= 0) {
                N = entries.size();
            }
            else {
                N = std::min(takeK, entries.size());
            }
            out.reserve(N);

            for (std::size_t i = 0; i < N; ++i) {
                const auto& e = entries[i];
                out.push_back(GridStartGoal{/*sc*/e.sc, /*sr*/e.sr, /*gc*/e.gc, /*gr*/e.gr});
            }
            return out;
        }

    } // namespace detail

    ReadResult<std::vector<StartGoal>> readGridMovingAIScenario(
        const IDiscreteGrid& G, const fs::path& scenPath, const std::size_t takeK){

        const auto entries = detail::parseMovingAIScenario(scenPath);
        if (!entries) {
            return ReadResult<std::vector<StartGoal>>::failed(entries.error);
        }
        for (std::size_t index = 0; index < entries.value->size(); ++index) {
            const auto& entry = (*entries.value)[index];
            const bool dimensionsMismatch =
                entry.width >= 0 &&
                (entry.width != G.gridWidthInCells() ||
                 entry.height != G.gridHeightInCells());
            const bool coordinatesOutOfBounds =
                entry.sc >= G.gridWidthInCells() ||
                entry.gc >= G.gridWidthInCells() ||
                entry.sr >= G.gridHeightInCells() ||
                entry.gr >= G.gridHeightInCells();
            if (dimensionsMismatch || coordinatesOutOfBounds) {
                return ReadResult<std::vector<StartGoal>>::failed(
                    "MovingAI scenario: record " +
                    std::to_string(index + 1) +
                    " does not match the loaded grid");
            }
        }
        const auto gridPairs = detail::toGridStartGoals(*entries.value, takeK);
        return ReadResult<std::vector<StartGoal>>::succeeded(
            toStartGoals(G, gridPairs));
    }

    bool writeGridMovingAIScenario(const IDiscreteGrid& G,
                                    const fs::path& path,
                                    const std::vector<b2Vec2>& starts_m,
                                    const std::vector<b2Vec2>& goals_m,
                                    const std::vector<int>& buckets,
                                    const std::vector<double>& dists){

        std::error_code ec; fs::create_directories(path.parent_path(), ec);
        std::ofstream out(path);
        if (!out) return false;
        if (starts_m.size() != goals_m.size()) {
            return false;
        }
        const std::size_t N = starts_m.size();

        out << "version 1\n";
        out << std::fixed << std::setprecision(8);

        const int W = G.gridWidthInCells(), H = G.gridHeightInCells();
        const std::string mapName = "map.map"; // neutral placeholder

        for (std::size_t a=0; a<N; ++a) {
            const auto startCell = G.worldToCell(starts_m[a]);
            const auto goalCell = G.worldToCell(goals_m[a]);

            const int bucket = (buckets.size()==N ? buckets[a] : static_cast<int>(a));
            const double distV = (dists.size()==N ? dists[a]: G.euclideanDistance(startCell, goalCell));

            out << bucket << "\t" << mapName << "\t"
                << W << "\t" << H << "\t"
                << startCell.c << "\t" << startCell.r << "\t"
                << goalCell.c << "\t" << goalCell.r << "\t"
                << distV << "\n";
        }
        out.close();
        return !out.fail();
    }

} // namespace grace::io
