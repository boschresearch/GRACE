// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "tools/MapConvertCli.h"

#include "tools/MapConvert.h"
#include "util/RunOutput.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <boost/program_options.hpp>

namespace fs = std::filesystem;
namespace po = boost::program_options;
using tools::mc::GridFormat;
using tools::mc::PRMOptions;
using tools::mc::RasterOptions;
using tools::mc::Result;

namespace {

class RunOutputScope {
public:
    RunOutputScope() {
        if (grace::output::currentRunDirectory().empty()) {
            (void)grace::output::startRunOutput();
            ownsSession_ = true;
        }
    }
    ~RunOutputScope() {
        if (ownsSession_) {
            grace::output::stopRunOutput();
        }
    }

    RunOutputScope(const RunOutputScope&) = delete;
    RunOutputScope& operator=(const RunOutputScope&) = delete;

private:
    bool ownsSession_{false};
};

GridFormat parseGridFormat(const std::string& value) {
    std::string normalized = value;
    for (auto& character : normalized) {
        character = static_cast<char>(
            std::tolower(static_cast<unsigned char>(character)));
    }
    if (normalized == "movingai" ||
        normalized == "moving-ai" ||
        normalized == "mai") {
        return GridFormat::MovingAI;
    }
    if (normalized == "xml" ||
        normalized == "xmlgrid" ||
        normalized == "xml-grid") {
        return GridFormat::Xml;
    }
    throw std::runtime_error(
        "Unknown grid format: " + value + " (use 'movingai' or 'xml')");
}

void ensureDirectory(const fs::path& path) {
    if (path.empty()) {
        return;
    }

    std::error_code error;
    fs::create_directories(path, error);
    if (error) {
        throw std::runtime_error(
            "Unable to create output directory '" + path.string() +
            "': " + error.message());
    }
}

void printResult(
    std::string_view label,
    const Result& result,
    std::ostream& output) {
    if (result.ok) {
        output << "[OK] " << label << " -> " << result.output.string() << '\n';
    } else {
        output << "[FAIL] " << label << " : " << result.message << '\n';
    }
}

/* ----------------------------- DEMO --------------------------------- */

bool isKnownMode(const std::string& mode) {
    static constexpr std::array<std::string_view, 13> modes{
        "demo", "c2r", "c2g", "g2y", "g2r", "r2y", "r2g",
        "y2s", "s2y", "y2t", "t2y", "s2t", "t2s",
    };
    return std::ranges::find(modes, mode) != modes.end();
}

int runDemo(
    const fs::path& outDir,
    const fs::path& dataRoot,
    std::ostream& output) {
    using namespace tools::mc;
    ensureDirectory(outDir);
    bool allOk = true;

    // Demo inputs (match your current mains)
    const fs::path continuousYaml =
        dataRoot / "maps/yaml/default/problem.yaml";
    const fs::path gridMovingAI =
        dataRoot / "maps/movingAI/default/random-32-32-20.map";
    const fs::path roadmapGraph =
        dataRoot / "maps/xml/default/roadmap/roadmap.xml";

    // Common options
    PRMOptions prm;
    prm.target_samples = 1500;  // grow a modest roadmap
    prm.time_sec       = 3.0;
    prm.robot_radius   = 0.25f;

    RasterOptions ras;
    ras.cell         = 0.5f;
    // ras.robot_radius = 0.25f;
    ras.inset        = 0.0f;

    const float wall = 0.25f;

    output << "== Demo: writing outputs under " << outDir << " ==\n";

    // 1) continuous (problem.yaml) -> roadmap (XML)
    {
        auto r = continuousYamlToRoadmap(
            continuousYaml,
            outDir / "cont_to_roadmap.xml",
            prm, wall);
        printResult("continuous->roadmap", r, output);
        allOk = allOk && r.ok;
    }

    // 2) continuous (problem.yaml) -> grid (MovingAI)
    {
        auto r = continuousYamlToGrid(
            continuousYaml,
            outDir / "cont_to_grid.map",
            GridFormat::MovingAI,
            ras,
            wall);
        printResult("continuous->grid(movingAI)", r, output);
        allOk = allOk && r.ok;
    }

    // 3) grid (MovingAI) -> environment-only YAML
    {
        auto r = gridToContinuousYaml(
            gridMovingAI,
            GridFormat::MovingAI,
            outDir / "grid_to_yaml.yaml",
            /*cell_length*/ 1.0f);
        printResult("grid(movingAI)->yaml", r, output);
        allOk = allOk && r.ok;
    }

    // 4) grid (MovingAI) -> roadmap (XML)
    {
        auto r = gridToRoadmap(
            gridMovingAI,
            GridFormat::MovingAI,
            outDir / "grid_to_roadmap.xml",
            prm,
            /*cell_length*/ 1.0f);
        printResult("grid(movingAI)->roadmap", r, output);
        allOk = allOk && r.ok;
    }

    // 5) roadmap (XML) -> environment-only YAML
    {
        auto r = roadmapToContinuousYaml(
            roadmapGraph,
            outDir / "roadmap_to_yaml.yaml",
            /*optional obstacles yaml*/ {}, wall);
        printResult("roadmap->yaml", r, output);
        allOk = allOk && r.ok;
    }

    // 6) roadmap (XML) -> grid (MovingAI), rasterizing only the bounds
    {
        auto r = roadmapToGrid(
            roadmapGraph,
            outDir / "roadmap_to_grid.map",
            GridFormat::MovingAI,
            /*optional obstacles yaml*/ {},
            ras,
            wall);
        printResult("roadmap->grid(movingAI)", r, output);
        allOk = allOk && r.ok;
    }

    output << "== Demo done ==\n";
    return allOk ? 0 : 3;
}

}  // namespace

int tools::mc::runMapConvertCli(
    int argc,
    char* argv[],
    const fs::path& dataRoot,
    std::ostream& output,
    std::ostream& error) {
    using namespace tools::mc;

    // Top-level options
    std::string mode;
    fs::path inPath;
    fs::path outPath;
    fs::path outDir = "results/convert_env";
    std::string gridFmtStr = "movingai";
    fs::path obsYaml;
    std::string defaultType = "unicycle_first_order_0";
    fs::path graphXml;
    fs::path mapPath;
    float cell = -1.0f;
    float wall = 0.25f;
    float robotR = 0.25f;
    float occThresh = 0.0f;
    int supersample = 9;
    int prmSamples = 2000;
    double prmTime = 5.0;
    int connectK = 12;
    float connectRadius = 2.0f;
    bool noPin = false;

    po::options_description generic("Options");
    generic.add_options()
        ("help,h",      "Show help")
        ("mode,m",      po::value<std::string>(&mode)->default_value("demo"),
                        "demo | c2r | c2g | g2y | g2r | r2y | r2g | y2s | s2y | y2t | t2y | s2t | t2s")
        ("in,i",        po::value<std::string>(), "Input path")
        ("out,o",       po::value<std::string>(), "Output path")
        ("outdir",      po::value<std::string>(), "Output directory for demo (default: results/convert_env)")
        ("grid-fmt",    po::value<std::string>(&gridFmtStr)->default_value("movingai"),
                        "Grid format for grid inputs/outputs: movingai | xml")
        ("cell",        po::value<float>(&cell)->default_value(-1.0f),
                            "Grid cell length (m). For continuous->grid: <=0 => auto = 2*max r_eff(S_i)")
        ("wall",        po::value<float>(&wall)->default_value(0.25f), "Boundary wall thickness (m)")
        ("radius",      po::value<float>(&robotR)->default_value(0.25f), "Robot radius used for checks")
        ("occ-thresh",  po::value<float>(&occThresh)->default_value(0.0f),
                                "Occupancy coverage threshold in [0,1]. 0 => any-overlap")
        ("supersample", po::value<int>(&supersample)->default_value(9),
                                "Supersamples per axis for coverage tests (NxN)")
        ("prm-samples", po::value<int>(&prmSamples)->default_value(2000), "PRM* target vertices")
        ("prm-time",    po::value<double>(&prmTime)->default_value(5.0),  "PRM* time budget (sec)")
        ("connect-k",      po::value<int>(&connectK)->default_value(12),   "k-NN for pin connections")
        ("connect-radius", po::value<float>(&connectRadius)->default_value(2.0f),
                           "max radius (m) for pin connections")
        ("no-pin",         po::bool_switch(&noPin),                        "do not pin starts/goals")
        ("obs-yaml",    po::value<std::string>(), "Optional obstacles YAML (for r2y/r2g)")
        ("graph",   po::value<std::string>(), "Roadmap GraphML (.xml) for scenario conversions")
        ("map",     po::value<std::string>(), "Grid map path (e.g., .map or .xml) for .scen conversions")
        ("default-type", po::value<std::string>(&defaultType)->default_value("unicycle_first_order_0"),
                         "Default robot type when synthesizing YAML from scenarios");
    ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, generic), vm);
        po::notify(vm);
    } catch (const std::exception& e) {
        error << "Arg parse error: " << e.what() << '\n';
        error << generic << '\n';
        return 1;
    }

    if (vm.count("help")) {
        output
            << "Environment conversion tool\n\n"
            << generic << "\n"
            << "Modes:\n"
            << "  demo            : run a full showcase with built-in sample paths\n"
            << "  c2r --in --out  : continuous YAML -> roadmap XML\n"
            << "  c2g --in --out  : continuous YAML -> grid (--grid-fmt)\n"
            << "  g2y --in --out  : grid (--grid-fmt) -> environment-only YAML\n"
            << "  g2r --in --out  : grid (--grid-fmt) -> roadmap XML\n"
            << "  r2y --in --out  : roadmap XML -> environment-only YAML\n"
            << "  r2g --in --out  : roadmap XML -> grid (--grid-fmt)\n"
            << "  y2s --in --out --map   : YAML scenario -> MovingAI .scen\n"
            << "  s2y --in --out --map   : MovingAI .scen -> YAML scenario\n"
            << "  y2t --in --out --graph : YAML scenario -> tasks.xml\n"
            << "  t2y --in --out --graph : tasks.xml -> YAML scenario\n"
            << "  s2t --in --out --map --graph : .scen -> tasks.xml\n"
            << "  t2s --in --out --map --graph : tasks.xml -> .scen\n";
        return 0;
    }

    if (!isKnownMode(mode)) {
        error << "Unknown mode: " << mode << '\n';
        return 9;
    }

    if (vm.count("outdir")) outDir = fs::path(vm["outdir"].as<std::string>());
    if (vm.count("in"))  inPath  = fs::path(vm["in"].as<std::string>());
    if (vm.count("out")) outPath = fs::path(vm["out"].as<std::string>());
    if (vm.count("obs-yaml")) obsYaml = fs::path(vm["obs-yaml"].as<std::string>());
    if (vm.count("graph")) graphXml = fs::path(vm["graph"].as<std::string>());
    if (vm.count("map"))   mapPath  = fs::path(vm["map"].as<std::string>());

    // Demo path: exercise everything
    if (mode == "demo") {
        try {
            const RunOutputScope outputScope;
            return runDemo(outDir, dataRoot, output);
        } catch (const std::exception& e) {
            error << "Exception: " << e.what() << '\n';
            return 10;
        }
    }

    // Single conversion path
    if (inPath.empty() || outPath.empty()) {
        error << "Please provide --in and --out for mode '" << mode
              << "'. Or use --mode demo.\n";
        return 2;
    }

    if ((mode == "y2s" || mode == "s2y") && mapPath.empty()) {
        error << "--map required\n";
        return mode == "y2s" ? 11 : 12;
    }
    if ((mode == "y2t" || mode == "t2y") && graphXml.empty()) {
        error << "--graph required\n";
        return mode == "y2t" ? 13 : 14;
    }
    if ((mode == "s2t" || mode == "t2s") &&
        (graphXml.empty() || mapPath.empty())) {
        error << "--graph and --map required\n";
        return mode == "s2t" ? 15 : 16;
    }

    GridFormat gridFormat;
    try {
        gridFormat = parseGridFormat(gridFmtStr);
    } catch (const std::exception& e) {
        error << "Exception: " << e.what() << '\n';
        return 10;
    }

    PRMOptions prm;
    prm.target_samples = prmSamples;
    prm.time_sec = prmTime;
    prm.robot_radius = robotR;
    prm.pin_starts_goals = !noPin;
    prm.connect_k = connectK;
    prm.connect_radius = connectRadius;

    RasterOptions ras;
    ras.cell = cell;
    ras.inset = 0.0f;
    ras.occ_threshold = occThresh;
    ras.supersample_px = supersample;

    try {
        const RunOutputScope outputScope;
        ensureDirectory(outPath.parent_path());
        if (mode == "c2r") {
            auto r = tools::mc::continuousYamlToRoadmap(inPath, outPath, prm, wall);
            printResult("c2r", r, output); return r.ok ? 0 : 3;
        } else if (mode == "c2g") {
            auto r = tools::mc::continuousYamlToGrid(inPath, outPath, gridFormat, ras, wall);
            printResult("c2g", r, output); return r.ok ? 0 : 4;
        } else if (mode == "g2y") {
            auto r = tools::mc::gridToContinuousYaml(
                inPath, gridFormat, outPath, /*cell_length*/ cell);
            printResult("g2y", r, output); return r.ok ? 0 : 5;
        } else if (mode == "g2r") {
            auto r = tools::mc::gridToRoadmap(
                inPath, gridFormat, outPath, prm, /*cell_length*/ cell);
            printResult("g2r", r, output); return r.ok ? 0 : 6;
        } else if (mode == "r2y") {
            auto r = tools::mc::roadmapToContinuousYaml(inPath, outPath, obsYaml, wall);
            printResult("r2y", r, output); return r.ok ? 0 : 7;
        } else if (mode == "r2g") {
            auto r = tools::mc::roadmapToGrid(
                inPath, outPath, gridFormat, obsYaml, ras, wall);
            printResult("r2g", r, output); return r.ok ? 0 : 8;
        } else if (mode == "y2s") {
            auto r = tools::mc::scenarioYamlToGridScen(inPath, mapPath, outPath, cell);
            printResult("y2s", r, output); return r.ok ? 0 : 11;
        } else if (mode == "s2y") {
            auto r = tools::mc::scenarioGridScenToYaml(
                mapPath, gridFormat, inPath, outPath, cell, defaultType);
            printResult("s2y", r, output); return r.ok ? 0 : 12;
        } else if (mode == "y2t") {
            auto r = tools::mc::scenarioYamlToTasksXml(inPath, graphXml, outPath);
            printResult("y2t", r, output); return r.ok ? 0 : 13;
        } else if (mode == "t2y") {
            auto r = tools::mc::scenarioTasksXmlToYaml(graphXml, inPath, outPath, defaultType);
            printResult("t2y", r, output); return r.ok ? 0 : 14;
        } else if (mode == "s2t") {
            auto r = tools::mc::scenarioGridScenToTasksXml(
                mapPath, gridFormat, cell, inPath, graphXml, outPath);
            printResult("s2t", r, output); return r.ok ? 0 : 15;
        } else if (mode == "t2s") {
            auto r = tools::mc::scenarioTasksXmlToGridScen(graphXml, inPath, mapPath, cell, outPath);
            printResult("t2s", r, output); return r.ok ? 0 : 16;
        }
        throw std::logic_error("Unhandled conversion mode: " + mode);
    } catch (const std::exception& e) {
        error << "Exception: " << e.what() << '\n';
        return 10;
    }
}
