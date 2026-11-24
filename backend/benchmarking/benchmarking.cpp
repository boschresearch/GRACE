// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "benchmarking/BenchmarkConfig.h"
#include "benchmarking/BenchmarkRun.h"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
    try {
        std::vector<std::string> arguments;
        arguments.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
        for (int index = 1; index < argc; ++index) {
            arguments.emplace_back(argv[index]);
        }

        auto commandLine =
            grace::benchmarking::parseBenchmarkCommandLine(arguments);
        if (commandLine.showHelp) {
            std::cout << commandLine.helpText << '\n';
            return 0;
        }

        const auto schedule =
            grace::benchmarking::makeBenchmarkSchedule(
                std::move(commandLine.config));
        bool allPlanned = true;
        for (const auto& run : schedule) {
            allPlanned =
                grace::benchmarking::runBenchmark(run) && allPlanned;
        }
        std::cout << "\n[SUMMARY] " << schedule.size()
                  << " run(s) scheduled/completed.\n";
        return allPlanned ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Benchmark error: " << error.what() << '\n';
        return 1;
    }
}
