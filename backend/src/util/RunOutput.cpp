// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "util/RunOutput.h"

#include "util/ResultsDir.h"

#include <mutex>
#include <stdexcept>

namespace grace::output {
namespace {

std::mutex outputMutex;
std::filesystem::path runDirectory;

std::filesystem::path reserveUniqueRunDirectory(
    const std::filesystem::path& baseDirectory) {
    std::error_code error;
    std::filesystem::create_directories(baseDirectory, error);
    if (error) {
        throw std::runtime_error(
            "Failed to create run output base directory: " +
            error.message());
    }

    const std::string timestamp = results::makeTimestamp();
    for (std::size_t suffix = 0;; ++suffix) {
        const std::string name = suffix == 0
            ? timestamp
            : timestamp + "-" + std::to_string(suffix);
        const auto candidate = baseDirectory / name;
        error.clear();
        if (std::filesystem::create_directory(candidate, error)) {
            const auto canonical =
                std::filesystem::weakly_canonical(candidate, error);
            return error ? candidate : canonical;
        }
        if (error) {
            throw std::runtime_error(
                "Failed to reserve run output directory: " +
                error.message());
        }
    }
}

}  // namespace

std::filesystem::path startRunOutput(const RunOutputConfig& config) {
    std::scoped_lock lock(outputMutex);
    if (config.baseDirectory.empty()) {
        throw std::invalid_argument(
            "Run output base directory must not be empty");
    }

    const auto directory = config.timestamp.empty()
        ? reserveUniqueRunDirectory(config.baseDirectory)
        : results::makeRunDir(config.baseDirectory, config.timestamp);
    if (!std::filesystem::is_directory(directory)) {
        throw std::runtime_error(
            "Failed to create run output directory: " +
            directory.string());
    }

    auto& logger = Logger::instance();
    logger.configureOutput(
        config.logLevel,
        config.throwLevel,
        (directory / "logging.log").string(),
        config.overwriteLog);

    runDirectory = directory;
    logger.logWithoutThrow(
        LogLevel::INFO,
        std::source_location::current(),
        "Results directory: " + directory.string());
    return directory;
}

std::filesystem::path currentRunDirectory() {
    std::scoped_lock lock(outputMutex);
    return runDirectory;
}

void stopRunOutput() {
    std::scoped_lock lock(outputMutex);
    Logger::instance().setFilename("");
    runDirectory.clear();
}

}  // namespace grace::output
