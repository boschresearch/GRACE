// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <fstream>
#include <mutex>
#include <source_location>
#include <atomic>
#include <string>
#include <string_view>

enum class LogLevel {
    TRACE, DEBUG, INFO, WARN, ERROR, FATAL, UNKNOWN
};

class Logger
{
public:
    static Logger& instance();

    // Configuration
    void setLevel(LogLevel level);
    void setThrowOnLevel(LogLevel throwLevel);
    void setFilename(const std::string& filename, bool overwrite = false);
    void configureOutput(LogLevel level,
                         LogLevel throwLevel,
                         const std::string& filename,
                         bool overwrite = false);
    void setConsoleOutputEnabled(bool enable);

    // Optional: control colors & timestamp zone
    void setColorEnabled(bool enable);
    void setUseUtcTime(bool enable);

    // Quick check before building expensive messages
    bool isEnabled(LogLevel level) const noexcept;

    // Convenience
    void trace(const std::string& msg, std::source_location loc = std::source_location::current());
    void debug(const std::string& msg, std::source_location loc = std::source_location::current());
    void info (const std::string& msg, std::source_location loc = std::source_location::current());
    void warn (const std::string& msg, std::source_location loc = std::source_location::current());
    void error(const std::string& msg, std::source_location loc = std::source_location::current());
    void fatal(const std::string& msg, std::source_location loc = std::source_location::current());

    // Master
    void log(LogLevel level, std::source_location loc, const std::string& msg);
    void logWithoutThrow(LogLevel level,
                         std::source_location loc,
                         const std::string& msg);

    // Utilities
    static std::string levelToString(LogLevel level);
    static LogLevel    stringToLevel(std::string_view s);

private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void writeMessage(LogLevel level, std::source_location loc, const std::string& msg);

    // Configuration (atomics allow lock-free fast path)
    std::atomic<LogLevel> m_currentLevel;   // min level to log
    std::atomic<LogLevel> m_throwLevel;     // min level to throw
    std::atomic<bool>     m_fileEnabled;
    std::atomic<bool>     m_consoleEnabled;
    std::atomic<bool>     m_colorEnabled;
    std::atomic<bool>     m_useUtcTime;

    // Sinks
    std::ofstream m_fileStream;

    // Serialize writeMessage and sink changes
    std::mutex m_mutex;
};
