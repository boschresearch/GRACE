// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/util/Logger.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <cctype>
#include <chrono>
#include <iomanip>
#include <stdexcept>
#if !defined(_WIN32)
  #include <unistd.h> // isatty
#else
  #include <io.h>
  #include <windows.h>
#endif

namespace {

// ---- level ↔ text -----------------------------------------------------------
constexpr std::string_view levelToSv(LogLevel lvl) {
    switch (lvl) {
        case LogLevel::TRACE: return "TRACE";
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default:              return "UNKNOWN";
    }
}

LogLevel parseLevelCI(std::string_view s) {
    std::string t;
    t.reserve(s.size());
    for (unsigned char c : s) t.push_back(static_cast<char>(std::toupper(c)));
    if (t == "TRACE") return LogLevel::TRACE;
    if (t == "DEBUG") return LogLevel::DEBUG;
    if (t == "INFO")  return LogLevel::INFO;
    if (t == "WARN")  return LogLevel::WARN;
    if (t == "ERROR") return LogLevel::ERROR;
    if (t == "FATAL") return LogLevel::FATAL;
    return LogLevel::UNKNOWN;
}

// ---- timestamp --------------------------------------------------------------
std::string timestamp(bool utc)
{
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto ms  = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::time_t tt = system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    if (utc) gmtime_s(&tm, &tt); else localtime_s(&tm, &tt);
#else
    if (utc) gmtime_r(&tt, &tm); else localtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

// ---- colors -----------------------------------------------------------------
const char* colorCodeForLevel(LogLevel level)
{
    switch (level) {
        case LogLevel::TRACE: return "\033[90m";
        case LogLevel::DEBUG: return "\033[36m";
        case LogLevel::INFO:  return "\033[32m";
        case LogLevel::WARN:  return "\033[33m";
        case LogLevel::ERROR: return "\033[31m";
        case LogLevel::FATAL: return "\033[91m";
        default:              return "\033[0m";
    }
}
constexpr const char* kColorReset = "\033[0m";

inline bool stderrIsTTY() {
#if defined(_WIN32)
    return _isatty(_fileno(stderr)) != 0;
#else
    return ::isatty(fileno(stderr)) != 0;
#endif
}

#if defined(_WIN32)
// Enable virtual terminal processing so ANSI codes work on newer Windows
void enableWinAnsiIfPossible() {
    HANDLE h = GetStdHandle(STD_ERROR_HANDLE);
    if (h == INVALID_HANDLE_VALUE) return;
    DWORD mode = 0;
    if (!GetConsoleMode(h, &mode)) return;
    SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}
#endif

} // anon

// ----------------------------------------------------------------------------

Logger& Logger::instance()
{
    static Logger s;
    return s;
}

Logger::Logger()
    : m_currentLevel(LogLevel::INFO)
    , m_throwLevel(static_cast<LogLevel>(999))
    , m_fileEnabled(false)
    , m_consoleEnabled(true)
    , m_colorEnabled(stderrIsTTY())
    , m_useUtcTime(false)
{
#if defined(_WIN32)
    if (m_colorEnabled.load()) enableWinAnsiIfPossible();
#endif
}

Logger::~Logger()
{
    std::scoped_lock lk(m_mutex);
    if (m_fileStream.is_open()) m_fileStream.close();
}

void Logger::setLevel(const LogLevel level)                 { m_currentLevel.store(level, std::memory_order_relaxed); }
void Logger::setThrowOnLevel(const LogLevel throwLevel)     { m_throwLevel.store(throwLevel, std::memory_order_relaxed); }
void Logger::setConsoleOutputEnabled(const bool enable)     { m_consoleEnabled.store(enable, std::memory_order_relaxed); }
void Logger::setColorEnabled(const bool enable)             { m_colorEnabled.store(enable, std::memory_order_relaxed); }
void Logger::setUseUtcTime(const bool enable)               { m_useUtcTime.store(enable, std::memory_order_relaxed); }

bool Logger::isEnabled(const LogLevel level) const noexcept {
    return level >= m_currentLevel.load(std::memory_order_relaxed);
}

void Logger::setFilename(const std::string& filename, const bool overwrite)
{
    if (filename.empty()) {
        std::scoped_lock lock(m_mutex);
        if (m_fileStream.is_open()) m_fileStream.close();
        m_fileEnabled.store(false, std::memory_order_relaxed);
        return;
    }

    const std::ios_base::openmode mode =
        std::ios::out | (overwrite ? std::ios::trunc : std::ios::app);
    std::ofstream replacement(filename, mode);
    if (!replacement.is_open()) {
        throw std::runtime_error("Logger: Failed to open file: " + filename);
    }

    std::scoped_lock lock(m_mutex);
    m_fileStream = std::move(replacement);
    m_fileEnabled.store(true, std::memory_order_relaxed);
}

void Logger::configureOutput(
    const LogLevel level,
    const LogLevel throwLevel,
    const std::string& filename,
    const bool overwrite)
{
    const std::ios_base::openmode mode =
        std::ios::out | (overwrite ? std::ios::trunc : std::ios::app);
    std::ofstream replacement(filename, mode);
    if (!replacement.is_open()) {
        throw std::runtime_error("Logger: Failed to open file: " + filename);
    }

    std::scoped_lock lock(m_mutex);
    m_fileStream = std::move(replacement);
    m_currentLevel.store(level, std::memory_order_relaxed);
    m_throwLevel.store(throwLevel, std::memory_order_relaxed);
    m_fileEnabled.store(true, std::memory_order_relaxed);
}

// Convenience methods unchanged
void Logger::trace(const std::string& msg, const std::source_location loc) { log(LogLevel::TRACE, loc, msg); }
void Logger::debug(const std::string& msg, const std::source_location loc) { log(LogLevel::DEBUG, loc, msg); }
void Logger::info (const std::string& msg, const std::source_location loc) { log(LogLevel::INFO,  loc, msg); }
void Logger::warn (const std::string& msg, const std::source_location loc) { log(LogLevel::WARN,  loc, msg); }
void Logger::error(const std::string& msg, const std::source_location loc) { log(LogLevel::ERROR, loc, msg); }
void Logger::fatal(const std::string& msg, const std::source_location loc) { log(LogLevel::FATAL, loc, msg); }

void Logger::log(const LogLevel level, const std::source_location loc, const std::string& msg)
{
    // Fast path: filtered-out logs don’t acquire the mutex
    if (level < m_currentLevel.load(std::memory_order_relaxed)) return;

    std::scoped_lock lock(m_mutex);

    // Re-check after acquiring the lock to avoid races with config changes
    if (level < m_currentLevel.load(std::memory_order_relaxed)) return;

    writeMessage(level, loc, msg);

    if (level >= m_throwLevel.load(std::memory_order_relaxed)) {
        std::ostringstream oss;
        oss << std::string(levelToSv(level)) << ' '
            << loc.file_name() << ':' << loc.line() << ' ' << msg;
        // loc.function_name()
        throw std::runtime_error(oss.str());
    }
}

void Logger::logWithoutThrow(
    const LogLevel level,
    const std::source_location loc,
    const std::string& msg)
{
    if (level < m_currentLevel.load(std::memory_order_relaxed)) return;

    std::scoped_lock lock(m_mutex);
    if (level < m_currentLevel.load(std::memory_order_relaxed)) return;
    writeMessage(level, loc, msg);
}

std::string Logger::levelToString(const LogLevel level) {
    return std::string(levelToSv(level));
}

LogLevel Logger::stringToLevel(std::string_view s) {
    return parseLevelCI(s);
}

void Logger::writeMessage(const LogLevel level, const std::source_location loc, const std::string& msg)
{
    const std::string ts = timestamp(m_useUtcTime.load(std::memory_order_relaxed));

    std::ostringstream line;
    line << "[" << ts << "] "
         << "[" << levelToSv(level) << "] "
         << loc.file_name() << ':' << loc.line() << ' ' << msg;
    // loc.function_name()

    const bool toConsole = m_consoleEnabled.load(std::memory_order_relaxed);
    const bool toFile    = m_fileEnabled.load(std::memory_order_relaxed);
    const bool color     = m_colorEnabled.load(std::memory_order_relaxed);

    if (toConsole) {
        if (color)
            std::cerr << colorCodeForLevel(level) << line.str() << kColorReset << '\n';
        else
            std::cerr << line.str() << '\n';
    }

    if (toFile && m_fileStream.is_open()) {
        m_fileStream << line.str() << '\n';
        m_fileStream.flush();
    }
}
