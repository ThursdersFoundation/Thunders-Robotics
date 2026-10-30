/**
 * @file logger.hpp
 * @brief Lightweight printf-style logger with severity levels and tags.
 *
 * A single shared `Logger` instance routes every log line through a single
 * sink (the configured `Print` — usually `Serial`). Includes a severity
 * filter so production builds can suppress `debug` and `info` without
 * removing the log calls from source.
 *
 * Usage:
 *     Logger::instance().info("[motor] PWM=%d", pwm);
 *     Logger::instance().error("%s failed: rc=%d", what, rc);
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_LOGGER_HPP
#define THUNDERS_LOGGER_HPP

#include <cstdarg>
#include <cstdint>

class Print;

namespace thunders::diagnostics {

enum class LogLevel : std::uint8_t {
    kTrace = 0,
    kDebug = 1,
    kInfo  = 2,
    kWarn  = 3,
    kError = 4,
    kNone  = 5,
};

class Logger {
public:
    static Logger& instance();

    /// Attach the output sink (e.g. `&Serial`). Must be called once before
    /// any log call. Optional minimum level default = kInfo.
    void begin(Print* sink, LogLevel min_level = LogLevel::kInfo);

    /// Set the minimum severity that will be emitted.
    void setLevel(LogLevel level) { minLevel_ = level; }

    void trace(const char* fmt, ...) const;
    void debug(const char* fmt, ...) const;
    void info (const char* fmt, ...) const;
    void warn (const char* fmt, ...) const;
    void error(const char* fmt, ...) const;

    // Counters — useful for health-monitor reporting.
    std::uint32_t traceCount() const { return counters_[0]; }
    std::uint32_t debugCount() const { return counters_[1]; }
    std::uint32_t infoCount()  const { return counters_[2]; }
    std::uint32_t warnCount()  const { return counters_[3]; }
    std::uint32_t errorCount() const { return counters_[4]; }

private:
    Logger() = default;
    void emit_(LogLevel level, const char* fmt, va_list ap) const;

    Print*       sink_     = nullptr;
    LogLevel     minLevel_ = LogLevel::kInfo;
    mutable std::uint32_t counters_[5] = {0, 0, 0, 0, 0};
};

}  // namespace thunders::diagnostics

#endif  // THUNDERS_LOGGER_HPP
