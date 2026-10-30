/**
 * @file logger.cpp
 * @brief Logger implementation.
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include "diagnostics/logger.hpp"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

namespace thunders::diagnostics {

namespace {
constexpr const char* kLevelTag[] = {"TRACE", "DEBUG", "INFO ", "WARN ", "ERROR"};
}

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

void Logger::begin(Print* sink, LogLevel min_level) {
    sink_     = sink;
    minLevel_ = min_level;
}

void Logger::emit_(LogLevel level, const char* fmt, va_list ap) const {
    if (!sink_ || level < minLevel_) return;

    const std::uint32_t now = millis();
    char header[32];
    snprintf(header, sizeof(header), "[%lu][%s] ",
              static_cast<unsigned long>(now),
              kLevelTag[static_cast<int>(level)]);
    sink_->print(header);

    char body[256];
    vsnprintf(body, sizeof(body), fmt, ap);
    sink_->print(body);
    sink_->print('\n');

    counters_[static_cast<int>(level)]++;
}

void Logger::trace(const char* fmt, ...) const {
    va_list ap; va_start(ap, fmt);
    emit_(LogLevel::kTrace, fmt, ap);
    va_end(ap);
}
void Logger::debug(const char* fmt, ...) const {
    va_list ap; va_start(ap, fmt);
    emit_(LogLevel::kDebug, fmt, ap);
    va_end(ap);
}
void Logger::info(const char* fmt, ...) const {
    va_list ap; va_start(ap, fmt);
    emit_(LogLevel::kInfo, fmt, ap);
    va_end(ap);
}
void Logger::warn(const char* fmt, ...) const {
    va_list ap; va_start(ap, fmt);
    emit_(LogLevel::kWarn, fmt, ap);
    va_end(ap);
}
void Logger::error(const char* fmt, ...) const {
    va_list ap; va_start(ap, fmt);
    emit_(LogLevel::kError, fmt, ap);
    va_end(ap);
}

}  // namespace thunders::diagnostics
