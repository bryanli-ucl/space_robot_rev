#include "logger.hpp"

#include <array>
#include <mbed.h>
#include <stdarg.h>
#include <stdio.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

static constexpr size_t LOGGER_QUEUE_DEPTH = 128;
static constexpr size_t WIRELESS_LOG_DEPTH = 32;

static Mail<std::array<char, Logger::MESSAGE_SIZE>, LOGGER_QUEUE_DEPTH> logger_mail;
static char wireless_logs[WIRELESS_LOG_DEPTH][Logger::MESSAGE_SIZE] = {};
static size_t wireless_log_head = 0;
static size_t wireless_log_tail = 0;
static size_t wireless_log_count = 0;

Logger& logger = Logger::instance();

static void push_wireless_log(const char* text) {
    if (text == nullptr) return;

    mbed::CriticalSectionLock lock;
    strlcpy(wireless_logs[wireless_log_head], text, Logger::MESSAGE_SIZE);
    wireless_log_head = (wireless_log_head + 1) % WIRELESS_LOG_DEPTH;
    if (wireless_log_count < WIRELESS_LOG_DEPTH) {
        wireless_log_count++;
    } else {
        wireless_log_tail = (wireless_log_tail + 1) % WIRELESS_LOG_DEPTH;
    }
}

Logger& Logger::instance() {
    static Logger instance;
    return instance;
}

bool Logger::add_output(Stream& stream, const char* name) {
    for (auto& output : outputs) {
        if (output.stream == &stream) {
            output.name = name;
            return true;
        }
    }

    for (auto& output : outputs) {
        if (output.stream == nullptr) {
            output.stream = &stream;
            output.name   = name;
            return true;
        }
    }

    return false;
}

void Logger::write(const char* text) {
    if (text == nullptr) return;

    for (auto& output : outputs) {
        if (output.stream == nullptr) continue;
        output.stream->print(text);
    }
}

bool Logger::pop_wireless_log(char* out, size_t out_size) {
    if (out == nullptr || out_size == 0) return false;

    mbed::CriticalSectionLock lock;
    if (wireless_log_count == 0) {
        out[0] = '\0';
        return false;
    }

    strlcpy(out, wireless_logs[wireless_log_tail], out_size);
    wireless_log_tail = (wireless_log_tail + 1) % WIRELESS_LOG_DEPTH;
    wireless_log_count--;
    return true;
}

void func_logger_entry() {
    while (true) {
        auto* message = logger_mail.try_get_for(100ms);
        if (message == nullptr) continue;

        logger.write(message->data());
        logger_mail.free(message);
    }
}

void loggf(const char* fmt, ...) {
    if (fmt == nullptr) return;

    auto* message = logger_mail.try_alloc();
    if (message == nullptr) return;

    char text[Logger::MESSAGE_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    snprintf(message->data(),
    message->size(),
    "[tid=%p] %s",
    ThisThread::get_id(),
    text);

    push_wireless_log(message->data());
    logger_mail.put(message);
}
