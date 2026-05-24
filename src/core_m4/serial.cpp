#include "core_m4/serial.hpp"

#include "config.hpp"
#include "core_m4/bash.hpp"
#include "core_m4/commands.hpp"

#include <Arduino.h>
#include <mbed.h>

#include <array>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

using namespace ::rtos;
using namespace std::chrono_literals;

extern Thread task_logger;
extern Thread task_serial_command;

namespace {

constexpr size_t LOGGER_QUEUE_DEPTH  = 64;
constexpr size_t LOGGER_MESSAGE_SIZE = 256;
constexpr size_t SERIAL_WRITE_CHUNK  = 32;

Mail<std::array<char, LOGGER_MESSAGE_SIZE>, LOGGER_QUEUE_DEPTH> logger_mail;
bool logger_started = false;
bool serial_command_started = false;

void serial_command_entry() {
    std::array<char, LOGGER_MESSAGE_SIZE> command;
    size_t command_len = 0;

    loggf("[m4-serial-cmd] task ready\n");

    while (true) {
        while (Serial2.available() > 0) {
            const char c = static_cast<char>(Serial2.read());

            if (c == '\r' || c == '\n') {
                if (command_len == 0) {
                    continue;
                }

                command[command_len] = '\0';
                loggf("[m4-serial-cmd] rx: %s\n", command.data());
                m4_command_enqueue("serial", command.data());
                command_len = 0;
            } else if (c == '\b' || c == 0x7f) {
                if (command_len > 0) {
                    command_len--;
                }
            } else if (command_len < command.size() - 1) {
                command[command_len++] = c;
            } else {
                command_len = 0;
                loggf("[m4-serial-cmd] overflow, command cleared\n");
            }
        }

        ThisThread::sleep_for(std::chrono::milliseconds(CONFIG::M4::SERIAL_DEBUG_TASK_INTERVAL_MS));
    }
}

void write_serial_chunked(const char* text, size_t len) {
    const char* cursor = text;
    size_t remaining   = len;

    while (remaining > 0) {
        const size_t chunk   = min(remaining, SERIAL_WRITE_CHUNK);
        const size_t written = Serial2.write(reinterpret_cast<const uint8_t*>(cursor), chunk);
        if (written == 0) {
            ThisThread::sleep_for(1ms);
            continue;
        }

        cursor += written;
        remaining -= written;
    }
}

void logger_entry() {
    while (true) {
        auto* message = logger_mail.try_get_for(100ms);
        if (message == nullptr) {
            continue;
        }

        const size_t len = strnlen(message->data(), message->size());
        write_serial_chunked(message->data(), len);
        logger_mail.free(message);
    }
}

void enqueue_log(const char* text) {
    if (text == nullptr) {
        return;
    }

    auto* message = logger_mail.try_alloc();
    if (message == nullptr) {
        return;
    }

    strlcpy(message->data(), text, message->size());
    logger_mail.put(message);
}

} // namespace

void serial_begin() {
    Serial2.begin(CONFIG::M4::SERIAL_BAUD);
    delay(200);
    logger_begin();
    loggf("\n\n==================== Serial Begin, Program Start Up =================\n");
}

void loggf(const char* fmt, ...) {
    char buf[LOGGER_MESSAGE_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    enqueue_log(buf);
}

void command_tx(const char* fmt, ...) {
    char buf[LOGGER_MESSAGE_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    loggf("%s", buf);
}

void logger_begin() {
    if (logger_started) {
        return;
    }

    const osStatus status = task_logger.start(logger_entry);
    logger_started        = status == osOK;
    if (!logger_started) {
        Serial2.write("[m4-logger] start failed\n");
    } else {
        Serial2.write("[m4-logger] start successed\n");
    }
}

void serial_command_begin() {
    if (serial_command_started || !CONFIG::M4::ENABLE_SERIAL_DEBUG_TASK) {
        return;
    }

    const osStatus status = task_serial_command.start(serial_command_entry);
    serial_command_started = status == osOK;
    if (!serial_command_started) {
        loggf("[m4-serial-cmd] start failed status=%d\n", status);
    }
}
