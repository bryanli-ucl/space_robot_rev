#pragma once

#include <Arduino.h>
#include <stddef.h>

class Logger {
    public:
    static constexpr size_t MESSAGE_SIZE  = 256;
    static constexpr size_t OUTPUTS_COUNT = 4;

    static Logger& instance();

    bool add_output(Stream& stream, const char* name);
    void write(const char* text);
    bool pop_wireless_log(char* out, size_t out_size);

    private:
    Logger()                        = default;
    Logger(const Logger&)            = delete;
    Logger& operator=(const Logger&) = delete;

    struct output_t {
        Stream* stream   = nullptr;
        const char* name = nullptr;
    };

    output_t outputs[OUTPUTS_COUNT];
};

extern Logger& logger;

void func_logger_entry();
void loggf(const char* fmt, ...);
