#include "core_m7/serial.hpp"

#include "config.hpp"

#include <Arduino.h>

#include <array>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void serial_begin() {
    Serial1.begin(CONFIG::M7::SERIAL_BAUD);
    delay(200);
    loggf("\n\n==================== Serial Begin, Program Start Up =================\n");
}

void loggf(const char* fmt, ...) {
    static char buf[512];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial1.write(buf);
}
