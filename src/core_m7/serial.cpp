#include "m7_serial.hpp"

#include "config.hpp"

#include <Arduino.h>

#include <stdarg.h>
#include <stdio.h>

void serial_begin() {
    Serial2.begin(CONFIG::M7::SERIAL_BAUD);
    delay(200);
}

void serial_logf(const char* fmt, ...) {
    static char buf[512];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial2.print(buf);
}
