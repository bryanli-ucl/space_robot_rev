#pragma once

void serial_begin();
void loggf(const char* fmt, ...);
void command_tx(const char* fmt, ...);
void logger_begin();
void serial_command_begin();
