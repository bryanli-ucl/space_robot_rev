#include "logger.hpp"
#include "shell.hpp"

#include <mbed.h>
#include <stdio.h>
#include <string.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

Shell& shell = Shell::instance();

ShellCommandRegister::ShellCommandRegister(const char* name, Shell::handler_t handler, const char* help) {
    Shell::instance().reg_command(name, handler, help);
}

Shell& Shell::instance() {
    static Shell instance;
    return instance;
}

bool Shell::add_input(Stream& stream, const char* name) {
    for (auto& input : inputs) {
        if (input.stream == &stream) {
            input.name    = name;
            input.len     = 0;
            input.line[0] = '\0';
            return true;
        }
    }

    for (auto& input : inputs) {
        if (input.stream == nullptr) {
            input.stream  = &stream;
            input.name    = name;
            input.len     = 0;
            input.line[0] = '\0';
            return true;
        }
    }

    return false;
}

void Shell::func_shell_entry() {
    while (true) {
        for (auto& input : shell.inputs) {
            if (input.stream == nullptr) continue;

            while (input.stream->available() > 0) {
                const char c = static_cast<char>(input.stream->read());

                if (c == '\r' || c == '\n') {
                    if (input.len == 0) {
                        continue;
                    }

                    input.line[input.len] = '\0';
                    shell.execute(input.line);
                    input.len     = 0;
                    input.line[0] = '\0';
                } else if (c == '\b' || c == 0x7f) {
                    if (input.len > 0) {
                        input.len--;
                    }
                } else if (input.len < sizeof(input.line) - 1) {
                    input.line[input.len++] = c;
                } else {
                    input.len     = 0;
                    input.line[0] = '\0';
                    shell.write("shell input overflow, line cleared\n");
                }
            }
        }

        ThisThread::sleep_for(10ms);
    }
}

void Shell::reg_command(const char* name, handler_t handler, const char* help) {
    if (name == nullptr || handler == nullptr) return;

    commands.emplace_back(name, handler, help);
}

void Shell::execute(const char* line) {
    if (line == nullptr) return;

    char copy[LINE_SIZE];
    strlcpy(copy, line, sizeof(copy));
    execute(copy);
}

void Shell::execute(char* line) {
    if (line == nullptr) return;

    char* argv[MAX_ARGS];
    int argc = 0;

    char* token = strtok(line, " \t");
    while (token != nullptr && argc < static_cast<int>(MAX_ARGS)) {
        argv[argc++] = token;
        token        = strtok(nullptr, " \t");
    }

    if (argc == 0) return;

    for (const auto& command : commands) {
        if (strcmp(argv[0], command.name) == 0) {
            command.handler(argc, argv);
            return;
        }
    }

    char message[LINE_SIZE];
    snprintf(message, sizeof(message), "Command '%s' not found.\n", argv[0]);
    write(message);
}

const std::vector<Shell::command_t>& Shell::command_list() const {
    return commands;
}

void Shell::write(const char* text) {
    if (text == nullptr) return;
    loggf("%s", text);
}
