#pragma once

#include "config.hpp"

#include <Arduino.h>
#include <stddef.h>
#include <vector>

class Shell {
    public:
    using handler_t = void (*)(int argc, char** argv);

    struct command_t {
        const char* name;
        handler_t handler;
        const char* help;

        command_t(const char* n, handler_t h, const char* he)
        : name(n), handler(h), help(he) {}
    };

    static constexpr size_t MAX_ARGS   = 8;
    static constexpr size_t LINE_SIZE  = 256;
    static constexpr size_t MAX_INPUTS = 4;

    static Shell& instance();

    bool add_input(Stream& stream, const char* name);
    static void func_shell_entry();

    void reg_command(const char* name, handler_t handler, const char* help);
    void execute(char* line);
    void execute(const char* line);
    void write(const char* text);
    const std::vector<command_t>& command_list() const;

    private:
    Shell()                        = default;
    Shell(const Shell&)            = delete;
    Shell& operator=(const Shell&) = delete;

    struct input_t {
        Stream* stream       = nullptr;
        const char* name     = nullptr;
        char line[LINE_SIZE] = {};
        size_t len           = 0;
    };

    std::vector<command_t> commands;
    input_t inputs[MAX_INPUTS];
};

extern Shell& shell;

class ShellCommandRegister {
    public:
    ShellCommandRegister(const char* name, Shell::handler_t handler, const char* help);
};

#define SHELL_COMMAND(name, func, help) static ShellCommandRegister __shell_cmd_##func(name, func, help);
