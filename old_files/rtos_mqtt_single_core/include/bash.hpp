#pragma once

#include "main.hpp"

class Bash {

    public:
    using handler_t = void (*)(int argc, char** argv);

    struct command_t {
        const char* name;
        handler_t handler;
        const char* help;

        command_t(const char* n, handler_t h, const char* he)
        : name(n), handler(h), help(he) {}
    };

    std::vector<command_t> commands;

    void reg_command(const char* name, handler_t handler, const char* help);
    void execute(char* line);
};

extern Bash bash;

class CommandRegister {
    public:
    CommandRegister(const char* name, Bash::handler_t handler, const char* help) {
        bash.reg_command(name, handler, help);
    }
};

#define BASH_COMMAND(name, func, help) static CommandRegister __cmd_##func(name, func, help);
