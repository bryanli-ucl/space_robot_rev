#include "bash.hpp"

Bash bash;

void Bash::reg_command(const char* name, handler_t handler, const char* help) {
    commands.emplace_back(name, handler, help);
}

void Bash::execute(char* line) {

    if (line == nullptr)
        return;

    char* argv[8];
    int argc = 0;

    char* token = strtok(line, " ");

    while (token && argc < 8) {
        argv[argc++] = token;
        token        = strtok(NULL, " ");
    }

    if (argc == 0) return;

    for (const auto& han : commands) {
        if (strcmp(argv[0], han.name) == 0) {
            han.handler(argc, argv);
            return;
        }
    }

    wifi_tx("Command '%s' not found.\n", argv[0]);
}
