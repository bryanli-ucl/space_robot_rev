#include "shell.hpp"

#include <stdio.h>
#include <string.h>

void help_cmd(int argc, char** argv) {
    if (argc == 1) {
        shell.write("commands:\n");
        for (const auto& command : shell.command_list()) {
            char line[160];
            snprintf(line, sizeof(line), "  %-10s %s\n", command.name, command.help ? command.help : "");
            shell.write(line);
        }
        return;
    }

    for (const auto& command : shell.command_list()) {
        if (strcmp(argv[1], command.name) == 0) {
            char line[160];
            snprintf(line, sizeof(line), "%s: %s\n", command.name, command.help ? command.help : "");
            shell.write(line);
            return;
        }
    }

    char line[160];
    snprintf(line, sizeof(line), "help: unknown command '%s'\n", argv[1]);
    shell.write(line);
}

SHELL_COMMAND("help", help_cmd, "list commands or show help: help [cmd]")
