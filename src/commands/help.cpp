#include "bash.hpp"

static void help(int argc, char** argv) {

    if (argc != 2) {
        command_tx("help needs exactly 1 arguments.\n");
        return;
    }
}

BASH_COMMAND("help", help, "get help of this bash")
