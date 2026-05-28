#include "rpc_bridge.hpp"

#include "logger.hpp"
#include "shell.hpp"

#include <RPC.h>
#include <string>

static int rpc_shell_command(std::string command) {
    if (command.empty()) return 0;

    shell.execute(command.c_str());
    return static_cast<int>(command.length());
}

static std::string rpc_log_pop() {
    char text[Logger::MESSAGE_SIZE] = {};
    if (!logger.pop_wireless_log(text, sizeof(text))) return std::string();
    return std::string(text);
}

void rpc_bridge_begin() {
    RPC.begin();
    RPC.bind("m4_shell_command", rpc_shell_command);
    RPC.bind("m4_log_pop", rpc_log_pop);
    loggf("rpc bridge ready\n");
}
