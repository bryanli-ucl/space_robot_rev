#pragma once

void m4_commands_begin();
void m4_command_worker_begin();
bool m4_command_enqueue(const char* source, const char* command);
