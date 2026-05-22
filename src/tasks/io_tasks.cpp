#include "tasks.hpp"

namespace {

constexpr size_t SERIAL_WRITE_CHUNK = 32;

const char* current_task_name() {
    osThreadId_t current = ThisThread::get_id();

    if (current == task_heartbeat.get_id()) return "heartbeat";
    if (current == task_serial_debug.get_id()) return "serial";
    if (current == task_sensors.get_id()) return "sensors";
    if (current == task_imu.get_id()) return "imu";
    if (current == task_rfid.get_id()) return "rfid";
    if (current == task_mission.get_id()) return "mission";
    if (current == task_chassis.get_id()) return "chassis";

    return "mqtt";
}

} // namespace

Mail<std::array<char, 256>, 64> mail_serial_debug;
void serial_tx(const char* fmt, ...) {
    std::array<char, 256>* mail = mail_serial_debug.try_alloc();
    if (mail == nullptr) {
        return;
    }

    char message[224];

    va_list args;
    va_start(args, fmt);
    vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    snprintf(mail->data(),
             mail->size() * sizeof(std::remove_pointer<decltype(mail->data())>::type),
             "[%s] %s",
             current_task_name(),
             message);

    mail_serial_debug.put(mail);
}

Mail<std::array<char, 256>, 64> mail_wifi_tx;
void wifi_tx(const char* fmt, ...) {
    std::array<char, 256>* mail = mail_wifi_tx.try_alloc();
    if (mail == nullptr) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(mail->data(), mail->size() * sizeof(std::remove_pointer<decltype(mail->data())>::type), fmt, args);
    va_end(args);

    mail_wifi_tx.put(mail);
}

void command_tx(const char* fmt, ...) {
    char buf[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    serial_tx("%s", buf);
    wifi_tx("%s", buf);
}

void func_serial_debug() {
    Serial1.begin(115200);
    ThisThread::sleep_for(100ms);

    std::array<char, 256> serial_cmd;
    size_t serial_cmd_len = 0;

    while (1) {
        while (!mail_serial_debug.empty()) {
            std::array<char, 256>* msg = mail_serial_debug.try_get();
            if (msg == nullptr) break;

            const char* cursor = msg->data();
            size_t remaining = strnlen(cursor, msg->size());
            while (remaining > 0) {
                const size_t chunk = std::min(remaining, SERIAL_WRITE_CHUNK);
                const size_t written = Serial1.write(reinterpret_cast<const uint8_t*>(cursor), chunk);
                if (written == 0) {
                    ThisThread::sleep_for(1ms);
                    continue;
                }

                cursor += written;
                remaining -= written;
            }

            mail_serial_debug.free(msg);
        }

        while (Serial1.available() > 0) {
            char c = static_cast<char>(Serial1.read());

            if (c == '\r' || c == '\n') {
                if (serial_cmd_len == 0) {
                    continue;
                }

                serial_cmd[serial_cmd_len] = '\0';
                serial_tx("SERIAL RX: %s\n", serial_cmd.data());
                bash.execute(serial_cmd.data());
                serial_cmd_len = 0;
            } else if (c == '\b' || c == 0x7f) {
                if (serial_cmd_len > 0) {
                    serial_cmd_len--;
                }
            } else if (serial_cmd_len < serial_cmd.size() - 1) {
                serial_cmd[serial_cmd_len++] = c;
            } else {
                serial_cmd_len = 0;
                serial_tx("SERIAL RX overflow, command cleared.\n");
            }
        }

        ThisThread::sleep_for(10ms);
    }
}
