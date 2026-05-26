#include "core_m7/wifi_mqtt.hpp"

#include "config.hpp"
#include "core_m7/ir.hpp"
#include "core_m7/rfid.hpp"
#include "core_m7/rpc_bridge.hpp"
#include "core_m7/serial.hpp"
#include "core_m7/sunlight.hpp"
#include "core_m7/ultrasonic.hpp"

#include <Arduino.h>
#include <MiniMessenger.h>
#include <RPC.h>
#include <WiFi.h>

#include <stdio.h>
#include <string.h>

namespace {

MiniMessenger messenger;

uint32_t last_scan_ms          = 0;
uint32_t last_status_ms        = 0;
uint32_t last_heartbeat_log_ms = 0;
uint32_t last_register_ms      = 0;
uint32_t heartbeat_count       = 0;
uint32_t status_counter        = 0;
uint32_t wifi_connected_ms     = 0;
uint32_t register_count        = 0;

bool last_connected    = false;
bool register_sent     = false;
bool safety_enabled    = false;
bool messenger_started = false;

void scan_wifi_networks(bool force = false);
bool connect_wifi_blocking();
void start_messenger();
void on_mqtt_message(const MessageMetadata& metadata, const uint8_t* payload, size_t length);
bool send_register();
void update_connection_state(bool connected);
void update_status_log(uint32_t now_ms, bool connected);
void update_register(bool connected);
bool extract_command(const char* msg, char* command, size_t command_size);
bool extract_door_response(const char* msg, bool* granted);
void push_door_response_to_m4(bool granted);

} // namespace

void wifi_mqtt_begin() {
    loggf("[m7-mqtt] broker=%s:%u group=%s board=%s\n",
    CONFIG::M7::MQTT_HOST,
    CONFIG::M7::MQTT_PORT,
    CONFIG::M7::GROUP_ID,
    CONFIG::M7::BOARD_ID);

    scan_wifi_networks(true);
    messenger.onMessage(on_mqtt_message);

    if (connect_wifi_blocking()) {
        start_messenger();
    } else {
        loggf("[m7-mqtt] MiniMessenger skipped until WiFi is connected\n");
    }
}

void wifi_mqtt_update(uint32_t now_ms) {
    if (WiFi.status() == WL_CONNECTED && wifi_connected_ms == 0) {
        wifi_connected_ms = now_ms;
    } else if (WiFi.status() != WL_CONNECTED) {
        wifi_connected_ms = 0;
    }

    if (!messenger_started) {
        if (WiFi.status() == WL_CONNECTED) {
            start_messenger();
        }
        update_status_log(now_ms, false);
        if (WiFi.status() != WL_CONNECTED) {
            scan_wifi_networks();
        }
        return;
    }

    const uint32_t loop_started_ms = millis();
    messenger.loop();
    const uint32_t loop_duration_ms = millis() - loop_started_ms;
    if (loop_duration_ms > 100) {
        loggf("[m7-mqtt] messenger.loop took %lums\n",
        static_cast<unsigned long>(loop_duration_ms));
    }

    const bool connected = messenger.isConnected();
    update_connection_state(connected);
    update_status_log(now_ms, connected);
    update_register(connected);
    if (!connected) {
        scan_wifi_networks();
    }
}

bool wifi_mqtt_is_wifi_connected() {
    return WiFi.status() == WL_CONNECTED;
}

bool wifi_mqtt_is_mqtt_connected() {
    return messenger_started && messenger.isConnected();
}

bool wifi_mqtt_is_safety_enabled() {
    return safety_enabled;
}

bool wifi_mqtt_send_to_server(const char* payload) {
    if (payload == nullptr || payload[0] == '\0' || !messenger_started || !messenger.isConnected()) {
        return false;
    }

    return messenger.sendToBoard(CONFIG::M7::SERVER_ID, payload);
}

namespace {

void scan_wifi_networks(bool force) {
    const uint32_t now = millis();
    if (!force &&
    last_scan_ms != 0 &&
    now - last_scan_ms < CONFIG::M7::WIFI_SCAN_INTERVAL_MS) {
        return;
    }
    last_scan_ms = now;

    loggf("[m7-mqtt] scanning WiFi networks\n");
    const int count = WiFi.scanNetworks();
    if (count < 0) {
        loggf("[m7-mqtt] WiFi scan failed rc=%d\n", count);
        return;
    }

    loggf("[m7-mqtt] found %d WiFi networks\n", count);
    for (int i = 0; i < count; i++) {
        loggf("[m7-mqtt] %2d: ssid=%s rssi=%ld enc=%d\n",
        i + 1,
        WiFi.SSID(i),
        WiFi.RSSI(i),
        WiFi.encryptionType(i));
    }
}

bool connect_wifi_blocking() {
    if (WiFi.status() == WL_CONNECTED) {
        loggf("[m7-mqtt] WiFi already connected ip=%s rssi=%ld\n",
        WiFi.localIP().toString().c_str(),
        WiFi.RSSI());
        return true;
    }

    int attempts = 10;

    while (attempts--) {

        loggf("[m7-mqtt] WiFi connecting ssid=%s, remaining attempts: %d\n", CONFIG::M7::WIFI_SSID, attempts);
        WiFi.disconnect();
        delay(100);
        WiFi.begin(CONFIG::M7::WIFI_SSID, CONFIG::M7::WIFI_PASS);

        const uint32_t started_ms = millis();
        int last_status           = -1;
        while (millis() - started_ms < CONFIG::M7::WIFI_CONNECT_TIMEOUT_MS) {
            const int status = WiFi.status();
            if (status == WL_CONNECTED) {
                wifi_connected_ms = millis();
                loggf("[m7-mqtt] WiFi connected ip=%s rssi=%ld\n",
                WiFi.localIP().toString().c_str(),
                WiFi.RSSI());
                return true;
            }

            if (status != last_status) {
                last_status = status;
                loggf("[m7-mqtt] WiFi status=%d while connecting\n", status);
            }
            delay(CONFIG::M7::WIFI_CONNECT_POLL_MS);
        }
    }

    loggf("[m7-mqtt] WiFi connect timeout status=%d ip=%s\n",
    WiFi.status(),
    WiFi.localIP().toString().c_str());

    return false;
}

void start_messenger() {
    const bool connected = messenger.begin(CONFIG::M7::WIFI_SSID,
    CONFIG::M7::WIFI_PASS,
    CONFIG::M7::MQTT_HOST,
    CONFIG::M7::MQTT_PORT,
    CONFIG::M7::GROUP_ID,
    CONFIG::M7::BOARD_ID);
    messenger_started    = true;
    last_connected       = connected;

    loggf("[m7-mqtt] MiniMessenger begin connected=%d client=%s ip=%s\n",
    connected,
    messenger.clientId(),
    WiFi.localIP().toString().c_str());

    update_status_log(millis(), connected);
    if (connected) {
        last_register_ms = millis();
        register_sent = send_register();
    }
}

void on_mqtt_message(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char msg[MiniMessenger::kMaxPayloadSize + 1];
    const size_t copy_len = (length < MiniMessenger::kMaxPayloadSize) ? length : MiniMessenger::kMaxPayloadSize;
    memcpy(msg, payload, copy_len);
    msg[copy_len] = '\0';

    if (strstr(msg, "type=heartbeat enable=1")) {
        heartbeat_count++;
        const uint32_t now = millis();
        if (!safety_enabled) {
            loggf("[m7-mqtt] safety heartbeat enabled\n");
        }
        if (now - last_heartbeat_log_ms >= CONFIG::M7::HEARTBEAT_LOG_INTERVAL_MS ||
        last_heartbeat_log_ms == 0) {
            last_heartbeat_log_ms = now;
            loggf("[m7-mqtt] heartbeat ok count=%lu from=%s target=%s\n",
            static_cast<unsigned long>(heartbeat_count),
            metadata.fromBoardId,
            metadata.target);
        }
        safety_enabled = true;
        return;
    }

    if (strstr(msg, "type=heartbeat enable=0") ||
    strstr(msg, "type=emergency enabled=true") ||
    strstr(msg, "type=disable enabled=false")) {
        if (safety_enabled) {
            loggf("[m7-mqtt] safety disabled\n");
        }
        safety_enabled = false;
        return;
    }

    bool door_granted = false;
    if (extract_door_response(msg, &door_granted)) {
        loggf("[m7-mqtt] door response granted=%d from=%s\n", door_granted ? 1 : 0, metadata.fromBoardId);
        push_door_response_to_m4(door_granted);
        return;
    }

    char command[MiniMessenger::kMaxPayloadSize + 1];
    if (extract_command(msg, command, sizeof(command))) {
        loggf("[m7-mqtt] command from=%s target=%s: %s\n",
        metadata.fromBoardId,
        metadata.target,
        command);
        rpc_bridge_send_m4_command(command);
        return;
    }

    loggf("[m7-mqtt] rx [%s -> %s group=%s]: %s\n",
    metadata.fromBoardId,
    metadata.target,
    metadata.groupId,
    msg);
}

bool send_register() {
    char payload[96];
    snprintf(payload,
    sizeof(payload),
    "type=register team_id=%s board_id=%s",
    CONFIG::M7::GROUP_ID,
    CONFIG::M7::BOARD_ID);

    const bool sent = messenger.sendToBoard(CONFIG::M7::SERVER_ID, payload);
    if (sent) {
        register_count++;
    }
    loggf("[m7-mqtt] register %s count=%lu to=%s: %s\n",
    sent ? "sent" : "failed",
    static_cast<unsigned long>(register_count),
    CONFIG::M7::SERVER_ID,
    payload);
    return sent;
}

void update_connection_state(bool connected) {
    if (connected == last_connected) {
        return;
    }

    last_connected = connected;
    if (!connected) {
        register_sent = false;
        last_register_ms = 0;
    }

    loggf("[m7-mqtt] MiniMessenger %s ip=%s\n",
    connected ? "connected" : "disconnected",
    WiFi.localIP().toString().c_str());

    if (connected) {
        last_register_ms = 0;
    }
}

void update_status_log(uint32_t now_ms, bool connected) {
    if (now_ms - last_status_ms < CONFIG::M7::STATUS_INTERVAL_MS &&
    last_status_ms != 0) {
        return;
    }

    last_status_ms = now_ms;
    loggf("[m7-mqtt] status ms=%lu counter=%lu wifi=%d mqtt=%d ip=%s safety=%d sunlight=%u ir=%u dist=%d/%d/%d us_dt=%lums rfid=%lu ready=%d\n",
    static_cast<unsigned long>(now_ms),
    static_cast<unsigned long>(status_counter++),
    WiFi.status(),
    connected,
    WiFi.localIP().toString().c_str(),
    safety_enabled,
    sunlight_value(),
    ir_position(),
    ultrasonic_front_cm(),
    ultrasonic_left_cm(),
    ultrasonic_right_cm(),
    static_cast<unsigned long>(ultrasonic_last_duration_ms()),
    static_cast<unsigned long>(rfid_last_uid()),
    rfid_is_ready());
}

void update_register(bool connected) {
    if (!connected) {
        return;
    }

    const uint32_t now_ms = millis();
    const uint32_t interval_ms = register_sent ?
        CONFIG::M7::MQTT_REGISTER_INTERVAL_MS :
        CONFIG::M7::MQTT_REGISTER_RETRY_MS;

    if (last_register_ms != 0 && now_ms - last_register_ms < interval_ms) {
        return;
    }

    last_register_ms = now_ms;
    register_sent = send_register();
}

bool extract_command(const char* msg, char* command, size_t command_size) {
    if (msg == nullptr || command == nullptr || command_size == 0) {
        return false;
    }

    const char* start = nullptr;
    if (strncmp(msg, "type=command", 12) == 0 || strncmp(msg, "type=cmd", 8) == 0) {
        start = strstr(msg, " cmd=");
        if (start != nullptr) {
            start += 5;
        } else {
            start = strstr(msg, " command=");
            if (start != nullptr) {
                start += 9;
            }
        }
    } else if (strncmp(msg, "cmd=", 4) == 0) {
        start = msg + 4;
    } else if (strncmp(msg, "command=", 8) == 0) {
        start = msg + 8;
    } else {
        start = msg;
    }

    if (start == nullptr) {
        return false;
    }

    while (*start == ' ' || *start == '\t') {
        start++;
    }

    if (*start == '\0') {
        return false;
    }

    strlcpy(command, start, command_size);
    return true;
}

bool extract_door_response(const char* msg, bool* granted) {
    if (msg == nullptr || granted == nullptr) {
        return false;
    }

    const bool is_response = strstr(msg, "type=door_response") != nullptr ||
                             strstr(msg, "type=door") != nullptr ||
                             strstr(msg, "type=exit_clearance") != nullptr;
    if (!is_response) {
        return false;
    }

    *granted = strstr(msg, "granted=1") != nullptr ||
               strstr(msg, "granted=true") != nullptr ||
               strstr(msg, "status=ok") != nullptr ||
               strstr(msg, "open=1") != nullptr ||
               strstr(msg, "open=true") != nullptr;
    return true;
}

void push_door_response_to_m4(bool granted) {
    static int door_response_seq = 0;
    const int seq = door_response_seq++;
    const int result = RPC.call("m4_door_response_update", seq, granted ? 1 : 0).as<int>();
    if (RPC.timedOut() || result != seq) {
        loggf("[m7-mqtt] door response RPC failed seq=%d rc=%d\n", seq, result);
    }
}

} // namespace
