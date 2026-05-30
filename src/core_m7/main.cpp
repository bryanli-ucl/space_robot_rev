#include "config.hpp"

#include <Arduino.h>
#include <RPC.h>
#include <WiFi.h>
#include <mbed.h>
#include <string>

using namespace ::rtos;
using namespace ::std::chrono_literals;

static WiFiServer debug_server(WIRELESS_DEBUG_PORT);
static WiFiClient debug_client;

static void log_local(const char* text) {
    if (text == nullptr) return;
    Serial.print(text);
    if (debug_client && debug_client.connected()) debug_client.print(text);
}

static bool connect_wifi() {
    static uint32_t last_wifi_attempt_ms = 0;
    static bool wifi_connected_printed = false;
    static bool debug_server_started = false;

    const uint32_t now_ms = millis();
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifi_connected_printed) {
            wifi_connected_printed = true;
            Serial.print("wifi connected ip=");
            Serial.println(WiFi.localIP());
        }

        if (!debug_server_started) {
            debug_server.begin();
            debug_server_started = true;
            Serial.print("wireless debug listening port=");
            Serial.println(WIRELESS_DEBUG_PORT);
        }
        return true;
    }

    wifi_connected_printed = false;
    debug_server_started   = false;
    if (last_wifi_attempt_ms != 0 && now_ms - last_wifi_attempt_ms < WIFI_CONNECT_RETRY_MS) return false;

    last_wifi_attempt_ms = now_ms;
    Serial.print("wifi begin ssid=");
    Serial.println(WIFI_SSID);
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    return false;
}

static void accept_client() {
    if (debug_client && debug_client.connected()) return;

    WiFiClient next_client = debug_server.accept();
    if (!next_client) return;

    debug_client = next_client;
    debug_client.print("space-robot wireless shell ready\r\n");
    debug_client.print("send M4 shell commands, one per line\r\n> ");
    Serial.println("wireless client connected");
}

static void execute_line(const char* line) {
    if (line == nullptr || line[0] == '\0') return;

    if (debug_client && debug_client.connected()) {
        debug_client.print("\r\n> ");
        debug_client.println(line);
    }

    const int result = RPC.call("m4_shell_command", std::string(line)).as<int>();
    if (RPC.timedOut() || result < 0) {
        log_local("rpc command failed\r\n");
    }
}

static void read_client_input() {
    static char input_line[WIRELESS_LINE_SIZE];
    static size_t input_len = 0;

    if (!debug_client || !debug_client.connected()) return;

    while (debug_client.available() > 0) {
        const char c = static_cast<char>(debug_client.read());
        if (c == '\r') continue;

        if (c == '\n') {
            input_line[input_len] = '\0';
            execute_line(input_line);
            input_len     = 0;
            input_line[0] = '\0';
        } else if (input_len < sizeof(input_line) - 1) {
            input_line[input_len++] = c;
        } else {
            input_len     = 0;
            input_line[0] = '\0';
            log_local("wireless input overflow, line cleared\r\n");
        }
    }
}

static void poll_m4_logs() {
    static uint32_t last_log_poll_ms = 0;

    if (!debug_client || !debug_client.connected()) return;

    const uint32_t now_ms = millis();
    if (now_ms - last_log_poll_ms < WIRELESS_LOG_POLL_MS && last_log_poll_ms != 0) return;
    last_log_poll_ms = now_ms;

    for (uint8_t i = 0; i < 8; i++) {
        std::string text = RPC.call("m4_log_pop").as<std::string>();
        if (RPC.timedOut() || text.empty()) return;
        debug_client.print(text.c_str());
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);

    Serial.println("m7 wireless debug starting");
    RPC.begin();
    connect_wifi();
}

void loop() {
    if (connect_wifi()) {
        accept_client();
        read_client_input();
        poll_m4_logs();
    }

    ThisThread::sleep_for(5ms);
}
