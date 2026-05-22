#include <Arduino.h>
#include <MiniMessenger.h>
#include <WiFi.h>
#include <Wire.h>
#include <mbed.h>


#include "main.hpp"

using namespace ::rtos;
using namespace ::std::chrono_literals;

Thread task_blink(osPriorityAboveNormal2);
MiniMessenger messenger;

namespace {

constexpr auto WIFI_RETRY_DELAY            = 2s;
constexpr auto WIFI_POLL_DELAY             = 500ms;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;

void log_line(const char* message) {
    Serial.print(message);
    Serial1.print(message);
}

bool connect_wifi() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    char buf[160];
    snprintf(buf, sizeof(buf), "Connecting WiFi SSID=%s\n", CONFIG::SSID);
    log_line(buf);

    WiFi.disconnect();
    ThisThread::sleep_for(100ms);
    WiFi.begin(CONFIG::SSID, CONFIG::PWD);

    const uint32_t started_ms = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started_ms < WIFI_CONNECT_TIMEOUT_MS) {
        log_line(".");
        ThisThread::sleep_for(WIFI_POLL_DELAY);
    }

    if (WiFi.status() != WL_CONNECTED) {
        snprintf(buf, sizeof(buf), "\nWiFi connect failed, status=%d\n", WiFi.status());
        log_line(buf);
        return false;
    }

    snprintf(buf, sizeof(buf), "\nWiFi connected, ip=%s\n", WiFi.localIP().toString().c_str());
    log_line(buf);
    return true;
}

void scan_wifi_networks() {
    log_line("Scanning WiFi networks...\n");

    const int count = WiFi.scanNetworks();
    if (count <= 0) {
        log_line("No WiFi networks found\n");
        return;
    }

    char buf[192];
    snprintf(buf, sizeof(buf), "Found %d WiFi networks:\n", count);
    log_line(buf);

    for (int i = 0; i < count; ++i) {
        snprintf(buf,
        sizeof(buf),
        "%2d: %s, RSSI=%ld dBm, encryption=%d\n",
        i + 1,
        WiFi.SSID(i),
        WiFi.RSSI(i),
        WiFi.encryptionType(i));
        log_line(buf);
    }
}

void on_mqtt_message(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char msg[MiniMessenger::kMaxPayloadSize + 1];
    const size_t copy_len = (length < MiniMessenger::kMaxPayloadSize) ? length : MiniMessenger::kMaxPayloadSize;
    memcpy(msg, payload, copy_len);
    msg[copy_len] = '\0';

    char buf[192];
    snprintf(buf,
    sizeof(buf),
    "MQTT RX [%s -> %s group=%s]: %s\n",
    metadata.fromBoardId,
    metadata.target,
    metadata.groupId,
    msg);
    log_line(buf);
}

void send_register() {
    char reg[96];
    snprintf(reg,
    sizeof(reg),
    "type=register team_id=%s board_id=%s",
    CONFIG::GROUP_ID,
    CONFIG::BOARD_ID);

    const bool sent = messenger.sendToBoard(CONFIG::SERVER_BOARD_ID, reg);

    char buf[160];
    snprintf(buf, sizeof(buf), "register %s: %s\n", sent ? "sent" : "failed", reg);
    log_line(buf);
}

} // namespace

void func_blink() {
    pinMode((int)PINS::BLUE_LED_PIN, OUTPUT);
    digitalWrite((int)PINS::BLUE_LED_PIN, HIGH);

    while (1) {
        ThisThread::sleep_for(500ms);
        digitalWrite((int)PINS::BLUE_LED_PIN, LOW);
        ThisThread::sleep_for(500ms);
        digitalWrite((int)PINS::BLUE_LED_PIN, HIGH);
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    delay(200);

    task_blink.start(func_blink);

    scan_wifi_networks();

    while (!connect_wifi()) {
        delay(2000);
    }

    messenger.onMessage(on_mqtt_message);
    const bool connected = messenger.begin(CONFIG::SSID,
    CONFIG::PWD,
    CONFIG::MQTT_BROKER_HOST,
    CONFIG::MQTT_BROKER_PORT,
    CONFIG::GROUP_ID,
    CONFIG::BOARD_ID);

    char buf[192];
    snprintf(buf,
    sizeof(buf),
    "MiniMessenger begin broker=%s:%d connected=%d client=%s\n",
    CONFIG::MQTT_BROKER_HOST,
    CONFIG::MQTT_BROKER_PORT,
    connected,
    messenger.clientId());
    log_line(buf);
}

void loop() {
    static unsigned long last_register_ms = 0;
    static unsigned long last_status_ms   = 0;
    static bool last_connected            = false;

    messenger.loop();

    const bool connected = messenger.isConnected();
    if (connected != last_connected) {
        last_connected = connected;

        char buf[160];
        snprintf(buf,
        sizeof(buf),
        "MiniMessenger %s, ip=%s\n",
        connected ? "connected" : "disconnected",
        WiFi.localIP().toString().c_str());
        log_line(buf);
    }

    if (millis() - last_status_ms > 5000 || last_status_ms == 0) {
        last_status_ms = millis();

        char buf[192];
        snprintf(buf,
        sizeof(buf),
        "MQTT status: wifi=%d mqtt=%d broker=%s:%d ip=%s\n",
        WiFi.status(),
        connected,
        CONFIG::MQTT_BROKER_HOST,
        CONFIG::MQTT_BROKER_PORT,
        WiFi.localIP().toString().c_str());
        log_line(buf);
    }

    if (connected && (millis() - last_register_ms > 10000 || last_register_ms == 0)) {
        last_register_ms = millis();
        send_register();
    }

    delay(10);
}
