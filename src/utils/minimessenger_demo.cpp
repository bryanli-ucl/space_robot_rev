#include <Arduino.h>
#include <MiniMessenger.h>
#include <Servo.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/*
 * MiniMessenger standalone safety demo for the PlatformIO mini_demo env.
 *
 * Build/upload with:
 *   pio run -e mini_demo
 *   pio run -e mini_demo -t upload
 *
 * This mirrors MiniMessenger's GIGA_Servo_Safety_Test example while using this
 * project's WiFi and challenge-server settings.
 */

namespace {

constexpr const char* WIFI_SSID     = "PhaseSpaceNetwork_2.4G";
constexpr const char* WIFI_PASSWORD = "8igMacNet";
constexpr const char* BROKER_HOST   = "192.168.0.74";
constexpr uint16_t BROKER_PORT      = 1883;
constexpr const char* GROUP_ID      = "12";
constexpr const char* BOARD_ID      = "Bryan";
constexpr const char* SERVER_ID     = "server";
constexpr int SERVO_PIN             = 9;

MiniMessenger messenger;
Servo myServo;

bool safetyEnabled = false;
unsigned long lastRegisterMs = 0;
unsigned long lastStatusMs   = 0;
unsigned long lastMoveMs     = 0;
bool lastConnected           = false;
int servoPos                 = 90;
int sweepDirection           = 1;

void logf(const char* fmt, ...) {
    char buf[192];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.print(buf);
    Serial1.print(buf);
}

void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
    char msg[160];
    const size_t copyLen = (length < sizeof(msg) - 1) ? length : sizeof(msg) - 1;
    memcpy(msg, payload, copyLen);
    msg[copyLen] = '\0';

    logf("MQTT RX [%s -> %s group=%s]: %s\n",
         metadata.fromBoardId,
         metadata.target,
         metadata.groupId,
         msg);

    if (strstr(msg, "type=heartbeat enable=1")) {
        if (!safetyEnabled) {
            logf("SAFETY: heartbeat enabled\n");
        }
        safetyEnabled = true;
        return;
    }

    if (strstr(msg, "type=heartbeat enable=0")) {
        if (safetyEnabled) {
            logf("SAFETY: heartbeat disabled\n");
        }
        safetyEnabled = false;
        return;
    }

    if (strstr(msg, "type=emergency enabled=true") ||
        strstr(msg, "type=disable enabled=false")) {
        safetyEnabled = false;
        logf("SAFETY: emergency/disable active\n");
        return;
    }
}

} // namespace

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    delay(200);

    logf("\n--- MINIMESSENGER MINI_DEMO START ---\n");
    logf("WiFi=%s broker=%s:%u group=%s board=%s\n",
         WIFI_SSID,
         BROKER_HOST,
         BROKER_PORT,
         GROUP_ID,
         BOARD_ID);

    myServo.attach(SERVO_PIN);
    myServo.write(90);

    messenger.onMessage(onMessage);
    const bool connected = messenger.begin(WIFI_SSID,
                                           WIFI_PASSWORD,
                                           BROKER_HOST,
                                           BROKER_PORT,
                                           GROUP_ID,
                                           BOARD_ID);

    lastConnected = connected;
    logf("MiniMessenger begin connected=%d client=%s\n",
         connected,
         messenger.clientId());
}

void loop() {
    messenger.loop();

    const bool connected = messenger.isConnected();
    if (connected != lastConnected) {
        lastConnected = connected;
        logf("MiniMessenger %s, ip=%s\n",
             connected ? "connected" : "disconnected",
             WiFi.localIP().toString().c_str());
    }

    if (millis() - lastStatusMs > 5000 || lastStatusMs == 0) {
        lastStatusMs = millis();
        logf("status: wifi=%d mqtt=%d ip=%s safety=%d\n",
             WiFi.status(),
             connected,
             WiFi.localIP().toString().c_str(),
             safetyEnabled);
    }

    if (millis() - lastRegisterMs > 10000 || lastRegisterMs == 0) {
        lastRegisterMs = millis();

        char reg[96];
        snprintf(reg,
                 sizeof(reg),
                 "type=register team_id=%s board_id=%s",
                 GROUP_ID,
                 BOARD_ID);

        const bool sent = messenger.sendToBoard(SERVER_ID, reg);
        logf("register %s: %s\n", sent ? "sent" : "failed", reg);
    }

    if (!safetyEnabled) {
        myServo.write(90);
        delay(10);
        return;
    }

    if (!myServo.attached()) {
        myServo.attach(SERVO_PIN);
    }

    if (millis() - lastMoveMs > 15) {
        lastMoveMs = millis();

        servoPos += sweepDirection;
        if (servoPos >= 180 || servoPos <= 0) {
            sweepDirection *= -1;
        }

        myServo.write(servoPos);
    }

    delay(10);
}
