#include <Arduino.h>
#include <MiniMessenger.h>

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

struct MotorPins {
    pin_size_t en;
    pin_size_t forward;
    pin_size_t backward;
};

constexpr MotorPins MOTORS[] = {
    {D3, D31, D29}, // front-left
    {D4, D24, D26}, // front-right
    {D2, D25, D27}, // rear-left
    {D5, D30, D28}, // rear-right
};

constexpr pin_size_t STATUS_LED_RED   = D14;
constexpr pin_size_t STATUS_LED_GREEN = D15;
constexpr pin_size_t STATUS_LED_BLUE  = LEDB;
constexpr int FULL_SPEED_PWM   = 255;

MiniMessenger messenger;

bool safetyEnabled = false;
unsigned long lastRegisterMs = 0;
unsigned long lastStatusMs   = 0;
unsigned long lastLedMs      = 0;
bool lastConnected           = false;
bool ledBlinkState           = false;
bool lastMotorEnabled        = false;

void logf(const char* fmt, ...) {
    char buf[192];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.print(buf);
    Serial1.print(buf);
}

void setLed(bool on) {
    // GIGA RGB LEDs are active-low.
    digitalWrite(STATUS_LED_RED, on ? LOW : HIGH);
    digitalWrite(STATUS_LED_GREEN, on ? LOW : HIGH);
    digitalWrite(STATUS_LED_BLUE, on ? LOW : HIGH);
}

void updateLed() {
    if (safetyEnabled) {
        setLed(true);
        return;
    }

    if (millis() - lastLedMs > 250 || lastLedMs == 0) {
        lastLedMs = millis();
        ledBlinkState = !ledBlinkState;
        setLed(ledBlinkState);
    }
}

void stopMotors() {
    for (const auto& motor : MOTORS) {
        analogWrite(motor.en, 0);
        digitalWrite(motor.forward, LOW);
        digitalWrite(motor.backward, LOW);
    }
}

void driveForwardFullSpeed() {
    for (const auto& motor : MOTORS) {
        digitalWrite(motor.forward, HIGH);
        digitalWrite(motor.backward, LOW);
        analogWrite(motor.en, FULL_SPEED_PWM);
    }
}

void updateMotors() {
    if (safetyEnabled == lastMotorEnabled) {
        return;
    }

    lastMotorEnabled = safetyEnabled;

    if (safetyEnabled) {
        driveForwardFullSpeed();
        logf("MOTORS: full speed forward\n");
    } else {
        stopMotors();
        logf("MOTORS: stopped\n");
    }
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
        if (safetyEnabled) {
            logf("SAFETY: emergency/disable active\n");
        }
        safetyEnabled = false;
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

    pinMode(STATUS_LED_RED, OUTPUT);
    pinMode(STATUS_LED_GREEN, OUTPUT);
    pinMode(STATUS_LED_BLUE, OUTPUT);
    setLed(false);

    for (const auto& motor : MOTORS) {
        pinMode(motor.en, OUTPUT);
        pinMode(motor.forward, OUTPUT);
        pinMode(motor.backward, OUTPUT);
    }
    stopMotors();

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
    updateLed();
    updateMotors();

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

    delay(10);
}
