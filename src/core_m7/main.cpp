#include "config.hpp"
#include "m7_ir.hpp"
#include "m7_led.hpp"
#include "m7_rfid.hpp"
#include "m7_serial.hpp"
#include "m7_sunlight.hpp"
#include "m7_ultrasonic.hpp"
#include "m7_wifi_mqtt.hpp"

#include <Arduino.h>
#include <RPC.h>

namespace {

void log_slow_step(const char* name, uint32_t started_ms) {
    const uint32_t duration_ms = millis() - started_ms;
    if (duration_ms > CONFIG::M7::LOOP_STEP_WARN_MS) {
        serial_logf("[m7-loop] %s took %lums\n",
                    name,
                    static_cast<unsigned long>(duration_ms));
    }
}

} // namespace

void setup() {

    serial_begin();

    serial_logf("\n[m7] boot\n");
    if (!RPC.begin()) {
        serial_logf("[m7] RPC.begin failed, M4 may not be running\n");
    } else {
        serial_logf("[m7] RPC ready, M4 boot requested\n");
    }

    led_begin();
    sunlight_begin();
    ir_begin();
    ultrasonic_begin();
    rfid_begin();
    wifi_mqtt_begin();

    serial_logf("================= ENTRY LOOP ================\n");
}

void loop() {
    const uint32_t now = millis();
    uint32_t step_started_ms = 0;

    step_started_ms = millis();
    led_update(now);
    log_slow_step("led", step_started_ms);

    step_started_ms = millis();
    wifi_mqtt_update(now);
    log_slow_step("mqtt", step_started_ms);

    step_started_ms = millis();
    sunlight_update(now);
    log_slow_step("sunlight", step_started_ms);

    if (CONFIG::M7::ENABLE_IR_UPDATE && wifi_mqtt_is_ready_for_sensors(now)) {
        step_started_ms = millis();
        ir_update(now);
        log_slow_step("ir", step_started_ms);
    }

    if (CONFIG::M7::ENABLE_ULTRASONIC_UPDATE) {
        step_started_ms = millis();
        ultrasonic_update(now);
        log_slow_step("ultrasonic", step_started_ms);
    }

    if (CONFIG::M7::ENABLE_RFID_UPDATE) {
        step_started_ms = millis();
        rfid_update(now);
        log_slow_step("rfid", step_started_ms);
    }

    delay(10);
}
