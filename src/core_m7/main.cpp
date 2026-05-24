#include "config.hpp"
#include "core_m7/ir.hpp"
#include "core_m7/rfid.hpp"
#include "core_m7/rpc_bridge.hpp"
#include "core_m7/serial.hpp"
#include "core_m7/sunlight.hpp"
#include "core_m7/ultrasonic.hpp"
#include "core_m7/wifi_mqtt.hpp"

#include <Arduino.h>

void setup() {

    serial_begin();
    rpc_bridge_begin();
    sunlight_begin();
    ir_begin();
    ultrasonic_begin();
    rfid_begin();
    wifi_mqtt_begin();

    loggf("================= ENTRY LOOP ================\n");
}

void loop() {
    const uint32_t now_ms = millis();

    if (CONFIG::M7::ENABLE_WIFI_UPDATE) {
        wifi_mqtt_update(now_ms);
    }

    if (CONFIG::M7::ENABLE_SUNLIGHT_UPDATE) {
        sunlight_update(now_ms);
    }

    if (CONFIG::M7::ENABLE_IR_UPDATE) {
        ir_update(now_ms);
    }

    if (CONFIG::M7::ENABLE_ULTRASONIC_UPDATE) {
        ultrasonic_update(now_ms);
    }

    if (CONFIG::M7::ENABLE_RFID_UPDATE) {
        rfid_update(now_ms);
    }

    if (CONFIG::M7::ENABLE_RPC_UPDATE) {
        rpc_bridge_update(now_ms);
    }

    delay(10);
}
