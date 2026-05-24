#include "core_m7/rpc_bridge.hpp"

#include "config.hpp"
#include "core_m7/ir.hpp"
#include "core_m7/rfid.hpp"
#include "core_m7/serial.hpp"
#include "core_m7/sunlight.hpp"
#include "core_m7/ultrasonic.hpp"
#include "core_m7/wifi_mqtt.hpp"

#include <Arduino.h>
#include <RPC.h>

namespace {

uint32_t last_fast_push_ms = 0;
uint32_t last_slow_push_ms = 0;
int fast_push_seq          = 0;
int slow_push_seq          = 0;

void run_rpc_benchmark() {
    if (!CONFIG::M7::ENABLE_RPC_BENCHMARK) {
        return;
    }

    loggf("[m7-rpc] benchmark start calls=%u\n", CONFIG::M7::RPC_BENCHMARK_CALLS);
    delay(CONFIG::M7::RPC_BENCHMARK_BOOT_WAIT_MS);

    uint32_t total_us = 0;
    uint32_t min_us   = UINT32_MAX;
    uint32_t max_us   = 0;
    uint16_t errors   = 0;

    for (uint16_t i = 0; i < CONFIG::M7::RPC_BENCHMARK_CALLS; i++) {
        const uint32_t started_us  = micros();
        const int result           = RPC.call("m4_ping", static_cast<int>(i)).as<int>();
        const uint32_t duration_us = micros() - started_us;

        total_us += duration_us;
        if (duration_us < min_us) {
            min_us = duration_us;
        }
        if (duration_us > max_us) {
            max_us = duration_us;
        }
        if (result != static_cast<int>(i + 1)) {
            errors++;
        }
        delay(5);
    }

    const uint32_t avg_us = total_us / CONFIG::M7::RPC_BENCHMARK_CALLS;
    loggf("[m7-rpc] benchmark done avg=%luus min=%luus max=%luus errors=%u\n",
    static_cast<unsigned long>(avg_us),
    static_cast<unsigned long>(min_us),
    static_cast<unsigned long>(max_us),
    errors);
}

void push_fast_sensor_data(uint32_t now_ms) {
    if (now_ms - last_fast_push_ms < CONFIG::M7::RPC_FAST_PUSH_INTERVAL_MS &&
    last_fast_push_ms != 0) {
        return;
    }
    last_fast_push_ms = now_ms;

    const uint16_t* ir = ir_values();
    const int seq    = fast_push_seq++;
    const int result = RPC.call("m4_sensor_fast_update",
                          seq,
                          static_cast<int>(ir_position()),
                          static_cast<int>(ir[0]),
                          static_cast<int>(ir[1]),
                          static_cast<int>(ir[2]),
                          static_cast<int>(ir[3]),
                          static_cast<int>(ir[4]),
                          static_cast<int>(ir[5]),
                          static_cast<int>(ir[6]),
                          static_cast<int>(ir[7]),
                          static_cast<int>(ir[8]),
                          static_cast<int>(ultrasonic_front_cm()),
                          static_cast<int>(ultrasonic_left_cm()),
                          static_cast<int>(ultrasonic_right_cm()))
                       .as<int>();
    if (result != seq) {
        loggf("[m7-rpc] fast push mismatch seq=%d rc=%d\n", seq, result);
    }
}

void push_slow_sensor_data(uint32_t now_ms) {
    if (now_ms - last_slow_push_ms < CONFIG::M7::RPC_SLOW_PUSH_INTERVAL_MS &&
    last_slow_push_ms != 0) {
        return;
    }

    last_slow_push_ms = now_ms;

    const int seq    = slow_push_seq++;
    const int result = RPC.call("m4_sensor_slow_update",
                          seq,
                          static_cast<int>(sunlight_value()),
                          static_cast<int>(rfid_last_uid()),
                          wifi_mqtt_is_safety_enabled() ? 1 : 0,
                          wifi_mqtt_is_wifi_connected() ? 1 : 0,
                          wifi_mqtt_is_mqtt_connected() ? 1 : 0)
                       .as<int>();
    if (result != seq) {
        loggf("[m7-rpc] slow push mismatch seq=%d rc=%d\n", seq, result);
    }
}

} // namespace

bool rpc_bridge_begin() {
    loggf("\n[m7] boot\n");
    if (!RPC.begin()) {
        loggf("[m7] RPC.begin failed, M4 may not be running\n");
        return false;
    }

    loggf("[m7] RPC ready, M4 boot requested\n");
    run_rpc_benchmark();
    return true;
}

void rpc_bridge_update(uint32_t now_ms) {
    push_fast_sensor_data(now_ms);
    push_slow_sensor_data(now_ms);
}
