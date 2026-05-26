#include "core_m4/heartbeat.hpp"

#include "config.hpp"
#include "core_m4/chassis.hpp"
#include "core_m4/motion_control.hpp"
#include "core_m4/rpc_bridge.hpp"
#include "core_m4/serial.hpp"
#include "core_m4/state.hpp"

#include <Arduino.h>
#include <mbed.h>

using namespace ::rtos;
using namespace std::chrono_literals;

extern Thread task_heartbeat;

namespace {

enum class LedStatus {
    Off,
    On,
    Blink,
};

bool heartbeat_started = false;

constexpr uint32_t SENSOR_LOG_INTERVAL_MS = 2000;

void write_led(pin_size_t pin, LedStatus status, bool blink_on) {
    bool active = false;
    switch (status) {
    case LedStatus::Off:
        active = false;
        break;
    case LedStatus::On:
        active = true;
        break;
    case LedStatus::Blink:
        active = blink_on;
        break;
    }

    digitalWrite(pin, active ? LOW : HIGH);
}

void apply_led_state(bool blink_on) {
    LedStatus red   = LedStatus::Off;
    LedStatus green = LedStatus::Off;
    LedStatus blue  = LedStatus::Blink;

    switch (running_state) {
    case RunningState::IDLE:
        red   = LedStatus::On;
        green = LedStatus::Off;
        blue  = LedStatus::Off;
        break;
    case RunningState::STOPPED:
        red   = LedStatus::Blink;
        green = LedStatus::Off;
        blue  = LedStatus::Blink;
        break;
    case RunningState::REVIVING:
        red   = LedStatus::Off;
        green = LedStatus::On;
        blue  = LedStatus::Off;
        break;
    }

    write_led(CONFIG::M4::STATUS_RED_LED_PIN, red, blink_on);
    write_led(CONFIG::M4::STATUS_GREEN_LED_PIN, green, blink_on);
    write_led(CONFIG::M4::STATUS_BLUE_LED_PIN, blue, blink_on);
}

void toggle_stop_state() {
    if (running_state == RunningState::STOPPED) {
        running_state = RunningState::IDLE;
        m4_chassis().enable();
        loggf("[m4-heartbeat] running_state=%s\n", running_state_name(running_state));
        return;
    }

    running_state = RunningState::STOPPED;
    motion_state  = MotionState::IDLE;
    motion_force_stop(true);
    loggf("[m4-heartbeat] running_state=%s\n", running_state_name(running_state));
}

void update_buttons(int8_t* kill_stable) {
    if (digitalRead(CONFIG::M4::KILLSWITCH_BUTTON_PIN) == LOW) {
        if (*kill_stable < INT8_MAX) {
            (*kill_stable)++;
        }

        if (*kill_stable == static_cast<int8_t>(CONFIG::M4::KILLSWITCH_DEBOUNCE_TICKS)) {
            toggle_stop_state();
        }
        return;
    }

    *kill_stable = 0;
    if (digitalRead(CONFIG::M4::REVIVING_BUTTON_PIN) == LOW) {
        if (running_state == RunningState::IDLE) {
            running_state = RunningState::REVIVING;
            loggf("[m4-heartbeat] running_state=%s\n", running_state_name(running_state));
        }
    } else if (running_state == RunningState::REVIVING) {
        running_state = RunningState::IDLE;
        m4_chassis().enable();
        loggf("[m4-heartbeat] running_state=%s\n", running_state_name(running_state));
    }
}

void log_sensor_snapshot() {
    loggf("[m4-heartbeat] sensors ir_pos=%d side=%d/%d ir=%d/%d/%d/%d/%d/%d/%d/%d/%d us=%d/%d/%d rfid=%d sunlight=%d\n",
    rpc_bridge_ir_pos(),
    rpc_bridge_ir_side_left() ? 1 : 0,
    rpc_bridge_ir_side_right() ? 1 : 0,
    rpc_bridge_ir_raw(0),
    rpc_bridge_ir_raw(1),
    rpc_bridge_ir_raw(2),
    rpc_bridge_ir_raw(3),
    rpc_bridge_ir_raw(4),
    rpc_bridge_ir_raw(5),
    rpc_bridge_ir_raw(6),
    rpc_bridge_ir_raw(7),
    rpc_bridge_ir_raw(8),
    rpc_bridge_ultrasonic_front_cm(),
    rpc_bridge_ultrasonic_left_cm(),
    rpc_bridge_ultrasonic_right_cm(),
    rpc_bridge_rfid_uid(),
    rpc_bridge_sunlight());
}

void heartbeat_entry() {
    pinMode(CONFIG::M4::STATUS_RED_LED_PIN, OUTPUT);
    pinMode(CONFIG::M4::STATUS_GREEN_LED_PIN, OUTPUT);
    pinMode(CONFIG::M4::STATUS_BLUE_LED_PIN, OUTPUT);
    pinMode(CONFIG::M4::REVIVING_BUTTON_PIN, INPUT_PULLUP);
    pinMode(CONFIG::M4::KILLSWITCH_BUTTON_PIN, INPUT_PULLUP);

    bool blink_on = false;
    uint8_t blink_ticks = 0;
    int8_t kill_stable = 0;
    uint32_t last_log_ms = 0;
    uint32_t last_sensor_log_ms = 0;

    apply_led_state(blink_on);
    loggf("[m4-heartbeat] task ready state=%s motion=%s\n",
    running_state_name(running_state),
    motion_state_name(motion_state));

    while (true) {
        if (++blink_ticks >= CONFIG::M4::HEARTBEAT_BLINK_TICKS) {
            blink_ticks = 0;
            blink_on    = !blink_on;
        }

        update_buttons(&kill_stable);

        if (running_state == RunningState::STOPPED) {
            motion_state = MotionState::IDLE;
            motion_force_stop(true);
        }

        apply_led_state(blink_on);

        const uint32_t now_ms = millis();
        if (last_sensor_log_ms == 0 || now_ms - last_sensor_log_ms >= SENSOR_LOG_INTERVAL_MS) {
            last_sensor_log_ms = now_ms;
            log_sensor_snapshot();
        }

        if (last_log_ms == 0 || now_ms - last_log_ms >= CONFIG::M4::HEARTBEAT_LOG_INTERVAL_MS) {
            last_log_ms = now_ms;
            loggf("[m4-heartbeat] state=%s motion=%s ms=%lu\n",
            running_state_name(running_state),
            motion_state_name(motion_state),
            static_cast<unsigned long>(now_ms));
        }

        ThisThread::sleep_for(std::chrono::milliseconds(CONFIG::M4::HEARTBEAT_TASK_INTERVAL_MS));
    }
}

} // namespace

void heartbeat_begin() {
    if (heartbeat_started || !CONFIG::M4::ENABLE_HEARTBEAT_TASK) {
        return;
    }

    const osStatus status = task_heartbeat.start(heartbeat_entry);
    heartbeat_started     = status == osOK;
    if (!heartbeat_started) {
        loggf("[m4-heartbeat] start failed status=%d\n", status);
    }
}
