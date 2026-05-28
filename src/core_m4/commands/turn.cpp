#include "chassis.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "logger.hpp"
#include "shell.hpp"
#include "state.hpp"

#include <Arduino.h>
#include <math.h>
#include <mbed.h>
#include <stdlib.h>
#include <string.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

static bool parse_float(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0') return false;

    *out = value;
    return true;
}

static bool parse_uint32(const char* text, uint32_t* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end = nullptr;
    const unsigned long value = strtoul(text, &end, 0);
    if (end == text || *end != '\0') return false;

    *out = static_cast<uint32_t>(value);
    return true;
}

static float wrap_deg_180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

static float wrap_deg_360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

static bool parse_turn_degrees(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    if (strcmp(text, "left") == 0) {
        *out = 90.0f;
        return true;
    }

    if (strcmp(text, "right") == 0) {
        *out = -90.0f;
        return true;
    }

    return parse_float(text, out);
}

static void turn_cmd(int argc, char** argv) {
    if (argc < 2 || argc > 5) {
        loggf("usage: turn <deg|left|right> [max_w] [tolerance_deg] [timeout_ms]\n");
        return;
    }

    if (running_state == RunningState::STOPPED) {
        loggf("turn aborted: robot is stopped. use start first.\n");
        return;
    }

    if (!imu.yaw_is_ready()) {
        loggf("turn aborted: imu yaw is not ready\n");
        return;
    }

    float delta_deg = 0.0f;
    if (!parse_turn_degrees(argv[1], &delta_deg)) {
        loggf("turn deg must be a number, left, or right\n");
        return;
    }

    float max_w = TURN_MAX_WHEEL_SPEED;
    if (argc >= 3 && !parse_float(argv[2], &max_w)) {
        loggf("turn max_w must be a number\n");
        return;
    }

    float tolerance_deg = TURN_TOLERANCE_DEG;
    if (argc >= 4 && !parse_float(argv[3], &tolerance_deg)) {
        loggf("turn tolerance_deg must be a number\n");
        return;
    }

    uint32_t timeout_ms = TURN_TIMEOUT_MS;
    if (argc >= 5 && !parse_uint32(argv[4], &timeout_ms)) {
        loggf("turn timeout_ms must be an integer\n");
        return;
    }

    max_w = constrain(fabsf(max_w), TURN_MIN_WHEEL_SPEED, CHASSIS_MAX_WHEEL_SPEED);
    tolerance_deg = fmaxf(0.2f, fabsf(tolerance_deg));

    const float start_yaw = imu.yaw_deg();
    const float target_yaw = wrap_deg_360(start_yaw + delta_deg);
    float prev_err = wrap_deg_180(target_yaw - start_yaw);
    uint8_t confirm = 0;
    const uint32_t start_ms = millis();
    uint32_t last_ms = start_ms;

    loggf("turn begin delta=%.2f start=%.2f target=%.2f max_w=%.1f tol=%.1f timeout=%lums\n",
          delta_deg,
          start_yaw,
          target_yaw,
          max_w,
          tolerance_deg,
          static_cast<unsigned long>(timeout_ms));

    while (millis() - start_ms < timeout_ms) {
        if (running_state == RunningState::STOPPED) {
            chassis_stop();
            loggf("turn stopped by state\n");
            return;
        }

        const uint32_t now_ms = millis();
        const float dt_s = fmaxf(0.001f, static_cast<float>(now_ms - last_ms) * 0.001f);
        last_ms = now_ms;

        const float yaw = imu.yaw_deg();
        const float err = wrap_deg_180(target_yaw - yaw);
        const float derr = (err - prev_err) / dt_s;
        prev_err = err;

        if (fabsf(err) <= tolerance_deg) {
            confirm++;
            chassis.set_target(0.0f, 0.0f, 0.0f);
            if (confirm >= TURN_CONFIRM_COUNT) {
                chassis_stop();
                loggf("turn done yaw=%.2f target=%.2f err=%.2f elapsed=%lums\n",
                      yaw,
                      target_yaw,
                      err,
                      static_cast<unsigned long>(millis() - start_ms));
                return;
            }
        } else {
            confirm = 0;
            float w = TURN_KP * err + TURN_KD * derr;
            w = constrain(w, -max_w, max_w);
            if (fabsf(w) < TURN_MIN_WHEEL_SPEED) w = w >= 0.0f ? TURN_MIN_WHEEL_SPEED : -TURN_MIN_WHEEL_SPEED;
            chassis.set_target(0.0f, 0.0f, TURN_DIRECTION * w);
        }

        ThisThread::sleep_for(20ms);
    }

    chassis_stop();
    loggf("turn timeout yaw=%.2f target=%.2f err=%.2f\n",
          imu.yaw_deg(),
          target_yaw,
          wrap_deg_180(target_yaw - imu.yaw_deg()));
}

SHELL_COMMAND("turn", turn_cmd, "turn by IMU yaw: turn <deg|left|right> [max_w] [tolerance_deg] [timeout_ms]")
