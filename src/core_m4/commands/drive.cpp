#include "chassis.hpp"
#include "config.hpp"
#include "line_follower.hpp"
#include "logger.hpp"
#include "mission.hpp"
#include "motor.hpp"
#include "sensors.hpp"
#include "shell.hpp"
#include "state.hpp"
#include "wall_follower.hpp"

#include <Arduino.h>
#include <math.h>
#include <mbed.h>
#include <stdlib.h>
#include <string.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

static int32_t start_fl_count = 0;
static int32_t start_fr_count = 0;
static int32_t start_rl_count = 0;
static int32_t start_rr_count = 0;
static float last_target_cm   = 0.0f;

static bool parse_float(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end         = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0') return false;

    *out = value;
    return true;
}

static bool parse_int16(const char* text, int16_t* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end        = nullptr;
    const long value = strtol(text, &end, 0);
    if (end == text || *end != '\0') return false;

    *out = static_cast<int16_t>(value);
    return true;
}

static void reset_drive_counts() {
    start_fl_count = motor_fl().count();
    start_fr_count = motor_fr().count();
    start_rl_count = motor_rl().count();
    start_rr_count = motor_rr().count();
}

static float traveled_cm() {
    const float fl             = static_cast<float>(motor_fl().count() - start_fl_count);
    const float fr             = static_cast<float>(motor_fr().count() - start_fr_count);
    const float rl             = static_cast<float>(motor_rl().count() - start_rl_count);
    const float rr             = static_cast<float>(motor_rr().count() - start_rr_count);
    const float forward_counts = fabsf((fl + fr + rl + rr) * 0.25f);
    return forward_counts / CHASSIS_ENCODER_COUNTS_PER_CM;
}

static void print_drive_status() {
    loggf("drive target=%.1f traveled=%.1f front=%d vx=%.1f wheel=%.1f/%.1f/%.1f/%.1f pwm=%d/%d/%d/%d\n",
    last_target_cm,
    traveled_cm(),
    sensors.ultrasonic_front_cm(),
    chassis.target_vx(),
    chassis.wheel_fl(),
    chassis.wheel_fr(),
    chassis.wheel_rl(),
    chassis.wheel_rr(),
    motor_fl().applied_pwm(),
    motor_fr().applied_pwm(),
    motor_rl().applied_pwm(),
    motor_rr().applied_pwm());
}

static bool ensure_running() {
    if (running_state != RunningState::STOPPED) return true;

    loggf("drive aborted: robot is stopped. use start first.\n");
    return false;
}

static void drive_until(float vx, float target_cm, int16_t front_stop_cm) {
    mission_stop();
    wall_follower_stop();
    line_follower_stop();
    reset_drive_counts();
    last_target_cm = target_cm;

    chassis.set_target(vx, 0.0f, 0.0f);
    loggf("drive begin vx=%.1f target=%.1f front=%d counts_per_cm=%.3f\n",
    vx,
    target_cm,
    front_stop_cm,
    CHASSIS_ENCODER_COUNTS_PER_CM);

    while (true) {
        if (running_state == RunningState::STOPPED) {
            chassis_stop();
            loggf("drive stopped by state traveled=%.1f\n", traveled_cm());
            return;
        }

        const int16_t front_cm = sensors.ultrasonic_front_cm();
        if (front_stop_cm > 0 && front_cm > 0 && front_cm <= front_stop_cm) {
            chassis_stop();
            loggf("drive front stop front=%d threshold=%d traveled=%.1f\n", front_cm, front_stop_cm, traveled_cm());
            return;
        }

        if (target_cm > 0.0f && traveled_cm() >= target_cm) {
            chassis_stop();
            loggf("drive done traveled=%.1f target=%.1f\n", traveled_cm(), target_cm);
            return;
        }

        ThisThread::sleep_for(20ms);
    }
}

static void drive_cmd(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        print_drive_status();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "stop") == 0) {
        mission_stop();
        wall_follower_stop();
        line_follower_stop();
        chassis_stop();
        print_drive_status();
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "front") == 0) {
        if (argc != 4) {
            loggf("usage: drive front <speed> <front_cm>\n");
            return;
        }

        if (!ensure_running()) return;

        float speed = 0.0f;
        int16_t front_cm = -1;
        if (!parse_float(argv[2], &speed) || !parse_int16(argv[3], &front_cm) || speed <= 0.0f || front_cm <= 0) {
            loggf("drive front args must be positive numbers\n");
            return;
        }

        drive_until(fabsf(speed), -1.0f, front_cm);
        return;
    }

    if (argc >= 2 && (strcmp(argv[1], "forward") == 0 || strcmp(argv[1], "backward") == 0)) {
        if (argc < 4 || argc > 5) {
            loggf("usage: drive forward <speed> <cm> [front_cm] | drive backward <speed> <cm>\n");
            return;
        }

        if (!ensure_running()) return;

        float speed = 0.0f;
        float distance_cm = 0.0f;
        int16_t front_cm = -1;
        if (!parse_float(argv[2], &speed) || !parse_float(argv[3], &distance_cm) || speed <= 0.0f || distance_cm <= 0.0f) {
            loggf("drive speed and cm must be positive numbers\n");
            return;
        }

        if (argc == 5 && (!parse_int16(argv[4], &front_cm) || front_cm <= 0)) {
            loggf("drive front_cm must be a positive integer\n");
            return;
        }

        const bool backward = strcmp(argv[1], "backward") == 0;
        if (backward) front_cm = -1;

        drive_until(backward ? -fabsf(speed) : fabsf(speed), distance_cm, front_cm);
        return;
    }

    loggf("usage: drive forward <speed> <cm> [front_cm] | drive backward <speed> <cm> | drive front <speed> <front_cm> | drive stop | drive status\n");
}

SHELL_COMMAND("drive", drive_cmd, "drive straight: drive forward/backward <speed> <cm>, drive front <speed> <front_cm>")
