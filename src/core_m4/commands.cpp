#include "core_m4/commands.hpp"

#include "core_m4/bash.hpp"
#include "core_m4/chassis.hpp"
#include "core_m4/imu.hpp"
#include "core_m4/motion_control.hpp"
#include "core_m4/rpc_bridge.hpp"
#include "core_m4/serial.hpp"
#include "core_m4/state.hpp"

#include <Arduino.h>
#include <mbed.h>
#include <mbed_stats.h>

#include <array>
#include <stdlib.h>
#include <string.h>

using namespace std::chrono_literals;
using namespace ::rtos;

extern Thread task_command_worker;

namespace {

constexpr size_t COMMAND_QUEUE_DEPTH = 16;
constexpr size_t COMMAND_MESSAGE_SIZE = 256;

Mail<std::array<char, COMMAND_MESSAGE_SIZE>, COMMAND_QUEUE_DEPTH> command_mail;

bool commands_registered = false;
bool command_worker_started = false;

bool parse_float(const char* text, float* out) {
    if (text == nullptr || out == nullptr) {
        return false;
    }

    char* end = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0') {
        return false;
    }

    *out = value;
    return true;
}

bool parse_int(const char* text, int* out) {
    if (text == nullptr || out == nullptr) {
        return false;
    }

    char* end = nullptr;
    const long value = strtol(text, &end, 0);
    if (end == text || *end != '\0') {
        return false;
    }

    *out = static_cast<int>(value);
    return true;
}

bool parse_uint32(const char* text, uint32_t* out) {
    if (text == nullptr || out == nullptr) {
        return false;
    }

    char* end = nullptr;
    const unsigned long value = strtoul(text, &end, 0);
    if (end == text || *end != '\0') {
        return false;
    }

    *out = static_cast<uint32_t>(value);
    return true;
}

bool parse_wall_side(const char* text, WallSide* side) {
    if (text == nullptr || side == nullptr) {
        return false;
    }

    if (strcmp(text, "l") == 0 || strcmp(text, "left") == 0) {
        *side = WallSide::Left;
        return true;
    }

    if (strcmp(text, "r") == 0 || strcmp(text, "right") == 0) {
        *side = WallSide::Right;
        return true;
    }

    return false;
}

Motor* select_motor(const char* id) {
    if (strcmp(id, "fl") == 0) return &m4_motor_fl();
    if (strcmp(id, "fr") == 0) return &m4_motor_fr();
    if (strcmp(id, "rl") == 0) return &m4_motor_rl();
    if (strcmp(id, "rr") == 0) return &m4_motor_rr();
    return nullptr;
}

void print_motor(const char* id, Motor& motor) {
    command_tx("motor %s enc=%ld d=%ld raw=%.2f speed=%.2f target=%.2f err=%.2f der=%.2f int=%.2f out=%.2f pwm=%d pid=%.3f %.3f %.3f mode=%s%s\n",
    id,
    static_cast<long>(motor.count()),
    static_cast<long>(motor.get_last_delta()),
    motor.get_raw_speed(),
    motor.get_current_speed(),
    motor.get_target_speed(),
    motor.get_last_error(),
    motor.get_derivative(),
    motor.get_integral(),
    motor.get_output(),
    motor.get_applied_pwm(),
    motor.get_kp(),
    motor.get_ki(),
    motor.get_kd(),
    motor.is_position_control() ? "pos" : "speed",
    motor.is_manual_pwm() ? "+manual" : "");
}

bool ensure_not_stopped(const char* name) {
    if (running_state != RunningState::STOPPED) {
        return true;
    }

    command_tx("%s aborted: robot is stopped. use 'start' first.\n", name);
    return false;
}

bool motion_ok(const char* step, MotionResult result, MotionResult expected) {
    command_tx("%s: %s\n", step, motion_result_name(result));
    return result == expected;
}

bool wait_for_door_response(uint32_t timeout_ms) {
    const uint32_t start_ms = millis();
    while (millis() - start_ms < timeout_ms) {
        if (running_state == RunningState::STOPPED) {
            return false;
        }

        if (rpc_bridge_door_response_ready()) {
            const bool granted = rpc_bridge_door_response_granted();
            command_tx("door response: granted=%d\n", granted ? 1 : 0);
            return granted;
        }

        ThisThread::sleep_for(50ms);
    }

    command_tx("door response timeout\n");
    return false;
}

bool wait_for_front_clear(float clear_cm, uint32_t timeout_ms) {
    const uint32_t start_ms = millis();
    uint8_t stable_count = 0;

    while (millis() - start_ms < timeout_ms) {
        if (running_state == RunningState::STOPPED) {
            return false;
        }

        const int front_cm = rpc_bridge_ultrasonic_front_cm();
        if (front_cm < 0 || static_cast<float>(front_cm) > clear_cm) {
            if (++stable_count >= 5) {
                command_tx("front clear: %dcm\n", front_cm);
                return true;
            }
        } else {
            stable_count = 0;
        }

        ThisThread::sleep_for(100ms);
    }

    command_tx("front clear timeout front=%dcm\n", rpc_bridge_ultrasonic_front_cm());
    return false;
}

uint32_t configured_or_arg_uid(uint32_t configured_uid, int argc, char** argv, int index) {
    uint32_t uid = configured_uid;
    if (argc > index) {
        parse_uint32(argv[index], &uid);
    }
    return uid;
}

void cmd_help(int argc, char** argv) {
    if (argc == 1) {
        command_tx("commands:\n");
        for (const auto& command : bash.commands) {
            command_tx("  %-10s %s\n", command.name, command.help);
        }
        return;
    }

    for (const auto& command : bash.commands) {
        if (strcmp(argv[1], command.name) == 0) {
            command_tx("%s: %s\n", command.name, command.help);
            return;
        }
    }

    command_tx("help: unknown command '%s'\n", argv[1]);
}

void cmd_start(int argc, char** argv) {
    (void)argc;
    (void)argv;

    if (running_state == RunningState::STOPPED) {
        running_state = RunningState::IDLE;
        m4_chassis().enable();
    }

    command_tx("state=%s motion=%s\n", running_state_name(running_state), motion_state_name(motion_state));
}

void cmd_stop(int argc, char** argv) {
    (void)argc;
    (void)argv;

    running_state = RunningState::STOPPED;
    motion_state = MotionState::IDLE;
    motion_force_stop(true);
    command_tx("state=%s motion=%s\n", running_state_name(running_state), motion_state_name(motion_state));
}

void cmd_state(int argc, char** argv) {
    (void)argc;
    (void)argv;

    command_tx("state=%s motion=%s chassis=%.2f/%.2f/%.2f yaw_ready=%d yaw=%.2f\n",
    running_state_name(running_state),
    motion_state_name(motion_state),
    m4_chassis().get_target_vx(),
    m4_chassis().get_target_vy(),
    m4_chassis().get_target_w(),
    imu_yaw_ready() ? 1 : 0,
    imu_yaw_deg());
}

void print_stat() {
    mbed_stats_heap_t heap;
    mbed_stats_heap_get(&heap);

    command_tx("heap total=%lu used=%lu peak=%lu free=%lu\n",
    heap.reserved_size,
    heap.current_size,
    heap.max_size,
    heap.reserved_size - heap.current_size);

    mbed_stats_stack_t stack[16];
    const size_t count = mbed_stats_stack_get_each(stack, 16);
    command_tx("stacks count=%u\n", static_cast<unsigned>(count));
    for (size_t i = 0; i < count; i++) {
        const unsigned long used = stack[i].max_size;
        const unsigned long total = stack[i].reserved_size;
        const unsigned long usage = total ? used * 100 / total : 0;
        command_tx("  tid=%lu used=%lu total=%lu use=%lu%%\n",
        static_cast<unsigned long>(stack[i].thread_id),
        used,
        total,
        usage);
    }
}

void cmd_print(int argc, char** argv) {
    if (argc != 2) {
        command_tx("usage: print ir|dist|rfid|sunlight|imu|state|motor|stat\n");
        return;
    }

    if (strcmp(argv[1], "ir") == 0) {
        command_tx("IR pos=%d side_l=%d side_r=%d raw=%d %d %d %d %d %d %d %d %d\n",
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
        rpc_bridge_ir_raw(8));
    } else if (strcmp(argv[1], "dist") == 0) {
        command_tx("Dist front=%dcm left=%dcm right=%dcm\n",
        rpc_bridge_ultrasonic_front_cm(),
        rpc_bridge_ultrasonic_left_cm(),
        rpc_bridge_ultrasonic_right_cm());
    } else if (strcmp(argv[1], "rfid") == 0) {
        command_tx("RFID uid=%lu\n", static_cast<unsigned long>(rpc_bridge_rfid_uid()));
    } else if (strcmp(argv[1], "sunlight") == 0) {
        command_tx("Sunlight=%d\n", rpc_bridge_sunlight());
    } else if (strcmp(argv[1], "imu") == 0) {
        command_tx("IMU ready=%d yaw_ready=%d yaw=%.2f\n",
        imu_is_ready() ? 1 : 0,
        imu_yaw_ready() ? 1 : 0,
        imu_yaw_deg());
    } else if (strcmp(argv[1], "state") == 0) {
        cmd_state(0, nullptr);
    } else if (strcmp(argv[1], "motor") == 0) {
        print_motor("fl", m4_motor_fl());
        print_motor("fr", m4_motor_fr());
        print_motor("rl", m4_motor_rl());
        print_motor("rr", m4_motor_rr());
    } else if (strcmp(argv[1], "stat") == 0) {
        print_stat();
    } else {
        command_tx("usage: print ir|dist|rfid|sunlight|imu|state|motor|stat\n");
    }
}

void cmd_imu(int argc, char** argv) {
    if (argc == 1 || strcmp(argv[1], "status") == 0) {
        command_tx("IMU ready=%d yaw_ready=%d calibrating=%d yaw=%.2f\n",
        imu_is_ready() ? 1 : 0,
        imu_yaw_ready() ? 1 : 0,
        imu_is_calibrating() ? 1 : 0,
        imu_yaw_deg());
        return;
    }

    bool gyro = false;
    bool mag = false;
    if (strcmp(argv[1], "gyro") == 0) {
        gyro = true;
    } else if (strcmp(argv[1], "mag") == 0) {
        mag = true;
    } else if (strcmp(argv[1], "all") == 0 || strcmp(argv[1], "calib") == 0 || strcmp(argv[1], "calibrate") == 0) {
        gyro = true;
        mag = true;
    } else {
        command_tx("usage: imu [status|gyro|mag|all]\n");
        return;
    }

    if (!imu_is_ready()) {
        command_tx("imu calibration rejected: IMU is not ready\n");
        return;
    }

    if (!imu_request_calibration(gyro, mag)) {
        command_tx("imu calibration rejected: busy or already queued\n");
        return;
    }

    command_tx("imu calibration queued gyro=%d mag=%d\n", gyro ? 1 : 0, mag ? 1 : 0);
}

void cmd_motor(int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "enc") == 0) {
        command_tx("motor enc fl=%ld fr=%ld rl=%ld rr=%ld\n",
        static_cast<long>(m4_motor_fl().count()),
        static_cast<long>(m4_motor_fr().count()),
        static_cast<long>(m4_motor_rl().count()),
        static_cast<long>(m4_motor_rr().count()));
        return;
    }

    if (argc == 2 && strcmp(argv[1], "enc0") == 0) {
        m4_motor_fl().reset_count();
        m4_motor_fr().reset_count();
        m4_motor_rl().reset_count();
        m4_motor_rr().reset_count();
        command_tx("motor enc counts reset\n");
        return;
    }

    if (argc < 3) {
        command_tx("usage: motor enc|enc0 OR motor <fl|fr|rl|rr> enc|enc0|auto|vel <target>|pid <kp> <ki> <kd>|<-255..255>\n");
        return;
    }

    Motor* motor = select_motor(argv[1]);
    if (motor == nullptr) {
        command_tx("unknown motor '%s'. use fl/fr/rl/rr.\n", argv[1]);
        return;
    }

    if (strcmp(argv[2], "enc") == 0) {
        command_tx("motor %s enc=%ld\n", argv[1], static_cast<long>(motor->count()));
    } else if (strcmp(argv[2], "enc0") == 0) {
        motor->reset_count();
        command_tx("motor %s enc reset\n", argv[1]);
    } else if (strcmp(argv[2], "auto") == 0) {
        motor->clear_manual_pwm();
        command_tx("motor %s returned to chassis control\n", argv[1]);
    } else if (strcmp(argv[2], "vel") == 0 && argc == 4) {
        float target = 0.0f;
        if (!parse_float(argv[3], &target)) {
            command_tx("motor vel target must be a number\n");
            return;
        }
        motor->set_test_speed(target);
        command_tx("motor %s velocity target=%.2f\n", argv[1], target);
    } else if (strcmp(argv[2], "pid") == 0 && argc == 6) {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        if (!parse_float(argv[3], &kp) || !parse_float(argv[4], &ki) || !parse_float(argv[5], &kd)) {
            command_tx("motor pid values must be numbers\n");
            return;
        }
        motor->set_pid(kp, ki, kd);
        command_tx("motor %s pid=%.3f %.3f %.3f\n", argv[1], kp, ki, kd);
    } else {
        int pwm = 0;
        if (!parse_int(argv[2], &pwm) || pwm < -255 || pwm > 255) {
            command_tx("motor command must be enc, enc0, auto, vel, pid, or pwm -255..255\n");
            return;
        }

        if (!ensure_not_stopped("motor")) {
            return;
        }

        motor->set_manual_pwm(pwm);
        command_tx("motor %s pwm=%d\n", argv[1], pwm);
    }
}

void cmd_move(int argc, char** argv) {
    if (argc < 4 || argc > 5) {
        command_tx("usage: move <vx> <vy> <w> [duration_ms]\n");
        return;
    }

    if (!ensure_not_stopped("move")) {
        return;
    }

    float vx = 0.0f;
    float vy = 0.0f;
    float w = 0.0f;
    if (!parse_float(argv[1], &vx) || !parse_float(argv[2], &vy) || !parse_float(argv[3], &w)) {
        command_tx("move vx vy w must be numbers\n");
        return;
    }

    uint32_t duration_ms = 0;
    if (argc == 5 && !parse_uint32(argv[4], &duration_ms)) {
        command_tx("move duration_ms must be a positive integer\n");
        return;
    }

    motion_state = MotionState::IDLE;
    m4_chassis().enable();
    m4_chassis().clear_position_control();
    m4_chassis().set_target(vx, vy, w);
    command_tx("move target=%.2f/%.2f/%.2f duration=%lums\n", vx, vy, w, static_cast<unsigned long>(duration_ms));

    if (duration_ms > 0) {
        const uint32_t start_ms = millis();
        while (millis() - start_ms < duration_ms) {
            if (running_state == RunningState::STOPPED) {
                command_tx("move aborted by stop\n");
                return;
            }
            ThisThread::sleep_for(20ms);
        }
        motion_force_stop(false);
        command_tx("move done\n");
    }
}

void cmd_forward(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        command_tx("usage: forward <pwm> [duration_ms]\n");
        return;
    }

    if (!ensure_not_stopped("forward")) {
        return;
    }

    int pwm = 0;
    if (!parse_int(argv[1], &pwm) || pwm < -255 || pwm > 255) {
        command_tx("forward pwm must be -255..255\n");
        return;
    }

    uint32_t duration_ms = 0;
    if (argc == 3 && !parse_uint32(argv[2], &duration_ms)) {
        command_tx("forward duration_ms must be a positive integer\n");
        return;
    }

    motion_state = MotionState::IDLE;
    m4_chassis().enable();
    m4_motor_fl().set_manual_pwm(pwm);
    m4_motor_fr().set_manual_pwm(pwm);
    m4_motor_rl().set_manual_pwm(pwm);
    m4_motor_rr().set_manual_pwm(pwm);
    command_tx("forward pwm=%d duration=%lums\n", pwm, static_cast<unsigned long>(duration_ms));

    if (duration_ms > 0) {
        const uint32_t start_ms = millis();
        while (millis() - start_ms < duration_ms) {
            if (running_state == RunningState::STOPPED) {
                command_tx("forward aborted by stop\n");
                return;
            }
            ThisThread::sleep_for(20ms);
        }
        motion_force_stop(false);
        command_tx("forward done\n");
    }
}

void cmd_turn(int argc, char** argv) {
    if (argc < 2 || argc > 5) {
        command_tx("usage: turn <left|right|90|-90|180|deg> [max_w] [tolerance_deg] [timeout_ms] OR turn ir <l|r> [max_w] [timeout_ms]\n");
        return;
    }

    if (!ensure_not_stopped("turn")) {
        return;
    }

    if (strcmp(argv[1], "ir") == 0) {
        if (argc < 3 || argc > 5) {
            command_tx("usage: turn ir <l|r|left|right> [max_w] [timeout_ms]\n");
            return;
        }

        bool left = false;
        if (strcmp(argv[2], "l") == 0 || strcmp(argv[2], "left") == 0 || strcmp(argv[2], "1") == 0) {
            left = true;
        } else if (strcmp(argv[2], "r") == 0 || strcmp(argv[2], "right") == 0 || strcmp(argv[2], "-1") == 0 || strcmp(argv[2], "0") == 0) {
            left = false;
        } else {
            command_tx("turn ir side must be l/left/1 or r/right/-1/0\n");
            return;
        }

        float max_w = CONFIG::M4::TURN_IR_W;
        uint32_t timeout = CONFIG::M4::TURN_IR_TIMEOUT_MS;
        if (argc >= 4 && !parse_float(argv[3], &max_w)) {
            command_tx("turn ir max_w must be a number\n");
            return;
        }
        if (argc >= 5 && !parse_uint32(argv[4], &timeout)) {
            command_tx("turn ir timeout must be an integer\n");
            return;
        }

        const MotionResult result = run_turn_until_ir_line(left, max_w, timeout);
        command_tx("turn ir done: %s ir_pos=%d\n", motion_result_name(result), rpc_bridge_ir_pos());
        return;
    }

    float delta = 0.0f;
    if (strcmp(argv[1], "left") == 0) {
        delta = 90.0f;
    } else if (strcmp(argv[1], "right") == 0) {
        delta = -90.0f;
    } else if (!parse_float(argv[1], &delta)) {
        command_tx("turn angle must be left/right/ir or a number\n");
        return;
    }

    float max_w = CONFIG::M4::TURN_MAX_W;
    float tolerance = CONFIG::M4::TURN_TOLERANCE_DEG;
    uint32_t timeout = CONFIG::M4::TURN_TIMEOUT_MS;
    if (argc >= 3 && !parse_float(argv[2], &max_w)) {
        command_tx("turn max_w must be a number\n");
        return;
    }
    if (argc >= 4 && !parse_float(argv[3], &tolerance)) {
        command_tx("turn tolerance must be a number\n");
        return;
    }
    if (argc >= 5 && !parse_uint32(argv[4], &timeout)) {
        command_tx("turn timeout must be an integer\n");
        return;
    }

    const MotionResult result = run_turn_deg(delta, max_w, tolerance, timeout);
    command_tx("turn done: %s yaw=%.2f\n", motion_result_name(result), imu_yaw_deg());
}

void cmd_line(int argc, char** argv) {
    if (argc < 2) {
        command_tx("usage: line front [front_cm] [speed] | line motor <cm> [speed] [front_cm] | line rfid [uid|any] [speed] [front_cm] | line cross [speed] [front_cm] | line no_line [speed] [front_cm] | line corner <l|r|any> [speed] [front_cm]\n");
        return;
    }

    if (!ensure_not_stopped("line")) {
        return;
    }

    MotionResult result = MotionResult::SensorInvalid;
    if (strcmp(argv[1], "front") == 0) {
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        if (argc >= 3 && !parse_float(argv[2], &front)) {
            command_tx("line front_cm must be a number\n");
            return;
        }
        if (argc >= 4 && !parse_float(argv[3], &speed)) {
            command_tx("line speed must be a number\n");
            return;
        }
        result = run_line_follow_until_front_cm(front, speed);
    } else if (strcmp(argv[1], "motor") == 0) {
        if (argc < 3) {
            command_tx("usage: line motor <cm> [speed] [front_cm]\n");
            return;
        }
        float cm = 0.0f;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        if (!parse_float(argv[2], &cm) ||
            (argc >= 4 && !parse_float(argv[3], &speed)) ||
            (argc >= 5 && !parse_float(argv[4], &front))) {
            command_tx("line motor args must be numbers\n");
            return;
        }
        result = run_line_follow_until_motor_cm(cm, speed, front);
    } else if (strcmp(argv[1], "rfid") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        uint32_t uid = 0;
        bool use_uid = false;
        int arg = 2;
        if (argc > arg && strcmp(argv[arg], "any") != 0) {
            if (parse_uint32(argv[arg], &uid)) {
                use_uid = true;
                arg++;
            }
        } else if (argc > arg) {
            arg++;
        }
        if (argc > arg && !parse_float(argv[arg++], &speed)) {
            command_tx("line rfid speed must be a number\n");
            return;
        }
        if (argc > arg && !parse_float(argv[arg], &front)) {
            command_tx("line rfid front_cm must be a number\n");
            return;
        }
        result = use_uid ? run_line_follow_until_rfid_uid(uid, speed, front) : run_line_follow_until_rfid(speed, front);
    } else if (strcmp(argv[1], "cross") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        if ((argc >= 3 && !parse_float(argv[2], &speed)) ||
            (argc >= 4 && !parse_float(argv[3], &front))) {
            command_tx("line cross args must be numbers\n");
            return;
        }
        result = run_line_follow_until_cross(speed, front);
    } else if (strcmp(argv[1], "no_line") == 0 || strcmp(argv[1], "noline") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        if ((argc >= 3 && !parse_float(argv[2], &speed)) ||
            (argc >= 4 && !parse_float(argv[3], &front))) {
            command_tx("line no_line args must be numbers\n");
            return;
        }
        result = run_line_follow_until_no_line(speed, front);
    } else if (strcmp(argv[1], "corner") == 0 || strcmp(argv[1], "right_angle") == 0) {
        if (argc < 3) {
            command_tx("usage: line corner <l|r|left|right|any> [speed] [front_cm]\n");
            return;
        }

        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        int arg = 3;
        if (argc > arg && !parse_float(argv[arg++], &speed)) {
            command_tx("line corner speed must be a number\n");
            return;
        }
        if (argc > arg && !parse_float(argv[arg], &front)) {
            command_tx("line corner front_cm must be a number\n");
            return;
        }

        if (strcmp(argv[2], "any") == 0) {
            result = run_line_follow_until_any_corner(speed, front);
        } else {
            WallSide side = WallSide::Left;
            if (!parse_wall_side(argv[2], &side)) {
                command_tx("line corner side must be l/left, r/right, or any\n");
                return;
            }
            result = run_line_follow_until_corner(side, speed, front);
        }
    } else {
        command_tx("usage: line front|motor|rfid|cross|no_line|corner ...\n");
        return;
    }

    command_tx("line done: %s\n", motion_result_name(result));
}

void cmd_drive(int argc, char** argv) {
    if (argc < 2) {
        command_tx("usage: drive front [front_cm] [speed] | drive motor <cm> [speed] [front_cm] | drive rfid [uid|any] [speed] [front_cm]\n");
        return;
    }

    if (!ensure_not_stopped("drive")) {
        return;
    }

    MotionResult result = MotionResult::SensorInvalid;
    if (strcmp(argv[1], "front") == 0) {
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        if ((argc >= 3 && !parse_float(argv[2], &front)) ||
            (argc >= 4 && !parse_float(argv[3], &speed))) {
            command_tx("drive front args must be numbers\n");
            return;
        }
        result = run_drive_until_front_cm(front, speed);
    } else if (strcmp(argv[1], "motor") == 0) {
        if (argc < 3) {
            command_tx("usage: drive motor <cm> [speed] [front_cm]\n");
            return;
        }
        float cm = 0.0f;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        if (!parse_float(argv[2], &cm) ||
            (argc >= 4 && !parse_float(argv[3], &speed)) ||
            (argc >= 5 && !parse_float(argv[4], &front))) {
            command_tx("drive motor args must be numbers\n");
            return;
        }
        result = run_drive_until_motor_cm(cm, speed, front);
    } else if (strcmp(argv[1], "rfid") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        uint32_t uid = 0;
        bool use_uid = false;
        int arg = 2;
        if (argc > arg && strcmp(argv[arg], "any") != 0) {
            if (parse_uint32(argv[arg], &uid)) {
                use_uid = true;
                arg++;
            }
        } else if (argc > arg) {
            arg++;
        }
        if (argc > arg && !parse_float(argv[arg++], &speed)) {
            command_tx("drive rfid speed must be a number\n");
            return;
        }
        if (argc > arg && !parse_float(argv[arg], &front)) {
            command_tx("drive rfid front_cm must be a number\n");
            return;
        }
        result = use_uid ? run_drive_until_rfid_uid(uid, speed, front) : run_drive_until_rfid(speed, front);
    } else {
        command_tx("usage: drive front|motor|rfid ...\n");
        return;
    }

    command_tx("drive done: %s\n", motion_result_name(result));
}

void cmd_wall(int argc, char** argv) {
    if (argc < 4) {
        command_tx("usage: wall front <l|r> <wall_cm> [front_cm] [speed] | wall motor <l|r> <wall_cm> <distance_cm> [speed] [front_cm] | wall rfid <l|r> <wall_cm> [uid|any] [speed] [front_cm]\n");
        return;
    }

    if (!ensure_not_stopped("wall")) {
        return;
    }

    WallSide side = WallSide::Right;
    if (!parse_wall_side(argv[2], &side)) {
        command_tx("wall side must be l/left or r/right\n");
        return;
    }

    float wall_cm = 0.0f;
    if (!parse_float(argv[3], &wall_cm) || wall_cm <= 0.0f) {
        command_tx("wall_cm must be positive\n");
        return;
    }

    MotionResult result = MotionResult::SensorInvalid;
    if (strcmp(argv[1], "front") == 0) {
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        if ((argc >= 5 && !parse_float(argv[4], &front)) ||
            (argc >= 6 && !parse_float(argv[5], &speed))) {
            command_tx("wall front args must be numbers\n");
            return;
        }
        result = run_wall_follow_until_front_cm(side, wall_cm, front, speed);
    } else if (strcmp(argv[1], "motor") == 0) {
        if (argc < 5) {
            command_tx("usage: wall motor <l|r> <wall_cm> <distance_cm> [speed] [front_cm]\n");
            return;
        }
        float distance = 0.0f;
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        if (!parse_float(argv[4], &distance) ||
            (argc >= 6 && !parse_float(argv[5], &speed)) ||
            (argc >= 7 && !parse_float(argv[6], &front))) {
            command_tx("wall motor args must be numbers\n");
            return;
        }
        result = run_wall_follow_until_motor_cm(side, wall_cm, distance, speed, front);
    } else if (strcmp(argv[1], "rfid") == 0) {
        float speed = MOTION_DEFAULT_SPEED_CM_S;
        float front = MOTION_DEFAULT_FRONT_STOP_CM;
        uint32_t uid = 0;
        bool use_uid = false;
        int arg = 4;
        if (argc > arg && strcmp(argv[arg], "any") != 0) {
            if (parse_uint32(argv[arg], &uid)) {
                use_uid = true;
                arg++;
            }
        } else if (argc > arg) {
            arg++;
        }
        if (argc > arg && !parse_float(argv[arg++], &speed)) {
            command_tx("wall rfid speed must be a number\n");
            return;
        }
        if (argc > arg && !parse_float(argv[arg], &front)) {
            command_tx("wall rfid front_cm must be a number\n");
            return;
        }
        result = use_uid ? run_wall_follow_until_rfid_uid(side, wall_cm, uid, speed, front) :
                           run_wall_follow_until_rfid(side, wall_cm, speed, front);
    } else {
        command_tx("usage: wall front|motor|rfid ...\n");
        return;
    }

    command_tx("wall done: %s\n", motion_result_name(result));
}

void cmd_base_a(int argc, char** argv) {
    if (!ensure_not_stopped("base_a")) {
        return;
    }

    const uint32_t uid_a = configured_or_arg_uid(CONFIG::M4::RFID_A_UID, argc, argv, 1);
    const float speed = CONFIG::M4::BASE_LINE_SPEED_CM_S;
    const float front = CONFIG::M4::BASE_FRONT_STOP_CM;

    command_tx("base_a: line to cross -> turn left -> line to RFID A uid=%lu\n", static_cast<unsigned long>(uid_a));
    if (!motion_ok("base_a line cross", run_line_follow_until_cross(speed, front), MotionResult::CrossLineDetected)) return;
    if (!motion_ok("base_a turn left", run_turn_deg(90.0f), MotionResult::AngleReached)) return;

    const MotionResult rfid_result = uid_a == 0 ?
        run_line_follow_until_rfid(speed, front) :
        run_line_follow_until_rfid_uid(uid_a, speed, front);
    motion_ok("base_a line rfid A", rfid_result, MotionResult::RfidDetected);
}

void cmd_base_b(int argc, char** argv) {
    if (!ensure_not_stopped("base_b")) {
        return;
    }

    const uint32_t uid_b = configured_or_arg_uid(CONFIG::M4::RFID_B_UID, argc, argv, 1);
    const float speed = CONFIG::M4::BASE_LINE_SPEED_CM_S;
    const float front = CONFIG::M4::BASE_FRONT_STOP_CM;

    command_tx("base_b: line to cross -> turn right -> line to RFID B uid=%lu\n", static_cast<unsigned long>(uid_b));
    if (!motion_ok("base_b line cross", run_line_follow_until_cross(speed, front), MotionResult::CrossLineDetected)) return;
    if (!motion_ok("base_b turn right", run_turn_deg(-90.0f), MotionResult::AngleReached)) return;

    const MotionResult b_result = uid_b == 0 ?
        run_line_follow_until_rfid(speed, front) :
        run_line_follow_until_rfid_uid(uid_b, speed, front);
    if (!motion_ok("base_b line rfid B", b_result, MotionResult::RfidDetected)) return;

    char request[128];
    snprintf(request,
             sizeof(request),
             "type=door_request door=B uid=%lu",
             static_cast<unsigned long>(uid_b));
    rpc_bridge_clear_door_response();
    if (!rpc_bridge_send_mqtt_to_server(request)) {
        command_tx("base_b door request send failed\n");
        return;
    }
    if (!wait_for_door_response(CONFIG::M4::BASE_DOOR_RESPONSE_TIMEOUT_MS)) {
        return;
    }

    if (!motion_ok("base_b line no-line", run_line_follow_until_no_line(speed, front), MotionResult::NoLineDetected)) return;
    if (!motion_ok("base_b wall to front", run_wall_follow_until_front_cm(WallSide::Right,
                                                                          CONFIG::M4::BASE_WALL_DIST_CM,
                                                                          CONFIG::M4::BASE_DOOR_FRONT_CM,
                                                                          speed),
                   MotionResult::FrontObstacle)) return;

    motion_force_stop(false);
    command_tx("base_b waiting for door open\n");
    if (!wait_for_front_clear(CONFIG::M4::BASE_DOOR_FRONT_CM, CONFIG::M4::BASE_DOOR_OPEN_TIMEOUT_MS)) {
        return;
    }

    const MotionResult final_result = run_line_follow_until_rfid(speed, front);
    motion_ok("base_b line to arena rfid", final_result, MotionResult::RfidDetected);
}

} // namespace

void m4_commands_begin() {
    if (commands_registered) {
        return;
    }

    bash.reg_command("help", cmd_help, "list commands or show help: help [cmd]");
    bash.reg_command("start", cmd_start, "leave STOPPED state and enable chassis");
    bash.reg_command("stop", cmd_stop, "enter STOPPED state and force stop chassis");
    bash.reg_command("state", cmd_state, "print running/motion/chassis state");
    bash.reg_command("print", cmd_print, "print ir|dist|rfid|sunlight|imu|state|motor|stat");
    bash.reg_command("imu", cmd_imu, "IMU status/calibration: imu [status|gyro|mag|all]");
    bash.reg_command("motor", cmd_motor, "motor debug/control");
    bash.reg_command("move", cmd_move, "set chassis target: move <vx> <vy> <w> [duration_ms]");
    bash.reg_command("forward", cmd_forward, "manual pwm all motors: forward <pwm> [duration_ms]");
    bash.reg_command("turn", cmd_turn, "turn by yaw or IR: turn <deg|left|right> ... | turn ir <l|r> [max_w] [timeout_ms]");
    bash.reg_command("line", cmd_line, "blocking line follow");
    bash.reg_command("drive", cmd_drive, "blocking straight drive");
    bash.reg_command("wall", cmd_wall, "blocking wall follow");
    bash.reg_command("base_a", cmd_base_a, "base workflow: line cross, turn left, stop at RFID A");
    bash.reg_command("base_b", cmd_base_b, "base workflow: line cross, turn right, RFID B, door request, tunnel exit");

    commands_registered = true;
    loggf("[m4-commands] registered %u commands\n", static_cast<unsigned>(bash.commands.size()));
}

void m4_command_worker_begin() {
    if (command_worker_started) {
        return;
    }

    const osStatus status = task_command_worker.start([] {
        loggf("[m4-command-worker] task ready\n");
        while (true) {
            auto* message = command_mail.try_get_for(100ms);
            if (message == nullptr) {
                continue;
            }

            loggf("[m4-command-worker] exec: %s\n", message->data());
            bash.execute(message->data());
            command_mail.free(message);
        }
    });

    command_worker_started = status == osOK;
    if (!command_worker_started) {
        loggf("[m4-command-worker] start failed status=%d\n", status);
    }
}

bool m4_command_enqueue(const char* source, const char* command) {
    if (command == nullptr || command[0] == '\0') {
        return false;
    }

    auto* message = command_mail.try_alloc();
    if (message == nullptr) {
        loggf("[m4-command-worker] queue full, drop from %s: %s\n", source ? source : "unknown", command);
        return false;
    }

    strlcpy(message->data(), command, message->size());
    command_mail.put(message);
    loggf("[m4-command-worker] queued from %s: %s\n", source ? source : "unknown", message->data());
    return true;
}
