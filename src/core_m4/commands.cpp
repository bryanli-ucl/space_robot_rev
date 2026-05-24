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

#include <stdlib.h>
#include <string.h>

using namespace std::chrono_literals;
using namespace ::rtos;

namespace {

bool commands_registered = false;

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
        command_tx("usage: print ir|dist|sunlight|imu|state|motor|stat\n");
        return;
    }

    if (strcmp(argv[1], "ir") == 0) {
        command_tx("IR pos=%d raw=%d %d %d %d %d %d %d %d %d\n",
        rpc_bridge_ir_pos(),
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
        command_tx("usage: print ir|dist|sunlight|imu|state|motor|stat\n");
    }
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
        command_tx("usage: turn <left|right|90|-90|180|deg> [max_w] [tolerance_deg] [timeout_ms]\n");
        return;
    }

    if (!ensure_not_stopped("turn")) {
        return;
    }

    float delta = 0.0f;
    if (strcmp(argv[1], "left") == 0) {
        delta = 90.0f;
    } else if (strcmp(argv[1], "right") == 0) {
        delta = -90.0f;
    } else if (!parse_float(argv[1], &delta)) {
        command_tx("turn angle must be left/right or a number\n");
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
        command_tx("usage: line front [front_cm] [speed] | line motor <cm> [speed] [front_cm] | line rfid [uid|any] [speed] [front_cm] | line cross [speed] [front_cm]\n");
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
    } else {
        command_tx("usage: line front|motor|rfid|cross ...\n");
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

} // namespace

void m4_commands_begin() {
    if (commands_registered) {
        return;
    }

    bash.reg_command("help", cmd_help, "list commands or show help: help [cmd]");
    bash.reg_command("start", cmd_start, "leave STOPPED state and enable chassis");
    bash.reg_command("stop", cmd_stop, "enter STOPPED state and force stop chassis");
    bash.reg_command("state", cmd_state, "print running/motion/chassis state");
    bash.reg_command("print", cmd_print, "print ir|dist|sunlight|imu|state|motor|stat");
    bash.reg_command("motor", cmd_motor, "motor debug/control");
    bash.reg_command("move", cmd_move, "set chassis target: move <vx> <vy> <w> [duration_ms]");
    bash.reg_command("forward", cmd_forward, "manual pwm all motors: forward <pwm> [duration_ms]");
    bash.reg_command("turn", cmd_turn, "turn by yaw: turn <deg|left|right> [max_w] [tol] [timeout_ms]");
    bash.reg_command("line", cmd_line, "blocking line follow");
    bash.reg_command("drive", cmd_drive, "blocking straight drive");

    commands_registered = true;
    loggf("[m4-commands] registered %u commands\n", static_cast<unsigned>(bash.commands.size()));
}
