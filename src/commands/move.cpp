#include "bash.hpp"

static bool parse_float_arg(const char* text, float* value) {
    char* end = nullptr;
    float val = strtof(text, &end);

    if (end == text || *end != '\0') {
        return false;
    }

    *value = val;
    return true;
}

static bool parse_uint32_arg(const char* text, uint32_t* value) {
    char* end = nullptr;
    unsigned long val = strtoul(text, &end, 10);

    if (end == text || *end != '\0') {
        return false;
    }

    *value = static_cast<uint32_t>(val);
    return true;
}

static float wrap_deg(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static bool parse_turn_delta(const char* text, float* delta_deg) {
    if (strcmp(text, "left") == 0 || strcmp(text, "l") == 0 || strcmp(text, "90") == 0) {
        *delta_deg = 90.0f;
        return true;
    }
    if (strcmp(text, "right") == 0 || strcmp(text, "r") == 0 || strcmp(text, "-90") == 0) {
        *delta_deg = -90.0f;
        return true;
    }
    if (strcmp(text, "180") == 0 || strcmp(text, "back") == 0 || strcmp(text, "u") == 0) {
        *delta_deg = 180.0f;
        return true;
    }
    if (strcmp(text, "-180") == 0 || strcmp(text, "backr") == 0) {
        *delta_deg = -180.0f;
        return true;
    }

    return parse_float_arg(text, delta_deg);
}

static bool parse_pwm_level(const char* text, int* pwm) {
    if (strcmp(text, "slow") == 0) {
        *pwm = 80;
        return true;
    }
    if (strcmp(text, "mid") == 0 || strcmp(text, "normal") == 0) {
        *pwm = 120;
        return true;
    }
    if (strcmp(text, "fast") == 0) {
        *pwm = 180;
        return true;
    }

    float value = 0.0f;
    if (!parse_float_arg(text, &value)) {
        return false;
    }

    *pwm = constrain(static_cast<int>(value), -255, 255);
    return true;
}

static void prepare_motion_command() {
    motion_state = MotionState::IDLE;
    chassis.enable();
}

static void stop_motors() {
    chassis.set_target(0.0f, 0.0f, 0.0f);
    mfl.clear_manual_pwm();
    mfr.clear_manual_pwm();
    mrl.clear_manual_pwm();
    mrr.clear_manual_pwm();
}

static bool stop_requested() {
    return button_state == ButtonState::STOPPED;
}

static void set_forward_pwm(int pwm) {
    mfl.set_manual_pwm(pwm);
    mfr.set_manual_pwm(pwm);
    mrl.set_manual_pwm(pwm);
    mrr.set_manual_pwm(pwm);
}

static void set_turn_pwm(int pwm) {
    mfl.set_manual_pwm(-pwm);
    mfr.set_manual_pwm(pwm);
    mrl.set_manual_pwm(-pwm);
    mrr.set_manual_pwm(pwm);
}

static void clear_manual_pwm_all() {
    mfl.clear_manual_pwm();
    mfr.clear_manual_pwm();
    mrl.clear_manual_pwm();
    mrr.clear_manual_pwm();
}

static void move_cmd(int argc, char** argv) {
    if (argc < 4 || argc > 5) {
        command_tx("usage: move <vx> <vy> <w> [duration_ms]\n");
        return;
    }

    float vx = 0.0f;
    float vy = 0.0f;
    float w  = 0.0f;
    if (!parse_float_arg(argv[1], &vx) || !parse_float_arg(argv[2], &vy) || !parse_float_arg(argv[3], &w)) {
        command_tx("move vx vy w must be numbers.\n");
        return;
    }

    uint32_t duration_ms = 1000;
    if (argc == 5 && !parse_uint32_arg(argv[4], &duration_ms)) {
        command_tx("move duration_ms must be a positive integer.\n");
        return;
    }

    prepare_motion_command();
    clear_manual_pwm_all();
    if (stop_requested()) {
        stop_motors();
        command_tx("move aborted: robot is stopped.\n");
        return;
    }

    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        if (stop_requested()) {
            stop_motors();
            command_tx("move aborted by stop button.\n");
            return;
        }

        chassis.set_target(vx, vy, w);
        ThisThread::sleep_for(20ms);
    }

    stop_motors();
    command_tx("move done: vx=%.2f vy=%.2f w=%.2f duration=%lums\n",
    vx, vy, w, static_cast<unsigned long>(duration_ms));
}

static void turn_cmd(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        command_tx("usage: turn <left|right|90|-90|180|-180> [pwm]\n");
        return;
    }

    if (!imu_yaw_ready) {
        command_tx("turn failed: IMU yaw is not ready.\n");
        return;
    }

    float delta_deg = 0.0f;
    if (!parse_turn_delta(argv[1], &delta_deg)) {
        command_tx("turn angle must be left/right/90/-90/180/-180 or a number.\n");
        return;
    }

    int turn_pwm = 100;
    if (argc == 3 && (!parse_pwm_level(argv[2], &turn_pwm) || turn_pwm == 0)) {
        command_tx("turn pwm must be slow/mid/fast or a non-zero number from -255 to 255.\n");
        return;
    }
    turn_pwm = abs(turn_pwm);

    prepare_motion_command();
    if (stop_requested()) {
        stop_motors();
        command_tx("turn aborted: robot is stopped.\n");
        return;
    }

    constexpr float done_err_deg = 2.0f;
    constexpr uint32_t timeout_ms = 8000;

    float target_yaw = wrap_deg(imu_yaw_deg + delta_deg);
    unsigned long start = millis();

    while (millis() - start < timeout_ms) {
        if (stop_requested()) {
            stop_motors();
            command_tx("turn aborted by stop button.\n");
            return;
        }

        float yaw = imu_yaw_deg;
        float err = wrap_deg(target_yaw - yaw);

        if (fabsf(err) <= done_err_deg) {
            break;
        }

        set_turn_pwm(err > 0.0f ? turn_pwm : -turn_pwm);
        ThisThread::sleep_for(20ms);
    }

    stop_motors();
    command_tx("turn done: pwm=%d target=%.2f yaw=%.2f err=%.2f\n",
    turn_pwm, target_yaw, imu_yaw_deg, wrap_deg(target_yaw - imu_yaw_deg));
}

static void forward_cmd(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        command_tx("usage: forward <slow|mid|fast|speed> [duration_ms]\n");
        return;
    }

    int pwm = 0;
    if (!parse_pwm_level(argv[1], &pwm)) {
        command_tx("forward pwm must be slow/mid/fast or a number from -255 to 255.\n");
        return;
    }

    uint32_t duration_ms = 1000;
    if (argc == 3 && !parse_uint32_arg(argv[2], &duration_ms)) {
        command_tx("forward duration_ms must be a positive integer.\n");
        return;
    }

    prepare_motion_command();
    if (stop_requested()) {
        stop_motors();
        command_tx("forward aborted: robot is stopped.\n");
        return;
    }

    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        if (stop_requested()) {
            stop_motors();
            command_tx("forward aborted by stop button.\n");
            return;
        }

        set_forward_pwm(pwm);
        ThisThread::sleep_for(20ms);
    }

    stop_motors();
    command_tx("forward done: pwm=%d duration=%lums\n", pwm, static_cast<unsigned long>(duration_ms));
}

BASH_COMMAND("move", move_cmd, "closed-loop chassis move: move <vx> <vy> <w> [duration_ms]")
BASH_COMMAND("turn", turn_cmd, "open-loop turn by IMU yaw: turn <left|right|90|-90|180|-180> [pwm]")
BASH_COMMAND("forward", forward_cmd, "open-loop drive forward: forward <slow|mid|fast|pwm> [duration_ms]")
