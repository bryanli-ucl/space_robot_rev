#include "bash.hpp"
#include "motion_control.hpp"

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
    char* end         = nullptr;
    unsigned long val = strtoul(text, &end, 10);

    if (end == text || *end != '\0') {
        return false;
    }

    *value = static_cast<uint32_t>(val);
    return true;
}

static bool parse_int32_arg(const char* text, int32_t* value) {
    char* end = nullptr;
    long val  = strtol(text, &end, 10);

    if (end == text || *end != '\0') {
        return false;
    }

    *value = static_cast<int32_t>(val);
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
    chassis.clear_position_control();
    chassis.set_target(0.0f, 0.0f, 0.0f);
    mfl.clear_manual_pwm();
    mfr.clear_manual_pwm();
    mrl.clear_manual_pwm();
    mrr.clear_manual_pwm();
}

static bool stop_requested() {
    return running_state == RunningState::STOPPED;
}

static void set_forward_pwm(int pwm) {
    mfl.set_manual_pwm(pwm);
    mfr.set_manual_pwm(pwm);
    mrl.set_manual_pwm(pwm);
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
    if (argc < 2 || argc > 5) {
        command_tx("usage: turn <left|right|90|-90|180|-180|deg> [max_w] [tolerance_deg] [timeout_ms]\n");
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

    float max_w = CONFIG::TURN_MAX_W;
    if (argc >= 3 && (!parse_float_arg(argv[2], &max_w) || fabsf(max_w) < 0.01f)) {
        command_tx("turn max_w must be a positive number.\n");
        return;
    }

    float tolerance_deg = CONFIG::TURN_TOLERANCE_DEG;
    if (argc >= 4 && (!parse_float_arg(argv[3], &tolerance_deg) || tolerance_deg <= 0.0f)) {
        command_tx("turn tolerance_deg must be a positive number.\n");
        return;
    }

    uint32_t timeout_ms = CONFIG::TURN_TIMEOUT_MS;
    if (argc >= 5 && !parse_uint32_arg(argv[4], &timeout_ms)) {
        command_tx("turn timeout_ms must be a positive integer.\n");
        return;
    }

    float target_yaw    = wrap_deg(imu_yaw_deg + delta_deg);
    MotionResult result = run_turn_deg(delta_deg, max_w, tolerance_deg, timeout_ms);
    command_tx("turn done: %s delta=%.2f yaw=%.2f err=%.2f\n",
    motion_result_name(result),
    delta_deg,
    imu_yaw_deg,
    wrap_deg(target_yaw - imu_yaw_deg));
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

static void movepos_cmd(int argc, char** argv) {
    if (argc < 2 || argc > 4) {
        command_tx("usage: movepos <encoder_counts> [max_speed_counts_s] [timeout_ms]\n");
        return;
    }

    int32_t counts = 0;
    if (!parse_int32_arg(argv[1], &counts) || counts == 0) {
        command_tx("movepos encoder_counts must be a non-zero integer.\n");
        return;
    }

    float max_speed = 160.0f;
    if (argc >= 3 && (!parse_float_arg(argv[2], &max_speed) || fabsf(max_speed) < 1.0f)) {
        command_tx("movepos max_speed_counts_s must be a positive number.\n");
        return;
    }
    max_speed = fabsf(max_speed);

    uint32_t timeout_ms = static_cast<uint32_t>(fabsf(counts) / max_speed * 1000.0f) + 3000;
    if (timeout_ms < 3000) {
        timeout_ms = 3000;
    }
    if (argc == 4 && !parse_uint32_arg(argv[3], &timeout_ms)) {
        command_tx("movepos timeout_ms must be a positive integer.\n");
        return;
    }

    prepare_motion_command();
    clear_manual_pwm_all();
    chassis.clear_position_control();
    if (stop_requested()) {
        stop_motors();
        command_tx("movepos aborted: robot is stopped.\n");
        return;
    }

    int32_t fl_start = mfl.count();
    int32_t fr_start = mfr.count();
    int32_t rl_start = mrl.count();
    int32_t rr_start = mrr.count();

    chassis.move_counts(counts, max_speed);

    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        if (stop_requested()) {
            stop_motors();
            command_tx("movepos aborted by stop button.\n");
            return;
        }

        if (chassis.position_reached()) {
            break;
        }

        ThisThread::sleep_for(20ms);
    }

    bool reached = chassis.position_reached();
    stop_motors();

    command_tx("movepos %s: target=%ld max_speed=%.2f timeout=%lums fl=%ld fr=%ld rl=%ld rr=%ld\n",
    reached ? "done" : "timeout",
    static_cast<long>(counts),
    max_speed,
    static_cast<unsigned long>(timeout_ms),
    static_cast<long>(mfl.count() - fl_start),
    static_cast<long>(mfr.count() - fr_start),
    static_cast<long>(mrl.count() - rl_start),
    static_cast<long>(mrr.count() - rr_start));
}

BASH_COMMAND("move", move_cmd, "closed-loop chassis move: move <vx> <vy> <w> [duration_ms]")
BASH_COMMAND("turn", turn_cmd, "PID turn by IMU yaw: turn <left|right|90|-90|180|-180|deg> [max_w] [tolerance_deg] [timeout_ms]")
BASH_COMMAND("forward", forward_cmd, "open-loop drive forward: forward <slow|mid|fast|pwm> [duration_ms]")
BASH_COMMAND("movepos", movepos_cmd, "closed-loop encoder move: movepos <encoder_counts> [max_speed_counts_s] [timeout_ms]")
