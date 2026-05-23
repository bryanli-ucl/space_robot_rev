#include "bash.hpp"

static Motor* get_motor(const char* name) {
    if (strcmp(name, "fl") == 0) return &mfl;
    if (strcmp(name, "fr") == 0) return &mfr;
    if (strcmp(name, "rl") == 0) return &mrl;
    if (strcmp(name, "rr") == 0) return &mrr;
    return nullptr;
}

static bool parse_pwm(const char* text, int* pwm) {
    char* end = nullptr;
    long val  = strtol(text, &end, 10);

    if (end == text || *end != '\0') {
        return false;
    }

    if (val < -255 || val > 255) {
        return false;
    }

    *pwm = static_cast<int>(val);
    return true;
}

static bool parse_float(const char* text, float* value) {
    char* end = nullptr;
    float val = strtof(text, &end);

    if (end == text || *end != '\0') {
        return false;
    }

    *value = val;
    return true;
}

static void print_motor_state(const char* name, Motor& motor) {
    command_tx("motor %s: enc=%ld d=%ld raw=%.2f speed=%.2f target=%.2f err=%.2f der=%.2f int=%.2f out=%.2f pwm=%d pid=%.3f %.3f %.3f mode=%s%s\n",
    name,
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
    motor.is_manual_pwm() ? "pwm" : "pid",
    motor.is_speed_override() ? "+test" : "");
}

static void motor(int argc, char** argv) {

    if (argc == 2 && strcmp(argv[1], "enc") == 0) {
        command_tx("motor enc: fl=%ld fr=%ld rl=%ld rr=%ld\n",
        static_cast<long>(mfl.count()),
        static_cast<long>(mfr.count()),
        static_cast<long>(mrl.count()),
        static_cast<long>(mrr.count()));
        return;
    }

    if (argc == 2 && strcmp(argv[1], "enc0") == 0) {
        mfl.reset_count();
        mfr.reset_count();
        mrl.reset_count();
        mrr.reset_count();
        command_tx("motor enc counts reset.\n");
        return;
    }

    if (argc == 2 && strcmp(argv[1], "vel") == 0) {
        print_motor_state("fl", mfl);
        print_motor_state("fr", mfr);
        print_motor_state("rl", mrl);
        print_motor_state("rr", mrr);
        return;
    }

    Motor* selected = get_motor(argv[1]);
    if (selected == nullptr) {
        command_tx("unknown motor '%s'. use fl/fr/rl/rr.\n", argv[1]);
        return;
    }

    if (argc == 4 && strcmp(argv[2], "vel") == 0) {
        float target = 0.0f;
        if (!parse_float(argv[3], &target)) {
            command_tx("motor velocity target must be a number.\n");
            return;
        }

        selected->set_test_speed(target);
        command_tx("motor %s velocity target set to %.2f. run 'start' to enable loop.\n", argv[1], target);
        return;
    }

    if (argc == 6 && strcmp(argv[2], "pid") == 0) {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        if (!parse_float(argv[3], &kp) || !parse_float(argv[4], &ki) || !parse_float(argv[5], &kd)) {
            command_tx("motor pid values must be numbers.\n");
            return;
        }

        selected->set_pid(kp, ki, kd);
        command_tx("motor %s pid set to %.3f %.3f %.3f.\n", argv[1], kp, ki, kd);
        return;
    }

    if (argc != 3) {
        command_tx("usage: motor enc|enc0|vel OR motor <fl|fr|rl|rr> <f|r|-255..255|auto|enc|enc0|vel> OR motor <id> vel <target> OR motor <id> pid <kp> <ki> <kd>\n");
        return;
    }

    int pwm = 0;
    if (strcmp(argv[2], "f") == 0) {
        pwm = 255;
    } else if (strcmp(argv[2], "r") == 0) {
        pwm = -255;
    } else if (strcmp(argv[2], "enc") == 0) {
        command_tx("motor %s enc count: %ld\n", argv[1], static_cast<long>(selected->count()));
        return;
    } else if (strcmp(argv[2], "enc0") == 0) {
        selected->reset_count();
        command_tx("motor %s enc count reset.\n", argv[1]);
        return;
    } else if (strcmp(argv[2], "auto") == 0) {
        selected->clear_manual_pwm();
        command_tx("motor %s returned to chassis control.\n", argv[1]);
        return;
    } else if (strcmp(argv[2], "vel") == 0) {
        print_motor_state(argv[1], *selected);
        return;
    } else if (!parse_pwm(argv[2], &pwm)) {
        command_tx("motor command must be f, r, auto, enc, enc0, vel, or an integer from -255 to 255.\n");
        return;
    }

    selected->set_manual_pwm(pwm);
    command_tx("motor %s pwm set to %d.\n", argv[1], pwm);
}

BASH_COMMAND("motor", motor, "test motor: motor enc|enc0|vel OR motor <fl|fr|rl|rr> <f|r|-255..255|auto|enc|enc0|vel> OR motor <id> vel <target> OR motor <id> pid <kp> <ki> <kd>")
