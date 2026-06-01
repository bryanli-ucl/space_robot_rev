#include "logger.hpp"
#include "motor.hpp"
#include "shell.hpp"

#include <mbed.h>
#include <stdlib.h>
#include <string.h>

using namespace ::rtos;
using namespace ::std::chrono_literals;

static Motor* select_motor(const char* id) {
    if (strcmp(id, "fl") == 0) return &motor_fl();
    if (strcmp(id, "fr") == 0) return &motor_fr();
    if (strcmp(id, "rl") == 0) return &motor_rl();
    if (strcmp(id, "rr") == 0) return &motor_rr();
    return nullptr;
}

static bool parse_int(const char* text, int* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end        = nullptr;
    const long value = strtol(text, &end, 0);
    if (end == text || *end != '\0') return false;

    *out = static_cast<int>(value);
    return true;
}

static bool parse_float(const char* text, float* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end         = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0') return false;

    *out = value;
    return true;
}

static void print_motor(const char* id, Motor& motor) {
    loggf("motor %s enc=%ld pwm=%d target=%.1f speed=%.1f raw=%.1f out=%.1f int=%.1f pid=%.3f/%.3f/%.3f\n",
    id,
    static_cast<long>(motor.count()),
    motor.applied_pwm(),
    motor.target_speed(),
    motor.current_speed(),
    motor.raw_speed(),
    motor.output(),
    motor.integral_value(),
    motor.kp(),
    motor.ki(),
    motor.kd());
}

static void print_all_motors() {
    print_motor("fl", motor_fl());
    print_motor("fr", motor_fr());
    print_motor("rl", motor_rl());
    print_motor("rr", motor_rr());
}

static void set_all_pwm(int pwm) {
    motor_fl().set_pwm(pwm);
    motor_fr().set_pwm(pwm);
    motor_rl().set_pwm(pwm);
    motor_rr().set_pwm(pwm);
}

static void set_all_pid(float kp, float ki, float kd) {
    motor_fl().set_pid(kp, ki, kd);
    motor_fr().set_pid(kp, ki, kd);
    motor_rl().set_pid(kp, ki, kd);
    motor_rr().set_pid(kp, ki, kd);
}

static void set_all_speed(float speed) {
    motor_fl().set_speed(speed);
    motor_fr().set_speed(speed);
    motor_rl().set_speed(speed);
    motor_rr().set_speed(speed);
}

static void print_pidtest_sample(uint32_t elapsed_ms, float target) {
    loggf("pidtest t=%lu target=%.1f fl=%.1f/%d/%.0f/%.0f fr=%.1f/%d/%.0f/%.0f rl=%.1f/%d/%.0f/%.0f rr=%.1f/%d/%.0f/%.0f\n",
    static_cast<unsigned long>(elapsed_ms),
    target,
    motor_fl().current_speed(), motor_fl().applied_pwm(), motor_fl().output(), motor_fl().integral_value(),
    motor_fr().current_speed(), motor_fr().applied_pwm(), motor_fr().output(), motor_fr().integral_value(),
    motor_rl().current_speed(), motor_rl().applied_pwm(), motor_rl().output(), motor_rl().integral_value(),
    motor_rr().current_speed(), motor_rr().applied_pwm(), motor_rr().output(), motor_rr().integral_value());
}

static void run_pidtest(float target) {
    set_all_speed(target);
    loggf("pidtest target=%.1f duration=5000ms interval=100ms\n", target);

    const uint32_t start_ms = millis();
    uint32_t next_ms = start_ms;
    while (millis() - start_ms <= 5000) {
        const uint32_t now_ms = millis();
        if (now_ms >= next_ms) {
            print_pidtest_sample(now_ms - start_ms, target);
            next_ms += 100;
        }
        ThisThread::sleep_for(10ms);
    }

    motors_stop_all();
    loggf("pidtest done\n");
}

static void motor_cmd(int argc, char** argv) {
    if (argc < 2) {
        loggf("usage: motor enc|enc0|pid P I D|pidtest N|all ... OR motor <fl|fr|rl|rr> ...\n");
        return;
    }

    if (strcmp(argv[1], "enc") == 0) {
        print_all_motors();
        return;
    }

    if (strcmp(argv[1], "enc0") == 0) {
        motors_reset_all_counts();
        loggf("motor all enc reset\n");
        return;
    }

    if (strcmp(argv[1], "pid") == 0) {
        if (argc != 5) {
            loggf("usage: motor pid <kp> <ki> <kd>\n");
            return;
        }

        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        if (!parse_float(argv[2], &kp) || !parse_float(argv[3], &ki) || !parse_float(argv[4], &kd)) {
            loggf("motor pid values must be numbers\n");
            return;
        }

        set_all_pid(kp, ki, kd);
        loggf("motor all pid=%.3f/%.3f/%.3f\n", kp, ki, kd);
        return;
    }

    if (strcmp(argv[1], "pidtest") == 0) {
        if (argc != 3) {
            loggf("usage: motor pidtest <target_counts_s>\n");
            return;
        }

        float target = 0.0f;
        if (!parse_float(argv[2], &target)) {
            loggf("motor pidtest target must be a number\n");
            return;
        }

        run_pidtest(target);
        return;
    }

    if (strcmp(argv[1], "all") == 0) {
        if (argc < 3) {
            loggf("usage: motor all <pwm|vel N|stop|enc|enc0>\n");
            return;
        }

        if (strcmp(argv[2], "enc") == 0) {
            print_all_motors();
            return;
        }

        if (strcmp(argv[2], "enc0") == 0) {
            motors_reset_all_counts();
            loggf("motor all enc reset\n");
            return;
        }

        if (strcmp(argv[2], "stop") == 0) {
            motors_stop_all();
            loggf("motor all stop\n");
            return;
        }

        if (strcmp(argv[2], "vel") == 0) {
            if (argc != 4) {
                loggf("usage: motor all vel <target_counts_s>\n");
                return;
            }

            float speed = 0.0f;
            if (!parse_float(argv[3], &speed)) {
                loggf("motor all vel target must be a number\n");
                return;
            }

            motor_fl().set_speed(speed);
            motor_fr().set_speed(speed);
            motor_rl().set_speed(speed);
            motor_rr().set_speed(speed);
            loggf("motor all vel=%.1f\n", speed);
            return;
        }

        int pwm = 0;
        if (!parse_int(argv[2], &pwm) || pwm < -MOTOR_PWM_MAX || pwm > MOTOR_PWM_MAX) {
            loggf("motor all pwm must be -%d..%d\n", MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            return;
        }

        set_all_pwm(pwm);
        loggf("motor all pwm=%d\n", pwm);
        return;
    }

    Motor* motor = select_motor(argv[1]);
    if (motor == nullptr) {
        loggf("unknown motor '%s'. use fl/fr/rl/rr/all.\n", argv[1]);
        return;
    }

    if (argc < 3) {
        loggf("usage: motor <fl|fr|rl|rr> <pwm|vel N|pid P I D|stop|enc|enc0>\n");
        return;
    }

    if (strcmp(argv[2], "enc") == 0) {
        print_motor(argv[1], *motor);
        return;
    }

    if (strcmp(argv[2], "enc0") == 0) {
        motor->reset_count();
        loggf("motor %s enc reset\n", argv[1]);
        return;
    }

    if (strcmp(argv[2], "stop") == 0) {
        motor->stop();
        loggf("motor %s stop\n", argv[1]);
        return;
    }

    if (strcmp(argv[2], "vel") == 0) {
        if (argc != 4) {
            loggf("usage: motor %s vel <target_counts_s>\n", argv[1]);
            return;
        }

        float speed = 0.0f;
        if (!parse_float(argv[3], &speed)) {
            loggf("motor vel target must be a number\n");
            return;
        }

        motor->set_speed(speed);
        loggf("motor %s vel=%.1f\n", argv[1], speed);
        return;
    }

    if (strcmp(argv[2], "pid") == 0) {
        if (argc != 6) {
            loggf("usage: motor %s pid <kp> <ki> <kd>\n", argv[1]);
            return;
        }

        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        if (!parse_float(argv[3], &kp) || !parse_float(argv[4], &ki) || !parse_float(argv[5], &kd)) {
            loggf("motor pid values must be numbers\n");
            return;
        }

        motor->set_pid(kp, ki, kd);
        loggf("motor %s pid=%.3f/%.3f/%.3f\n", argv[1], kp, ki, kd);
        return;
    }

    int pwm = 0;
    if (!parse_int(argv[2], &pwm) || pwm < -MOTOR_PWM_MAX || pwm > MOTOR_PWM_MAX) {
        loggf("motor pwm must be -%d..%d\n", MOTOR_PWM_MAX, MOTOR_PWM_MAX);
        return;
    }

    motor->set_pwm(pwm);
    loggf("motor %s pwm=%d\n", argv[1], pwm);
}

SHELL_COMMAND("motor", motor_cmd, "motor test: pwm/vel/pid/enc commands")
