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

static void motor(int argc, char** argv) {

    if (argc != 3) {
        wifi_tx("usage: motor <fl|fr|rl|rr> <f|r|-255..255|auto>\n");
        return;
    }

    Motor* selected = get_motor(argv[1]);
    if (selected == nullptr) {
        wifi_tx("unknown motor '%s'. use fl/fr/rl/rr.\n", argv[1]);
        return;
    }

    int pwm = 0;
    if (strcmp(argv[2], "f") == 0) {
        pwm = 255;
    } else if (strcmp(argv[2], "r") == 0) {
        pwm = -255;
    } else if (strcmp(argv[2], "auto") == 0) {
        selected->clear_manual_pwm();
        wifi_tx("motor %s returned to chassis control.\n", argv[1]);
        return;
    } else if (!parse_pwm(argv[2], &pwm)) {
        wifi_tx("motor pwm must be f, r, auto, or an integer from -255 to 255.\n");
        return;
    }

    selected->set_manual_pwm(pwm);
    wifi_tx("motor %s pwm set to %d.\n", argv[1], pwm);
}
BASH_COMMAND("motor", motor, "test motor: motor <fl|fr|rl|rr> <f|r|-255..255|auto>")
