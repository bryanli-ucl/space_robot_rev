#include "seed_dropper.hpp"

#include "logger.hpp"
#include "shell.hpp"

#include <stdlib.h>
#include <string.h>

static bool parse_int32(const char* text, int32_t* out) {
    if (text == nullptr || out == nullptr) return false;

    char* end = nullptr;
    const long value = strtol(text, &end, 0);
    if (end == text || *end != '\0') return false;

    *out = static_cast<int32_t>(value);
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

static void print_seed_status() {
    loggf("seed running=%d pwm=%u count=%ld default=%ld timeout=%lums pins=%u/%u/%u\n",
    seed_dropper.is_running() ? 1 : 0,
    static_cast<unsigned>(seed_dropper.pwm()),
    static_cast<long>(seed_dropper.count()),
    static_cast<long>(SEED_DROPPER_DROP_COUNTS),
    static_cast<unsigned long>(SEED_DROPPER_DROP_TIMEOUT_MS),
    static_cast<unsigned>(SEED_DROPPER_PWM_PIN),
    static_cast<unsigned>(SEED_DROPPER_ENC_A_PIN),
    static_cast<unsigned>(SEED_DROPPER_ENC_B_PIN));
}

static void seed_cmd(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && strcmp(argv[1], "status") == 0)) {
        print_seed_status();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "on") == 0) {
        seed_dropper.start();
        print_seed_status();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "off") == 0) {
        seed_dropper.stop();
        print_seed_status();
        return;
    }

    if (argc == 2 && strcmp(argv[1], "count0") == 0) {
        seed_dropper.reset_count();
        print_seed_status();
        return;
    }

    if (argc >= 2 && strcmp(argv[1], "drop") == 0) {
        if (argc > 4) {
            loggf("usage: seed drop [counts] [timeout_ms]\n");
            return;
        }

        int32_t counts = SEED_DROPPER_DROP_COUNTS;
        uint32_t timeout_ms = SEED_DROPPER_DROP_TIMEOUT_MS;
        if (argc >= 3 && (!parse_int32(argv[2], &counts) || counts <= 0)) {
            loggf("seed drop counts must be a positive integer\n");
            return;
        }

        if (argc == 4 && (!parse_uint32(argv[3], &timeout_ms) || timeout_ms == 0)) {
            loggf("seed drop timeout_ms must be a positive integer\n");
            return;
        }

        seed_dropper.drop_one(counts, timeout_ms);
        return;
    }

    loggf("usage: seed status|on|off|count0|drop [counts] [timeout_ms]\n");
}

SHELL_COMMAND("seed", seed_cmd, "seed dropper: seed drop/status/on/off/count0")
