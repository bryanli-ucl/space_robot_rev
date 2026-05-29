#pragma once

#include <stdint.h>

class Rfid {
    public:
    static Rfid& instance();

    void begin();
    void update();
    void set_log_enabled(bool enabled) { log_enabled = enabled; }

    bool is_ready() const { return ready; }
    bool log_is_enabled() const { return log_enabled; }
    uint32_t last_uid() const { return uid; }
    uint32_t last_seen_ms() const { return seen_ms; }
    uint8_t last_uid_size() const { return uid_size; }
    uint32_t last_duration_ms() const { return duration_ms; }

    private:
    Rfid()                       = default;
    Rfid(const Rfid&)            = delete;
    Rfid& operator=(const Rfid&) = delete;

    bool ready = false;
    uint32_t uid = 0;
    uint32_t reported_uid = 0;
    uint32_t seen_ms = 0;
    uint32_t reported_ms = 0;
    uint32_t sample_ms = 0;
    uint32_t duration_ms = 0;
    uint8_t uid_size = 0;
    bool log_enabled = false;
};

extern Rfid& rfid;

void rfid_begin();
void rfid_update();
