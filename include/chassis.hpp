#pragma once

class Chassis {
    public:
    static Chassis& instance();

    void begin();
    void set_target(float vx, float vy, float w);
    void update(float dt_s);
    void stop();

    float target_vx() const { return vx; }
    float target_vy() const { return vy; }
    float target_w() const { return w; }

    float wheel_fl() const { return fl; }
    float wheel_fr() const { return fr; }
    float wheel_rl() const { return rl; }
    float wheel_rr() const { return rr; }

    private:
    Chassis()                          = default;
    Chassis(const Chassis&)            = delete;
    Chassis& operator=(const Chassis&) = delete;

    float vx = 0.0f;
    float vy = 0.0f;
    float w  = 0.0f;

    float fl = 0.0f;
    float fr = 0.0f;
    float rl = 0.0f;
    float rr = 0.0f;
};

extern Chassis& chassis;

void chassis_begin();
void chassis_stop();
void func_chassis_entry();
