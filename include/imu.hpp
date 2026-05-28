#pragma once

class Imu {
    public:
    static Imu& instance();

    void begin();
    void entry();

    bool is_ready() const;
    bool yaw_is_ready() const;
    bool is_calibrating() const;
    float yaw_deg() const;
    float pitch_deg() const;
    float roll_deg() const;
    float accel_x() const;
    float accel_y() const;
    float accel_z() const;
    float gyro_x() const;
    float gyro_y() const;
    float gyro_z() const;
    float mag_x() const;
    float mag_y() const;
    float mag_z() const;
    float gyro_bias_x() const;
    float gyro_bias_y() const;
    float gyro_bias_z() const;
    void zero_yaw(float yaw = 0.0f);
    bool request_calibration(bool gyro, bool mag);

    private:
    Imu()                       = default;
    Imu(const Imu&)             = delete;
    Imu& operator=(const Imu&)  = delete;

    volatile bool ready_value = false;
    volatile bool yaw_ready_value = false;
    volatile bool calibration_busy_value = false;
    volatile bool gyro_calibration_requested = false;
    volatile bool mag_calibration_requested = false;
    volatile float yaw_deg_value = 0.0f;
    volatile float pitch_deg_value = 0.0f;
    volatile float roll_deg_value = 0.0f;
    volatile float accel_x_value = 0.0f;
    volatile float accel_y_value = 0.0f;
    volatile float accel_z_value = 0.0f;
    volatile float gyro_x_value = 0.0f;
    volatile float gyro_y_value = 0.0f;
    volatile float gyro_z_value = 0.0f;
    volatile float mag_x_value = 0.0f;
    volatile float mag_y_value = 0.0f;
    volatile float mag_z_value = 0.0f;
    volatile float gyro_bias_x_value = 0.0f;
    volatile float gyro_bias_y_value = 0.0f;
    volatile float gyro_bias_z_value = 0.0f;

    void set_orientation(float yaw, float pitch, float roll);
    void set_raw(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void set_gyro_bias(float bx, float by, float bz);
    void set_ready(bool ready);
    void set_calibration_busy(bool busy);
    void run_requested_calibration(bool gyro, bool mag);
    void service_calibration_request();
};

extern Imu& imu;

void imu_begin();
void func_imu_entry();
