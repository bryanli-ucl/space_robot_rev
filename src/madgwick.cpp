#include "madgwick.hpp"
#include <cmath>

Madgwick::Madgwick(float beta) : beta(beta), inv_sample_freq(0.f), q(1.0f, 0.0f, 0.0f, 0.0f) {}

Madgwick::~Madgwick() {}

void Madgwick::begin(float sample_freq) {
    inv_sample_freq = 1.0f / sample_freq;
    q               = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
}

float Madgwick::inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

void Madgwick::update(Eigen::Vector3f acc, Eigen::Vector3f gyro, Eigen::Vector3f mag, float dt) {
    constexpr float DEG_TO_RAD = M_PI / 180.0f;

    float ax = acc.x(), ay = acc.y(), az = acc.z();
    float gx = gyro.x() * DEG_TO_RAD, gy = gyro.y() * DEG_TO_RAD, gz = gyro.z() * DEG_TO_RAD;
    float mx = mag.x(), my = mag.y(), mz = mag.z();

    float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU-only algorithm if magnetometer measurement invalid (avoids NaN)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            _2q0       = 2.0f * q0;
            _2q1       = 2.0f * q1;
            _2q2       = 2.0f * q2;
            _2q3       = 2.0f * q3;
            float _4q0 = 4.0f * q0;
            float _4q1 = 4.0f * q1;
            float _4q2 = 4.0f * q2;
            float _8q1 = 8.0f * q1;
            float _8q2 = 8.0f * q2;
            q0q0       = q0 * q0;
            q1q1       = q1 * q1;
            q2q2       = q2 * q2;
            q3q3       = q3 * q3;

            s0        = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1        = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2        = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3        = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        q = Eigen::Quaternionf(q0, q1, q2, q3);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0   = 2.0f * q0;
        _2q1   = 2.0f * q1;
        _2q2   = 2.0f * q2;
        _2q3   = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0   = q0 * q0;
        q0q1   = q0 * q1;
        q0q2   = q0 * q2;
        q0q3   = q0 * q3;
        q1q1   = q1 * q1;
        q1q2   = q1 * q2;
        q1q3   = q1 * q3;
        q2q2   = q2 * q2;
        q2q3   = q2 * q3;
        q3q3   = q3 * q3;

        hx   = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy   = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        s0        = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _4bz * q1) * (_2bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * q2 * (_2bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
        s1        = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _4bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _4bz * q0) * (_2bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
        s2        = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _4bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _4bz * q3) * (_2bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
        s3        = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _4bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _4bz * q2) * (_2bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q = Eigen::Quaternionf(q0, q1, q2, q3);
}

float Madgwick::get_roll() const {
    float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    return atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
}

float Madgwick::get_pitch() const {
    float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >= 1.0f) return M_PI / 2.0f;
    if (sinp <= -1.0f) return -M_PI / 2.0f;
    return asinf(sinp);
}

float Madgwick::get_yaw() const {
    float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
