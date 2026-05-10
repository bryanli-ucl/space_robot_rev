#pragma once

#define EIGEN_NO_MALLOC
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#ifdef abs
#undef abs
#endif

// Undef Arduino pin macros that clash with Eigen internal variable names
#ifdef D0
#undef D0
#endif
#ifdef D1
#undef D1
#endif
#ifdef D2
#undef D2
#endif
#ifdef D3
#undef D3
#endif
#ifdef D4
#undef D4
#endif
#ifdef D5
#undef D5
#endif
#ifdef D6
#undef D6
#endif
#ifdef D7
#undef D7
#endif
#ifdef D8
#undef D8
#endif
#ifdef D9
#undef D9
#endif

#include <Eigen/Eigen>

class Madgwick {
private:
    float beta;
    float inv_sample_freq;

    Eigen::Quaternionf q;

public:
    Madgwick(float beta = 0.041f);
    ~Madgwick();

    void begin(float sample_freq);

    void update(Eigen::Vector3f acc, Eigen::Vector3f gyro, Eigen::Vector3f mag, float dt);

    float get_roll() const;
    float get_pitch() const;
    float get_yaw() const;

    Eigen::Quaternionf get_quaternion() const { return q; }

private:
    static float inv_sqrt(float x);
};
