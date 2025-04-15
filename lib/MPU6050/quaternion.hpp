#include <iostream>
#include <cmath>

#define RAD2DEG(x) ((x) * 180.0f / M_PI)
#define DEG2RAD(x) ((x) * M_PI) / 180.0f

struct MadgwickAHRS {
    float beta;     // Algorithm gain
    float q0, q1, q2, q3; // Quaternion

    MadgwickAHRS(float beta_ = 0.1f) : beta(beta_), q0(1), q1(0), q2(0), q3(0) {}

    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        float norm, s0, s1, s2, s3;
        float qDot0, qDot1, qDot2, qDot3;

        // Normalize accelerometer
        norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // avoid divide by zero
        ax /= norm;
        ay /= norm;
        az /= norm;

        // Rate of change of quaternion from gyroscope
        qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
        qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
        qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

        // Gradient descent algorithm corrective step
        float f1 = 2*(q1*q3 - q0*q2) - ax;
        float f2 = 2*(q0*q1 + q2*q3) - ay;
        float f3 = 2*(0.5f - q1*q1 - q2*q2) - az;
        s0 = -2*q2*f1 + 2*q1*f2;
        s1 = 2*q3*f1 + 2*q0*f2 - 4*q1*f3;
        s2 = -2*q0*f1 + 2*q3*f2 - 4*q2*f3;
        s3 = 2*q1*f1 + 2*q2*f2;

        norm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (norm == 0.0f) return;
        s0 /= norm;
        s1 /= norm;
        s2 /= norm;
        s3 /= norm;

        // Apply feedback step
        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;

        // Integrate rate of change
        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;

        // Normalize quaternion
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }

    void getEuler(float &roll, float &pitch, float &yaw) const {
        roll = RAD2DEG(atan2(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)));
        pitch = RAD2DEG(asin(2.0f*(q0*q2 - q3*q1)));
        yaw = atan2(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
    }
};