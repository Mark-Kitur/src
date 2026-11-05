
#include "imu_sensor.h"
#include <Wire.h>
#include <math.h>

// ============================================================
// MPU6050 AND IMU STATE
// ============================================================

MPU6050 mpu;

bool imuReady = false;

int16_t ax, ay, az;
int16_t gx, gy, gz;

const float GRAVITY = 9.80665f;
const float DEG_TO_RAD_F = 0.01745329252f;

const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE = 131.0f;

const float FILTER_ALPHA = 0.98f;

float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

float orientation_x = 0.0f;
float orientation_y = 0.0f;
float orientation_z = 0.0f;
float orientation_w = 1.0f;

float angular_velocity_x = 0.0f;
float angular_velocity_y = 0.0f;
float angular_velocity_z = 0.0f;

float linear_acceleration_x = 0.0f;
float linear_acceleration_y = 0.0f;
float linear_acceleration_z = 0.0f;

unsigned long last_imu_us = 0;


void calibrateGyroscope()
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    uint16_t samples = 0;

    // Collect up to 500 valid samples, with a bounded wait.
    for (uint16_t i = 0; i < 600 && samples < 500; i++)
    {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (Wire.getWireTimeoutFlag())
        {
            Wire.clearWireTimeoutFlag();
            delay(2);
            continue;
        }

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        samples++;
        delay(2);
    }

    // Only accept calibration if enough samples succeeded.
    if (samples >= 400)
    {
        gyro_offset_x = (float)sum_x / samples;
        gyro_offset_y = (float)sum_y / samples;
        gyro_offset_z = (float)sum_z / samples;
    }
}

void updateQuaternion()
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    orientation_x = sr * cp * cy - cr * sp * sy;
    orientation_y = cr * sp * cy + sr * cp * sy;
    orientation_z = cr * cp * sy - sr * sp * cy;
    orientation_w = cr * cp * cy + sr * sp * sy;
}

void updateIMU()
{
    if (!imuReady)
        return;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (Wire.getWireTimeoutFlag())
    {
        Wire.clearWireTimeoutFlag();

        // Avoid integrating across a failed I2C transaction.
        last_imu_us = micros();

        angular_velocity_x = 0.0f;
        angular_velocity_y = 0.0f;
        angular_velocity_z = 0.0f;

        linear_acceleration_x = 0.0f;
        linear_acceleration_y = 0.0f;
        linear_acceleration_z = 0.0f;

        return;
    }

    unsigned long now = micros();

    float dt = (now - last_imu_us) * 1.0e-6f;
    last_imu_us = now;

    // Reject implausible time steps.
    if (dt <= 0.0f || dt > 0.1f)
        return;

    // Accelerometer: m/s^2
    float accel_x = ((float)ax / ACCEL_SCALE) * GRAVITY;
    float accel_y = ((float)ay / ACCEL_SCALE) * GRAVITY;
    float accel_z = ((float)az / ACCEL_SCALE) * GRAVITY;

    // Gyroscope: rad/s
    angular_velocity_x =
        ((float)gx - gyro_offset_x) / GYRO_SCALE * DEG_TO_RAD_F;

    angular_velocity_y =
        ((float)gy - gyro_offset_y) / GYRO_SCALE * DEG_TO_RAD_F;

    angular_velocity_z =
        ((float)gz - gyro_offset_z) / GYRO_SCALE * DEG_TO_RAD_F;

    // Accelerometer-derived roll and pitch.
    float accel_roll = atan2f(accel_y, accel_z);

    float accel_pitch = atan2f(
        -accel_x,
        sqrtf(accel_y * accel_y + accel_z * accel_z)
    );

    // Complementary filter.
    roll = FILTER_ALPHA *
               (roll + angular_velocity_x * dt)
         + (1.0f - FILTER_ALPHA) * accel_roll;

    pitch = FILTER_ALPHA *
                (pitch + angular_velocity_y * dt)
          + (1.0f - FILTER_ALPHA) * accel_pitch;

    // Yaw is relative and will drift without a magnetometer.
    yaw += angular_velocity_z * dt;

    updateQuaternion();

    // Estimate gravity in the sensor/body frame.
    float gravity_x =
        2.0f * (orientation_x * orientation_z
              - orientation_w * orientation_y);

    float gravity_y =
        2.0f * (orientation_w * orientation_x
              + orientation_y * orientation_z);

    float gravity_z =
        orientation_w * orientation_w
        - orientation_x * orientation_x
        - orientation_y * orientation_y
        + orientation_z * orientation_z;

    // Remove estimated gravity from accelerometer readings.
    linear_acceleration_x = accel_x - gravity_x * GRAVITY;
    linear_acceleration_y = accel_y - gravity_y * GRAVITY;
    linear_acceleration_z = accel_z - gravity_z * GRAVITY;
}
