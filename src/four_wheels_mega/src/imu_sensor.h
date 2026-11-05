
#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <MPU6050.h>

extern MPU6050 mpu;

extern bool imuReady;

extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

// Gyroscope calibration
extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;

// Euler angles, radians
extern float roll;
extern float pitch;
extern float yaw;

// Quaternion
extern float orientation_x;
extern float orientation_y;
extern float orientation_z;
extern float orientation_w;

// Angular velocity, rad/s
extern float angular_velocity_x;
extern float angular_velocity_y;
extern float angular_velocity_z;

// Linear acceleration, m/s^2
extern float linear_acceleration_x;
extern float linear_acceleration_y;
extern float linear_acceleration_z;

extern unsigned long last_imu_us;

// IMU functions
void calibrateGyroscope();
void updateQuaternion();
void updateIMU();

#endif
