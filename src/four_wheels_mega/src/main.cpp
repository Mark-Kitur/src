#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "motor_control.h"
#include "wheel_encoders.h"
#include "imu_sensor.h"

// ============================================================
// SERIAL COMMAND PARSER
// Receives ROS 2 commands: "left_velocity,right_velocity\n"
// ============================================================



char commandBuffer[32];
uint8_t commandIndex = 0;

void receiveVelocityCommand()
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            commandBuffer[commandIndex] = '\0';

            char *comma = strchr(commandBuffer, ',');

            if (comma != nullptr)
            {
                *comma = '\0';

                left_vel_cmd = atof(commandBuffer);
                right_vel_cmd = atof(comma + 1);
            }

            commandIndex = 0;
        }
        else if (commandIndex < sizeof(commandBuffer) - 1)
        {
            commandBuffer[commandIndex++] = c;
        }
        else
        {
            // Discard an overlong command.
            commandIndex = 0;
        }
    }
}

// ============================================================
// SEND ROS 2 FEEDBACK
// Exactly 14 fields:
// left_position,left_velocity,right_position,right_velocity,
// qx,qy,qz,qw,wx,wy,wz,ax,ay,az
// ============================================================

void sendFeedback()
{
    Serial.print(left_position, 4);
    Serial.print(',');

    Serial.print(ang_vel_left, 4);
    Serial.print(',');

    Serial.print(right_position, 4);
    Serial.print(',');

    Serial.print(ang_vel_right, 4);
    Serial.print(',');

    Serial.print(orientation_x, 4);
    Serial.print(',');

    Serial.print(orientation_y, 4);
    Serial.print(',');

    Serial.print(orientation_z, 4);
    Serial.print(',');

    Serial.print(orientation_w, 4);
    Serial.print(',');

    Serial.print(angular_velocity_x, 4);
    Serial.print(',');

    Serial.print(angular_velocity_y, 4);
    Serial.print(',');

    Serial.print(angular_velocity_z, 4);
    Serial.print(',');

    Serial.print(linear_acceleration_x, 4);
    Serial.print(',');

    Serial.print(linear_acceleration_y, 4);
    Serial.print(',');

    Serial.println(linear_acceleration_z, 4);
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    // ROS 2 serial communication
    Serial.begin(115200);
    //Serial3.begin(115200);

    // I2C setup for MPU6050
    Wire.begin();
    Wire.setClock(400000);

    // Timeout in microseconds; reset TWI peripheral on timeout.
    Wire.setWireTimeout(25000, true);

    // --------------------------------------------------------
    // MOTOR INITIALIZATION
    // --------------------------------------------------------

    initMotors();

    // --------------------------------------------------------
    // ENCODER INITIALIZATION
    // --------------------------------------------------------

    pinMode(ENC_interruptPin_left, INPUT_PULLUP);
    pinMode(ENC_interruptPin_right, INPUT_PULLUP);

    pinMode(ENC_interruptPin_left_front, INPUT_PULLUP);
    pinMode(ENC_interruptPin_right_front, INPUT_PULLUP);



    pinMode(dir_left, INPUT_PULLUP);
    pinMode(dir_right, INPUT_PULLUP);

    pinMode(dir_left_front, INPUT_PULLUP);
    pinMode(dir_right_front, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(ENC_interruptPin_left),
        wheel_pulse,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(ENC_interruptPin_right),
        wheel_pulse_right,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(ENC_interruptPin_left_front),
        wheel_pulse_front,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(ENC_interruptPin_right_front),
        wheel_pulse_right_front,
        RISING
    );

    // --------------------------------------------------------
    // MPU6050 INITIALIZATION
    // Direct sensor readings; no DMP or MPU interrupt.
    // --------------------------------------------------------

    mpu.initialize();

    if (mpu.testConnection() && !Wire.getWireTimeoutFlag())
    {
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

        // Keep the robot stationary during calibration.
        calibrateGyroscope();

        if (!Wire.getWireTimeoutFlag())
        {
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            if (!Wire.getWireTimeoutFlag())
            {
                // Initial roll and pitch from gravity.
                float accel_x = (float)ax;
                float accel_y = (float)ay;
                float accel_z = (float)az;

                roll = atan2f(accel_y, accel_z);

                pitch = atan2f(
                    -accel_x,
                    sqrtf(
                        accel_y * accel_y +
                        accel_z * accel_z
                    )
                );

                // Yaw has no absolute reference without
                // a magnetometer.
                yaw = 0.0f;

                updateQuaternion();

                imuReady = true;
            }
        }
    }

    // Clear any I2C timeout flag left during initialization.
    if (Wire.getWireTimeoutFlag())
        Wire.clearWireTimeoutFlag();

    // Do not print diagnostic messages on the ROS feedback UART.
    // If IMU initialization fails, default orientation remains
    // identity and IMU measurements remain zero.

    previousMillis = millis();
    last_imu_us = micros();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // Receive motor commands from Raspberry Pi.
    receiveVelocityCommand();

    // Update wheel odometry and IMU measurements.
    // Run the wheel PIDs once per fresh RPM measurement (every 50 ms).
    if (updateEncoders()) {
        setMotorVelocity();

        // Tuning output on a second UART (Serial3). Remove when done.
        // Serial3.print(left_vel_cmd * 60.0f / (2.0f * PI)); Serial3.print(',');
        // Serial3.print(rpm_left);        Serial3.print(',');
        // Serial3.print(rpm_left_front);  Serial3.print(',');
        // Serial3.print(rpm_right);       Serial3.print(',');
        // Serial3.println(rpm_right_front);
        
        // Serial3.print(digitalRead(ENC_interruptPin_left_front)); Serial3.print(',');
        // Serial3.print(digitalRead(dir_left_front)); Serial3.print(',');
        // Serial3.println(encoder_leftValue_front);
    }

    
    updateIMU();

    // Transmit exactly 14 feedback fields.
    sendFeedback();

    // Approximately 50 Hz loop.
    delay(20);
}
