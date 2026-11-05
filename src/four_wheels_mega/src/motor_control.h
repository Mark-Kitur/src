
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "wheel_encoders.h"  // get rpm


// Motor driver pins
constexpr uint8_t IN1 = 4;
constexpr uint8_t IN2 = 5;
constexpr uint8_t ENA = 9;

constexpr uint8_t IN1_ = 42;
constexpr uint8_t IN2_ = 43;
constexpr uint8_t ENA_ = 44;




constexpr uint8_t IN3 = 6;
constexpr uint8_t IN4 = 7;
constexpr uint8_t ENB = 10;

constexpr uint8_t IN3_ = 47;
constexpr uint8_t IN4_ = 48;
constexpr uint8_t ENB_ = 46;

// Velocity commands received from ROS 2
extern float left_vel_cmd;
extern float right_vel_cmd;



// Initialize motor driver pins and stop motors
void initMotors();

// Set left and right motor commands
void setMotorVelocity();

// Drive an individual motor
void setMotor(
    uint8_t enable_pin,
    uint8_t input1_pin,
    uint8_t input2_pin,
    float velocity
);

// PID

struct PID {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_measurement;

    float out_min;
    float out_max;

    bool initialized;
};

void pid_init(
    PID *pid,
    float kp,
    float ki,
    float kd,
    float out_min,
    float out_max
);

float pid_update(
    PID *pid,
    float setpoint,
    float measurement,
    float dt
);



#endif
