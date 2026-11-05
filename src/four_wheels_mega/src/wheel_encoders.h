
#ifndef WHEEL_ENCODERS_H
#define WHEEL_ENCODERS_H

#include <Arduino.h>

// Encoder configuration
#define PPR 988


#define ENC_interruptPin_left  2
#define ENC_interruptPin_right 3

#define ENC_interruptPin_left_front  19
#define ENC_interruptPin_right_front  18



#define dir_left  11
#define dir_right 12

#define dir_left_front  22
#define dir_right_front 24

// Encoder interrupt counters
extern volatile long encoder_leftValue;
extern volatile long encoder_rightValue;

extern volatile long encoder_leftValue_front;
extern volatile long encoder_rightValue_front;

// Wheel parameters
extern const float RPM_TO_RAD;
extern const float WHEEL_DIAMETER;
extern const float WHEEL_CIRCUMFERENCE;

extern const unsigned long ENCODER_INTERVAL;
extern unsigned long previousMillis;

// Wheel measurements
extern float rpm_left;
extern float rpm_right;

extern float rpm_left_front;
extern float rpm_right_front;

extern float ang_vel_left;
extern float ang_vel_right;

extern double left_position;
extern double right_position;

extern float distance_left;
extern float distance_right;

extern float vel_left;
extern float vel_right;

// Encoder interrupt handlers
void wheel_pulse();
void wheel_pulse_right();

void wheel_pulse_front();
void wheel_pulse_right_front();

// Encoder measurement update
bool updateEncoders();

#endif
