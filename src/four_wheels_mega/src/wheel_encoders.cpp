
#include "wheel_encoders.h"

// ============================================================
// ENCODER COUNTERS
// ============================================================

volatile long encoder_leftValue = 0;
volatile long encoder_rightValue = 0;

volatile long encoder_leftValue_front = 0;
volatile long encoder_rightValue_front = 0;

// ============================================================
// WHEEL PARAMETERS
// ============================================================

const float RPM_TO_RAD = 0.104719755f;

const float WHEEL_DIAMETER = 0.09f; // 9cm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// const unsigned long ENCODER_INTERVAL = 20;

unsigned long previousMillis = 0;

// ============================================================
// WHEEL MEASUREMENTS
// ============================================================

float rpm_left = 0.0f;
float rpm_right = 0.0f;

float rpm_left_front = 0.0f;
float rpm_right_front = 0.0f;

float ang_vel_left = 0.0f;
float ang_vel_right = 0.0f;

float ang_vel_left_front = 0.0f;
float ang_vel_right_front = 0.0f;

double left_position = 0.0;
double right_position = 0.0;

float distance_left = 0.0f;
float distance_right = 0.0f;

float vel_left = 0.0f;
float vel_right = 0.0f;

// ============================================================
// ENCODER INTERRUPTS
// ============================================================

void wheel_pulse()
{
    if (digitalRead(dir_left) == LOW)
        encoder_leftValue++;
    else
        encoder_leftValue--;
}

void wheel_pulse_front()
{
    if (digitalRead(dir_left_front) == LOW)
        encoder_leftValue_front++;
    else
        encoder_leftValue_front--;
}

void wheel_pulse_right()
{
    if (digitalRead(dir_right) == LOW)
        encoder_rightValue--;
    else
        encoder_rightValue++;
}

void wheel_pulse_right_front()
{
    if (digitalRead(dir_right_front) == LOW)
        encoder_rightValue_front--;
    else
        encoder_rightValue_front++;
}
// ============================================================
// UPDATE ENCODER MEASUREMENTS
// ============================================================

const unsigned long ENCODER_INTERVAL = 50;   // was 20 (1 count = ~3 RPM at 20 ms)
constexpr float RPM_FILTER = 0.6f;           // 0 = no filtering, higher = smoother

static inline void filt(float &rpm, float raw)
{
    rpm = RPM_FILTER * rpm + (1.0f - RPM_FILTER) * raw;
}

bool updateEncoders()
{
    const unsigned long now = millis();
    const unsigned long elapsed = now - previousMillis;
    if (elapsed < ENCODER_INTERVAL) return false;
    previousMillis = now;

    noInterrupts();
    const long dl  = encoder_leftValue,        dr  = encoder_rightValue;
    const long dlf = encoder_leftValue_front,  drf = encoder_rightValue_front;
    encoder_leftValue = encoder_rightValue = 0;
    encoder_leftValue_front = encoder_rightValue_front = 0;
    interrupts();

    // Serial3.print(dlf);
    // Serial3.print("  ");
    // Serial3.println(dl);

    const float dt = elapsed / 1000.0f;
    const float k  = 60.0f / (PPR * dt);          // counts -> RPM

    filt(rpm_left,        dl  * k);
    filt(rpm_right,       dr  * k);
    filt(rpm_left_front,  dlf * k);
    filt(rpm_right_front, drf * k);

    ang_vel_left        = rpm_left        * RPM_TO_RAD;
    ang_vel_right       = rpm_right       * RPM_TO_RAD;
    ang_vel_left_front  = rpm_left_front  * RPM_TO_RAD;
    ang_vel_right_front = rpm_right_front * RPM_TO_RAD;

    left_position  += ((double)dl / PPR) * (2.0 * PI);
    right_position += ((double)dr / PPR) * (2.0 * PI);
    return true;
}