
#include "motor_control.h"
#include <math.h>

// ============================================================
// MOTOR COMMAND STATE
// ============================================================

float left_vel_cmd = 0.0f;
float right_vel_cmd = 0.0f;



// PWM conversion scale
// Preserves your original velocity-to-PWM mapping.
constexpr float PWM_SCALE = 100.0f;


// =========================
// PID
// ================


void pid_init(
    PID *pid,
    float kp,
    float ki,
    float kd,
    float out_min,
    float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->initialized = false;
}

float pid_update(PID *pid,float setpoint,float measurement,float dt)
{
    if (dt <= 0.0f || dt > 0.1f) {
        return 0.0f;
    }

    const float error = setpoint - measurement;

    if (!pid->initialized) {
        pid->prev_measurement = measurement;
        pid->initialized = true;
    }

    const float derivative =-(measurement - pid->prev_measurement) / dt;

    const float candidate_integral = pid->integral + error * dt;

    const float raw =
        pid->kp * error
        + pid->ki * candidate_integral
        + pid->kd * derivative;

    // Conditional integration: avoid winding up when saturated.
    if ((raw >= pid->out_min && raw <= pid->out_max) ||
        (raw > pid->out_max && error < 0.0f) ||
        (raw < pid->out_min && error > 0.0f)) {
        pid->integral = candidate_integral;
    }

    float output =
          pid->kp * error
        + pid->ki * pid->integral
        + pid->kd * derivative;

    output = constrain(output, pid->out_min, pid->out_max);

    pid->prev_measurement = measurement;

    return output;
}

// Feedforward: PWM per RPM. Set to 255 / (max no-load RPM at full PWM).
constexpr float KFF = 255.0f / 200.0f;   // <-- measure and replace 200

struct Wheel {
    uint8_t en, in1, in2;
    PID pid;
    const float *rpm;     // measured RPM for this wheel
};

static Wheel wheels[4] = {
    { ENA,  IN1,  IN2,  {}, &rpm_left        },  // left rear
    { ENA_, IN1_, IN2_, {}, &rpm_left_front  },  // left front
    { ENB,  IN3,  IN4,  {}, &rpm_right       },  // right rear
    { ENB_, IN3_, IN4_, {}, &rpm_right_front },  // right front
};

void initMotors()
{
    for (auto &w : wheels) {
        pinMode(w.en,  OUTPUT);
        pinMode(w.in1, OUTPUT);
        pinMode(w.in2, OUTPUT);
        setMotor(w.en, w.in1, w.in2, 0.0f);
        pid_init(&w.pid, 3.0f, 1.5f, 1.0f, 0.0f, 255.0f);
    }
}

void setMotor(uint8_t enable_pin, uint8_t input1_pin,
              uint8_t input2_pin, float velocity)
{
    int pwm = constrain((int)fabsf(velocity), 0, 255);

    if (velocity > 0.01f) {
        digitalWrite(input1_pin, HIGH);
        digitalWrite(input2_pin, LOW);
    } else if (velocity < -0.01f) {
        digitalWrite(input1_pin, LOW);
        digitalWrite(input2_pin, HIGH);
    } else {
        digitalWrite(input1_pin, LOW);
        digitalWrite(input2_pin, LOW);
        pwm = 0;
    }
    analogWrite(enable_pin, pwm);
}
constexpr float MIN_PWM = 55.0f;  // Tune experimentally
constexpr float MAX_PWM = 255.0f;


static void driveWheel(Wheel &w, float target_rpm, float dt)
{
    // Stop the motor when the target is effectively zero
    if (fabsf(target_rpm) < 0.05f) {
        w.pid.integral = 0.0f;
        w.pid.initialized = false;

        setMotor(w.en, w.in1, w.in2, 0.0f);
        return;
    }

    const float sign = (target_rpm > 0.0f) ? 1.0f : -1.0f;
    const float mag  = fabsf(target_rpm);

    // Feedforward and PID feedback
    const float ff = KFF * mag;
    const float fb = pid_update(
        &w.pid,
        mag,
        fabsf(*w.rpm),
        dt
    );

    float pwm = constrain(ff + fb, 0.0f, MAX_PWM);

    // Apply minimum PWM only when the controller requests motion
    // if (pwm > 0.0f && pwm < MIN_PWM) {
    //     pwm = MIN_PWM;
    // }

    setMotor(w.en, w.in1, w.in2, sign * pwm);

    Serial3.println(sign * pwm);
}



void setMotorVelocity()
{
    static uint32_t last_us = 0;
    const uint32_t now = micros();
    float dt = (now - last_us) * 1e-6f;
    last_us = now;
    if (dt > 0.1f) dt = ENCODER_INTERVAL / 1000.0f;   // first call / hiccup

    const float left_rpm  = left_vel_cmd  * 60.0f / (2.0f * PI);
    const float right_rpm = right_vel_cmd * 60.0f / (2.0f * PI);

    driveWheel(wheels[0], left_rpm,  dt);
    driveWheel(wheels[1], left_rpm,  dt);
    driveWheel(wheels[2], right_rpm, dt);
    driveWheel(wheels[3], right_rpm, dt);

    // Serial3.print(left_vel_cmd);
    // Serial3.print("  ");
    // Serial3.println(right_vel_cmd);


    // setMotor(ENA, IN1,IN2,left_vel_cmd);
    // setMotor(ENB, IN3,IN4,right_vel_cmd);

    // setMotor(ENA_, IN1_,IN2_,left_vel_cmd);
    // setMotor(ENB_, IN3_,IN4_,right_vel_cmd);


 
}
    





