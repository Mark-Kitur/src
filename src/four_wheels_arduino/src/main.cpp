#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


// ============================================================
// ENCODERS
// ============================================================

#define Pulses_Rev 995

#define ENC_interruptPin_left  2
#define ENC_interruptPin_right 3

#define dir_left  11
#define dir_right 12

volatile long encoder_leftValue = 0;
volatile long encoder_rightValue = 0;

bool dir_left_state = true;
bool dir_right_state = true;

void wheel_pulse()
{
    int val = digitalRead(dir_left);

    if (val == LOW)
        dir_left_state = true;
    else
        dir_left_state = false;

    if (dir_left_state)
        encoder_leftValue++;
    else
        encoder_leftValue--;
}

void wheel_pulse_right()
{
    int val = digitalRead(dir_right);

    if (val == LOW)
        dir_right_state = false;
    else
        dir_right_state = true;

    if (dir_right_state)
        encoder_rightValue++;
    else
        encoder_rightValue--;
}


// ============================================================
// WHEEL PARAMETERS
// ============================================================

const float WHEEL_RADIUS_M = 0.09f;

const float RPM_TO_RAD = 0.104719755f;
//const float RAD_TO_DEG = 57.2957795f;

const unsigned long ENCODER_INTERVAL = 1000;

unsigned long previousMillis = 0;

float rpm_left = 0.0f;
float rpm_right = 0.0f;

float ang_vel_left = 0.0f;
float ang_vel_right = 0.0f;

float linear_velocity_left = 0.0f;
float linear_velocity_right = 0.0f;

double left_position = 0.0;
double right_position = 0.0;


// ============================================================
// MOTOR PINS
// ============================================================

//const int driver_vcc = 8;

// Left motor
const int IN1 = 4;
const int IN2 = 5;
const int ENA = 9;

// Right motor
const int IN3 = 6;
const int IN4 = 7;
const int ENB = 10;


// ============================================================
// MOTOR CONTROL
// ============================================================

float left_vel = 0.0f;
float right_vel = 0.0f;

const float PWM_SCALE = 100.0f;

void setMotor(int en, int in1, int in2, float velocity)
{
    int pwm = (int)(fabs(velocity) * PWM_SCALE);

    pwm = constrain(pwm, 0, 255);

    if (velocity > 0.0f)
    {
        // Forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if (velocity < 0.0f)
    {
        // Reverse
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else
    {
        // Stop
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    analogWrite(en, pwm);
}


// ============================================================
// MPU6050 DMP
// ============================================================

#define MPU_INT_PIN 13

MPU6050 mpu;

bool dmpReady = false;

volatile bool mpuInterrupt = false;

uint8_t mpuIntStatus;
uint8_t devStatus;

uint16_t packetSize;
uint16_t fifoCount;

uint8_t fifoBuffer[64];


// DMP objects
Quaternion q;
VectorFloat gravity;

VectorInt16 aa;
VectorInt16 aaReal;

float ypr[3];


// ============================================================
// IMU OUTPUT VARIABLES
// ============================================================

// Quaternion
float orientation_x = 0.0f;
float orientation_y = 0.0f;
float orientation_z = 0.0f;
float orientation_w = 1.0f;

// Angular velocity [rad/s]
float angular_velocity_x = 0.0f;
float angular_velocity_y = 0.0f;
float angular_velocity_z = 0.0f;

// Linear acceleration [m/s²]
float linear_acceleration_x = 0.0f;
float linear_acceleration_y = 0.0f;
float linear_acceleration_z = 0.0f;


// ============================================================
// MPU INTERRUPT
// ============================================================

void dmpDataReady()
{
    mpuInterrupt = true;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.begin();

    // 400 kHz I2C
    Wire.setClock(400000);


    // --------------------------------------------------------
    // MOTOR SETUP
    // --------------------------------------------------------

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(driver_vcc, OUTPUT);
    digitalWrite(driver_vcc, HIGH);

    // Motors initially stopped
    setMotor(ENA, IN1, IN2, 0.0f);
    setMotor(ENB, IN3, IN4, 0.0f);


    // --------------------------------------------------------
    // ENCODERS
    // --------------------------------------------------------

    pinMode(ENC_interruptPin_left, INPUT_PULLUP);
    pinMode(ENC_interruptPin_right, INPUT_PULLUP);

    pinMode(dir_left, INPUT_PULLUP);
    pinMode(dir_right, INPUT_PULLUP);

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


    // --------------------------------------------------------
    // MPU6050
    // --------------------------------------------------------

    Serial.println("Initializing MPU6050...");

    mpu.initialize();

    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed!");

        while (1)
        {
            delay(1000);
        }
    }

    Serial.println("MPU6050 connection successful.");


    // --------------------------------------------------------
    // DMP INITIALIZATION
    // --------------------------------------------------------

    Serial.println("Initializing DMP...");

    devStatus = mpu.dmpInitialize();


    // --------------------------------------------------------
    // YOUR CALIBRATION OFFSETS
    // --------------------------------------------------------

    mpu.setXGyroOffset(256);
    mpu.setYGyroOffset(12);
    mpu.setZGyroOffset(46);

    mpu.setZAccelOffset(37);


    // --------------------------------------------------------
    // CHECK DMP
    // --------------------------------------------------------

    if (devStatus == 0)
    {
        Serial.println("DMP initialization successful.");

        mpu.setDMPEnabled(true);

        pinMode(MPU_INT_PIN, INPUT);

        attachInterrupt(
            digitalPinToInterrupt(MPU_INT_PIN),
            dmpDataReady,
            RISING
        );

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();

        Serial.print("DMP packet size: ");
        Serial.println(packetSize);
    }
    else
    {
        Serial.print("DMP initialization failed. Code: ");
        Serial.println(devStatus);

        while (1)
        {
            delay(1000);
        }
    }


    // --------------------------------------------------------
    // SERIAL READY
    // --------------------------------------------------------

    while (!Serial)
    {
    }

    previousMillis = millis();

    Serial.println("Robot controller ready.");
}


// ============================================================
// UPDATE ENCODERS
// ============================================================

void updateEncoders()
{
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= ENCODER_INTERVAL)
    {
        unsigned long dt = currentMillis - previousMillis;

        previousMillis = currentMillis;


        // Safely copy encoder counts
        noInterrupts();

        long dl = encoder_leftValue;
        long dr = encoder_rightValue;

        encoder_leftValue = 0;
        encoder_rightValue = 0;

        interrupts();


        // ----------------------------------------------------
        // RPM
        // ----------------------------------------------------

        rpm_left =
            (dl * 60000.0f) /
            (Pulses_Rev * dt);

        rpm_right =
            (dr * 60000.0f) /
            (Pulses_Rev * dt);


        // ----------------------------------------------------
        // Angular wheel velocity
        // ----------------------------------------------------

        ang_vel_left =
            rpm_left * RPM_TO_RAD;

        ang_vel_right =
            rpm_right * RPM_TO_RAD;


        // ----------------------------------------------------
        // Linear wheel velocity
        // ----------------------------------------------------

        linear_velocity_left =
            ang_vel_left * WHEEL_RADIUS_M;

        linear_velocity_right =
            ang_vel_right * WHEEL_RADIUS_M;


        // ----------------------------------------------------
        // Wheel position
        // ----------------------------------------------------

        left_position +=
            ((double)dl / Pulses_Rev) *
            (2.0 * PI * WHEEL_RADIUS_M);

        right_position +=
            ((double)dr / Pulses_Rev) *
            (2.0 * PI * WHEEL_RADIUS_M);
    }
}


// ============================================================
// UPDATE MPU6050 DMP
// ============================================================

void updateIMU()
{
    if (!dmpReady)
        return;


    // --------------------------------------------------------
    // Check if DMP has produced a packet
    // --------------------------------------------------------

    if (!mpuInterrupt)
        return;


    mpuInterrupt = false;

    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();


    // --------------------------------------------------------
    // FIFO OVERFLOW
    // --------------------------------------------------------

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        mpu.resetFIFO();

        return;
    }


    // --------------------------------------------------------
    // DMP DATA READY
    // --------------------------------------------------------

    if (mpuIntStatus & 0x02)
    {
        // Wait until a complete packet exists
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }


        // Read DMP packet
        mpu.getFIFOBytes(
            fifoBuffer,
            packetSize
        );

        fifoCount -= packetSize;


        // ----------------------------------------------------
        // QUATERNION
        // ----------------------------------------------------

        mpu.dmpGetQuaternion(
            &q,
            fifoBuffer
        );

        orientation_x = q.x;
        orientation_y = q.y;
        orientation_z = q.z;
        orientation_w = q.w;


        // ----------------------------------------------------
        // GRAVITY VECTOR
        // ----------------------------------------------------

        mpu.dmpGetGravity(
            &gravity,
            &q
        );


        // ----------------------------------------------------
        // LINEAR ACCELERATION
        // ----------------------------------------------------

        mpu.dmpGetAccel(
            &aa,
            fifoBuffer
        );

        mpu.dmpGetLinearAccel(
            &aaReal,
            &aa,
            &gravity
        );


        // MPU6050 default sensitivity:
        // ±2g -> 16384 LSB/g

        linear_acceleration_x =
            (aaReal.x / 16384.0f) * 9.80665f;

        linear_acceleration_y =
            (aaReal.y / 16384.0f) * 9.80665f;

        linear_acceleration_z =
            (aaReal.z / 16384.0f) * 9.80665f;


        // ----------------------------------------------------
        // RAW GYROSCOPE
        // ----------------------------------------------------

        int16_t gx, gy, gz;

        mpu.getRotation(
            &gx,
            &gy,
            &gz
        );


        // MPU6050 default:
        // ±250 deg/s -> 131 LSB/(deg/s)

        angular_velocity_x =
            (gx / 131.0f) *
            DEG_TO_RAD;

        angular_velocity_y =
            (gy / 131.0f) *
            DEG_TO_RAD;

        angular_velocity_z =
            (gz / 131.0f) *
            DEG_TO_RAD;
    }
}


// ============================================================
// RECEIVE ROS2 / RASPBERRY PI COMMAND
// ============================================================

void receiveVelocityCommand()
{
    if (Serial.available())
    {
        String line =
            Serial.readStringUntil('\n');

        line.trim();

        int comma =
            line.indexOf(',');

        if (comma != -1)
        {
            left_vel =
                line.substring(
                    0,
                    comma
                ).toFloat();

            right_vel =
                line.substring(
                    comma + 1
                ).toFloat();
        }
    }
}


// ============================================================
// SEND ROBOT FEEDBACK
// ============================================================

void sendFeedback()
{
    // --------------------------------------------------------
    // Wheel position
    // --------------------------------------------------------

    Serial.print(left_position, 3);
    Serial.print(",");

    Serial.print(linear_velocity_left, 3);
    Serial.print(",");

    Serial.print(right_position, 3);
    Serial.print(",");

    Serial.print(linear_velocity_right, 3);
    Serial.print(",");


    // --------------------------------------------------------
    // Quaternion
    // --------------------------------------------------------

    Serial.print(orientation_x, 4);
    Serial.print(",");

    Serial.print(orientation_y, 4);
    Serial.print(",");

    Serial.print(orientation_z, 4);
    Serial.print(",");

    Serial.print(orientation_w, 4);
    Serial.print(",");


    // --------------------------------------------------------
    // Angular velocity
    // --------------------------------------------------------

    Serial.print(angular_velocity_x, 4);
    Serial.print(",");

    Serial.print(angular_velocity_y, 4);
    Serial.print(",");

    Serial.print(angular_velocity_z, 4);
    Serial.print(",");


    // --------------------------------------------------------
    // Linear acceleration
    // --------------------------------------------------------

    Serial.print(linear_acceleration_x, 4);
    Serial.print(",");

    Serial.print(linear_acceleration_y, 4);
    Serial.print(",");

    Serial.print(linear_acceleration_z, 4);


    Serial.println();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // Receive commands from Raspberry Pi
    receiveVelocityCommand();


    // Drive motors
    setMotor(
        ENA,
        IN1,
        IN2,
        left_vel
    );

    setMotor(
        ENB,
        IN3,
        IN4,
        right_vel
    );


    // Update encoder calculations
    updateEncoders();


    // Update MPU6050 DMP
    updateIMU();


    // Send feedback
    sendFeedback();


    // ~10 Hz feedback
    delay(100);
}