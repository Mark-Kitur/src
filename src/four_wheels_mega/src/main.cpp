#include <Arduino.h>

// ===========================================================================
// Arduino Mega Firmware for ROS 2 four_wheel_hardware
// 
// Protocol:
//   RX (from host): "left_cmd,right_cmd\n"          (2 doubles)
//   TX (to host):   "l_pos,l_vel,r_pos,r_vel\n"     (4 doubles)
// ===========================================================================

// Note: Arduino Mega uses HardwareSerial (Serial = USB, Serial1/2/3 = pins)
// Connect your host to Serial (USB) or Serial1/2/3 as needed.
// Mega has 2560 bytes RAM — keep buffers small.

// Variables to hold the parsed commands
double left_cmd = 0.0;
double right_cmd = 0.0;
double offload_cmd = 1.0;

// Variables to hold the feedback data
double left_pos = 0.0,  left_vel = 0.0;
double right_pos = 0.0, right_vel = 0.0;

// Timer for sending feedback
unsigned long last_send_time = 0;
const unsigned long SEND_INTERVAL_MS = 20; // Send at 50 Hz

void setup() {
  Serial.begin(115200);
  // Mega's USB serial doesn't need to wait for connection like ESP32-S3,
  // but a brief delay helps ensure the port is ready.
  delay(100);
  pinMode(13, OUTPUT);

  // Optional: Set up motor drivers and IMU here
  // pinMode(...)
  // imu.begin()
}

void loop() {
  // -------------------------------------------------------------------------
  // 1. Read 2 values from serial (left_cmd, right_cmd)
  // -------------------------------------------------------------------------
  if (Serial.available() > 0) {
    left_cmd = Serial.parseFloat();

    // Discard separators (comma, whitespace) until next number starts
    while (Serial.available() > 0 && Serial.peek() != '-' && Serial.peek() != '.' && !isDigit(Serial.peek())) {
      Serial.read();
    }

    right_cmd = Serial.parseFloat();

    while (Serial.available() > 0 && Serial.peek() != '-' && Serial.peek() != '.' && !isDigit(Serial.peek())) {
      Serial.read();
    }

    offload_cmd = Serial.parseFloat();

    // Clear rest of line
    while (Serial.available() > 0 && Serial.read() != '\n');

    // --- Apply commands to motors here ---
    // set_motor_speed(left_cmd, right_cmd);
  }


  // -------------------------------------------------------------------------
  // 2. Write 4 values to serial at a fixed rate
  // -------------------------------------------------------------------------
  if (millis() - last_send_time >= SEND_INTERVAL_MS) {
    last_send_time = millis();

    // --- Read sensors here ---
    // left_pos = read_encoder_left();
    // left_vel = calculate_velocity_left();

    // Simulated data
    left_pos += 0.01;
    right_pos += 0.01;

    Serial.print(left_pos, 3);
    Serial.print(",");
    Serial.print(left_vel, 3);
    Serial.print(",");
    Serial.print(right_pos, 3);
    Serial.print(",");
    Serial.println(right_vel, 3);
  
  }
}