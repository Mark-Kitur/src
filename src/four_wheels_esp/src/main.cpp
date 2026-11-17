#include <Arduino.h>

// ===========================================================================
// ESP32-S3 Firmware for ROS 2 four_wheel_hardware
// 
// Protocol:
//   RX (from host): "left_cmd,right_cmd\n"          (2 doubles)
//   TX (to host):   "l_pos,l_vel,r_pos,r_vel,       (14 doubles)
//                    ox,oy,oz,ow,
//                    wx,wy,wz,
//                    ax,ay,az\n"
// ===========================================================================

// Use Serial (USB CDC) so the host sees /dev/ttyACM0
// Make sure ARDUINO_USB_CDC_ON_BOOT=1 is set in your build flags!

// Variables to hold the parsed commands
double left_cmd = 0.0;
double right_cmd = 0.0;

// Variables to hold the feedback data (initialize to 0.0)
double left_pos = 0.0,  left_vel = 0.0;
double right_pos = 0.0, right_vel = 0.0;

double orientation_x = 0.0, orientation_y = 0.0, orientation_z = 0.0, orientation_w = 1.0;
double angular_velocity_x = 0.0, angular_velocity_y = 0.0, angular_velocity_z = 0.0;
double linear_acceleration_x = 0.0, linear_acceleration_y = 0.0, linear_acceleration_z = 0.0;

// Timer for sending feedback
unsigned long last_send_time = 0;
const unsigned long SEND_INTERVAL_MS = 20; // Send at 50 Hz

void setup() {
  // Initialize USB Serial (wait up to 2 seconds for host to connect)
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2000)) {
    delay(10);
  }

  // Optional: Set up your motor drivers and IMU here
  // pinMode(...)
  // imu.begin()
}

void loop() {
  // -------------------------------------------------------------------------
  // 1. Read 2 values from serial (left_cmd, right_cmd)
  // -------------------------------------------------------------------------
  // We check if data is available, and use parseFloat() which reads until 
  // it hits a non-numeric character (like a comma or newline).
  if (Serial.available() > 0) {
    // Read the first float
    left_cmd = Serial.parseFloat();
    
    // The stream is now positioned right after the first number. 
    // Usually it stops at the comma. We should read the comma out of the way.
    while (Serial.available() > 0 && Serial.peek() != '-' && Serial.peek() != '.' && !isDigit(Serial.peek())) {
      Serial.read(); // discard comma, spaces, etc.
    }

    // Read the second float
    right_cmd = Serial.parseFloat();

    // Read until newline to clear the rest of the buffer
    while (Serial.available() > 0 && Serial.read() != '\n');

    // --- Apply commands to your motors here ---
    // set_motor_speed(left_cmd, right_cmd);
  }

  // -------------------------------------------------------------------------
  // 2. Write 14 values to serial at a fixed rate
  // -------------------------------------------------------------------------
  if (millis() - last_send_time >= SEND_INTERVAL_MS) {
    last_send_time = millis();

    // --- Read your sensors here to update variables ---
    // left_pos = read_encoder_left();
    // left_vel = calculate_velocity_left();
    // imu.read(...); // etc.
    
    // In this demo, we just make up some simulated data to show it works
    left_pos += 0.01;
    right_pos += 0.01;

    // Print exactly 14 values, comma-separated, ending with a newline
    // Using %.6f gives 6 decimal places (very common for ROS control)
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "%.6f,%.6f,%.6f,%.6f,"   // 4 wheel values
             "%.6f,%.6f,%.6f,%.6f,"   // 4 orientation values
             "%.6f,%.6f,%.6f,"        // 3 angular velocity values
             "%.6f,%.6f,%.6f\n",      // 3 linear acceleration values
             
             left_pos, left_vel, right_pos, right_vel,
             orientation_x, orientation_y, orientation_z, orientation_w,
             angular_velocity_x, angular_velocity_y, angular_velocity_z,
             linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

    Serial.print(buffer);
  }
}