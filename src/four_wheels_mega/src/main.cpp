#include <Arduino.h>

// ---- Motor pins ----
const int IN1 = 3;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 6;
const int ENA = 9;
const int ENB = 10;

// ---- Command state ----
float left_vel = 0.0f;   // command from host (velocity setpoint)
float right_vel = 0.0f;  // command from host (velocity setpoint)

// ---- Feedback state ----
float left_position = 0.0f;
float right_position = 0.0f;

// Position integration (dummy for now; replace with real encoders later)
unsigned long last_time = 0;

// ---- IMU placeholders (replace with real sensor reads) ----
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

// ---- Feedback rate ----
const unsigned long FEEDBACK_PERIOD_MS = 20; // 50 Hz
unsigned long last_feedback_time = 0;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Do NOT block on !Serial. On the Mega, USB may not be ready
  // when setup() runs, and blocking here stalls the whole control loop.
  // while (!Serial) { }

  // Brief startup delay so the host can open the port without
  // catching a partial first line.
  delay(200);

  last_time = millis();
  last_feedback_time = millis();
}

// Read a "left,right\n" command from the host, if available.
// Uses a small fixed buffer (no String) to avoid heap churn.
void readCommand() {
  static char buf[64];
  static uint8_t idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n') {
      buf[idx] = '\0';
      idx = 0;

      // Parse "left,right"
      char *comma = strchr(buf, ',');
      if (comma != nullptr) {
        *comma = '\0';
        left_vel  = atof(buf);
        right_vel = atof(comma + 1);
      }
      continue;
    }

    if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      // Overflow: drop the partial line and resync on next '\n'.
      idx = 0;
    }
  }
}

// Apply the current velocity command to the motor drivers.
void applyMotors() {
  // Scale the command into PWM range (0-255).
  // Adjust the gain to match your motor driver / gearing.
  int pwm_left  = (int)constrain(left_vel  * 10.0f, 0.0f, 255.0f);
  int pwm_right = (int)constrain(right_vel * 10.0f, 0.0f, 255.0f);

  analogWrite(ENA, pwm_left);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, pwm_right);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Update the feedback state (positions, IMU). Replace with real sources.
void updateFeedback(unsigned long dt_ms) {
  // Dummy integration of position from commanded velocity.
  // With real encoders, replace this with encoder reads.
  float dt_s = dt_ms / 1000.0f;
  left_position  += left_vel  * dt_s;
  right_position += right_vel * dt_s;

  // IMU placeholders: fill these from your actual IMU driver.
  // orientation_x/y/z/w, angular_velocity_*, linear_acceleration_*
}

// Send ONE line with exactly 14 comma-separated fields, terminated by a
// single '\n'. Do NOT put a println() in the middle of the message.
void sendFeedback() {
  Serial.print(left_position, 3);            Serial.print(',');
  Serial.print(left_vel, 3);                 Serial.print(',');
  Serial.print(right_position, 3);           Serial.print(',');
  Serial.print(right_vel, 3);                Serial.print(',');

  Serial.print(orientation_x, 3);            Serial.print(',');
  Serial.print(orientation_y, 3);            Serial.print(',');
  Serial.print(orientation_z, 3);            Serial.print(',');
  Serial.print(orientation_w, 3);            Serial.print(',');

  Serial.print(angular_velocity_x, 3);       Serial.print(',');
  Serial.print(angular_velocity_y, 3);       Serial.print(',');
  Serial.print(angular_velocity_z, 3);       Serial.print(',');

  Serial.print(linear_acceleration_x, 3);    Serial.print(',');
  Serial.print(linear_acceleration_y, 3);    Serial.print(',');
  Serial.println(linear_acceleration_z, 3);  // <-- only ONE println, at the very end
}

void loop() {
  // 1) Non-blocking command read.
  readCommand();

  // 2) Apply motor outputs every loop.
  applyMotors();

  // 3) Update feedback and send at a fixed rate.
  unsigned long now = millis();
  if (now - last_feedback_time >= FEEDBACK_PERIOD_MS) {
    unsigned long dt = now - last_time;
    last_time = now;
    last_feedback_time = now;

    updateFeedback(dt);
    sendFeedback();
  }
}