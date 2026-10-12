#pragma once

#include <string>

namespace four_wheel_hardware {
class ArduinoInterface {
public:
  ArduinoInterface();
  ~ArduinoInterface();

  bool connect_f(const std::string &port, int baudrate);
  void disconnect_f();

  bool writeCommand_f(double left, double right);
  bool readFeedback_f(double &left_pos, double &left_vel, double &right_pos,
                      double &right_vel,

                      double &orientation_x, double &orientation_y,
                      double &orientation_z, double &orientation_w,

                      double &angular_velocity_x, double &angular_velocity_y,
                      double &angular_velocity_z,

                      double &linear_acceleration_x,
                      double &linear_acceleration_y,
                      double &linear_acceleration_z

  );

private:
  int serial_fd_;
  bool readLine(std::string &line);
};

} // namespace four_wheel_hardware
