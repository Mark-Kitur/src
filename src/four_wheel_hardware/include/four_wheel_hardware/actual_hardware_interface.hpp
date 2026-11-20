#pragma once

#include <string>

namespace four_wheel_hardware {
class ArduinoInterface {
public:
  ArduinoInterface();
  ~ArduinoInterface();

  bool connect_f(const std::string &port, int baudrate);
  void disconnect_f();

  bool writeCommand_f(double, double, double);
  bool readFeedback_f(double &, double &, double &, double &);

private:
  int serial_fd_;
  bool readLine(std::string &line);
};

} // namespace four_wheel_hardware
