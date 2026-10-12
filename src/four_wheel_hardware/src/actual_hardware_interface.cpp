#include <cstdio>
#include <four_wheel_hardware/actual_hardware_interface.hpp>

#include <fcntl.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace four_wheel_hardware {

ArduinoInterface::ArduinoInterface() : serial_fd_(-1) {
  RCLCPP_INFO(
      rclcpp::get_logger("logger"),
      "---------------------------------------ARDUINO---WAKE------------------"
      "-----------");
}
ArduinoInterface::~ArduinoInterface() {

  RCLCPP_INFO(
      rclcpp::get_logger("logger"),
      "---------------------------------------ARDUINO---DIE------------------"
      "-----------");
}

bool ArduinoInterface::connect_f(const std::string &port, int baudrate) {
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);

  if (serial_fd_ < 0)
    return false;

  struct termios tty{};

  if (tcgetattr(serial_fd_, &tty) != 0)
    return false;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag &= ~PARENB; // No parity
  tty.c_cflag &= ~CSTOPB; // One stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8; // 8 data bits

  tty.c_cflag &= ~CRTSCTS; // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control

  tty.c_lflag = 0; // Raw input
  tty.c_oflag = 0; // Raw output

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(serial_fd_, TCSANOW, &tty))
    return false;

  return true;
}

void ArduinoInterface::disconnect_f() {
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ArduinoInterface::readLine(std::string &line) {
  line.clear();

  char c;

  while (true) {
    ssize_t n = read(serial_fd_, &c, 1);

    if (n < 0)
      return false;

    if (n == 0) {
      // No data yet.
      if (line.empty())
        return false;

      // We've started a line, keep waiting.
      continue;
    }

    if (c == '\r')
      continue;

    if (c == '\n')
      return true;

    line += c;
  }
}
bool ArduinoInterface::readFeedback_f(
    double &left_pos, double &left_vel, double &right_pos, double &right_vel,

    double &orientation_x, double &orientation_y, double &orientation_z,
    double &orientation_w,

    double &angular_velocity_x, double &angular_velocity_y,
    double &angular_velocity_z,

    double &linear_acceleration_x, double &linear_acceleration_y,
    double &linear_acceleration_z) {

  std::string line;

  if (!readLine(line)) {
    RCLCPP_ERROR(rclcpp::get_logger("arduino"), "readLine() failed");

    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("arduino"), "Received: '%s'", line.c_str());

  int parsed = std::sscanf(
      line.c_str(), "%lf,%lf,%lf,%lf, %lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
      &left_pos, &left_vel, &right_pos, &right_vel,

      &orientation_x, &orientation_y, &orientation_z, &orientation_w,

      &angular_velocity_x, &angular_velocity_y, &angular_velocity_z,

      &linear_acceleration_x, &linear_acceleration_y, &linear_acceleration_z);

  RCLCPP_INFO(rclcpp::get_logger("arduino"), "Parsed %d values", parsed);

  return parsed == 4;
}

bool ArduinoInterface::writeCommand_f(double left, double right) {
  std::ostringstream msg;

  msg << left << "," << right << "\n";

  const std::string data = msg.str();

  ssize_t n = write(serial_fd_, data.c_str(), data.size());

  return n == static_cast<ssize_t>(data.size());
}
} // namespace four_wheel_hardware
