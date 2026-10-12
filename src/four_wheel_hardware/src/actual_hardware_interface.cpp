#include <cstdio>
#include <four_wheel_hardware/actual_hardware_interface.hpp>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
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

  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("arduino"), "open(%s) failed: %s",
                 port.c_str(), std::strerror(errno));
    return false;
  }

  struct termios tty{};

  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("arduino"), "tcgetattr failed: %s",
                 std::strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Map the requested baudrate to a termios speed constant.
  // --- Baud ---
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // --- 8N1, no flow control ---
  tty.c_cflag &= ~PARENB; // no parity
  tty.c_cflag &= ~CSTOPB; // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;      // 8 data bits
  tty.c_cflag &= ~CRTSCTS; // no HW flow control
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_cflag &= ~HUPCL; // <-- don't drop DTR on close (prevents Mega reset)

  // --- Raw input: clear ALL processing, not just IXON/IXOFF/IXANY ---
  tty.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL | IXON | IXOFF | IXANY | IMAXBEL);
#ifdef IUCLC
  tty.c_iflag &= ~IUCLC;
#endif

  // --- Raw output ---
  tty.c_oflag &= ~OPOST;
#ifdef ONLCR
  tty.c_oflag &= ~ONLCR;
#endif

  // --- Raw local mode ---
  tty.c_lflag &= ~(ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHONL | IEXTEN);
#ifdef XCASE
  tty.c_lflag &= ~XCASE;
#endif

  // --- Read behavior: blocking per byte, so VTIME isn't the bottleneck ---
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("arduino"), "tcsetattr failed: %s",
                 std::strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Flush any stale bytes left over from a previous session or a reset.
  tcflush(serial_fd_, TCIOFLUSH);

  // Warm-up: discard a possibly partial line that was already in flight.
  // This prevents the reader from locking onto a mid-line boundary.
  {
    char buf[256];
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start)
               .count() < 300) {
      ssize_t n = read(serial_fd_, buf, sizeof(buf));
      if (n > 0 && std::memchr(buf, '\n', n) != nullptr)
        break;
    }
  }

  // Debug: print effective termios settings once.
  {
    struct termios check{};
    tcgetattr(serial_fd_, &check);
    RCLCPP_INFO(rclcpp::get_logger("arduino"),
                "port=%s baud=%d ispeed=%d ospeed=%d cflag=0x%x iflag=0x%x "
                "lflag=0x%x oflag=0x%x",
                port.c_str(), baudrate, (int)cfgetispeed(&check),
                (int)cfgetospeed(&check), (unsigned)check.c_cflag,
                (unsigned)check.c_iflag, (unsigned)check.c_lflag,
                (unsigned)check.c_oflag);
  }

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

  // Wall-clock timeout for a complete line.
  // At 115200 baud a ~100-char line takes ~9 ms, so 25 ms is plenty.
  // The OLD value of 500 ms caused every failed read to stall the
  // control loop for half a second → overruns → RX buffer starvation.
  constexpr int kLineTimeoutMs = 25;

  auto last_byte_time = std::chrono::steady_clock::now();

  char c;

  while (true) {
    ssize_t n = read(serial_fd_, &c, 1);

    if (n < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }

    if (n == 0) {
      // read() timed out. Decide whether to keep waiting.
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now - last_byte_time)
                         .count();

      if (elapsed >= kLineTimeoutMs) {
        // If we have partial data and the timeout expired, drain the rest
        // of this partial line so the next call starts on a fresh boundary.
        if (!line.empty()) {
          char drain;
          while (read(serial_fd_, &drain, 1) == 1 && drain != '\n') {
            // discard
          }
        }
        return false;
      }
      continue;
    }

    // Reset the clock whenever we actually receive a byte.
    last_byte_time = std::chrono::steady_clock::now();

    if (c == '\r')
      continue;

    if (c == '\n')
      return true;

    line += c;

    // Safety cap.
    if (line.size() > 4096)
      return false;
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

  constexpr int kExpectedFields = 14;

  std::string line;

  if (!readLine(line)) {
    RCLCPP_ERROR(rclcpp::get_logger("arduino"), "readLine() failed");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("arduino"), "Received: '%s'", line.c_str());

  int parsed = std::sscanf(
      line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
      &left_pos, &left_vel, &right_pos, &right_vel,

      &orientation_x, &orientation_y, &orientation_z, &orientation_w,

      &angular_velocity_x, &angular_velocity_y, &angular_velocity_z,

      &linear_acceleration_x, &linear_acceleration_y, &linear_acceleration_z);

  if (parsed != kExpectedFields) {
    RCLCPP_WARN(rclcpp::get_logger("arduino"),
                "Parsed %d/%d values from '%s' — resyncing", parsed,
                kExpectedFields, line.c_str());

    // Drain to the next newline so the next readLine() starts aligned.
    // (readLine() already did this on timeout, but a successful read with
    // a bad parse needs the same treatment.)
    char c;
    while (true) {
      ssize_t n = read(serial_fd_, &c, 1);
      if (n <= 0)
        break;
      if (c == '\n')
        break;
    }
    return false;
  }

  return true;
}

bool ArduinoInterface::writeCommand_f(double left, double right) {
  if (serial_fd_ < 0)
    return false;

  std::ostringstream msg;
  msg << left << "," << right << "\n";
  const std::string data = msg.str();

  ssize_t n = write(serial_fd_, data.c_str(), data.size());

  if (n != static_cast<ssize_t>(data.size())) {
    RCLCPP_ERROR(rclcpp::get_logger("arduino"),
                 "writeCommand_f() wrote %zd of %zu bytes", n, data.size());
    return false;
  }

  // Make sure the bytes actually leave the kernel buffer.
  tcdrain(serial_fd_);

  return true;
}

} // namespace four_wheel_hardware
