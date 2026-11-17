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
#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace four_wheel_hardware {

ArduinoInterface::ArduinoInterface() : serial_fd_(-1) {
  RCLCPP_INFO(
      rclcpp::get_logger("logger"),
      "---------------------------------------ESP32---WAKE------------------"
      "-----------");
}

ArduinoInterface::~ArduinoInterface() {
  RCLCPP_INFO(
      rclcpp::get_logger("logger"),
      "---------------------------------------ESP32---DIE------------------"
      "-----------");
}

bool ArduinoInterface::connect_f(const std::string &port, int baudrate) {
  // [ESP32-S3] O_NONBLOCK is set only during the open/config phase so that
  // the DTR/RTS ioctl dance and the warm-up drain can never hang the node
  // if the CDC stack isn't ready. We restore blocking mode right before
  // returning, so readLine()/writeCommand_f() keep their original semantics.
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("esp32"), "open(%s) failed: %s",
                 port.c_str(), std::strerror(errno));
    return false;
  }

  struct termios tty{};

  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("esp32"), "tcgetattr failed: %s",
                 std::strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Map the requested baudrate to a termios speed constant.
  // [ESP32-S3] NOTE: For native USB CDC this is effectively cosmetic —
  // the USB link negotiates its own speed (12 Mbps full-speed). We keep
  // the call for compatibility with real UART bridges (CP2102/CH340).
  // --- Baud ---
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // --- 8N1, no flow control ---
  tty.c_cflag &= ~PARENB; // no parity
  tty.c_cflag &= ~CSTOPB; // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;      // 8 data bits
  tty.c_cflag &= ~CRTSCTS; // no HW flow control (CDC doesn't support it anyway)
  tty.c_cflag |= CREAD | CLOCAL;
  // [ESP32-S3] We deliberately do NOT clear HUPCL here. For USB CDC the
  // important thing is that DTR/RTS are actively asserted below; HUPCL
  // behavior on close is harmless for the S3 CDC stack and matches the
  // kernel default. (For the Mega you needed ~HUPCL to avoid auto-reset.)

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

  // --- Read behavior: blocking per byte (VTIME unused, we time out in SW) ---
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("esp32"), "tcsetattr failed: %s",
                 std::strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // [ESP32-S3] *** CRITICAL ***
  // The ESP32-S3 native USB CDC only starts forwarding data to the host
  // after the host asserts DTR (the CDC SET_CONTROL_LINE_STATE request).
  // Without this ioctl the link opens fine but never delivers bytes —
  // the classic "silent serial port" symptom on S3 boards.
  {
    int modem_bits = TIOCM_DTR | TIOCM_RTS;
    if (ioctl(serial_fd_, TIOCMBIS, &modem_bits) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("esp32"),
                  "ioctl(TIOCMBIS DTR|RTS) failed: %s — CDC may not send",
                  std::strerror(errno));
      // Not fatal: some USB-UART bridges don't implement this and still work.
    } else {
      RCLCPP_INFO(rclcpp::get_logger("arduino"), "asserted DTR+RTS on %s",
                  port.c_str());
    }
  }

  // Flush any stale bytes left over from a previous session or a reset.
  tcflush(serial_fd_, TCIOFLUSH);

  // [ESP32-S3] Warm-up window: extended from 300 ms to 800 ms because the
  // S3 boot banner / CDC enumeration produces more pre-data noise than the
  // Mega. We keep discarding until we see the first newline boundary.
  {
    char buf[256];
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start)
               .count() < 800) {
      ssize_t n = read(serial_fd_, buf, sizeof(buf));
      if (n > 0 && std::memchr(buf, '\n', n) != nullptr)
        break;
      // n == -1 with EAGAIN is normal in O_NONBLOCK mode — just wait.
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // [ESP32-S3] Restore blocking mode for the hot path. readLine() and
  // writeCommand_f() rely on blocking read()/write() semantics + their
  // own software timeouts.
  {
    int flags = fcntl(serial_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(serial_fd_, F_SETFL, flags & ~O_NONBLOCK) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("esp32"),
                   "failed to clear O_NONBLOCK: %s", std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
  }

  // Debug: print effective termios settings once.
  {
    struct termios check{};
    tcgetattr(serial_fd_, &check);
    RCLCPP_INFO(rclcpp::get_logger("esp32"),
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
    // [ESP32-S3] Lower DTR/RTS cleanly before close so the S3 doesn't see
    // a hard drop (which can trigger a CDC reset on some Arduino cores).
    // We use TIOCMBIC (clear bits) with the same mask used at open.
    int modem_bits = TIOCM_DTR | TIOCM_RTS;
    ioctl(serial_fd_, TIOCMBIC, &modem_bits);

    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ArduinoInterface::readLine(std::string &line) {
  line.clear();

  // [ESP32-S3] Timeout bumped from 25 ms to 120 ms.
  // Over the Mega's UART, a byte arrives every ~87 µs and 25 ms was ample.
  // Over USB CDC, the kernel delivers data in 1 ms bulk frames, but the
  // ESP32-S3 firmware often emits one field per Serial.print() call, so the
  // host sees several small transfers with USB-scheduling jitter of
  // 10–30 ms between them. 120 ms comfortably absorbs that without
  // stalling the control loop for the old 500 ms we previously avoided.
  constexpr int kLineTimeoutMs = 120;

  auto last_byte_time = std::chrono::steady_clock::now();

  char c;

  while (true) {
    ssize_t n = read(serial_fd_, &c, 1);

    if (n < 0) {
      if (errno == EINTR)
        continue;
      // [ESP32-S3] On USB CDC, a transient EAGAIN/ENODEV during a re-enum
      // can happen if the cable is wiggled. Treat as timeout instead of
      // a hard failure so the caller retries instead of crashing.
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        continue;
      return false;
    }

    if (n == 0) {
      // read() returned 0 — either a timeout or (on some kernels) a spurious
      // zero-byte read from the CDC stack. Decide whether to keep waiting.
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now - last_byte_time)
                         .count();

      if (elapsed >= kLineTimeoutMs) {
        // If we have partial data and the timeout expired, drain the rest
        // of this partial line so the next call starts on a fresh boundary.
        // [ESP32-S3] Drain in a non-blocking way to avoid a second hang.
        if (!line.empty()) {
          int old_flags = fcntl(serial_fd_, F_GETFL, 0);
          fcntl(serial_fd_, F_SETFL, old_flags | O_NONBLOCK);
          char drain;
          while (read(serial_fd_, &drain, 1) == 1) {
            if (drain == '\n')
              break;
          }
          fcntl(serial_fd_, F_SETFL, old_flags);
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
    // [ESP32-S3] Downgraded from ERROR to WARN. Timeouts over USB CDC are
    // expected occasionally (cable noise, re-enumeration) and shouldn't
    // spam the log at ERROR level every control cycle.
    RCLCPP_WARN(rclcpp::get_logger("esp32"), "readLine() failed");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("esp32"), "Received: '%s'", line.c_str());

  int parsed = std::sscanf(
      line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
      &left_pos, &left_vel, &right_pos, &right_vel,

      &orientation_x, &orientation_y, &orientation_z, &orientation_w,

      &angular_velocity_x, &angular_velocity_y, &angular_velocity_z,

      &linear_acceleration_x, &linear_acceleration_y, &linear_acceleration_z);

  if (parsed != kExpectedFields) {
    RCLCPP_WARN(rclcpp::get_logger("esp32"),
                "Parsed %d/%d values from '%s' — resyncing", parsed,
                kExpectedFields, line.c_str());

    // Drain to the next newline so the next readLine() starts aligned.
    // [ESP32-S3] Same non-blocking drain as in readLine() to guarantee we
    // don't stall the control loop if the ESP32 is mid-transmission.
    int old_flags = fcntl(serial_fd_, F_GETFL, 0);
    fcntl(serial_fd_, F_SETFL, old_flags | O_NONBLOCK);
    char c;
    while (true) {
      ssize_t n = read(serial_fd_, &c, 1);
      if (n <= 0)
        break;
      if (c == '\n')
        break;
    }
    fcntl(serial_fd_, F_SETFL, old_flags);
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
    RCLCPP_ERROR(rclcpp::get_logger("esp32"),
                 "writeCommand_f() wrote %zd of %zu bytes", n, data.size());
    return false;
  }

  // [ESP32-S3] tcdrain() on a USB CDC tty returns immediately — there is no
  // hardware TX FIFO to wait for. It's harmless, but don't depend on it for
  // timing. If you ever need strict end-to-end delivery guarantees, add a
  // reply/ACK in the protocol instead.
  tcdrain(serial_fd_);

  return true;
}

} // namespace four_wheel_hardware
