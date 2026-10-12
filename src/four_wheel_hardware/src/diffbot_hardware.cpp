#include "four_wheel_hardware/diffbot_hardware.hpp"
#include "four_wheel_hardware/actual_hardware_interface.hpp"
#include <cassert>
#include <chrono>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <iomanip>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <sstream>
#include <string>

// INFO: INIT
namespace four_wheel_hardware {
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {

  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  this->hw_start_sec_ = hardware_interface::stod(
      info_.hardware_parameters["hw_start_duration_sec"]);

  this->hw_stop_sec_ = hardware_interface::stod(
      info_.hardware_parameters["hw_stop_duration_sec"]);

  this->serial_port_ = info_.hardware_parameters["serial_port"];
  this->baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);

  // TODO:
  // add wheel specific code, like a struct or something
  //

  RCLCPP_INFO(get_logger(), "ON_INIT-------------------------------");

  this->arduino_ = std::make_unique<ArduinoInterface>();

  // grabs joints
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check number of state interfaces
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check first is pos and secod is velocity
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // specifically for IMU
  for (const hardware_interface::ComponentInfo &sensor : info_.sensors) {

    // checked number, make sure they are 10
    if (sensor.state_interfaces.size() != 10) {
      RCLCPP_FATAL(get_logger(), "Expected 10 entries for the IMU sensor");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check all 10 entries
    if (sensor.state_interfaces[0].name != "orientation.x") {
      RCLCPP_FATAL(get_logger(), " orientation.x expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (sensor.state_interfaces[1].name != "orientation.y") {
      RCLCPP_FATAL(get_logger(), " orientation.y expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (sensor.state_interfaces[2].name != "orientation.z") {
      RCLCPP_FATAL(get_logger(), "orientation.z expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (sensor.state_interfaces[3].name != "orientation.w") {
      RCLCPP_FATAL(get_logger(), "orientation.w expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (sensor.state_interfaces[4].name != "angular_velocity.x") {
      RCLCPP_FATAL(get_logger(), "angular_velocity.x expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (sensor.state_interfaces[5].name != "angular_velocity.y") {
      RCLCPP_FATAL(get_logger(), "angular_velocity.y expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (sensor.state_interfaces[6].name != "angular_velocity.z") {
      RCLCPP_FATAL(get_logger(), "angular_velocity.z expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (sensor.state_interfaces[7].name != "linear_acceleration.x") {
      RCLCPP_FATAL(get_logger(), "linear_acceleration.x expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (sensor.state_interfaces[8].name != "linear_acceleration.y") {
      RCLCPP_FATAL(get_logger(), "linear_acceleration.y expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (sensor.state_interfaces[9].name != "linear_acceleration.z") {
      RCLCPP_FATAL(get_logger(), "linear_acceleration.z expected.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // /// Name of the component.
    // std::string name;
    // /// Type of the component: sensor, joint, or GPIO.
    // std::string type;
    //
    // std::vector<InterfaceInfo> command_interfaces;
    // /**
    //  * Name of the state interfaces that can be read, e.g. "position",
    //  "velocity", etc.
    //  * Used by joints, sensors and GPIOs.
    //  */
  }

  // debug log for joints
  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    RCLCPP_INFO(get_logger(), "Joint name: %s", joint.name.c_str());

    RCLCPP_INFO(get_logger(), "Command interfaces:");
    for (const auto &cmd : joint.command_interfaces) {
      RCLCPP_INFO(get_logger(), "  - %s", cmd.name.c_str());
    }

    RCLCPP_INFO(get_logger(), "State interfaces:");
    for (const auto &state : joint.state_interfaces) {
      RCLCPP_INFO(get_logger(), "  - %s", state.name.c_str());
    }
  }
  // debug log for sensors
  for (const hardware_interface::ComponentInfo &sensor : info_.sensors) {
    RCLCPP_INFO(get_logger(), "Joint name: %s", sensor.name.c_str());

    RCLCPP_INFO(get_logger(), "State interfaces:");
    for (const auto &state : sensor.state_interfaces) {
      RCLCPP_INFO(get_logger(), "  - %s", state.name.c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// INFO: CONFIGURE
hardware_interface::CallbackReturn
DiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "ON "
                            "CONFIGURE-----------------------------------------"
                            "----------------------");

  for (const auto &[name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }

  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }

  // configure the sensor interfaces
  for (const auto &[name, descr] : sensor_state_interfaces_) {
    set_state(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// INFO: ACTIVATE
hardware_interface::CallbackReturn
DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(),
              "ON_ACTIVATE--------------------------------------------------");

  // for (int i = 0; i < this->hw_start_sec_; i++) {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(get_logger(), "%.1f sec left..", hw_start_sec_ - i);
  //   // TODO:
  //   // add serial initialization to either /tty/ACM0 or /tty/USB0
  // }

  if (!this->arduino_->connect_f(this->serial_port_, this->baudrate_)) {
    RCLCPP_ERROR(get_logger(), "FAILED TO CONNECT TO ARDUINO");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto &[name, descr] : joint_state_interfaces_) {
    set_state(name, get_state(name));
    RCLCPP_INFO(get_logger(), "State Interface ---> %s", name.c_str());
  }

  for (const auto &[name, descr] : sensor_state_interfaces_) {
    set_state(name, get_state(name));
    RCLCPP_INFO(get_logger(), "Sensor State Interface ---> %s", name.c_str());
  }

  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, get_state(name));
    RCLCPP_INFO(get_logger(), "Command Interface ---> %s", name.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// INFO: DEACTIVATE
hardware_interface::CallbackReturn
DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(
      get_logger(),
      "ON_DEACTIVATE----------------------------------------------------");

  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }

  // this->arduino_->disconnect_f();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// INFO: READ
hardware_interface::return_type
DiffBotSystemHardware::read(const rclcpp::Time &,
                            const rclcpp::Duration &period) {

  double left_pos, left_vel;
  double right_pos, right_vel;

  double orientation_x, orientation_y, orientation_z, orientation_w;
  double angular_velocity_x, angular_velocity_y, angular_velocity_z;
  double linear_acceleration_x, linear_acceleration_y, linear_acceleration_z;

  if (!this->arduino_->readFeedback_f(
          left_pos, left_vel, right_pos, right_vel, orientation_x,
          orientation_y, orientation_z, orientation_w, angular_velocity_x,
          angular_velocity_y, angular_velocity_z, linear_acceleration_x,
          linear_acceleration_y, linear_acceleration_z)) {

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "FAILED READING ARDUINO");

    return hardware_interface::return_type::OK;
  }

  set_state("back_left_wheel_joint/position", left_pos);
  set_state("back_left_wheel_joint/velocity", left_vel);

  set_state("back_right_wheel_joint/position", right_pos);
  set_state("back_right_wheel_joint/velocity", right_vel);

  set_state("imu_sensor/orientation.x", orientation_x);
  set_state("imu_sensor/orientation.y", orientation_y);
  set_state("imu_sensor/orientation.z", orientation_z);
  set_state("imu_sensor/orientation.w", orientation_y);

  set_state("imu_sensor/angular_velocity.x", angular_velocity_x);
  set_state("imu_sensor/angular_velocity.y", angular_velocity_y);
  set_state("imu_sensor/angular_velocity.z", angular_velocity_z);

  set_state("imu_sensor/linear_acceleration.x", linear_acceleration_x);
  set_state("imu_sensor/linear_acceleration.y", linear_acceleration_y);
  set_state("imu_sensor/linear_acceleration.z", linear_acceleration_z);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200, "UP");
  return hardware_interface::return_type::OK;
}

// INFO: WRITE
hardware_interface::return_type
DiffBotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {

  double left_cmd = get_command("back_left_wheel_joint/velocity");

  double right_cmd = get_command("back_right_wheel_joint/velocity");

  if (!arduino_->writeCommand_f(left_cmd, right_cmd)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Failed writing to Arduino");
  }

  return hardware_interface::return_type::OK;
}

} // namespace four_wheel_hardware
  //
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(four_wheel_hardware::DiffBotSystemHardware,
                       hardware_interface::SystemInterface)
