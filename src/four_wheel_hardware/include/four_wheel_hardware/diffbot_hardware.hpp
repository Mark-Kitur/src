#pragma once
#include "four_wheel_hardware/actual_hardware_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <memory>
#include <rclcpp/context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace four_wheel_hardware {
class DiffBotSystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;

  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

private:
  std::unique_ptr<ArduinoInterface> arduino_;

  std::string serial_port_;
  int baudrate_;

  double hw_start_sec_, hw_stop_sec_;
};

} // namespace four_wheel_hardware
