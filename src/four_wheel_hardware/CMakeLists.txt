cmake_minimum_required(VERSION 3.8)
project(four_wheel_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

#four wheel ROS hardware
add_library(
	four_wheel_hardware	
	SHARED
	src/diffbot_hardware.cpp
)

target_include_directories(
	four_wheel_hardware	
	PUBLIC
	include
)

#arduino hardware
add_library(
	actual_hardware_interface
	SHARED
	src/actual_hardware_interface.cpp
)


target_include_directories(
	actual_hardware_interface
	PUBLIC
	include
)

ament_target_dependencies(
	actual_hardware_interface
	rclcpp
)

target_link_libraries(
	four_wheel_hardware
	actual_hardware_interface
)

ament_target_dependencies(
	four_wheel_hardware
	rclcpp
	hardware_interface
	pluginlib
	rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
	hardware_interface
	plugin.xml
)

install(
	TARGETS
	four_wheel_hardware
	LIBRARY
	DESTINATION
	lib
)

install(
	DIRECTORY
	include/
	DESTINATION
	include
)

install(
	FILES
	plugin.xml
	DESTINATION
	share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
