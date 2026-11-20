cmake_minimum_required(VERSION 3.5)
project(kiwidrive_arduino)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)

add_library(kiwidrive_arduino SHARED
  src/kiwidrive_arduino.cpp
  src/wheel.cpp
  src/arduino_comms.cpp
)

target_include_directories(kiwidrive_arduino PRIVATE include)

ament_target_dependencies(
  kiwidrive_arduino
  hardware_interface
  controller_manager
  serial
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface kiwidrive_arduino.xml)

install(TARGETS kiwidrive_arduino DESTINATION lib)

ament_export_libraries(kiwidrive_arduino)
ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

ament_package()
