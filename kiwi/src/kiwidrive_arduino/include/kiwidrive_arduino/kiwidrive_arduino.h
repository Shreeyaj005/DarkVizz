#ifndef KIWIDRIVE_ARDUINO_KIWIDRIVE_ARDUINO_H
#define KIWIDRIVE_ARDUINO_KIWIDRIVE_ARDUINO_H

#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Three wheel joints, in the order they are listed in ros2_control.xacro:
//   joint #1 -> firmware slot 0 (LEFT),  joint #2 -> slot 1 (RIGHT),  joint #3 -> slot 2 (BACK)
class KiwiDriveArduino : public hardware_interface::SystemInterface
{
public:
  KiwiDriveArduino();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Config cfg_;
  ArduinoComms arduino_;

  std::array<Wheel, 3> wheels_;
  int bad_reads_ = 0;

  rclcpp::Logger logger_;
};

#endif // KIWIDRIVE_ARDUINO_KIWIDRIVE_ARDUINO_H
