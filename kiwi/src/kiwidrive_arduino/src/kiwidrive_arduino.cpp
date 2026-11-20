#include "kiwidrive_arduino/kiwidrive_arduino.h"

#include <cmath>
#include <exception>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{
std::string getParam(
  const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & def)
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? it->second : def;
}
}  // namespace

KiwiDriveArduino::KiwiDriveArduino()
: logger_(rclcpp::get_logger("KiwiDriveArduino"))
{}

CallbackReturn KiwiDriveArduino::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  if (info_.joints.size() != 3) {
    RCLCPP_FATAL(logger_, "Expected exactly 3 wheel joints in ros2_control.xacro, found %zu",
      info_.joints.size());
    return CallbackReturn::ERROR;
  }

  try {
    cfg_.loop_rate = std::stof(getParam(info_, "loop_rate", "30"));
    cfg_.device = getParam(info_, "device", "/dev/ttyUSB0");
    cfg_.baud_rate = std::stoi(getParam(info_, "baud_rate", "115200"));
    cfg_.timeout = std::stoi(getParam(info_, "timeout", "1000"));
    cfg_.enc_counts_per_rev = std::stoi(getParam(info_, "enc_counts_per_rev", "1920"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Bad <param> value in ros2_control.xacro: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (cfg_.enc_counts_per_rev <= 0 || cfg_.loop_rate <= 0) {
    RCLCPP_FATAL(logger_, "enc_counts_per_rev and loop_rate must be > 0");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < 3; ++i) {
    const auto & joint = info_.joints[i];
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' must have exactly one command interface: velocity",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    wheels_[i].setup(joint.name, cfg_.enc_counts_per_rev);
  }

  RCLCPP_INFO(logger_, "Finished configuration (wheels: %s, %s, %s)",
    wheels_[0].name.c_str(), wheels_[1].name.c_str(), wheels_[2].name.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn KiwiDriveArduino::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Opening %s @ %d baud (waiting for ESP32 to boot)...",
    cfg_.device.c_str(), cfg_.baud_rate);
  try {
    arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Could not open serial port %s: %s", cfg_.device.c_str(), e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn KiwiDriveArduino::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  arduino_.close();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KiwiDriveArduino::export_state_interfaces()
{
  // A velocity and a position state interface for every wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & w : wheels_) {
    state_interfaces.emplace_back(w.name, hardware_interface::HW_IF_VELOCITY, &w.vel);
    state_interfaces.emplace_back(w.name, hardware_interface::HW_IF_POSITION, &w.pos);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KiwiDriveArduino::export_command_interfaces()
{
  // A velocity command interface (rad/s) for every wheel
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & w : wheels_) {
    command_interfaces.emplace_back(w.name, hardware_interface::HW_IF_VELOCITY, &w.cmd);
  }
  return command_interfaces;
}

CallbackReturn KiwiDriveArduino::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting controller...");
  try {
    arduino_.resetEncoders();   // counts and PID back to zero so odometry starts clean
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial error during activation: %s", e.what());
    return CallbackReturn::ERROR;
  }
  for (auto & w : wheels_) {
    w.enc = 0;
    w.pos = 0;
    w.vel = 0;
    w.cmd = 0;
  }
  bad_reads_ = 0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn KiwiDriveArduino::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping controller...");
  try {
    if (arduino_.connected()) {
      arduino_.setMotorValues(0, 0, 0);
    }
  } catch (const std::exception &) {
  }
  return CallbackReturn::SUCCESS;
}

return_type KiwiDriveArduino::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!arduino_.connected()) {
    return return_type::ERROR;
  }

  int e1 = 0, e2 = 0, e3 = 0;
  bool ok = false;
  try {
    ok = arduino_.readEncoderValues(e1, e2, e3);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial read failed: %s", e.what());
    return return_type::ERROR;
  }

  if (!ok) {
    // One bad/empty line: keep the old values. Several in a row: the ESP32 is gone.
    if (++bad_reads_ >= 5) {
      RCLCPP_ERROR(logger_, "No valid encoder reply from the ESP32 (5 in a row)");
      return return_type::ERROR;
    }
    return return_type::OK;
  }
  bad_reads_ = 0;

  wheels_[0].enc = e1;
  wheels_[1].enc = e2;
  wheels_[2].enc = e3;

  const double dt = period.seconds();
  for (auto & w : wheels_) {
    const double pos_prev = w.pos;
    w.pos = w.calcEncAngle();
    if (dt > 0.0) {
      w.vel = (w.pos - pos_prev) / dt;
    }
  }

  return return_type::OK;
}

return_type KiwiDriveArduino::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!arduino_.connected()) {
    return return_type::ERROR;
  }

  // rad/s -> encoder ticks per PID frame (what the firmware's "m" command expects)
  int ticks[3];
  for (size_t i = 0; i < 3; ++i) {
    const double cmd = std::isfinite(wheels_[i].cmd) ? wheels_[i].cmd : 0.0;
    ticks[i] = static_cast<int>(std::lround(cmd / wheels_[i].rads_per_count / cfg_.loop_rate));
  }

  try {
    arduino_.setMotorValues(ticks[0], ticks[1], ticks[2]);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Serial write failed: %s", e.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  KiwiDriveArduino,
  hardware_interface::SystemInterface
)
