#ifndef KIWIDRIVE_ARDUINO_CONFIG_H
#define KIWIDRIVE_ARDUINO_CONFIG_H

#include <string>

// Filled from the <hardware><param> entries in description/ros2_control.xacro
struct Config
{
  float loop_rate = 30;                 // must equal PID_RATE in the ESP32 firmware
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 115200;               // must equal BAUDRATE in the ESP32 firmware
  int timeout = 1000;                   // serial read timeout (ms)
  int enc_counts_per_rev = 1920;        // encoder counts per WHEEL revolution (4x quadrature * gear ratio)
};

#endif // KIWIDRIVE_ARDUINO_CONFIG_H
