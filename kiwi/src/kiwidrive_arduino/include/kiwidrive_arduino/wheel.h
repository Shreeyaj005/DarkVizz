#ifndef KIWIDRIVE_ARDUINO_WHEEL_H
#define KIWIDRIVE_ARDUINO_WHEEL_H

#include <string>

class Wheel
{
public:
  std::string name = "";
  int enc = 0;                 // latest encoder count
  double cmd = 0;              // commanded velocity (rad/s) from ros2_control
  double pos = 0;              // wheel angle (rad)
  double vel = 0;              // wheel velocity (rad/s)
  double rads_per_count = 0;

  Wheel() = default;
  Wheel(const std::string &wheel_name, int counts_per_rev);

  void setup(const std::string &wheel_name, int counts_per_rev);
  double calcEncAngle();
};

#endif // KIWIDRIVE_ARDUINO_WHEEL_H
