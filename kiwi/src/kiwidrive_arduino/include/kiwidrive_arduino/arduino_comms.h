#ifndef KIWIDRIVE_ARDUINO_ARDUINO_COMMS_H
#define KIWIDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <serial/serial.h>
#include <cstdint>
#include <string>

// Talks to the ESP32 firmware (ROS2_ESP32_BRIDGE) over serial.
// Every command ends with '\r' and every command gets exactly one reply line.
class ArduinoComms
{
public:
  ArduinoComms() = default;

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void close();
  bool connected() const { return serial_conn_.isOpen(); }

  bool readEncoderValues(int &val_1, int &val_2, int &val_3);   // "e"  -> "<w1> <w2> <w3>"
  void setMotorValues(int val_1, int val_2, int val_3);         // "m <w1> <w2> <w3>" (ticks per PID frame)
  void resetEncoders();                                         // "r"  -> resets counts and PID

  std::string sendMsg(const std::string &msg_to_send);

private:
  serial::Serial serial_conn_;
};

#endif // KIWIDRIVE_ARDUINO_ARDUINO_COMMS_H
