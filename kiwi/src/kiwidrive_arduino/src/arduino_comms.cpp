#include "kiwidrive_arduino/arduino_comms.h"

#include <chrono>
#include <sstream>
#include <thread>

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  serial_conn_.setPort(serial_device);
  serial_conn_.setBaudrate(baud_rate);
  serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
  serial_conn_.setTimeout(tt);
  serial_conn_.open();

  // Opening the port resets most ESP32 boards (DTR/RTS). Wait for the boot to finish,
  // then throw away the boot-log garbage so the first reply we read is a real one.
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  serial_conn_.flushInput();
}

void ArduinoComms::close()
{
  if (serial_conn_.isOpen())
  {
    serial_conn_.close();
  }
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send)
{
  serial_conn_.write(msg_to_send);
  return serial_conn_.readline();
}

bool ArduinoComms::readEncoderValues(int &val_1, int &val_2, int &val_3)
{
  std::string response = sendMsg("e\r");

  std::istringstream iss(response);
  long a, b, c;
  if (!(iss >> a >> b >> c))
  {
    return false;   // timeout or garbage line - caller keeps the previous values
  }
  val_1 = static_cast<int>(a);
  val_2 = static_cast<int>(b);
  val_3 = static_cast<int>(c);
  return true;
}

void ArduinoComms::setMotorValues(int val_1, int val_2, int val_3)
{
  std::stringstream ss;
  ss << "m " << val_1 << " " << val_2 << " " << val_3 << "\r";
  sendMsg(ss.str());
}

void ArduinoComms::resetEncoders()
{
  sendMsg("r\r");
}
