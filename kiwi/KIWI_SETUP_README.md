# Kiwi drive: real-robot additions

Extract this zip ON TOP of your existing `kiwi/` workspace folder (merge/replace when asked).
Your Gazebo simulation keeps working: `use_sim` defaults to `true`.

## 1. What each file is and what it does

### Firmware: `firmware/ROS2_ESP32_BRIDGE/` (flash to the ESP32 with Arduino IDE)
| File | Purpose |
|---|---|
| `ROS2_ESP32_BRIDGE.ino` | Serial command parser + main loop. Now takes 3 arguments on `m` / `o`, returns 3 encoder counts on `e`. |
| `config.h` | ALL pins, PID gains, `AUTO_STOP_INTERVAL` (500 ms), encoder direction signs `ENC0/1/2_SIGN`. |
| `encoder.cpp/.h` | Quadrature decoding for 3 encoders (same decoding for all three). |
| `motor_driver.cpp/.h` | PWM + direction for 3 motors (L298N style, PWM on IN pins). |
| `pid.h` | 3 independent speed PID loops (one per wheel). |
| `commands.h` | Command letters + `LEFT=0, RIGHT=1, BACK=2` slot numbers. |

### ROS package: `src/kiwidrive_arduino/` (NEW; the 3-wheel version of your diffdrive_arduino)
| File | Purpose |
|---|---|
| `kiwidrive_arduino.cpp/.h` | The ros2_control hardware plugin: exports velocity command + velocity/position state for `wheel_1/2/3_joint`, converts rad/s -> ticks/frame, reads encoders, stops motors on deactivate. |
| `arduino_comms.cpp/.h` | Serial talking to the ESP32 (`e`, `m a b c`, `r`), waits for the ESP32 to boot after opening the port. |
| `wheel.cpp/.h` | Per-wheel state + encoder-count -> radians conversion. |
| `config.h` | Parameter struct (device, baud, loop rate, counts per rev). |
| `kiwidrive_arduino.xml` | Registers the plugin so ros2_control can find it. |
| `CMakeLists.txt`, `package.xml` | Build files. |

### ROS package: `src/serial/` (NEW to this workspace; your serial-humble, unchanged)
The serial-port library the hardware plugin links against.

### In `src/my_bot/` (your existing package)
| File | Status | Purpose |
|---|---|---|
| `description/ros2_control.xacro` | NEW | Real-robot `<ros2_control>` block (replaces Gazebo's): plugin, serial port, baud, loop rate, counts/rev, 3 joints. |
| `description/robot.urdf.xacro` | REPLACED | Adds `use_sim` switch: `true` -> gazebo_control.xacro, `false` -> ros2_control.xacro. |
| `config/kiwi_controllers_real.yaml` | NEW | Controller config for the real robot (30 Hz, no sim time). |
| `launch/launch_robot.launch.py` | NEW | Starts robot_state_publisher, ros2_control_node, both controllers and `kiwi_kinematics.py`. |
| `package.xml` | REPLACED | Your file plus the missing dependencies. |

Unchanged: `kiwi_kinematics.py`, `robot_core.xacro`, `gazebo_control.xacro`, `launch_sim`, `rsp`, `kiwi_controllers.yaml`, `CMakeLists.txt`.

## 2. Wiring (matches `config.h`)

| Wheel | Motor driver | Encoder A / B |
|---|---|---|
| wheel_1 (angle 30 deg) | L298N #1 IN3=GPIO26, IN4=GPIO27 | GPIO32 / GPIO33 |
| wheel_2 (angle 150 deg) | L298N #1 IN1=GPIO14, IN2=GPIO25 | GPIO21 / GPIO22 |
| wheel_3 (angle 270 deg) | L298N #2 IN1=GPIO18, IN2=GPIO19 | GPIO13 / GPIO4 |

* Keep the ENA/ENB jumpers ON. Encoders on 3.3 V. Common ground between ESP32, both drivers, battery.
* Motor wheel numbering must match the URDF: wheel_1 at (+x,+y), wheel_2 at (-x,+y), wheel_3 at (0,-y).
* Power the motors from a separate supply/rail with the ESP32 grounds joined; add decoupling capacitors.

## 3. Install and build

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
                 ros-humble-teleop-twist-keyboard
sudo usermod -aG dialout $USER        # then log out and back in (once)

cd ~/kiwi                              # your workspace root (the folder containing src/)
colcon build --symlink-install
source install/setup.bash
```

Flash the firmware: Arduino IDE -> open `firmware/ROS2_ESP32_BRIDGE/ROS2_ESP32_BRIDGE.ino`
-> board "ESP32 Dev Module" -> Upload.

## 4. Test the firmware BEFORE ROS (Arduino Serial Monitor: 115200, line ending = "Carriage Return")

Put the robot on blocks (wheels in the air).

1. `r`, then `o 100 0 0`, wait 1 s, `o 0 0 0`, then `e`. First number must be POSITIVE. Repeat with `o 0 100 0` (second number) and `o 0 0 100` (third number).
2. `o 100 100 100`: all three wheels must turn the same way and the robot would rotate counter-clockwise seen from above.
   - A wheel turns opposite to the others -> swap that motor's two wires.
3. Counts must rise for positive PWM. A count going down -> set that wheel's `ENCx_SIGN` to `-1` in `config.h`, re-flash.
4. `m 10 10 10`: speed should settle and stay steady. If a wheel races to full speed, its encoder sign is wrong (step 3).
5. `o 0 0 0` to stop.

## 5. Set the encoder counts per revolution

Use the same `enc_counts_per_rev` value you used on your differential-drive robot (same motors, same firmware decoding).
To verify: `r`, turn ONE wheel by hand exactly one full turn, `e`, read that wheel's number. That is the value
(for a quick sanity check, the diffdrive package default was 1920). Pass it at launch (below).

## 6. Run

```bash
ros2 launch my_bot launch_robot.launch.py serial_port:=/dev/ttyUSB0 enc_counts_per_rev:=1920 \
                                          wheel_radius:=0.04 base_radius:=0.17
```
`wheel_radius` and `base_radius` (centre -> wheel centre, metres) must be your REAL measured values.

Check:
```bash
ros2 control list_hardware_interfaces      # 3 velocity commands + 6 states, all [available]
ros2 control list_controllers              # both active
ros2 topic echo /joint_states
ros2 run teleop_twist_keyboard teleop_twist_keyboard    # robot driving
```
Quick manual wheel test (rad/s): `ros2 topic pub -r 5 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0, 2.0, 2.0]}"`

Conversion example: 0.2 m/s forward -> wheels (-2.5, -2.5, +5.0) rad/s -> at 1920 counts/rev and 30 Hz that is about (-25, -25, +51) ticks per frame sent to the ESP32.

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Could not open serial port` | Wrong `serial_port:=`, no `dialout` permission, or Arduino Serial Monitor still open. |
| Controllers fail to spawn | The ESP32 needs ~3 s to boot after the port opens; relaunch, and check the port name. |
| `No valid encoder reply` errors | Baud mismatch (firmware 115200 / xacro 115200), bad USB cable, or ESP32 brownout from motor current. |
| One wheel runs away at full speed under `m` | Its encoder counts backwards: flip `ENCx_SIGN` (section 4). |
| Robot drives wrong direction / rotates when it should translate | Wheel-to-joint mapping or motor polarity: redo section 4 step 1-2. |
| Odometry scale is off | Check `enc_counts_per_rev`, `wheel_radius`, `base_radius`. |
| Motors stop after 0.5 s | Intended safety stop: ROS stopped sending commands. |
