# ESP32 3-motor closed-loop bridge (kiwi drive)

Open `ROS2_ESP32_BRIDGE.ino` in Arduino IDE (ESP32 Arduino core 3.x, board "ESP32 Dev Module").
The folder name matches the .ino name, so the IDE opens it without asking to move files.

## Pins (all in `config.h`)

| Wheel (ROS joint) | Firmware slot | Motor driver pins | Encoder A / B |
|---|---|---|---|
| wheel_1_joint | 0 / LEFT  | L298N #1 IN3 = GPIO 26, IN4 = GPIO 27 | GPIO 32 / 33 |
| wheel_2_joint | 1 / RIGHT | L298N #1 IN1 = GPIO 14, IN2 = GPIO 25 | GPIO 21 / 22 |
| wheel_3_joint | 2 / BACK  | L298N #2 IN1 = GPIO 18, IN2 = GPIO 19 | GPIO 13 / 4  |

* Leave the ENA/ENB jumpers ON the L298N boards: PWM is applied directly on the IN pins.
* All encoder pins have internal pull-ups, so no external resistors are needed.
* Power the encoders from 3.3 V (ESP32 inputs are not 5 V tolerant). Common GND everywhere.

## Serial commands (115200 baud, end every command with `\r`)

| Command | Meaning |
|---|---|
| `e` | read encoders -> `w1 w2 w3` |
| `r` | reset encoders + PID -> `OK` |
| `m a b c` | closed-loop speed, ticks per PID frame (PID_RATE = 30 Hz) |
| `o a b c` | raw PWM -255..255, no PID |
| `u kp:kd:ki:ko` | change PID (only with IDENTICAL_PID_GAINS) |
| `b`, `a`, `d`, `w`, `x`, `c` | baud / analog / digital GPIO helpers from the original code |

Safety: all motors stop if no `m`/`o` arrives for `AUTO_STOP_INTERVAL` (500 ms).

## Direction rules (kiwi wheels are mounted identically)

1. Positive PWM must turn every wheel the same way: `o 100 100 100` makes the whole robot rotate counter-clockwise
   seen from above. Wheel turns the wrong way -> swap that motor's two wires.
2. Positive PWM must give positive encoder counts. Count goes down -> set that wheel's `ENCx_SIGN` to `-1` in `config.h`.
