# Without-EMG

Serial-controlled firmware for the four-finger hand. No sensor is needed, so this is the simplest way to check that the motors, drivers and wiring work. EMG control lives in `EMG-Control/Dry-EMG-Control/`.

This folder is a PlatformIO project. Open it as the root folder in VS Code, as described in the main README.

## Contents

| Path | What it does |
|---|---|
| `src/Four-fingers.cpp` | Controls all four fingers from the serial monitor. This is the default build. |
| `hardware-checks/Finger-tester.cpp` | Drives one motor with `o` (open), `c` (close) and `x` (stop). |
| `hardware-checks/HBridge-tester.cpp` | Ramps one motor forward and then in reverse through an L298N, in a loop. No input needed. |

The sketches in `hardware-checks/` are not part of the default build. Each has its own environment.

## Environments

| Environment | Sketch |
|---|---|
| `arduino_nano_esp32` | `Four-fingers` (default) |
| `finger_tester` | `Finger-tester` |
| `HBridge_tester` | `HBridge-tester` |

Pick one in PlatformIO > Project Tasks, or from the terminal:

    pio run -e finger_tester -t upload

## Four-fingers

Open the serial monitor at 115200 baud. Commands are single characters.

| Key | Action |
|---|---|
| `1` `3` `5` `7` | Open finger 1 to 4 |
| `2` `4` `6` `8` | Close finger 1 to 4 |
| `0` | Stop all |

Motors run at a fixed PWM of 150 (`testSpeed`).

Finger pins (IN1, IN2, ENA):

| Finger | IN1 | IN2 | ENA |
|---|---|---|---|
| 1 | 2 | 3 | 10 |
| 2 | 4 | 5 | 11 |
| 3 | 6 | 7 | 12 |
| 4 | 8 | 9 | 13 |

## Known issues

- `Finger-tester` sets `testSpeed = 700`, which is above the 0 to 255 PWM range. Set it to 255 or lower.