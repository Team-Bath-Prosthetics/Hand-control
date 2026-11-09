# EMG-Control archive (MyoWare)

Earlier firmware built around the MyoWare 2.0 muscle sensor. It is kept for reference only. The current firmware is in `EMG-Control/Dry-EMG-Control/`.

## What MyoWare is

The MyoWare 2.0 (Advancer Technologies) is an analog surface EMG sensor. It measures the electrical activity of a muscle through electrodes on the skin and outputs a voltage that rises as the muscle flexes harder.

- **Electrodes:** bipolar, with two snap-on electrodes over the muscle and a third reference electrode on a neutral spot.
- **Outputs:** `ENV` (smoothed envelope, gain set by an onboard potentiometer), `RECT` (rectified signal) and `RAW` (amplified raw EMG centred on half the supply voltage).
- **Band:** first-order pass band of about 20 to 500 Hz.
- **Supply:** 2.27 V to 5.47 V.

These sketches read the `ENV` pin on A0, so the envelope was already computed in hardware and `analogRead()` returned muscle activity directly.

## Why it is not used

- **Different sensor:** the current firmware uses two DFRobot SEN0240 sensors with dry metal electrodes, one on the closing muscles (A0) and one on the opening muscles (A1). The MyoWare sketches use a single sensor and a single channel.
- **Different signal:** the SEN0240 outputs raw EMG, so the envelope is built in firmware (DC removal, square, 150 ms moving mean, square root). The MyoWare `ENV` pin did this in hardware.
- **Numbers do not transfer:** the MyoWare thresholds and baselines (rest about 90 counts, max about 870, thresholds 140/220) apply only to this sensor and are not valid for the SEN0240.

## Contents

| Sketch | What it does |
|---|---|
| `Single-myo-pad-test-raw` | Prints the raw ADC reading and lights the LED above a threshold. First check that the sensor works. |
| `single_myo_test_env` | Plots the ENV output against a threshold. |
| `no-pad-1-soft-rectify` | Builds an envelope in software from the raw pin (baseline tracking, rectify, smooth). Ancestor of the current signal chain. |
| `EMG_threshold_calibration` | Two-stage calibration (rest, max) with no motors. Found that the ENV pin idles near 90 counts, so thresholds must be referenced to rest. |
| `EMG_one_finger` | First attempt to drive a finger from the sensor. May not compile. |
| `EMG_One_finger_testing` | One finger with hysteresis (on at 220, off at 140). |
| `EMG_Four_fingers` | All four fingers with a single threshold. |
| `EMG_calibrated_hand_control` | Hand control with runtime calibration and a safety interlock. |
| `EMG_proportional_hand_control` | Adds proportional speed (PWM 100 to 220) from a smoothed envelope. Last MyoWare version. |

`MyoWare_v2_AdvancedGuide-Updated.pdf` is the manufacturer's datasheet.

## Build and flash

Open this folder (the one containing `platformio.ini`) in VS Code, as described in the main README. Each sketch is its own environment, so pick one in PlatformIO > Project Tasks, or from the terminal:

    pio run -e <environment> -t upload

| Environment | Sketch |
|---|---|
| `<env name>` | `Myo-1-no-mcp` |
| `<env name>` | `no-pad-1-soft-rectify` |
| ... | ... |

Use the same finger pin map as the current firmware.