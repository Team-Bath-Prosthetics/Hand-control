# dry-EMG-control

Current EMG firmware for the four-finger hand. Two DFRobot Gravity Analog EMG sensors (SEN0240, dry metal electrodes) drive the fingers with proportional speed control. A0 sits over the closing muscles and A1 over the opening muscles. The earlier MyoWare work is in `EMG-control-archive/`, and serial-only control is in `Without-EMG/`.

This folder is a PlatformIO project. Open it as the root folder in VS Code, as described in the main README.

## Contents

| Path | What it does |
|---|---|
| `src/dual_channel_proportional_control.cpp` | The whole firmware: signal chain, calibration, control and serial commands |

## Hardware

- Arduino Nano ESP32 (3.3 V logic, 12-bit ADC)
- 2x SEN0240 EMG sensors: A0 (GPIO1) closes, A1 (GPIO2) opens
- 2x L298N drivers, 4x N20 motors
- Sensors are powered from the Nano's 3V3 rail. Their grounds return to the Nano, not through the L298N or motor ground.

| Finger | IN1 | IN2 | ENA |
|---|---|---|---|
| 1 | 2 | 3 | 10 |
| 2 | 4 | 5 | 11 |
| 3 | 6 | 7 | 12 |
| 4 | 8 | 9 | 13 |

## How it works

The SEN0240 outputs raw EMG centred on about 1.5 V, so the envelope is built in firmware, once per channel, on a fixed 1 kHz schedule:

    raw -> DC removal -> pre-filter (pass-through) -> square -> 150 ms moving mean -> square root -> envelope

The envelope feeds a hysteresis state machine: a channel triggers above its high threshold and releases below its low threshold. Motor speed then follows the envelope through that channel's own map, from `PWM_MIN` 100 at the low threshold to `PWM_MAX` 220 at the calibrated max.

## Calibration

EMG control starts **disarmed** and stays that way until a successful `calibrate` in the current session. Nothing is saved across power cycles.

1. Run `calibrate`. Motors stop first.
2. Four stages, each started by typing `r` and Enter, then a 5 s capture: rest (A0), maximum closing (A0), rest (A1), maximum opening (A1). Type `x` at any prompt to abort.
3. Thresholds are `rest + fraction x (max - rest)`, with fractions 0.3 (low) and 0.5 (high) on both channels.
4. The routine warns about weak spans, false triggers at rest, clipping, and crosstalk between channels.

## Serial commands

Open the monitor at 115200 baud with line ending set to **New Line**.

| Command | Action |
|---|---|
| `1` `3` `5` `7` | Open finger 1 to 4 at `testSpeed` (150) |
| `2` `4` `6` `8` | Close finger 1 to 4 at `testSpeed` |
| `0` | Stop all |
| `a` / `c` | Open all / close all |
| `s` | Swap the roles of A0 and A1 |
| `calibrate` | Run the four-stage calibration |
| `e` | Turn EMG control on or off |
| `t` | Print status, thresholds and PWM map |
| `m` | Toggle the live stream (`centred0, env0, centred1, env1, pwm`), readable by the Serial Plotter |

Manual commands work while disarmed.

## Safety

- **Co-contraction lockout:** if both channels pass their trigger together, the hand stops and EMG is locked out for 2 s, then until both muscles relax.
- **Failsafe:** continuous EMG drive for 2.2 s stops the hand and turns EMG off. Send `e` to re-enable.
- **Interlock:** motors stop before calibration starts.
- **Startup gate:** EMG stays disabled until the offsets are seeded and the filters have settled (about 1.2 s).

## Tuning

| Setting | Value | Note |
|---|---|---|
| `RMS_WINDOW_MS` | 150 | Shorter is snappier but noisier, longer is smoother but laggier |
| `LOW_FRACTION` / `HIGH_FRACTION` | 0.3 / 0.5 | Per channel |
| `MIN_VALID_SPAN` | 35 | A starting guess for the RMS envelope, re-measure it |
| `PWM_MIN` / `PWM_MAX` | 100 / 220 | Measured at the default PWM frequency. Re-measure if that changes |
| `CONFLICT_LOCKOUT_MS` | 2000 | |
| `MAX_ACTIVE_MS` | 2200 | |

## Known issues

- `startDrive()` only moves finger 1 under EMG, not all four.
- The failsafe message says 3 s but the limit is 2.2 s.
- The startup help text does not list `c` or `e`.
- No stall detection: if you keep contracting after the fingers bottom out, the drivers sit at stall current.