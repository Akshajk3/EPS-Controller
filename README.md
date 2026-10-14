# ESP32 Steering Angle Controller

ESP32 firmware that closes a position loop around a steering column. It reads two PWM-output absolute angle sensors, runs a PID controller toward a target angle, and drives a pair of complementary analog (DAC) signals that command the steering actuator. A Bluetooth gamepad can be used for manual control through [Bluepad32](https://github.com/ricardoquesada/bluepad32).

---

## How it works

```
 AS1 (PWM) ──┐                                     ┌── DAC1 (GPIO25) ──┐
             ├─► angle unwrap ─► avg angle ─► PID ─┤                   ├─► actuator / EPS controller
 AS2 (PWM) ──┘                                     └── DAC2 (GPIO26) ──┘
                                                          ▲
                              Bluetooth gamepad (left stick X) overrides
```

1. **Angle sensing.** Each sensor outputs a PWM signal whose duty cycle (10%–90%) encodes angle. Pin-change interrupts measure the high and low times of each pulse; the main loop converts them to duty cycle.
2. **Unwrapping.** Duty-cycle changes are converted to angle deltas (2.25° per 1% duty, i.e. the 80% usable span = 180°) and accumulated, with wraparound handling, so the angle keeps counting past a single sensor revolution. Each sensor has its own direction setting (`ASx_CCW_POSITIVE`).
3. **Averaging.** The two sensor angles are averaged into a single steering angle.
4. **PID.** `PID_v1` computes an output in ±`MAX_OUTPUT` from the error between the measured angle and `TARGET_ANGLE`.
5. **Torque signal.** The output is applied as complementary offsets around a center value:
   - `t1 = centerV − output`
   - `t2 = centerV + output`

   Both are clamped to the safe range `v2`–`v1`. This mirrors a dual-channel torque-sensor style signal. Inside the angle deadband, a small constant `HOLD_OUTPUT` is applied instead; at a setpoint near 0°, both channels return to `centerV`.
6. **Output.** `t1` and `t2` are written to the ESP32's 8-bit DACs every 150 ms.

---

## Hardware

| Part | Notes |
|---|---|
| ESP32 (original, with DAC) | GPIO25 and GPIO26 are DAC pins on the classic ESP32 only. S2/S3/C3 boards will not work as-is. |
| 2× PWM-output absolute angle sensors | Duty cycle must stay within 10%–90% |
| Steering actuator / EPS controller | Must accept the two analog command voltages |
| Bluetooth gamepad (optional) | Any controller supported by Bluepad32 |

### Pinout

| Signal | ESP32 pin | Notes |
|---|---|---|
| `DAC1` | GPIO25 | Torque channel 1 |
| `DAC2` | GPIO26 | Torque channel 2 |
| `AS1` | GPIO14 | Angle sensor 1 PWM (white wire, top pin) |
| `AS2` | GPIO13 | Angle sensor 2 PWM (red wire, bottom pin) |

### DAC levels

Values are 8-bit DAC codes. Approximate voltages assume a 3.3 V reference (`code / 255 × 3.3 V`).

| Constant | Code | Approx. voltage | Meaning |
|---|---|---|---|
| `centerV` | 198 | ~2.56 V | Neutral (no torque) |
| `v1` | 234 | ~3.03 V | Upper limit |
| `v2` | 148 | ~1.91 V | Lower limit |

---

## Software setup

### Dependencies

- Arduino IDE or PlatformIO
- [Bluepad32 for Arduino](https://bluepad32.readthedocs.io/) (installed as an ESP32 board package, not a normal library; follow its install guide)
- [PID_v1](https://github.com/br3ttb/Arduino-PID-Library) by Brett Beauregard

### Build and flash

1. Install the Bluepad32 ESP32 board package and select your ESP32 board.
2. Install the `PID_v1` library (Library Manager → "PID").
3. Open the sketch, then build and upload.
4. Open the Serial Monitor at **9600 baud**.

On boot the firmware waits **10 seconds** before entering the loop.

---

## Usage

### Startup

The sensor accumulators start from `lastDuty1 = 80` and `lastDuty2 = 30`, and the reported angle is **relative to that starting position**. Power the board on with the steering at its intended zero/center position, or set `lastDuty1`/`lastDuty2` to match the sensor readings at center.

### Serial output

Each loop prints one status line:

```
Target: 65.0  Current: 12.3  Output: 33.1  T1: 165  T2: 231
```

Uncomment the commented `Serial.print` lines in `loop()` to see raw duty cycle and frequency for each sensor.

### Gamepad control

When a connected gamepad sends data, the left stick X axis (−512…512) is mapped directly onto `t1`/`t2` (`v1→v2` and `v2→v1`), overriding the PID output for that cycle. Pairing keys are cleared on every boot (`BP32.forgetBluetoothKeys()`), so re-pair the controller each time.

---

## Configuration

All tuning values are `#define`s at the top of the sketch.

| Constant | Default | Description |
|---|---|---|
| `KP` / `KI` / `KD` | 1.25 / 0.0 / 0.7 | PID gains |
| `TARGET_ANGLE` | 65.0° | Setpoint |
| `MAX_OUTPUT` | 40.0 | PID output limit (±) |
| `ANGLE_DEADBAND` | 15.0° | Within this error, hold output is used instead of PID output |
| `MIN_MOTOR_OUTPUT` | 15.0 | Minimum output magnitude outside the deadband (overcomes actuator stiction) |
| `HOLD_OUTPUT` | 10.0 | Output applied inside the deadband |
| `MIN_DUTY` / `MAX_DUTY` | 10.0 / 90.0 | Valid sensor duty range; readings outside are ignored |
| `ASx_CCW_POSITIVE` | false / true | Sensor direction polarity |
| `ASx_CENTER` | 80 / 30 | Center duty values (currently only used as initial values) |

The PID sample time is 150 ms (`SetSampleTime(150)`), matching the `delay(150)` at the end of `loop()`.

### Tuning tips

- Start with `KI = 0` and raise `KP` until the response is quick but not oscillating, then add `KD` to damp it.
- If the actuator buzzes near the target, shrink `MIN_MOTOR_OUTPUT` or widen `ANGLE_DEADBAND`.
- If the wheel runs away from the target, flip the polarity of the DAC outputs or the `ASx_CCW_POSITIVE` flags.

---

## Known issues

- `onDisconnect()` searches for an empty slot (`== nullptr`) instead of the disconnecting controller, so a disconnected gamepad is never removed from the list.
- In `processGamepad()`, the deadband branch sets `t1`/`t2` to center but doesn't `return`, so the following `map()` calls overwrite it. Stick drift will cause slight torque output.
- Gamepad input overrides the PID without a mode switch, and the PID keeps running underneath it.
- `intialized1`/`intialized2` are always `true` (and misspelled), so there is no startup validation of sensor readings.
- No fault handling: a lost or out-of-range sensor simply holds the last angle, and the controller keeps driving.
- `AS1_CENTER` and `AS2_CENTER` are defined but not used, and the `freq1`/`freq2` values are computed but unused.
- The 150 ms loop is slow for a steering loop; the main loop also blocks on `delay()`.

---
