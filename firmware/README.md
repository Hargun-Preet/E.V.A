# Firmware

[`EVA_firmware/EVA_firmware.ino`](EVA_firmware/EVA_firmware.ino) runs on the Arduino Uno. It drives the six servos through the PCA9685, and implements smooth motion, inverse kinematics and the
**one-motor-at-a-time scheduler**. The folder has the same name as the sketch, as the Arduino IDE requires.

## Build and upload

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. *Sketch → Include Library → Manage Libraries…* → install **Adafruit PWM Servo Driver Library** (it pulls in *Adafruit BusIO*).
3. Open `EVA_firmware.ino`, choose *Tools → Board → Arduino Uno* and the right port, and click *Upload*.
4. Open the Serial Monitor at **115200 baud** with the line ending set to **Newline**. After about 2.5 s it prints `Ready`.

**Edit the joint table first.** The table near the top of the sketch is the calibration of *my* arm. Follow the [calibration guide](../docs/03-calibration-guide.md) for yours,
and put your link lengths in `H0`, `L1`, `L2`. Upload with the arm resting at the home pose and the servo power **off**, then switch the servo power on.

## How it moves

- A command such as `ALL`, `XYZ`, `ANGLE` or `HOME` only sets **targets**.
- Every 20 ms the scheduler lets **one** joint move (in the order `ORDER` = shoulder, elbow, base, pitch, roll, gripper), ramping its speed with a limited acceleration so it starts and stops smoothly.
- When it arrives it waits `GAP_MS` (150 ms) and the next joint with a pending target gets its turn. Idle joints keep holding their position.
- `STOP` makes the moving joint decelerate and holds everything else where it is.

## Safety features in the sketch

| Feature | Constant |
|---|---|
| One joint at a time | `ORDER`, `GAP_MS` |
| Servos never commanded below 8° or above 172° | `SERVO_MIN_DEG`, `SERVO_MAX_DEG` |
| Speed cannot exceed 80% | `SPEED_CAP` |
| Joint limits; targets are clamped, `XYZ` is rejected instead | `minJ`, `maxJ` |
| Servos wake up one at a time at power-up | `setup()` |
| Incomplete or over-long commands ignored | `handleLine()`, `loop()` |

There is **no current or voltage sensing** in this release; see [safety](../docs/06-safety.md) and the [roadmap](../docs/09-roadmap-and-improvements.md).

## Commands

See the [serial protocol](../docs/serial-protocol.md).

## Tests

The scheduler and the inverse kinematics are tested on a PC by compiling this very sketch against stand-ins for the Arduino libraries:

```bash
sh tests/firmware_sim/run.sh
```

## Keeping the dashboard in step

`H0`, `L1`, `L2`, `ORDER`, the limits and the home angles in the sketch must match `CONFIG` and the safety model in `software/eva_control.py`.
