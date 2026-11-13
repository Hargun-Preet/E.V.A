# Serial protocol

The firmware speaks a plain-text protocol over USB serial: **115200 baud, 8N1, one command per line, line ending `\n`** (Newline). Commands are case-insensitive. Numbers are decimal.
Joint indices: `0` base, `1` shoulder, `2` elbow, `3` roll, `4` pitch, `5` gripper. Angles are real [joint angles](05-kinematics.md#coordinates-and-joint-conventions), not raw servo angles, unless noted.

After a reset the firmware wakes the servos one at a time (about 2.4 s) and prints `Ready`. Opening the serial port resets an Arduino Uno, so wait a few seconds before sending commands.

## Motion commands

All of these only set **targets**. A scheduler then moves one joint at a time (shoulder, elbow, base, pitch, roll, gripper), so the arm keeps moving after the command returns. Poll `Q` to wait for the end.

| Command | Effect | Reply |
|---|---|---|
| `ANGLE <j> <deg>` | Target for one joint, clamped to its limits | none |
| `ALL a0 a1 a2 a3 a4 a5` | Targets for all six joints. An incomplete line is ignored | none |
| `XYZ x y z toolPitch roll grip` | Inverse kinematics for the **wrist pivot** (cm, degrees). Sets all six targets | `OK`, or `ERR out of reach`, `ERR joint <j> needs <deg>`, `ERR need 6 values`. Nothing moves on an error |
| `HOME` | Targets = home pose | none |
| `STOP` | Stop moving; the moving joint decelerates and every joint holds | none |
| `SPEED <1-100>` | Speed scale in percent, capped at `SPEED_CAP` (80) | none |

## Status

| Command | Reply |
|---|---|
| `P` | `a0 a1 a2 a3 a4 a5 m` : the six **actual** joint angles (one decimal) and `m`, the index of the joint that is moving (`-1` if none) |
| `Q` | `BUSY` if any joint is moving or has a pending target, otherwise `IDLE` |

## Calibration commands

| Command | Effect |
|---|---|
| `SERVO <j> <servoDeg>` | Move joint `j` to a raw servo angle (8°–172°); it goes through the joint table, so limits and the guard still apply |
| `RAW <ch> <us>` | Send a raw pulse (500–2500 µs) to a PCA9685 channel. Only accepted when the arm is idle |
| `RELAX <j>` | Stop the PWM on a joint (it goes limp). The joint stays released until the Arduino is reset |

## Examples

```text
> SPEED 30
> ANGLE 1 70
> Q
BUSY
> Q
IDLE
> XYZ 18 0 14 0 0 0
OK
> P
0.0 39.2 59.9 0.0 -20.7 0.0 -1
> XYZ 12 0 12 0 0 0
ERR joint 2 needs 111.3
```

## Notes

- Targets beyond a joint's limit are clamped (`ANGLE`, `ALL`). `XYZ` is stricter: it refuses.
- The protocol has **no version command** yet, so a dashboard cannot tell whether the firmware is recent enough ([roadmap](09-roadmap-and-improvements.md)).
- The dashboard only forwards `ANGLE`, `XYZ`, `HOME`, `SPEED`, `ALL` and `STOP`, with numeric arguments.
