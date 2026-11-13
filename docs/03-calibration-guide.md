# 03 · Calibration guide

Hobby servos are not precise: a command of "180°" rarely gives 180°, and two servos of the same model differ. Calibration makes
the angle you *command* equal the angle you *get*. The numbers live in the joint table in
[`EVA_firmware.ino`](../firmware/EVA_firmware/EVA_firmware.ino):

```cpp
//  ch minUs maxUs dir servoHome jointHome minJ  maxJ  speed accel posDiv
{0,  490, 2415,  1,   90,   0,   -80,   80,  30,  60, 1.00},   // base
```

| Column | Meaning |
|---|---|
| `ch` | PCA9685 channel |
| `minUs`, `maxUs` | Pulse width (µs) that this servo treats as 0° and 180°. Sets the **scale**. |
| `dir` | `+1` or `-1`: does a positive joint angle raise the servo angle? |
| `servoHome` | Servo angle at which the joint is in the home pose. Sets the **zero**. |
| `jointHome` | The joint angle at home (for example 90 for the shoulder, because 90° means vertical). |
| `minJ`, `maxJ` | Software limits in joint angle. |
| `speed`, `accel` | Maximum speed (°/s) and acceleration (°/s²). |
| `posDiv` | Divides motion on the positive side of home. Corrects a servo that moves more one way than the other (1.0 = no correction). |

Use the Serial Monitor at 115200 baud with the line ending set to *Newline*. Keep the speed low and a hand near the power switch.

## 1. Find each servoHome

Use `SERVO <joint> <servoAngle>` (raw servo angle) and adjust until the joint is physically in its home pose:

| Joint | Home pose |
|---|---|
| Base | facing straight forward |
| Shoulder | upper arm vertical |
| Elbow | forearm in line with the upper arm |
| Roll | gripper flat |
| Pitch | gripper in line with the forearm |
| Gripper | half open |

Step in 1–2° increments and **approach the final value from the same side every time** (servos have a deadband of a few degrees).
Put the result in `servoHome`. If a joint is only a few degrees off, trim `servoHome` instead of removing the horn.

## 2. Keep every servo off its end stops

Hobby servos rarely give a true 0–180°. Pushing a servo against its internal stop makes it buzz, heat up and draw extra current.
Choose home angles so the range you need sits in the middle (see the [build guide](01-build-guide.md#3-mount-the-servos-at-their-home-angles)),
and set `minJ` / `maxJ` so the servo stays at least about 10° from either end. The firmware also refuses to command anything below 8° or above 172°.

## 3. Find the pulse range (optional, but worth it)

The defaults `600`–`2400` µs are safe. To find the real ends, with the arm free to move, use `RAW <channel> <µs>`
(only works when the arm is idle). Step from 1500 µs outwards in 50 µs steps until the servo stops moving or starts to buzz,
then back off about 50 µs. Do this one servo at a time.

## 4. Check the direction of each joint

Send `HOME`, then a small positive angle, then `HOME` again:

| Command | Expected |
|---|---|
| `ANGLE 0 20` | base turns **left** (towards +Y) |
| `ANGLE 1 80` | shoulder leans **forward** |
| `ANGLE 2 20` | forearm bends **forward/down** |
| `ANGLE 4 20` | gripper tips **forward/down** |
| `ANGLE 5 20` | gripper **closes** |

If a joint goes the wrong way, flip its `dir`. (On this build the pitch and gripper joints needed `-1`.)

## 5. Check and fix the scale

Put a phone with an inclinometer app (or a protractor / printed angle template) against the link.

1. Send `HOME` and note the reading.
2. Send a large move, for example `ANGLE 1 60` (30° from vertical), and read again.
3. Compute `k = measured / commanded`. If `k` is within a degree or two of 1 over a 30° swing, leave the joint alone.
4. Otherwise rescale the pulse range **without moving home**:

```
P_home     = minUs + (maxUs - minUs) * servoHome / 180
span_new   = (maxUs - minUs) / k
minUs_new  = P_home - span_new * servoHome / 180
maxUs_new  = P_home + span_new * (180 - servoHome) / 180
```

Illustrative example: `minUs 600`, `maxUs 2400`, `servoHome 115`, `k = 0.83` gives `P_home = 1750`, `span_new = 2169`,
`minUs_new = 364`, `maxUs_new = 2533`. These are only numbers in the table; the limits keep the servo inside its real range.

Tips: always send `HOME` before each test so every move approaches from the same side; test both directions; repeat the same move
several times (1–2° of variation is normal); and measure against a template rather than by eye, because small reading errors
change `k` a lot. On this build, readings for the base first suggested `k ≈ 0.8` and later `k ≈ 1.17`; the difference was measurement error.

If one direction overshoots more than the other (the shoulder leaning backwards moved about 18% too far), use `posDiv`
(`1.18` for the shoulder).

## 6. Set the limits

Set `minJ` / `maxJ` so the arm cannot hit itself, the table or the gripper's own stops. For the gripper, find the closed angle that holds
your object firmly without stalling the servo (a stalled micro servo cooks itself).

## 7. Verify with inverse kinematics

Send `XYZ 18 0 14 0 0 0`. The firmware should answer `OK`, and `P` should show roughly shoulder 39°, elbow 60°, pitch −21°
(with the link lengths in the sketch). Check the links with the inclinometer: the upper arm about 39° above horizontal, the forearm
about 21° below, the gripper level. If those match, the geometry is calibrated.

## The author's final table (for reference only)

| Joint | Servo | ch | µs range | dir | servoHome | limits (joint °) |
|---|---|---|---|---|---|---|
| Base | MG996R | 0 | 490–2415 | +1 | 90 | −80…80 |
| Shoulder | MG996R | 1 | 600–2400 | +1 | 95 | 20…120 (posDiv 1.18) |
| Elbow | MG996R | 2 | 600–2400 | +1 | 55 | −40…110 |
| Roll | MG90S | 3 | 500–2400 | +1 | 100 | −80…70 |
| Pitch | MG90S | 4 | 500–2400 | −1 | 90 | −80…80 |
| Gripper | MG90S | 5 | 500–2400 | −1 | 90 | −40…40 |

Do not copy these blindly: they are the calibration of one particular arm.
