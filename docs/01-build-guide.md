# 01 · Build guide

From a pile of parts to an arm that moves. Budget a few evenings; the calibration takes the longest.

## 1. Get the parts and tools

See the [bill of materials](../hardware/bill-of-materials.md). You also need: a 3D printer (or a print service), M3 screws and nuts,
a small screwdriver set, a multimeter, wire strippers, heat-shrink or insulation tape, a protractor or a phone with an
inclinometer app, and a laptop with the Arduino IDE and Python 3.

## 2. Print the parts

The arm in the photos is printed in black PLA with white wrist and gripper parts. Put your files in [`hardware/cad`](../hardware/cad/README.md)
and record your print settings there (layer height, infill, supports). Tips:

- Print the servo mounts with enough walls that the screws do not strip the plastic.
- Test-fit every servo before assembling; printed pockets are often a little tight or a little loose.

## 3. Mount the servos at their home angles

The firmware assumes each joint's *home pose* (arm straight up, wrist in line with the forearm, gripper half open) corresponds to a
specific servo angle (`servoHome` in the joint table). Mount the horns at those angles; do not just snap every horn on at 90°.

1. Wire the electronics first ([guide 02](02-wiring-and-power.md)) and flash the firmware ([firmware README](../firmware/README.md)).
2. Send `SERVO <joint> <angle>` from the Serial Monitor to put a servo at the angle you want.
3. With the servo holding that angle, press the horn onto the spline so the link is in its home position.
4. Do not turn a powered servo by hand: that can strip its gears.

Why it matters: on this build the elbow horn was first mounted so that "straight" was servo angle 115°, which left only about 65° of
bend before the servo ran out of range, and the servo buzzed against its end stop. Remounting it so that "straight" is 55° gave
about 110° of usable bend. Choose each home angle so that the range you need sits in the *middle* of the servo's range:

| Joint | Needs | Good home angle |
|---|---|---|
| Base | both directions | near 90° |
| Shoulder | mostly leaning forward | near 90° |
| Elbow | bends in one direction only | low (about 55°), so the bend has room |
| Roll, pitch, gripper | both directions | near 90° |

## 4. Assemble from the base upwards

1. Base turntable and the base servo (MG996R).
2. Shoulder yoke and the shoulder servo (MG996R).
3. Upper arm and the elbow servo (MG996R).
4. Forearm and the wrist roll servo (MG90S).
5. Wrist bracket and the wrist pitch servo (MG90S).
6. Gripper with its servo (MG90S).

Leave a service loop of wire at every joint so no cable is pulled tight at the ends of travel, and tie the wires down along the
links (a spiral wrap works well). Loose wire loops near the wrist can snag the cube or the table.

## 5. Wire the electronics

Follow [02 · Wiring and power](02-wiring-and-power.md). Check every connection with a multimeter **before** the first power-up,
especially that the servo supply is not connected to the Arduino's 5 V pin.

## 6. Flash the firmware

Open `firmware/EVA_firmware/EVA_firmware.ino`, install *Adafruit PWM Servo Driver Library*, select *Arduino Uno*, and upload.
The serial monitor should print `Ready` after the servos wake up one by one.

## 7. Calibrate

Follow [03 · Calibration guide](03-calibration-guide.md) and edit the joint table in the sketch. Do this *before* you try inverse
kinematics, teach mode or pick & place: they rely on the angles being right.

## 8. Run the dashboard

```bash
pip install -r software/requirements.txt
python software/eva_control.py COM3
```

Open <http://localhost:5000>. Close the Arduino Serial Monitor first.

## 9. First-move checklist

- [ ] The arm rests at the home pose before you switch the servo power on.
- [ ] Speed slider at 20–30%.
- [ ] One joint at a time with the sliders, checking the direction of each ([calibration guide](03-calibration-guide.md#4-check-the-direction-of-each-joint)).
- [ ] A hand near the power switch and the STOP button (Space bar).
- [ ] The extended-arm test: hold the arm horizontally for a few seconds and watch for buzzing or a hot converter.
- [ ] Only then: XYZ moves, teach & replay, gestures, pick & place.
