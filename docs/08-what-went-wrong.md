# 08 · What went wrong

Every project has failures. These are the ones that shaped E.V.A., with what was known, what was suspected, and what changed. Where the cause was not proven, it says so.

## Summary

| # | Incident | Severity | Status |
|---|---|---|---|
| 1 | Burned an Arduino Uno and a PCA9685 | Lost hardware | Mitigated (firmware); more protection recommended |
| 2 | Gripper length under-estimated by 3–6 cm | Would have crashed the gripper | Fixed |
| 3 | Servos buzzing against their end stops | Heat, wasted current | Fixed |
| 4 | Elbow horn mounted with too little range | Lost bend | Fixed (remounted) |
| 5 | Wrong assumption about the pulse range | Servos at the ends of travel | Fixed |
| 6 | Conflicting scale measurements | Wasted calibration time | Process improved |
| 7 | Firmware and app out of step | "Teach & replay does nothing" | Open: no version handshake yet |
| 8 | Presets that the arm could not do | Tool-down presets out of range | Fixed |
| 9 | Small slips (joint mix-up, busy port, snapping home) | Confusion | Fixed |

## 1. Two boards burned during teach & replay

**What happened.** While testing teach & replay, an Arduino Uno and a PCA9685 were destroyed. The firmware at the time ramped several joints *in parallel*, so several MG996R servos
accelerated together, powered from a buck converter rated 3 A.

**Cause.** Not conclusively diagnosed. My reading at the time was back-EMF and voltage spikes from the motors. Contributing factors that were certainly present:

- three MG996R servos can each draw about 2.5 A when stalled, far more than a 3 A converter can supply, and converter ratings are usually peak values;
- there was no fuse and no TVS diode on the servo rail;
- in the photos from that period the electrolytic capacitor was a loose component with bare leads rather than firmly mounted and insulated, so it may have been like that before the accident.

I did not measure the current or the voltage, so the exact mechanism is unknown. It is also possible that a short, a reversed connection or hot-plugging contributed.

**Fixes.**

- The firmware now moves **one joint at a time** ([scheduler](../firmware/EVA_firmware/EVA_firmware.ino)); playback, XYZ, gestures and pick & place all go through it.
- Speed is capped, motion is ramped, and the staggered start-up wakes servos one by one.
- The capacitor is part of the [schematic](../hardware/schematics/README.md); a fuse, TVS diode, bigger converter and a Li-ion protection board are [recommended](02-wiring-and-power.md#protection-worth-adding).

**Lesson.** "It won't use all the power at once" is not a design. Size the supply for the worst case, protect it, and test it with the arm in its worst pose.

## 2. The gripper was 3–6 cm longer than the model

**What happened.** The kinematics assumed a gripper 8.75 cm beyond the wrist. Measuring the real gripper gave **15.2 cm closed and 11.8 cm open**. With the real length, the "reach down" preset
would have put the fingertips about 0.6 cm *below* the table while the model said they were 5 cm above it.

**Cause.** An estimate that was never measured; the table check trusted it.

**Fix.** Measured both lengths, modelled the opening ([kinematics](05-kinematics.md#the-gripper-model)), and added tests. **Lesson:** a safety check is only as good as its inputs. Measure first.

## 3. Servos buzzing at the end of their range

**What happened.** The elbow needed a servo angle of about 175° for a common pose, and the roll joint could be sent to 180°. Both buzzed against their internal stops.

**Cause.** Home angles chosen without checking the range each joint needs; limits set to the servo's nominal 0–180° rather than its real usable range.

**Fix.** Re-homed the joints so the needed range sits mid-way, tightened `minJ`/`maxJ`, added an 8°–172° guard in the firmware and a 3° margin in pick & place. **Lesson:** never run a servo at its limit.

## 4. The elbow had too little range

The elbow horn was first mounted so that "straight" was servo angle 115°. That left about 65° of bend, and a typical reaching pose needed more. Remounting it so that "straight" is 55° gave about 110° of bend.
The XYZ test `XYZ 12 0 12 0 0 0` showed the symptom (it needed an elbow bend of 111°).

## 5. The pulse range was wrong

The first code used 150–600 PCA9685 ticks (about 730–2930 µs), wider than these servos accept. Each servo now has its own range ([calibration](03-calibration-guide.md)).
Hobby servos rarely give a true 180°, so the scale has to be measured.

## 6. Conflicting scale measurements

Readings for the base first suggested the scale was too small (k ≈ 0.8) and later too large (k ≈ 1.17). The cause was measuring by eye and approaching the target from different sides (deadband).
The shoulder also moved about 18% further backwards than forwards. The fix was a repeatable method: always send `HOME` first, measure with a template or inclinometer, test both directions,
and use `posDiv` for a one-sided error.

## 7. Firmware and app out of step

Teach & replay did nothing at first because the app used two commands (`ALL`, `Q`) that the firmware running on the board did not have yet. There is no version handshake, so the failure was silent.
Open item: a `VER` command that the dashboard checks on connect ([roadmap](09-roadmap-and-improvements.md)).

## 8. Presets the arm could not do

The first "reach down" presets needed more wrist pitch than the servo's ±80° range allowed, and put the fingertips in the table. Both problems were found by the feasibility checker, and the presets
were rebuilt. They also showed that pointing straight down only works in a narrow band (wrist about 17–19 cm high, within about 15 cm of the base).

## 9. Small slips

- **Raw servo angle vs joint angle.** `SERVO` takes a raw servo angle (90 is *not* home for most joints); `ANGLE` takes a joint angle. The two are now documented side by side.
- **Roll and pitch swapped** in the inverse-kinematics output array; fixed and covered by tests.
- **"Access is denied" on the serial port** because the Arduino Serial Monitor was open.
- **The arm snapped to home** at the end of a sequence because one code path wrote the final pulse directly instead of ramping.
- **Loose wires** at the wrist can snag the cube; strain relief is on the [roadmap](09-roadmap-and-improvements.md).
