# 07 · How I built it

<!-- Add your own details here: when you started, how long it took, what it cost, why you chose a robot arm, and photos of the build. -->

This is the story of how E.V.A. went from a sketch to a calibrated, tested arm. The failures are in [08 · What went wrong](08-what-went-wrong.md).

## 1. Design

I started with a paper sketch of the links and joints ([`hardware/dimensions`](../hardware/dimensions/README.md)): a 9.7 cm base, a 12 cm upper arm, a 9.3 cm forearm and a
2.75 cm wrist block, with six servos. The three lower joints carry the weight of the arm, so they are **MG996R** servos; the wrist and gripper use lighter **MG90S** micro servos.
The links are 3D printed, black for the structure and white for the wrist and gripper.

## 2. Electronics

An Arduino Uno talks to a **PCA9685** 16-channel PWM driver over I²C, so the servos are not driven from the Arduino at all. The servos run from a separate 5 V rail:
two Li-ion cells in series feed a 3 A buck converter, and the Arduino is powered by USB, with a common ground through the PCA9685. The shop that sold me the cells and the converter
said I would not need to power all the motors at once, and that careful code would keep a 3 A converter sufficient. That is partly true: what matters is the *peak* current, and code can
lower it, which is exactly what the final firmware does. It cannot remove the holding current or protect a supply with no headroom ([the full story](08-what-went-wrong.md)).

## 3. First code

The first sketch moved every servo to 90° and back, which proved that wiring, power and all six motors worked. The first inverse-kinematics attempt was written directly in the sketch
(law of cosines for the shoulder and elbow, `atan2` for the base) with a single pulse range for all servos and the assumption that "90° on every servo" was the arm's home pose.
Both assumptions turned out to be wrong.

## 4. Calibration

Calibration took the longest, and it was done by measurement, with a phone inclinometer and a protractor, not by trusting the numbers:

- Each servo got its own **home angle**, found by moving it with raw servo angles until the link was physically straight. Several horns were remounted for a better range.
- Each servo got its own **pulse range** (scale). The shoulder also needed a correction for moving more in one direction than the other.
- Each joint got **limits** that keep it away from its end stops.
- Directions were checked with small moves and flipped where needed (pitch and gripper).

The result is the joint table in the firmware and the procedure in the [calibration guide](03-calibration-guide.md).

## 5. From a sketch to a software stack

| Step | What changed | Why |
|---|---|---|
| 1 | Sample sketch: all servos to 90° | Prove the hardware works |
| 2 | Smooth moves, then a calibrated joint table | Stop the sudden jumps; make angles real |
| 3 | Serial protocol (`ANGLE`, `ALL`, `XYZ`, `HOME`, …) and IK in the firmware | Control the arm from a PC |
| 4 | Python/Flask dashboard | Sliders and XYZ boxes are faster than typing commands |
| 5 | Teach & replay | Record poses and play them back |
| 6 | **One-motor-at-a-time scheduler** | After the power accident, no two servos ever move together |
| 7 | Three.js digital twin | See the arm's real angles, and check calibration against a ruler |
| 8 | Bulk pose import and a feasibility checker | Refuse poses that would hit the table, the base, or the limits |
| 9 | Measured gripper length; fingertip model | Fix a 3–6 cm error that would have crashed the fingertips |
| 10 | Pick & place generator, gesture library | Demos |
| 11 | Python tests and a firmware simulation, CI | Make changes safely |

## 6. Design decisions

- **Serialise the motors.** Cheapest and most effective protection I had, and it needs no extra parts.
- **Joint-space protocol.** The firmware speaks joint angles (plus a simple `XYZ` for the wrist). The PC does the planning, checking and the pretty parts.
- **Reject, don't clamp.** An unreachable `XYZ` or an out-of-range pose is refused with a reason, because a silent clamp moves the arm somewhere unexpected.
- **Check before moving.** The same safety model is used for imports, pick & place, gestures and every playback.
- **Measure, then model.** Every number in the model (link lengths, gripper length, scale factors) came from a measurement, and where I guessed I said so in the docs.
- **Keep it open-loop and simple.** No feedback sensors yet; see the [roadmap](09-roadmap-and-improvements.md).

## 7. Tools

Arduino IDE and the Adafruit PCA9685 library; Python 3, Flask and pyserial; three.js for the 3D view; a multimeter, a protractor and a phone inclinometer for calibration;
a phone camera for the demo videos.

## 8. What I would tell myself at the start

1. Design the power supply for the worst case, not the typical case, and add a fuse and a TVS diode from day one.
2. Mount horns at the angles the software expects, with the range you need in the middle of the servo's range.
3. Measure everything (including the gripper) before writing the kinematics.
4. Put safety checks in software early; they are cheap and they catch mistakes before the hardware does.
5. Version your protocol, so the firmware and the app can tell when they do not match.
