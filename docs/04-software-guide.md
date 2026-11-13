# 04 · Software guide

The dashboard is [`software/eva_control.py`](../software/eva_control.py): a small Flask server that talks to the Arduino over USB
and serves a web page with a 3D digital twin and all the controls.

## Run it

```bash
pip install -r software/requirements.txt
python software/eva_control.py COM3          # Windows;  /dev/ttyUSB0 or /dev/ttyACM0 on Linux; the port is auto-detected if omitted
```

Wait about four seconds (the Arduino resets when the port opens and the servos wake up one by one), then open <http://localhost:5000>.
The page only listens on your own computer (`127.0.0.1`).

> Only one program can use the serial port. Close the Arduino Serial Monitor first ("Access is denied" means the port is busy).

## The page

**Header.** Connection status, the speed slider (5–80%), **Home**, and the red **STOP** button. **Space** also stops, unless you are typing in a field.
STOP halts all motion and holds position; it does not cut power.

**3D digital twin.** Drag to rotate, scroll to zoom. It shows the arm's *actual* angles as reported by the firmware, not the commanded ones.
It also shows the wrist-pivot and estimated fingertip coordinates (cm), highlights the joint that is currently moving and draws a trail of the fingertip.
Comparing the readout with a ruler is a quick check of your calibration.

**Joints.** One slider per joint (blue = commanded value, green = actual value). The ⌂ button sends only that joint home. The chips below open and close the gripper.

**Move to position.** Enter X (forward), Y (left), Z (up) of the **wrist pivot** in cm, plus tool pitch, roll and gripper angle. Pitch 0 is a level gripper
and −90 points straight down. The fingertips are 12–15 cm beyond the wrist pivot, so keep Z high when pointing down. Presets are provided.
If a position needs more than a joint can do, you get a plain-language message and the arm does not move.

**Pick & place.** See [below](#pick--place).

**Gestures.** Wave, nod, shake, bow, point and handshake. **Play** runs it once (optionally at its suggested speed); **Add** appends its poses to your teach list.

**Teach & replay.**

- **Record pose** stores all six actual joint angles (wait for the arm to stop first).
- Each pose card has a name, a pause, **Go**, **Set to current**, ↑/↓ reorder, delete, and six editable joint values. Typed values are clamped to the joint limits.
- **Add a pose by typing joint angles** creates a pose from six numbers; **Use current angles** and **Reset to home** fill the fields for you.
- **Play** runs the list once, **Loop** repeats it. Joints move one after another inside every pose.
- Poses are saved automatically to `poses.json` in the folder you ran the program from.

## Import many poses at once

Under *Teach & replay*: paste text or choose a file, pick **Add to the end** or **Replace current poses**, and press **Check poses**.
Every pose is parsed, then checked. You get a report (✓ OK, ! warning, ✕ not feasible) with the reason. **Nothing is loaded until you press a button:**
**Load all** if everything is feasible, otherwise **Load the feasible poses only**.

Accepted formats:

```text
# text / CSV: name, base, shoulder, elbow, roll, pitch, gripper, pause   (name and pause are optional)
Raise hand, 0, 80, 35, 0, 0, -30, 0.8
0 80 35 30 0 -30
```

```json
[{"name": "Wave 1", "angles": [0, 80, 35, 30, 0, -30], "pause": 0.2}]
```

JSON also accepts a list of 6-number lists, or objects with the joint names (`base`, `shoulder`, `elbow`, `roll`, `pitch`, `gripper`).
Examples are in [`poses/`](../poses/README.md).

What "feasible" means ([details](05-kinematics.md#the-safety-model)):

1. every value is a number and inside the joint limits (values are rejected, never silently clamped);
2. the elbow, wrist and forearm stay above the table and clear of the base;
3. the fingertips do not go through the table (touching it is only a warning);
4. the **in-between positions** on the way from the previous pose are also safe, because the joints move one at a time.

## Pick & place

1. Click the top-view map to place the **cube**, choose *the drop spot* and click again, or type the numbers.
   The green band is where the arm can do the *whole* sequence with the current settings (about 11–18 cm from the base axis with the defaults).
   A marker turns red when it is outside the band.
2. Set the heights (measured at the fingertips with the gripper closed on the cube) and the gripper angles. **Grip angle** is the closed angle on the cube: tune it
   until the cube is held firmly without the gripper servo buzzing.
3. **Check sequence** shows the generated steps: open, hover above the cube, descend, close, lift, move over the drop spot, lower, release, lift away, return home.
   The vertical moves are split into small steps so the one-at-a-time motion cannot dip the fingertips into the table.
4. **Load into sequence** adds the steps to the teach list; **Play now** runs them once without touching your list.

Under *More settings*: drop height, roll, **max tilt** (how far the gripper may tilt forward from vertical to stay within the wrist's range; a strictly vertical approach does not fit at
most hover heights), the number of smooth steps, and **trim X / Y / Z** to correct a constant offset after your first real try. Every generated pose stays at least 3° away from every joint limit.

Always do the first run at a low speed.

## Playback safety

Before any playback (teach list, gesture, pick & place) the program checks the poses **from the arm's real position** (and from the last pose back to the first when looping).
If a pose is unsafe you get the reason and a *Play anyway* option. While something is playing, manual commands are refused until you press STOP.

## Configuration

At the top of `eva_control.py` the `CONFIG` dictionary holds the numbers the dashboard needs. **Keep them in step with the firmware:**

| `CONFIG` entry | Must match |
|---|---|
| `links` `H0`, `L1`, `L2` | the constants at the top of the sketch |
| `limits` | `minJ` / `maxJ` in the joint table |
| `home` | `jointHome` in the joint table |
| `links` `L3`, `GRIP` and `grip` | your gripper measurements ([kinematics](05-kinematics.md#the-gripper-model)) |

Other constants: `BAUD` (115200), `MAX_SPEED` (80), `POSE_FILE` (`poses.json`).

## HTTP API

The page talks to the server with a few JSON endpoints (`/api/cmd`, `/api/state`, `/api/play`, `/api/import`, `/api/pp_check`, …). They are not a stable public API,
but reading the code is the quickest way to script the arm. To talk to the firmware directly use the [serial protocol](serial-protocol.md).
