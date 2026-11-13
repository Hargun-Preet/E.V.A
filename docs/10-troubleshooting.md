# 10 · Troubleshooting

| Symptom | Likely cause | What to do |
|---|---|---|
| `Access is denied` / `PermissionError` opening the port | Another program has the serial port (Arduino Serial Monitor, an old `eva_control.py`) | Close it, or unplug and replug the USB cable |
| Dashboard says *Disconnected* | Wrong port, cable, or the Arduino is resetting | Check the port number in Device Manager; restart the app; check the USB cable |
| No servo moves, no `Ready` message | Sketch not uploaded, wrong baud rate or line ending | Re-upload; Serial Monitor at 115200 with *Newline* |
| The PCA9685 power LED is off | Arduino 5 V or GND wire missing | Check the logic wiring ([guide 02](02-wiring-and-power.md)) |
| Servos twitch or do nothing | Servo supply missing, or too low | Measure the buck output at the PCA9685 terminal (about 5 V); check the switch and the cells |
| Servos misbehave after power-cycling only the PCA9685 | The PCA9685 lost its 50 Hz setting | Press the Arduino reset button |
| Arduino resets when a servo moves | Supply sagging or spiking | Check cells and converter; lower the speed; add capacitance / TVS; see [guide 02](02-wiring-and-power.md) |
| One servo buzzes | At or near its end stop, binding mechanically, or tugged by a wire | Move it by hand (power off) through its range; widen the margin from the end stops; check wires |
| Buzzing in one pose only | That pose needs an angle at the servo's limit | Lower the joint limits, re-home the joint, or change the pose |
| Joint moves the wrong way | `dir` has the wrong sign | Flip `dir` in the joint table ([calibration](03-calibration-guide.md#4-check-the-direction-of-each-joint)) |
| Home pose is not straight | `servoHome` is off | Re-find it with `SERVO` and trim ([calibration](03-calibration-guide.md#1-find-each-servohome)) |
| Angles do not match a protractor | The pulse range is wrong | Measure `k` and rescale ([calibration](03-calibration-guide.md#5-check-and-fix-the-scale)) |
| `XYZ` → `ERR joint N needs …` | That position needs more than the joint's limit | Choose a position closer, higher or with a different pitch |
| `XYZ` → `ERR out of reach` | Farther than `L1 + L2` from the shoulder | Move the target closer |
| Playback says *Not played: …* | The checker found an unsafe pose or path | Read the reason; fix the pose; use *Play anyway* only if you are sure |
| Teach & replay does nothing | Firmware older than the app (no `ALL` / `Q`) | Upload the matching firmware |
| Pick & place: "cannot place the fingertips" | The spot is outside the reachable zone, or the hover is too high | Move the marker into the green area; lower the hover height; raise *max tilt* |
| Gripper buzzes while holding the cube | The grip angle is too closed | Reduce *Grip angle* until it holds without stalling |
| The 3D twin and the real arm disagree | Table values (limits, links, home) differ between firmware and app | Make `CONFIG` in `eva_control.py` match the sketch ([software guide](04-software-guide.md#configuration)) |
| Arm droops when stretched out | Not enough torque or supply for the pose | Shorten the reach, lighten the load, check the supply |
| Buck converter very hot | Running beyond its continuous rating | Stop; reduce load; use a larger converter |
| Pytest cannot import the app | Dependencies missing | `pip install -r software/requirements.txt pytest` |
