# 09 · Roadmap and improvements

What is missing, in rough priority order. Contributions are welcome ([CONTRIBUTING](../CONTRIBUTING.md)).

## Power and protection (highest priority)

- [ ] Fuse, TVS diode, larger low-ESR capacitor, and a 5 V converter with real headroom (5 A or more continuous).
- [ ] A 2S Li-ion protection board (BMS) and a proper low-voltage cut-off.
- [ ] **Current and voltage monitoring** of the servo rail. This was prototyped with an INA219 on the + wire (address 0x41, because the PCA9685 uses 0x40) and a firmware
  trip on sustained overcurrent / undervoltage. It was **removed from the first release because the sensor was not on the build**, and the design is still sound.
  Cautions: the module's 0.1 Ω shunt drops about 0.1 V per amp (set the buck slightly higher) and it reads at most about 3.2 A.
- [ ] Wire the PCA9685 `OE` pin to an Arduino pin so the firmware can disable all outputs.
- [ ] A hardware emergency stop that cuts servo power.

## Accuracy and feedback

- [ ] Measure the wrist pitch-axis offset (the pitch axis is above the end of the forearm) and add it to the kinematics.
- [ ] Position feedback: read the servo potentiometers (modified servos), or add an IMU on the gripper, so the arm can detect and correct errors.
- [ ] A guided calibration wizard in the dashboard instead of editing the table by hand.
- [ ] Fit the gripper opening model with more than two measurements.
- [ ] Stall detection (for example from current) so a blocked joint stops itself.

## Software

- [ ] **Version handshake** (`VER`) so the app refuses to run against firmware that lacks its commands.
- [ ] Move the configuration (`links`, `limits`, `home`) into one shared file that both firmware and app are generated from.
- [ ] Jog mode: nudge the fingertips in ±X/±Y/±Z with arrow keys using the inverse kinematics.
- [ ] Fingertip-based `XYZ` in the firmware (currently it targets the wrist pivot).
- [ ] Trajectory planning with several joints interpolated safely (without breaking the one-at-a-time rule), and obstacle awareness.
- [ ] Export and import of named sequences; a library of recorded demos.
- [ ] A ROS 2 node and a URDF model, so E.V.A. can be used from standard robotics tools.
- [ ] Voice control.

## Vision

- [ ] A fixed webcam with ArUco markers or colour detection to find the cube automatically and feed the pick & place generator.
- [ ] Camera calibration against the arm's coordinates.

## Mechanical and electrical

- [ ] Strain relief and cable routing at the wrist; replace jumper wires with a small PCB or a terminal board.
- [ ] A gripper with force or contact feedback.
- [ ] Stronger or closed-loop servos for the shoulder and elbow.
- [ ] A heat check of the shoulder and elbow servos during long runs.

## Project and documentation

- [ ] CAD files (STEP and STL) with print settings in [`hardware/cad`](../hardware/cad/README.md).
- [ ] A redrawn schematic in KiCad or Fritzing, including the switch and protection parts.
- [ ] Photos and a demo video in [`media`](../media/README.md).
- [ ] A bill of materials with prices and sources.
