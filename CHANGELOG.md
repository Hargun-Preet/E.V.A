# Changelog

All notable changes to E.V.A. are listed here. The format follows [Keep a Changelog](https://keepachangelog.com/).

## [1.0.0] — first public release

### Firmware
- Per-joint calibration table: pulse range, direction, home angle, limits, speed, acceleration.
- Speed- and acceleration-limited motion; non-blocking main loop.
- **One-motor-at-a-time scheduler** for `ANGLE`, `ALL`, `XYZ` and `HOME`.
- Servo end-stop guard (8°–172°), speed cap, `STOP`, `RELAX`, `Q` (busy/idle) and `P` (state) commands.
- Inverse kinematics for the wrist pivot with limit checking (rejects instead of clamping).

### Dashboard
- Joint sliders with per-joint home buttons, XYZ moves, speed slider, STOP button (Space bar).
- Three.js digital twin with live wrist and fingertip coordinates and a fingertip trail.
- Teach and replay with editable joint values, reordering and pauses.
- Bulk pose import (text, CSV, JSON) with a feasibility report.
- Pick & place generator with a clickable reachable-zone map.
- Gesture library: wave, nod, shake, bow, point, handshake.
- Playback safety check from the arm's real position, including the in-between positions.

### Tests and tooling
- 28 Python tests and 16 firmware-simulation checks; GitHub Actions workflow.

### Notes
- A power-monitor feature using an INA219 current sensor was prototyped and removed from this release (no sensor on the build).
  It is listed in the [roadmap](docs/09-roadmap-and-improvements.md).
