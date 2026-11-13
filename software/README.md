# Software

[`eva_control.py`](eva_control.py) is the E.V.A. dashboard: a Flask server that talks to the Arduino over USB serial and serves a page with the 3D digital twin and all controls.

## Install and run

```bash
pip install -r requirements.txt
python eva_control.py COM3        # Linux: /dev/ttyUSB0 or /dev/ttyACM0; the port is auto-detected if omitted
```

Open <http://localhost:5000>. Close the Arduino Serial Monitor first. The 3D view loads three.js from a CDN, so the browser needs internet access.

## What it does

Joint sliders and per-joint home, XYZ moves, a 3D digital twin, teach & replay with editable values, bulk pose import with a feasibility report, a pick & place generator with a reachable-zone map,
a gesture library, a speed limit and a STOP button. Every playback is safety-checked first. Full guide: [docs/04-software-guide.md](../docs/04-software-guide.md).

## Files

| File | Purpose |
|---|---|
| `eva_control.py` | Server, kinematics, safety checker, pick & place, gestures and the web page (all in one file so it is easy to copy and run) |
| `requirements.txt` | `flask`, `pyserial` |
| `poses.json` | Created in the folder you run it from; your recorded poses. Not tracked by git |

## Configuration

Edit `CONFIG` at the top of `eva_control.py`. It must match the firmware: [software guide](../docs/04-software-guide.md#configuration).

## Tests

```bash
pip install pytest
python -m pytest          # from the repository root
```
