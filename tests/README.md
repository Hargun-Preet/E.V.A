# Tests

| Test | What it checks |
|---|---|
| `test_kinematics.py` | Inverse and forward kinematics agree with each other and with numbers measured on the real arm; the gripper model reproduces 15.2 / 11.8 cm |
| `test_checker.py` | The feasibility checker and importer: formats, limits, the table, in-between positions, load-all vs feasible-only; every file in `poses/` |
| `test_pick_place_and_gestures.py` | The pick & place generator, the reachable zone, the 3° limit margin, and that every gesture is safe from home |
| `test_api_and_playback.py` | Command whitelist, speed cap, STOP, the playback safety check, pose editing |
| `firmware_sim/` | Compiles the real `EVA_firmware.ino` on a PC against stand-in Arduino libraries and checks the one-motor-at-a-time rule, IK results, STOP, limits and guards |

## Run

```bash
pip install -r software/requirements.txt pytest
python -m pytest                    # Python tests
sh tests/firmware_sim/run.sh        # firmware simulation (needs g++)
```

The Python tests use a fake Arduino (`conftest.py`), so no hardware is needed. They do **not** replace testing on the real arm: they cannot check servo behaviour, wiring or power.
