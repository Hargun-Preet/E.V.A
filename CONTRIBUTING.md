# Contributing

Thanks for taking a look at E.V.A.! Issues and pull requests are welcome.

## Before you open a pull request

1. Run the tests:
   ```bash
   pip install -r software/requirements.txt pytest
   python -m pytest                 # Python tests
   sh tests/firmware_sim/run.sh     # firmware simulation (needs g++)
   ```
2. If you change motion, kinematics or the joint table, add or update a test.
3. Keep the firmware and the dashboard in step: if you change the [serial protocol](docs/serial-protocol.md), update the docs and both sides.
4. Describe how you tested it. For anything that moves hardware, say whether you ran it on a real arm.

## Safety

Hardware changes can destroy electronics (this project has [already lost two boards](docs/08-what-went-wrong.md)).
Please do not remove the one-motor-at-a-time scheduler, the servo end-stop guard or the playback safety checks without
explaining what replaces them.

## Reporting a problem

Include: what you did, what you expected, what happened, the firmware version, and your servo / power setup.
Photos of the wiring help a lot.
