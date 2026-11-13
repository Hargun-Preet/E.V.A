# 06 · Safety

E.V.A. is a small arm, but it has Li-ion cells, a switching converter and servos that can crush a finger or burn out. This project has already destroyed two electronics boards.

## Hazards and what mitigates them

| Hazard | What this project does | What is still missing |
|---|---|---|
| Supply overload and voltage spikes | One-motor-at-a-time firmware, speed cap, smooth acceleration, 1000 µF capacitor | Fuse, TVS diode, bigger converter, current sensing |
| Li-ion cells | Series pack with a switch | **No cell protection board (BMS)**: over-discharge and short-circuit are unprotected |
| Servo stalled against an end stop | End-stop guard (8°–172°), limits, a 3° margin in pick & place | No stall detection |
| Gripper or arm hits the table | Table, base and in-between checks before every playback; XYZ is rejected if a joint would exceed its limit | The checker does not know about objects or cables |
| Pinched fingers | Low speeds | Keep hands clear: the gripper can close with real force |
| Surprise motion at power-up | Staggered wake-up of the servos | A servo jumps to its commanded angle at power-up: start with the arm at home |
| Bad commands over the network | The server only listens on `127.0.0.1` and only forwards a whitelist of validated commands | — |
| Heat | None active | Check the converter and the shoulder/elbow servos by touch during long runs |

## Before every session

- [ ] Arm resting at the home pose; nothing in its path; table clear.
- [ ] Wiring unchanged since the last good run; no loose capacitor leads or exposed copper.
- [ ] Speed slider low (20–30%) for anything new.
- [ ] USB connected first, then the servo power switch.
- [ ] You know where the servo power switch is.

## While it runs

- Keep a hand near the power switch and the STOP button (Space).
- Watch for buzzing, jitter, a warm converter, or the Arduino resetting: these mean the supply or a servo is struggling.
- **STOP only stops motion.** The servos still hold position. To remove torque, switch the servo power off (the arm will drop).

## After

- Power down in reverse order: servo switch first, then USB.
- Do not plug or unplug servo, signal or I²C wires with the servo power on.
- Store Li-ion cells at a partial charge and never leave them charging unattended.

## If something goes wrong

1. Press STOP, then switch the servo power off.
2. Unplug the battery if anything smells hot.
3. Check wiring, the capacitor and polarity before powering up again; see [troubleshooting](10-troubleshooting.md).
4. Write down what happened. Failures are how this design got better: [what went wrong](08-what-went-wrong.md).
