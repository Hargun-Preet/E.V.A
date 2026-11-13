# Poses

Example pose sequences you can load with the dashboard's **Import many poses at once** box ([guide](../docs/04-software-guide.md#import-many-poses-at-once)).
Angles are real joint angles in the order **base, shoulder, elbow, roll, pitch, gripper**.

| File | What it is |
|---|---|
| [`hi_wave_poses.json`](hi_wave_poses.json) | An 8-pose wave: raise the hand, rock the wrist roll four times, lower the hand (JSON format) |
| [`example_text_format.txt`](example_text_format.txt) | A small sequence in the text / CSV format |

## Using them

1. Start the dashboard and put the arm at the home pose.
2. Paste the file's contents into *Import many poses at once*, or choose the file.
3. Press **Check poses**, read the report, then **Load**.
4. Press **Play** (best at speed 50–70 for the wave).

Your own recorded poses are saved by the dashboard to `poses.json` in the folder it runs from; that file is not tracked by git.
To share a sequence, copy it into this folder under a descriptive name. The automated tests check that every file here is feasible.

These poses were made for the calibration and link lengths of the author's arm. Check them with the importer before playing them on yours.
