# 05 · Kinematics

## Coordinates and joint conventions

Origin on the base axis at table height; **X forward, Y left, Z up**. The arm is a base rotation plus a planar arm that tilts in the vertical plane
through the base axis, followed by a wrist roll, a wrist pitch and a gripper.

| Joint | Angle means | Home |
|---|---|---|
| 0 Base | rotation about the vertical axis; 0 = facing +X; positive turns towards +Y (left) | 0 |
| 1 Shoulder | angle of the upper arm above the horizontal; 90 = straight up | 90 |
| 2 Elbow | bend of the forearm relative to the upper arm; 0 = straight; positive bends forward/down | 0 |
| 3 Roll | rotation of the gripper about the forearm axis | 0 |
| 4 Pitch | bend of the gripper relative to the forearm; positive tips it forward/down | 0 |
| 5 Gripper | 0 = half open; positive closes | 0 |

Link lengths: `H0` = 9.7 cm (table to shoulder pivot), `L1` = 12.0 cm (upper arm), `L2` = 9.3 cm (elbow to the wrist pivot, the end of the forearm).

## Inverse kinematics (wrist position)

For a target wrist pivot `(x, y, z)` and a tool pitch `θ` (direction of the gripper above the horizontal; −90° is straight down):

```
base      = atan2(y, x)
r         = sqrt(x² + y²)            z' = z − H0            D = sqrt(r² + z'²)
α         = acos((L1² + D² − L2²) / (2·L1·D))              β = atan2(z', r)
shoulder  = β + α                                          (elbow-up solution)
γ         = acos((L1² + L2² − D²) / (2·L1·L2))
elbow     = 180° − γ
pitch     = (shoulder − elbow) − θ
```

The target is rejected if `D > L1 + L2` or `D < |L1 − L2|`, or if any joint would leave its limits (the firmware answers `ERR ...` and moves nothing).
Reference result: `XYZ 18 0 14 0 0 0` gives shoulder 39.2°, elbow 59.9°, pitch −20.7°.

Forward kinematics (used by the 3D twin and the safety checker):

```
elbow position = ( L1·cos(shoulder),  H0 + L1·sin(shoulder) )
wrist position = elbow + L2·( cos(shoulder − elbow),  sin(shoulder − elbow) )
tool direction = shoulder − elbow − pitch
```

Because the arm has five joints (plus the gripper), you can choose a position and the tool's pitch but not its full orientation, and only some pitches are reachable at a given point
(the wrist's ±80° range is the usual limit when pointing down).

## The gripper model

The fingertips are far beyond the wrist pivot, so the *fingertip* position matters for anything near the table. Measured on this arm, from the top of the forearm to the fingertips:
**15.2 cm with the fingers closed** and **11.8 cm fully open** (the fingers pivot and swing outwards, which shortens the reach).
The model reproduces both with two numbers: the finger pivots sit `L3` = 4.8 cm above the forearm end and the fingers are `GRIP` = 10.4 cm long, opening to at most 48°:

```
fingertip distance from the wrist pivot = L3 + GRIP · cos(φ)
φ(g) = φmax · (closedDeg − g) / (closedDeg − openDeg), clamped to 0…φmax      (g = gripper joint angle)
```

with `closedDeg` = 25°, `openDeg` = −35° and `φmax` = 48°. These come from measurements and a linear assumption; re-measure for your gripper
(they are in `CONFIG["links"]` and `CONFIG["grip"]` in the dashboard).

Early versions of this project assumed 8.75 cm. That under-estimated the reach by 3–6 cm and would have driven the fingertips into the table
([what went wrong](08-what-went-wrong.md)).

## The safety model

The dashboard's checker (`hazards`, `transition_hazards`, `check_items`) evaluates a pose with forward kinematics:

| Check | Result |
|---|---|
| any joint outside `minJ`…`maxJ` | error |
| elbow or wrist less than 1 cm above the table | error |
| forearm passes through the base cylinder (radius 5.8 cm, height 7.5 cm) | error |
| fingertips more than 0.5 cm below the table | error |
| fingertips within 0.3 cm of the table, or the gripper touching the base | warning |

Because the firmware moves **one joint at a time** in a fixed order (`ORDER` = shoulder, elbow, base, pitch, roll, gripper), the checker also
simulates every in-between position between two poses. Two poses can be safe while the path between them is not: for example, a large shoulder move
made before the wrist moves can drag the fingertips through the table. If you change `ORDER` in the firmware, change it in `eva_control.py` too.

Pick & place keeps the extra 3° margin from every joint limit so no servo sits on its end stop.

## Known limitations

- The wrist pitch axis is assumed to be at the end of the forearm. In reality it sits a little above it, so positions may be off by about the same amount (1–2 cm). Measure it and add it to `L2` if you need accuracy.
- Only the elbow-up solution is used.
- The checker knows the table and the base only. It knows nothing about the cube, the cables or anything else on your desk.
- The gripper opening model is linear and was fitted to two measurements.
- With no position feedback the real arm can differ from the model by a few degrees ([roadmap](09-roadmap-and-improvements.md)).
