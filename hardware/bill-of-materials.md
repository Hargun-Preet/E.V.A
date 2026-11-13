# Bill of materials

Fill in the price and source columns for your own build.

## Used in E.V.A.

| Part | Qty | Used for | Notes | Price | Source |
|---|---|---|---|---|---|
| Arduino Uno (R3 or compatible) | 1 | Controller | Powered over USB | | |
| PCA9685 16-channel PWM / servo driver board | 1 | Drives the six servos | I²C address 0x40; has a 5 mm screw terminal for servo power | | |
| MG996R servo | 3 | Base, shoulder, elbow | Metal-gear, high torque | | |
| MG90S micro servo | 3 | Wrist roll, wrist pitch, gripper | Metal-gear micro servo | | |
| Li-ion cell, 3.7 V (18650 format) | 2 | Servo power | Used in series (7.4 V nominal) | | |
| 2-cell battery holder | 1 | Holds the cells | Wired in series | | |
| 5 V buck converter, 3 A | 1 | Servo supply | Set to 5.0–5.3 V before connecting servos | | |
| Toggle switch | 1 | Servo power switch | In the battery + line | | |
| Electrolytic capacitor 1000 µF, 16 V | 1 | Servo rail reservoir | Across the PCA9685 `V+` and `GND` terminal | | |
| USB cable (A to B) | 1 | Arduino link and power | | | |
| Jumper wires, servo extension leads | – | Wiring | | | |
| M3 screws and nuts, assorted | – | Assembly | | | |
| Spiral cable wrap | – | Cable management | | | |
| 3D-printed parts (black and white PLA) | – | Structure | See [`cad/`](cad/README.md) | | |

## Recommended additions (not on the first build)

| Part | Why |
|---|---|
| Fuse holder and a 3–5 A fuse (or polyfuse) | Protects the cells and wiring from a short |
| TVS diode SMBJ5.0A | Absorbs voltage spikes on the servo rail |
| 2S Li-ion protection board (BMS) | Over-discharge and short-circuit protection for the cells |
| 5 V UBEC / buck converter rated 5 A or more continuous | Headroom for the servos |
| Capacitor 2200–4700 µF, low ESR, 10 V or more | Larger reservoir, firmly mounted with insulated leads |
| Heat-shrink tubing | Insulation |
| Terminal block or small PCB | Replace loose jumper wires |
| INA219 current sensor (optional) | Live current and voltage; see the [roadmap](../docs/09-roadmap-and-improvements.md) |
