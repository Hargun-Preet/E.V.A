"""
E.V.A. control dashboard: web UI, digital twin, teach and replay, pick and place, gestures.
Talks to firmware/EVA_firmware/EVA_firmware.ino over USB serial.

Setup:   pip install flask pyserial
Run:     python eva_control.py COM3      (port is auto-detected if omitted)
Open:    http://localhost:5000
Close the Arduino Serial Monitor first: only one program can use the port.

Safety: the Arduino firmware moves ONE joint at a time. This app also caps speed,
validates every command, and has a STOP button (or press Space).
"""
import json
import math
import os
import re
import sys
import threading
import time

import serial
import serial.tools.list_ports
from flask import Flask, Response, jsonify, request

BAUD = 115200
POSE_FILE = "poses.json"
ALLOWED = {"ANGLE", "XYZ", "HOME", "SPEED", "ALL", "STOP"}
MAX_SPEED = 80
CMD_RE = re.compile(r"^[A-Za-z]+( -?\d+(\.\d+)?)*$")

# Keep in sync with the joint table in firmware/EVA_firmware/EVA_firmware.ino
CONFIG = {
    "names": ["Base", "Shoulder", "Elbow", "Roll", "Pitch", "Gripper"],
    "limits": [[-80, 80], [20, 120], [-40, 110], [-80, 70], [-80, 80], [-40, 40]],
    "home": [0, 90, 0, 0, 0, 0],
    # Link lengths in cm. L3 = forearm end to the finger pivots, GRIP = finger length, so the fingertip is
    # L3 + GRIP*cos(opening) from the forearm end: 15.2 closed, 11.8 fully open (measured).
    "links": {"H0": 9.7, "L1": 12.0, "L2": 9.3, "L3": 4.8, "GRIP": 10.4},
    # Gripper joint angles that look closed / fully open, and the finger opening (deg) when fully open.
    "grip": {"closedDeg": 25, "openDeg": -35, "phiMax": 48},
    "maxSpeed": MAX_SPEED,
}


def find_port():
    for p in serial.tools.list_ports.comports():
        d = (p.description + " " + (p.manufacturer or "")).lower()
        if any(k in d for k in ("arduino", "ch340", "usb serial", "usb-serial", "wch")):
            return p.device
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None


class Arm:
    def __init__(self, port):
        self.lock = threading.Lock()
        self.ok = False
        self.cache = {"angles": None, "moving": -1}
        self.ser = serial.Serial(port, BAUD, timeout=1)
        time.sleep(3.5)  # Arduino resets on connect; wait for its staggered startup
        self.ser.reset_input_buffer()
        self.ok = True

    def send(self, line, reply=False, accept=None):
        with self.lock:
            try:
                if reply:
                    self.ser.reset_input_buffer()
                self.ser.write((line + "\n").encode())
                self.ok = True
                if reply:
                    for _ in range(4):  # skip unrelated lines that are not the reply we want
                        r = self.ser.readline().decode(errors="ignore").strip()
                        if not r or accept is None or accept(r):
                            return r
                    return ""
            except (serial.SerialException, OSError):
                self.ok = False
        return ""

    def status(self):
        """Returns (angles[6], moving_joint) or (None, -1)."""
        r = self.send("P", True, lambda s: s[0] in "-0123456789")
        try:
            v = [float(x) for x in r.split()]
        except ValueError:
            return None, -1
        if len(v) < 6:
            return None, -1
        return v[:6], int(v[6]) if len(v) > 6 else -1

    def busy(self):
        return self.send("Q", True, lambda s: s in ("BUSY", "IDLE")) == "BUSY"

    def wait_idle(self, stop, timeout=120):
        t0 = time.time()
        time.sleep(0.2)
        while self.busy() and not stop.is_set() and time.time() - t0 < timeout:
            time.sleep(0.1)


app = Flask(__name__)
arm = None
poses = []
player = {"stop": threading.Event(), "thread": None, "idx": -1, "name": ""}


def save_poses():
    with open(POSE_FILE, "w") as f:
        json.dump(poses, f, indent=1)


def clamp_angles(vals):
    return [round(min(max(float(v), lo), hi), 1) for v, (lo, hi) in zip(vals, CONFIG["limits"])]


def parse_six(raw):
    vals = [float(x) for x in raw]
    if len(vals) != 6:
        raise ValueError("need 6 values")
    return vals


# ---------------------------------------------------------------- bulk import
ORDER = [1, 2, 0, 4, 3, 5]    # same turn order as the firmware scheduler
FLOOR_MIN = 1.0               # cm: elbow / wrist must stay at least this high
TIP_ERR = -0.5                # cm: tool tip below this means "pushing into the table"
BASE_R, BASE_TOP = 5.8, 7.5   # cm: radius / height of the fixed base cylinder
JOINT_KEYS = ["base", "shoulder", "elbow", "roll", "pitch", "gripper"]
MAX_POSES = 300


def chain(a):
    """Forward kinematics in the arm's vertical plane: (radius, height) of elbow, wrist, tool tip."""
    k = CONFIG["links"]
    fa = a[1] - a[2]                       # forearm direction from horizontal
    e = (k["L1"] * math.cos(math.radians(a[1])), k["H0"] + k["L1"] * math.sin(math.radians(a[1])))
    w = (e[0] + k["L2"] * math.cos(math.radians(fa)), e[1] + k["L2"] * math.sin(math.radians(fa)))
    c = CONFIG["grip"]
    phi = min(max(c["phiMax"] * (c["closedDeg"] - a[5]) / (c["closedDeg"] - c["openDeg"]), 0), c["phiMax"])
    g = k["L3"] + k["GRIP"] * math.cos(math.radians(phi))   # fingertip distance from the wrist pivot
    tool = math.radians(fa - a[4])
    return e, w, (w[0] + g * math.cos(tool), w[1] + g * math.sin(tool))


def in_base(p):
    return abs(p[0]) < BASE_R and p[1] < BASE_TOP


def samples(p, q, n=6):
    return [(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n) for i in range(n + 1)]


def hazards(a):
    e, w, t = chain(a)
    out = []
    if e[1] < FLOOR_MIN:
        out.append(("error", f"the elbow would be only {e[1]:.1f} cm above the table"))
    if w[1] < FLOOR_MIN:
        out.append(("error", f"the wrist would be only {w[1]:.1f} cm above the table"))
    if any(in_base(p) for p in samples(e, w)):
        out.append(("error", "the forearm would hit the base"))
    if t[1] < TIP_ERR:
        out.append(("error", f"the gripper tip would be {-t[1]:.1f} cm below the table"))
    elif t[1] < 0.3:
        out.append(("warn", "the gripper tip touches the table"))
    if any(in_base(p) for p in samples(w, t)):
        out.append(("warn", "the gripper may touch the base"))
    return out


def transition_hazards(prev, a):
    """Hazards at the in-between positions, joints moving one at a time in ORDER."""
    out, st = [], list(prev)
    moved = [j for j in ORDER if abs(a[j] - st[j]) > 0.15]
    for n, j in enumerate(moved[:-1]):
        st[j] = a[j]
        for lvl, msg in hazards(st):
            out.append((lvl, f"while the {CONFIG['names'][j].lower()} moves, {msg}"))
    return out


def _num(x):
    if isinstance(x, bool):
        return None
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _bad(src, msg, name=None):
    return {"src": src, "name": name, "angles": None, "pause": 0.5, "status": "error",
            "issues": [{"level": "error", "msg": msg}]}


def _item(src, name, angles, pause):
    return {"src": src, "name": (str(name)[:40] if name else None), "angles": angles,
            "pause": pause, "status": "ok", "issues": []}


def parse_json(text):
    try:
        data = json.loads(text)
    except ValueError as e:
        return [_bad("JSON", f"not valid JSON ({getattr(e, 'msg', e)}, line {getattr(e, 'lineno', '?')})")]
    if isinstance(data, dict):
        data = data.get("poses", data.get("sequence"))
    if not isinstance(data, list):
        return [_bad("JSON", "expected a list of poses")]
    items = []
    for n, o in enumerate(data, 1):
        src, name = f"#{n}", None
        try:
            pause = 0.5
            if isinstance(o, list):
                vals = [_num(x) for x in o]
                if None in vals or len(vals) not in (6, 7):
                    raise ValueError("expected 6 numbers (plus an optional pause)")
                angles = vals[:6]
                if len(vals) == 7:
                    pause = vals[6]
            elif isinstance(o, dict):
                low = {str(k).lower(): v for k, v in o.items()}
                name = low.get("name")
                if "angles" in low:
                    raw = low["angles"]
                elif all(k in low for k in JOINT_KEYS):
                    raw = [low[k] for k in JOINT_KEYS]
                else:
                    raise ValueError("needs \"angles\" or all six joint names")
                if not isinstance(raw, list):
                    raise ValueError("\"angles\" must be a list")
                angles = [_num(x) for x in raw]
                if len(angles) != 6 or None in angles:
                    raise ValueError("\"angles\" needs exactly 6 numbers")
                if "pause" in low:
                    pause = _num(low["pause"])
                    if pause is None:
                        raise ValueError("pause must be a number")
            else:
                raise ValueError("each pose must be a list or an object")
            items.append(_item(src, name, angles, pause))
        except ValueError as e:
            items.append(_bad(src, str(e), name))
    return items


def parse_text(text):
    items = []
    for ln, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith(("#", "//")):
            continue
        src = f"line {ln}"
        toks = re.split(r"[,;\t]", line) if re.search(r"[,;\t]", line) else line.split()
        toks = [t.strip() for t in toks if t.strip() != ""]
        name = None
        if toks and _num(toks[0]) is None:
            name = toks.pop(0)
        if toks and all(_num(t) is None for t in toks):
            continue                                   # header row such as "name,base,shoulder,..."
        bad = [t for t in toks if _num(t) is None]
        if bad:
            items.append(_bad(src, f"\"{bad[0]}\" is not a number", name))
            continue
        vals = [_num(t) for t in toks]
        if len(vals) not in (6, 7):
            items.append(_bad(src, f"expected 6 angles (plus an optional pause), found {len(vals)} numbers", name))
            continue
        items.append(_item(src, name, vals[:6], vals[6] if len(vals) == 7 else 0.5))
    return items


def check_items(items, start):
    state = list(start)
    for it in items:
        if it["angles"] is None:
            continue
        a, issues = it["angles"], []
        for j, (v, (lo, hi)) in enumerate(zip(a, CONFIG["limits"])):
            if not lo <= v <= hi:
                issues.append({"level": "error",
                               "msg": f"{CONFIG['names'][j]} {v:g} deg is outside the allowed {lo} to {hi}"})
        if it["pause"] < 0:
            issues.append({"level": "error", "msg": "pause cannot be negative"})
        elif it["pause"] > 30:
            issues.append({"level": "warn", "msg": f"pause is {it['pause']:g} s, which is very long"})
        if not any(i["level"] == "error" for i in issues):
            seen = set()
            found = transition_hazards(state, a) + [(l, "at this pose, " + m) for l, m in hazards(a)]
            for lvl, msg in found:
                if (lvl, msg) not in seen:
                    seen.add((lvl, msg))
                    issues.append({"level": lvl, "msg": msg[0].upper() + msg[1:]})
        it["issues"] = issues
        it["status"] = "error" if any(i["level"] == "error" for i in issues) else (
            "warn" if issues else "ok")
        if it["status"] != "error":
            state = a                                  # next pose starts from here
    return items


def build_report(text, mode):
    text = text.strip()
    items = parse_json(text) if text[:1] in ("[", "{") else parse_text(text)
    start = poses[-1]["angles"] if mode == "append" and poses else CONFIG["home"]
    return check_items(items, start)


def summarize(items):
    c = {"total": len(items), "ok": 0, "warn": 0, "error": 0}
    for it in items:
        c[it["status"]] += 1
    return c


# ---------------------------------------------------------------- gestures
# Each pose: (name, [base, shoulder, elbow, roll, pitch, gripper], pause in seconds). They start from home.
GESTURES = [
    {"id": "wave", "name": "Wave hello", "speed": 60,
     "desc": "Raises the hand and waves it from side to side with the wrist roll.",
     "poses": [("Raise hand", [0, 80, 35, 0, 0, -30], 0.8), ("Wave 1", [0, 80, 35, 30, 0, -30], 0.2),
               ("Wave 2", [0, 80, 35, -30, 0, -30], 0.2), ("Wave 3", [0, 80, 35, 30, 0, -30], 0.2),
               ("Wave 4", [0, 80, 35, -30, 0, -30], 0.2), ("Hand steady", [0, 80, 35, 0, 0, -30], 0.6),
               ("Lower hand", [0, 90, 0, 0, 0, 0], 0.5)]},
    {"id": "nod", "name": "Nod yes", "speed": 70,
     "desc": "Tilts the wrist forward and back twice, like nodding.",
     "poses": [("Ready", [0, 85, 20, 0, 0, -10], 0.6), ("Nod 1 down", [0, 85, 20, 0, 28, -10], 0.15),
               ("Nod 1 up", [0, 85, 20, 0, -8, -10], 0.15), ("Nod 2 down", [0, 85, 20, 0, 28, -10], 0.15),
               ("Nod 2 up", [0, 85, 20, 0, -8, -10], 0.15), ("Centre", [0, 85, 20, 0, 0, -10], 0.5),
               ("Home", [0, 90, 0, 0, 0, 0], 0.5)]},
    {"id": "no", "name": "Shake no", "speed": 60,
     "desc": "Turns the whole arm left and right, like shaking a head.",
     "poses": [("Ready", [0, 85, 20, 0, 0, -10], 0.6), ("Look left", [18, 85, 20, 0, 0, -10], 0.2),
               ("Look right", [-18, 85, 20, 0, 0, -10], 0.2), ("Look left", [18, 85, 20, 0, 0, -10], 0.2),
               ("Look right", [-18, 85, 20, 0, 0, -10], 0.2), ("Centre", [0, 85, 20, 0, 0, -10], 0.5),
               ("Home", [0, 90, 0, 0, 0, 0], 0.5)]},
    {"id": "bow", "name": "Bow", "speed": 40,
     "desc": "Leans forward and folds the wrist in a polite bow, then stands up.",
     "poses": [("Lean", [0, 60, 10, 0, 15, 0], 0.4), ("Bow", [0, 40, 35, 0, 35, 0], 1.2),
               ("Rise", [0, 60, 10, 0, 15, 0], 0.4), ("Home", [0, 90, 0, 0, 0, 0], 0.5)]},
    {"id": "point", "name": "Point", "speed": 50,
     "desc": "Extends the arm with a closed gripper and points left, then right.",
     "poses": [("Point ahead", [0, 50, 0, 0, 0, 25], 0.8), ("Point left", [25, 50, 0, 0, 0, 25], 0.8),
               ("Point right", [-25, 50, 0, 0, 0, 25], 0.8), ("Point ahead", [0, 50, 0, 0, 0, 25], 0.5),
               ("Home", [0, 90, 0, 0, 0, 0], 0.5)]},
    {"id": "handshake", "name": "Handshake", "speed": 40,
     "desc": "Reaches out with an open gripper, closes it, pumps the hand twice, then lets go.",
     "poses": [("Reach out", [0, 39, 60, 0, -21, -35], 0.8), ("Grip", [0, 39, 60, 0, -21, 15], 0.5),
               ("Pump up", [0, 39, 48, 0, -21, 15], 0.15), ("Pump down", [0, 39, 62, 0, -21, 15], 0.15),
               ("Pump up", [0, 39, 48, 0, -21, 15], 0.15), ("Pump down", [0, 39, 62, 0, -21, 15], 0.3),
               ("Let go", [0, 39, 60, 0, -21, -35], 0.5), ("Home", [0, 90, 0, 0, 0, 0], 0.5)]},
]


def gesture_items(g):
    return [_item(f"step {n}", name, [float(v) for v in ang], pause)
            for n, (name, ang, pause) in enumerate(g["poses"], 1)]


# ---------------------------------------------------------------- pick and place
def phi_of(g):
    c = CONFIG["grip"]
    return min(max(c["phiMax"] * (c["closedDeg"] - g) / (c["closedDeg"] - c["openDeg"]), 0), c["phiMax"])


def tool_len(g):
    k = CONFIG["links"]
    return k["L3"] + k["GRIP"] * math.cos(math.radians(phi_of(g)))


def limit_issue(a, margin=0):
    for j, (v, (lo, hi)) in enumerate(zip(a, CONFIG["limits"])):
        if not lo + margin <= v <= hi - margin:
            return f"the {CONFIG['names'][j].lower()} would need {v:.0f} deg (usable range {lo + margin} to {hi - margin})"
    return None


def solve_wrist(r, z, base, roll, g, pitch):
    """Joint angles that put the wrist pivot at radius r, height z with the tool at `pitch` degrees."""
    k = CONFIG["links"]
    if r < 1.0:
        raise ValueError("too close to the base")
    zp = z - k["H0"]
    D = math.hypot(r, zp)
    if D > k["L1"] + k["L2"] - 0.1:
        raise ValueError(f"the wrist would have to be {D:.1f} cm from the shoulder, the arm reaches {k['L1'] + k['L2']:.1f}")
    if D < abs(k["L1"] - k["L2"]) + 0.1:
        raise ValueError("too close to the shoulder")
    alpha = math.acos(max(-1, min(1, (k["L1"] ** 2 + D * D - k["L2"] ** 2) / (2 * k["L1"] * D))))
    shoulder = math.degrees(math.atan2(zp, r) + alpha)                    # elbow-up
    gamma = math.acos(max(-1, min(1, (k["L1"] ** 2 + k["L2"] ** 2 - D * D) / (2 * k["L1"] * k["L2"]))))
    elbow = 180 - math.degrees(gamma)
    return [base, shoulder, elbow, roll, (shoulder - elbow) - pitch, g]


def solve_tip(x, y, tip_z, g_len, g_cmd, roll, max_tilt):
    """Joint angles that put the fingertips at (x, y, tip_z) when the gripper opening is g_len and the gripper
    is commanded to g_cmd. The tool points down; it tilts forward by up to max_tilt degrees only if needed.
    Returns (angles, tilt_used). Raises ValueError with the reason."""
    L = tool_len(g_len)
    r = math.hypot(x, y)
    base = math.degrees(math.atan2(y, x)) if r > 0.2 else 0.0
    why, tilt = "not reachable", 0.0
    while tilt <= max_tilt + 1e-9:
        pitch = -90.0 + tilt
        th = math.radians(pitch)
        try:
            a = solve_wrist(r - L * math.cos(th), tip_z - L * math.sin(th), base, roll, g_cmd, pitch)
        except ValueError as e:
            why, tilt = str(e), tilt + 2.0
            continue
        bad = limit_issue(a, 3)   # stay 3 deg away from every joint limit so no servo sits on its end stop
        if not bad:
            return a, tilt
        why, tilt = bad, tilt + 2.0
    raise ValueError(why)


def max_tip_z(x, y, g_len, g_cmd, roll, max_tilt):
    z = 30.0
    while z >= 0:
        try:
            solve_tip(x, y, z, g_len, g_cmd, roll, max_tilt)
            return z
        except ValueError:
            z -= 0.5
    return None


def pp_params(b):
    def f(key, default, lo, hi):
        v = _num(b.get(key, default))
        if v is None or not lo <= v <= hi:
            raise ValueError(f"\"{key}\" must be a number from {lo:g} to {hi:g}")
        return v
    p = {"cube": (f("cx", 15, 4, 26), f("cy", -5, -26, 26)), "drop": (f("dx", 15, 4, 26), f("dy", 8, -26, 26)),
         "grabZ": f("grabZ", 2.0, 0.5, 15), "hoverZ": f("hoverZ", 8.0, 3, 25),
         "openDeg": f("openDeg", -35, -40, 40), "grabDeg": f("grabDeg", 12, -40, 40),
         "roll": f("roll", 0, -80, 70), "maxTilt": f("maxTilt", 20, 0, 35),
         "steps": int(f("steps", 2, 1, 4)),
         "trim": (f("tx", 0, -5, 5), f("ty", 0, -5, 5), f("tz", 0, -5, 5))}
    p["placeZ"] = f("placeZ", p["grabZ"], 0.5, 15)
    if p["hoverZ"] < max(p["grabZ"], p["placeZ"]) + 1:
        raise ValueError("the hover height must be at least 1 cm above the grab and drop heights")
    if p["openDeg"] >= p["grabDeg"]:
        raise ValueError("the open gripper angle must be lower than the grip angle")
    return p


def gen_pick_place(p):
    og, gg, roll, tilt = p["openDeg"], p["grabDeg"], p["roll"], p["maxTilt"]
    tx, ty, tz = p["trim"]
    items = []

    def fixed(name, angles, pause):
        items.append(_item(f"step {len(items) + 1}", name, [float(v) for v in angles], pause))

    def at(name, xy, z, g_cmd, pause=0.4, hover=False):
        src = f"step {len(items) + 1}"
        x, y = xy[0] + tx, xy[1] + ty
        try:
            a, used = solve_tip(x, y, z + tz, gg, g_cmd, roll, tilt)
            it = _item(src, name, a, pause)
            if used >= 2:
                it["note"] = f"gripper tilted {used:.0f} deg to reach this"
        except ValueError as e:
            msg = f"the fingertips cannot be placed at x {xy[0]:g}, y {xy[1]:g}, height {z:g} cm: {e}"
            if hover:
                hz = max_tip_z(x, y, gg, g_cmd, roll, tilt)
                msg += (f". The highest hover possible here is about {hz:.0f} cm, so lower the hover height."
                        if hz is not None else ". Move this spot closer to the arm's reachable zone.")
            it = _bad(src, msg[0].upper() + msg[1:], name)
        items.append(it)

    def vertical(label, xy, z_from, z_to, g_cmd):
        n = p["steps"]
        for k in range(1, n + 1):
            z = z_from + (z_to - z_from) * k / n
            at(f"{label} {k}/{n}" if n > 1 else label, xy, z, g_cmd)

    cube, drop, hz = p["cube"], p["drop"], p["hoverZ"]
    fixed("Open gripper", [0, 90, 0, 0, 0, og], 0.5)
    at("Hover above cube", cube, hz, og, 0.5, hover=True)
    vertical("Descend to cube", cube, hz, p["grabZ"], og)
    at("Close on cube", cube, p["grabZ"], gg, 0.8)
    vertical("Lift cube", cube, p["grabZ"], hz, gg)
    at("Move over drop spot", drop, hz, gg, 0.4, hover=True)
    vertical("Lower to drop spot", drop, hz, p["placeZ"], gg)
    at("Release cube", drop, p["placeZ"], og, 0.6)
    at("Lift away", drop, hz, og, 0.4)
    fixed("Return home", [0, 90, 0, 0, 0, 0], 0.5)
    return items


def pp_report(b):
    p = pp_params(b)
    mode = "replace" if b.get("mode") == "replace" else "append"
    items = gen_pick_place(p)
    start = poses[-1]["angles"] if mode == "append" and poses else CONFIG["home"]
    return check_items(items, start), mode


def load_items(items, mode):
    if mode == "replace":
        poses.clear()
    for it in items:
        poses.append({"name": it["name"] or f"Pose {len(poses) + 1}",
                      "angles": [round(x, 1) for x in it["angles"]], "pause": round(it["pause"], 2)})
    save_poses()


def all_cmd(angles):
    return "ALL " + " ".join(f"{a:.1f}" for a in angles)


def is_playing():
    t = player["thread"]
    return bool(t and t.is_alive())


def seq_problem(seq, loop):
    """Checks a list of poses from the arm's real position (and from the last pose back to the first when looping).
    Returns a message about the first error, or None."""
    start = arm.cache["angles"] or CONFIG["home"]
    items = [{"angles": list(p["angles"]), "pause": float(p.get("pause", 0.5)), "name": p.get("name"),
              "status": "ok", "issues": []} for p in seq]
    if loop and items:
        items.append({"angles": list(items[0]["angles"]), "pause": 0.0, "name": "back to the start",
                      "status": "ok", "issues": []})
    check_items(items, start)
    for n, it in enumerate(items, 1):
        for i in it["issues"]:
            if i["level"] == "error":
                return f"Pose {n} ({it['name'] or 'unnamed'}): {i['msg']}"
    return None


def play_worker(seq, loop, name, main):
    stop = player["stop"]
    player["name"] = name
    try:
        while not stop.is_set():
            for i, p in enumerate(seq):
                if stop.is_set():
                    break
                player["idx"] = i if main else -1
                arm.send(all_cmd(p["angles"]))  # firmware runs the joints one after another
                arm.wait_idle(stop)
                end = time.time() + float(p.get("pause", 0.5))
                while time.time() < end and not stop.is_set():
                    time.sleep(0.05)
            if not loop:
                break
    finally:
        player["idx"] = -1
        player["name"] = ""


def start_playback(seq, loop, name, main, force=False):
    if is_playing():
        return {"error": "Something is already playing. Press STOP first."}
    if not seq:
        return {"error": "There is nothing to play."}
    if not force:
        problem = seq_problem(seq, loop)
        if problem:
            return {"error": f"Not played: {problem}", "force": True}
    player["stop"] = threading.Event()
    t = threading.Thread(target=play_worker, args=(seq, loop, name, main), daemon=True)
    player["thread"] = t
    t.start()
    return {"ok": True}


@app.get("/")
def index():
    return Response(HTML.replace("__CONFIG__", json.dumps(CONFIG)), mimetype="text/html")


@app.post("/api/cmd")
def cmd():
    line = (request.json or {}).get("cmd", "").strip()
    word = line.split(" ")[0].upper() if line else ""
    if word not in ALLOWED or not CMD_RE.match(line):
        return jsonify(error="command not allowed"), 400
    if word == "STOP":
        player["stop"].set()
        arm.send("STOP")
        return jsonify(reply="stopped")
    if is_playing() and word != "SPEED":
        return jsonify(error="Playback is running. Press STOP first."), 409
    if word == "SPEED":
        parts = line.split(" ")
        try:
            line = f"SPEED {min(max(float(parts[1]), 1), MAX_SPEED):.0f}"
        except (IndexError, ValueError):
            return jsonify(error="bad speed"), 400
    reply = arm.send(line, reply=(word == "XYZ"),
                     accept=(lambda r: r == "OK" or r.startswith("ERR")) if word == "XYZ" else None)
    return jsonify(reply=reply)


@app.get("/api/state")
def state():
    c = arm.cache
    return jsonify(connected=bool(arm.ok and c["angles"] is not None), angles=c["angles"], moving=c["moving"],
                   playing=is_playing(), idx=player["idx"],
                   name=player["name"], poses=poses)


@app.post("/api/record")
def record():
    ang, moving = arm.status()
    if not ang:
        return jsonify(error="Could not read the arm."), 500
    if arm.busy():
        return jsonify(error="Wait for the arm to stop moving before recording."), 409
    poses.append({"name": f"Pose {len(poses) + 1}", "angles": ang, "pause": 0.5})
    save_poses()
    return jsonify(ok=True)


@app.post("/api/import_check")
def import_check():
    b = request.json or {}
    text = str(b.get("text", ""))
    if len(text) > 200000:
        return jsonify(error="That file is too large."), 400
    items = build_report(text, b.get("mode", "append"))
    if not items:
        return jsonify(error="No poses found in that text."), 400
    return jsonify(items=items, summary=summarize(items))


@app.post("/api/import")
def import_poses():
    if is_playing():
        return jsonify(error="Playback is running. Press STOP first."), 409
    b = request.json or {}
    mode = "replace" if b.get("mode") == "replace" else "append"
    only = bool(b.get("only_feasible"))
    items = build_report(str(b.get("text", ""))[:200000], mode)
    if not items:
        return jsonify(error="No poses found in that text."), 400
    if any(i["status"] == "error" for i in items) and not only:
        return jsonify(error="Some poses are not feasible, so nothing was loaded.",
                       items=items, summary=summarize(items)), 400
    good = [i for i in items if i["status"] != "error"]
    if not good:
        return jsonify(error="None of the poses are feasible.", items=items, summary=summarize(items)), 400
    if only:  # re-check the reduced list: removing poses changes the moves between the rest
        for g in good:
            g["status"], g["issues"] = "ok", []
        start = poses[-1]["angles"] if mode == "append" and poses else CONFIG["home"]
        good = check_items(good, start)
        if any(g["status"] == "error" for g in good):
            return jsonify(error="Skipping the unfeasible poses makes the remaining moves unsafe.",
                           items=good, summary=summarize(good)), 400
    base_count = 0 if mode == "replace" else len(poses)
    if base_count + len(good) > MAX_POSES:
        return jsonify(error=f"Too many poses (the limit is {MAX_POSES})."), 400
    if mode == "replace":
        poses.clear()
    for g in good:
        poses.append({"name": g["name"] or f"Pose {len(poses) + 1}",
                      "angles": [round(x, 1) for x in g["angles"]],
                      "pause": round(g["pause"], 2)})
    save_poses()
    return jsonify(ok=True, loaded=len(good), skipped=len(items) - len(good))


@app.post("/api/pp_zone")
def pp_zone():
    try:
        p = pp_params(request.json or {})
    except ValueError as e:
        return jsonify(error=str(e)), 400
    good, r = [], 4.0
    while r <= 27.0:
        try:
            for z in (p["grabZ"], p["hoverZ"], p["placeZ"]):
                solve_tip(r, 0.0, z, p["grabDeg"], p["openDeg"], 0, p["maxTilt"])
            good.append(r)
        except ValueError:
            pass
        r += 0.5
    ranges = []
    for r in good:
        if ranges and r - ranges[-1][1] <= 0.51:
            ranges[-1][1] = r
        else:
            ranges.append([r, r])
    return jsonify(ranges=ranges)


@app.post("/api/pp_check")
def pp_check():
    try:
        items, mode = pp_report(request.json or {})
    except ValueError as e:
        return jsonify(error=str(e)), 400
    return jsonify(items=items, summary=summarize(items))


@app.post("/api/pp_play")
def pp_play():
    b = request.json or {}
    try:
        items, _ = pp_report({**b, "mode": "replace"})
    except ValueError as e:
        return jsonify(error=str(e)), 400
    if any(i["status"] == "error" for i in items):
        return jsonify(error="Some steps are not feasible, so nothing was played.",
                       items=items, summary=summarize(items)), 400
    seq = [{"name": i["name"], "angles": i["angles"], "pause": i["pause"]} for i in items]
    r = start_playback(seq, False, "Pick & place", False, bool(b.get("force")))
    return jsonify(r), (200 if r.get("ok") else 409)


@app.post("/api/pp_load")
def pp_load():
    if is_playing():
        return jsonify(error="Playback is running. Press STOP first."), 409
    try:
        items, mode = pp_report(request.json or {})
    except ValueError as e:
        return jsonify(error=str(e)), 400
    if any(i["status"] == "error" for i in items):
        return jsonify(error="Some steps are not feasible, so nothing was loaded.",
                       items=items, summary=summarize(items)), 400
    if (0 if mode == "replace" else len(poses)) + len(items) > MAX_POSES:
        return jsonify(error=f"Too many poses (the limit is {MAX_POSES})."), 400
    load_items(items, mode)
    return jsonify(ok=True, loaded=len(items))


@app.get("/api/gestures")
def gestures_list():
    return jsonify([{"id": g["id"], "name": g["name"], "desc": g["desc"], "speed": g["speed"],
                     "count": len(g["poses"])} for g in GESTURES])


@app.post("/api/gesture/<gid>")
def gesture_do(gid):
    g = next((x for x in GESTURES if x["id"] == gid), None)
    if g is None:
        return jsonify(error="unknown gesture"), 404
    b = request.json or {}
    items = gesture_items(g)
    if b.get("action") == "add":
        if is_playing():
            return jsonify(error="Playback is running. Press STOP first."), 409
        check_items(items, poses[-1]["angles"] if poses else CONFIG["home"])
        bad = [i for i in items if i["status"] == "error"]
        if bad:
            return jsonify(error=f"{g['name']} is not safe from the current last pose: {bad[0]['issues'][0]['msg']}"), 400
        load_items(items, "append")
        return jsonify(ok=True, loaded=len(items))
    seq = [{"name": i["name"], "angles": i["angles"], "pause": i["pause"]} for i in items]
    r = start_playback(seq, False, g["name"], False, bool(b.get("force")))
    return jsonify(r), (200 if r.get("ok") else 409)


@app.post("/api/pose_add")
def pose_add():
    b = request.json or {}
    try:
        vals = parse_six(b.get("angles", []))
    except (TypeError, ValueError):
        return jsonify(error="Enter all 6 joint angles as numbers."), 400
    name = str(b.get("name") or f"Pose {len(poses) + 1}")[:40]
    poses.append({"name": name, "angles": clamp_angles(vals), "pause": 0.5})
    save_poses()
    return jsonify(ok=True)


@app.post("/api/pose/<int:i>")
def pose_edit(i):
    if not 0 <= i < len(poses):
        return jsonify(error="bad index"), 404
    b = request.json or {}
    if b.get("delete"):
        poses.pop(i)
    elif "move" in b:
        j = i + (1 if b["move"] > 0 else -1)
        if 0 <= j < len(poses):
            poses[i], poses[j] = poses[j], poses[i]
    else:
        if "angles" in b:
            try:
                poses[i]["angles"] = clamp_angles(parse_six(b["angles"]))
            except (TypeError, ValueError):
                return jsonify(error="Enter all 6 joint angles as numbers."), 400
        if b.get("from_current"):
            ang, _ = arm.status()
            if not ang:
                return jsonify(error="Could not read the arm."), 500
            if arm.busy():
                return jsonify(error="Wait for the arm to stop moving first."), 409
            poses[i]["angles"] = clamp_angles(ang)
        if "pause" in b:
            poses[i]["pause"] = max(0.0, float(b["pause"]))
        if "name" in b:
            poses[i]["name"] = str(b["name"])[:40]
        if b.get("go"):
            if is_playing():
                return jsonify(error="Playback is running. Press STOP first."), 409
            arm.send(all_cmd(poses[i]["angles"]))
    save_poses()
    return jsonify(ok=True)


@app.post("/api/play")
def play():
    b = request.json or {}
    r = start_playback([dict(p) for p in poses], bool(b.get("loop")), "Sequence", True, bool(b.get("force")))
    return jsonify(r), (200 if r.get("ok") else 409)


@app.post("/api/clear")
def clear():
    player["stop"].set()
    poses.clear()
    save_poses()
    return jsonify(ok=True)


HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>E.V.A. Control</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<style>
:root{--bg:#0b0e13;--panel:#141922;--panel2:#1a212c;--line:#263040;--txt:#e8ecf2;--mut:#8593a6;
 --acc:#4f8cff;--acc2:#35e0a1;--warn:#ffb454;--bad:#ff5d5d;--r:14px}
*{box-sizing:border-box}
body{margin:0;background:radial-gradient(1200px 600px at 20% -10%,#16203180,transparent),var(--bg);color:var(--txt);
 font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
header{display:flex;align-items:center;gap:14px;padding:12px 20px;border-bottom:1px solid var(--line);
 position:sticky;top:0;z-index:20;background:#0b0e13e6;backdrop-filter:blur(8px);flex-wrap:wrap}
h1{font-size:17px;margin:0;font-weight:650;letter-spacing:.2px}
.pill{display:flex;align-items:center;gap:7px;padding:5px 11px;border-radius:99px;background:var(--panel2);
 border:1px solid var(--line);font-size:12px;color:var(--mut)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--bad)}.dot.on{background:var(--acc2);box-shadow:0 0 8px var(--acc2)}
.sp{flex:1}
.speed{display:flex;align-items:center;gap:8px;color:var(--mut)}
.speed input{width:150px}
button{font:inherit;background:var(--panel2);color:var(--txt);border:1px solid var(--line);padding:8px 13px;border-radius:10px;
 cursor:pointer;transition:.15s}
button:hover{border-color:#3a4b66;background:#202a38}
button.pri{background:var(--acc);border-color:var(--acc);color:#fff}button.pri:hover{filter:brightness(1.1)}
button.stop{background:var(--bad);border-color:var(--bad);color:#fff;font-weight:700;letter-spacing:.5px}
button.sm{padding:5px 9px;font-size:12px;border-radius:8px}
main{display:grid;grid-template-columns:minmax(0,1.35fr) minmax(0,1fr);gap:16px;padding:16px 20px;max-width:1500px;margin:auto}
@media(max-width:1000px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:var(--r);padding:14px 16px;margin-bottom:16px}
.card h2{margin:0 0 10px;font-size:12px;text-transform:uppercase;letter-spacing:1.2px;color:var(--mut);font-weight:600}
.view{position:relative;padding:0;overflow:hidden}
#cv{width:100%;height:520px}
.hud{position:absolute;left:12px;top:12px;display:grid;gap:6px;pointer-events:none}
.hud div{background:#0b0e13b8;border:1px solid var(--line);border-radius:9px;padding:6px 10px;font-size:12px;color:var(--mut)}
.hud b{color:var(--txt);font-variant-numeric:tabular-nums}
.vbtn{position:absolute;right:12px;top:12px;display:flex;gap:6px}
.banner{display:none;position:absolute;left:50%;bottom:14px;transform:translateX(-50%);background:var(--acc);padding:7px 16px;
 border-radius:99px;font-weight:600}
.joint{display:grid;grid-template-columns:86px 1fr 44px 50px 30px;gap:10px;align-items:center;padding:7px 8px;border-radius:10px;border:1px solid transparent}
.joint.mv{background:#4f8cff18;border-color:#4f8cff55}
.joint .n{display:flex;align-items:center;gap:8px;font-weight:600}
.joint .c{width:9px;height:9px;border-radius:3px}
.joint .cmd{text-align:right;color:var(--mut);font-variant-numeric:tabular-nums}
.joint .act{text-align:right;color:var(--acc2);font-variant-numeric:tabular-nums}
input[type=range]{-webkit-appearance:none;appearance:none;height:5px;border-radius:5px;background:var(--line);width:100%}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:17px;height:17px;border-radius:50%;background:var(--acc);
 border:2px solid #0b0e13;box-shadow:0 0 0 1px var(--acc);cursor:pointer}
.lbl{font-size:11px;color:var(--mut);display:grid;gap:3px}
input[type=number],input[type=text]{background:#0f141b;color:var(--txt);border:1px solid var(--line);border-radius:8px;padding:7px 8px;width:100%;font:inherit}
.xyz{display:grid;grid-template-columns:repeat(6,1fr);gap:8px}
.chips{display:flex;gap:6px;flex-wrap:wrap;margin-top:10px}
.chip{padding:5px 10px;border-radius:99px;font-size:12px;background:var(--panel2);border:1px solid var(--line);cursor:pointer;color:var(--mut)}
.chip:hover{color:var(--txt);border-color:#3a4b66}
.rowb{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px}
.pose{padding:10px 11px;border:1px solid var(--line);border-radius:11px;margin-bottom:8px;background:var(--panel2)}
.ph{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.pv{display:grid;grid-template-columns:repeat(6,1fr);gap:8px;margin-top:9px}
.addp{border-top:1px solid var(--line);margin-top:12px;padding-top:12px}
.hb{padding:3px 0;width:28px;height:28px;border-radius:8px;font-size:15px;line-height:1;color:var(--mut)}
.hb:hover{color:var(--txt)}
.pose.on{border-color:var(--acc2);box-shadow:0 0 0 1px var(--acc2) inset}
.pose .i{width:26px;height:26px;border-radius:8px;background:#0f141b;display:grid;place-items:center;color:var(--mut);font-size:12px}
.pose input.pn{background:transparent;border:0;padding:2px 0;font-weight:600;flex:1;min-width:110px;width:auto}
.pose small{color:var(--mut);font-variant-numeric:tabular-nums}
.pose .pa{display:flex;gap:5px;align-items:center;color:var(--mut);font-size:12px;flex-wrap:wrap}
.pose .pa input{width:58px;padding:4px 6px}
.empty{color:var(--mut);padding:8px 2px;font-size:13px}
textarea{width:100%;background:#0f141b;color:var(--txt);border:1px solid var(--line);border-radius:8px;padding:8px;
 font:12px/1.4 ui-monospace,Consolas,monospace;resize:vertical;margin:8px 0}
select{background:#0f141b;color:var(--txt);border:1px solid var(--line);border-radius:8px;padding:7px 8px;font:inherit}
.rep{max-height:360px;overflow:auto;margin:8px 0}
.ri{display:flex;gap:10px;padding:8px 10px;border:1px solid var(--line);border-left-width:4px;border-radius:10px;margin-bottom:6px;background:var(--panel2)}
.ri.ok{border-left-color:var(--acc2)}.ri.warn{border-left-color:var(--warn)}.ri.error{border-left-color:var(--bad)}
.rs{width:16px;text-align:center;font-weight:700}
.ri.ok .rs{color:var(--acc2)}.ri.warn .rs{color:var(--warn)}.ri.error .rs{color:var(--bad)}
.ri small{color:var(--mut);margin-left:6px;font-variant-numeric:tabular-nums}
.iss{font-size:12px;margin-top:3px}.iss.error{color:var(--bad)}.iss.warn{color:var(--warn)}
.sumline{font-weight:600;margin:10px 0 2px}
details.hint summary{cursor:pointer}
details.hint code{background:#0f141b;padding:1px 5px;border-radius:5px}
.ppmap{width:100%;max-width:340px;display:block;margin:0 auto 10px;background:#0f141b;border:1px solid var(--line);border-radius:10px;cursor:crosshair}
.ppg{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-bottom:8px}
.chip input{vertical-align:middle;margin-right:4px}
.gest{display:flex;align-items:center;gap:10px;justify-content:space-between;padding:8px 0;border-bottom:1px solid var(--line)}
.gest .ga{display:flex;gap:6px;flex-shrink:0}
.iss.note{color:var(--mut)}
#toasts{position:fixed;right:16px;bottom:16px;display:grid;gap:8px;z-index:50}
.toast{background:var(--panel2);border:1px solid var(--line);border-left:4px solid var(--acc);padding:10px 14px;border-radius:10px;max-width:360px;
 box-shadow:0 8px 24px #0008}
.toast.err{border-left-color:var(--bad)}.toast.ok{border-left-color:var(--acc2)}
.hint{font-size:12px;color:var(--mut);margin-top:8px}
</style></head><body>
<header>
 <h1>E.V.A. <span style="color:var(--mut);font-weight:400;font-size:13px">robotic arm control</span></h1>
 <div class="pill"><span class="dot" id="dot"></span><span id="conn">Connecting...</span></div>
 <div class="pill" title="Only one motor runs at a time">One motor at a time</div>
 <div class="sp"></div>
 <div class="speed">Speed <input type="range" id="spd" min="5" max="80" value="30"><b id="spdv" style="color:var(--txt);width:26px">30</b></div>
 <button onclick="goHome()">Home</button>
 <button class="stop" onclick="stopAll()">STOP</button>
</header>
<main>
 <section>
  <div class="card view">
   <canvas id="cv"></canvas>
   <div class="hud">
    <div>Wrist pivot &nbsp;<b id="hw">-</b></div>
    <div>Tool tip (est.) &nbsp;<b id="ht">-</b></div>
    <div>Moving &nbsp;<b id="hm">-</b></div>
   </div>
   <div class="vbtn">
    <button class="sm" onclick="resetView()">Reset view</button>
    <button class="sm" onclick="clearTrail()">Clear trail</button>
   </div>
   <div class="banner" id="banner">Playing sequence - press STOP to halt</div>
  </div>
  <div class="card">
   <h2>Teach &amp; replay</h2>
   <div class="rowb">
    <button class="pri" onclick="record()">Record pose</button>
    <button onclick="play(false)">Play</button>
    <button onclick="play(true)">Loop</button>
    <button class="sm" style="margin-left:auto" onclick="if(confirm('Delete all poses?'))post('/api/clear')">Clear all</button>
   </div>
   <div id="poses"></div>
   <div class="addp">
    <div class="lbl">Add a pose by typing joint angles (degrees)</div>
    <div class="pv" id="np"></div>
    <div class="rowb" style="margin-top:10px;margin-bottom:0">
     <button class="pri" onclick="addPose()">Add pose</button>
     <button onclick="fillCurrent()">Use current angles</button>
     <button onclick="fillHome()">Reset to home</button>
    </div>
   </div>
   <div class="addp">
    <div class="lbl">Import many poses at once (paste text or choose a file)</div>
    <textarea id="imptext" rows="5" spellcheck="false" placeholder="Raise hand, 0, 80, 35, 0, 0, -30, 0.8&#10;Wave 1, 0, 80, 35, 30, 0, -30, 0.2"></textarea>
    <div class="rowb" style="margin-bottom:0">
     <label class="chip" style="cursor:pointer">Choose file<input type="file" id="impfile" accept=".json,.csv,.txt,.text" hidden></label>
     <select id="impmode" title="Where the imported poses go"><option value="append">Add to the end</option><option value="replace">Replace current poses</option></select>
     <button class="pri" onclick="impCheck()">Check poses</button>
    </div>
    <div id="impreport"></div>
    <details class="hint"><summary>Accepted formats</summary>
     <p><b>Text / CSV</b>, one pose per line: <code>name, base, shoulder, elbow, roll, pitch, gripper, pause</code>. The name and the pause are optional, so <code>0 80 35 0 0 -30</code> also works. Lines starting with # are ignored.</p>
     <p><b>JSON</b>: a list such as <code>[{"name":"Wave","angles":[0,80,35,30,0,-30],"pause":0.3}]</code>, or a list of 6-number lists.</p>
     <p>Every pose is checked against the joint limits, the table, the base, and the in-between positions on the way from the previous pose. Nothing is loaded until you confirm.</p>
    </details>
   </div>
   <div class="hint">Move the arm with the sliders, then Record. Each pose keeps all 6 joint angles. Replay runs joints one after another.</div>
  </div>
 </section>
 <section>
  <div class="card"><h2>Joints</h2><div id="joints"></div>
   <div class="chips">
    <span class="chip" onclick="cmd('ANGLE 5 -35')">Open gripper</span>
    <span class="chip" onclick="cmd('ANGLE 5 25')">Close gripper</span>
   </div>
  </div>
  <div class="card"><h2>Move to position (cm)</h2>
   <div class="xyz">
    <label class="lbl">X forward<input id="x" type="number" value="18" step="0.5"></label>
    <label class="lbl">Y left<input id="y" type="number" value="0" step="0.5"></label>
    <label class="lbl">Z up<input id="z" type="number" value="14" step="0.5"></label>
    <label class="lbl">Pitch<input id="p" type="number" value="0"></label>
    <label class="lbl">Roll<input id="r" type="number" value="0"></label>
    <label class="lbl">Grip<input id="g" type="number" value="0"></label>
   </div>
   <div class="rowb" style="margin-top:10px;margin-bottom:0"><button class="pri" onclick="goXYZ()">Go</button></div>
   <div class="chips" id="presets"></div>
   <div class="hint">Position of the wrist pivot. The fingertips are 12 to 15 cm beyond it, so keep Z high when pointing down. Pitch 0 = level, -90 = down.</div>
  </div>
  <div class="card"><h2>Pick &amp; place</h2>
   <canvas id="ppmap" class="ppmap" width="300" height="210" title="Top view: the arm base is at the bottom, forward is up"></canvas>
   <div class="rowb" style="justify-content:center;margin-bottom:8px">
    <label class="chip"><input type="radio" name="ppsel" value="cube" checked>Click the map to set the cube</label>
    <label class="chip"><input type="radio" name="ppsel" value="drop">the drop spot</label>
   </div>
   <div class="ppg" id="ppmain"></div>
   <details class="hint"><summary>More settings</summary><div class="ppg" id="ppmore" style="margin-top:8px"></div></details>
   <div class="rowb" style="margin-top:10px;margin-bottom:0">
    <button class="pri" onclick="ppCheck()">Check sequence</button>
    <select id="ppmode" onchange="ppChanged()"><option value="append">Add to the end</option><option value="replace">Replace current poses</option></select>
   </div>
   <div id="ppreport"></div>
   <div class="hint">Positions are where the fingertips go, in cm from the base axis (X forward, Y left). Heights are measured at the fingertips with the gripper closed on the cube. The green area is where the arm can do the whole sequence. Always check the first run at a low speed.</div>
  </div>
  <div class="card"><h2>Gestures</h2>
   <div id="gest"></div>
   <label class="chip" style="display:inline-block;margin-top:10px"><input type="checkbox" id="gauto" checked>Use each gesture's suggested speed</label>
   <div class="hint">Gestures start from the home pose and are checked before they move.</div>
  </div>
 </section>
</main>
<div id="toasts"></div>

<script>
const CFG=__CONFIG__;
const COLORS=["#4f8cff","#35e0a1","#ffb454","#c792ea","#ff7a90","#5dd3f3"];
const $=id=>document.getElementById(id);
const api=(u,b)=>fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})}).then(r=>r.json());
function toast(t,kind){const d=document.createElement('div');d.className='toast '+(kind||'');d.textContent=t;$('toasts').appendChild(d);setTimeout(()=>d.remove(),4200)}
const cmd=c=>api('/api/cmd',{cmd:c}).then(r=>{if(r.error)toast(r.error,'err');return r}).catch(()=>toast('Lost connection to the app','err'));
const post=(u,b)=>api(u,b).then(r=>{if(r.error)toast(r.error,'err');poll();return r});

/* ---------- joints ---------- */
const lastTouch=[0,0,0,0,0,0];let dragging=-1,lastSend=0;
function sendAngle(j,v,force){const n=Date.now();if(!force&&n-lastSend<120)return;lastSend=n;cmd(`ANGLE ${j} ${v}`)}
CFG.names.forEach((nm,j)=>{
 const d=document.createElement('div');d.className='joint';d.id='jr'+j;
 d.innerHTML=`<div class="n"><span class="c" style="background:${COLORS[j]}"></span>${nm}</div>
  <input type="range" id="rg${j}" min="${CFG.limits[j][0]}" max="${CFG.limits[j][1]}" step="1" value="${CFG.home[j]}">
  <div class="cmd" id="cv${j}">${CFG.home[j]}</div><div class="act" id="ac${j}">-</div>
  <button class="hb" title="Return ${nm} to home (${CFG.home[j]})">&#8962;</button>`;
 d.querySelector('.hb').onclick=()=>{lastTouch[j]=0;cmd(`ANGLE ${j} ${CFG.home[j]}`)};
 const s=d.querySelector('input');
 s.oninput=()=>{$('cv'+j).textContent=s.value;lastTouch[j]=Date.now();sendAngle(j,s.value,false)};
 s.onchange=()=>{lastTouch[j]=Date.now();sendAngle(j,s.value,true)};
 s.onpointerdown=()=>dragging=j;s.onpointerup=()=>dragging=-1;
 $('joints').appendChild(d);
});
function releaseSliders(){for(let j=0;j<6;j++)lastTouch[j]=0}
function goHome(){releaseSliders();cmd('HOME')}
function stopAll(){api('/api/cmd',{cmd:'STOP'}).then(()=>{toast('Stopped','ok');poll()})}
document.addEventListener('keydown',e=>{if(e.code==='Space'&&!/INPUT|TEXTAREA/.test(document.activeElement.tagName)){e.preventDefault();stopAll()}});
$('spd').oninput=e=>$('spdv').textContent=e.target.value;
$('spd').onchange=e=>cmd('SPEED '+e.target.value);

/* ---------- XYZ ---------- */
const PRESETS=[["Forward, level",[18,0,14,0,0,0]],["Reach down",[15,0,18,-90,0,-35]],["Left, down",[13,6,18,-90,0,-35]],
 ["Right, down",[13,-6,18,-90,0,-35]],["High forward",[14,0,20,0,0,0]],["Near base, down",[11,0,18,-90,0,-35]]];
PRESETS.forEach(([n,v])=>{const c=document.createElement('span');c.className='chip';c.textContent=n;
 c.onclick=()=>{['x','y','z','p','r','g'].forEach((id,i)=>$(id).value=v[i]);goXYZ()};$('presets').appendChild(c)});
function goXYZ(){
 const f=id=>$(id).value;releaseSliders();
 cmd(`XYZ ${f('x')} ${f('y')} ${f('z')} ${f('p')} ${f('r')} ${f('g')}`).then(r=>{
  if(!r)return;const t=r.reply||'';
  let m=t.match(/ERR joint (\d) needs ([\-\d.]+)/);
  if(m){const j=+m[1];toast(`${CFG.names[j]} would need ${m[2]} deg (limit ${CFG.limits[j][0]} to ${CFG.limits[j][1]}). Try a different position.`,'err')}
  else if(t.startsWith('ERR'))toast('That position is out of reach.','err');
  else if(t==='OK')toast('Moving, one joint at a time','ok');
 });
}

/* ---------- teach ---------- */
function record(){post('/api/record').then(r=>{if(r&&r.ok)toast('Pose recorded','ok')})}
function play(loop,force){
 api('/api/play',{loop,force}).then(r=>{
  if(r.error){if(r.force&&!force){if(confirm(r.error+'\n\nPlay anyway?'))play(loop,true)}else toast(r.error,'err')}
  poll()});
}
CFG.names.forEach((nm,j)=>{const l=document.createElement('label');l.className='lbl';
 l.innerHTML=`${nm}<input type="number" step="0.5" id="np${j}" min="${CFG.limits[j][0]}" max="${CFG.limits[j][1]}" value="${CFG.home[j]}">`;$('np').appendChild(l)});
function addPose(){
 const v=CFG.names.map((_,j)=>parseFloat($('np'+j).value));
 if(v.some(isNaN)){toast('Fill in all 6 angles','err');return}
 post('/api/pose_add',{angles:v}).then(r=>{if(r&&r.ok)toast('Pose added','ok')});
}
function fillCurrent(){if(!actual){toast('No angle data yet','err');return}actual.forEach((a,j)=>$('np'+j).value=a.toFixed(1))}
function fillHome(){CFG.home.forEach((h,j)=>$('np'+j).value=h)}
const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
let impChecked=null;
function impReset(){impChecked=null;$('impreport').innerHTML=''}
$('imptext').oninput=impReset;
$('impfile').onchange=e=>{const f=e.target.files[0];if(!f)return;const r=new FileReader();
 r.onload=()=>{$('imptext').value=r.result;impReset();impCheck()};r.readAsText(f);e.target.value=''};
function impCheck(){
 const text=$('imptext').value;if(!text.trim()){toast('Paste some poses or choose a file first','err');return}
 const mode=$('impmode').value;
 api('/api/import_check',{text,mode}).then(r=>{
  if(r.error){toast(r.error,'err');return}
  impChecked={text,mode};renderReport(r);
 }).catch(()=>toast('Lost connection to the app','err'));
}
function renderReport(r){
 const s=r.summary,ico={ok:'&#10003;',warn:'!',error:'&#10005;'};
 const rows=r.items.map((it,i)=>{
  const ang=it.angles?it.angles.map(a=>+(+a).toFixed(1)).join(' / '):'';
  return `<div class="ri ${it.status}"><div class="rs">${ico[it.status]}</div><div><b>${esc(it.name||('Pose '+(i+1)))}</b>
   <small>${esc(it.src)}${ang?' &middot; '+ang:''}</small>${it.issues.map(x=>`<div class="iss ${x.level}">${esc(x.msg)}</div>`).join('')}</div></div>`}).join('');
 const feasible=s.ok+s.warn;let btns='';
 if(s.error===0)btns=`<button class="pri" onclick="impLoad(false)">Load all ${s.total} poses</button>`;
 else if(feasible>0)btns=`<button class="pri" onclick="impLoad(true)">Load the ${feasible} feasible poses only</button>`;
 const mode=impChecked&&impChecked.mode==='replace'?' Your current poses will be replaced.':'';
 $('impreport').innerHTML=`<div class="sumline">${s.total} checked: ${s.ok} OK, ${s.warn} with warnings, ${s.error} not feasible</div>
  <div class="rep">${rows}</div><div class="rowb" style="margin-bottom:0">${btns}</div>
  <div class="hint">${s.error?'Fix the marked lines and check again, or load only the feasible ones. ':''}Nothing is loaded until you press a button.${mode}</div>`;
}
function impLoad(only){
 if(!impChecked)return;
 if(impChecked.mode==='replace'&&!confirm('Replace all current poses?'))return;
 api('/api/import',{text:impChecked.text,mode:impChecked.mode,only_feasible:only}).then(r=>{
  if(r.error){toast(r.error,'err');if(r.items)renderReport(r);return}
  toast(`Loaded ${r.loaded} pose${r.loaded===1?'':'s'}`+(r.skipped?` (${r.skipped} skipped)`:''),'ok');
  $('imptext').value='';impReset();poll();
 }).catch(()=>toast('Lost connection to the app','err'));
}
/* ---------- gestures ---------- */
function gestPlay(g,force){
 if(!force&&$('gauto').checked){$('spd').value=g.speed;$('spdv').textContent=g.speed;cmd('SPEED '+g.speed)}
 api('/api/gesture/'+g.id,{action:'play',force}).then(r=>{
  if(r.error){if(r.force&&!force){if(confirm(r.error+'\n\nPlay anyway?'))gestPlay(g,true)}else toast(r.error,'err')}
  poll()});
}
fetch('/api/gestures').then(r=>r.json()).then(list=>{
 list.forEach(g=>{
  const d=document.createElement('div');d.className='gest';
  d.innerHTML=`<div><b>${esc(g.name)}</b><div class="hint" style="margin:2px 0 0">${esc(g.desc)} <span>(speed ${g.speed})</span></div></div>
   <div class="ga"><button class="pri sm">Play</button><button class="sm" title="Add these poses to the end of the teach list">Add</button></div>`;
  const [pl,ad]=d.querySelectorAll('button');
  pl.onclick=()=>gestPlay(g,false);
  ad.onclick=()=>post('/api/gesture/'+g.id,{action:'add'}).then(r=>{if(r&&r.ok)toast(`Added ${r.loaded} poses to the teach list`,'ok')});
  $('gest').appendChild(d);
 });
}).catch(()=>{});

/* ---------- pick & place ---------- */
const ICO={ok:'&#10003;',warn:'!',error:'&#10005;'};
const PPF=[['cx','Cube X',14,0.5],['cy','Cube Y',-4,0.5],['dx','Drop X',14,0.5],['dy','Drop Y',6,0.5],
 ['grabZ','Grab height',2,0.5],['hoverZ','Hover height',8,0.5],['grabDeg','Grip angle',12,1],['openDeg','Open angle',-35,1]];
const PPM=[['placeZ','Drop height','',0.5],['roll','Roll',0,1],['maxTilt','Max tilt',20,1],['steps','Smooth steps',2,1],
 ['tx','Trim X',0,0.1],['ty','Trim Y',0,0.1],['tz','Trim Z',0,0.1]];
let ppRanges=[],ppTimer=0;const MAP={bx:150,by:205,s:7.5};
[[PPF,'ppmain'],[PPM,'ppmore']].forEach(([list,box])=>list.forEach(([k,lb,df,st])=>{
 const l=document.createElement('label');l.className='lbl';
 l.innerHTML=`${lb}<input type="number" id="pp_${k}" step="${st}" value="${df}"${k==='placeZ'?' placeholder="same as grab"':''}>`;
 l.querySelector('input').oninput=ppChanged;$(box).appendChild(l)}));
try{const sv=JSON.parse(localStorage.getItem('armPP')||'{}');[...PPF,...PPM].forEach(([k])=>{if(sv[k]!==undefined)$('pp_'+k).value=sv[k]});
 if(sv.mode)$('ppmode').value=sv.mode}catch(e){}
function ppParams(){
 const o={mode:$('ppmode').value};
 [...PPF,...PPM].forEach(([k])=>{const v=$('pp_'+k).value;if(v!=='')o[k]=parseFloat(v)});return o;
}
function ppSave(){try{const o={mode:$('ppmode').value};[...PPF,...PPM].forEach(([k])=>o[k]=$('pp_'+k).value);localStorage.setItem('armPP',JSON.stringify(o))}catch(e){}}
function ppChanged(){ppSave();$('ppreport').innerHTML='';clearTimeout(ppTimer);ppTimer=setTimeout(ppZone,250);drawMap()}
function ppZone(){api('/api/pp_zone',ppParams()).then(r=>{ppRanges=r.error?[]:r.ranges;drawMap()}).catch(()=>{})}
const mp=(r,t)=>{const a=t*Math.PI/180;return[MAP.bx-r*Math.sin(a)*MAP.s,MAP.by-r*Math.cos(a)*MAP.s]};
function inZone(x,y){const r=Math.hypot(x,y),a=Math.abs(Math.atan2(y,x)*180/Math.PI);return a<=80&&ppRanges.some(([a0,a1])=>r>=a0&&r<=a1)}
function drawMap(){
 const cv=$('ppmap'),g=cv.getContext('2d');g.clearRect(0,0,cv.width,cv.height);
 g.font='10px system-ui';g.lineWidth=1;
 for(let r=5;r<=25;r+=5){g.strokeStyle='#1d2633';g.beginPath();for(let t=-90;t<=90;t+=3){const[x,y]=mp(r,t);t===-90?g.moveTo(x,y):g.lineTo(x,y)}g.stroke();
  g.fillStyle='#4a586d';const[lx,ly]=mp(r,0);g.fillText(r+' cm',lx+3,ly-2)}
 ppRanges.forEach(([r0,r1])=>{g.beginPath();for(let t=-80;t<=80;t+=4){const[x,y]=mp(r1,t);t===-80?g.moveTo(x,y):g.lineTo(x,y)}
  for(let t=80;t>=-80;t-=4){const[x,y]=mp(r0,t);g.lineTo(x,y)}g.closePath();g.fillStyle='rgba(53,224,161,.16)';g.fill();g.strokeStyle='rgba(53,224,161,.5)';g.stroke()});
 g.fillStyle='#3a4352';g.beginPath();g.arc(MAP.bx,MAP.by,5.5*MAP.s,Math.PI,2*Math.PI);g.fill();
 const f=id=>parseFloat($('pp_'+id).value),cx=f('cx'),cy=f('cy'),dx=f('dx'),dy=f('dy');
 g.fillStyle=ppRanges.length?'#35e0a1':'#ffb454';
 g.fillText(ppRanges.length?'usable: '+ppRanges.map(r=>r[0]+' to '+r[1]+' cm').join(', '):'no usable zone with these settings',6,12);
 if([cx,cy,dx,dy].some(isNaN))return;
 const [ax,ay]=[MAP.bx-cy*MAP.s,MAP.by-cx*MAP.s],[bx,by]=[MAP.bx-dy*MAP.s,MAP.by-dx*MAP.s];
 g.setLineDash([4,3]);g.strokeStyle='#8593a6';g.beginPath();g.moveTo(ax,ay);g.lineTo(bx,by);g.stroke();g.setLineDash([]);
 g.fillStyle=inZone(cx,cy)?'#4f8cff':'#ff5d5d';g.fillRect(ax-5,ay-5,10,10);g.fillStyle='#e8ecf2';g.fillText('cube',ax+8,ay+3);
 g.fillStyle=inZone(dx,dy)?'#35e0a1':'#ff5d5d';g.beginPath();g.arc(bx,by,5.5,0,7);g.fill();g.fillStyle='#e8ecf2';g.fillText('drop',bx+9,by+3);
}
$('ppmap').onclick=e=>{
 const b=e.target.getBoundingClientRect(),px=(e.clientX-b.left)*(e.target.width/b.width),py=(e.clientY-b.top)*(e.target.height/b.height);
 const x=Math.round((MAP.by-py)/MAP.s*2)/2,y=Math.round((MAP.bx-px)/MAP.s*2)/2;
 if(document.querySelector('input[name=ppsel]:checked').value==='cube'){$('pp_cx').value=x;$('pp_cy').value=y}else{$('pp_dx').value=x;$('pp_dy').value=y}
 ppChanged();
};
function renderPP(r){
 const s=r.summary,ok=s.error===0;
 const rows=r.items.map(it=>`<div class="ri ${it.status}"><div class="rs">${ICO[it.status]}</div><div><b>${esc(it.name||'')}</b>
  <small>${it.angles?it.angles.map(a=>+(+a).toFixed(0)).join(' / '):''}</small>${it.note?`<div class="iss note">${esc(it.note)}</div>`:''}
  ${it.issues.map(x=>`<div class="iss ${x.level}">${esc(x.msg)}</div>`).join('')}</div></div>`).join('');
 $('ppreport').innerHTML=`<div class="sumline">${s.total} steps: ${s.ok} OK, ${s.warn} with warnings, ${s.error} not feasible</div><div class="rep">${rows}</div>`+
  (ok?`<div class="rowb" style="margin-bottom:0"><button class="pri" onclick="ppLoad()">Load into sequence</button><button onclick="ppPlay(false)">Play now</button></div>
   <div class="hint">Nothing moves until you press a button. Play now runs the sequence once without changing your teach list.</div>`
  :`<div class="hint">Fix the marked steps (or move the cube or drop spot into the green area) and check again. Nothing was loaded.</div>`);
}
function ppCheck(){api('/api/pp_check',ppParams()).then(r=>{if(r.error){toast(r.error,'err');return}renderPP(r)}).catch(()=>toast('Lost connection to the app','err'))}
function ppLoad(){
 api('/api/pp_load',ppParams()).then(r=>{
  if(r.error){toast(r.error,'err');if(r.items)renderPP(r);return}
  toast(`Loaded ${r.loaded} steps into the teach list`,'ok');$('ppreport').innerHTML='';poll()});
}
function ppPlay(force){
 api('/api/pp_play',{...ppParams(),force}).then(r=>{
  if(r.error){if(r.items)renderPP(r);
   if(r.force&&!force){if(confirm(r.error+'\n\nPlay anyway?'))ppPlay(true)}else toast(r.error,'err')}
  poll()});
}
ppZone();drawMap();

let lastPoses='';
function renderPoses(poses,idx){
 const key=JSON.stringify(poses)+idx;if(key===lastPoses)return;lastPoses=key;
 const el=$('poses');el.innerHTML='';
 if(!poses.length){el.innerHTML='<div class="empty">No poses yet.</div>';return}
 poses.forEach((p,i)=>{
  const d=document.createElement('div');d.className='pose'+(i===idx?' on':'');
  const ins=p.angles.map((a,j)=>`<label class="lbl">${CFG.names[j]}<input type="number" step="0.5" min="${CFG.limits[j][0]}" max="${CFG.limits[j][1]}" value="${a}"></label>`).join('');
  d.innerHTML=`<div class="ph"><div class="i">${i+1}</div><input type="text" class="pn" value="${p.name.replace(/"/g,'&quot;')}">
   <div class="pa">pause <input type="number" class="pp" min="0" step="0.5" value="${p.pause}">s
    <button class="sm">Go</button><button class="sm" title="Overwrite with the arm's current angles">Set to current</button>
    <button class="sm">&uarr;</button><button class="sm">&darr;</button><button class="sm" style="color:var(--bad)">&times;</button></div></div>
   <div class="pv">${ins}</div>`;
  const b=d.querySelectorAll('.pa button');
  d.querySelector('.pn').onchange=e=>post('/api/pose/'+i,{name:e.target.value});
  d.querySelector('.pp').onchange=e=>post('/api/pose/'+i,{pause:e.target.value});
  d.querySelectorAll('.pv input').forEach(inp=>inp.onchange=()=>{
   const v=[...d.querySelectorAll('.pv input')].map(x=>parseFloat(x.value));
   if(v.some(isNaN)){toast('Enter all 6 angles','err');return}
   post('/api/pose/'+i,{angles:v});
  });
  b[0].onclick=()=>{releaseSliders();post('/api/pose/'+i,{go:true})};
  b[1].onclick=()=>post('/api/pose/'+i,{from_current:true}).then(r=>{if(r&&r.ok)toast('Pose updated from the arm','ok')});
  b[2].onclick=()=>post('/api/pose/'+i,{move:-1});
  b[3].onclick=()=>post('/api/pose/'+i,{move:1});
  b[4].onclick=()=>post('/api/pose/'+i,{delete:true});
  el.appendChild(d);
 });
}

/* ---------- polling ---------- */
let actual=null,shown=null,movingJ=-1,firstSync=true;
function poll(){
 return fetch('/api/state').then(r=>r.json()).then(s=>{
  $('dot').classList.toggle('on',!!s.connected);$('conn').textContent=s.connected?'Connected':'Disconnected';
  $('banner').style.display=s.playing?'block':'none';
  $('banner').textContent=s.playing?'Playing '+(s.name||'sequence')+' - press STOP to halt':'';
  movingJ=s.moving;
  if(s.angles){
   actual=s.angles;if(!shown)shown=s.angles.slice();
   s.angles.forEach((a,j)=>{
    $('ac'+j).textContent=a.toFixed(1);
    const stale=Date.now()-lastTouch[j]>4000&&dragging!==j;
    if(stale){$('rg'+j).value=Math.round(a);$('cv'+j).textContent=Math.round(a)}
    $('jr'+j).classList.toggle('mv',j===movingJ);
   });
   if(firstSync){firstSync=false;cmd('SPEED '+$('spd').value)}
  }
  renderPoses(s.poses,s.idx);
 }).catch(()=>{$('dot').classList.remove('on');$('conn').textContent='App offline'});
}
setInterval(poll,250);

/* ---------- 3D digital twin ---------- */
const L=CFG.links;
let scene,camera,renderer,controls,J0,J1,J2,J3,J4,fingerL,fingerR,wristObj,tipObj;
const capMats=[],trail=[];let trailLine,lastTip=new THREE.Vector3(),hudT=0;
const M={};
function mk(c,r,m){return new THREE.MeshStandardMaterial({color:c,roughness:r,metalness:m})}
function addMesh(geo,mat,x,y,z,parent,rot){const o=new THREE.Mesh(geo,mat);o.position.set(x,y,z);if(rot)o.rotation.set(rot[0],rot[1],rot[2]);
 o.castShadow=true;o.receiveShadow=true;parent.add(o);return o}
const box=(w,h,d,m,x,y,z,p)=>addMesh(new THREE.BoxGeometry(w,h,d),m,x,y,z,p);
const cylZ=(r,len,m,x,y,z,p,seg)=>addMesh(new THREE.CylinderGeometry(r,r,len,seg||40),m,x,y,z,p,[Math.PI/2,0,0]);
const cylY=(r,len,m,x,y,z,p,seg)=>addMesh(new THREE.CylinderGeometry(r,r,len,seg||40),m,x,y,z,p);
function servo(p,x,y,z,big){ // body with mounting ears, horn on top
 const w=big?4.1:2.3,h=big?4.0:2.2,d=big?2.0:1.2;
 box(w,h,d,M.servo,x,y,z,p);box(w+1.3,0.25,d,M.servo,x,y+h*0.2,z,p);
 box(w*0.5,0.3,d*0.9,M.label,x,y+h/2+0.01,z,p);
}
function label(text,x,y,z){const c=document.createElement('canvas');c.width=256;c.height=64;const g=c.getContext('2d');
 g.fillStyle='#8593a6';g.font='bold 30px system-ui';g.textAlign='center';g.fillText(text,128,42);
 const s=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(c),transparent:true}));s.position.set(x,y,z);s.scale.set(8,2,1);scene.add(s)}

function build(){
 const cv=$('cv');
 scene=new THREE.Scene();scene.background=new THREE.Color(0x0b0e13);scene.fog=new THREE.Fog(0x0b0e13,70,150);
 camera=new THREE.PerspectiveCamera(42,cv.clientWidth/cv.clientHeight,0.1,400);camera.position.set(38,26,38);
 renderer=new THREE.WebGLRenderer({canvas:cv,antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));
 renderer.setSize(cv.clientWidth,cv.clientHeight,false);renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
 controls=new THREE.OrbitControls(camera,cv);controls.enableDamping=true;controls.target.set(6,11,0);controls.maxPolarAngle=Math.PI*0.495;
 scene.add(new THREE.HemisphereLight(0xbfd4ff,0x1a1f2a,0.75));
 const sun=new THREE.DirectionalLight(0xffffff,0.9);sun.position.set(26,46,22);sun.castShadow=true;
 sun.shadow.mapSize.set(2048,2048);Object.assign(sun.shadow.camera,{left:-40,right:40,top:40,bottom:-40,near:1,far:140});scene.add(sun);
 const rim=new THREE.DirectionalLight(0x4f8cff,0.35);rim.position.set(-30,18,-26);scene.add(rim);

 const floor=new THREE.Mesh(new THREE.PlaneGeometry(260,260),mk(0x10151d,0.9,0));floor.rotation.x=-Math.PI/2;floor.receiveShadow=true;scene.add(floor);
 const grid=new THREE.GridHelper(80,40,0x2b3a52,0x1a2230);grid.position.y=0.02;scene.add(grid);
 label('FRONT (+X)',26,0.1,0);

 Object.assign(M,{black:mk(0x1d2026,0.55,0.1),white:mk(0xeceff3,0.5,0.05),servo:mk(0x2c323c,0.45,0.35),
  label:mk(0x59636f,0.6,0.2),horn:mk(0xc7ced6,0.3,0.85),screw:mk(0x8f99a4,0.3,0.9),pad:mk(0x3a3f48,0.9,0)});
 for(let j=0;j<6;j++)capMats[j]=new THREE.MeshStandardMaterial({color:0x2b3340,roughness:0.35,metalness:0.7,emissive:0x000000});

 // ---- fixed base ----
 cylY(5.4,6.0,M.white,0,3.0,0,scene,64);
 cylY(5.5,0.6,M.black,0,6.3,0,scene,64);
 for(let a=0;a<6;a++){const t=a*Math.PI/3;cylY(0.3,0.25,M.screw,Math.cos(t)*4.5,0.13,Math.sin(t)*4.5,scene,12)}

 // ---- J0: base turntable ----
 J0=new THREE.Group();scene.add(J0);
 cylY(4.6,0.9,M.black,0,7.05,0,J0,64);
 // shoulder yoke plates + servo
 [-1,1].forEach(s=>{
  box(3.4,H(0)-6.6+2.4,0.8,M.black,0,(6.6+H(0)+2.4)/2,s*3.3,J0);
  cylZ(2.2,0.8,M.black,0,H(0),s*3.3,J0,48);
 });
 box(5.2,0.9,6.8,M.black,0,7.4,0,J0);
 servo(J0,-0.9,H(0),5.0,true);
 cylZ(1.55,0.5,capMats[1],0,H(0),-3.95,J0,40);cylZ(0.5,0.6,M.screw,0,H(0),-4.0,J0,16);

 // ---- J1: shoulder ----
 J1=new THREE.Group();J1.position.set(0,H(0),0);J0.add(J1);
 box(4.4,L.L1*0.5,4.8,M.black,0,L.L1*0.25+0.8,0,J1);
 box(3.6,L.L1*0.5,4.4,M.black,0,L.L1*0.75-0.6,0,J1);
 cylZ(2.3,4.8,M.black,0,0.2,0,J1,48);cylZ(2.0,4.4,M.black,0,L.L1,0,J1,48);
 cylZ(1.45,0.5,capMats[1],0,0,2.55,J1,40);
 servo(J1,0.6,L.L1-0.3,3.7,true);   // elbow servo on the upper arm
 // fillet detail strip
 box(0.4,L.L1*0.8,5.0,M.white,2.1,L.L1*0.5,0,J1).scale.set(1,1,1);

 // ---- J2: elbow ----
 J2=new THREE.Group();J2.position.set(0,L.L1,0);J1.add(J2);
 cylZ(1.65,0.5,capMats[2],0,0,-2.55,J2,40);cylZ(0.45,0.6,M.screw,0,0,-2.6,J2,16);
 cylZ(1.8,3.2,M.black,0,0,0,J2,40);
 box(3.0,L.L2-2.4,3.0,M.black,0,L.L2/2-0.2,0,J2);
 box(0.35,L.L2-3.2,3.1,M.white,1.65,L.L2/2-0.4,0,J2);
 box(2.8,2.2,2.8,M.servo,0,L.L2-1.4,0,J2);   // roll servo housing

 // ---- J3: roll about forearm axis ----
 J3=new THREE.Group();J3.position.set(0,L.L2,0);J2.add(J3);
 cylY(1.6,0.5,capMats[3],0,-0.5,0,J3,40);
 [-1,1].forEach(s=>box(3.0,3.6,0.55,M.white,0,0.5,s*1.8,J3));
 box(3.0,0.6,3.9,M.white,0,-1.1,0,J3);
 cylZ(0.95,0.4,capMats[4],0,0,2.25,J3,32);cylZ(0.35,0.5,M.screw,0,0,2.35,J3,12);
 servo(J3,-0.2,0.4,-3.4,false);               // pitch servo, rides with roll

 // ---- J4: pitch + gripper ----
 J4=new THREE.Group();J3.add(J4);
 wristObj=new THREE.Object3D();J4.add(wristObj);
 box(3.0,L.L3,2.6,M.white,0,L.L3/2,0,J4);
 box(2.2,1.0,0.35,M.black,0,L.L3-0.3,1.45,J4);
 servo(J4,0,L.L3-0.6,-1.9,false);              // gripper servo
 fingerL=new THREE.Group();fingerL.position.set(-1.1,L.L3,0);J4.add(fingerL);
 fingerR=new THREE.Group();fingerR.position.set(1.1,L.L3,0);J4.add(fingerR);
 [fingerL,fingerR].forEach((g,k)=>{const s=k?1:-1;
  box(0.55,L.GRIP*0.9,1.9,M.black,0,L.GRIP*0.45,0,g);
  box(0.3,L.GRIP*0.45,1.7,M.pad,-s*0.4,L.GRIP*0.72,0,g);
  box(0.55,1.2,1.9,M.black,-s*0.3,L.GRIP*0.95,0,g);
  cylZ(0.45,2.0,M.horn,0,0.2,0,g,20);
 });
 tipObj=new THREE.Mesh(new THREE.SphereGeometry(0.28,16,16),new THREE.MeshBasicMaterial({color:0x35e0a1}));
 tipObj.position.set(0,L.L3+L.GRIP,0);J4.add(tipObj);

 trailLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x35e0a1,transparent:true,opacity:0.7}));
 trailLine.frustumCulled=false;scene.add(trailLine);
 addEventListener('resize',resize);
}
function H(_){return L.H0}
function resize(){const cv=$('cv');camera.aspect=cv.clientWidth/cv.clientHeight;camera.updateProjectionMatrix();renderer.setSize(cv.clientWidth,cv.clientHeight,false)}
function resetView(){camera.position.set(38,26,38);controls.target.set(6,11,0)}
function clearTrail(){trail.length=0;trailLine.geometry.setFromPoints([])}

const rad=d=>d*Math.PI/180;
function applyPose(a){
 J0.rotation.y=rad(a[0]);
 J1.rotation.z=-rad(90-a[1]);
 J2.rotation.z=-rad(a[2]);
 J3.rotation.y=rad(a[3]);
 J4.rotation.z=-rad(a[4]);
 const G=CFG.grip,open=Math.max(0,Math.min(G.phiMax,G.phiMax*(G.closedDeg-a[5])/(G.closedDeg-G.openDeg)));
 fingerL.rotation.z=rad(open);fingerR.rotation.z=-rad(open);
 tipObj.position.y=L.L3+L.GRIP*Math.cos(rad(open));
}
const v1=new THREE.Vector3(),v2=new THREE.Vector3();
function fmt(v){return `${v.x.toFixed(1)}, ${(-v.z).toFixed(1)}, ${v.y.toFixed(1)}`}
function frame(t){
 requestAnimationFrame(frame);
 if(shown&&actual){for(let j=0;j<6;j++)shown[j]+=(actual[j]-shown[j])*0.22;applyPose(shown)}
 capMats.forEach((m,j)=>m.emissive.setHex(j===movingJ?0xff7a1a:0x000000));
 controls.update();
 scene.updateMatrixWorld(true);
 wristObj.getWorldPosition(v1);tipObj.getWorldPosition(v2);
 if(v2.distanceTo(lastTip)>0.15){lastTip.copy(v2);trail.push(v2.clone());if(trail.length>500)trail.shift();trailLine.geometry.setFromPoints(trail)}
 if(t-hudT>120){hudT=t;$('hw').textContent=fmt(v1)+' cm';$('ht').textContent=fmt(v2)+' cm';
  $('hm').textContent=movingJ>=0?CFG.names[movingJ]:'none'}
 renderer.render(scene,camera);
}
build();requestAnimationFrame(frame);poll();
</script></body></html>"""


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        sys.exit("No serial port found. Pass it as an argument, e.g. python eva_control.py COM3")
    print("Connecting to", port, "(wait about 4 seconds)...")
    arm = Arm(port)

    def monitor():  # keeps the angle readings fresh for the dashboard
        while True:
            ang, moving = arm.status()
            arm.cache = {"angles": ang, "moving": moving}
            time.sleep(0.2)

    threading.Thread(target=monitor, daemon=True).start()
    if os.path.exists(POSE_FILE):
        with open(POSE_FILE) as f:
            poses = json.load(f)
    print("Ready: open http://localhost:5000")
    app.run(host="127.0.0.1", port=5000, threaded=True)
