"""Feasibility checker and bulk importer."""
import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]


def report(client, text, mode="replace"):
    return client.post("/api/import_check", json={"text": text, "mode": mode}).get_json()


def test_wave_file_is_fully_feasible(client):
    r = report(client, (ROOT / "poses" / "hi_wave_poses.json").read_text())
    assert r["summary"] == {"total": 8, "ok": 8, "warn": 0, "error": 0}


def test_every_example_in_the_poses_folder_is_feasible(client):
    files = [f for f in (ROOT / "poses").iterdir() if f.suffix in (".json", ".txt")]
    assert files
    for f in files:
        r = report(client, f.read_text())
        assert r["summary"]["error"] == 0, f.name


def test_text_format_with_header_comments_and_optional_fields(client):
    text = "# comment\nname,base,shoulder,elbow,roll,pitch,gripper,pause\nRaise,0,80,35,0,0,-30,0.8\n0 80 35 30 0 -30\n"
    r = report(client, text)
    assert r["summary"]["total"] == 2 and r["summary"]["error"] == 0
    assert r["items"][0]["name"] == "Raise" and r["items"][0]["pause"] == 0.8
    assert r["items"][1]["pause"] == 0.5


def test_bad_lines_are_reported_with_their_line_number(client):
    r = report(client, "A,0,90,0,0,0,0\nB,0,90,0,0\nC,0,abc,0,0,0,0")
    assert [i["status"] for i in r["items"]] == ["ok", "error", "error"]
    assert r["items"][1]["src"] == "line 2" and r["items"][2]["src"] == "line 3"


def test_joint_limits_are_enforced_not_clamped(client):
    r = report(client, "Too far,0,130,0,0,0,0")
    assert r["items"][0]["status"] == "error"
    assert "outside" in r["items"][0]["issues"][0]["msg"]


def test_fingertips_below_the_table_are_rejected(client):
    r = report(client, "Home,0,90,0,0,0,0\nDive,0,25,110,0,0,0")
    assert r["items"][1]["status"] == "error"
    assert "below the table" in r["items"][1]["issues"][0]["msg"]


def test_in_between_positions_are_checked_too(eva):
    # Both poses are fine on their own, but moving the joints one at a time (shoulder first)
    # drags the gripper tip 2 cm below the table on the way from A to B.
    a = [-20.0, 77.0, 92.0, 40.0, 71.0, -3.0]
    b = [24.0, 40.0, 68.0, 43.0, 23.0, 17.0]
    assert not [h for h in eva.hazards(a) if h[0] == "error"]
    assert not [h for h in eva.hazards(b) if h[0] == "error"]
    items = [eva._item("b", "B", b, 0.5)]
    eva.check_items(items, a)
    assert items[0]["status"] == "error"
    assert any("while the shoulder moves" in i["msg"].lower() for i in items[0]["issues"])


def test_load_everything_or_only_the_feasible_poses(client, eva):
    text = "A,0,90,0,0,0,0\nB,0,130,0,0,0,0\n0 85 5 0 0 0"
    r = client.post("/api/import", json={"text": text, "mode": "replace"})
    assert r.status_code == 400 and len(eva.poses) == 0          # nothing is loaded when something is bad
    r = client.post("/api/import", json={"text": text, "mode": "replace", "only_feasible": True})
    assert r.get_json()["loaded"] == 2 and len(eva.poses) == 2


def test_invalid_json_is_reported(client):
    r = report(client, "[{'name': 1}")
    assert r["summary"]["error"] == 1
