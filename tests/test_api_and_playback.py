"""Command validation, speed limiting and the playback safety check."""
import time


def test_only_known_commands_with_numeric_arguments_are_forwarded(client, eva):
    assert client.post("/api/cmd", json={"cmd": "ANGLE 1 70"}).status_code == 200
    for bad in ("RAW 0 1500", "ANGLE 1 70; rm -rf", "FORMAT C:", ""):
        assert client.post("/api/cmd", json={"cmd": bad}).status_code == 400
    assert "ANGLE 1 70" in eva.arm.log


def test_speed_is_capped(client, eva):
    client.post("/api/cmd", json={"cmd": "SPEED 500"})
    assert eva.arm.log[-1] == f"SPEED {eva.MAX_SPEED}"


def test_stop_reaches_the_arduino_and_stops_playback(client, eva):
    eva.poses.append({"name": "A", "angles": [0, 90, 0, 0, 0, 0], "pause": 5})
    client.post("/api/play", json={})
    time.sleep(0.1)
    assert client.post("/api/cmd", json={"cmd": "STOP"}).status_code == 200
    time.sleep(0.2)
    assert "STOP" in eva.arm.log
    assert client.get("/api/state").get_json()["playing"] is False


def test_unsafe_sequence_is_refused_unless_forced(client, eva):
    eva.poses.append({"name": "Dive", "angles": [0, 25, 110, 0, 0, 0], "pause": 0.1})
    r = client.post("/api/play", json={})
    assert r.status_code == 409 and r.get_json()["force"] is True
    assert client.post("/api/play", json={"force": True}).get_json()["ok"] is True


def test_motion_is_blocked_while_playing(client, eva):
    eva.poses.append({"name": "A", "angles": [0, 90, 0, 0, 0, 0], "pause": 5})
    client.post("/api/play", json={})
    time.sleep(0.1)
    assert client.post("/api/cmd", json={"cmd": "ANGLE 1 70"}).status_code == 409


def test_pose_editing_clamps_to_the_limits(client, eva):
    client.post("/api/pose_add", json={"angles": [200, 10, 150, -120, 0, 50]})
    assert eva.poses[0]["angles"] == [80, 20, 110, -80, 0.0, 40]
    assert client.post("/api/pose_add", json={"angles": [1, 2, 3]}).status_code == 400
