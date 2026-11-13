"""Pick and place generator, usable zone, and the gesture library."""


def test_default_pick_and_place_is_feasible(client):
    r = client.post("/api/pp_check", json={}).get_json()
    assert r["summary"]["error"] == 0 and r["summary"]["total"] == 13
    assert [i["name"] for i in r["items"]][0] == "Open gripper"
    assert r["items"][-1]["name"] == "Return home"


def test_generated_poses_keep_a_margin_from_the_joint_limits(eva):
    items = eva.gen_pick_place(eva.pp_params({}))
    for it in items:
        if it["name"] in ("Open gripper", "Return home"):
            continue
        for v, (lo, hi) in zip(it["angles"], eva.CONFIG["limits"]):
            assert lo + 2.9 <= v <= hi - 2.9


def test_unreachable_cube_is_reported_not_loaded(client, eva):
    r = client.post("/api/pp_check", json={"cx": 26, "cy": 0}).get_json()
    assert r["summary"]["error"] > 0
    assert client.post("/api/pp_load", json={"cx": 26, "cy": 0}).status_code == 400
    assert eva.poses == []


def test_too_high_hover_suggests_a_lower_one(client):
    r = client.post("/api/pp_check", json={"hoverZ": 20, "cx": 15}).get_json()
    msgs = " ".join(i["issues"][0]["msg"] for i in r["items"] if i["status"] == "error")
    assert "highest hover" in msgs.lower()


def test_usable_zone_contains_the_default_spots(client):
    zone = client.post("/api/pp_zone", json={}).get_json()["ranges"]
    assert zone and any(a <= 14.5 <= b for a, b in zone)


def test_load_pick_and_place_into_the_teach_list(client, eva):
    assert client.post("/api/pp_load", json={"mode": "replace"}).get_json()["loaded"] == 13
    assert len(eva.poses) == 13


def test_every_gesture_is_safe_from_home(eva):
    for g in eva.GESTURES:
        items = eva.check_items(eva.gesture_items(g), eva.CONFIG["home"])
        assert not [i for i in items if i["status"] == "error"], g["name"]


def test_gesture_can_be_added_to_the_teach_list(client, eva):
    r = client.post("/api/gesture/bow", json={"action": "add"}).get_json()
    assert r["ok"] and len(eva.poses) == r["loaded"]
