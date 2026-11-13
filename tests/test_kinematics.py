"""The forward and inverse kinematics must agree with each other and with the numbers measured on the real arm."""
import math


def test_ik_matches_the_reference_pose_measured_on_the_arm(eva):
    # XYZ 18 0 14 0 0 0 gave shoulder 39.2, elbow 59.9, pitch -20.7 on the real arm
    a = eva.solve_wrist(18.0, 14.0, 0.0, 0.0, 0.0, 0.0)
    assert [round(v, 1) for v in a] == [0.0, 39.2, 59.9, 0.0, -20.7, 0.0]


def test_forward_kinematics_round_trip(eva):
    for r in (10, 13, 16, 19):
        for z in (12, 16, 20):
            try:
                a = eva.solve_wrist(r, z, 0.0, 0.0, 0.0, -10.0)
            except ValueError:
                continue                                # unreachable points are skipped
            _, wrist, _ = eva.chain(a)
            assert math.isclose(wrist[0], r, abs_tol=0.01) and math.isclose(wrist[1], z, abs_tol=0.01)


def test_home_pose_height_is_the_sum_of_the_links(eva):
    _, wrist, _ = eva.chain([0, 90, 0, 0, 0, 0])
    k = eva.CONFIG["links"]
    assert math.isclose(wrist[1], k["H0"] + k["L1"] + k["L2"], abs_tol=1e-6)


def test_fingertip_distance_matches_the_gripper_measurements(eva):
    assert math.isclose(eva.tool_len(25), 15.2, abs_tol=0.05)     # closed
    assert math.isclose(eva.tool_len(-35), 11.8, abs_tol=0.1)     # fully open


def test_unreachable_points_raise(eva):
    import pytest
    with pytest.raises(ValueError):
        eva.solve_wrist(30, 10, 0, 0, 0, 0)
