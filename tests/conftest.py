"""Shared test setup: loads software/eva_control.py and gives every test a fake Arduino."""
import importlib.util
import pathlib
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_module():
    spec = importlib.util.spec_from_file_location("eva_control", ROOT / "software" / "eva_control.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FakeArm:
    """Stands in for the serial connection: records commands and pretends to be idle."""
    ok = True

    def __init__(self):
        self.cache = {"angles": [0, 90, 0, 0, 0, 0], "moving": -1}
        self.log = []
        self._n = 0

    def send(self, line, reply=False, accept=None):
        self.log.append(line)
        return "OK" if line.startswith("XYZ") else ""

    def status(self):
        return self.cache["angles"], -1

    def busy(self):
        self._n += 1
        return self._n % 3 != 0

    def wait_idle(self, stop, timeout=120):
        while self.busy() and not stop.is_set():
            time.sleep(0.002)


@pytest.fixture()
def eva(tmp_path):
    mod = load_module()
    mod.arm = FakeArm()
    mod.POSE_FILE = str(tmp_path / "poses.json")
    mod.poses.clear()
    yield mod
    mod.player["stop"].set()
    time.sleep(0.05)


@pytest.fixture()
def client(eva):
    return eva.app.test_client()
