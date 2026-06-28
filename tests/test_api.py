"""End-to-end API tests against a mock-hardware robot (no Pi required)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

fastapi_testclient = pytest.importorskip("fastapi.testclient")

from rubik_robot.api import create_app  # noqa: E402
from rubik_robot.hardware.camera import MockCamera  # noqa: E402
from rubik_robot.hardware.servos import MockServoDriver  # noqa: E402
from rubik_robot.robot import Robot  # noqa: E402


@pytest.fixture()
def client(tmp_path, monkeypatch):
    monkeypatch.setenv("RUBIK_DATA_DIR", str(tmp_path))
    # Zero out the settle delays so tests run fast with mock servos.
    monkeypatch.setattr("rubik_robot.robot.config.SLEEP_GRIP", 0.0)
    robot = Robot(MockServoDriver(), MockCamera())
    robot.cal.sleep = 0.0
    app = create_app(robot)
    return fastapi_testclient.TestClient(app)


def test_health(client):
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json()["status"] == "ok"


def test_home_and_grip(client):
    assert client.post("/home").status_code == 200
    assert client.post("/grip").status_code == 200
    assert client.post("/regrip").status_code == 200


def test_single_move(client):
    r = client.post("/move", json={"move": "U"})
    assert r.status_code == 200
    assert r.json()["move"] == "U"
    assert r.json()["actions"]  # non-empty action string


def test_invalid_move_rejected(client):
    r = client.post("/move", json={"move": "Q"})
    assert r.status_code == 422


def test_move_sequence(client):
    r = client.post("/moves", json={"moves": ["U", "R'", "F2", "D"], "reset_orientation": True})
    assert r.status_code == 200
    assert r.json()["count"] == 4


def test_invalid_moves_rejected(client):
    r = client.post("/moves", json={"moves": ["U", "ZZ"]})
    assert r.status_code == 422


def test_scan_returns_54_facelets(client):
    r = client.post("/scan")
    assert r.status_code == 200
    assert len(r.json()["facelets"]) == 54


def test_tune_roundtrip(client):
    assert client.get("/tune").json()["load"] == 30
    r = client.put("/tune", json={"load": 42, "sleep": 0.25})
    assert r.status_code == 200
    body = r.json()
    assert body["load"] == 42 and body["sleep"] == 0.25
    assert client.get("/tune").json()["load"] == 42


def test_raw_action_validation(client):
    assert client.post("/action", json={"actions": "abAB"}).status_code == 200
    assert client.post("/action", json={"actions": "abQ"}).status_code == 422
