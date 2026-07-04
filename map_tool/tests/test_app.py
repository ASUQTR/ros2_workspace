"""Smoke tests for the FastAPI app that don't require ROS 2 or a rosbridge."""

import io

import pytest
from fastapi.testclient import TestClient

import app as app_module
from app import app

client = TestClient(app)


@pytest.fixture(autouse=True)
def _no_api_key(monkeypatch):
    # Default most tests to "no auth required".
    monkeypatch.setattr(app_module.config, "api_key", "")


def test_config_endpoint():
    res = client.get("/api/config")
    assert res.status_code == 200
    body = res.json()
    assert "ros_bridge_ip" in body
    assert "max_upload_mb" in body
    assert body["api_key_required"] is False


def test_index_served():
    res = client.get("/")
    assert res.status_code == 200
    assert "Sub Path Visualizer" in res.text


def test_list_rosbags():
    res = client.get("/api/rosbags")
    assert res.status_code == 200
    assert "rosbags" in res.json()


def test_unknown_rosbag_is_404():
    assert client.get("/api/rosbag/nope/status").status_code == 404
    assert client.get("/api/rosbag/nope/metadata").status_code == 404
    assert client.get("/api/rosbag/nope/odometry").status_code == 404


def test_upload_rejects_bad_filename():
    files = {"file": ("../evil.db3", io.BytesIO(b"x"), "application/octet-stream")}
    res = client.post("/api/upload-rosbag", files=files)
    assert res.status_code == 400


def test_upload_rejects_bad_extension():
    files = {"file": ("notabag.txt", io.BytesIO(b"x"), "text/plain")}
    res = client.post("/api/upload-rosbag", files=files)
    assert res.status_code == 400


def test_upload_rejects_oversized(monkeypatch):
    # Shrink the limit so a tiny payload trips it.
    monkeypatch.setattr(app_module.config, "max_upload_mb", 0)
    payload = b"0" * 1024
    files = {"file": ("big.db3", io.BytesIO(payload), "application/octet-stream")}
    res = client.post("/api/upload-rosbag", files=files)
    assert res.status_code == 413


def test_api_key_enforced(monkeypatch):
    monkeypatch.setattr(app_module.config, "api_key", "secret")
    assert client.get("/api/config").status_code == 401
    ok = client.get("/api/config", headers={"X-API-Key": "secret"})
    assert ok.status_code == 200
