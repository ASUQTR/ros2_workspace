import math

import pytest

from rosbridge_client import parse_odom_message


def _msg(px, py, qz, qw, sec=10, nanosec=500_000_000):
    return {
        "header": {"stamp": {"sec": sec, "nanosec": nanosec}},
        "pose": {
            "pose": {
                "position": {"x": px, "y": py, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            }
        },
    }


def test_parse_extracts_position_and_timestamp():
    msg = _msg(1.5, 2.3, 0.0, 1.0)
    point = parse_odom_message(msg)
    assert point["x"] == pytest.approx(1.5)
    assert point["y"] == pytest.approx(2.3)
    assert point["heading"] == pytest.approx(0.0)
    assert point["timestamp"] == pytest.approx(10.5)


def test_parse_yaw_from_quaternion():
    # 90-degree yaw quaternion.
    msg = _msg(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    point = parse_odom_message(msg)
    assert point["heading"] == pytest.approx(math.pi / 2, abs=1e-9)


def test_parse_applies_heading_offset_and_wraps():
    msg = _msg(0.0, 0.0, math.sin(math.pi / 2), math.cos(math.pi / 2))  # yaw = pi
    point = parse_odom_message(msg, heading_offset_rad=math.pi / 2)
    # pi + pi/2 wraps to -pi/2.
    assert point["heading"] == pytest.approx(-math.pi / 2, abs=1e-9)


def test_parse_accepts_ros1_stamp_keys():
    msg = _msg(0.0, 0.0, 0.0, 1.0)
    del msg["header"]["stamp"]["sec"]
    del msg["header"]["stamp"]["nanosec"]
    msg["header"]["stamp"]["secs"] = 3
    msg["header"]["stamp"]["nsecs"] = 250_000_000
    point = parse_odom_message(msg)
    assert point["timestamp"] == pytest.approx(3.25)


def test_parse_raises_on_malformed_message():
    with pytest.raises((KeyError, TypeError)):
        parse_odom_message({"pose": {}})
