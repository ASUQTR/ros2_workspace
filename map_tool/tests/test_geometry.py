import math

import pytest

from geometry import next_backoff, quaternion_to_yaw, wrap_angle


def yaw_quaternion(yaw):
    """Build a pure-yaw quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


@pytest.mark.parametrize(
    "yaw",
    [0.0, math.pi / 2, -math.pi / 2, math.pi / 4, -math.pi / 4, 2.5, -2.5],
)
def test_quaternion_to_yaw_roundtrip(yaw):
    x, y, z, w = yaw_quaternion(yaw)
    assert quaternion_to_yaw(x, y, z, w) == pytest.approx(yaw, abs=1e-9)


def test_identity_quaternion_is_zero_yaw():
    # In an ENU frame, yaw 0 points East.
    assert quaternion_to_yaw(0, 0, 0, 1) == pytest.approx(0.0)


def test_180_degrees_wraps_to_pi():
    x, y, z, w = yaw_quaternion(math.pi)
    assert abs(quaternion_to_yaw(x, y, z, w)) == pytest.approx(math.pi, abs=1e-9)


def test_wrap_angle_range():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)  # -pi maps to +pi
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(2 * math.pi + 0.5) == pytest.approx(0.5, abs=1e-9)
    assert wrap_angle(-2 * math.pi - 0.5) == pytest.approx(-0.5, abs=1e-9)


def test_next_backoff_sequence():
    b = 0.0
    seq = []
    for _ in range(8):
        b = next_backoff(b, cap=30.0)
        seq.append(b)
    assert seq == [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0, 30.0]


def test_next_backoff_respects_cap():
    assert next_backoff(20.0, cap=30.0) == 30.0
    assert next_backoff(30.0, cap=30.0) == 30.0
