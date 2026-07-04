"""Pure math helpers shared by the rosbridge client and the rosbag reader.

Kept dependency-free (standard library only) so it imports cleanly on a laptop
with no ROS 2 installation and is trivial to unit-test.
"""

import math

TWO_PI = 2.0 * math.pi


def quaternion_to_yaw(x, y, z, w):
    """Extract the yaw (rotation about Z) from a quaternion, in radians.

    Uses the standard ZYX Tait-Bryan convention. The result is in
    ``(-pi, pi]``. For an identity quaternion this returns ``0.0`` which, in an
    ENU frame, points East.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    """Wrap an angle in radians to the range ``(-pi, pi]``."""
    wrapped = math.fmod(angle + math.pi, TWO_PI)
    if wrapped <= 0.0:
        wrapped += TWO_PI
    return wrapped - math.pi


def next_backoff(current, cap, base=1.0, factor=2.0):
    """Return the next exponential-backoff delay (seconds).

    The sequence is ``base, base*factor, base*factor^2, ...`` clamped to
    ``cap``. Passing ``0`` (or any value below ``base``) as ``current`` yields
    the first delay, so callers can start from ``0.0`` on a fresh connection.
    """
    if current < base:
        return min(base, cap)
    return min(current * factor, cap)
