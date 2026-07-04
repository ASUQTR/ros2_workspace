"""Backend configuration.

Values are read from environment variables, optionally seeded from a ``.env``
file (see ``.env.example``). A single module-level :data:`config` instance is
imported by the rest of the app.
"""

import math
import os
from dataclasses import dataclass

try:  # python-dotenv is optional; env vars can also be set in the shell.
    from dotenv import load_dotenv

    load_dotenv()
except ImportError:  # pragma: no cover - trivial fallback
    pass


def _get_str(name, default):
    val = os.getenv(name)
    return default if val is None or val.strip() == "" else val.strip()


def _get_int(name, default):
    val = os.getenv(name)
    if val is None or val.strip() == "":
        return default
    return int(val)


def _get_float(name, default):
    val = os.getenv(name)
    if val is None or val.strip() == "":
        return default
    return float(val)


def _get_bool(name, default):
    val = os.getenv(name)
    if val is None or val.strip() == "":
        return default
    return val.strip().lower() in ("1", "true", "yes", "on")


@dataclass
class Config:
    # Remote ROS 2 system (rosbridge WebSocket endpoint).
    ros_bridge_ip: str = "127.0.0.1"
    ros_bridge_port: int = 9090
    odom_topic: str = "/odometry/filtered"
    odom_type: str = "nav_msgs/msg/Odometry"

    # Backend HTTP server. 0.0.0.0 is a *listen* address; browsers connect via
    # http://localhost:<port> (or the machine's actual LAN IP).
    backend_host: str = "0.0.0.0"
    backend_port: int = 8001
    debug: bool = False

    # Optional shared secret. When non-empty, /api/* and /ws/live require it
    # (X-API-Key header, or ?key= query param for the WebSocket).
    api_key: str = ""

    # Rosbag storage.
    upload_dir: str = "./recordings/uploads"
    cache_dir: str = "./recordings/cache"
    max_upload_mb: int = 500

    # Connection tuning.
    rosbridge_timeout_sec: float = 5.0
    reconnect_max_sec: float = 30.0
    heartbeat_interval_sec: float = 5.0
    heartbeat_timeout_sec: float = 10.0

    # Frame convention fix-up: added to every extracted yaw before it reaches
    # the frontend. Leave at 0 for a standard ENU frame (yaw=0 -> East).
    heading_offset_deg: float = 0.0

    @property
    def ros_bridge_url(self):
        return f"ws://{self.ros_bridge_ip}:{self.ros_bridge_port}"

    @property
    def heading_offset_rad(self):
        return math.radians(self.heading_offset_deg)

    @property
    def max_upload_bytes(self):
        return self.max_upload_mb * 1024 * 1024

    @property
    def api_key_required(self):
        return bool(self.api_key)

    @classmethod
    def from_env(cls):
        return cls(
            ros_bridge_ip=_get_str("ROS_BRIDGE_IP", cls.ros_bridge_ip),
            ros_bridge_port=_get_int("ROS_BRIDGE_PORT", cls.ros_bridge_port),
            odom_topic=_get_str("ODOM_TOPIC", cls.odom_topic),
            odom_type=_get_str("ODOM_TYPE", cls.odom_type),
            backend_host=_get_str("BACKEND_HOST", cls.backend_host),
            backend_port=_get_int("BACKEND_PORT", cls.backend_port),
            debug=_get_bool("DEBUG", cls.debug),
            api_key=_get_str("API_KEY", cls.api_key),
            upload_dir=_get_str("ROSBAG_UPLOAD_DIR", cls.upload_dir),
            cache_dir=_get_str("ROSBAG_CACHE_DIR", cls.cache_dir),
            max_upload_mb=_get_int("MAX_UPLOAD_SIZE_MB", cls.max_upload_mb),
            rosbridge_timeout_sec=_get_float(
                "ROSBRIDGE_TIMEOUT_SEC", cls.rosbridge_timeout_sec
            ),
            reconnect_max_sec=_get_float(
                "ROSBRIDGE_RECONNECT_MAX_SEC", cls.reconnect_max_sec
            ),
            heartbeat_interval_sec=_get_float(
                "ROSBRIDGE_HEARTBEAT_INTERVAL_SEC", cls.heartbeat_interval_sec
            ),
            heartbeat_timeout_sec=_get_float(
                "ROSBRIDGE_HEARTBEAT_TIMEOUT_SEC", cls.heartbeat_timeout_sec
            ),
            heading_offset_deg=_get_float("HEADING_OFFSET_DEG", cls.heading_offset_deg),
        )


config = Config.from_env()
