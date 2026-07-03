"""Rosbag2 odometry extraction.

Reads a recorded bag and returns the odometry track as plain JSON-serialisable
data. This module deliberately imports ``rosbag2_py`` and friends *lazily*
(inside functions) so the rest of the backend — including live mode — runs on a
laptop with no ROS 2 installation. Playback via uploaded bags (Option A) does
require ROS 2 Humble libraries to be importable.

Supported inputs:
  * a rosbag2 directory (contains ``metadata.yaml`` + storage files)
  * a standalone ``.mcap`` file
  * a standalone ``.db3`` (sqlite3) file
  * a ``.zip`` containing any of the above
"""

import os
import shutil
import tempfile
import zipfile

from geometry import quaternion_to_yaw, wrap_angle


class RosbagError(Exception):
    """Raised for any user-facing rosbag problem (missing topic, corrupt, ...)."""


def _import_rosbag2():
    try:
        import rosbag2_py  # noqa: WPS433 - intentional lazy import
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        return rosbag2_py, deserialize_message, get_message
    except ImportError as exc:  # pragma: no cover - depends on host ROS install
        raise RosbagError(
            "rosbag2 Python libraries are not available. Playback via uploaded "
            "bags requires ROS 2 Humble: install 'ros-humble-rosbag2-py' and "
            "'source /opt/ros/humble/setup.bash' before starting the backend, "
            "or use remote bag replay (Option B)."
        ) from exc


def _storage_id_for(path):
    """Best-effort storage plugin id for a bag path."""
    lower = path.lower()
    if lower.endswith(".mcap"):
        return "mcap"
    if lower.endswith(".db3"):
        return "sqlite3"
    if os.path.isdir(path):
        for entry in os.listdir(path):
            if entry.lower().endswith(".mcap"):
                return "mcap"
        return "sqlite3"
    return "sqlite3"


def _find_bag_root(directory):
    """Return the directory that actually holds bag storage files."""
    for root, _dirs, files in os.walk(directory):
        lowered = [f.lower() for f in files]
        if "metadata.yaml" in lowered:
            return root
        if any(f.endswith((".mcap", ".db3")) for f in lowered):
            return root
    return directory


def _open_reader(uri, storage_id, topic):
    rosbag2_py, _deserialize, _get_message = _import_rosbag2()
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as exc:  # noqa: BLE001
        raise RosbagError(f"Could not open rosbag: {exc}") from exc

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        available = ", ".join(sorted(type_map)) or "(none)"
        raise RosbagError(
            f"Topic '{topic}' not found in rosbag. Available topics: {available}"
        )
    try:  # Restrict reads to the topic we care about for speed.
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    except Exception:  # noqa: BLE001 - older APIs may lack set_filter
        pass
    return reader, type_map[topic]


def _topic_message_count(uri, storage_id, topic):
    """Total messages on ``topic`` from bag metadata, or ``None`` if unknown."""
    try:
        rosbag2_py, _deserialize, _get_message = _import_rosbag2()
        info = rosbag2_py.Info()
        metadata = info.read_metadata(uri, storage_id)
        for entry in metadata.topics_with_message_count:
            if entry.topic_metadata.name == topic:
                return entry.message_count
    except Exception:  # noqa: BLE001 - metadata is best-effort
        return None
    return None


def _resolve_uri(path):
    """Return ``(uri, storage_id, cleanup_dir)`` for a bag path.

    ``cleanup_dir`` is a temp directory to remove afterwards (for zip inputs) or
    ``None``.
    """
    if path.lower().endswith(".zip"):
        tmp = tempfile.mkdtemp(prefix="map_tool_bag_")
        try:
            with zipfile.ZipFile(path) as zf:
                _safe_extract_zip(zf, tmp)
        except (zipfile.BadZipFile, RosbagError):
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        root = _find_bag_root(tmp)
        return root, _storage_id_for(root), tmp
    return path, _storage_id_for(path), None


def _safe_extract_zip(zf, dest):
    """Extract a zip while rejecting path-traversal entries."""
    dest_abs = os.path.abspath(dest)
    for member in zf.namelist():
        target = os.path.abspath(os.path.join(dest, member))
        if not (target == dest_abs or target.startswith(dest_abs + os.sep)):
            raise RosbagError(f"Unsafe path in zip archive: {member}")
    zf.extractall(dest)


def validate_rosbag(path, topic):
    """Check the bag opens and contains ``topic`` with at least one message.

    Raises :class:`RosbagError` with a descriptive message otherwise.
    """
    if not os.path.exists(path):
        raise RosbagError(f"File not found: {path}")
    uri, storage_id, cleanup = _resolve_uri(path)
    try:
        reader, _msg_type = _open_reader(uri, storage_id, topic)
        del reader  # opening + topic check is enough for validation
        count = _topic_message_count(uri, storage_id, topic)
        if count == 0:
            raise RosbagError(f"Topic '{topic}' exists but contains no messages.")
    finally:
        if cleanup:
            shutil.rmtree(cleanup, ignore_errors=True)


def extract_odometry(path, topic, heading_offset_rad=0.0, progress_cb=None):
    """Extract every odometry message on ``topic``.

    Returns ``{"metadata": {...}, "odometry": [{x, y, heading, timestamp}, ...]}``.
    ``progress_cb(percent)`` is called periodically with an int in ``[0, 100]``.
    """
    _rosbag2, deserialize_message, get_message = _import_rosbag2()

    if not os.path.exists(path):
        raise RosbagError(f"File not found: {path}")

    uri, storage_id, cleanup = _resolve_uri(path)
    try:
        reader, type_name = _open_reader(uri, storage_id, topic)
        msg_class = get_message(type_name)
        total = _topic_message_count(uri, storage_id, topic)

        points = []
        read = 0
        last_pct = -1
        while reader.has_next():
            topic_name, data, _bag_ts = reader.read_next()
            if topic_name != topic:
                continue
            msg = deserialize_message(data, msg_class)
            points.append(_point_from_msg(msg, heading_offset_rad))
            read += 1
            if progress_cb is not None:
                pct = int(read * 100 / total) if total else min(99, read // 100)
                if pct != last_pct:
                    last_pct = pct
                    progress_cb(min(pct, 100))
    finally:
        if cleanup:
            shutil.rmtree(cleanup, ignore_errors=True)

    if not points:
        raise RosbagError(f"No odometry messages extracted from topic '{topic}'.")

    if progress_cb is not None:
        progress_cb(100)

    return {"metadata": _build_metadata(points), "odometry": points}


def _point_from_msg(msg, heading_offset_rad):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    stamp = msg.header.stamp
    timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
    return {
        "x": float(pos.x),
        "y": float(pos.y),
        "heading": wrap_angle(yaw + heading_offset_rad),
        "timestamp": timestamp,
    }


def _build_metadata(points):
    start = points[0]["timestamp"]
    end = points[-1]["timestamp"]
    return {
        "start": start,
        "end": end,
        "duration": max(0.0, end - start),
        "count": len(points),
    }
