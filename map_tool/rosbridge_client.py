"""Async rosbridge WebSocket client.

Connects to a remote ``rosbridge_server``, subscribes to the odometry topic,
and forwards parsed points to a callback. Handles reconnection with exponential
backoff; connection liveness is detected via WebSocket-level ping/pong
(``ping_interval`` / ``ping_timeout``), which is the standard, robust heartbeat
mechanism and does not depend on any rosbridge-specific op.
"""

import asyncio
import json

import websockets

from geometry import next_backoff, quaternion_to_yaw, wrap_angle


def parse_odom_message(msg, heading_offset_rad=0.0):
    """Turn a rosbridge ``nav_msgs/Odometry`` JSON payload into a plain point.

    Accepts both ROS 2 stamp keys (``sec``/``nanosec``) and legacy ROS 1 keys
    (``secs``/``nsecs``) for robustness. Raises ``KeyError``/``TypeError`` on a
    malformed message so the caller can skip it.
    """
    pose = msg["pose"]["pose"]
    position = pose["position"]
    orientation = pose["orientation"]

    stamp = msg.get("header", {}).get("stamp", {})
    sec = stamp.get("sec", stamp.get("secs", 0))
    nanosec = stamp.get("nanosec", stamp.get("nsecs", 0))
    timestamp = float(sec) + float(nanosec) * 1e-9

    yaw = quaternion_to_yaw(
        orientation["x"], orientation["y"], orientation["z"], orientation["w"]
    )
    return {
        "x": float(position["x"]),
        "y": float(position["y"]),
        "heading": wrap_angle(yaw + heading_offset_rad),
        "timestamp": timestamp,
    }


class RosbridgeClient:
    """Maintains a subscription to the odometry topic on a remote rosbridge."""

    def __init__(
        self,
        url,
        topic,
        msg_type,
        *,
        open_timeout=5.0,
        heartbeat_interval=5.0,
        heartbeat_timeout=10.0,
        reconnect_max=30.0,
        heading_offset_rad=0.0,
    ):
        self._url = url
        self._topic = topic
        self._msg_type = msg_type
        self._open_timeout = open_timeout
        self._heartbeat_interval = heartbeat_interval
        self._heartbeat_timeout = heartbeat_timeout
        self._reconnect_max = reconnect_max
        self._heading_offset = heading_offset_rad
        self._stop = False

    def stop(self):
        """Request the run loop to exit at the next opportunity."""
        self._stop = True

    def _subscribe_msg(self):
        return json.dumps(
            {
                "op": "subscribe",
                "topic": self._topic,
                "type": self._msg_type,
                "throttle_rate": 0,
                "queue_length": 1,
            }
        )

    async def run(self, on_message, on_status):
        """Connect, subscribe, and stream until :meth:`stop` is called.

        ``on_message(point)`` and ``on_status(state, detail=None)`` are awaited
        coroutines. ``state`` is one of ``connecting``, ``connected``,
        ``reconnecting``, ``disconnected``.
        """
        backoff = 0.0
        await on_status("connecting", {"url": self._url})

        while not self._stop:
            try:
                async with websockets.connect(
                    self._url,
                    open_timeout=self._open_timeout,
                    ping_interval=self._heartbeat_interval,
                    ping_timeout=self._heartbeat_timeout,
                    max_size=None,
                ) as ws:
                    backoff = 0.0
                    await on_status("connected", {"url": self._url})
                    await ws.send(self._subscribe_msg())

                    async for raw in ws:
                        if self._stop:
                            break
                        point = self._handle_raw(raw)
                        if point is not None:
                            await on_message(point)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - surface any failure as reconnect
                if self._stop:
                    break
                backoff = next_backoff(backoff, self._reconnect_max)
                await on_status(
                    "reconnecting", {"error": str(exc), "retry_in": backoff}
                )
                try:
                    await asyncio.sleep(backoff)
                except asyncio.CancelledError:
                    raise

        await on_status("disconnected")

    def _handle_raw(self, raw):
        try:
            data = json.loads(raw)
        except (ValueError, TypeError):
            return None
        if data.get("op") != "publish" or data.get("topic") != self._topic:
            return None
        try:
            return parse_odom_message(data["msg"], self._heading_offset)
        except (KeyError, TypeError, ValueError):
            return None
