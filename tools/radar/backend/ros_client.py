"""Thin wrapper around roslibpy.

This is the only module in the whole tool that talks to ROS at all — it
owns the single rosbridge connection, relays the /service/scan and
/service/localization topics to a callback, and exposes the two trigger
services as plain blocking Python calls (safe to run in a thread pool
worker from an async endpoint).
"""

import logging
import threading
from typing import Callable, Optional

import roslibpy

import config

logger = logging.getLogger("radar.ros_client")


class ServiceCallError(RuntimeError):
    pass


class RosBridgeClient:
    def __init__(self, on_scan: Callable[[dict], None], on_localization: Callable[[dict], None]):
        self._on_scan = on_scan
        self._on_localization = on_localization
        self.ros = roslibpy.Ros(host=config.ROSBRIDGE_HOST, port=config.ROSBRIDGE_PORT)
        self._scan_topic: Optional[roslibpy.Topic] = None
        self._localization_topic: Optional[roslibpy.Topic] = None
        self._scan_service: Optional[roslibpy.Service] = None
        self._localization_service: Optional[roslibpy.Service] = None

    def connect(self) -> None:
        self.ros.run()

        self._scan_topic = roslibpy.Topic(self.ros, config.SCAN_TOPIC, config.SCAN_TOPIC_TYPE)
        self._scan_topic.subscribe(self._on_scan)

        self._localization_topic = roslibpy.Topic(
            self.ros, config.LOCALIZATION_TOPIC, config.LOCALIZATION_TOPIC_TYPE
        )
        self._localization_topic.subscribe(self._on_localization)

        self._scan_service = roslibpy.Service(self.ros, config.SCAN_SERVICE, config.SCAN_SERVICE_TYPE)
        self._localization_service = roslibpy.Service(
            self.ros, config.LOCALIZATION_SERVICE, config.LOCALIZATION_SERVICE_TYPE
        )
        logger.info(
            "Connecting to rosbridge at %s:%s", config.ROSBRIDGE_HOST, config.ROSBRIDGE_PORT
        )

    def close(self) -> None:
        if self._scan_topic is not None:
            self._scan_topic.unsubscribe()
        if self._localization_topic is not None:
            self._localization_topic.unsubscribe()
        if self.ros.is_connected:
            self.ros.close()

    def _call(self, service: roslibpy.Service, request: dict, timeout: float) -> dict:
        done = threading.Event()
        outcome: dict = {}

        def _on_result(result):
            outcome["result"] = result
            done.set()

        def _on_error(error):
            outcome["error"] = str(error)
            done.set()

        service.call(roslibpy.ServiceRequest(request), callback=_on_result, errback=_on_error)

        if not done.wait(timeout):
            raise ServiceCallError(f"{service.name} call timed out after {timeout}s")
        if "error" in outcome:
            raise ServiceCallError(f"{service.name} call failed: {outcome['error']}")
        return dict(outcome["result"])

    def call_scan(self, start_angle: int, stop_angle: int, desired_range: int) -> dict:
        assert self._scan_service is not None, "call connect() first"
        return self._call(
            self._scan_service,
            {
                "start_angle": start_angle,
                "stop_angle": stop_angle,
                "desired_range": desired_range,
            },
            config.SERVICE_CALL_TIMEOUT_S,
        )

    def call_localize(self) -> dict:
        assert self._localization_service is not None, "call connect() first"
        return self._call(self._localization_service, {}, config.SERVICE_CALL_TIMEOUT_S)
