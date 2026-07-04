import asyncio
import logging
from pathlib import Path
from typing import Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

import config
from ros_client import RosBridgeClient, ServiceCallError

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("radar.app")

FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"

app = FastAPI(title="Radar visualization tool")


class ConnectionManager:
    def __init__(self):
        self._clients: Set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self._lock:
            self._clients.add(websocket)

    async def disconnect(self, websocket: WebSocket):
        async with self._lock:
            self._clients.discard(websocket)

    async def broadcast(self, payload: dict):
        async with self._lock:
            clients = list(self._clients)
        for client in clients:
            try:
                await client.send_json(payload)
            except Exception:
                await self.disconnect(client)


manager = ConnectionManager()
main_loop: asyncio.AbstractEventLoop | None = None


def _on_scan(message: dict):
    if main_loop is not None:
        asyncio.run_coroutine_threadsafe(
            manager.broadcast({"type": "scan", "data": message}), main_loop
        )


def _on_localization(message: dict):
    if main_loop is not None:
        asyncio.run_coroutine_threadsafe(
            manager.broadcast({"type": "localization", "data": message}), main_loop
        )


ros_client = RosBridgeClient(on_scan=_on_scan, on_localization=_on_localization)


@app.on_event("startup")
async def on_startup():
    global main_loop
    main_loop = asyncio.get_event_loop()
    ros_client.connect()


@app.on_event("shutdown")
async def on_shutdown():
    ros_client.close()


class ScanRequest(BaseModel):
    start_angle: int = config.DEFAULT_SCAN_START_ANGLE
    stop_angle: int = config.DEFAULT_SCAN_STOP_ANGLE
    desired_range: int = config.DEFAULT_SCAN_RANGE_M


@app.post("/api/scan")
async def trigger_scan(req: ScanRequest):
    loop = asyncio.get_event_loop()
    try:
        result = await loop.run_in_executor(
            None, ros_client.call_scan, req.start_angle, req.stop_angle, req.desired_range
        )
    except ServiceCallError as exc:
        return {"success": False, "message": str(exc)}
    if not result.get("success", False):
        logger.warning("Scan call returned failure: %s", result.get("message"))
    return result


@app.post("/api/localize")
async def trigger_localize():
    loop = asyncio.get_event_loop()
    try:
        result = await loop.run_in_executor(None, ros_client.call_localize)
    except ServiceCallError as exc:
        return {"success": False, "message": str(exc)}
    if not result.get("success", False):
        logger.warning("Localization call returned failure: %s", result.get("message"))
    return result


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # This tool never expects messages from the browser on this
            # socket; just keep the connection open until the client closes it.
            await websocket.receive_text()
    except WebSocketDisconnect:
        await manager.disconnect(websocket)


app.mount("/", StaticFiles(directory=FRONTEND_DIR, html=True), name="frontend")
