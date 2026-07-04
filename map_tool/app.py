"""FastAPI backend for the submarine path visualization tool.

Serves the static frontend, exposes the rosbag playback REST API, and bridges a
remote rosbridge odometry subscription to the browser over ``/ws/live``.

Run with::

    python app.py
    # or: uvicorn app:app --host 0.0.0.0 --port 8001
"""

import asyncio
import contextlib
import logging
import os

from fastapi import (
    Depends,
    FastAPI,
    Header,
    HTTPException,
    Query,
    Request,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from config import config
from rosbridge_client import RosbridgeClient
from upload_handler import RosbagStore, sanitize_filename

logging.basicConfig(
    level=logging.DEBUG if config.debug else logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)
logger = logging.getLogger("map_tool")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")

app = FastAPI(title="Submarine Path Visualizer", debug=config.debug)

store = RosbagStore(
    upload_dir=config.upload_dir,
    cache_dir=config.cache_dir,
    topic=config.odom_topic,
    heading_offset_rad=config.heading_offset_rad,
)

_UPLOAD_CHUNK = 1024 * 1024  # 1 MiB


# --------------------------------------------------------------------------- #
# Auth
# --------------------------------------------------------------------------- #
def require_api_key(x_api_key: str = Header(default=None)):
    """Dependency enforcing the optional shared secret on REST routes."""
    if config.api_key and x_api_key != config.api_key:
        raise HTTPException(status_code=401, detail="Invalid or missing API key.")


def _ws_authorized(websocket: WebSocket) -> bool:
    if not config.api_key:
        return True
    provided = websocket.query_params.get("key") or websocket.headers.get("x-api-key")
    return provided == config.api_key


# --------------------------------------------------------------------------- #
# Frontend + config
# --------------------------------------------------------------------------- #
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
def index():
    return FileResponse(os.path.join(STATIC_DIR, "index.html"))


@app.get("/api/config")
def get_config(_auth=Depends(require_api_key)):
    return {
        "ros_bridge_ip": config.ros_bridge_ip,
        "ros_bridge_port": config.ros_bridge_port,
        "odom_topic": config.odom_topic,
        "max_upload_mb": config.max_upload_mb,
        "heading_offset_deg": config.heading_offset_deg,
        "api_key_required": config.api_key_required,
    }


# --------------------------------------------------------------------------- #
# Rosbag playback REST API
# --------------------------------------------------------------------------- #
@app.get("/api/rosbags")
def list_rosbags(_auth=Depends(require_api_key)):
    return {"rosbags": store.list()}


@app.post("/api/upload-rosbag")
async def upload_rosbag(file: UploadFile, _auth=Depends(require_api_key)):
    try:
        safe_name = sanitize_filename(file.filename)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    dest = store.dest_path(safe_name)
    written = 0
    try:
        with open(dest, "wb") as out:
            while True:
                chunk = await file.read(_UPLOAD_CHUNK)
                if not chunk:
                    break
                written += len(chunk)
                if written > config.max_upload_bytes:
                    raise HTTPException(
                        status_code=413,
                        detail=f"File exceeds limit of {config.max_upload_mb} MB.",
                    )
                out.write(chunk)
    except HTTPException:
        _safe_unlink(dest)
        raise
    except Exception as exc:  # noqa: BLE001
        _safe_unlink(dest)
        raise HTTPException(status_code=500, detail=f"Upload failed: {exc}") from exc
    finally:
        await file.close()

    bag_id = store.register_upload(os.path.basename(dest), dest)
    store.ensure_extraction(bag_id)
    logger.info("Uploaded rosbag %s (%d bytes) -> %s", safe_name, written, bag_id)
    return {"id": bag_id, "filename": os.path.basename(dest), "status": "extracting"}


@app.get("/api/rosbag/{bag_id}/status")
def rosbag_status(bag_id: str, _auth=Depends(require_api_key)):
    try:
        store.ensure_extraction(bag_id)  # kick off pre-loaded bags on first look
        return store.status(bag_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Unknown rosbag id.") from exc


@app.get("/api/rosbag/{bag_id}/metadata")
def rosbag_metadata(bag_id: str, _auth=Depends(require_api_key)):
    try:
        meta = store.metadata(bag_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Unknown rosbag id.") from exc
    if meta is None:
        raise HTTPException(status_code=409, detail="Rosbag not extracted yet.")
    return meta


@app.get("/api/rosbag/{bag_id}/odometry")
def rosbag_odometry(bag_id: str, _auth=Depends(require_api_key)):
    try:
        data = store.odometry(bag_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Unknown rosbag id.") from exc
    if data is None:
        raise HTTPException(status_code=409, detail="Rosbag not extracted yet.")
    return JSONResponse(data)


@app.delete("/api/rosbag/{bag_id}")
def delete_rosbag(bag_id: str, _auth=Depends(require_api_key)):
    try:
        store.delete(bag_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Unknown rosbag id.") from exc
    return {"deleted": bag_id}


# --------------------------------------------------------------------------- #
# Live streaming over WebSocket
# --------------------------------------------------------------------------- #
@app.websocket("/ws/live")
async def ws_live(
    websocket: WebSocket,
    ip: str = Query(default=None),
    port: int = Query(default=None),
):
    if not _ws_authorized(websocket):
        await websocket.close(code=1008)
        return

    await websocket.accept()

    ros_ip = ip or config.ros_bridge_ip
    ros_port = port or config.ros_bridge_port
    url = f"ws://{ros_ip}:{ros_port}"

    client = RosbridgeClient(
        url,
        config.odom_topic,
        config.odom_type,
        open_timeout=config.rosbridge_timeout_sec,
        heartbeat_interval=config.heartbeat_interval_sec,
        heartbeat_timeout=config.heartbeat_timeout_sec,
        reconnect_max=config.reconnect_max_sec,
        heading_offset_rad=config.heading_offset_rad,
    )

    send_lock = asyncio.Lock()

    async def send(payload):
        async with send_lock:
            await websocket.send_json(payload)

    async def on_message(point):
        await send({"type": "odom", "data": point})

    async def on_status(state, detail=None):
        await send({"type": "status", "state": state, "detail": detail})

    task = asyncio.create_task(client.run(on_message, on_status))
    logger.info("Live client connected; bridging %s topic=%s", url, config.odom_topic)

    try:
        # Keep the socket alive and detect browser disconnect. Any inbound text
        # is ignored (reserved for future client->server commands).
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    except Exception:  # noqa: BLE001
        pass
    finally:
        client.stop()
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await task
        logger.info("Live client disconnected")


def _safe_unlink(path):
    with contextlib.suppress(OSError):
        if os.path.exists(path):
            os.remove(path)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        app,
        host=config.backend_host,
        port=config.backend_port,
        log_level="debug" if config.debug else "info",
    )
