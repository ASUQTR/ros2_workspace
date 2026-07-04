# Radar visualization tool

A standalone web tool (not a ROS package) that visualizes:

1. **Layer 1 — Raw sonar scan**: the Ping360 driver's `/sonar/scan` response,
   rendered as a polar sweep centered on the AUV.
2. **Layer 2 — Pool & AUV position**: the localization service's
   `get_sonar_position` response, rendered as a top-down pool rectangle with
   an AUV marker.

See `PLAN.md` for the full design rationale.

## Architecture

- `sonar_node` (`sub_hardware`) and `sonar_localization` (`sub_control`)
  keep their services (`/sonar/scan`, `get_sonar_position`) as pure
  request/response — callable by this tool's buttons, and later by other
  components (e.g. playback control). Each node also mirrors its own
  response onto a topic right before returning it: `/service/scan`
  (`sub_interfaces/msg/SonarScanResult`) and `/service/localization`
  (`sub_interfaces/msg/SonarPositionResult`).
- `backend/` is a small FastAPI app — the only piece of this tool that
  speaks ROS. It connects to `rosbridge_websocket` via `roslibpy`, exposes
  `POST /api/scan` and `POST /api/localize` to trigger the two services,
  and relays `/service/scan` / `/service/localization` to connected
  browsers over its own `/ws` WebSocket.
- `frontend/` is plain HTML/CSS/JS (no build step, no `roslibjs`) — it only
  ever talks to the FastAPI backend.

## Prerequisites

- `rosbridge_websocket` running and reachable (already launched by
  `sub.launch.yaml`, `hardware.launch.yaml`, and `sim.launch.yaml`, default
  port 9090).
- `sonar_node` and `sonar_localization` running (part of `sim.launch.yaml`,
  or `sonar.launch.yaml` + `sub_control` on hardware).

## Running

```bash
cd tools/radar/backend
python3 -m venv .venv && source .venv/bin/activate   # optional
pip install -r requirements.txt

# Point at the machine running rosbridge if it's not localhost:
export RADAR_ROSBRIDGE_HOST=localhost
export RADAR_ROSBRIDGE_PORT=9090

uvicorn app:app --host 0.0.0.0 --port 8000
```

Then open `http://localhost:8000` in a browser.

## Notes

- Triggering "Localization" internally causes 4 `/sonar/scan` calls (one
  per cardinal wall), so Layer 1 will visibly update 4 times before Layer 2
  updates once with the final pose — this is a byproduct of both nodes
  mirroring every call to their topic, not something the tool special-cases.
- A failed call (`success: false`) is shown as a status message instead of
  being drawn as if it were valid data.
