# Submarine Path Visualizer (`map_tool`)

Web-based 2D top-down visualizer for the ASUQTR AUV's `odometry/filtered`
track. The backend runs on a **developer laptop** and talks to a **remote ROS 2
system** (Jetson + submarine, or a desktop container) over **rosbridge
WebSocket**. No Docker on the laptop.

Two modes:

- **Live** — subscribes to `/odometry/filtered` through the remote rosbridge and
  draws the submarine + growing path in real time (~25 Hz).
- **Playback** — upload a rosbag; the backend extracts the odometry track and
  the frontend gives you a scrubbable timeline with play/pause/speed.

```
Laptop (browser) ──HTTP/WS──> FastAPI backend (this app) ──WS(9090)──> rosbridge (Jetson)
```

---

## Requirements

| Feature | Needs |
| --- | --- |
| Backend + Live mode | Python 3.10+, `pip install -r requirements.txt` |
| Playback (Option A, uploaded bags) | **ROS 2 Humble** libs on the laptop (`ros-humble-rosbag2-py`), sourced before launch |
| Playback (Option B, remote replay) | Nothing extra on the laptop — replay runs on the Jetson |

> `rosbag2-py` is **not** a pip package; it ships with ROS 2. Live mode and the
> whole REST/WebSocket layer work without any ROS install — only local bag
> extraction (Option A) needs it.

---

## Quick start

Given a Jetson at `192.168.1.100` running `sub.launch.yaml` (which includes
rosbridge on port 9090):

```bash
cd map_tool/
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Only needed for Option A (local rosbag extraction):
source /opt/ros/humble/setup.bash

export ROS_BRIDGE_IP=192.168.1.100      # or set it in the Settings panel
python app.py
```

Open <http://localhost:8001>. Click **Connect** in Live mode, or switch to
**Playback** and upload a bag.

Configuration can come from environment variables, a `.env` file
(copy `.env.example`), or the in-browser **Settings** panel (IP/port/API key,
persisted in `localStorage`).

---

## Playback options

**Option A — upload a bag (default).** Drag a `.mcap`, `.db3`, or a `.zip` of a
rosbag2 directory into the uploader. The backend validates it (topic present,
size within `MAX_UPLOAD_SIZE_MB`), extracts asynchronously with a progress bar,
and caches the result as JSON under `recordings/cache/`. Requires ROS 2 on the
laptop.

**Option B — replay on the Jetson.** No ROS on the laptop:

```bash
# on the Jetson
ros2 bag play <bag> --topics /odometry/filtered
```

Then use Live mode — the replayed messages stream through rosbridge exactly like
live data.

---

## Security

Designed for **trusted LANs**. Set `API_KEY` in `.env` to require a shared
secret (`X-API-Key` header on `/api/*`, `?key=` query param on `/ws/live`).
Uploaded filenames are sanitized against path traversal, and uploads are
size-capped. Transport is plain `ws://`/`http://`; for untrusted networks put a
TLS reverse proxy (nginx) in front.

---

## Configuration reference

See [`.env.example`](.env.example). Key variables:

| Variable | Default | Meaning |
| --- | --- | --- |
| `ROS_BRIDGE_IP` / `ROS_BRIDGE_PORT` | `127.0.0.1` / `9090` | remote rosbridge endpoint |
| `ODOM_TOPIC` | `/odometry/filtered` | topic to subscribe/extract |
| `BACKEND_HOST` / `BACKEND_PORT` | `0.0.0.0` / `8001` | listen address (browser uses `localhost`) |
| `API_KEY` | *(empty)* | shared secret; empty = no auth |
| `MAX_UPLOAD_SIZE_MB` | `500` | upload cap |
| `HEADING_OFFSET_DEG` | `0` | added to every yaw; fix ENU/NED mismatches here |

> **Frame convention:** yaw is interpreted as ENU (0 = East, +90° = North).
> Verify against your `ekf_node` output; if it differs, set `HEADING_OFFSET_DEG`.

---

## HTTP / WS API

| Method | Path | Purpose |
| --- | --- | --- |
| `GET` | `/` | frontend |
| `GET` | `/api/config` | backend defaults for the UI |
| `GET` | `/api/rosbags` | list known bags |
| `POST` | `/api/upload-rosbag` | multipart upload; starts extraction |
| `GET` | `/api/rosbag/{id}/status` | extraction status/progress (also kicks off pre-loaded bags) |
| `GET` | `/api/rosbag/{id}/metadata` | duration / start / end / count |
| `GET` | `/api/rosbag/{id}/odometry` | full extracted track |
| `DELETE` | `/api/rosbag/{id}` | delete bag + cache |
| `WS` | `/ws/live?ip=&port=&key=` | live odometry stream |

Live WS frames: `{"type":"odom","data":{x,y,heading,timestamp}}` and
`{"type":"status","state":...,"detail":...}`.

---

## Tests

```bash
cd map_tool/
pytest                # geometry, upload sanitization, odom parsing, app smoke
```

Tests need only the pip requirements (no ROS 2); rosbag2 imports are lazy.

---

## Layout

```
map_tool/
├── app.py                 FastAPI app (REST + /ws/live + static)
├── config.py              env/.env configuration
├── geometry.py            quaternion→yaw, angle wrap, backoff (dependency-free)
├── rosbridge_client.py    async rosbridge subscription + reconnect
├── rosbag_reader.py       rosbag2 extraction (lazy ROS 2 import)
├── upload_handler.py      filename sanitization + RosbagStore
├── static/                index.html, css/, js/ (vanilla, canvas)
├── tests/                 pytest suite
├── recordings/            uploads/ + cache/ (gitignored)
├── requirements.txt
└── .env.example
```
