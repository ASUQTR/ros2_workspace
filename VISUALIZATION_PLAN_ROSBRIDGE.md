# Submarine Path Visualization Tool — Revised Plan (ROSbridge Architecture)

## Overview

**Architecture**: Web-based visualization tool where the backend runs on a **development laptop** and connects to a **remote ROS2 system** (Jetson + submarine or desktop container) via **rosbridge WebSocket**. This enables:
- Developer laptop acts as visualization/playback server (no Docker required)
- ROS2 system is remote (could be physical submarine on Jetson, or a container on another PC)
- No direct file system access needed; data flows through rosbridge + rosbag replay

> **⚠️ Dependency note**: `rosbag2-py` has deep ROS2 library dependencies and is **not standalone**. The laptop needs either a minimal ROS2 Humble installation (apt packages `ros-humble-rosbag2-py`) or the rosbag extraction must run on the Jetson (Option B). See [Rosbag Playback Flow](#rosbag-playback-flow-two-options) for details.

---

## High-Level Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│ Developer Laptop (Windows/Mac/Linux)                            │
│ ┌─────────────────────────────────────────────────────────────┐   │
│ │ path_visualizer/ (Python FastAPI service)                 │   │
│ │ ├─ Backend: Connects to remote rosbridge WebSocket        │   │
│ │ │  ├─ Real-time: Subscribe to odometry/filtered via RB    │   │
│ │ │  └─ Playback: Upload rosbag → extract locally → serve   │   │
│ │ └─ Frontend: HTML5 Canvas visualization + controls        │   │
│ │    └─ Fetch data from local backend API                   │   │
│ └─────────────────────────────────────────────────────────────┘   │
│                              ▲                                  │
│                              │ HTTP/WebSocket                   │
│                    (localhost:8001)                             │
└──────────────────────────────┼──────────────────────────────────┘
                               │
                               │ Network (LAN/WiFi)
                               │
┌──────────────────────────────┼──────────────────────────────────┐
│ Remote ROS2 System (Jetson/Container on separate PC)            │
│ ┌────────────────────────────────────────────────────────────┐  │
│ │ ROS2 Nodes & Topics                                        │  │
│ │ ├─ odometry/filtered (from ekf_node) @ 25 Hz               │  │
│ │ ├─ rosbridge_server (WebSocket endpoint)                   │  │
│ │ └─ Optional: ros2 bag play <rosbag> (for playback)         │  │
│ └────────────────────────────────────────────────────────────┘  │
│                         ▲                                       │
│         WebSocket port 9090 (or custom)                         │
└─────────────────────────────────────────────────────────────────┘
```

---

## Data Flow

### **Live Mode**

1. **User opens browser** → `http://localhost:8001/`
2. **Frontend loads** and auto-connects to `/api/live-mode` endpoint
3. **Backend** (`app.py`):
   - Connects to remote rosbridge WebSocket: `ws://<ROS_PC_IP>:9090`
   - Subscribes to `/odometry/filtered` topic
   - Receives `nav_msgs/Odometry` messages from rosbridge in JSON format
   - Extracts: `x`, `y`, `heading` (yaw), `timestamp`
   - Pushes to frontend via local WebSocket at ~25 Hz
4. **Frontend** re-renders submarine position + extends path line
5. **Continuous streaming** until user closes browser or switches mode

### **Playback Mode**

1. **User selects "Playback Mode"** in frontend
2. **User uploads rosbag file** (or selects from pre-uploaded list) via form
   - Either direct file upload (multipart/form-data to `/api/upload-rosbag`)
   - Or pre-load from `recordings/` subdirectory if file already exists
3. **Backend** (`rosbag_reader.py`):
   - Validates rosbag integrity and confirms `/odometry/filtered` topic exists with messages
   - Receives rosbag file (or references local file)
   - Uses rosbag2 Python API to extract all `odometry/filtered` messages (async, with progress)
   - Converts to JSON: `[{x, y, heading, ts}, ...]`
   - Returns to frontend
4. **Frontend**:
   - Renders timeline slider (0–100% of recording duration)
   - Displays all extracted points as path line
   - On user interaction (slider, play, pause, speed):
     - Update current submarine position based on timeline
     - Re-render canvas
5. **No real-time ROS connection needed** for playback — all data is local

---

## Backend Components

### **1. Main FastAPI App (`app.py`)**

**Responsibilities**:
- Serve HTML frontend
- Expose REST API endpoints
- Manage WebSocket connections (live mode)
- Handle rosbag file uploads/processing
- Maintain rosbridge connection state
- Enforce API key authentication on all non-frontend routes

**Key endpoints**:
- `GET /` — Serve `static/index.html`
- `GET /api/config` — Return backend config (remote ROS IP, rosbridge port, available modes)
- `POST /api/upload-rosbag` — Accept rosbag file upload, store locally, trigger async extraction
- `GET /api/rosbags` — List available rosbag files (uploaded or pre-loaded)
- `GET /api/rosbag/<rosbag_id>/metadata` — Return rosbag duration, start/end timestamps, odometry count
- `GET /api/rosbag/<rosbag_id>/odometry` — Return full extracted odometry data (JSON array)
- `GET /api/rosbag/<rosbag_id>/status` — Return async extraction progress (0–100%)
- `WebSocket /ws/live` — Stream live odometry data from rosbridge subscription

### **2. ROSbridge Client (`rosbridge_client.py`)**

**Responsibilities**:
- Maintain WebSocket connection to remote rosbridge server
- Subscribe to `/odometry/filtered` topic
- Decode incoming `nav_msgs/Odometry` JSON messages
- Extract position and heading (yaw from quaternion)
- Provide subscription callback for live streaming
- Reconnect with exponential backoff on disconnection
- Send/receive WebSocket ping-pong heartbeats to detect stale connections

**Key functions**:
- `connect(ros_ip, ros_port)` — Establish rosbridge connection
- `subscribe_odometry(callback)` — Subscribe with callback handler
- `unsubscribe()` — Clean up subscription
- `parse_odom_message(msg)` — Extract `{x, y, heading, timestamp}` from nav_msgs/Odometry JSON
- `shutdown()` — Close connection gracefully

**Reconnection policy**: Use exponential backoff starting at 1 s, doubling each attempt, capped at 30 s. Emit distinct connection states: `connecting`, `connected`, `reconnecting`, `failed`.

**Heartbeat**: Send a rosbridge ping every 5 s; treat no pong within 10 s as a dropped connection and re-enter reconnection flow.

### **3. Rosbag2 Reader (`rosbag_reader.py`)**

**Responsibilities**:
- Open rosbag2 files
- Extract all messages from `odometry/filtered` topic
- Convert nav_msgs/Odometry to JSON format
- Cache extracted data in memory or disk (for performance)
- Emit extraction progress events for async status polling

**Key functions**:
- `extract_odometry(rosbag_path, progress_cb)` — Returns `{metadata: {start, end, duration, count}, odometry: [...]}`; calls `progress_cb(pct)` periodically
- `validate_rosbag(path)` — Check file is valid, contains `/odometry/filtered` topic with at least one message; raises descriptive error otherwise
- `cache_key(path)` — Generate cache filename

> **⚠️ Runtime requirement**: `rosbag2-py` requires ROS2 shared libraries at runtime. On the laptop this means `source /opt/ros/humble/setup.bash` before starting the backend, or installing the `ros-humble-rosbag2-py` apt package. If a ROS2 install on the laptop is undesirable, use **Option B** (remote extraction) instead.

### **4. Rosbag Upload Handler (`upload_handler.py`)**

**Responsibilities**:
- Accept multipart rosbag file uploads
- **Validate filename** (allow only alphanumeric, hyphens, underscores, and `.db3`/`.mcap` extensions; reject path traversal sequences such as `../`)
- Enforce file size limit (`MAX_UPLOAD_SIZE_MB`)
- Save to disk (`recordings/uploads/`)
- Trigger async extraction via rosbag_reader
- Return rosbag ID + metadata to frontend

**Key functions**:
- `sanitize_filename(name)` → safe filename string; raises `ValueError` on invalid input
- `save_upload(file)` → `rosbag_id`
- `extract_and_cache(rosbag_id)` → starts background thread, returns task ID for status polling

---

## Frontend Components

### **1. Mode Selector & UI (`static/index.html` + `static/css/style.css`)**

**Layout**:
- **Header bar**:
  - Mode toggle: "Live" vs "Playback" (radio buttons or tabs)
  - Status indicator (connecting / connected / reconnecting / disconnected, rosbag loaded, etc.)
  - Settings gear icon (ROS IP/port config)

- **Live Mode Panel** (initially hidden):
  - "Connect to ROSbridge" button
  - Status: "Connecting…" / "Connected" / "Reconnecting (attempt N)…" / "Error: timeout"
  - Real-time statistics: current position, heading, update rate

- **Playback Mode Panel** (initially hidden):
  - "Upload Rosbag" file input
  - Progress bar shown during upload and async extraction
  - OR dropdown list of pre-loaded rosbags (with delete options)
  - Timeline slider + time display (MM:SS / total MM:SS)
  - Playback controls: ⏵ Play / ⏸ Pause / ⏹ Stop
  - Speed selector: 0.5x, 1x, 2x, 4x, 8x
  - Rosbag metadata: Duration, point count, file size
  - User-facing error messages (corrupt file, missing topic, oversized upload)

- **Canvas area**: Full viewport for 2D visualization
  - 2D top-down overhead view
  - Path rendered as connected line
  - Submarine marker with heading gizmo
  - Grid/crosshairs for reference
  - Zoom/pan via mouse (wheel to zoom, drag to pan)
  - "Fit to view" button

### **2. Canvas Renderer (`static/js/canvas-renderer.js`)**

**Responsibilities**:
- 2D HTML5 Canvas rendering via `requestAnimationFrame` (not fixed intervals)
- Coordinate transformation (world → screen, zoom/pan)
- Path line rendering (with decimation if >10 000 points to avoid frame drops)
- Submarine gizmo rendering (directional arrow/triangle)

**Key functions**:
- `render_path(points)` — Draw connected polyline through all points (decimate if large)
- `render_submarine(x, y, heading)` — Draw triangle/arrow at position with rotation
- `render_grid()` — Draw optional reference grid
- `handle_zoom(delta)` — Update camera zoom level
- `handle_pan(dx, dy)` — Update camera pan offset
- `world_to_screen(x, y)` → `{sx, sy}` — Coordinate transformation
- `fit_to_bounds(points)` — Auto-zoom to show all points

### **3. Live Mode Handler (`static/js/live-mode.js`)**

**Responsibilities**:
- WebSocket connection to `/ws/live`
- Append incoming odometry to local path array
- Maintain rolling buffer of last 50 points (so path survives brief disconnects)
- Trigger canvas re-render
- Implement exponential backoff reconnection with distinct UI states

**Key functions**:
- `connect_live()` — Open WebSocket, reset backoff counter
- `on_odom_message(data)` — Append to path, trigger render
- `on_connection_lost()` — Show "Reconnecting…" UI, schedule retry with backoff
- `disconnect()` — Close WebSocket, clear retry timer

**Reconnection**: Start retry at 1 s, double each attempt, cap at 30 s. Display countdown: "Reconnecting in 8 s…". Offer a "Retry now" button.

### **4. Playback Mode Handler (`static/js/playback-mode.js`)**

**Responsibilities**:
- Load rosbag list from `/api/rosbags`
- Handle file upload to `/api/upload-rosbag`, poll `/api/rosbag/<id>/status` for extraction progress
- Fetch rosbag metadata and full odometry data
- Manage timeline slider (user drag, auto-advance on play)
- Play/pause/speed state machine
- Interpolate submarine position (position and heading) at current timeline point

**Key functions**:
- `load_rosbag_list()` — Fetch and display available bags
- `upload_rosbag(file)` — Send to backend, poll status endpoint, show progress bar
- `load_rosbag(id)` — Fetch metadata + odometry data
- `on_timeline_change(t)` — Update current frame index, re-render
- `play()` / `pause()` — Start/stop timeline advance
- `set_playback_speed(factor)` — 0.5x–8x
- `interpolate_position(t)` → `{x, y, heading}` — Linear interpolation for position; slerp (or angle-aware lerp) for heading to avoid wrap-around artifacts

### **5. Settings/Config Panel (`static/js/settings.js`)**

**Responsibilities**:
- Display/edit ROS connection parameters
- Store in localStorage (persist across sessions)
- Validate IP/port input
- Trigger backend re-connection with new config

**Key functions**:
- `load_config()` — Fetch `/api/config`, display in UI
- `save_config()` — Update and store in localStorage
- `validate_ros_endpoint()` — Test connection

---

## Configuration

### **Backend Configuration** (`config.py` or `.env` file)

```ini
# Remote ROS2 system
ROS_BRIDGE_IP=192.168.1.100          # Jetson or remote PC IP
ROS_BRIDGE_PORT=9090                 # rosbridge_server port (default 9090)
ROS_BRIDGE_URL=ws://${ROS_BRIDGE_IP}:${ROS_BRIDGE_PORT}

# Backend server
# NOTE: 0.0.0.0 is a listen address, not a client-reachable address.
# The browser must connect to http://localhost:8001 (or the machine's actual LAN IP).
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8001
DEBUG=False

# Security (simple API key for LAN deployments)
# Set a non-empty value to require X-API-Key header on all /api/* routes.
API_KEY=

# Rosbag storage
ROSBAG_UPLOAD_DIR=./recordings/uploads/
ROSBAG_CACHE_DIR=./recordings/cache/
MAX_UPLOAD_SIZE_MB=500               # Limit rosbag file size

# Timeouts
ROSBRIDGE_TIMEOUT_SEC=5
# Reconnect uses exponential backoff (1s → 2s → 4s … up to 30s cap)
ROSBRIDGE_RECONNECT_MAX_SEC=30
ROSBRIDGE_HEARTBEAT_INTERVAL_SEC=5
ROSBRIDGE_HEARTBEAT_TIMEOUT_SEC=10
```

### **Frontend Configuration** (stored in browser localStorage)

- ROS bridge IP/port (editable via settings panel)
- Last used mode (live vs playback)
- Canvas zoom/pan camera state (per session)
- Playback speed preference
- Color scheme (optional: dark/light theme)

---

## Network & Connectivity

### **Quick-start: I have a Jetson at `192.168.1.100`, how do I run this?**

```bash
# On developer laptop (Linux with ROS2 Humble, or see note below for no-ROS option)
source /opt/ros/humble/setup.bash   # required for rosbag2-py
export ROS_BRIDGE_IP=192.168.1.100
cd path_visualizer/
pip install -r requirements.txt
python app.py
# Open http://localhost:8001 in your browser
```

If you do **not** have ROS2 on the laptop, use **Option B** (remote bag replay) for playback — no rosbag2-py required.

### **Prerequisite: Remote ROS2 System Setup**

On the **Jetson/container PC**:

1. **Verify rosbridge_server is running**:
   - Should be launched by `ros2 launch sub_launch sub.launch.yaml` (already includes rosbridge)
   - Listens on `0.0.0.0:9090` (accessible from network)
   - Verify: `netstat -tlnp | grep 9090`

2. **Network accessibility**:
   - Jetson/container must be on same network as developer laptop
   - Firewall must allow port 9090 (TCP + WebSocket)
   - Developer laptop must know Jetson/container IP (**static IP strongly recommended**; mDNS (`jetson.local`) is a fallback if avahi is configured)

### **Developer Laptop Setup**

1. Clone/sync `path_visualizer/` from workspace
2. Install Python dependencies: `pip install -r requirements.txt`
3. Set environment variable (or edit `config.py`):
   ```bash
   export ROS_BRIDGE_IP=192.168.1.100  # Jetson IP
   ```
4. Run: `python app.py`
5. Open browser: `http://localhost:8001`

### **Rosbag Playback Flow** (Two Options)

#### **Option A: User Uploads Rosbag File**

1. User has rosbag file on laptop
2. UI: Click "Upload Rosbag" → select file → submit
3. Backend validates filename (no path traversal, allowed extension) and file size
4. Backend validates rosbag integrity and confirms `/odometry/filtered` topic exists
5. Backend starts async extraction with progress reporting; frontend polls `/api/rosbag/<id>/status`
6. Frontend loads extracted data, renders playback

**Pros**: Fully isolated, simple UX, no ROS connection needed during playback
**Cons**: Rosbag file must be copied to laptop first; **requires minimal ROS2 install on laptop** for `rosbag2-py`

#### **Option B: Rosbag Replay on Remote ROS (No ROS2 on Laptop)**

1. On Jetson/container: `ros2 bag play <rosbag_path> --topic /odometry/filtered`
   - This re-publishes recorded messages to the topic
2. Backend subscribes to `/odometry/filtered` via rosbridge (same as live mode)
3. Frontend receives replayed data, renders with playback controls

**Pros**: No ROS2 on laptop at all; uses existing ROS infrastructure; handles any rosbag format natively
**Cons**: Requires SSH/access to Jetson to start replay; timeline scrubbing requires `ros2 bag play --start-offset` (limited precision)

**Recommendation**: If laptop has ROS2 installed, use **Option A** for full timeline control. Otherwise use **Option B**.

---

## Data Format & Message Parsing

### **Odometry Message from rosbridge**

rosbridge transmits `nav_msgs/Odometry` as JSON:

```json
{
  "header": {
    "seq": 1234,
    "stamp": {"secs": 1656432100, "nsecs": 123456789},
    "frame_id": "odom"
  },
  "child_frame_id": "base_link",
  "pose": {
    "pose": {
      "position": {"x": 1.5, "y": 2.3, "z": -0.8},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.707, "w": 0.707}
    },
    "covariance": [...]
  },
  "twist": { ... }
}
```

### **Extraction** (Backend does this)

From each message:
- **Position**: `msg.pose.pose.position.x`, `.y` (ignore `.z` for top-down view)
- **Heading (Yaw)**: Convert quaternion `{x, y, z, w}` → Euler angles, extract yaw
  - Formula: `yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))`
  - Or use library: `tf_transformations.euler_from_quaternion([x, y, z, w])[2]`
  - **⚠️ Coordinate frame**: Verify whether `ekf_node` publishes in ENU (East-North-Up) or NED (North-East-Down). In ROS2/EKF default ENU: yaw=0 points East, yaw=π/2 points North. Confirm against the URDF/EKF config before rendering. Add a `HEADING_OFFSET_DEG` config knob if convention differs.
- **Timestamp**: `msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9` (seconds since epoch)
  - Convert to human-readable format in frontend

### **Frontend Internal Format**

```typescript
interface OdomPoint {
  x: number;           // meters
  y: number;           // meters
  heading: number;     // radians (convention: verify ENU/NED from ekf_node)
  timestamp: number;   // seconds (unix epoch or relative)
  index: number;       // for timeline indexing
}

interface OdomData {
  points: OdomPoint[];
  startTime: number;
  endTime: number;
  duration: number;    // seconds
}
```

---

## Security

This tool is designed for **trusted LAN deployment only**. The following baseline controls are included:

| Concern | Mitigation |
|---------|------------|
| **Unauthorized API access** | Optional `X-API-Key` header check on all `/api/*` routes (set `API_KEY` in `.env`) |
| **File path traversal** | `sanitize_filename()` in `upload_handler.py` strips `../`, rejects absolute paths |
| **Oversized uploads** | Enforce `MAX_UPLOAD_SIZE_MB` before writing to disk |
| **DoS via repeated extraction** | Check cache before re-extracting; queue requests per rosbag ID |
| **Unencrypted transport** | Uses `ws://` and `http://`; acceptable for LAN. For untrusted networks, put a TLS reverse proxy (nginx) in front of both port 8001 and 9090 |

---

## File Structure

```
path_visualizer/
├── app.py                           # FastAPI main app
├── config.py                        # Configuration, .env loading
├── rosbridge_client.py              # ROSbridge connection & subscription
├── rosbag_reader.py                 # Rosbag extraction (rosbag2 API)
├── upload_handler.py                # Rosbag file upload, validation & sanitization
├── requirements.txt                 # Dependencies
├── .env.example                     # Template config (copy to .env and fill in)
├── recordings/
│   ├── uploads/                     # User-uploaded rosbags
│   └── cache/                       # Extracted odometry (JSON cache)
├── static/
│   ├── index.html                   # Main frontend
│   ├── css/
│   │   └── style.css                # Styling
│   └── js/
│       ├── canvas-renderer.js       # Canvas drawing logic
│       ├── live-mode.js             # Live streaming handler
│       ├── playback-mode.js         # Playback & timeline logic
│       ├── settings.js              # Config panel
│       └── utils.js                 # Helpers (quaternion conversion, etc.)
├── tests/
│   ├── test_quaternion.py           # Unit tests for quaternion→yaw conversion
│   ├── test_upload_handler.py       # Filename sanitization & size checks
│   └── test_rosbridge_client.py     # Reconnection / heartbeat logic
├── README.md                        # Setup & usage docs
└── .gitignore                       # Ignore rosbag uploads/cache
```

---

## Dependencies

### **Python** (`requirements.txt`)

```
fastapi==0.104.1
uvicorn==0.24.0
websockets==12.0
numpy==1.24.3
scipy==1.11.2
# rosbag2-py requires ROS2 Humble libraries at runtime.
# On laptop: source /opt/ros/humble/setup.bash before starting.
# Pin to the version shipped with Humble; do not pin lower.
rosbag2-py
rosbag2-storage-default-plugins
python-dotenv==1.0.0           # Environment config
```

> **Note on `rclpy`**: Remove the `rclpy` dependency. It was listed as "fallback if needed" but rosbridge communication is handled over plain WebSocket — `rclpy` would require a full ROS2 context, defeating the purpose.

### **Frontend** (CDN/bundled)

- **Canvas rendering**: Native HTML5 Canvas (no external library needed)
- **Math**: `gl-matrix.js` or custom helpers (quaternion, vector ops)
- **UI**: Vanilla JavaScript is acceptable for MVP. If state management for live/playback mode, reconnection state, and progress reporting grows complex, consider a minimal reactive library (Alpine.js adds ~15 kB and avoids the full React/Vue build toolchain)

**Browser support**: Chrome 90+, Firefox 88+, Safari 15+. Edge (Chromium-based) is fully supported. Do not implement HTTP long-polling fallback — all target environments support WebSocket.

---

## Advantages of ROSbridge Architecture

1. **Isolation**: Developer laptop has no Docker required
   - Only Python + FastAPI + rosbag2 library (note: rosbag2 still needs ROS2 libs; see dependency note)
   - ROS2 full stack stays on remote system (Jetson)

2. **Network-friendly**:
   - Works over LAN, WiFi, or VPN
   - Compressed JSON over WebSocket (efficient)
   - Exponential-backoff reconnection with heartbeat detection

3. **Flexible playback**:
   - User uploads rosbag from any source
   - No need for shared file system (NFS, SMB)
   - Backend does extraction locally (Option A) or remote replay (Option B)

4. **Modularity**:
   - Backend and frontend completely decoupled
   - Could serve multiple frontends simultaneously
   - Easy to add additional visualization clients

5. **No Docker required** on developer machine
   - Run `python app.py` on Windows/Mac/Linux (with ROS2 Humble for Option A)
   - Access via browser from anywhere on network

---

## Potential Challenges & Mitigations

| Challenge | Cause | Mitigation |
|-----------|-------|-----------|
| **Network latency** | WiFi jitter, Jetson CPU load | Buffer 1–2 frames, average timestamps |
| **Rosbag extraction slow** | Large file, many messages | Async extraction with progress bar; cache result to disk |
| **Memory overflow** | Large rosbag → huge JSON array | Implement pagination/streaming (fetch chunk by chunk) |
| **Rosbridge disconnect** | Network hiccup, ROS restart | Exponential backoff auto-reconnect + heartbeat detection |
| **Stale connection not detected** | TCP keepalive gaps | WS ping-pong every 5 s; treat 10 s no-pong as dead |
| **Browser WebSocket limit** | Old browser, proxy | Target Chrome/Firefox/Safari/Edge only; no long-polling fallback |
| **Rosbag file corrupt** | Partial upload, disk error | Validate before extraction; return descriptive error to UI |
| **Missing odometry topic** | Wrong rosbag file | `validate_rosbag()` checks topic existence before extraction |
| **Path traversal upload** | Malicious filename | `sanitize_filename()` in upload handler |
| **Wrong heading convention** | ENU vs NED mismatch | Verify against URDF/EKF config; expose `HEADING_OFFSET_DEG` |

---

## Verification Plan

1. **Backend startup** (on laptop):
   - `python app.py` → logs "Uvicorn running on http://0.0.0.0:8001"
   - Verify no full ROS2 stack required (only rosbag2 libs for Option A)

2. **Frontend loads**:
   - Browser: `http://localhost:8001` → HTML loads, no errors in console
   - Settings panel shows ROS bridge IP input

3. **Quaternion conversion unit test** (run before any other tests):
   - `pytest tests/test_quaternion.py -v`
   - Covers: identity quaternion → yaw=0; 90° yaw; 180° yaw; negative yaw; wrap-around at ±π
   - Verify against known ENU frame: identity should yield heading East (0 rad)

4. **Live mode** (with remote ROS running):
   - Input Jetson IP in settings, click "Connect"
   - Backend connects to rosbridge WebSocket (verify in backend logs)
   - Submarine marker appears and updates smoothly @ 25 Hz
   - Path line grows as submarine moves
   - Heading gizmo rotates correctly
   - Kill network → UI shows "Reconnecting…" with countdown; restore network → auto-reconnects

5. **Playback mode**:
   - Upload a real rosbag from Jetson (or a small synthetic one)
   - Backend extracts with progress bar visible in UI
   - Test with corrupt rosbag → clear error message, no crash
   - Test with rosbag missing `/odometry/filtered` → clear error message
   - Frontend loads and renders all points
   - Timeline slider works: drag → submarine jumps to correct position
   - Heading interpolation: verify no discontinuous jumps at 0/2π wrap-around
   - Play/pause/speed controls function
   - Stress test: upload a ~500 MB rosbag; verify no OOM crash

6. **Security checks**:
   - Upload a file named `../../etc/passwd.db3` → rejected with 400
   - Upload a file exceeding `MAX_UPLOAD_SIZE_MB` → rejected with 413
   - If `API_KEY` is set, request without header → 401

7. **Network isolation**:
   - Disconnect laptop from Jetson network
   - Playback mode still works (local extracted data)
   - Live mode shows "Connection lost" gracefully with reconnect countdown
   - Reconnect: "Connect" button re-establishes websocket

---

## Future Enhancements (Post-MVP)

1. **Sonar localization overlay**: If sonar position fixes available, overlay absolute pool coordinates
2. **Multi-rosbag comparison**: Load and render multiple mission paths side-by-side
3. **Telemetry overlay**: Display DVL velocity, depth, IMU data alongside position
4. **Trajectory optimization**: Compute and display "shortest path" comparison vs actual path
5. **Export**: Save rendered path as image or video (ffmpeg backend)
6. **Synchronized multi-client**: Multiple browser viewers watch same live stream in sync
7. **Rosbag recording trigger**: Frontend button to remote-start/stop rosbag recording on Jetson
8. **TLS/mTLS**: Nginx reverse proxy with self-signed certs for competition network environments

---

## Summary

This revised architecture allows the **visualization tool to run on a developer laptop** (no Docker, no full ROS2 stack) while connecting to a **remote ROS2 system via rosbridge WebSocket**. Playback uses rosbag files (either uploaded or pre-transferred) and requires a minimal ROS2 install for `rosbag2-py` on the laptop (or uses remote replay to avoid this). Live mode streams via rosbridge subscription. The backend is a minimal FastAPI server with async extraction, exponential-backoff reconnection, and input validation. The frontend is pure HTML5 Canvas. Network-isolated, secure for LAN use, and developer-friendly.
