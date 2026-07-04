# Radar Visualization Tool — Plan (v3)

Status: **draft for review** — no implementation code has been written yet.

This revises v2: the `radar_bridge` poller node is dropped. The scan and
localization services stay pure request/response, callable by anyone
(the radar tool's own buttons today, a future "playback control"
component later). `sonar_node` and `sonar_localization` each publish a
full mirror of their own service response onto a topic immediately before
returning it to the caller — so every trigger, no matter who made it,
also shows up for passive dashboard subscribers.

## 1. Confirmed flow (from your description)

1. Radar tool (browser) calls `get_sonar_position` when the user presses
   a button.
2. `sonar_localization`'s callback calls `/sonar/scan` internally
   (already true today, 4 cardinal sectors).
3. `sonar_node`'s `/sonar/scan` callback processes the request, **then
   publishes its own response onto `/service/scan`**, then returns the
   response to whoever called it (in this case, `sonar_localization`).
4. `sonar_localization` processes the 4 scan results (RANSAC/SVD/
   triangulation, unchanged), **then publishes its own response onto
   `/service/localization`**, then returns the response to the caller (in
   this case, the radar tool).
5. The radar tool gets the direct call's response back (for immediate
   per-request validation/feedback), and separately/continuously listens
   on `/service/scan` and `/service/localization` to drive the dashboard
   — so it also sees the 4 intermediate `/service/scan` publishes that
   happen *during* its own localization call, and it would see activity
   from any other future caller (e.g. playback control) too.

This means the topic messages need to carry the **whole** response
(including `success`/`message`), not just the raw payload — a plain
`sensor_msgs/MultiEchoLaserScan` on `/service/scan` wouldn't carry
success/failure, and a plain `PoseWithCovarianceStamped` on
`/service/localization` wouldn't carry the pool dimensions. So each topic
gets a small new message type that mirrors its service's response fields
1:1.

## 2. ROS-side changes

### 2.1 `sub_interfaces/srv/SonarScan.srv` — add validation fields

```
uint16 start_angle
uint16 stop_angle
uint16 desired_range
---
sensor_msgs/MultiEchoLaserScan data
bool success
string message
```
`sonar_node.py`'s scan callback wraps the beam loop in try/except,
setting `success=False, message=str(exc)` on hardware error, `True`/`''`
otherwise.

### 2.2 New `sub_interfaces/msg/SonarScanResult.msg`

```
bool success
string message
sensor_msgs/MultiEchoLaserScan data
```
Mirrors the `SonarScan.srv` response exactly. `sonar_node.py` gets a
`create_publisher(SonarScanResult, '/service/scan', 10)` and publishes
this message right before every `return response` in the scan callback
— on success and on failure alike, so a topic-only subscriber can see
failed attempts too.

### 2.3 `sub_interfaces/srv/GetSonarPosition.srv` — add pool dimensions

```
---
bool success
string message
geometry_msgs/PoseWithCovarianceStamped pose
float64 pool_width_x
float64 pool_length_y
```
`sonar_localization.py` already has `self.pool_x`/`self.pool_y` from its
declared parameters — populated into the response on every exit path.

### 2.4 New `sub_interfaces/msg/SonarPositionResult.msg`

```
bool success
string message
geometry_msgs/PoseWithCovarianceStamped pose
float64 pool_width_x
float64 pool_length_y
```
Mirrors the `GetSonarPosition.srv` response exactly. `sonar_localization.py`
gets a `create_publisher(SonarPositionResult, '/service/localization', 10)`.
`sonar_position_callback` currently has 4 return points (early-outs for
"scan service unavailable," "pipeline error," "triangulation failed," and
the final success path) — each one publishes the mirrored message
immediately before returning, so every outcome (including failures) is
visible on the topic.

### 2.5 `sub_interfaces/CMakeLists.txt`

Add the two new `.msg` files to the existing `rosidl_generate_interfaces`
call (already lists `ThrusterCommand.msg`, `LedCommand.msg`, and both
`.srv` files, so this is one more line each, no new dependencies needed —
`sensor_msgs`/`geometry_msgs` are already listed).

### 2.6 Launch files

Only `sim.launch.yaml` needs a change now (no new node to launch,
`sonar_node`/`sonar_localization` already run everywhere they need to):
add the same `rosbridge_server`/`rosbridge_websocket` stanza already
present in `sub.launch.yaml` and `hardware.launch.yaml`.

## 3. FastAPI backend (`tools/radar/backend/`)

Now does two things (not subscribe-only, since the tool itself triggers
calls via buttons):

- **Trigger endpoints** — forward a button press to the actual ROS
  service via `roslibpy.Service`:
  - `POST /api/scan {start_angle, stop_angle, desired_range}` → calls
    `/sonar/scan`, returns `{success, message, data}` straight back to
    the browser for immediate feedback.
  - `POST /api/localize` → calls `get_sonar_position`, returns
    `{success, message, pose, pool_width_x, pool_length_y}`.
  - Both check `success` on the call return before treating the result
    as usable, per your validation requirement — a failed call surfaces
    `message` as an error in the UI rather than being drawn as if valid.
- **Live dashboard feed** — subscribes to `/service/scan`
  (`SonarScanResult`) and `/service/localization` (`SonarPositionResult`)
  via `roslibpy.Topic`, and pushes every message to connected browsers
  over the backend's own `WS /ws`, tagged by type. This is what actually
  drives the canvas — it reflects *all* activity on those topics, not
  just the responses to this browser's own button presses.
- `requirements.txt`: `fastapi`, `uvicorn[standard]`, `roslibpy`.
- `config.py`: rosbridge host/port (default `localhost:9090`), topic and
  service names, overridable via env var/CLI so it's easy to point at the
  Jetson from a dev laptop.

## 4. Frontend (`tools/radar/frontend/`)

- `index.html`, `css/style.css`.
- Controls: a "Scan" panel (start/stop/range inputs + "Trigger Scan"
  button) and a "Localize" panel ("Trigger Localization" button), each
  showing the last call's success/message.
- `js/app.js` — opens a `WebSocket` to the backend's `/ws`, dispatches
  `scan`/`localization` frames to the two view modules; wires the two
  buttons to `POST /api/scan` / `POST /api/localize`.
- `js/sonar-view.js` — Layer 1, polar sweep rendered from the latest
  `SonarScanResult` (skips drawing if `success` is false, shows the
  `message` as a status line instead).
- `js/pool-view.js` — Layer 2, pool rectangle sized from the latest
  `SonarPositionResult`'s `pool_width_x`/`pool_length_y`, AUV marker from
  `pose` (position + yaw from quaternion). Since a full localization call
  triggers 4 internal `/sonar/scan` calls first, Layer 1 will visibly
  update 4 times (one per cardinal sector) while a "Trigger Localization"
  call is still in flight, then Layer 2 updates once at the end — a
  reasonable side effect of the shared topic, not something the tool
  needs to special-case.

## 5. File structure

```
tools/radar/
  PLAN.md
  README.md
  backend/
    app.py
    config.py
    requirements.txt
  frontend/
    index.html
    css/style.css
    js/
      app.js
      sonar-view.js
      pool-view.js

workspace/packages/sub_interfaces/msg/SonarScanResult.msg       (new)
workspace/packages/sub_interfaces/msg/SonarPositionResult.msg   (new)
workspace/packages/sub_interfaces/srv/SonarScan.srv             (edit: +success/+message)
workspace/packages/sub_interfaces/srv/GetSonarPosition.srv      (edit: +pool_width_x/+pool_length_y)
workspace/packages/sub_interfaces/CMakeLists.txt                (edit: register 2 new msgs)
workspace/packages/sub_hardware/scripts/sonar_node.py           (edit: +publisher, +try/except)
workspace/packages/sub_control/scripts/sonar_localization.py   (edit: +publisher on every return path)
workspace/packages/sub_launch/launch/sim.launch.yaml            (edit: +rosbridge)
```

## 6. Open questions

1. Message/topic names — I used `SonarScanResult`/`SonarPositionResult`
   for the new `.msg` types and kept your exact topic names
   `/service/scan` / `/service/localization`. Fine, or do you have
   preferred names for the message types?
2. Should `sonar_node`/`sonar_localization` publish on *every* callback
   return (including early-exit failures like "scan service
   unavailable"), or only on the success path? I've defaulted to
   "always publish" so a topic-only observer can see failures too, but
   it's a one-line change to restrict it to success only if you'd rather.

## 7. Out of scope (v1)

- Recording/replaying scans.
- Auth on the rosbridge connection or the backend's own WebSocket
  (assumed trusted local network).
- 3D rendering — everything is a 2D top-down view.
