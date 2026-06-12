#!/bin/bash
set -euo pipefail

SESSION="asuqtr-unity"
CONTAINER="ros2-desktop"

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed on the host. Install tmux or open the commands manually."
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Container '$CONTAINER' is not running."
  echo "Start it first with: ./start-desktop.sh"
  exit 1
fi

ROS_ENV='source /opt/ros/humble/setup.bash; source /opt/underlay_ws/install/setup.bash 2>/dev/null || true; source /workspace/install/setup.bash'

if ! docker exec "$CONTAINER" bash -lc 'test -f /workspace/install/setup.bash'; then
  echo "Container '$CONTAINER' is running, but /workspace/install/setup.bash is missing."
  echo "That usually means the container was started without mounting this repo workspace."
  echo
  echo "Fix:"
  echo "  docker stop $CONTAINER"
  echo "  ./start-desktop.sh"
  echo
  echo "Then, inside another host terminal:"
  echo "  cd $(pwd)"
  echo "  ./start-unity.sh"
  exit 1
fi

if ! docker exec "$CONTAINER" bash -lc "$ROS_ENV; ros2 pkg prefix sub_launch >/dev/null 2>&1"; then
  echo "ROS package 'sub_launch' is not available in '$CONTAINER'."
  echo "The workspace is probably not built or not sourced correctly."
  echo
  echo "Try:"
  echo "  docker exec -it $CONTAINER bash"
  echo "  cd /workspace"
  echo "  ./build.sh"
  echo
  echo "Then rerun:"
  echo "  ./start-unity.sh"
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[1/3] Closing old tmux session '$SESSION'..."
  tmux kill-session -t "$SESSION"
fi

echo "[2/3] Cleaning old Unity simulation ROS processes..."
docker exec "$CONTAINER" bash -lc '
  for pattern in \
    "[r]os2 launch sub_launch unity_sim.launch.yaml" \
    "[r]os2 launch sub_launch sub.launch.yaml" \
    "[r]os2 launch sub_launch lqr_tuning.launch.yaml" \
    "[c]ontrol_node.py" \
    "[r]obot_state_publisher" \
    "[r]osbridge_websocket" \
    "[v]ectornav" \
    "[m]ock_odom.py" \
    "[c]ontrol_node params" \
    "[r]osbridge clients" \
    "[/]thruster_cmd" \
    "[/]debug/target_pose" \
    "[/]odometry/filtered"
  do
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$pids" ]; then
      kill $pids 2>/dev/null || true
    fi
  done

  sleep 0.5

  for pattern in \
    "[r]os2 launch sub_launch unity_sim.launch.yaml" \
    "[r]os2 launch sub_launch sub.launch.yaml" \
    "[r]os2 launch sub_launch lqr_tuning.launch.yaml" \
    "[c]ontrol_node.py" \
    "[r]obot_state_publisher" \
    "[r]osbridge_websocket" \
    "[v]ectornav" \
    "[m]ock_odom.py" \
    "[c]ontrol_node params" \
    "[r]osbridge clients" \
    "[/]thruster_cmd" \
    "[/]debug/target_pose" \
    "[/]odometry/filtered"
  do
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$pids" ]; then
      kill -9 $pids 2>/dev/null || true
    fi
  done
'

run_ros_cmd() {
  local cmd="$1"
  printf "docker exec -e TERM=xterm-256color %q bash -lc %q" "$CONTAINER" "export TERM=xterm-256color; $ROS_ENV; $cmd"
}

monitor_cmd() {
  local title="$1"
  local cmd="$2"
  local refresh="${3:-0}"

  if [ "$refresh" = "0" ]; then
    run_ros_cmd "printf '\033]2;%s\033\\\\' '$title'; clear; echo '===== $title ====='; echo; $cmd"
  else
    run_ros_cmd "printf '\033]2;%s\033\\\\' '$title'; while true; do clear; echo '===== $title ====='; echo; $cmd; sleep $refresh; done"
  fi
}

echo "[3/3] Starting tmux dashboard '$SESSION'..."

tmux new-session -d -s "$SESSION" -n "launch" "$(run_ros_cmd "ros2 launch sub_launch unity_sim.launch.yaml")"

sleep 2

tmux new-window -t "$SESSION" -n "dashboard" "$(monitor_cmd "/thruster_cmd" "ros2 topic echo /thruster_cmd sub_interfaces/msg/ThrusterCommand --qos-reliability best_effort" 0)"
tmux split-window -h -t "$SESSION:dashboard" "$(monitor_cmd "/debug/target_pose" "ros2 topic echo /debug/target_pose geometry_msgs/msg/PoseStamped --qos-reliability best_effort" 0)"
tmux split-window -v -t "$SESSION:dashboard.0" "$(monitor_cmd "/odometry/filtered" "ros2 topic echo /odometry/filtered nav_msgs/msg/Odometry --qos-reliability reliable" 0)"
tmux split-window -v -t "$SESSION:dashboard.1" "$(monitor_cmd "control_node params" "ros2 param get /control_node control_mode; echo; ros2 topic info /thruster_cmd -v" 2)"
tmux select-layout -t "$SESSION:dashboard" tiled

tmux new-window -t "$SESSION" -n "system" "$(monitor_cmd "ROS nodes/topics" "ros2 node list; echo; ros2 topic list -t" 2)"
tmux split-window -h -t "$SESSION:system" "$(monitor_cmd "rosbridge clients" "ros2 topic echo /client_count --once 2>/dev/null || true; echo; ros2 topic echo /connected_clients --once 2>/dev/null || true" 2)"
tmux select-layout -t "$SESSION:system" tiled

tmux select-window -t "$SESSION:dashboard"

echo
echo "Unity simulation ROS environment started."
echo "Attach dashboard:"
echo "  tmux attach -t $SESSION"
echo
echo "Useful keys:"
echo "  Ctrl-b arrow keys    move between panes"
echo "  Ctrl-b n             next window"
echo "  Ctrl-b p             previous window"
echo "  Ctrl-b w             choose a window"
echo "  Ctrl-b d    detach without stopping"
echo
echo "Stop everything later:"
echo "  tmux kill-session -t $SESSION"
