#!/bin/bash
# =============================================================================
# ASUQTR ROS 2 — Desktop aliases (inside the ros2-desktop container)
# Auto-sourced on every terminal open inside the container.
# To source manually: source /workspace/asuqtr_aliases.sh
# =============================================================================

# --------------- BUILD -------------------------------------------------------

# Build the full workspace and source it
alias asuqtr-build='cd /workspace && bash build.sh && source install/setup.bash'

# --------------- LAUNCH ------------------------------------------------------

# Desktop mode: IMU only (no Bar30 / DVL / GPIO / Actuators)
alias asuqtr-launch='ros2 launch sub_launch desktop.launch.yaml'

# Full system launch (Xavier only — all hardware nodes)
alias asuqtr-launch-full='ros2 launch sub_launch sub.launch.yaml'

# --------------- LQR CONTROLLER MONITORING -----------------------------------

# Commands sent to the 8 thrusters in Newtons — direct LQR output
alias asuqtr-motors='ros2 topic echo /thruster_cmd'

# Internal angles computed by the LQR
alias asuqtr-lqr-angles='ros2 topic echo /debug/lqr_angles'

# Velocities commanded by the LQR
alias asuqtr-lqr-vel='ros2 topic echo /debug/lqr_velocity'

# Accelerations commanded by the LQR
alias asuqtr-lqr-accel='ros2 topic echo /debug/lqr_accel_cmd'

# Internal LQR dynamics (computed forces/torques)
alias asuqtr-lqr-dyn='ros2 topic echo /debug/lqr_dynamics'

# --------------- EKF LOCALIZATION MONITORING ---------------------------------

# Estimated position (world frame, in meters)
alias asuqtr-pos='ros2 topic echo /odometry/filtered --field pose.pose.position'

# Estimated linear velocities (body frame, in m/s)
alias asuqtr-vel='ros2 topic echo /odometry/filtered --field twist.twist.linear'

# Full odometry (position + orientation + velocities)
alias asuqtr-odom='ros2 topic echo /odometry/filtered'

# --------------- IMU MONITORING ----------------------------------------------

# Raw VectorNav IMU data (orientation quaternion + angular velocity)
alias asuqtr-imu='ros2 topic echo /vectornav/imu'

# EKF output orientation (quaternion)
alias asuqtr-ekf-orient='ros2 topic echo /odometry/filtered --field pose.pose.orientation'

# 3D visual dashboard: submarine model in NED frame, toggle IMU/EKF with I/O/T keys
alias asuqtr-dashboard='ros2 run sub_control imu_realtime_dashboard.py'


# --------------- COMMANDS ----------------------------------------------------

# Send a target pose to the controller (requires control_mode: lqr_tuning in params.yaml)
# Position in meters — Orientation Roll/Pitch/Yaw in degrees (mapped to x/y/z)
# Usage: asuqtr-target <x> <y> <z> <roll> <pitch> <yaw>
# Example: asuqtr-target 0 0 0.5 0 0 45   (dive 0.5m, heading 45 deg)
asuqtr-target() {
    local x=${1:-0.0} y=${2:-0.0} z=${3:-0.0}
    local roll=${4:-0.0} pitch=${5:-0.0} yaw=${6:-0.0}
    ros2 topic pub -1 /debug/target_pose geometry_msgs/msg/PoseStamped \
        "{pose: {position: {x: $x, y: $y, z: $z}, orientation: {x: $roll, y: $pitch, z: $yaw}}}"
}

# --------------- UTILITIES ---------------------------------------------------

# List all active ROS 2 topics
alias asuqtr-topics='ros2 topic list'

# Publication frequency of a topic
# Usage: asuqtr-hz /thruster_cmd
alias asuqtr-hz='ros2 topic hz'

# Print all available aliases
asuqtr-help() {
    echo ""
    echo "  ASUQTR ROS 2 — Desktop Aliases"
    echo "  ================================"
    echo "  BUILD"
    echo "    asuqtr-build                   Build & source the workspace"
    echo ""
    echo "  LAUNCH"
    echo "    asuqtr-launch                  Desktop: IMU only"
    echo "    asuqtr-launch-full             Xavier: full system (all hardware)"
    echo ""
    echo "  LQR CONTROLLER"
    echo "    asuqtr-motors                  Thruster forces in Newtons (/thruster_cmd)"
    echo "    asuqtr-lqr-angles              Internal LQR angles"
    echo "    asuqtr-lqr-vel                 LQR velocities"
    echo "    asuqtr-lqr-accel               LQR accelerations"
    echo "    asuqtr-lqr-dyn                 LQR dynamics"
    echo ""
    echo "  EKF LOCALIZATION"
    echo "    asuqtr-pos                     Estimated XYZ position (meters)"
    echo "    asuqtr-vel                     Linear velocities (m/s)"
    echo "    asuqtr-odom                    Full odometry"
    echo ""
    echo "  IMU / VISUALISATION"
    echo "    asuqtr-imu                     Raw VectorNav IMU (/vectornav/imu)"
    echo "    asuqtr-ekf-orient              EKF orientation quaternion"
    echo "    asuqtr-dashboard               Dashboard 3D sous-marin (1/2/3/4 = NED raw / ENU IMU / EKF ENU / NED LQR, U = NED/ENU toggle)"
    echo ""
    echo "  COMMANDS"
    echo "    asuqtr-target x y z r p y      Send target pose (m / degrees, lqr_tuning mode)"
    echo ""
    echo "  UTILITIES"
    echo "    asuqtr-topics                  List all active topics"
    echo "    asuqtr-hz <topic>              Topic publication frequency"
    echo "    asuqtr-help                    Show this help"
    echo ""
}
