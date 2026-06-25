#!/usr/bin/env python3
from typing import cast
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sub_interfaces.msg import ControlTarget
from sub_interfaces.srv import PlaybackCommand, PlaybackRecording, PlaybackPath

class PlaybackNode(Node):
    def __init__(self):
        super().__init__('playback_node')
        self.get_logger().info('PlaybackNode has been started.')

        self.waypoint_timing = cast(
            float,
            self.declare_parameter('waypoint_timing', 0.2).value
        )
        self.position_tolerance = float(
            self.declare_parameter('position_tolerance', 0.15).value
        )
        self.depth_tolerance = float(
            self.declare_parameter('depth_tolerance', 0.10).value
        )
        self.angle_tolerance = math.radians(float(
            self.declare_parameter('angle_tolerance_deg', 10.0).value
        ))
        self.waypoint_timeout = float(
            self.declare_parameter('waypoint_timeout_sec', 8.0).value
        )
        self.min_waypoint_hold = float(
            self.declare_parameter('min_waypoint_hold_sec', 0.2).value
        )
        self.target_republish_period = float(
            self.declare_parameter('target_republish_period_sec', 0.2).value
        )
        self.waypoints: list[dict] = []
        self.last_clock = None
        self.recording_started_at = None
        self.is_recording = False
        self.loaded_waypoints: list[dict] = []
        self.loaded_file_path = ''
        self.is_playing = False
        self.playback_loop = False
        self.playback_started_at = None
        self.active_waypoint_started_at = None
        self.last_target_publish_at = None
        self.playback_index = 0
        self.current_odometry = None
        self.control_stopped = True

        self.playback_service = self.create_service(
            PlaybackRecording,
            'playback_recording',
            self.handle_playback_recording
        )
        self.playback_path_service = self.create_service(
            PlaybackPath,
            'playback_path',
            self.handle_playback_path
        )
        self.playback_command_service = self.create_service(
            PlaybackCommand,
            'playback_command',
            self.handle_playback_command
        )

        only_latest_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.target_cb_group = MutuallyExclusiveCallbackGroup()
        self.target_subscription = self.create_subscription(
            ControlTarget, 'control/target', self.target_callback, only_latest_qos,
            callback_group=self.target_cb_group
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, 'odometry/filtered', self.odometry_callback, only_latest_qos,
            callback_group=self.target_cb_group
        )
        self.playback_target_pub = self.create_publisher(
            ControlTarget, 'control/playback_target', only_latest_qos
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.control_stop_subscription = self.create_subscription(
            Bool, 'disable_pwm', self.control_stop_callback, latched_qos
        )
        self.playback_timer = self.create_timer(0.05, self.playback_timer_callback)

    def handle_playback_recording(self, request, response):
        self.get_logger().info(
            f'Recording request received: start_recording={request.start_recording}'
        )
        
        if request.start_recording:
            self.start_recording()
            response.success = True
        else:
            self.stop_recording()

            now_msg = self.get_clock().now().to_msg()
            filename = f'/tmp/waypoints_{now_msg.sec}_{now_msg.nanosec}.json'
            latest_filename = '/tmp/waypoints_latest.json'
            try:
                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                with open(latest_filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                self.get_logger().info(f'Waypoints dumped to {filename} and {latest_filename}')
                response.success = True
            except Exception as e:
                self.get_logger().error(f'Failed to write waypoints to {filename}: {e}')
                response.success = False

            self.waypoints.clear()

        return response

    def handle_playback_command(self, request, response):
        command = request.command.strip().lower()
        if command == 'load':
            success, message = self.load_playback_file(request.file_path)
        elif command == 'play':
            request_file_path = request.file_path.strip()
            if request_file_path and request_file_path.lower() != 'none':
                success, message = self.load_playback_file(request.file_path)
                if not success:
                    response.success = False
                    response.message = message
                    response.waypoint_count = len(self.loaded_waypoints)
                    return response
            success, message = self.start_playback(loop=request.loop)
        elif command == 'stop':
            self.stop_playback()
            success = True
            message = 'Playback stopped'
        else:
            success = False
            message = f'Unknown playback command: {request.command}'

        response.success = success
        response.message = message
        response.waypoint_count = len(self.loaded_waypoints)
        return response

    def load_playback_file(self, file_path):
        clean_path = file_path.strip()
        if not clean_path:
            return False, 'No JSON file path provided'

        try:
            with open(clean_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            return False, f'Failed to read {clean_path}: {e}'

        if not isinstance(data, list):
            return False, 'Playback JSON must contain a list of waypoints'

        waypoints = []
        for idx, item in enumerate(data):
            try:
                waypoints.append(self.normalize_waypoint(item, idx))
            except Exception as e:
                return False, f'Invalid waypoint {idx}: {e}'

        self.loaded_waypoints = waypoints
        self.loaded_file_path = clean_path
        self.playback_index = 0
        self.stop_playback()
        message = f'Loaded {len(self.loaded_waypoints)} waypoint(s) from {clean_path}'
        self.get_logger().info(message)
        return True, message

    def start_playback(self, loop=False):
        if not self.loaded_waypoints:
            return False, 'No playback file loaded'
        if self.current_odometry is None:
            return False, 'Cannot start playback: no odometry available'
        if self.control_stopped:
            return False, 'Cannot start playback: MANUAL_ASSISTED is stopped; press Start first'

        self.playback_loop = bool(loop)
        self.playback_started_at = self.get_clock().now()
        self.playback_index = 0
        self.is_playing = True
        self.activate_current_waypoint()
        message = (
            f'Started arrival-gated playback with '
            f'{len(self.loaded_waypoints)} waypoint(s)'
        )
        self.get_logger().info(message)
        return True, message

    def stop_playback(self):
        self.is_playing = False
        self.playback_started_at = None
        self.active_waypoint_started_at = None
        self.last_target_publish_at = None
        self.playback_index = 0

    def odometry_callback(self, msg):
        self.current_odometry = msg

    def control_stop_callback(self, msg):
        self.control_stopped = bool(msg.data)
        if self.control_stopped and self.is_playing:
            self.stop_playback()
            self.get_logger().warn('Playback stopped because MANUAL_ASSISTED was stopped')

    def normalize_waypoint(self, item, idx):
        if isinstance(item, dict):
            pos = item.get('position')
            angles = item.get('angles', [0.0, 0.0, 0.0, 1.0])
            linear = item.get('linear_velocity', [0.0, 0.0, 0.0])
            angular = item.get('angular_velocity', [0.0, 0.0, 0.0])
            carrot = item.get('carrot_body', [0.0, 0.0])
            time_from_start = float(item.get('time_from_start', idx * self.waypoint_timing))
            frame_id = str(item.get('frame_id') or 'odom')
        elif isinstance(item, (list, tuple)) and len(item) >= 3:
            pos = [item[0], item[1], item[2]]
            angles = [0.0, 0.0, 0.0, 1.0]
            linear = [0.0, 0.0, 0.0]
            angular = [0.0, 0.0, 0.0]
            carrot = [0.0, 0.0]
            time_from_start = idx * self.waypoint_timing
            frame_id = 'odom'
        else:
            raise ValueError('unsupported format')

        if pos is None or len(pos) < 3:
            raise ValueError('missing position[3]')
        if len(angles) < 4:
            angles = [0.0, 0.0, 0.0, 1.0]
        if len(linear) < 3:
            linear = [0.0, 0.0, 0.0]
        if len(angular) < 3:
            angular = [0.0, 0.0, 0.0]
        if len(carrot) < 2:
            carrot = [0.0, 0.0]

        return {
            'time_from_start': time_from_start,
            'frame_id': frame_id,
            'position': [float(pos[0]), float(pos[1]), float(pos[2])],
            'angles': [float(angles[0]), float(angles[1]), float(angles[2]), float(angles[3])],
            'linear_velocity': [float(linear[0]), float(linear[1]), float(linear[2])],
            'angular_velocity': [float(angular[0]), float(angular[1]), float(angular[2])],
            'carrot_body': [float(carrot[0]), float(carrot[1])]
        }

    def playback_timer_callback(self):
        if not self.is_playing or self.active_waypoint_started_at is None:
            return
        if not self.loaded_waypoints:
            self.stop_playback()
            return
        if self.control_stopped:
            self.stop_playback()
            return
        if self.current_odometry is None:
            return

        now = self.get_clock().now()
        active_elapsed = (now - self.active_waypoint_started_at).nanoseconds / 1e9
        since_publish = float('inf')
        if self.last_target_publish_at is not None:
            since_publish = (now - self.last_target_publish_at).nanoseconds / 1e9

        waypoint = self.loaded_waypoints[self.playback_index]
        if since_publish >= self.target_republish_period:
            self.publish_playback_target(waypoint)
            self.last_target_publish_at = now

        if active_elapsed >= self.min_waypoint_hold and self.is_waypoint_reached(waypoint):
            self.advance_waypoint()
            return

        if active_elapsed >= self.waypoint_timeout:
            failed_index = self.playback_index
            self.stop_playback()
            self.get_logger().error(
                f'Playback stopped: waypoint {failed_index} timed out after '
                f'{self.waypoint_timeout:.1f} s'
            )

    def activate_current_waypoint(self):
        if not self.is_playing or self.playback_index >= len(self.loaded_waypoints):
            return
        now = self.get_clock().now()
        self.active_waypoint_started_at = now
        self.last_target_publish_at = now
        self.publish_playback_target(self.loaded_waypoints[self.playback_index])
        self.get_logger().info(
            f'Playback target {self.playback_index + 1}/{len(self.loaded_waypoints)}'
        )

    def advance_waypoint(self):
        self.playback_index += 1
        if self.playback_index < len(self.loaded_waypoints):
            self.activate_current_waypoint()
            return

        if self.playback_loop:
            self.playback_index = 0
            self.playback_started_at = self.get_clock().now()
            self.activate_current_waypoint()
            return

        self.stop_playback()
        self.get_logger().info('Playback completed')

    def is_waypoint_reached(self, waypoint):
        pose = self.current_odometry.pose.pose
        dx = waypoint['position'][0] - pose.position.x
        dy = waypoint['position'][1] - pose.position.y
        dz = waypoint['position'][2] - pose.position.z
        horizontal_error = math.hypot(dx, dy)
        depth_error = abs(dz)
        angle_error = self.quaternion_angular_distance(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ],
            waypoint['angles']
        )
        return (
            horizontal_error <= self.position_tolerance
            and depth_error <= self.depth_tolerance
            and angle_error <= self.angle_tolerance
        )

    @staticmethod
    def quaternion_angular_distance(first, second):
        first_norm = math.sqrt(sum(value * value for value in first))
        second_norm = math.sqrt(sum(value * value for value in second))
        if first_norm <= 1e-9 or second_norm <= 1e-9:
            return math.inf
        dot = sum(a * b for a, b in zip(first, second)) / (first_norm * second_norm)
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    def publish_playback_target(self, waypoint):
        msg = ControlTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = waypoint['frame_id']
        msg.source_mode = 'playback_manual_assisted'
        msg.pose.position.x = waypoint['position'][0]
        msg.pose.position.y = waypoint['position'][1]
        msg.pose.position.z = waypoint['position'][2]
        msg.pose.orientation.x = waypoint['angles'][0]
        msg.pose.orientation.y = waypoint['angles'][1]
        msg.pose.orientation.z = waypoint['angles'][2]
        msg.pose.orientation.w = waypoint['angles'][3]
        msg.twist.linear.x = waypoint['linear_velocity'][0]
        msg.twist.linear.y = waypoint['linear_velocity'][1]
        msg.twist.linear.z = waypoint['linear_velocity'][2]
        msg.twist.angular.x = waypoint['angular_velocity'][0]
        msg.twist.angular.y = waypoint['angular_velocity'][1]
        msg.twist.angular.z = waypoint['angular_velocity'][2]
        msg.carrot_forward = waypoint['carrot_body'][0]
        msg.carrot_right = waypoint['carrot_body'][1]
        self.playback_target_pub.publish(msg)
    
    def handle_playback_path(self, request, response):
        # Use the provided file path in the request to load stored waypoints
        try:
            with open(request.file_path, 'r') as f:
                waypoints = json.load(f)

            idx = int(request.idx)
            if idx < 0 or idx >= len(waypoints):
                raise IndexError('Requested index out of range')

            wp = waypoints[idx]
            # wp may be dict with position or older tuple format
            if isinstance(wp, dict) and 'position' in wp:
                pos = wp['position']
                ang = wp.get('angles', [0.0, 0.0, 0.0, 0.0])
            elif isinstance(wp, (list, tuple)) and len(wp) >= 3:
                pos = [wp[0], wp[1], wp[2]]
                ang = [0.0, 0.0, 0.0, 0.0]
            else:
                raise ValueError('Waypoint has unsupported format')

            response.position = [float(pos[0]), float(pos[1]), float(pos[2])]
            response.angles = [float(ang[0]), float(ang[1]), float(ang[2]), float(ang[3])] if len(ang) == 4 else [0.0, 0.0, 0.0, 0.0]
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from {request.file_path}: {e}')
            response.position = [0.0, 0.0, 0.0]
            response.angles = [0.0, 0.0, 0.0, 0.0]
        return response
    
    def start_recording(self):
        self.get_logger().info('Starting waypoint recording...')
        self.waypoints.clear()
        self.last_clock = self.get_clock().now()
        self.recording_started_at = self.last_clock
        self.is_recording = True

    def stop_recording(self):
        self.get_logger().info('Stopping waypoint recording...')
        self.last_clock = None
        self.recording_started_at = None
        self.is_recording = False

    def target_callback(self, msg):
        if not self.is_recording:
            return

        if msg is None:
            self.get_logger().warning('Received None message in target callback')
            return
        if msg.source_mode != 'manual_assisted':
            return

        current_time = self.get_clock().now()
        if self.last_clock is None:
            time_diff = float('inf')
        else:
            time_diff = (current_time - self.last_clock).nanoseconds / 1e9

        if time_diff < self.waypoint_timing:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        q = msg.pose.orientation
        linear = msg.twist.linear
        angular = msg.twist.angular
        elapsed = 0.0
        if self.recording_started_at is not None:
            elapsed = (current_time - self.recording_started_at).nanoseconds / 1e9

        waypoint = {
            'time_from_start': elapsed,
            'frame_id': msg.header.frame_id,
            'source_mode': msg.source_mode,
            'position': [x, y, z],
            'angles': [q.x, q.y, q.z, q.w],
            'linear_velocity': [linear.x, linear.y, linear.z],
            'angular_velocity': [angular.x, angular.y, angular.z],
            'carrot_body': [msg.carrot_forward, msg.carrot_right]
        }
        self.waypoints.append(waypoint)
        self.last_clock = current_time

        self.get_logger().info(
            f'Recorded waypoint {len(self.waypoints)}: '
            f'position=({x:.3f}, {y:.3f}, {z:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    playback_node = PlaybackNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(playback_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        playback_node.get_logger().info('Keyboard interrupt detected, shutting down...')
    finally:
        executor.shutdown()
        playback_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
