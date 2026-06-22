#!/usr/bin/env python3
from typing import cast
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sub_interfaces.msg import ControlTarget
from sub_interfaces.srv import PlaybackRecording, PlaybackPath

class PlaybackNode(Node):
    def __init__(self):
        super().__init__('playback_node')
        self.get_logger().info('PlaybackNode has been started.')

        self.waypoint_timing = cast(
            float,
            self.declare_parameter('waypoint_timing', 0.2).value
        )
        self.waypoints: list[dict] = []
        self.last_clock = None
        self.recording_started_at = None
        self.is_recording = False

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
            try:
                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                self.get_logger().info(f'Waypoints dumped to {filename}')
                response.success = True
            except Exception as e:
                self.get_logger().error(f'Failed to write waypoints to {filename}: {e}')
                response.success = False

            self.waypoints.clear()

        return response
    
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
