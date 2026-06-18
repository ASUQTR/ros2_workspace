#!/usr/bin/env python3
from typing import cast
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry

from sub_interfaces.srv import PlaybackRecording, PlaybackPath

class PlaybackNode(Node):
    # Parameters
    waypoint_timing: float = 5.0  # seconds

    # Each waypoint is a dict: {"position": [x,y,z], "angles": [roll,pitch,yaw]}
    waypoints: list[dict] = []
    last_clock = None
    is_playback_running = False

    def __init__(self):
        super().__init__('playback_node')
        self.get_logger().info('PlaybackNode has been started.')

        # Retrieve parameters
        self.waypoint_timing = cast(float, self.declare_parameter('waypoint_timing', self.waypoint_timing).value)

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
        
        self.localization_cb_group = MutuallyExclusiveCallbackGroup()
        self.localization_subscription = self.create_subscription(
            Odometry, 'odometry/filtered', self.localization_callback, only_latest_qos,
            callback_group=self.localization_cb_group
        )

    def handle_playback_recording(self, request, response):
        self.get_logger().info(f'Playback request received: start_recording={request.start_recording}')
        
        if request.start_recording:
            self.start_recording()
            response.success = 1
        else:
            # Stop and dump waypoints to a uniquely named json file based on node clock
            self.stop_recording()

            now_msg = self.get_clock().now().to_msg()
            filename = f'/tmp/waypoints_{now_msg.sec}_{now_msg.nanosec}.json'
            try:
                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=4)
                self.get_logger().info(f'Waypoints dumped to {filename}')
            except Exception as e:
                self.get_logger().error(f'Failed to write waypoints to {filename}: {e}')

            self.waypoints.clear()
            response.success = 0

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
        self.get_logger().info('Starting playback...')
        self.last_clock = self.get_clock().now()
        self.is_playback_running = True

    def stop_recording(self):
        self.get_logger().info('Stopping playback...')
        self.last_clock = None
        self.is_playback_running = False

    def localization_callback(self, msg):
        if not self.is_playback_running:
            return

        if msg is None:
            self.get_logger().warning('Received None message in localization callback')
            return

        current_time = self.get_clock().now()
        if self.last_clock is None:
            time_diff = float('inf')
        else:
            time_diff = (current_time - self.last_clock).nanoseconds / 1e9
            self.get_logger().info(f'Time since last localization update: {time_diff:.2f} seconds')
            self.get_logger().info(f'Current time: {current_time.to_msg().sec}.{current_time.to_msg().nanosec}, Last clock: {self.last_clock.to_msg().sec}.{self.last_clock.to_msg().nanosec}')

        if time_diff < self.waypoint_timing:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation

        waypoint = {
            'position': [x, y, z],
            'angles': [q.x, q.y, q.z, q.w]
        }
        self.waypoints.append(waypoint)
        self.last_clock = current_time

        self.get_logger().info(f'Received localization update: position=({x}, {y}, {z}), angles=(x={q.x:.3f}, y={q.y:.3f}, z={q.z:.3f}, w={q.w:.3f})')


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