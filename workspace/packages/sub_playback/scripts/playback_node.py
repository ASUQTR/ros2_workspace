#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry

from sub_interfaces.srv import PlaybackRecording, PlaybackPath

class PlaybackNode(Node):
    # Parameters
    waypoint_timing = 5.0  # seconds

    waypoints: list[tuple[float, float, float]] = []
    last_clock = None
    is_playback_running = False

    def __init__(self):
        super().__init__('playback_node')
        self.get_logger().info('PlaybackNode has been started.')

        self.playback_service = self.create_service(
            PlaybackRecording,
            'playback_recording',
            self.handle_playback_recording
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
            self.start_playback()
            response.success = 1
        else:
            self.stop_playback()
            response.success = 0

            # Dump waypoints in a json file
            import json
            with open('/tmp/waypoints.json', 'w') as f:
                json.dump(self.waypoints, f, indent=4)
            self.waypoints.clear()

        return response
    
    def start_playback(self):
        self.get_logger().info('Starting playback...')
        self.last_clock = self.get_clock().now()
        self.is_playback_running = True

    def stop_playback(self):
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
        if self.last_clock is not None:
            time_diff = (current_time - self.last_clock).nanoseconds / 1e9
            self.get_logger().info(f'Time since last localization update: {time_diff:.2f} seconds')
            self.get_logger().info(f'Current time: {current_time.to_msg().sec}.{current_time.to_msg().nanosec}, Last clock: {self.last_clock.to_msg().sec}.{self.last_clock.to_msg().nanosec}')
        
        if time_diff < 5.0:
            return

        self.waypoints.append((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        self.last_clock = current_time

        self.get_logger().info(f'Received localization update: position=({msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z})')

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