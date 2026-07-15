#!/usr/bin/env python3
"""
Publish a Bar30-compatible depth estimate from DVL altitude.

This is a pool-test fallback for when the Bar30/I2C chain is unreliable. The
EKF already consumes nav_msgs/Odometry on the "depth" topic, so this node keeps
that interface unchanged:

    depth_m = pool_depth_m - dvl_altitude_m
    odom.pose.pose.position.z = -depth_m  # ROS ENU: up is positive
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String


class DvlDepthProxyNode(Node):
    def __init__(self):
        super().__init__('dvl_depth_proxy_node')

        self.pool_depth_m = float(self.declare_parameter('pool_depth_m', 7.0).value)
        self.min_altitude_m = float(self.declare_parameter('min_altitude_m', 0.2).value)
        self.max_altitude_m = float(self.declare_parameter('max_altitude_m', 20.0).value)
        self.depth_variance = float(self.declare_parameter('depth_variance', 0.04).value)
        self.require_bottom_lock = bool(self.declare_parameter('require_bottom_lock', True).value)
        self.publish_source = bool(self.declare_parameter('publish_source', True).value)

        sensor_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.depth_pub = self.create_publisher(Odometry, 'depth', sensor_qos)
        self.source_pub = self.create_publisher(String, 'depth/source', 10)

        self.bottom_lock = False
        self.bottom_lock_seen = False
        self.create_subscription(Bool, 'dvl/bottom_lock', self._bottom_lock_callback, sensor_qos)
        self.create_subscription(PoseStamped, 'dvl/altitude', self._altitude_callback, sensor_qos)

        self.get_logger().warn(
            'DVL depth proxy active: publishing /depth from /dvl/altitude. '
            'Do not run sensor_node.py Bar30 at the same time.'
        )
        self.get_logger().info(
            f'pool_depth_m={self.pool_depth_m:.2f}, require_bottom_lock={self.require_bottom_lock}, '
            f'depth_variance={self.depth_variance:.4f}'
        )

    def _bottom_lock_callback(self, msg):
        self.bottom_lock_seen = True
        self.bottom_lock = bool(msg.data)

    def _altitude_callback(self, msg):
        if self.require_bottom_lock and (not self.bottom_lock_seen or not self.bottom_lock):
            self._publish_source('none')
            return

        altitude = float(msg.pose.position.z)
        if not math.isfinite(altitude):
            self._publish_source('none')
            return
        if altitude < self.min_altitude_m or altitude > self.max_altitude_m:
            self.get_logger().warn(
                f'DVL altitude {altitude:.2f}m outside [{self.min_altitude_m:.2f}, '
                f'{self.max_altitude_m:.2f}]m; skipping depth publish.',
                throttle_duration_sec=1.0,
            )
            self._publish_source('none')
            return

        depth_m = self.pool_depth_m - altitude
        if depth_m < -0.5:
            self.get_logger().warn(
                f'Computed depth is negative ({depth_m:.2f}m). Check pool_depth_m or DVL altitude.',
                throttle_duration_sec=1.0,
            )

        depth_msg = Odometry()
        depth_msg.header.stamp = msg.header.stamp
        depth_msg.header.frame_id = 'odom'
        depth_msg.child_frame_id = 'dvl_depth_proxy_link'
        depth_msg.pose.pose.position.z = -depth_m
        depth_msg.pose.covariance[14] = self.depth_variance
        self.depth_pub.publish(depth_msg)
        self._publish_source('dvl_proxy')

    def _publish_source(self, source):
        if not self.publish_source:
            return
        self.source_pub.publish(String(data=source))


def main(args=None):
    rclpy.init(args=args)
    node = DvlDepthProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Shutting down DVL depth proxy.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
