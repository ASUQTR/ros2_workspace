#!/usr/bin/env python3

import numpy as np
import math
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros


class LQRTuning(Node):
    def __init__(self):
        super().__init__('lqr_tuning')

        only_latest_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        imu_grp = MutuallyExclusiveCallbackGroup()
        odom_grp = MutuallyExclusiveCallbackGroup()

        self.localization_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.localization_callback,
            only_latest_qos,
            callback_group=odom_grp,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'vectornav/imu',
            self.imu_callback,
            only_latest_qos,
            callback_group=imu_grp,
        )

        self.filtered_state = np.full(12, np.nan, dtype=np.float64)
        self.imu_state = np.full(12, np.nan, dtype=np.float64)

        self.filtered_euler_pub = self.create_publisher(
            Quaternion, 'filtered_ned_euler', only_latest_qos
        )
        self.imu_euler_pub = self.create_publisher(
            Quaternion, 'imu_ned_euler', only_latest_qos,
        )
        self.euler_error_pub = self.create_publisher(
            Quaternion, 'ned_euler_error', only_latest_qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.base_link_frame = 'base_link'
        self._cached_sensor_frame = None
        self._cached_R_sensor_base = None

        # ENU -> NED
        self.M_ned_enu = np.array([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0]
        ])

        # FLU -> FRD
        self.M_frd_flu = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])

    def _get_sensor_to_base_rotation(self, sensor_frame: str):
        """
        Returns rotation from base_link to sensor frame, expressed as R_sensor_base.
        This lets us convert world->sensor orientation into world->base orientation:
            R_world_base = R_world_sensor * R_sensor_base
        """
        if self._cached_sensor_frame == sensor_frame and self._cached_R_sensor_base is not None:
            return self._cached_R_sensor_base

        try:
            # target=sensor_frame, source=base_link gives transform sensor <- base
            tf_msg = self.tf_buffer.lookup_transform(
                sensor_frame,
                self.base_link_frame,
                Time()
            )

            q = tf_msg.transform.rotation
            R_sensor_base = R.from_quat([q.x, q.y, q.z, q.w])

            self._cached_sensor_frame = sensor_frame
            self._cached_R_sensor_base = R_sensor_base
            return R_sensor_base

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(
                f'Cannot get TF {sensor_frame} <- {self.base_link_frame}: {ex}',
                throttle_duration_sec=1.0
            )
            return None

    def _publish_euler_deg(self, pub, roll_rad, pitch_rad, yaw_rad):
        msg = Quaternion()
        msg.x = math.degrees(roll_rad)
        msg.y = math.degrees(pitch_rad)
        msg.z = math.degrees(yaw_rad)
        msg.w = 1.0
        pub.publish(msg)

    def _apply_temp_yaw_offset(self, yaw_rad: float) -> float:
        """Temporary yaw offset to match control_node debug convention."""
        yaw_offset_deg = 58.3
        yaw_offset_rad = math.radians(yaw_offset_deg)
        yaw_corrected = yaw_rad - yaw_offset_rad
        return (yaw_corrected + math.pi) % (2 * math.pi) - math.pi

    def imu_callback(self, msg: Imu):
        quat_ros = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # Orientation of sensor frame in ROS world frame (ENU/FLU convention from driver)
        R_world_sensor = R.from_quat(quat_ros)

        sensor_frame = msg.header.frame_id if msg.header.frame_id else 'vectornav_imu'
        R_sensor_base = self._get_sensor_to_base_rotation(sensor_frame)
        if R_sensor_base is None:
            return

        # Convert sensor orientation into base_link orientation
        R_world_base = R_world_sensor * R_sensor_base

        # Convert ROS ENU/FLU into NED/FRD for debug display
        R_enu_flu = R_world_base.as_matrix()
        R_ned_frd_mat = self.M_ned_enu @ R_enu_flu @ self.M_frd_flu
        self.R_imu_ned = R.from_matrix(R_ned_frd_mat)

        yaw_ned, pitch_ned, roll_ned = self.R_imu_ned.as_euler('zyx', degrees=False)
        yaw_ned = self._apply_temp_yaw_offset(yaw_ned)

        self.imu_state[3] = roll_ned
        self.imu_state[4] = pitch_ned
        self.imu_state[5] = yaw_ned

        self._publish_euler_deg(self.imu_euler_pub, roll_ned, pitch_ned, yaw_ned)

    def localization_callback(self, msg: Odometry):
        quat_ros = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        R_enu_flu = R.from_quat(quat_ros).as_matrix()
        R_ned_frd_mat = self.M_ned_enu @ R_enu_flu @ self.M_frd_flu
        self.R_filtered_ned = R.from_matrix(R_ned_frd_mat)

        yaw_ned, pitch_ned, roll_ned = self.R_filtered_ned.as_euler('zyx', degrees=False)
        yaw_ned = self._apply_temp_yaw_offset(yaw_ned)

        self.filtered_state[3] = roll_ned
        self.filtered_state[4] = pitch_ned
        self.filtered_state[5] = yaw_ned

        self._publish_euler_deg(self.filtered_euler_pub, roll_ned, pitch_ned, yaw_ned)

        if hasattr(self, 'R_imu_ned') and hasattr(self, 'R_filtered_ned'):
            R_error = self.R_filtered_ned.inv() * self.R_imu_ned
            err_yaw, err_pitch, err_roll = R_error.as_euler('zyx', degrees=True)

            euler_error_msg = Quaternion()
            euler_error_msg.x = err_roll
            euler_error_msg.y = err_pitch
            euler_error_msg.z = err_yaw
            euler_error_msg.w = 1.0
            self.euler_error_pub.publish(euler_error_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LQRTuning()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected, shutting down...')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()