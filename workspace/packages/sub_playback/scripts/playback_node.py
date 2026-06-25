#!/usr/bin/env python3
import json
import math
from typing import cast

import rclpy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool

from sub_interfaces.msg import ControlTarget, PlaybackStatus
from sub_interfaces.srv import PlaybackCommand, PlaybackPath, PlaybackRecording


class PlaybackNode(Node):
    FORMAT_VERSION = 2
    VALID_REFERENCE_MODES = {'world', 'body'}
    VALID_PROGRESSION_MODES = {'setpoint', 'timestamp'}

    def __init__(self):
        super().__init__('playback_node')

        self.record_sample_period = cast(
            float, self.declare_parameter('waypoint_timing', 0.2).value
        )
        self.keyframe_translation = float(
            self.declare_parameter('keyframe_translation_m', 0.25).value
        )
        self.keyframe_yaw = math.radians(float(
            self.declare_parameter('keyframe_yaw_deg', 10.0).value
        ))
        self.keyframe_depth = float(
            self.declare_parameter('keyframe_depth_m', 0.10).value
        )
        self.keyframe_max_interval = float(
            self.declare_parameter('keyframe_max_interval_sec', 2.0).value
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
        self.odometry_timeout = float(
            self.declare_parameter('odometry_timeout_sec', 0.75).value
        )
        self.magnetic_play_enabled = bool(
            self.declare_parameter('magnetic_play_enabled', True).value
        )
        self.require_kill_switch_armed = bool(
            self.declare_parameter('require_kill_switch_armed', True).value
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.recorded_waypoints = []
        self.loaded_waypoints = []
        self.loaded_file_path = ''
        self.latest_control_target = None
        self.current_odometry = None
        self.last_odometry_at = None
        self.previous_keyframe = None
        self.recording_started_at = None
        self.last_record_sample_at = None
        self.is_recording = False

        self.is_playing = False
        self.playback_loop = False
        self.playback_started_at = None
        self.active_waypoint_started_at = None
        self.last_target_publish_at = None
        self.playback_index = 0
        self.resolved_target = None
        self.reference_mode = 'body'
        self.progression_mode = 'setpoint'
        self.control_stopped = True
        self.kill_switch_armed = not self.require_kill_switch_armed
        self.last_magnetic_switch_state = False
        self.last_error = ''
        self.last_timed_out = False

        self.recording_service = self.create_service(
            PlaybackRecording, 'playback_recording', self.handle_playback_recording
        )
        self.playback_path_service = self.create_service(
            PlaybackPath, 'playback_path', self.handle_playback_path
        )
        self.command_service = self.create_service(
            PlaybackCommand, 'playback_command', self.handle_playback_command
        )

        sensor_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.record_cb_group = MutuallyExclusiveCallbackGroup()
        self.target_subscription = self.create_subscription(
            ControlTarget,
            'control/target',
            self.target_callback,
            sensor_qos,
            callback_group=self.record_cb_group
        )
        self.odometry_subscription = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odometry_callback,
            sensor_qos,
            callback_group=self.record_cb_group
        )
        self.control_stop_subscription = self.create_subscription(
            Bool, 'disable_pwm', self.control_stop_callback, latched_qos
        )
        self.kill_switch_subscription = self.create_subscription(
            Bool, 'kill_switch', self.kill_switch_callback, latched_qos
        )
        self.magnetic_switch_subscription = self.create_subscription(
            Bool, 'magnetic_switch_2', self.magnetic_switch_callback, latched_qos
        )
        self.playback_target_pub = self.create_publisher(
            ControlTarget, 'control/playback_target', sensor_qos
        )
        self.status_pub = self.create_publisher(
            PlaybackStatus, 'playback/status', latched_qos
        )
        self.playback_timer = self.create_timer(0.05, self.playback_timer_callback)
        self.publish_status('idle')
        self.get_logger().info('PlaybackNode started in body + setpoint mode.')

    def parameter_callback(self, params):
        updates = {}
        for param in params:
            if param.name in (
                'position_tolerance',
                'depth_tolerance',
                'angle_tolerance_deg',
                'waypoint_timeout_sec',
                'min_waypoint_hold_sec',
                'target_republish_period_sec',
                'odometry_timeout_sec'
            ):
                if param.type_ != Parameter.Type.DOUBLE or float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} must be a positive float'
                    )
                updates[param.name] = float(param.value)
            elif param.name == 'magnetic_play_enabled':
                updates[param.name] = bool(param.value)
            elif param.name == 'require_kill_switch_armed':
                updates[param.name] = bool(param.value)
            else:
                return SetParametersResult(
                    successful=False, reason=f'Unsupported parameter: {param.name}'
                )

        for name, value in updates.items():
            if name == 'position_tolerance':
                self.position_tolerance = value
            elif name == 'depth_tolerance':
                self.depth_tolerance = value
            elif name == 'angle_tolerance_deg':
                self.angle_tolerance = math.radians(value)
            elif name == 'waypoint_timeout_sec':
                self.waypoint_timeout = value
            elif name == 'min_waypoint_hold_sec':
                self.min_waypoint_hold = value
            elif name == 'target_republish_period_sec':
                self.target_republish_period = value
            elif name == 'odometry_timeout_sec':
                self.odometry_timeout = value
            elif name == 'magnetic_play_enabled':
                self.magnetic_play_enabled = value
            elif name == 'require_kill_switch_armed':
                self.require_kill_switch_armed = value
                if not value:
                    self.kill_switch_armed = True
        return SetParametersResult(successful=True)

    # Recording
    def handle_playback_recording(self, request, response):
        if request.start_recording:
            if self.is_playing:
                response.success = False
                return response
            response.success = self.start_recording()
            return response

        response.success = self.stop_recording_and_save()
        return response

    def start_recording(self):
        if self.control_stopped:
            self.last_error = 'Cannot record: press Start first'
            self.publish_status('error')
            return False
        if self.require_kill_switch_armed and not self.kill_switch_armed:
            self.last_error = 'Cannot record: physical kill switch is not armed'
            self.publish_status('error')
            return False
        if not self.odometry_is_fresh():
            self.last_error = 'Cannot record: odometry is unavailable or stale'
            self.publish_status('error')
            return False
        self.recorded_waypoints = []
        self.previous_keyframe = None
        self.recording_started_at = self.get_clock().now()
        self.last_record_sample_at = None
        self.is_recording = True
        self.last_error = ''
        self.publish_status('recording')
        self.get_logger().info('Recording odometry keyframes.')
        return True

    def stop_recording_and_save(self):
        if not self.is_recording:
            self.last_error = 'Recording was not active'
            self.publish_status('error')
            return False

        if self.current_odometry is not None:
            self.maybe_record_keyframe(self.current_odometry, force=True)
        self.is_recording = False
        now_msg = self.get_clock().now().to_msg()
        filename = f'/tmp/waypoints_{now_msg.sec}_{now_msg.nanosec}.json'
        latest_filename = '/tmp/waypoints_latest.json'
        payload = {
            'format_version': self.FORMAT_VERSION,
            'reference_frame': 'odom',
            'record_source': 'odometry/filtered',
            'keyframe_thresholds': {
                'translation_m': self.keyframe_translation,
                'yaw_deg': math.degrees(self.keyframe_yaw),
                'depth_m': self.keyframe_depth,
                'max_interval_sec': self.keyframe_max_interval
            },
            'waypoints': self.recorded_waypoints
        }
        try:
            for output_path in (filename, latest_filename):
                with open(output_path, 'w') as output:
                    json.dump(payload, output, indent=2)
        except Exception as exc:
            self.last_error = f'Failed to save recording: {exc}'
            self.publish_status('error')
            self.get_logger().error(self.last_error)
            return False

        self.loaded_waypoints = [
            self.normalize_waypoint(item, idx, self.recorded_waypoints)
            for idx, item in enumerate(self.recorded_waypoints)
        ]
        self.loaded_file_path = latest_filename
        self.publish_status('loaded')
        self.get_logger().info(
            f'Saved {len(self.recorded_waypoints)} keyframe(s) to {latest_filename}'
        )
        return True

    def target_callback(self, msg):
        if msg.source_mode == 'manual_assisted':
            self.latest_control_target = msg

    def odometry_callback(self, msg):
        self.current_odometry = msg
        self.last_odometry_at = self.get_clock().now()
        if self.is_recording:
            self.maybe_record_keyframe(msg)

    def maybe_record_keyframe(self, odometry, force=False):
        now = self.get_clock().now()
        if self.last_record_sample_at is not None:
            sample_age = (now - self.last_record_sample_at).nanoseconds / 1e9
            if not force and sample_age < self.record_sample_period:
                return
        self.last_record_sample_at = now

        candidate = self.odometry_to_keyframe(odometry, now)
        if self.previous_keyframe is not None and not force:
            if not self.is_significant_keyframe(self.previous_keyframe, candidate):
                return
        if self.previous_keyframe is not None and self.same_pose(
            self.previous_keyframe, candidate
        ):
            return

        candidate['body_delta'] = self.calculate_body_delta(
            self.previous_keyframe, candidate
        )
        self.recorded_waypoints.append(candidate)
        self.previous_keyframe = candidate
        self.publish_status('recording')

    def odometry_to_keyframe(self, odometry, now):
        pose = odometry.pose.pose
        roll, pitch, yaw = self.quaternion_to_euler(pose.orientation)
        elapsed = 0.0
        if self.recording_started_at is not None:
            elapsed = (now - self.recording_started_at).nanoseconds / 1e9
        carrot = [0.0, 0.0]
        if self.latest_control_target is not None:
            carrot = [
                float(self.latest_control_target.carrot_forward),
                float(self.latest_control_target.carrot_right)
            ]
        return {
            'time_from_start': elapsed,
            'world_pose': {
                'position': [
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z)
                ],
                'orientation': [
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w)
                ]
            },
            'body_delta': {'forward': 0.0, 'right': 0.0, 'yaw': 0.0},
            'depth_target': float(-pose.position.z),
            'roll': float(roll),
            'pitch': float(pitch),
            'yaw': float(yaw),
            'carrot_body': carrot
        }

    def is_significant_keyframe(self, previous, candidate):
        dx = candidate['world_pose']['position'][0] - previous['world_pose']['position'][0]
        dy = candidate['world_pose']['position'][1] - previous['world_pose']['position'][1]
        translation = math.hypot(dx, dy)
        depth = abs(candidate['depth_target'] - previous['depth_target'])
        yaw = abs(self.wrap_angle(candidate['yaw'] - previous['yaw']))
        elapsed = candidate['time_from_start'] - previous['time_from_start']
        return (
            translation >= self.keyframe_translation
            or depth >= self.keyframe_depth
            or yaw >= self.keyframe_yaw
            or elapsed >= self.keyframe_max_interval
        )

    @staticmethod
    def same_pose(first, second):
        return (
            first['world_pose']['position'] == second['world_pose']['position']
            and first['world_pose']['orientation'] == second['world_pose']['orientation']
        )

    def calculate_body_delta(self, previous, current):
        if previous is None:
            return {'forward': 0.0, 'right': 0.0, 'yaw': 0.0}
        dx = current['world_pose']['position'][0] - previous['world_pose']['position'][0]
        dy = current['world_pose']['position'][1] - previous['world_pose']['position'][1]
        previous_yaw = previous['yaw']
        return {
            'forward': math.cos(previous_yaw) * dx + math.sin(previous_yaw) * dy,
            'right': math.sin(previous_yaw) * dx - math.cos(previous_yaw) * dy,
            'yaw': self.wrap_angle(current['yaw'] - previous_yaw)
        }

    # Commands and file loading
    def handle_playback_command(self, request, response):
        command = request.command.strip().lower()
        reference_mode = request.reference_mode.strip().lower() or self.reference_mode
        progression_mode = request.progression_mode.strip().lower() or self.progression_mode
        if reference_mode not in self.VALID_REFERENCE_MODES:
            return self.command_response(
                response, False, f'Invalid reference mode: {reference_mode}'
            )
        if progression_mode not in self.VALID_PROGRESSION_MODES:
            return self.command_response(
                response, False, f'Invalid progression mode: {progression_mode}'
            )
        if self.is_playing or self.is_recording:
            if (
                reference_mode != self.reference_mode
                or progression_mode != self.progression_mode
            ):
                return self.command_response(
                    response, False, 'Playback modes are locked while active'
                )
        else:
            self.reference_mode = reference_mode
            self.progression_mode = progression_mode

        if command == 'load':
            success, message = self.load_playback_file(request.file_path)
        elif command == 'play':
            requested_path = request.file_path.strip()
            if requested_path and requested_path.lower() != 'none':
                success, message = self.load_playback_file(requested_path)
                if not success:
                    return self.command_response(response, success, message)
            success, message = self.start_playback(request.loop)
        elif command == 'stop':
            self.stop_playback('stopped')
            success, message = True, 'Playback stopped'
        else:
            success, message = False, f'Unknown playback command: {request.command}'
        return self.command_response(response, success, message)

    def command_response(self, response, success, message):
        response.success = success
        response.message = message
        response.waypoint_count = len(self.loaded_waypoints)
        return response

    def load_playback_file(self, file_path):
        clean_path = file_path.strip()
        if not clean_path:
            return False, 'No JSON file path provided'
        try:
            with open(clean_path, 'r') as source:
                payload = json.load(source)
        except Exception as exc:
            return False, f'Failed to read {clean_path}: {exc}'

        raw_waypoints = payload.get('waypoints') if isinstance(payload, dict) else payload
        if not isinstance(raw_waypoints, list):
            return False, 'Playback JSON must contain a waypoint list'
        try:
            self.loaded_waypoints = [
                self.normalize_waypoint(item, idx, raw_waypoints)
                for idx, item in enumerate(raw_waypoints)
            ]
        except Exception as exc:
            self.loaded_waypoints = []
            return False, f'Invalid playback waypoint: {exc}'

        self.loaded_file_path = clean_path
        self.stop_playback('loaded')
        message = f'Loaded {len(self.loaded_waypoints)} waypoint(s) from {clean_path}'
        self.get_logger().info(message)
        return True, message

    def normalize_waypoint(self, item, idx, all_items):
        if not isinstance(item, dict):
            if not isinstance(item, (list, tuple)) or len(item) < 3:
                raise ValueError(f'unsupported waypoint {idx}')
            item = {'position': list(item[:3])}

        if 'world_pose' in item:
            world_position = item['world_pose']['position']
            world_orientation = item['world_pose']['orientation']
            roll = float(item.get('roll', 0.0))
            pitch = float(item.get('pitch', 0.0))
            yaw = float(item.get('yaw', self.quaternion_to_euler_values(world_orientation)[2]))
            body = item.get('body_delta', {})
            body_forward = float(body.get('forward', 0.0))
            body_right = float(body.get('right', 0.0))
            body_yaw = float(body.get('yaw', 0.0))
            depth_target = float(item.get('depth_target', -world_position[2]))
        else:
            world_position = item.get('position')
            world_orientation = item.get('angles', [0.0, 0.0, 0.0, 1.0])
            if world_position is None:
                raise ValueError(f'missing world position at {idx}')
            roll, pitch, yaw = self.quaternion_to_euler_values(world_orientation)
            body_forward, body_right, body_yaw = self.derive_legacy_body_delta(
                item, idx, all_items, yaw
            )
            depth_target = float(-world_position[2])

        return {
            'time_from_start': float(
                item.get('time_from_start', idx * self.record_sample_period)
            ),
            'world_position': [float(value) for value in world_position[:3]],
            'world_orientation': [float(value) for value in world_orientation[:4]],
            'body_forward': body_forward,
            'body_right': body_right,
            'body_yaw': body_yaw,
            'depth_target': depth_target,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'carrot_body': [
                float(value) for value in item.get('carrot_body', [0.0, 0.0])[:2]
            ]
        }

    def derive_legacy_body_delta(self, item, idx, all_items, current_yaw):
        if idx == 0:
            return 0.0, 0.0, 0.0
        previous = all_items[idx - 1]
        previous_position = previous.get('position')
        previous_orientation = previous.get('angles', [0.0, 0.0, 0.0, 1.0])
        current_position = item.get('position')
        if previous_position is None or current_position is None:
            return 0.0, 0.0, 0.0
        previous_yaw = self.quaternion_to_euler_values(previous_orientation)[2]
        dx = float(current_position[0]) - float(previous_position[0])
        dy = float(current_position[1]) - float(previous_position[1])
        return (
            math.cos(previous_yaw) * dx + math.sin(previous_yaw) * dy,
            math.sin(previous_yaw) * dx - math.cos(previous_yaw) * dy,
            self.wrap_angle(current_yaw - previous_yaw)
        )

    # Playback
    def start_playback(self, loop=False):
        if not self.loaded_waypoints:
            return False, 'No playback file loaded'
        if self.require_kill_switch_armed and not self.kill_switch_armed:
            return False, 'Cannot start playback: physical kill switch is not armed'
        if not self.odometry_is_fresh():
            return False, 'Cannot start playback: odometry is unavailable or stale'
        if self.control_stopped:
            return False, 'Cannot start playback: press Start first'
        self.playback_loop = bool(loop)
        self.playback_started_at = self.get_clock().now()
        self.playback_index = 0
        self.is_playing = True
        self.last_error = ''
        self.last_timed_out = False
        self.activate_current_waypoint()
        self.publish_status('playing')
        return True, (
            f'Started {self.reference_mode} + {self.progression_mode} playback '
            f'with {len(self.loaded_waypoints)} waypoint(s)'
        )

    def stop_playback(self, state='idle', error='', timed_out=False):
        self.is_playing = False
        self.playback_started_at = None
        self.active_waypoint_started_at = None
        self.last_target_publish_at = None
        self.resolved_target = None
        self.last_error = error
        self.last_timed_out = timed_out
        self.publish_status(state)

    def playback_timer_callback(self):
        if not self.is_playing:
            return
        if self.control_stopped:
            self.stop_playback('stopped', 'MANUAL_ASSISTED was stopped')
            return
        if not self.odometry_is_fresh():
            self.stop_playback('error', 'Odometry became stale')
            return
        if self.progression_mode == 'timestamp':
            self.timestamp_progression()
        else:
            self.setpoint_progression()

    def timestamp_progression(self):
        elapsed = (self.get_clock().now() - self.playback_started_at).nanoseconds / 1e9
        while (
            self.playback_index + 1 < len(self.loaded_waypoints)
            and elapsed >= self.loaded_waypoints[self.playback_index + 1]['time_from_start']
        ):
            self.playback_index += 1
            self.activate_current_waypoint()
        if (
            self.playback_index == len(self.loaded_waypoints) - 1
            and elapsed >= self.loaded_waypoints[-1]['time_from_start']
        ):
            final_age = (self.get_clock().now() - self.active_waypoint_started_at).nanoseconds / 1e9
            if final_age >= self.min_waypoint_hold:
                self.complete_or_loop()
                return
        self.republish_active_target()

    def setpoint_progression(self):
        now = self.get_clock().now()
        active_age = (now - self.active_waypoint_started_at).nanoseconds / 1e9
        self.republish_active_target()
        if active_age >= self.min_waypoint_hold and self.is_target_reached():
            self.playback_index += 1
            if self.playback_index >= len(self.loaded_waypoints):
                self.complete_or_loop()
            else:
                self.activate_current_waypoint()
            return
        if active_age >= self.waypoint_timeout:
            error = (
                f'Waypoint {self.playback_index} timed out after '
                f'{self.waypoint_timeout:.1f} s'
            )
            self.get_logger().error(error)
            self.stop_playback('timeout', error, True)

    def activate_current_waypoint(self):
        waypoint = self.loaded_waypoints[self.playback_index]
        self.resolved_target = self.resolve_target(waypoint)
        now = self.get_clock().now()
        self.active_waypoint_started_at = now
        self.last_target_publish_at = now
        self.publish_playback_target(self.resolved_target, waypoint)
        self.publish_status('playing')

    def resolve_target(self, waypoint):
        if self.reference_mode == 'world':
            return {
                'position': waypoint['world_position'],
                'orientation': waypoint['world_orientation']
            }
        pose = self.current_odometry.pose.pose
        _, _, current_yaw = self.quaternion_to_euler(pose.orientation)
        forward = waypoint['body_forward']
        right = waypoint['body_right']
        target_x = (
            pose.position.x
            + math.cos(current_yaw) * forward
            + math.sin(current_yaw) * right
        )
        target_y = (
            pose.position.y
            + math.sin(current_yaw) * forward
            - math.cos(current_yaw) * right
        )
        target_yaw = self.wrap_angle(current_yaw + waypoint['body_yaw'])
        return {
            'position': [target_x, target_y, -waypoint['depth_target']],
            'orientation': self.euler_to_quaternion(0.0, 0.0, target_yaw)
        }

    def republish_active_target(self):
        if self.resolved_target is None:
            return
        now = self.get_clock().now()
        if self.last_target_publish_at is None:
            publish_age = float('inf')
        else:
            publish_age = (now - self.last_target_publish_at).nanoseconds / 1e9
        if publish_age >= self.target_republish_period:
            self.publish_playback_target(
                self.resolved_target, self.loaded_waypoints[self.playback_index]
            )
            self.last_target_publish_at = now

    def publish_playback_target(self, target, waypoint):
        msg = ControlTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.source_mode = 'playback_manual_assisted'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target['position']
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ) = target['orientation']
        msg.carrot_forward = waypoint['carrot_body'][0]
        msg.carrot_right = waypoint['carrot_body'][1]
        self.playback_target_pub.publish(msg)

    def is_target_reached(self):
        pose = self.current_odometry.pose.pose
        target_position = self.resolved_target['position']
        horizontal_error = math.hypot(
            target_position[0] - pose.position.x,
            target_position[1] - pose.position.y
        )
        depth_error = abs(target_position[2] - pose.position.z)
        angle_error = self.quaternion_angular_distance(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ],
            self.resolved_target['orientation']
        )
        return (
            horizontal_error <= self.position_tolerance
            and depth_error <= self.depth_tolerance
            and angle_error <= self.angle_tolerance
        )

    def complete_or_loop(self):
        if self.playback_loop:
            self.playback_index = 0
            self.playback_started_at = self.get_clock().now()
            self.activate_current_waypoint()
            return
        self.stop_playback('completed')
        self.get_logger().info('Playback completed')

    # Safety and status
    def control_stop_callback(self, msg):
        self.control_stopped = bool(msg.data)
        if self.control_stopped and self.is_recording:
            self.stop_recording_and_save()
        if self.control_stopped and self.is_playing:
            self.stop_playback('stopped', 'MANUAL_ASSISTED was stopped')

    def kill_switch_callback(self, msg):
        self.kill_switch_armed = bool(msg.data)
        if self.kill_switch_armed:
            return
        if self.is_recording:
            self.stop_recording_and_save()
        if self.is_playing:
            self.stop_playback('stopped', 'Physical kill switch was pulled')
        else:
            self.last_error = 'Physical kill switch is not armed'
            self.publish_status('stopped')

    def magnetic_switch_callback(self, msg):
        state = bool(msg.data)
        rising_edge = state and not self.last_magnetic_switch_state
        self.last_magnetic_switch_state = state
        if not rising_edge or not self.magnetic_play_enabled:
            return
        if self.is_playing:
            return
        success, message = self.start_playback(False)
        if not success:
            self.last_error = f'Magnetic play rejected: {message}'
            self.publish_status('error')
            self.get_logger().warn(self.last_error)

    def odometry_is_fresh(self):
        if self.current_odometry is None or self.last_odometry_at is None:
            return False
        age = (self.get_clock().now() - self.last_odometry_at).nanoseconds / 1e9
        return age <= self.odometry_timeout

    def publish_status(self, state):
        msg = PlaybackStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.current_index = int(self.playback_index)
        msg.waypoint_count = len(self.loaded_waypoints)
        msg.reference_mode = self.reference_mode
        msg.progression_mode = self.progression_mode
        msg.error = self.last_error
        msg.timed_out = self.last_timed_out
        self.status_pub.publish(msg)

    # Legacy indexed service
    def handle_playback_path(self, request, response):
        success, message = self.load_playback_file(request.file_path)
        if not success:
            self.get_logger().error(message)
            response.position = [0.0, 0.0, 0.0]
            response.angles = [0.0, 0.0, 0.0, 1.0]
            return response
        idx = int(request.idx)
        if idx < 0 or idx >= len(self.loaded_waypoints):
            response.position = [0.0, 0.0, 0.0]
            response.angles = [0.0, 0.0, 0.0, 1.0]
            return response
        waypoint = self.loaded_waypoints[idx]
        response.position = waypoint['world_position']
        response.angles = waypoint['world_orientation']
        return response

    # Math helpers
    @staticmethod
    def wrap_angle(value):
        return (value + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def quaternion_to_euler(quaternion):
        return PlaybackNode.quaternion_to_euler_values([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])

    @staticmethod
    def quaternion_to_euler_values(values):
        x, y, z, w = [float(value) for value in values[:4]]
        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y)
        )
        pitch_argument = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )
        return roll, pitch, yaw

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]

    @staticmethod
    def quaternion_angular_distance(first, second):
        first_norm = math.sqrt(sum(value * value for value in first))
        second_norm = math.sqrt(sum(value * value for value in second))
        if first_norm <= 1e-9 or second_norm <= 1e-9:
            return math.inf
        dot = sum(a * b for a, b in zip(first, second)) / (
            first_norm * second_norm
        )
        return 2.0 * math.acos(max(-1.0, min(1.0, abs(dot))))


def main(args=None):
    rclpy.init(args=args)
    node = PlaybackNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Playback interrupted.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
