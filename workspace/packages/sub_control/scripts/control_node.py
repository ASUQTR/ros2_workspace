#!/usr/bin/env python3
"""
================================================================================
ASUQTR Submarine ROS 2 Control Node.
================================================================================
This module acts as the central nervous system for the sub's movement. It bridges 
the gap between high-level autonomous behaviors (FlexBE), manual gamepad inputs, 
and the low-level LQR mathematical solver.
"""

# ==========================================
# PYTHON STANDARD & 3RD PARTY IMPORTS
# ==========================================
import threading
import asyncio
import math
import numpy as np
from enum import IntEnum
import time

# ==========================================
# ROS 2 IMPORTS
# ==========================================
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Messages and TF2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from sub_interfaces.msg import ControlTarget, PlaybackStatus, ThrusterCommand
from sub_interfaces.action import Control
import tf2_ros

# ==========================================
# ASUQTR IMPORTS
# ==========================================
from sub_control.lqr_solver import SubLQRSolver, THRUST_ALLOC_MAT, Bm

# Default cost matrix parameter values for LQR controller
# Q: State error penalties (Position/Angle springs and Velocity dampers)
DEFAULT_Q = [4*1052.4762, 4*578.7808, 4*670.0731, 4*3134.6757, 4*3959.9523, 20*1453.9726, 4*265.1124, 4*303.5306, 4*204.2912, 4*12.3207, 4*19.8116, 20*6.2442]
# R: Thruster energy penalties (Higher = less aggressive thrust)
DEFAULT_R = [10.0, 10.0, 1000.0, 1000.0, 1000.0, 1000.0, 10.0, 10.0]
DEFAULT_DAMPING_SIGN = -1.0
DEFAULT_MAX_THRUSTER_FORCE = 14.4
DEFAULT_MAX_THROTTLE = 0.8
DEFAULT_JOY_DEAD_ZONE = 0.1
DEFAULT_LQR_PROFILE_TRANSITION_SEC = 1.0
DEFAULT_MANUAL_ASSISTED_DEPTH = 0.5
DEFAULT_MANUAL_ASSISTED_MIN_DEPTH = 0.2
DEFAULT_MANUAL_ASSISTED_MAX_DEPTH = 3.0
DEFAULT_MANUAL_ASSISTED_DEPTH_STEP = 0.1
DEFAULT_MANUAL_ASSISTED_MAX_DEPTH_OFFSET = 0.25
DEFAULT_MANUAL_ASSISTED_MAX_CARROT_DISTANCE = 0.35
DEFAULT_MANUAL_ASSISTED_MAX_CARROT_SPEED = 0.35
DEFAULT_MANUAL_ASSISTED_MAX_FORWARD_SPEED = 0.5
DEFAULT_MANUAL_ASSISTED_MAX_STRAFE_SPEED = 0.0
DEFAULT_MANUAL_ASSISTED_MAX_YAW_RATE = 0.6
DEFAULT_MANUAL_ASSISTED_MAX_ROLL_DEG = 20.0
DEFAULT_MANUAL_ASSISTED_MAX_PITCH_DEG = 15.0
DEFAULT_MANUAL_ASSISTED_JOYSTICK_CURVE = 1.6
DEFAULT_MANUAL_ASSISTED_GAMEPAD_TIMEOUT = 0.75

# ==========================================
# ENUMS
# ==========================================
class ControlMode(IntEnum):
    """
    Defines the operational state of the Submarine.
    
    Attributes:
        BEHAVIOR: Fully autonomous. Listens to Action Server goals.
        LQR_TUNING: Debug mode. Listens to simple pose topics (e.g., from RViz).
        MANUAL: Human override. Listens to Gamepad joystick inputs.
        MANUAL_ASSISTED: Human piloting through bounded LQR target generation.
    """
    BEHAVIOR = 0
    LQR_TUNING = 1
    MANUAL = 2
    MANUAL_ASSISTED = 3

# ==========================================
# NODE DEFINITION
# ==========================================
class ControlNode(Node):
    """
    Main execution class for AUV Control.
    
    This class utilizes a MultiThreadedExecutor. Callback groups are strictly 
    separated into MutuallyExclusiveCallbackGroups to ensure that long-running 
    Action Server loops do not block the high-frequency Odometry callbacks.
    
    To bypass Python Garbage Collection micro-stutters, the core mathematical 
    states (`self.current_state` and `self.target_state`) are statically 
    pre-allocated as 12-element NumPy arrays. All callback updates are performed 
    in-place to achieve near RTOS-level determinism in Python.
    """
    
    def __init__(self):
        """Initialize the node, parameters, pre-allocated memory, ROS publishers/Subscribers
        callback mutex groups and TF2 buffer & frame to allow frame transfer when receiving target
        commands """
        super().__init__('control_node')
        
        # --- Parameters ---
        self.declare_parameter('control_mode', 'behavior')
        self.declare_parameter('state_cost_matrix', DEFAULT_Q)
        self.declare_parameter('thruster_cost_matrix', DEFAULT_R)
        self.declare_parameter('lqr_profile', 'standstill')
        self.declare_parameter('lqr_profile_transition_sec', DEFAULT_LQR_PROFILE_TRANSITION_SEC)
        self.declare_parameter('lqr_profiles.standstill_q', DEFAULT_Q)
        self.declare_parameter('lqr_profiles.standstill_r', DEFAULT_R)
        self.declare_parameter('lqr_profiles.forward_q', DEFAULT_Q)
        self.declare_parameter('lqr_profiles.forward_r', DEFAULT_R)
        self.declare_parameter('lqr_profiles.forward_tuned', False)
        self.declare_parameter('lqr_profiles.turning_q', DEFAULT_Q)
        self.declare_parameter('lqr_profiles.turning_r', DEFAULT_R)
        self.declare_parameter('lqr_profiles.turning_tuned', False)
        self.declare_parameter('publish_lqr_debug_angles', True)
        self.declare_parameter('publish_lqr_dynamics_debug', True)
        self.declare_parameter('debug_invert_roll', False)
        self.declare_parameter('debug_invert_pitch', False)
        self.declare_parameter('debug_invert_yaw', False)
        self.declare_parameter('damping_sign', DEFAULT_DAMPING_SIGN)
        self.declare_parameter('max_thruster_force_newton', DEFAULT_MAX_THRUSTER_FORCE)
        self.declare_parameter('manual_assisted.default_depth_target', DEFAULT_MANUAL_ASSISTED_DEPTH)
        self.declare_parameter('manual_assisted.depth_limits_enabled', True)
        self.declare_parameter('manual_assisted.min_depth_target', DEFAULT_MANUAL_ASSISTED_MIN_DEPTH)
        self.declare_parameter('manual_assisted.max_depth_target', DEFAULT_MANUAL_ASSISTED_MAX_DEPTH)
        self.declare_parameter('manual_assisted.depth_step', DEFAULT_MANUAL_ASSISTED_DEPTH_STEP)
        self.declare_parameter('manual_assisted.max_depth_offset', DEFAULT_MANUAL_ASSISTED_MAX_DEPTH_OFFSET)
        self.declare_parameter('manual_assisted.max_carrot_distance', DEFAULT_MANUAL_ASSISTED_MAX_CARROT_DISTANCE)
        self.declare_parameter('manual_assisted.max_carrot_speed', DEFAULT_MANUAL_ASSISTED_MAX_CARROT_SPEED)
        self.declare_parameter('manual_assisted.max_forward_speed', DEFAULT_MANUAL_ASSISTED_MAX_FORWARD_SPEED)
        self.declare_parameter('manual_assisted.max_strafe_speed', DEFAULT_MANUAL_ASSISTED_MAX_STRAFE_SPEED)
        self.declare_parameter('manual_assisted.max_yaw_rate', DEFAULT_MANUAL_ASSISTED_MAX_YAW_RATE)
        self.declare_parameter('manual_assisted.max_roll_deg', DEFAULT_MANUAL_ASSISTED_MAX_ROLL_DEG)
        self.declare_parameter('manual_assisted.max_pitch_deg', DEFAULT_MANUAL_ASSISTED_MAX_PITCH_DEG)
        self.declare_parameter('manual_assisted.joystick_curve', DEFAULT_MANUAL_ASSISTED_JOYSTICK_CURVE)
        self.declare_parameter('manual_assisted.gamepad_timeout_sec', DEFAULT_MANUAL_ASSISTED_GAMEPAD_TIMEOUT)

        # PERFORMANCE ARCHITECTURE: Pre-allocate State Arrays
        # Initializing with NaN ensures the LQR math functions won't 
        # accidentally process empty zero-data before the first localization message arrives.
        self.current_state = np.full(12, np.nan, dtype=np.float64)
        self.target_state = np.full(12, np.nan, dtype=np.float64) 
        self.manual_assisted_carrot_body = np.zeros(2, dtype=np.float64)
        self.manual_assisted_default_depth_target = DEFAULT_MANUAL_ASSISTED_DEPTH
        self.manual_assisted_depth_limits_enabled = True
        self.manual_assisted_min_depth_target = DEFAULT_MANUAL_ASSISTED_MIN_DEPTH
        self.manual_assisted_max_depth_target = DEFAULT_MANUAL_ASSISTED_MAX_DEPTH
        self.manual_assisted_depth_step = DEFAULT_MANUAL_ASSISTED_DEPTH_STEP
        self.manual_assisted_max_depth_offset = DEFAULT_MANUAL_ASSISTED_MAX_DEPTH_OFFSET
        self.manual_assisted_max_carrot_distance = DEFAULT_MANUAL_ASSISTED_MAX_CARROT_DISTANCE
        self.manual_assisted_max_carrot_speed = DEFAULT_MANUAL_ASSISTED_MAX_CARROT_SPEED
        self.manual_assisted_max_forward_speed = DEFAULT_MANUAL_ASSISTED_MAX_FORWARD_SPEED
        self.manual_assisted_max_strafe_speed = DEFAULT_MANUAL_ASSISTED_MAX_STRAFE_SPEED
        self.manual_assisted_max_yaw_rate = DEFAULT_MANUAL_ASSISTED_MAX_YAW_RATE
        self.manual_assisted_max_roll = math.radians(DEFAULT_MANUAL_ASSISTED_MAX_ROLL_DEG)
        self.manual_assisted_max_pitch = math.radians(DEFAULT_MANUAL_ASSISTED_MAX_PITCH_DEG)
        self.manual_assisted_joystick_curve = DEFAULT_MANUAL_ASSISTED_JOYSTICK_CURVE
        self.manual_assisted_gamepad_timeout = DEFAULT_MANUAL_ASSISTED_GAMEPAD_TIMEOUT
        self.manual_assisted_base_depth_target = DEFAULT_MANUAL_ASSISTED_DEPTH
        self.manual_assisted_depth_target = DEFAULT_MANUAL_ASSISTED_DEPTH
        self.manual_assisted_yaw_target = 0.0
        self.manual_assisted_last_update_time = None
        self.manual_assisted_last_gamepad_time = None
        self.manual_assisted_last_playback_time = None
        self.manual_assisted_playback_active = False
        self.last_depth_up_button_state = False
        self.last_depth_down_button_state = False
        self.last_hold_button_state = False
        self.last_lqr_profile_button_states = {
            'standstill': False,
            'forward': False,
            'turning': False
        }
        self.software_kill_active = False
        self._last_mode_for_transition = None
        self.q_values = np.array(DEFAULT_Q, dtype=np.float64)
        self.r_values = np.array(DEFAULT_R, dtype=np.float64)
        self.lqr_profile_transition_sec = DEFAULT_LQR_PROFILE_TRANSITION_SEC
        self.lqr_profile_transition = None

        # Load initial parameters
        mode_str = self.get_parameter('control_mode').value.lower()
        self._set_mode_from_string(mode_str)
        self.update_q_matrix(self.get_parameter('state_cost_matrix').value)
        self.update_r_matrix(self.get_parameter('thruster_cost_matrix').value)
        self.lqr_profile = str(self.get_parameter('lqr_profile').value)
        self.lqr_profile_transition_sec = float(self.get_parameter('lqr_profile_transition_sec').value)
        self.publish_lqr_debug_angles = bool(self.get_parameter('publish_lqr_debug_angles').value)
        self.publish_lqr_dynamics_debug = bool(self.get_parameter('publish_lqr_dynamics_debug').value)
        self.debug_invert_roll = bool(self.get_parameter('debug_invert_roll').value)
        self.debug_invert_pitch = bool(self.get_parameter('debug_invert_pitch').value)
        self.debug_invert_yaw = bool(self.get_parameter('debug_invert_yaw').value)
        self.damping_sign = float(self.get_parameter('damping_sign').value)
        self.max_thruster_force_newton = float(self.get_parameter('max_thruster_force_newton').value)
        self.manual_assisted_default_depth_target = float(self.get_parameter('manual_assisted.default_depth_target').value)
        self.manual_assisted_depth_limits_enabled = bool(self.get_parameter('manual_assisted.depth_limits_enabled').value)
        self.manual_assisted_min_depth_target = float(self.get_parameter('manual_assisted.min_depth_target').value)
        self.manual_assisted_max_depth_target = float(self.get_parameter('manual_assisted.max_depth_target').value)
        self.manual_assisted_depth_step = float(self.get_parameter('manual_assisted.depth_step').value)
        self.manual_assisted_max_depth_offset = float(self.get_parameter('manual_assisted.max_depth_offset').value)
        self.manual_assisted_max_carrot_distance = float(self.get_parameter('manual_assisted.max_carrot_distance').value)
        self.manual_assisted_max_carrot_speed = float(self.get_parameter('manual_assisted.max_carrot_speed').value)
        self.manual_assisted_max_forward_speed = float(self.get_parameter('manual_assisted.max_forward_speed').value)
        self.manual_assisted_max_strafe_speed = float(self.get_parameter('manual_assisted.max_strafe_speed').value)
        self.manual_assisted_max_yaw_rate = float(self.get_parameter('manual_assisted.max_yaw_rate').value)
        self.manual_assisted_max_roll = math.radians(float(self.get_parameter('manual_assisted.max_roll_deg').value))
        self.manual_assisted_max_pitch = math.radians(float(self.get_parameter('manual_assisted.max_pitch_deg').value))
        self.manual_assisted_joystick_curve = float(self.get_parameter('manual_assisted.joystick_curve').value)
        self.manual_assisted_gamepad_timeout = float(self.get_parameter('manual_assisted.gamepad_timeout_sec').value)
        self.manual_assisted_depth_target = self.manual_assisted_default_depth_target
        if self.manual_assisted_depth_limits_enabled:
            self.manual_assisted_depth_target = self._clamp(
                self.manual_assisted_depth_target,
                self.manual_assisted_min_depth_target,
                self.manual_assisted_max_depth_target
            )
        
        # Bind dynamic reconfigure callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- Instantiate Mathematical Engine ---
        self.lqr_solver = SubLQRSolver()
        if self.damping_sign not in (-1.0, 1.0):
            self.get_logger().warn(
                f"Invalid damping_sign={self.damping_sign}. Using default {DEFAULT_DAMPING_SIGN}. Allowed values are -1.0 or 1.0."
            )
            self.damping_sign = DEFAULT_DAMPING_SIGN
        if self.max_thruster_force_newton <= 0.0:
            self.get_logger().warn(
                f"Invalid max_thruster_force_newton={self.max_thruster_force_newton}. Using default {DEFAULT_MAX_THRUSTER_FORCE}."
            )
            self.max_thruster_force_newton = DEFAULT_MAX_THRUSTER_FORCE
        self.lqr_solver.set_damping_sign(self.damping_sign)
        
        # --- Thread Safety & State ---
        # The Action Server thread can update the target_state at the exact 
        # same time the Odometry thread is reading it for LQR math. This lock
        # prevents race conditions resulting in torn data reads.
        self.target_state_lock = threading.Lock()
        
        # --- Callback Groups ---
        # Those allow to make sure the callback cannot interrupt itself if the topic
        # is spammed. In onther words, it makes the callbacks non reentrant
        self.localization_cb_group = MutuallyExclusiveCallbackGroup()
        self.action_cb_group = MutuallyExclusiveCallbackGroup()
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.global_reference_frame = 'odom'

        # --- Subscriptions & Publishers  & QoS ---
        # For sensors/thruster command, we only want latest value and avoid latency introduced by RELIABLE QoS
        only_latest_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.localization_sub = self.create_subscription(
            Odometry, 'odometry/filtered', self.localization_callback, only_latest_qos,
            callback_group=self.localization_cb_group
        )
        
        self.debug_target_sub = self.create_subscription(
            PoseStamped, 'debug/target_pose', self.debug_target_callback, only_latest_qos,
            callback_group=self.action_cb_group 
        )
        
        self.gamepad_sub = self.create_subscription(
             Joy, 'dashboard/gamepad', self.gamepad_callback, only_latest_qos,
             callback_group=self.sensor_cb_group
        )
        self.playback_target_sub = self.create_subscription(
             ControlTarget, 'control/playback_target', self.playback_target_callback, only_latest_qos,
             callback_group=self.action_cb_group
        )
        self.playback_status_sub = self.create_subscription(
             PlaybackStatus, 'playback/status', self.playback_status_callback, latched_qos,
             callback_group=self.action_cb_group
        )
        
        self.thruster_pub = self.create_publisher(
            ThrusterCommand, 'thruster_cmd', only_latest_qos
        )
        self.disable_pwm_pub = self.create_publisher(
            Bool, 'disable_pwm', latched_qos
        )
        self.lqr_debug_pub = self.create_publisher(
            Float64MultiArray, 'debug/lqr_angles', only_latest_qos
        )
        self.lqr_velocity_pub = self.create_publisher(
            Float64MultiArray, 'debug/lqr_velocity', only_latest_qos
        )
        self.lqr_accel_cmd_pub = self.create_publisher(
            Float64MultiArray, 'debug/lqr_accel_cmd', only_latest_qos
        )
        self.lqr_dynamics_pub = self.create_publisher(
            Float64MultiArray, 'debug/lqr_dynamics', only_latest_qos
        )
        self.manual_assisted_debug_pub = self.create_publisher(
            Float64MultiArray, 'debug/manual_assisted', only_latest_qos
        )
        self.control_target_pub = self.create_publisher(
            ControlTarget, 'control/target', only_latest_qos
        )

        if self._current_mode == ControlMode.MANUAL_ASSISTED:
            self._publish_software_kill_state(True)
            self._publish_zero_thrust()
            self.get_logger().warn(
                "MANUAL_ASSISTED ready but STOPPED. Press Start to arm."
            )

        # --- Action Server ---
        self._action_server = ActionServer(
            self, Control, 'navigate_sub', self.control_action_callback,
            callback_group=self.action_cb_group
        )

        self.last_mode_switch_button_state = False
        self.get_logger().info("Sub Control Node Initialized.")
        self.get_logger().info(
            f"LQR debug angles pub: {self.publish_lqr_debug_angles} | "
            f"LQR dynamics debug pub: {self.publish_lqr_dynamics_debug} | "
            f"invert R/P/Y: {self.debug_invert_roll}/{self.debug_invert_pitch}/{self.debug_invert_yaw} | "
                        f"damping_sign: {self.damping_sign} | "
                        f"max_thruster_force_newton: {self.max_thruster_force_newton} | "
                        f"manual_assisted depth/carrot/yaw: "
                        f"{self.manual_assisted_depth_target:.2f}m/"
                        f"{self.manual_assisted_max_carrot_distance:.2f}m/"
                        f"{self.manual_assisted_max_yaw_rate:.2f}rad/s"
        )


    # ==========================================
    # PARAMETER HANDLING
    # ==========================================

    def parameter_callback(self, params):
        """
        Dynamically update node parameters without requiring a restart.
        
        Args:
            params (list): A list of rclpy.parameter.Parameter objects.
            
        Returns:
            SetParametersResult: Success or failure status of the parameter update.
        """
        validation_result = self._validate_parameter_batch(params)
        if not validation_result.successful:
            return validation_result

        matrix_params = {
            param.name: param for param in params
            if param.name in ('state_cost_matrix', 'thruster_cost_matrix')
        }
        if matrix_params:
            q_values = matrix_params.get(
                'state_cost_matrix',
                Parameter('state_cost_matrix', value=np.diag(self.q_matrix).tolist())
            ).value
            r_values = matrix_params.get(
                'thruster_cost_matrix',
                Parameter('thruster_cost_matrix', value=np.diag(self.r_matrix).tolist())
            ).value
            q_values = list(q_values)
            r_values = list(r_values)
            matrix_error = self._validate_cost_matrices(q_values, r_values)
            if matrix_error:
                return SetParametersResult(successful=False, reason=matrix_error)
            self.lqr_profile_transition = None
            self.update_q_matrix(q_values)
            self.update_r_matrix(r_values)

        for param in params:
            if param.name in matrix_params:
                continue
            if param.name == 'control_mode':
                if self._set_mode_from_string(param.value.lower()):
                    continue
                return SetParametersResult(successful=False, reason="Invalid mode string")
            elif param.name == 'lqr_profile':
                result = self._set_lqr_profile(str(param.value))
                if not result.successful:
                    return result
            elif param.name == 'lqr_profile_transition_sec':
                if param.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason='lqr_profile_transition_sec must be a float (double)'
                    )
                self.lqr_profile_transition_sec = max(0.0, float(param.value))
            elif param.name == 'publish_lqr_debug_angles':
                self.publish_lqr_debug_angles = bool(param.value)
            elif param.name == 'publish_lqr_dynamics_debug':
                self.publish_lqr_dynamics_debug = bool(param.value)
            elif param.name == 'debug_invert_roll':
                self.debug_invert_roll = bool(param.value)
            elif param.name == 'debug_invert_pitch':
                self.debug_invert_pitch = bool(param.value)
            elif param.name == 'debug_invert_yaw':
                self.debug_invert_yaw = bool(param.value)
            elif param.name == 'damping_sign':
                if param.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(successful=False, reason='damping_sign must be a float (double)')
                if float(param.value) not in (-1.0, 1.0):
                    return SetParametersResult(successful=False, reason='damping_sign must be either -1.0 or 1.0')
                self.damping_sign = float(param.value)
                self.lqr_solver.set_damping_sign(self.damping_sign)
                self.get_logger().info(f"Damping sign set to: {self.damping_sign}")
            elif param.name == 'max_thruster_force_newton':
                if param.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(successful=False, reason='max_thruster_force_newton must be a float (double)')
                if float(param.value) <= 0.0:
                    return SetParametersResult(successful=False, reason='max_thruster_force_newton must be > 0.0')
                self.max_thruster_force_newton = float(param.value)
                self.get_logger().info(f"Max thruster force set to: +/-{self.max_thruster_force_newton} N")
            elif param.name.startswith('manual_assisted.'):
                result = self._set_manual_assisted_parameter(param)
                if not result.successful:
                    return result
            elif param.name.startswith('lqr_profiles.'):
                continue
            else:
                return SetParametersResult(
                    successful=False, reason=f'Unhandled parameter: {param.name}'
                )

        return SetParametersResult(successful=True)

    def _validate_parameter_batch(self, params):
        """Validate a complete ROS parameter request before changing runtime state."""
        by_name = {param.name: param for param in params}

        if 'state_cost_matrix' in by_name or 'thruster_cost_matrix' in by_name:
            q_values = list(by_name.get(
                'state_cost_matrix',
                Parameter('state_cost_matrix', value=np.diag(self.q_matrix).tolist())
            ).value)
            r_values = list(by_name.get(
                'thruster_cost_matrix',
                Parameter('thruster_cost_matrix', value=np.diag(self.r_matrix).tolist())
            ).value)
            matrix_error = self._validate_cost_matrices(q_values, r_values)
            if matrix_error:
                return SetParametersResult(successful=False, reason=matrix_error)

        depth_limits_enabled = bool(by_name.get(
            'manual_assisted.depth_limits_enabled',
            Parameter(
                'manual_assisted.depth_limits_enabled',
                value=self.manual_assisted_depth_limits_enabled
            )
        ).value)
        try:
            min_depth = float(by_name.get(
                'manual_assisted.min_depth_target',
                Parameter(
                    'manual_assisted.min_depth_target',
                    value=self.manual_assisted_min_depth_target
                )
            ).value)
            max_depth = float(by_name.get(
                'manual_assisted.max_depth_target',
                Parameter(
                    'manual_assisted.max_depth_target',
                    value=self.manual_assisted_max_depth_target
                )
            ).value)
        except (TypeError, ValueError):
            return SetParametersResult(
                successful=False,
                reason='manual_assisted depth limits must be numeric'
            )
        if depth_limits_enabled and min_depth > max_depth:
            return SetParametersResult(
                successful=False,
                reason='manual_assisted.min_depth_target must be <= max_depth_target'
            )

        for param in params:
            if param.name in ('state_cost_matrix', 'thruster_cost_matrix'):
                continue
            if param.name == 'control_mode':
                if str(param.value).lower() not in (
                    'behavior', 'manual', 'lqr_tuning', 'manual_assisted'
                ):
                    return SetParametersResult(
                        successful=False, reason='Invalid mode string'
                    )
            elif param.name == 'lqr_profile':
                result = self._validate_lqr_profile(str(param.value))
                if not result.successful:
                    return result
            elif param.name == 'lqr_profile_transition_sec':
                if param.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason='lqr_profile_transition_sec must be a float (double)'
                    )
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='lqr_profile_transition_sec must be >= 0.0'
                    )
            elif param.name in (
                'publish_lqr_debug_angles',
                'publish_lqr_dynamics_debug',
                'debug_invert_roll',
                'debug_invert_pitch',
                'debug_invert_yaw'
            ):
                continue
            elif param.name == 'damping_sign':
                if param.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason='damping_sign must be a float (double)'
                    )
                if float(param.value) not in (-1.0, 1.0):
                    return SetParametersResult(
                        successful=False,
                        reason='damping_sign must be either -1.0 or 1.0'
                    )
            elif param.name == 'max_thruster_force_newton':
                if param.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason='max_thruster_force_newton must be a float (double)'
                    )
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='max_thruster_force_newton must be > 0.0'
                    )
            elif param.name.startswith('manual_assisted.'):
                result = self._validate_manual_assisted_parameter(param)
                if not result.successful:
                    return result
            elif param.name.startswith('lqr_profiles.'):
                continue
            else:
                return SetParametersResult(
                    successful=False, reason=f'Unhandled parameter: {param.name}'
                )

        return SetParametersResult(successful=True)

    @staticmethod
    def _validate_cost_matrices(q_values, r_values):
        if not isinstance(q_values, (list, tuple)) or len(q_values) != 12:
            return 'state_cost_matrix must contain 12 values'
        if not isinstance(r_values, (list, tuple)) or len(r_values) != 8:
            return 'thruster_cost_matrix must contain 8 values'
        if any(not math.isfinite(float(value)) or float(value) < 0.0 for value in q_values):
            return 'state_cost_matrix values must be finite and >= 0'
        if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in r_values):
            return 'thruster_cost_matrix values must be finite and > 0'
        return ''

    def _set_lqr_profile(self, profile_name):
        validation_result = self._validate_lqr_profile(profile_name)
        if not validation_result.successful:
            return validation_result

        normalized = profile_name.strip().lower()
        q_values = list(self.get_parameter(f'lqr_profiles.{normalized}_q').value)
        r_values = list(self.get_parameter(f'lqr_profiles.{normalized}_r').value)
        self._start_lqr_profile_transition(q_values, r_values, normalized)
        self.lqr_profile = normalized
        self.get_logger().warn(
            f'LQR profile target changed to {normalized} '
            f'over {self.lqr_profile_transition_sec:.2f} s'
        )
        return SetParametersResult(successful=True)

    def _validate_lqr_profile(self, profile_name):
        normalized = profile_name.strip().lower()
        if normalized not in ('standstill', 'forward', 'turning'):
            return SetParametersResult(successful=False, reason='Unknown LQR profile')
        q_values = list(self.get_parameter(f'lqr_profiles.{normalized}_q').value)
        r_values = list(self.get_parameter(f'lqr_profiles.{normalized}_r').value)
        error = self._validate_cost_matrices(q_values, r_values)
        if error:
            return SetParametersResult(successful=False, reason=error)
        return SetParametersResult(successful=True)

    def _start_lqr_profile_transition(self, q_target, r_target, profile_name):
        q_target = np.array(q_target, dtype=np.float64)
        r_target = np.array(r_target, dtype=np.float64)
        duration = float(self.lqr_profile_transition_sec)
        if duration <= 1e-6:
            self.lqr_profile_transition = None
            self.update_q_matrix(q_target.tolist())
            self.update_r_matrix(r_target.tolist())
            return

        self.lqr_profile_transition = {
            'profile': profile_name,
            'start_time': time.monotonic(),
            'duration': duration,
            'q_start': self.q_values.copy(),
            'r_start': self.r_values.copy(),
            'q_target': q_target,
            'r_target': r_target,
        }

    def _update_lqr_profile_transition(self):
        transition = self.lqr_profile_transition
        if transition is None:
            return
        duration = max(transition['duration'], 1e-6)
        alpha = (time.monotonic() - transition['start_time']) / duration
        alpha = max(0.0, min(1.0, alpha))
        q_values = (
            (1.0 - alpha) * transition['q_start']
            + alpha * transition['q_target']
        )
        r_values = (
            (1.0 - alpha) * transition['r_start']
            + alpha * transition['r_target']
        )
        self.update_q_matrix(q_values.tolist())
        self.update_r_matrix(r_values.tolist())
        if alpha >= 1.0:
            self.lqr_profile_transition = None
            self.get_logger().info(
                f"LQR profile transition complete: {transition['profile']}"
            )
        
    def _set_mode_from_string(self, mode_str):
        """
        Map a string parameter to the internal ControlMode Enum.
        
        Args:
            mode_str (str): The requested control mode as a string.
            
        Returns:
            bool: True if the mode was successfully mapped, False otherwise.
        """
        if mode_str == 'behavior':
            self._current_mode = ControlMode.BEHAVIOR
        elif mode_str == 'lqr_tuning':
            self._current_mode = ControlMode.LQR_TUNING
        elif mode_str == 'manual':
            self._current_mode = ControlMode.MANUAL
        elif mode_str == 'manual_assisted':
            self._current_mode = ControlMode.MANUAL_ASSISTED
        else:
            self.get_logger().warn(f"Unknown control mode: {mode_str}")
            return False
        if hasattr(self, '_last_mode_for_transition') and self._last_mode_for_transition != self._current_mode:
            if self._current_mode == ControlMode.MANUAL_ASSISTED:
                self.software_kill_active = True
                if hasattr(self, 'disable_pwm_pub'):
                    self._publish_software_kill_state(True)
                    self._publish_zero_thrust()
                    self.get_logger().warn(
                        "MANUAL_ASSISTED selected but STOPPED. Press Start to arm."
                    )
            self._last_mode_for_transition = self._current_mode
        self.get_logger().info(f"SUB Control mode set as: {self._current_mode.name}")
        return True
    
    def update_q_matrix(self, q_list):
        """
        Convert a 1D Python list parameter into the 2D diagonal numpy array.
        
        Args:
            q_list (list): 12-element list representing state cost weights.
            
        Raises:
            ValueError: If the input list does not contain exactly 12 elements.
        """
        try:
            if isinstance(q_list, (list, tuple)) and len(q_list) == 12:
                self.q_values = np.array(q_list, dtype=np.float64)
                self.q_matrix = np.diag(self.q_values).astype(np.float64)
                return SetParametersResult(successful=True)
            else:
                raise ValueError("Q matrix parameter must be a list of 12 elements")
        except Exception as e:
            return SetParametersResult(successful=False)


    def update_r_matrix(self, r_list):
        """
        Convert list to 2D diagonal array and pre-calculate the inverse.
        
        Args:
            r_list (list): 8-element list representing thruster cost weights.
            
        Raises:
            ValueError: If the input list does not contain exactly 8 elements.
        """
        try:
            if isinstance(r_list, (list, tuple)) and len(r_list) == 8:
                self.r_values = np.array(r_list, dtype=np.float64)
                self.r_matrix = np.diag(self.r_values).astype(np.float64)
                # Math optimization: K = R^-1 * B^T * X. We pre-compute R^-1 here so we 
                # don't waste CPU cycles inverting the matrix during the hot control loop.
                self.inv_r_matrix = np.linalg.inv(self.r_matrix)
                return SetParametersResult(successful=True)
            else:
                raise ValueError("R matrix parameter must be a list of 8 elements")
        except Exception as e:
            return SetParametersResult(successful=False)


    # ==========================================
    # LOCALIZATION & TARGET STATE CALLBACKS
    # ==========================================
    
    def wrap_angles_to_pi(self, angles):
        """
        Wrap an array of angles to the [-pi, pi] range.
        
        This ensures the LQR angular math calculates the shortest rotational path 
        (e.g., executing a 90-degree right turn instead of a 270-degree left turn).
        
        Args:
            angles (np.ndarray): An array of angles in radians.
            
        Returns:
            np.ndarray: The wrapped angles.
        """
        return (angles + np.pi) % (2 * np.pi) - np.pi
    
    def localization_callback(self, msg):
        """
        Process incoming localization data, update state, and trigger the LQR solver.
        
        This is the heartbeat of the controller. It executes entirely synchronously 
        to guarantee zero phase-lag between sensor reception and thruster command output.
        
        Args:
            msg (Odometry): The incoming filtered odometry message.
        """
        # Start the profiling stopwatch
        start_time = time.perf_counter()
        
        roll_flu, pitch_flu, yaw_flu = self.quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # ----------------------------------------------------------------------
        # FRAME TRANSLATION: ROS ENU/FLU to Marine NED/FRD
        # ROS strictly uses East-North-Up (World) and Forward-Left-Up (Body).
        # Our physical LQR math model assumes North-East-Down (World) and 
        # Forward-Right-Down (Body). We intercept and translate them here.
        # ----------------------------------------------------------------------
        
        # Update the pre-allocated current_state array IN-PLACE (zero-copy overhead)
        
        # World Frame: ENU -> NED
        self.current_state[0] = msg.pose.pose.position.y     # X becomes North (Y)
        self.current_state[1] = msg.pose.pose.position.x     # Y becomes East (X)
        self.current_state[2] = -msg.pose.pose.position.z    # Z is inverted (Down)
        self.current_state[3] = roll_flu
        self.current_state[4] = -pitch_flu                   # Pitch is inverted
        
        # Shift Yaw 90 degrees so 0 Rad faces North instead of East
        yaw_ned = (math.pi / 2.0) - yaw_flu
        self.current_state[5] = (yaw_ned + math.pi) % (2 * math.pi) - math.pi

        # Runtime diagnostics: optional sign flips to isolate axis convention issues
        if self.debug_invert_roll:
            self.current_state[3] = -self.current_state[3]
        if self.debug_invert_pitch:
            self.current_state[4] = -self.current_state[4]
        if self.debug_invert_yaw:
            self.current_state[5] = -self.current_state[5]
            self.current_state[5] = (self.current_state[5] + math.pi) % (2 * math.pi) - math.pi

        # Body Frame: FLU -> FRD (Velocities)
        self.current_state[6] = msg.twist.twist.linear.x
        self.current_state[7] = -msg.twist.twist.linear.y    # Left to Right
        self.current_state[8] = -msg.twist.twist.linear.z    # Up to Down
        self.current_state[9] = msg.twist.twist.angular.x
        self.current_state[10] = -msg.twist.twist.angular.y  # Pitch to -Pitch
        self.current_state[11] = -msg.twist.twist.angular.z  # Yaw to -Yaw

        if self.publish_lqr_debug_angles:
            dbg = Float64MultiArray()
            dbg.data = [
                roll_flu, pitch_flu, yaw_flu,
                self.current_state[3], self.current_state[4], self.current_state[5]
            ]
            self.lqr_debug_pub.publish(dbg)
        
        # --- Target State Management ---
        with self.target_state_lock:
            # First-run initialization: If the Sub just booted, set its target 
            # coordinate to exactly where it currently is so it holds position.
            if np.isnan(self.target_state[0]):
                self.get_logger().info("Initializing target_state to current position (zero velocity).")
                self.target_state[0:6] = self.current_state[0:6]
                self.target_state[6:12] = 0.0 # Force target velocities to 0 for station-keeping

            if self._current_mode == ControlMode.MANUAL_ASSISTED and not self._has_recent_playback_target():
                self._apply_manual_assisted_timeout()
                
            # Copy to local thread-safe variable for math below so the Action Server
            # doesn't overwrite the memory addresses mid-calculation.
            target_state_copy = self.target_state.copy()

        if self._current_mode == ControlMode.MANUAL_ASSISTED:
            self._publish_manual_assisted_target(target_state_copy, msg.header.stamp)

        if self.software_kill_active:
            self._publish_zero_thrust()
            return
        
        # --- Execute Control ---
        if self._current_mode in [ControlMode.BEHAVIOR, ControlMode.LQR_TUNING, ControlMode.MANUAL_ASSISTED]:
            self._update_lqr_profile_transition()
            
            # LQR convention: x_error = current - target, then u = -K * x_error.
            # Using target - current here flips the feedback sign and drives the
            # sub away from the target.
            lqr_error = self.current_state - target_state_copy
            
            # Crucial: Wrap Roll/Pitch/Yaw errors so the sub doesn't aggressively spin 360 degrees
            lqr_error[3:6] = self.wrap_angles_to_pi(lqr_error[3:6])
            
            # Fire the mathematical solver
            thrusters_force = self.lqr_solver.compute_thrust_force(
                self.current_state, lqr_error, self.q_matrix, self.r_matrix, self.inv_r_matrix
            )

            frozen = self.lqr_solver.frozen_gain_count
            max_frozen = self.lqr_solver.max_frozen_cycles
            if frozen > max_frozen:
                self.get_logger().error(
                    f"ARE solver failed for {frozen} consecutive cycle(s): "
                    f"frozen gain limit ({max_frozen}) exceeded, zeroing thrust. "
                    f"pitch={math.degrees(self.current_state[4]):.1f} deg",
                    throttle_duration_sec=1.0
                )
            elif frozen > 0:
                self.get_logger().warn(
                    f"ARE solver failed: using frozen gain for {frozen}/{max_frozen} cycle(s). "
                    f"pitch={math.degrees(self.current_state[4]):.1f} deg",
                    throttle_duration_sec=1.0
                )

            # Enforce physical actuator limits in software (per thruster, Newtons).
            thrusters_force = np.clip(
                thrusters_force,
                -self.max_thruster_force_newton,
                self.max_thruster_force_newton
            )

            if self.publish_lqr_dynamics_debug:
                # Predicted accelerations from current thruster command in NED/FRD dynamics.
                accel_cmd = Bm[6:12, :].dot(thrusters_force)

                vel_msg = Float64MultiArray()
                # Order: [u, v, w, p, q, r]
                vel_msg.data = [
                    self.current_state[6], self.current_state[7], self.current_state[8],
                    self.current_state[9], self.current_state[10], self.current_state[11]
                ]
                self.lqr_velocity_pub.publish(vel_msg)

                accel_msg = Float64MultiArray()
                # Order: [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot]
                accel_msg.data = [
                    accel_cmd[0], accel_cmd[1], accel_cmd[2],
                    accel_cmd[3], accel_cmd[4], accel_cmd[5]
                ]
                self.lqr_accel_cmd_pub.publish(accel_msg)

                dyn_msg = Float64MultiArray()
                dyn_msg.data = [
                    self.current_state[6], self.current_state[7], self.current_state[8],
                    self.current_state[9], self.current_state[10], self.current_state[11],
                    accel_cmd[0], accel_cmd[1], accel_cmd[2],
                    accel_cmd[3], accel_cmd[4], accel_cmd[5]
                ]
                self.lqr_dynamics_pub.publish(dyn_msg)
            
            # Pack and fire to the hardware node
            thrust_msg = ThrusterCommand(efforts=thrusters_force.tolist())
            self.thruster_pub.publish(thrust_msg)
    	# Stop the stopwatch and calculate duration in milliseconds
        end_time = time.perf_counter()
        exec_time_ms = (end_time - start_time) * 1000.0
        # Log it safely (Only prints once per second to prevent I/O lag)
        self.get_logger().info(f"LQR Math Execution Time: {exec_time_ms:.3f} ms", throttle_duration_sec=1.0)

    def debug_target_callback(self, msg):
        """
        Directly update the target_state if in LQR tuning mode.
        
        **HUMAN-FRIENDLY OVERRIDE**
        To make debugging easier via CLI or the ASUQTR Dashboard, this callback 
        completely bypasses standard ROS ENU/Quaternion rules. 
        
        Position is expected STRICTLY in NED (North, East, Down).
        - msg.pose.position.x = North (meters)
        - msg.pose.position.y = East (meters)
        - msg.pose.position.z = Down (meters)
        
        Orientation is expected STRICTLY in NED Euler Degrees.
        - msg.pose.orientation.x = Roll (Degrees, NED)
        - msg.pose.orientation.y = Pitch (Degrees, NED)
        - msg.pose.orientation.z = Yaw (Degrees, NED)
        
        WARNING: Do NOT use RViz's 2D Nav Goal with this topic. RViz sends 
        ENU quaternions, which this function will wildly misinterpret.
        
        Args:
            msg (PoseStamped): The requested target pose. 
        """
        if self._current_mode != ControlMode.LQR_TUNING:
            return 
            
        with self.target_state_lock:
            if not np.isnan(self.target_state[0]):
                
                # 1. POSITION: Direct NED Mapping
                self.target_state[0] = msg.pose.position.x     # North
                self.target_state[1] = msg.pose.position.y     # East
                self.target_state[2] = msg.pose.position.z     # Down
                
                # 2. ORIENTATION: Direct NED Mapping + Degrees to Radians
                self.target_state[3] = math.radians(msg.pose.orientation.x) # Roll
                self.target_state[4] = math.radians(msg.pose.orientation.y) # Pitch
                
                # 3. Wrap Yaw safely in radians
                yaw_rad = math.radians(msg.pose.orientation.z)
                self.target_state[5] = (yaw_rad + math.pi) % (2 * math.pi) - math.pi
                
                self.get_logger().info(
                    f"Debug Target (NED): N:{self.target_state[0]:.1f}, E:{self.target_state[1]:.1f}, D:{self.target_state[2]:.1f} | "
                    f"R:{msg.pose.orientation.x:.1f}°, P:{msg.pose.orientation.y:.1f}°, Y:{msg.pose.orientation.z:.1f}°"
                )

    def playback_target_callback(self, msg):
        """Apply recorded MANUAL_ASSISTED targets while keeping the LQR active."""
        if self._current_mode != ControlMode.MANUAL_ASSISTED:
            return
        if msg.source_mode != 'playback_manual_assisted':
            return

        roll_flu, pitch_flu, yaw_flu = self.quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        yaw_ned = (math.pi / 2.0) - yaw_flu

        with self.target_state_lock:
            self.target_state[0] = msg.pose.position.y
            self.target_state[1] = msg.pose.position.x
            self.target_state[2] = -msg.pose.position.z
            self.target_state[3] = roll_flu
            self.target_state[4] = -pitch_flu
            self.target_state[5] = (yaw_ned + math.pi) % (2 * math.pi) - math.pi
            self.target_state[6] = msg.twist.linear.x
            self.target_state[7] = -msg.twist.linear.y
            self.target_state[8] = -msg.twist.linear.z
            self.target_state[9] = msg.twist.angular.x
            self.target_state[10] = -msg.twist.angular.y
            self.target_state[11] = -msg.twist.angular.z
            self.manual_assisted_carrot_body[0] = msg.carrot_forward
            self.manual_assisted_carrot_body[1] = -msg.carrot_right
            self.manual_assisted_depth_target = self.target_state[2]
            self.manual_assisted_yaw_target = self.target_state[5]
            self.manual_assisted_last_playback_time = self._now_seconds()

    def playback_status_callback(self, msg):
        """Give playback exclusive ownership of the assisted target while playing."""
        was_active = self.manual_assisted_playback_active
        self.manual_assisted_playback_active = msg.state == 'playing'
        if was_active and not self.manual_assisted_playback_active:
            self.manual_assisted_last_playback_time = None
            self.manual_assisted_carrot_body[:] = 0.0
            self.manual_assisted_last_update_time = self._now_seconds()
            self.get_logger().info(
                f"Playback target released after state '{msg.state}'."
            )

    def _has_recent_playback_target(self):
        if self.manual_assisted_playback_active:
            return True
        last = self.manual_assisted_last_playback_time
        if last is None:
            return False
        return (self._now_seconds() - last) <= self.manual_assisted_gamepad_timeout

    # ==========================================
    # GAMEPAD CALLBACK
    # ==========================================
    def gamepad_callback(self, msg):
        """
        Map gamepad joysticks directly to physical movement vectors.
        
        This translates manual analog sticks into thrust vectors using the 
        Thruster Allocation Matrix, completely bypassing the LQR controller.
        
        Args:
            msg (Joy): The incoming gamepad state.
        """
        # Extracting axis, button and data
        button_a = self._button(msg, 0)
        button_b = self._button(msg, 1)
        button_x = self._button(msg, 2)
        button_y = self._button(msg, 3)
        button_select = self._button(msg, 6)
        button_start = self._button(msg, 7)
        left_stick_x = self._axis(msg, 0)
        left_stick_y = self._axis(msg, 1)
        right_stick_x = self._axis(msg, 3)
        right_stick_y = -self._axis(msg, 4)
        triggers_axis = (self._axis(msg, 2, 1.0) - self._axis(msg, 5, 1.0)) / 2.0

        if button_select:
            self._activate_software_kill()
            return

        if self._is_mode_switch_requested(button_start):
            return

        if self._current_mode == ControlMode.MANUAL_ASSISTED:
            self._handle_lqr_profile_buttons(
                standstill_button=button_a,
                forward_button=button_y,
                turning_button=button_b
            )
            if self.software_kill_active:
                return
            if self._has_recent_playback_target():
                return
            self._manual_assisted_gamepad_update(
                forward_cmd=-left_stick_y,
                yaw_cmd=left_stick_x,
                vertical_cmd=-triggers_axis,
                roll_cmd=0.0,
                pitch_cmd=0.0
            )
            return
        
        if self._current_mode != ControlMode.MANUAL:
            return

        right_stick_y = self._avoid_joystick_dead_zone(right_stick_y)
        right_stick_x = self._avoid_joystick_dead_zone(right_stick_x)
        left_stick_x = self._avoid_joystick_dead_zone(left_stick_x)
        left_stick_y = self._avoid_joystick_dead_zone(left_stick_y)

        # Build requested 6-DOF force vector from joysticks
        self.raw_cmd_vector = [left_stick_y, left_stick_x, triggers_axis, (button_a - button_b), -right_stick_y, right_stick_x]
        
        # Multiply by Thruster Allocation Matrix to determine which motors must spin 
        # to achieve the desired joystick vector.
        raw_thrusts = np.clip(THRUST_ALLOC_MAT.dot(self.raw_cmd_vector) * DEFAULT_MAX_THROTTLE, -DEFAULT_MAX_THROTTLE, DEFAULT_MAX_THROTTLE)
        
        thrust_msg = ThrusterCommand(efforts=raw_thrusts.tolist())
        self.thruster_pub.publish(thrust_msg)

    # ==========================================
    # ACTION SERVER (FLEXBE INTERFACE)
    # ==========================================
    async def control_action_callback(self, goal_handle):
        """
        Handle incoming autonomous navigation commands from state machines.
        
        Because it is `async`, it can safely block and loop while checking 
        `is_target_reached()` without starving the node's executor or blocking 
        the continuous execution of the LQR math in the localization callback.
        
        Args:
            goal_handle: The ROS 2 action server goal handle.
            
        Returns:
            Control.Result: The success or failure state of the navigation goal.
        """
        if self._current_mode != ControlMode.BEHAVIOR or np.isnan(self.current_state[0]):
            self.get_logger().warn("Rejecting Action Goal: Not in BEHAVIOR mode or missing state.")
            goal_handle.abort()
            return Control.Result(success=False)

        incoming_pose = goal_handle.request.target_pose
        
        try:
            # Validate target against the TF tree (handles frames like camera_link -> odom)
            target_pose_in_map = self.tf_buffer.transform(
                incoming_pose, self.global_reference_frame, timeout=Duration(seconds=1.0)
            )
            
            target_roll_enu, target_pitch_enu, target_yaw_enu = self.quaternion_to_euler(
                target_pose_in_map.pose.orientation.x, target_pose_in_map.pose.orientation.y,
                target_pose_in_map.pose.orientation.z, target_pose_in_map.pose.orientation.w
            )
            
            yaw_ned = (math.pi / 2.0) - target_yaw_enu

            # Write the new objective to the shared memory space
            with self.target_state_lock:
                if not np.isnan(self.target_state[0]):
                    # Apply Target ENU -> NED in-place
                    self.target_state[0] = target_pose_in_map.pose.position.y
                    self.target_state[1] = target_pose_in_map.pose.position.x
                    self.target_state[2] = -target_pose_in_map.pose.position.z
                    self.target_state[3] = target_roll_enu
                    self.target_state[4] = -target_pitch_enu
                    self.target_state[5] = (yaw_ned + math.pi) % (2 * math.pi) - math.pi
            
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'TF Error: {ex}')
            goal_handle.abort()
            return Control.Result(success=False)

        # --- Wait For Completion Loop ---
        while not self.is_target_reached():
            
            # Handle Preemption (e.g., FlexBE aborted the behavior)
            if goal_handle.is_cancel_requested:
                with self.target_state_lock:
                    # SAFE CANCELLATION: Instantly copy current array into target array memory.
                    # This tells the LQR "Your target is exactly where you are right now,"
                    # causing the Sub to brake heavily and station-keep safely.
                    self.target_state[:] = self.current_state[:]
                goal_handle.canceled()
                return Control.Result(success=False)
            
            # Yield execution back to the executor so other callbacks can run
            await asyncio.sleep(0.1) 
            
        goal_handle.succeed()
        return Control.Result(success=True)

    # ==========================================
    # HELPER FUNCTIONS
    # ==========================================
    def is_target_reached(self):
        """
        Evaluate if the Sub's current 6-DOF position satisfies the goal tolerances.
        
        Returns:
            bool: True if position and angle errors are within tolerance, False otherwise.
        """
        if np.isnan(self.current_state[0]):
            return False
            
        with self.target_state_lock:
            if np.isnan(self.target_state[0]):
                return False
            target_np = self.target_state.copy()
            
        # Fast array math to get absolute error magnitudes
        lqr_error = target_np - self.current_state
        lqr_error[3:6] = self.wrap_angles_to_pi(lqr_error[3:6])
        
        position_errors = lqr_error[0:3] 
        angle_errors = lqr_error[3:6]    
        
        pos_tolerance = 0.5 # 0.5 meters
        angle_tolerance = math.radians(10)  # 10 degrees
        
        if np.any(np.abs(position_errors) > pos_tolerance) or np.any(np.abs(angle_errors) > angle_tolerance):
            return False
        return True

    def _set_manual_assisted_parameter(self, param):
        """Update one manual assisted tuning parameter at runtime."""
        if param.name == 'manual_assisted.depth_limits_enabled':
            if bool(param.value) and self.manual_assisted_min_depth_target > self.manual_assisted_max_depth_target:
                return SetParametersResult(
                    successful=False,
                    reason='manual_assisted.min_depth_target must be <= max_depth_target'
                )
            self.manual_assisted_depth_limits_enabled = bool(param.value)
            if self.manual_assisted_depth_limits_enabled:
                self._clamp_manual_assisted_depth_targets()
            self.get_logger().warn(
                f"MANUAL_ASSISTED depth limits "
                f"{'enabled' if self.manual_assisted_depth_limits_enabled else 'disabled'}."
            )
            return SetParametersResult(successful=True)

        try:
            value = float(param.value)
        except (TypeError, ValueError):
            return SetParametersResult(successful=False, reason=f'{param.name} must be numeric')

        if param.name == 'manual_assisted.default_depth_target':
            self.manual_assisted_default_depth_target = value
            self.manual_assisted_base_depth_target = value
            self.manual_assisted_depth_target = value
        elif param.name == 'manual_assisted.min_depth_target':
            if self.manual_assisted_depth_limits_enabled and value > self.manual_assisted_max_depth_target:
                return SetParametersResult(
                    successful=False,
                    reason='manual_assisted.min_depth_target must be <= max_depth_target'
                )
            self.manual_assisted_min_depth_target = value
        elif param.name == 'manual_assisted.max_depth_target':
            if self.manual_assisted_depth_limits_enabled and value < self.manual_assisted_min_depth_target:
                return SetParametersResult(
                    successful=False,
                    reason='manual_assisted.max_depth_target must be >= min_depth_target'
                )
            self.manual_assisted_max_depth_target = value
        elif param.name == 'manual_assisted.depth_step':
            if value <= 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.depth_step must be > 0.0')
            self.manual_assisted_depth_step = value
        elif param.name == 'manual_assisted.max_depth_offset':
            if value < 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.max_depth_offset must be >= 0.0')
            self.manual_assisted_max_depth_offset = value
        elif param.name == 'manual_assisted.max_carrot_distance':
            if value <= 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.max_carrot_distance must be > 0.0')
            self.manual_assisted_max_carrot_distance = value
        elif param.name == 'manual_assisted.max_carrot_speed':
            if value <= 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.max_carrot_speed must be > 0.0')
            self.manual_assisted_max_carrot_speed = value
        elif param.name == 'manual_assisted.max_forward_speed':
            if value < 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.max_forward_speed must be >= 0.0')
            self.manual_assisted_max_forward_speed = value
        elif param.name == 'manual_assisted.max_strafe_speed':
            if value < 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.max_strafe_speed must be >= 0.0')
            self.manual_assisted_max_strafe_speed = value
        elif param.name == 'manual_assisted.max_yaw_rate':
            if value < 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.max_yaw_rate must be >= 0.0')
            self.manual_assisted_max_yaw_rate = value
        elif param.name == 'manual_assisted.max_roll_deg':
            if value < 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.max_roll_deg must be >= 0.0')
            self.manual_assisted_max_roll = math.radians(value)
        elif param.name == 'manual_assisted.max_pitch_deg':
            if value < 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.max_pitch_deg must be >= 0.0')
            self.manual_assisted_max_pitch = math.radians(value)
        elif param.name == 'manual_assisted.joystick_curve':
            if value < 1.0:
                return SetParametersResult(successful=False, reason='manual_assisted.joystick_curve must be >= 1.0')
            self.manual_assisted_joystick_curve = value
        elif param.name == 'manual_assisted.gamepad_timeout_sec':
            if value <= 0.0:
                return SetParametersResult(successful=False, reason='manual_assisted.gamepad_timeout_sec must be > 0.0')
            self.manual_assisted_gamepad_timeout = value
        else:
            return SetParametersResult(successful=False, reason=f'Unhandled parameter: {param.name}')

        if (
            self.manual_assisted_depth_limits_enabled
            and self.manual_assisted_min_depth_target > self.manual_assisted_max_depth_target
        ):
            return SetParametersResult(
                successful=False,
                reason='manual_assisted.min_depth_target must be <= max_depth_target'
            )
        if self.manual_assisted_depth_limits_enabled:
            self._clamp_manual_assisted_depth_targets()
        self._limit_manual_assisted_carrot()
        return SetParametersResult(successful=True)

    def _validate_manual_assisted_parameter(self, param):
        """Validate one manual-assisted value without changing controller state."""
        if param.name == 'manual_assisted.depth_limits_enabled':
            return SetParametersResult(successful=True)

        try:
            value = float(param.value)
        except (TypeError, ValueError):
            return SetParametersResult(
                successful=False, reason=f'{param.name} must be numeric'
            )

        limits = {
            'manual_assisted.depth_step': (0.0, False, 'must be > 0.0'),
            'manual_assisted.max_depth_offset': (0.0, True, 'must be >= 0.0'),
            'manual_assisted.max_carrot_distance': (0.0, False, 'must be > 0.0'),
            'manual_assisted.max_carrot_speed': (0.0, False, 'must be > 0.0'),
            'manual_assisted.max_forward_speed': (0.0, True, 'must be >= 0.0'),
            'manual_assisted.max_strafe_speed': (0.0, True, 'must be >= 0.0'),
            'manual_assisted.max_yaw_rate': (0.0, True, 'must be >= 0.0'),
            'manual_assisted.max_roll_deg': (0.0, True, 'must be >= 0.0'),
            'manual_assisted.max_pitch_deg': (0.0, True, 'must be >= 0.0'),
            'manual_assisted.joystick_curve': (1.0, True, 'must be >= 1.0'),
            'manual_assisted.gamepad_timeout_sec': (0.0, False, 'must be > 0.0'),
        }
        if param.name in (
            'manual_assisted.default_depth_target',
            'manual_assisted.min_depth_target',
            'manual_assisted.max_depth_target'
        ):
            return SetParametersResult(successful=True)
        if param.name not in limits:
            return SetParametersResult(
                successful=False, reason=f'Unhandled parameter: {param.name}'
            )

        boundary, inclusive, message = limits[param.name]
        valid = value >= boundary if inclusive else value > boundary
        if not valid:
            return SetParametersResult(
                successful=False, reason=f'{param.name} {message}'
            )
        return SetParametersResult(successful=True)

    def _clamp_manual_assisted_depth_targets(self):
        self.manual_assisted_depth_target = self._clamp(
            self.manual_assisted_depth_target,
            self.manual_assisted_min_depth_target,
            self.manual_assisted_max_depth_target
        )
        self.manual_assisted_base_depth_target = self._clamp(
            self.manual_assisted_base_depth_target,
            self.manual_assisted_min_depth_target,
            self.manual_assisted_max_depth_target
        )

    def _activate_software_kill(self):
        """Disable PWM and zero thrusters until Start re-arms manual assisted mode."""
        if not self.software_kill_active:
            self.get_logger().error("SOFTWARE KILL ACTIVATED: disabling PWM and zeroing thrusters.")
        self.software_kill_active = True
        self._publish_software_kill_state(True)
        self._publish_zero_thrust()

    def _clear_software_kill(self):
        """Re-enable PWM after an explicit Start command."""
        if self.software_kill_active:
            self.get_logger().warn("SOFTWARE KILL CLEARED: enabling PWM.")
        self.software_kill_active = False
        self._publish_software_kill_state(False)

    def _publish_software_kill_state(self, disabled):
        disable_msg = Bool()
        disable_msg.data = bool(disabled)
        self.disable_pwm_pub.publish(disable_msg)

    def _publish_zero_thrust(self):
        self.thruster_pub.publish(ThrusterCommand(efforts=[0.0] * 8))

    def _initialize_manual_assisted_targets(self):
        """Capture the current pose as the assisted piloting hold target."""
        self.manual_assisted_carrot_body[:] = 0.0
        self.manual_assisted_last_update_time = self._now_seconds()
        self.manual_assisted_last_gamepad_time = self.manual_assisted_last_update_time

        if np.isnan(self.current_state[0]):
            self.manual_assisted_base_depth_target = self.manual_assisted_default_depth_target
            if self.manual_assisted_depth_limits_enabled:
                self.manual_assisted_base_depth_target = self._clamp(
                    self.manual_assisted_base_depth_target,
                    self.manual_assisted_min_depth_target,
                    self.manual_assisted_max_depth_target
                )
            self.manual_assisted_depth_target = self.manual_assisted_base_depth_target
            self.manual_assisted_yaw_target = 0.0
            self.get_logger().warn(
                "Cannot arm MANUAL_ASSISTED: odometry is not available.",
                throttle_duration_sec=2.0
            )
            return False

        self.manual_assisted_base_depth_target = self.manual_assisted_default_depth_target
        if self.manual_assisted_depth_limits_enabled:
            self.manual_assisted_base_depth_target = self._clamp(
                self.manual_assisted_base_depth_target,
                self.manual_assisted_min_depth_target,
                self.manual_assisted_max_depth_target
            )
        self.manual_assisted_depth_target = self.manual_assisted_base_depth_target
        self.manual_assisted_yaw_target = self.current_state[5]

        if hasattr(self, 'target_state_lock'):
            with self.target_state_lock:
                self.target_state[0:6] = self.current_state[0:6]
                self.target_state[2] = self.manual_assisted_depth_target
                self.target_state[3] = 0.0
                self.target_state[4] = 0.0
                self.target_state[5] = self.manual_assisted_yaw_target
                self.target_state[6:12] = 0.0

        self.get_logger().info(
            f"MANUAL_ASSISTED armed: depth={self.manual_assisted_depth_target:.2f} m, "
            f"yaw={math.degrees(self.manual_assisted_yaw_target):.1f} deg"
        )
        return True

    def _manual_assisted_gamepad_update(
        self,
        forward_cmd,
        yaw_cmd,
        vertical_cmd,
        roll_cmd,
        pitch_cmd
    ):
        """
        Convert gamepad intent into a short, bounded LQR target.
        """
        if np.isnan(self.current_state[0]):
            self.get_logger().warn(
                "Ignoring MANUAL_ASSISTED gamepad command: no odometry yet.",
                throttle_duration_sec=1.0
            )
            return

        now = self._now_seconds()
        if self.manual_assisted_last_update_time is None:
            self.manual_assisted_last_update_time = now
        dt = self._clamp(now - self.manual_assisted_last_update_time, 0.0, 0.1)
        self.manual_assisted_last_update_time = now
        self.manual_assisted_last_gamepad_time = now

        # Pool-test POV mapping: left stick Y moves a short carrot forward/back,
        # left stick X rotates the yaw target. The LQR still drives the thrusters.
        forward = self._shape_joystick(forward_cmd)
        yaw = self._shape_joystick(yaw_cmd)
        depth_offset_cmd = self._shape_joystick(vertical_cmd)

        self.manual_assisted_depth_target = (
            self.manual_assisted_base_depth_target
            + depth_offset_cmd * self.manual_assisted_max_depth_offset
        )
        if self.manual_assisted_depth_limits_enabled:
            self.manual_assisted_depth_target = self._clamp(
                self.manual_assisted_depth_target,
                self.manual_assisted_min_depth_target,
                self.manual_assisted_max_depth_target
            )

        if forward == 0.0:
            self.manual_assisted_carrot_body[:] = 0.0
        else:
            self.manual_assisted_carrot_body[0] += forward * self.manual_assisted_max_carrot_speed * dt
            self.manual_assisted_carrot_body[1] = 0.0
            self._limit_manual_assisted_carrot()

        self.manual_assisted_yaw_target = self.wrap_angles_to_pi(np.array([
            self.manual_assisted_yaw_target + yaw * self.manual_assisted_max_yaw_rate * dt
        ]))[0]

        with self.target_state_lock:
            self._write_manual_assisted_target_state(
                forward_cmd=forward,
                strafe_cmd=0.0,
                yaw_cmd=yaw,
                roll_target=roll_cmd * self.manual_assisted_max_roll,
                pitch_target=pitch_cmd * self.manual_assisted_max_pitch,
                vertical_cmd=depth_offset_cmd
            )

    def _write_manual_assisted_target_state(
        self,
        forward_cmd=0.0,
        strafe_cmd=0.0,
        yaw_cmd=0.0,
        roll_target=0.0,
        pitch_target=0.0,
        vertical_cmd=0.0
    ):
        """Write the current manual assisted carrot into target_state. Caller holds lock."""
        yaw = self.current_state[5]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        forward_offset = self.manual_assisted_carrot_body[0]
        right_offset = self.manual_assisted_carrot_body[1]

        north_offset = forward_offset * cos_yaw - right_offset * sin_yaw
        east_offset = forward_offset * sin_yaw + right_offset * cos_yaw

        self.target_state[0] = self.current_state[0] + north_offset
        self.target_state[1] = self.current_state[1] + east_offset
        self.target_state[2] = self.manual_assisted_depth_target
        self.target_state[3] = roll_target
        self.target_state[4] = pitch_target
        self.target_state[5] = self.manual_assisted_yaw_target
        self.target_state[6] = forward_cmd * self.manual_assisted_max_forward_speed
        self.target_state[7] = strafe_cmd * self.manual_assisted_max_strafe_speed
        self.target_state[8] = 0.0
        self.target_state[9] = 0.0
        self.target_state[10] = 0.0
        self.target_state[11] = yaw_cmd * self.manual_assisted_max_yaw_rate

        dbg = Float64MultiArray()
        dbg.data = [
            self.manual_assisted_carrot_body[0],
            self.manual_assisted_carrot_body[1],
            self.manual_assisted_depth_target,
            self.manual_assisted_yaw_target,
            self.target_state[0],
            self.target_state[1],
            self.target_state[2],
            self.target_state[5],
            forward_cmd,
            strafe_cmd,
            yaw_cmd,
            vertical_cmd,
            roll_target,
            pitch_target
        ]
        self.manual_assisted_debug_pub.publish(dbg)

    def _publish_manual_assisted_target(self, target_state, stamp):
        """Publish the LQR carrot in ROS ENU/FLU coordinates for recording."""
        if np.isnan(target_state[0]):
            return

        roll_flu = target_state[3]
        pitch_flu = -target_state[4]
        yaw_flu = (math.pi / 2.0) - target_state[5]
        yaw_flu = (yaw_flu + math.pi) % (2 * math.pi) - math.pi
        qx, qy, qz, qw = self.euler_to_quaternion(roll_flu, pitch_flu, yaw_flu)

        msg = ControlTarget()
        msg.header.stamp = stamp
        msg.header.frame_id = self.global_reference_frame
        msg.source_mode = 'manual_assisted'
        msg.pose.position.x = float(target_state[1])
        msg.pose.position.y = float(target_state[0])
        msg.pose.position.z = float(-target_state[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        msg.twist.linear.x = float(target_state[6])
        msg.twist.linear.y = float(-target_state[7])
        msg.twist.linear.z = float(-target_state[8])
        msg.twist.angular.x = float(target_state[9])
        msg.twist.angular.y = float(-target_state[10])
        msg.twist.angular.z = float(-target_state[11])
        msg.carrot_forward = float(self.manual_assisted_carrot_body[0])
        msg.carrot_right = float(-self.manual_assisted_carrot_body[1])
        self.control_target_pub.publish(msg)

    def _apply_manual_assisted_timeout(self):
        """Hold position if gamepad updates stop. Caller holds target_state_lock."""
        now = self._now_seconds()
        last = self.manual_assisted_last_gamepad_time
        if last is not None and (now - last) <= self.manual_assisted_gamepad_timeout:
            return

        self.manual_assisted_carrot_body[:] = 0.0
        if not np.isnan(self.current_state[0]):
            self.manual_assisted_yaw_target = self.current_state[5]
            self.target_state[0:6] = self.current_state[0:6]
            self.target_state[2] = self.manual_assisted_depth_target
            self.target_state[3] = 0.0
            self.target_state[4] = 0.0
            self.target_state[5] = self.manual_assisted_yaw_target
            self.target_state[6:12] = 0.0

    def _limit_manual_assisted_carrot(self):
        norm = float(np.linalg.norm(self.manual_assisted_carrot_body))
        if norm > self.manual_assisted_max_carrot_distance:
            self.manual_assisted_carrot_body *= self.manual_assisted_max_carrot_distance / norm

    def _shape_joystick(self, value):
        value = self._avoid_joystick_dead_zone(float(value))
        if value == 0:
            return 0.0
        return math.copysign(abs(value) ** self.manual_assisted_joystick_curve, value)

    @staticmethod
    def _axis(msg, index, default=0.0):
        return float(msg.axes[index]) if index < len(msg.axes) else float(default)

    @staticmethod
    def _button(msg, index):
        return int(msg.buttons[index]) if index < len(msg.buttons) else 0

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _handle_lqr_profile_buttons(self, standstill_button, forward_button, turning_button):
        profile_buttons = {
            'standstill': bool(standstill_button),
            'forward': bool(forward_button),
            'turning': bool(turning_button)
        }
        selected_profile = None
        for profile, pressed in profile_buttons.items():
            if pressed and not self.last_lqr_profile_button_states.get(profile, False):
                selected_profile = profile
                break
        self.last_lqr_profile_button_states = profile_buttons

        if selected_profile is None or selected_profile == self.lqr_profile:
            return

        result = self.set_parameters([Parameter('lqr_profile', value=selected_profile)])
        if result and not result[0].successful:
            self.get_logger().warn(
                f"Could not switch LQR profile to {selected_profile}: {result[0].reason}"
            )
    
    def _is_mode_switch_requested(self, mode_switch_button):
        """
        Intercept a gamepad button press and cycle through the ControlMode enum.
        
        Args:
            mode_switch_button (int): The state of the mapped switch button.
            
        Returns:
            bool: True if a switch occurred, False otherwise.
        """
        retval = False
        if mode_switch_button and not self.last_mode_switch_button_state:
            if self._current_mode != ControlMode.MANUAL_ASSISTED:
                updated_param = Parameter('control_mode', value='manual_assisted')
                self.set_parameters([updated_param])
            if self._initialize_manual_assisted_targets():
                self._clear_software_kill()
            retval = True
        self.last_mode_switch_button_state = mode_switch_button
        return retval
    
    def _avoid_joystick_dead_zone(self, joystick):
        """
        Ignore minor joystick drift to prevent the sub from slowly creeping.
        
        Args:
            joystick (float): The raw analog axis reading.
            
        Returns:
            float: 0.0 if within dead zone, otherwise the raw reading.
        """
        return 0 if (-DEFAULT_JOY_DEAD_ZONE <= joystick <= DEFAULT_JOY_DEAD_ZONE) else joystick

    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert standard ROS roll, pitch and yaw angles to a quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        )

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        Convert a ROS 2 geometry_msgs Quaternion into standard Euler angles.
        
        Args:
            x (float): Quaternion X.
            y (float): Quaternion Y.
            z (float): Quaternion Z.
            w (float): Quaternion W.
            
        Returns:
            tuple: (roll, pitch, yaw) in radians.
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    
    # Run the node with 4 threads so Action Servers and topic Subscriptions 
    # callbacks can process concurrently without blocking each other.
    executor = MultiThreadedExecutor(num_threads=4) 
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
