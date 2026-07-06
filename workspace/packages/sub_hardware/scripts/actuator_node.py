#!/usr/bin/env python3
"""
================================================================================
ASUQTR Submarine's ROS 2 Actuator Node
================================================================================

This node acts as the primary hardware interface between the ROS 2 network and
the physical actuators (thrusters, grippers, lights, and torpedoes) on the Submarine.
It translates logical commands (e.g., Newtons of force) into hardware-level PWM
signals sent over the I2C bus to a PCA9685 PWM driver.

Design Highlights:
- I2C Thread Safety: Uses a MutuallyExclusiveCallbackGroup to prevent I2C bus collisions
  when multiple actuator topics receive commands concurrently under a MultiThreadedExecutor.
- Non-Linear Thrust Mapping: Uses NumPy to interpolate requested forces against
  empirical thruster data, accounting for deadbands and asymmetric thrust curves.
- Dynamic Configuration: Supports live updates to the thruster neutral-point offset
  to compensate for electrical ground loops without restarting the node.
- Safety watchdog to disable thrusters if no cmd is sent

ROS 2 Interface:
- Subscriptions:
  * `thruster_cmd` (sub_interfaces/ThrusterCommand): Output forces in newtons to thrusters
  * `gripper` (std_msgs/Float32): command for sub's gripper actuator
  * `subsea_light` (std_msgs/Float32): command for subsea light 
  * `torpedo` (std_msgs/Float32): command for sub's torpedo actuator
  * `kill_switch` (sensor_msgs/Bool): state change of magnetic kill switch on 
"""

# ==========================================
# PYTHON STANDARD & 3RD PARTY IMPORTS
# ==========================================
import time
import numpy as np
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import ContinuousServo, Servo
from busio import I2C
from board import SCL, SDA

# ==========================================
# ROS 2 IMPORTS
# ==========================================
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Float32
from sub_interfaces.msg import ThrusterCommand
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


# Found experimentally with Adafruit's tuning script for PCA9685 to ensure accurate PWM
DEFAULT_REF_CLK_SPEED = 24821760
DEFAULT_PCA9685_ADDR = 0x40
DEFAULT_PCA9685_FREQ = 60


class SER110XLauncher(Servo):
    """
    Hardware interface for Blue Trail Engineering’s SER-110X depth-rated underwater servo.
    
    This operates as a standard positional servo (0 to 180 degrees mapping).
    The pulse width is varied within the range of 850 – 2350 microseconds to control 
    the absolute position of the servo horn.
    """
    MIN_PULSE = 850
    MAX_PULSE = 2350

    def __init__(self, pwm_out):
        super().__init__(pwm_out,
                         min_pulse=self.MIN_PULSE,
                         max_pulse=self.MAX_PULSE)


class T200Thruster(ContinuousServo):
    """
    Hardware interface for Blue Robotics T200 Thrusters.
    
    Unlike standard servos, thruster Electronic Speed Controllers (ESCs) require 
    ContinuousServo logic, where the PWM signal maps to speed/throttle (-1.0 to 1.0) 
    rather than absolute position.
    
    Data Source:
        Performance data derived from the official Blue Robotics T200 Performance Charts.
        Reference: https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/

    Voltage Estimation (15.5V table):
        A 4S LiPo starts at ~16.8V (full charge) and ends near ~14V (depleted). During a
        typical competition mission, the operating voltage sits around 15-16V, with most
        of the run near 15.5V. Rather than using either the raw 14V or 16V dataset alone,
        this table was pre-computed by linearly interpolating between both at 15.5V:

            t = (15.5 - 14.0) / (16.0 - 14.0) = 0.75
            force_15.5V = force_14V + t * (force_16V - force_14V)

        This minimizes the Newton-to-PWM mapping error at the most common operating point.
        Example at 1600µs: 14V table = 4.91N, 16V table = 5.89N, this table = 5.65N.
        If the battery is at exactly 15.5V, the interpolated value is correct by construction
        (assuming thrust scales linearly with voltage between 14V and 16V, which holds well
        for brushless motors). Error grows as voltage deviates from 15.5V during the run.

    Future Improvements:
        1. Voltage-adaptive mapping:
           Replace this static 15.5V table with a runtime interpolation driven by the
           actual battery voltage read from a BMS or voltage sensor on a /battery_voltage
           ROS topic. Both the 14V and 16V datasets are available in the Blue Robotics
           performance charts and can be stored as two static arrays, interpolated at
           runtime using: t = (V - 14.0) / (16.0 - 14.0); force = f14 + t * (f16 - f14).

        2. Per-motor throttle offsets:
           The current thruster_throttle_offset parameter applies one global offset to all
           8 thrusters. In practice, each ESC can have a slightly different neutral point,
           causing some motors to spin at a commanded force of 0N. This should be replaced
           by a per-motor list (e.g. thruster_throttle_offsets: [0.0, 0.0, ..., 0.0] in
           params.yaml) applied individually to each thruster before sending PWM commands.
           Calibration procedure: spin each motor alone at a very low command, observe
           drift, and correct with its individual offset.

        3. ESC PWM range calibration:
           The Blue Robotics Basic ESC supports a one-time PWM range calibration that
           ensures all ESCs interpret 1100-1900µs identically and centers the deadband
           at 1500µs. This must be done in hardware (requires power-cycling the ESC with
           the signal held at max before power-on). Steps:
             a. Cut ESC power.
             b. Hold PWM signal at 1900µs.
             c. Power on ESC — wait for the tone sequence.
             d. Move signal to 1100µs — ESC saves the range permanently.
           This should be done once per ESC after any hardware change, before relying on
           the per-motor offset calibration in improvement #2.
    """
    MIN_PULSE = 1100
    MAX_PULSE = 1900

    # T200 performance at 15.5V — interpolated from 14V and 16V datasets (t=0.75).
    # Forces in Newtons. Deadband: 1468µs to 1532µs (ESC produces no thrust in this range).
    # Array must be monotonically increasing for np.interp to function correctly.
    known_forces_n = np.array([
            -38.58,  # 1100 us — Max Reverse
            -32.30,  # 1150 us
            -25.71,  # 1200 us
            -18.83,  # 1250 us
            -13.63,  # 1300 us
             -8.49,  # 1350 us
             -4.51,  # 1400 us
             -1.11,  # 1450 us
             -0.40,  # 1464 us — last non-zero before deadband
              0.0,   # 1468 us — deadband lower boundary
              0.0,   # 1500 us — center / stopped
              0.0,   # 1532 us — deadband upper boundary
              0.42,  # 1536 us — first non-zero after deadband
              1.13,  # 1548 us
              5.65,  # 1600 us
             10.88,  # 1650 us
             17.19,  # 1700 us
             24.38,  # 1750 us
             32.50,  # 1800 us
             41.56,  # 1850 us
             49.71,  # 1900 us — Max Forward
        ])

    # Corresponding ContinuousServo API commands [-1.0 to 1.0]
    # api_cmd = (PWM_us - 1500) / 400
    known_api_cmds = np.array([
            -1.000,  # 1100 us
            -0.875,  # 1150 us
            -0.750,  # 1200 us
            -0.625,  # 1250 us
            -0.500,  # 1300 us
            -0.375,  # 1350 us
            -0.250,  # 1400 us
            -0.125,  # 1450 us
            -0.090,  # 1464 us
            -0.080,  # 1468 us — deadband lower
             0.000,  # 1500 us — stopped
             0.080,  # 1532 us — deadband upper
             0.090,  # 1536 us
             0.120,  # 1548 us
             0.250,  # 1600 us
             0.375,  # 1650 us
             0.500,  # 1700 us
             0.625,  # 1750 us
             0.750,  # 1800 us
             0.875,  # 1850 us
             1.000,  # 1900 us
        ])

    def __init__(self, pwm_out):
        super().__init__(pwm_out,
                         min_pulse=self.MIN_PULSE,
                         max_pulse=self.MAX_PULSE)


class NewtonSubseaGripper(ContinuousServo):
    """
    Hardware interface for the Blue Robotics Newton Subsea Gripper.
    
    Operates via a ContinuousServo interface where PWM dictates the speed and 
    direction of the open/close mechanism.
    """
    MIN_PULSE = 1100
    MAX_PULSE = 1900
    
    def __init__(self, pwm_out):
        super().__init__(pwm_out,
                         min_pulse=self.MIN_PULSE,
                         max_pulse=self.MAX_PULSE)


class LumenSubseaLight(ContinuousServo):
    """
    Hardware interface for the Blue Robotics
DEFAULT_PCA9685_ADDR = 0x40 Lumen Subsea Light.
    
    Operates via a ContinuousServo interface where the PWM signal maps to 
    brightness intensity rather than position or speed.
    """
    MIN_PULSE = 1100
    MAX_PULSE = 1900

    def __init__(self, pwm_out):
        super().__init__(pwm_out,
                         min_pulse=self.MIN_PULSE,
                         max_pulse=self.MAX_PULSE)


class ActuatorNode(Node):
    """
    ROS 2 Node managing I2C physical actuators on the Submarine.
    """
    def __init__(self):
        super().__init__('actuator_node')
        
        # --- Load ROS2 parameters ---
        self.declare_parameter('pca_ref_clk_speed', DEFAULT_REF_CLK_SPEED)
        self.declare_parameter('thruster_throttle_offset', 0.0) # Used to fix neutral drift
        self.declare_parameter('use_flat_thruster_mapping', False)
        self.declare_parameter('flat_thruster_gain_primary', 0.2)
        self.declare_parameter('flat_thruster_gain_secondary', 0.1)
        self.declare_parameter('enable_thrusters_watchdog', True)
        self.declare_parameter('thrusters_watchdog_timeout_sec', 0.5)

        pca_ref_clk_speed = self.get_parameter('pca_ref_clk_speed').value
        self.thruster_throttle_offset = self.get_parameter('thruster_throttle_offset').value
        self.use_flat_thruster_mapping = bool(self.get_parameter('use_flat_thruster_mapping').value)
        self.flat_thruster_gain_primary = float(self.get_parameter('flat_thruster_gain_primary').value)
        self.flat_thruster_gain_secondary = float(self.get_parameter('flat_thruster_gain_secondary').value)
        self.thrusters_watchdog_enabled = self.get_parameter('enable_thrusters_watchdog').value
        self.thrusters_watchdog_timeout = Duration(
            seconds=float(self.get_parameter('thrusters_watchdog_timeout_sec').value)
        )

        # Bind dynamic ROS2 reconfigure callback to allow live tuning
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- Initialize Hardware ---
        # Initialize I2C bus and PCA9685 driver.
        self.pca = PCA9685(I2C(SCL, SDA), address=DEFAULT_PCA9685_ADDR, reference_clock_speed=pca_ref_clk_speed)
        self.pca.frequency = DEFAULT_PCA9685_FREQ
        self.get_logger().info('PCA9685 initiated! Starting thrusters initialization...')

        # Hardware pin assignments (Refer to ASUQTR Pcb Control V01 Sch)
        thruster_pins = [7, 6, 5, 4, 3, 2, 15, 14]
        gripper_pin = 8
        led_pin = 9
        torpedo_pin = 13

        # Instantiate actuator objects using specific PCA channels
        self.thrusters = [T200Thruster(self.pca.channels[i]) for i in thruster_pins]
        self.gripper = NewtonSubseaGripper(self.pca.channels[gripper_pin])
        self.subsea_light = LumenSubseaLight(self.pca.channels[led_pin])
        self.torpedo = SER110XLauncher(self.pca.channels[torpedo_pin])

        # --- ROS 2 Thread Safety ---
        # The I2C bus is not thread-safe. Because we are using a MultiThreadedExecutor, 
        # concurrent messages on different topics (e.g., a gripper command and a thruster command 
        # arriving simultaneously) could attempt to write to the I2C bus at the exact same time, 
        # resulting in an OSError [Errno 121] Remote I/O error.
        # Grouping all actuator callbacks into this mutually exclusive group forces the 
        # executor to lock the I2C bus and process hardware commands sequentially.
        self.i2c_cb_group = MutuallyExclusiveCallbackGroup()

        # --- Subscriptions & QoS ---
        # 1. Define the MATCHING QoS Profile
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        only_latest_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.thruster_sub = self.create_subscription(
            ThrusterCommand, 'thruster_cmd', self.thrusters_callback, only_latest_qos, callback_group=self.i2c_cb_group)
        
        self.gripper_sub = self.create_subscription(
            Float32, 'gripper', self.gripper_callback, only_latest_qos, callback_group=self.i2c_cb_group)
        
        self.subsea_light_sub = self.create_subscription(
            Float32, 'subsea_light', self.subsea_light_callback, only_latest_qos, callback_group=self.i2c_cb_group)
        
        self.torpedo_sub = self.create_subscription(
            Float32, 'torpedo', self.torpedo_callback, only_latest_qos, callback_group=self.i2c_cb_group)
        
        self.kill_switch_sub = self.create_subscription(
            Bool, 'kill_switch', self.kill_switch_callback, latched_qos, callback_group=self.i2c_cb_group)

        self.disable_pwm_pub = self.create_publisher(Bool, 'disable_pwm', latched_qos)

        # --- Thruster Watchdog Timer for safety---
        # If thruster commands stop arriving, force all thrusters to neutral.
        # Since this callback can operate thrusters, i needs to be in the i2c mutex callback group
        # Set it using seconds for human readability
        self.last_watchdog_kick_time = self.get_clock().now()
        self.thrusters_watchdog_timer = self.create_timer(
            0.05, # Check for thruster cmd timout at 20hz frequency
            self.thrusters_watchdog_callback,
            callback_group=self.i2c_cb_group 
        )

        # --- Initialization State ---
        self.thrusters_need_init = False
        self.init_timer = None        
        self.trigger_thrusters_init()
        

    # ==========================================
    # PARAMETER HANDLING
    # ==========================================

    def parameter_callback(self, params):
        """
        Dynamically handles updates to ROS parameters at runtime.
        Specifically used to tune the thruster neutral offset if the Submarine 
        exhibits drifting while armed but idle.
        """
        successful = True
        reason = ""
        for param in params:
            if param.name == 'thruster_throttle_offset':
                if param.type_ == Parameter.Type.DOUBLE:
                    # Bound the offset to prevent dangerous surges if a typo occurs
                    if -0.15 <= param.value <= 0.15:
                        self.thruster_throttle_offset = param.value
                        self.get_logger().info(f"Successfully updated thruster throttle offset to: {self.thruster_throttle_offset}")
                    else:
                        successful = False
                        reason = "Offset rejected: Value must be safely between -0.15 and 0.15"
                else:
                    successful = False
                    reason = "Type rejected: thruster_throttle_offset must be a float (double)"
            elif param.name == 'use_flat_thruster_mapping':
                if param.type_ == Parameter.Type.BOOL:
                    self.use_flat_thruster_mapping = bool(param.value)
                    self.get_logger().info(f"Flat thruster mapping set to: {self.use_flat_thruster_mapping}")
                else:
                    successful = False
                    reason = "Type rejected: use_flat_thruster_mapping must be a bool"
            elif param.name == 'thrusters_watchdog_timeout_sec':
                if param.type_ == Parameter.Type.DOUBLE:
                    if 0.1 <= param.value <= 5.0:
                        self.thrusters_watchdog_timeout = Duration(seconds=float(param.value))
                        self.get_logger().info(
                            f"Thruster watchdog timeout set to: {float(param.value):.2f} s"
                        )
                    else:
                        successful = False
                        reason = "Timeout rejected: thrusters_watchdog_timeout_sec must be between 0.1 and 5.0"
                else:
                    successful = False
                    reason = "Type rejected: thrusters_watchdog_timeout_sec must be a float (double)"
            elif param.name == 'flat_thruster_gain_primary':
                if param.type_ == Parameter.Type.DOUBLE:
                    if 0.0 <= param.value <= 1.0:
                        self.flat_thruster_gain_primary = float(param.value)
                        self.get_logger().info(f"Flat thruster primary gain set to: {self.flat_thruster_gain_primary}")
                    else:
                        successful = False
                        reason = "Gain rejected: flat_thruster_gain_primary must be between 0.0 and 1.0"
                else:
                    successful = False
                    reason = "Type rejected: flat_thruster_gain_primary must be a float (double)"
            elif param.name == 'flat_thruster_gain_secondary':
                if param.type_ == Parameter.Type.DOUBLE:
                    if 0.0 <= param.value <= 1.0:
                        self.flat_thruster_gain_secondary = float(param.value)
                        self.get_logger().info(f"Flat thruster secondary gain set to: {self.flat_thruster_gain_secondary}")
                    else:
                        successful = False
                        reason = "Gain rejected: flat_thruster_gain_secondary must be between 0.0 and 1.0"
                else:
                    successful = False
                    reason = "Type rejected: flat_thruster_gain_secondary must be a float (double)"

        return SetParametersResult(successful=successful, reason=reason)
    
    # ==========================================
    # ACTUATORS CALLBACKS
    # ==========================================
    def gripper_callback(self, msg):
        """Processes continuous servo commands [-1.0 to 1.0] for the gripper."""
        cmd = msg.data
        if -1.0 <= cmd <= 1.0:
           self.gripper.throttle = cmd
        else:
            self.get_logger().warn(f"Ignored out of bound Gripper command {cmd}! must be [-1.0, 1.0]")

    def torpedo_callback(self, msg):
        """Processes continuous servo commands [-1.0 to 1.0] for the torpedo launcher."""
        cmd = msg.data
        if -1.0 <= cmd <= 1.0:
            self.torpedo.throttle = cmd
        else:
            self.get_logger().warn(f"Ignored out of bound Torpedo command {cmd}! must be [-1.0, 1.0]")

    def subsea_light_callback(self, msg):
        """Processes brightness commands for the subsea lights."""
        cmd = msg.data
        # Servo class uses 'angle' commands from 0 to 180(default temp value)
        # This does not make sense conceptually, but is good enough to command the light for now
        # TODO fix this to make more sense, angle cmd to a light is wrong
        if 0 <= cmd <= 180:
            self.subsea_light.angle = cmd
        else:
            self.get_logger().warn(f"Ignored out of bound Subsea Light command {cmd}! must be [0, 180]")
    
    # ==========================================
    # THRUSTERS CALLBACKS
    # ==========================================
    def thrusters_callback(self, msg):
        """
        Maps requested Newtons of force to PWM throttle signals for the T200 thrusters.
        """
        # Kick thrusters watchdog because new command was received. It has to be kicked
        # even if the thruster are in initialization sequence, to avoid the watchdog trying to
        # send a new command to the thrusters
        self.last_watchdog_kick_time = self.get_clock().now()
        # Drop incoming commands if the ESCs are undergoing their boot/arming sequence
        if self.thrusters_need_init:
            return
        
        efforts = np.clip(np.asarray(msg.efforts, dtype=np.float64), -14.4, 14.4)

        if self.use_flat_thruster_mapping:
            # Temporary debug mode: ignore the non-linear curve and use a flat gain.
            flat_gains = np.array([
                self.flat_thruster_gain_primary,   # 0
                self.flat_thruster_gain_primary,   # 1
                self.flat_thruster_gain_secondary, # 2
                self.flat_thruster_gain_secondary, # 3
                self.flat_thruster_gain_secondary, # 4
                self.flat_thruster_gain_secondary, # 5
                self.flat_thruster_gain_primary,   # 6
                self.flat_thruster_gain_primary,   # 7
            ], dtype=np.float64)
            interpolated_throttles_cmd = efforts * flat_gains
        else:
            # np.interp handles both non-linear mapping and bounds-clipping in one optimized pass.
            interpolated_throttles_cmd = np.interp(
                efforts,
                T200Thruster.known_forces_n,
                T200Thruster.known_api_cmds
            )
        
        # Apply the user-defined electrical neutral offset to fix drifting
        offset_cmds = interpolated_throttles_cmd + self.thruster_throttle_offset
        
        # Hard clip again to ensure the applied offset doesn't push the command past limits
        final_cmds = np.clip(offset_cmds, -1.0, 1.0)
        
        # Transmit to hardware
        for i, throttle in enumerate(final_cmds):
            self.thrusters[i].throttle = throttle

    def thrusters_init_complete_callback(self):
        """
        Fires once the thrusters' initialization timer ends, releasing the lockout on thruster commands.
        """
        self.get_logger().info("Thrusters' Electronic Speed Controller initialization sequence completed! Resuming normal thrusters operation.")
        self.thrusters_need_init = False
        self.init_timer.cancel()

    def kill_switch_callback(self, msg):
        """
        Triggers a safety lockout and re-arms the ESCs when a "killswitch latched" command is received.
        """
        if msg.data and not self.thrusters_need_init:
            self.thrusters_need_init = True         # Lock out incoming commands in the thrusters_callback
            self.trigger_thrusters_init()

    def thrusters_watchdog_callback(self):
        """ 
        Safety Watchdog Monitoring: If no thruster command is received, this can mean that ROS2 nodes
        responsible for thruster force are dead or hanging. This watchdog detects that no thruster command has been
        set in the last safe interval (1 sec, could be changed) and sends 0 commands to thrusters
        to kill them
        """
        # If you need to test this node as standalone, you would want the thrusters' watchdog
        # to be disabled, so it is possible to do so with a config yaml file.
        if self.thrusters_watchdog_enabled:
            now = self.get_clock().now()
            if (now - self.last_watchdog_kick_time) > self.thrusters_watchdog_timeout:
                sec_elapsed = (now - self.last_watchdog_kick_time).nanoseconds / 1e9
                self.get_logger().warn(
                    f"SAFETY FAULT: Last thruster command received {sec_elapsed:.2f} seconds ago. Sending 0N to thrusters!",
                    throttle_duration_sec=5.0 
                )
                for thruster in self.thrusters:
                    thruster.throttle = 0.0


    # ==========================================
    # HELPER FUNCTIONS
    # ==========================================

    def assert_pwm_disable(self):
        """Requests gpio_node to gate PWM off at the PCA9685 /OE pin."""
        self.disable_pwm_pub.publish(Bool(data=True))

    def trigger_thrusters_init(self):
        """
        Forces thrusters to neutral (0-PWM) and enforces a 1-second software lockout.
        
        This non-blocking approach guarantees hardware safety while allowing the 
        MultiThreadedExecutor to process other topics (like lights or grippers) 
        during the 1-second Thrusters' ESC boot sequence.
        """
        self.get_logger().info("Starting T200 Thrusters' ESC initialization sequence...")
        
        # Safely force hardware to neutral state
        for thruster in self.thrusters:
            thruster.throttle = 0.0
            
        # Clear any existing timers (e.g., if kill_switch is spammed)
        if self.init_timer is not None:
            self.init_timer.cancel()
            
        # Spawn a one-shot timer to release the lock. Placed in the same callback group 
        # to guarantee it safely accesses shared state without race conditions.
        self.init_timer = self.create_timer(
            1.0, # seconds
            self.thrusters_init_complete_callback, 
            callback_group=self.i2c_cb_group
        )
                

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    
    executor = MultiThreadedExecutor(num_threads=2) 
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected, shutting down...')
    finally:
        # Join worker threads before zeroing thrusters, so a stale in-flight callback
        # can't write a nonzero throttle after we do.
        executor.shutdown()

        try:
            node.assert_pwm_disable()
            time.sleep(0.2)
        except Exception as e:
            print(f"actuator_node: disable_pwm publish failed during shutdown: {e}")

        for thruster in node.thrusters:
            thruster.throttle = 0.0
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
