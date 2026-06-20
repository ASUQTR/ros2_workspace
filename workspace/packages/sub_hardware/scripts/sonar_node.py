#!/usr/bin/env python3
import numpy as np
import math

import rclpy
import brping
from rclpy.node import Node

from sub_interfaces.srv import SonarScan
from sensor_msgs.msg import MultiEchoLaserScan, LaserEcho
from std_msgs.msg import Header
from typing import cast

# ----------------------------
# Paramètres sonar
# ----------------------------
SAMPLE_PERIOD = 1200

TRANSMIT_DURATION = 20
GAIN_SETTING = 1

SPEED_OF_SOUND = 1480.0 # m/s

class SonarNode(Node):
    device: brping.Ping360

    config_baudrate: int
    config_speed_of_sound: float
    config_port: str
    
    def __init__(self):
        super().__init__('sonar_node')
        self.get_logger().info('Sonar node has been started.')

        self.config_baudrate = cast(int, self.get_parameter('baudrate').value)
        self.config_speed_of_sound = cast(float, self.get_parameter('speed_of_sound').value)
        self.config_port = cast(str, self.get_parameter('port').value)

        self.device = brping.Ping360()
        self.device.connect_serial(self.config_port, self.config_baudrate)

        initialized = self.device.initialize()
        if not initialized:
            print("Warning: initialize() failed, attempting direct configuration...")
            # If initialization fails, we'll configure directly without reading device state first
            # Use control_transducer with reasonable defaults
            self.device._mode = 1
            self.device._gain_setting = GAIN_SETTING
            self.device._transmit_duration = TRANSMIT_DURATION
            self.device._sample_period = SAMPLE_PERIOD
            self.device._transmit_frequency = 750
        
        # Rate at 10Hz (0.1 seconds)
        self.rate = self.create_rate(10)
        self.scan_service = self.create_service(
            SonarScan,
            'scan',
            self.scan
        )

    def calculate_samples(self, range: int) -> int:
        # Calculate the number of samples needed for the given range
        # Time for sound to travel to the target and back
        time_for_echo = (2 * range) / self.config_speed_of_sound  # seconds
        # Number of samples needed based on sample period
        num_samples = int(time_for_echo * 1e6 / SAMPLE_PERIOD)  # Convert seconds to microseconds
        return min(num_samples, 1200)  # Cap at NUM_SAMPLES

    def sample_to_distance(self, sample_index, sample_period, speed_of_sound=SPEED_OF_SOUND):
        """
        Conversion basée sur la doc Ping360.

        sample_period est en unités de 25 ns.
        """
        dt = sample_period * 25e-9

        distance = (
            sample_index
            * dt
            * speed_of_sound
            / 2.0
        )

        return distance

    def estimate_wall_distance(self, data):

        samples = np.frombuffer(data, dtype=np.uint8).copy()

        # Aggressive ringdown rejection to skip transducer saturation
        ringdown_samples = 250
        samples[:ringdown_samples] = 0

        # Use a more aggressive threshold to detect only real echoes,
        # not just the saturation tail. Calculate threshold relative to
        # the data in the tail (which should be noise/weaker echoes).
        tail_data = samples[ringdown_samples:] if len(samples) > ringdown_samples else samples
        
        if len(tail_data) > 0:
            noise_floor = np.percentile(tail_data, 25)  # 25th percentile as noise floor
            threshold = noise_floor + 80  # Detect signals 80 units above noise
        else:
            threshold = 100
        
        # Find first sample above threshold
        above_threshold = np.where(samples > threshold)[0]
        
        if len(above_threshold) > 0:
            idx = above_threshold[0]
            strength = samples[idx]
        else:
            # Fallback: use argmax on tail
            idx = np.argmax(samples[ringdown_samples:]) + ringdown_samples if len(samples) > ringdown_samples else np.argmax(samples)
            strength = samples[idx]

        return idx, strength

    def scan(self, request, response):
        num_samples = self.calculate_samples(request.range)
        self.device._number_of_samples = num_samples

        # Initialize the MultiEchoLaserScan message
        scan_msg = MultiEchoLaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'sonar_link'
        
        # Sonar configuration
        start_angle = request.start_angle
        end_angle = request.end_angle
        angle_increment = 1.0  # degrees
        
        scan_msg.angle_min = math.radians(start_angle)
        scan_msg.angle_max = math.radians(end_angle)
        scan_msg.angle_increment = math.radians(angle_increment)
        scan_msg.time_increment = 0.0
        scan_msg.range_min = 0.0
        scan_msg.range_max = float(request.range)
        
        # Collect scan data
        ranges = []
        intensities = []
        
        for angle in range(start_angle, end_angle + 1, 1):
            angle_grads = int(angle * 400 / 360)
            msg = self.device.transmitAngle(angle_grads)
            
            if msg is None:
                self.get_logger().error(f"Failed to transmit at angle {angle} degrees.")
                # Add empty echo for this angle
                ranges.append(LaserEcho(ranges=[]))
                intensities.append(0.0)
                continue
            
            idx, strength = self.estimate_wall_distance(msg)
            distance = self.sample_to_distance(idx, SAMPLE_PERIOD, self.config_speed_of_sound)
            
            # Create LaserEcho with detected distance
            laser_echo = LaserEcho(ranges=[distance])
            ranges.append(laser_echo)
            intensities.append(float(strength))
            
            # Sleep 100 ms
            self.rate.sleep()
        
        # Populate response
        scan_msg.ranges = ranges
        scan_msg.intensities = intensities
        response.data = scan_msg
        
        return response
        


def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    rclpy.spin(sonar_node)
    sonar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()