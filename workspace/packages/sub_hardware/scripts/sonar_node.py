#!/usr/bin/env python3
import math

import brping
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserEcho, MultiEchoLaserScan
from std_msgs.msg import Header
from typing import cast

from sub_interfaces.srv import SonarScan

# ----------------------------
# Sonar hardware constants
# ----------------------------
# SAMPLE_PERIOD: duration of each sample window, in units of 25 ns.
# 1200 × 25 ns = 30 µs per sample.
SAMPLE_PERIOD = 1200

TRANSMIT_DURATION = 20
GAIN_SETTING = 1

SPEED_OF_SOUND = 1480.0  # m/s


class SonarNode(Node):
    device: brping.Ping360

    config_baudrate: int
    config_speed_of_sound: float
    config_port: str

    def __init__(self):
        super().__init__('sonar_node')
        self.get_logger().info('Sonar node has been started.')

        # BUG FIX: declare parameters before reading them so ROS2 does not throw
        # ParameterNotDeclaredException on startup.
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('speed_of_sound', SPEED_OF_SOUND)
        self.declare_parameter('port', '/dev/ttyUSB0')

        self.config_baudrate = cast(int, self.get_parameter('baudrate').value)
        self.config_speed_of_sound = cast(float, self.get_parameter('speed_of_sound').value)
        self.config_port = cast(str, self.get_parameter('port').value)

        self.device = brping.Ping360()
        self.device.connect_serial(self.config_port, self.config_baudrate)

        initialized = self.device.initialize()
        if not initialized:
            self.get_logger().warn(
                'initialize() failed; applying direct configuration defaults.'
            )
            self.device._mode = 1
            self.device._gain_setting = GAIN_SETTING
            self.device._transmit_duration = TRANSMIT_DURATION
            self.device._sample_period = SAMPLE_PERIOD
            self.device._transmit_frequency = 750

        # Rate limiter: 10 Hz = 100 ms between pings.
        # The Ping360 needs ~100 ms per transmit/receive cycle.
        self.rate = self.create_rate(10)
        self.scan_service = self.create_service(
            SonarScan,
            '/sonar/scan',
            self.scan,
        )

    # ------------------------------------------------------------------
    # Range / sample conversion helpers
    # ------------------------------------------------------------------

    def calculate_samples(self, range_m: int) -> int:
        """Return number of samples required to cover range_m meters.

        The round-trip travel time for an echo at distance r is 2r/c.
        Dividing by the sample window (SAMPLE_PERIOD × 25 ns) gives the
        sample count.  Capped at 1200 (Ping360 hardware maximum).
        """
        time_for_echo = (2 * range_m) / self.config_speed_of_sound  # seconds
        num_samples = int(time_for_echo * 1e6 / SAMPLE_PERIOD)       # µs / (µs per sample)
        return min(num_samples, 1200)

    def sample_to_distance(self, sample_index: int, sample_period: int) -> float:
        """Convert a sample index to distance in meters.

        The Ping360 firmware counts samples in windows of (sample_period × 25 ns).
        Round-trip time for sample i:  t = i × sample_period × 25 ns
        One-way distance:              d = (c × t) / 2

            d = sample_index × sample_period × 25e-9 × c / 2
        """
        dt = sample_period * 25e-9          # window duration in seconds
        return sample_index * dt * self.config_speed_of_sound / 2.0

    # ------------------------------------------------------------------
    # Amplitude peak detection
    # ------------------------------------------------------------------

    def estimate_wall_distance(self, raw_data: bytes):
        """Return (sample_index, amplitude) for the first strong echo.

        1. Zero the first 250 samples (transducer ringdown / saturation zone).
        2. Estimate noise floor as the 25th percentile of the remaining samples.
        3. Detect the first sample that exceeds noise_floor + 80.
        4. Fall back to argmax of the tail if nothing clears the threshold.
        """
        samples = np.frombuffer(raw_data, dtype=np.uint8).copy()

        ringdown_samples = 250
        samples[:ringdown_samples] = 0

        tail = samples[ringdown_samples:] if len(samples) > ringdown_samples else samples

        if len(tail) > 0:
            noise_floor = np.percentile(tail, 25)
            threshold = noise_floor + 80
        else:
            threshold = 100

        above = np.where(samples > threshold)[0]
        if len(above) > 0:
            idx = above[0]
        else:
            # No threshold crossing — use argmax of tail as fallback
            idx = (np.argmax(tail) + ringdown_samples
                   if len(samples) > ringdown_samples else np.argmax(samples))

        return idx, int(samples[idx])

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------

    def scan(self, request, response):
        """Perform a sector scan and return a MultiEchoLaserScan.

        Request fields (SonarScan.srv):
            start_angle   uint16  start of sector in degrees (0–359)
            stop_angle    uint16  end of sector in degrees (0–359)
            desired_range uint16  maximum range in meters

        If start_angle > stop_angle the scan wraps through 0°
        (e.g. start=345, stop=15 scans 345→359 then 0→15).

        Each beam in the returned MultiEchoLaserScan has exactly one
        LaserEcho in ranges (the detected distance) and one in intensities
        (the echo amplitude, 0–255).
        """
        # BUG FIX (field names): srv uses stop_angle and desired_range,
        # not end_angle and range.
        start_deg = int(request.start_angle)
        stop_deg = int(request.stop_angle)
        range_m = int(request.desired_range)

        num_samples = self.calculate_samples(range_m)
        self.device._number_of_samples = num_samples

        # Build the ordered list of integer degree values to scan.
        # If start <= stop: straightforward [start, ..., stop]
        # If start > stop:  wrap through 0°: [start, ..., 359, 0, ..., stop]
        if start_deg <= stop_deg:
            angles_deg = list(range(start_deg, stop_deg + 1))
        else:
            angles_deg = list(range(start_deg, 360)) + list(range(0, stop_deg + 1))

        angle_increment_deg = 1.0

        scan_msg = MultiEchoLaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'sonar_link'

        # angle_min is the first degree in sequential order.
        # For wrapped scans the angles monotonically increase beyond 360°
        # (e.g. 345, 346, …, 360, 361, …, 375) so that
        #   angle_at_index_i = angle_min + i × angle_increment
        # remains valid and trig functions (which are 2π-periodic) give
        # the correct Cartesian projections in the localization layer.
        scan_msg.angle_min = math.radians(angles_deg[0])
        scan_msg.angle_max = math.radians(
            angles_deg[0] + len(angles_deg) - 1  # monotonically increasing
        )
        scan_msg.angle_increment = math.radians(angle_increment_deg)
        scan_msg.time_increment = 0.0
        scan_msg.range_min = 0.0
        scan_msg.range_max = float(range_m)

        ranges = []
        intensities = []

        for angle_deg in angles_deg:
            # Ping360 uses gradians (400 per full revolution).
            # Convert from degrees: grads = degrees × 400 / 360
            angle_grads = int(angle_deg * 400 / 360)
            msg = self.device.transmitAngle(angle_grads)

            if msg is None:
                self.get_logger().error(f'transmitAngle failed at {angle_deg}°')
                ranges.append(LaserEcho(ranges=[0.0]))
                intensities.append(LaserEcho(ranges=[0.0]))
                continue

            idx, strength = self.estimate_wall_distance(msg.data)
            distance = self.sample_to_distance(idx, SAMPLE_PERIOD)

            # BUG FIX: MultiEchoLaserScan.intensities expects LaserEcho[], not float[].
            ranges.append(LaserEcho(ranges=[distance]))
            intensities.append(LaserEcho(ranges=[float(strength)]))

            self.rate.sleep()

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
