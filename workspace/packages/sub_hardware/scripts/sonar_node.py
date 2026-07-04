#!/usr/bin/env python3
import math
import time

import brping
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserEcho, MultiEchoLaserScan
from std_msgs.msg import Header
from typing import cast

from sub_interfaces.msg import SonarScanResult
from sub_interfaces.srv import SonarScan

# ----------------------------
# Sonar hardware constants (BlueRobotics Ping360)
# ----------------------------
# The Ping360 always returns a FIXED number of samples per ping. Range is not
# set by changing that count; it is set by stretching the time between samples
# (sample_period) so the fixed sample window spans the desired round trip, and
# then matching the transmit pulse length (transmit_duration) to that range.
# This mirrors BlueRobotics ping-viewer, which keeps the sample count fixed and
# varies sample_period + transmit_duration.

# Duration of one sample_period tick, in seconds (Ping360 firmware unit).
SAMPLE_PERIOD_TICK_DURATION = 25e-9

# Firmware transmit-duration limits, in microseconds.
FIRMWARE_MIN_TRANSMIT_DURATION = 5
FIRMWARE_MAX_TRANSMIT_DURATION = 500

# Firmware minimum sample_period, in 25 ns ticks (2 µs).
FIRMWARE_MIN_SAMPLE_PERIOD = 80

# Fixed number of samples per ping. Higher = finer range bins
# (range resolution = desired_range / NUMBER_OF_SAMPLES). 1200 is the max.
NUMBER_OF_SAMPLES = 1200

GAIN_SETTING = 1
TRANSMIT_FREQUENCY = 750  # kHz

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
            self.device._transmit_frequency = TRANSMIT_FREQUENCY
            self.device._number_of_samples = NUMBER_OF_SAMPLES
            # sample_period and transmit_duration are derived from the requested
            # range in scan(), so there is no meaningful default to seed here.

        self.scan_service = self.create_service(
            SonarScan,
            '/sonar/scan',
            self.scan,
        )
        self.scan_result_pub = self.create_publisher(
            SonarScanResult, '/service/scan', 10
        )

    # ------------------------------------------------------------------
    # Range / sample conversion helpers
    # ------------------------------------------------------------------

    def compute_sample_period(self, range_m: float) -> int:
        """Sample interval (in 25 ns ticks) so NUMBER_OF_SAMPLES span range_m.

        The round trip to a target at range_m and back takes 2·range_m/c
        seconds. Spreading that time evenly across the fixed NUMBER_OF_SAMPLES
        gives the per-sample interval; dividing by the 25 ns tick converts to
        firmware units. Clamped to the firmware minimum.
        """
        sample_period = int(
            2 * range_m
            / (NUMBER_OF_SAMPLES * self.config_speed_of_sound * SAMPLE_PERIOD_TICK_DURATION)
        )
        return max(FIRMWARE_MIN_SAMPLE_PERIOD, sample_period)

    def transmit_duration_max(self, sample_period: int) -> int:
        """Firmware upper bound on transmit_duration (µs) for a sample_period.

        The firmware limits the pulse to whichever is smaller: 500 µs, or
        64e6 × (sample_period in seconds).
        """
        return min(
            FIRMWARE_MAX_TRANSMIT_DURATION,
            int(sample_period * SAMPLE_PERIOD_TICK_DURATION * 64e6),
        )

    def adjust_transmit_duration(self, range_m: float, sample_period: int) -> int:
        """Transmit pulse length (µs) for a range, per BlueRobotics firmware.

        1. Base pulse: 8000 × one-way range / speed_of_sound.
        2. Widen to at least 2.5 × the sample interval (µs) so the pulse spans
           several samples.
        3. Clamp to [firmware min, firmware max-for-this-sample-period].
        """
        transmit_duration = round(8000.0 * range_m / self.config_speed_of_sound)
        sample_period_us = sample_period * SAMPLE_PERIOD_TICK_DURATION * 1e6
        transmit_duration = max(2.5 * sample_period_us, transmit_duration)
        return int(max(
            FIRMWARE_MIN_TRANSMIT_DURATION,
            min(self.transmit_duration_max(sample_period), transmit_duration),
        ))

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

        ringdown_samples = 50
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

        scan_msg = MultiEchoLaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'sonar_link'
        scan_msg.range_min = 0.0
        scan_msg.range_max = float(range_m)

        try:
            # Set the range by stretching the sample period over a FIXED number
            # of samples, then match the transmit pulse to that range
            # (BlueRobotics ping-viewer approach). sample_period is needed later
            # to convert a sample index back into a distance, so keep it local.
            sample_period = self.compute_sample_period(range_m)
            transmit_duration = self.adjust_transmit_duration(range_m, sample_period)

            self.device._number_of_samples = NUMBER_OF_SAMPLES
            self.device._sample_period = sample_period
            self.device._transmit_duration = transmit_duration
            self.device._gain_setting = GAIN_SETTING
            self.device._transmit_frequency = TRANSMIT_FREQUENCY

            self.get_logger().info(
                f'Scan range={range_m} m -> sample_period={sample_period} ticks, '
                f'transmit_duration={transmit_duration} us, '
                f'samples={NUMBER_OF_SAMPLES}'
            )

            # Build the ordered list of integer degree values to scan.
            # If start <= stop: straightforward [start, ..., stop]
            # If start > stop:  wrap through 0°: [start, ..., 359, 0, ..., stop]
            if start_deg <= stop_deg:
                angles_deg = list(range(start_deg, stop_deg + 1))
            else:
                angles_deg = list(range(start_deg, 360)) + list(range(0, stop_deg + 1))

            angle_increment_deg = 1.0

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

            ranges = []
            intensities = []

            for angle_deg in angles_deg:
                # Ping360 uses gradians (400 per full revolution).
                # Convert from degrees: grads = degrees × 400 / 360
                angle_grads = int(angle_deg * 400 / 360)
                msg = None
                ctr = 0
                CIRCUIT_BREAKER = 400

                while(msg is None):
                    if (ctr >= CIRCUIT_BREAKER):
                        raise RuntimeError(f'transmitAngle failed at {angle_deg}°')

                    msg = self.device.transmitAngle(angle_grads)
                    self.get_logger().debug(f"Retry {ctr} ({angle_deg})")

                    if msg is not None:
                        # msg.data is the echo amplitude array (return strengths).
                        # NOT msg.msg_data, which is the full serialized wire frame
                        # (header + payload + checksum) and contains no usable echoes.
                        idx, strength = self.estimate_wall_distance(msg.data)
                        # Use the sample_period actually configured for this scan,
                        # not a fixed constant — distance-per-sample scales with it.
                        distance = self.sample_to_distance(idx, sample_period)

                        # BUG FIX: MultiEchoLaserScan.intensities expects LaserEcho[], not float[].
                        ranges.append(LaserEcho(echoes=[distance]))
                        intensities.append(LaserEcho(echoes=[float(strength)]))

                    ctr = ctr + 1

                    # Rate limiter: 10 Hz = 100 ms between pings, matching the
                    # Ping360's transmit/receive cycle. Plain sleep (not rclpy Rate):
                    # this callback runs inside the node's own executor thread, and
                    # an rclpy Rate needs that same executor to spin to wake it up —
                    # calling rate.sleep() here deadlocks forever.
                    time.sleep(0.1)

            scan_msg.ranges = ranges
            scan_msg.intensities = intensities

            response.data = scan_msg
            response.success = True
            response.message = ''

        except Exception as exc:
            self.get_logger().error(f'Sonar scan failed: {exc}')
            response.data = scan_msg
            response.success = False
            response.message = str(exc)

        self.get_logger().info(f"{response.data}")
        self._publish_scan_result(response)
        return response

    def _publish_scan_result(self, response):
        """Mirror the service response onto /service/scan for passive subscribers."""
        result = SonarScanResult()
        result.success = response.success
        result.message = response.message
        result.data = response.data
        self.scan_result_pub.publish(result)

def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    rclpy.spin(sonar_node)
    sonar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
