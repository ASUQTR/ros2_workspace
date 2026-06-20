#!/usr/bin/env python3
"""
================================================================================
Sonar-Based Localization Service for AUV in Pool Environment
================================================================================

This node provides an on-demand sonar localization service. Instead of continuous
scanning (which would interfere with fast sensor rates), the mission manager
calls this service when position correction is needed (e.g., every 10 minutes).

Architecture:
- IMU + DVL: Continuous high-frequency state estimation (25 Hz)
- Sonar: On-demand low-frequency position correction
- Mission Manager: Decides when to request sonar position updates

Key Features:
- Service-based interface for mission manager control
- Triangulation from multiple wall distances
- Configurable pool dimensions (x, y)
- Returns pose with confidence covariance
- Multi-angle averaging for robustness
"""

import numpy as np
import math
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sub_interfaces.srv import SonarScan, GetSonarPosition
from std_msgs.msg import Header


class SonarLocalizationNode(Node):
    """
    Provides on-demand sonar localization service for position estimation.
    
    The mission manager calls this service when position correction is needed.
    This avoids continuous scanning which would interfere with other operations.
    """
    
    def __init__(self):
        super().__init__('sonar_localization')
        
        # Declare and retrieve pool dimensions
        self.declare_parameter('pool_width_x', 5.0)
        self.declare_parameter('pool_length_y', 10.0)
        self.declare_parameter('position_variance', 0.25)
        self.declare_parameter('sonar_max_range', 20.0)
        self.declare_parameter('enable_4wall_triangulation', True)
        
        self.pool_x = self.get_parameter('pool_width_x').value
        self.pool_y = self.get_parameter('pool_length_y').value
        self.position_variance = self.get_parameter('position_variance').value
        self.sonar_max_range = self.get_parameter('sonar_max_range').value
        self.enable_4wall_triangulation = self.get_parameter('enable_4wall_triangulation').value
        
        self.get_logger().info(
            f'Sonar Localization Service initialized\n'
            f'  Pool dimensions: {self.pool_x:.2f}m (x) × {self.pool_y:.2f}m (y)\n'
            f'  Position variance: {self.position_variance}\n'
            f'  Mode: On-demand service (not continuous)'
        )
        
        # Sonar scan service client
        self.sonar_client = self.create_client(SonarScan, '/sonar/scan')
        
        # Service server: mission manager calls this for position updates
        self.localization_service = self.create_service(
            GetSonarPosition,
            'get_sonar_position',
            self.sonar_position_callback
        )
        
        self.get_logger().info(
            'Sonar localization service ready. '
            'Mission manager can call: ros2 service call /get_sonar_position sub_control/srv/GetSonarPosition'
        )
    
    def sonar_position_callback(self, request, response):
        """
        Service callback: triggered by mission manager request.
        Scans sonar and returns position estimate.
        """
        self.get_logger().info('Sonar position request received - starting scan...')
        
        try:
            # Wait for sonar service availability
            if not self.sonar_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Sonar service not available')
                response.success = False
                response.message = 'Sonar service unavailable'
                return response
            
            # Perform sonar scan based on configuration
            if self.enable_4wall_triangulation:
                x, y = self._scan_four_walls()
            else:
                x, y = self._scan_two_walls()
            
            # Check if scan was successful
            if x is None or y is None:
                self.get_logger().warn('Could not determine position from sonar scan')
                response.success = False
                response.message = 'Insufficient sonar detections'
                return response
            
            # Create and populate response pose
            pose = PoseWithCovarianceStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            
            # Set position
            pose.pose.pose.position.x = float(x)
            pose.pose.pose.position.y = float(y)
            pose.pose.pose.position.z = 0.0
            
            # Orientation unknown from sonar
            pose.pose.pose.orientation.w = 1.0
            
            # Set covariance
            covariance = [0.0] * 36
            covariance[0] = self.position_variance   # x variance
            covariance[7] = self.position_variance   # y variance
            covariance[14] = 1e6                     # z variance (no info)
            covariance[21] = 1e6                     # roll (no info)
            covariance[28] = 1e6                     # pitch (no info)
            covariance[35] = 1e6                     # yaw (no info)
            pose.pose.covariance = covariance
            
            # Set response
            response.success = True
            response.message = f'Position: x={x:.2f}m, y={y:.2f}m'
            response.pose = pose
            
            self.get_logger().info(
                f'Sonar scan complete: x={x:.2f}m, y={y:.2f}m '
                f'(variance={self.position_variance})'
            )
            
            return response
        
        except Exception as e:
            self.get_logger().error(f'Sonar position callback error: {e}')
            response.success = False
            response.message = f'Error: {str(e)}'
            return response
    
    def _scan_four_walls(self) -> Tuple[Optional[float], Optional[float]]:
        """
        Scan in 4 directions to triangulate from walls.
        
        Scanning pattern:
        - 0°   → Right wall (distance = pool_x - x_position)
        - 90°  → Far wall   (distance = pool_y - y_position)
        - 180° → Left wall  (distance = x_position)
        - 270° → Near wall  (distance = y_position)
        
        Returns: (x, y) position estimates in meters
        """
        
        # Define scan angles in degrees
        scan_angles = [0, 90, 180, 270]
        measured_distances = []
        
        for angle in scan_angles:
            # Scan narrow beam at each cardinal direction
            distance = self._scan_at_angle(angle, beam_width=10)
            measured_distances.append(distance)
            self.get_logger().debug(f'Sonar at {angle}°: {distance:.2f}m' if distance else f'Sonar at {angle}°: no detection')
        
        # Triangulate position from wall distances
        x_estimates = []
        y_estimates = []
        
        # Right wall (0°): x = pool_x - distance
        if measured_distances[0] is not None:
            x_estimates.append(self.pool_x - measured_distances[0])
        
        # Far wall (90°): y = pool_y - distance
        if measured_distances[1] is not None:
            y_estimates.append(self.pool_y - measured_distances[1])
        
        # Left wall (180°): x = distance
        if measured_distances[2] is not None:
            x_estimates.append(measured_distances[2])
        
        # Near wall (270°): y = distance
        if measured_distances[3] is not None:
            y_estimates.append(measured_distances[3])
        
        # Average estimates from opposite walls
        x = np.mean(x_estimates) if x_estimates else None
        y = np.mean(y_estimates) if y_estimates else None
        
        # Validate estimates are within pool bounds
        if x is not None:
            if x < 0 or x > self.pool_x:
                self.get_logger().warn(f'X estimate {x:.2f}m outside pool bounds [0, {self.pool_x}]')
                x = None
        
        if y is not None:
            if y < 0 or y > self.pool_y:
                self.get_logger().warn(f'Y estimate {y:.2f}m outside pool bounds [0, {self.pool_y}]')
                y = None
        
        return x, y
    
    def _scan_two_walls(self) -> Tuple[Optional[float], Optional[float]]:
        """
        Simplified scan: left/right walls only for basic position estimation.
        
        Returns: (x, y) where only x is reliable
        """
        
        # Scan left-right (0-180°)
        distance_right = self._scan_at_angle(0, beam_width=30)
        distance_left = self._scan_at_angle(180, beam_width=30)
        
        x = None
        
        if distance_right is not None:
            x = self.pool_x - distance_right
        elif distance_left is not None:
            x = distance_left
        
        # Y is not reliably determinable with only 2-wall scan
        # Would need wall-following or additional sensors
        y = self.pool_y / 2.0  # Assume center position as default
        
        return x, y
    
    def _scan_at_angle(self, angle_deg: int, beam_width: int = 10) -> Optional[float]:
        """
        Perform sonar scan at a specific angle and return nearest object distance.
        
        Args:
            angle_deg: Scan direction in degrees (0-359)
            beam_width: Beam width in degrees (narrow = directional)
        
        Returns:
            Distance to nearest object, or None if no detection
        """
        
        try:
            # Convert to sonar device angles (0-400 grads = 0-360°)
            start_angle = int((angle_deg - beam_width/2) * 400 / 360)
            end_angle = int((angle_deg + beam_width/2) * 400 / 360)
            
            # Clamp to valid range
            start_angle = max(0, min(400, start_angle))
            end_angle = max(0, min(400, end_angle))
            
            # Create sonar scan request
            request = SonarScan.Request(
                start_angle=start_angle,
                end_angle=end_angle,
                desired_range=int(self.sonar_max_range)
            )
            
            # Call sonar service
            future = self.sonar_client.call_async(request)
            
            # Wait for response (with timeout)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is None:
                return None
            
            scan_msg = future.result().data
            
            # Extract closest detection
            min_distance = float('inf')
            detected = False
            
            for laser_echo in scan_msg.ranges:
                if laser_echo.ranges:  # Has measurement
                    distance = laser_echo.ranges[0]
                    if distance > 0:  # Valid detection
                        min_distance = min(min_distance, distance)
                        detected = True
            
            return min_distance if detected else None
        
        except Exception as e:
            self.get_logger().error(f'Sonar scan error at {angle_deg}°: {e}')
            return None


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    sonar_loc_node = SonarLocalizationNode()
    
    try:
        rclpy.spin(sonar_loc_node)
    except KeyboardInterrupt:
        sonar_loc_node.get_logger().info('Shutting down sonar localization service')
    finally:
        sonar_loc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
