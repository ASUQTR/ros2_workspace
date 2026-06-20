#!/usr/bin/env python3
"""
Example Mission Manager Integration with Sonar Localization Service

This demonstrates how a mission manager would use the on-demand sonar
localization service to periodically correct position drift.

Usage:
    ros2 run sub_control example_mission_manager.py
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sub_interfaces.srv import GetSonarPosition


class ExampleMissionManager(Node):
    """
    Simple mission manager that demonstrates periodic sonar correction.
    """
    
    def __init__(self):
        super().__init__('example_mission_manager')
        
        # Configuration
        self.declare_parameter('sonar_update_interval', 600.0)  # 10 minutes
        self.sonar_update_interval = self.get_parameter('sonar_update_interval').value
        
        # Last sonar update time
        self.last_sonar_update = 0.0
        self.mission_start_time = time.time()
        
        # Create client for sonar localization service
        self.sonar_client = self.create_client(
            GetSonarPosition,
            '/sonar_localization/get_sonar_position'
        )
        
        # Publisher for position corrections to EKF
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'pose_estimate',
            10
        )
        
        # Timer for mission updates
        self.timer = self.create_timer(1.0, self.mission_update)
        
        self.get_logger().info(
            f'Example Mission Manager started\n'
            f'  Sonar update interval: {self.sonar_update_interval:.0f} seconds '
            f'({self.sonar_update_interval/60:.1f} minutes)'
        )
    
    def mission_update(self):
        """Called periodically (1 Hz) to manage mission logic."""
        
        current_time = time.time() - self.mission_start_time
        
        # Check if it's time for a sonar position correction
        if current_time - self.last_sonar_update >= self.sonar_update_interval:
            self.get_logger().info(
                f'Mission time: {current_time:.1f}s - '
                f'Requesting sonar position correction...'
            )
            
            # Request position update from sonar service
            self.request_sonar_position()
            
            self.last_sonar_update = current_time
        
        # Log periodic status
        if int(current_time) % 60 == 0 and int(current_time) != int(current_time - 1):
            time_until_next = self.sonar_update_interval - (current_time - self.last_sonar_update)
            self.get_logger().info(
                f'Mission time: {current_time:.0f}s - '
                f'Time until next sonar correction: {time_until_next:.0f}s'
            )
    
    def request_sonar_position(self):
        """Call the sonar localization service."""
        
        try:
            # Ensure service is available
            if not self.sonar_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Sonar localization service not available')
                return
            
            # Create request
            request = GetSonarPosition.Request()
            
            # Call service
            future = self.sonar_client.call_async(request)
            
            # Wait for response
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is None:
                self.get_logger().error('Sonar service call timed out')
                return
            
            response = future.result()
            
            # Handle response
            if response.success:
                pose = response.pose
                x = pose.pose.pose.position.x
                y = pose.pose.pose.position.y
                
                self.get_logger().info(
                    f'Sonar position obtained: x={x:.2f}m, y={y:.2f}m\n'
                    f'  Status: {response.message}\n'
                    f'  Publishing to EKF for correction...'
                )
                
                # Publish pose estimate to EKF for correction
                self.pose_pub.publish(pose)
                
            else:
                self.get_logger().warn(
                    f'Sonar position request failed: {response.message}'
                )
                # Continue mission without correction
        
        except Exception as e:
            self.get_logger().error(f'Error requesting sonar position: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    mission = ExampleMissionManager()
    
    try:
        rclpy.spin(mission)
    except KeyboardInterrupt:
        mission.get_logger().info('Mission interrupted')
    finally:
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
