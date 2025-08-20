#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Initialize the navigator
        self.navigator = BasicNavigator()
        
        # Wait for Nav2 to be active
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active, ready to follow waypoints')

    def create_pose(self, x, y, yaw=0.0):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        return pose

    def follow_warehouse_route(self):
        """Follow a predefined warehouse delivery route"""
        waypoints = []
        
        # Define delivery route: base -> shelf_1 -> station_A -> shelf_2 -> station_B -> base
        waypoints.append(self.create_pose(8.574, -0.18, math.radians(90)))
        waypoints.append(self.create_pose(5.086, 1.933, math.radians(180)))
        waypoints.append(self.create_pose(0.0, 0.0, 0.0))    # Return to base

        self.get_logger().info(f'Starting delivery route with {len(waypoints)} waypoints')
        
        # Follow the waypoints
        self.navigator.followWaypoints(waypoints)

        # Monitor progress
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                current_waypoint = feedback.current_waypoint
                self.get_logger().info(
                    f'Executing waypoint {current_waypoint + 1} of {len(waypoints)}'
                )
            rclpy.spin_once(self, timeout_sec=1.0)

        # Check final result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Delivery route completed successfully!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Delivery route was canceled')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Delivery route failed!')

def main(args=None):
    rclpy.init(args=args)
    
    waypoint_follower = WaypointFollower()
    
    try:
        # Execute the warehouse delivery route
        waypoint_follower.follow_warehouse_route()
    except KeyboardInterrupt:
        waypoint_follower.get_logger().info('Waypoint following interrupted')
    finally:
        waypoint_follower.navigator.lifecycleShutdown()
        waypoint_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()