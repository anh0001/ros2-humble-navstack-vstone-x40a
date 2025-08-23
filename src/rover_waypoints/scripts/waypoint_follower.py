#!/usr/bin/env python3

import math
import time
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

    def follow_warehouse_route(self, dwell_sec=5.0):
        """Follow a predefined warehouse delivery route with delays at each waypoint"""
        waypoints = [
            self.create_pose(3.385, -3.065, 0.0),
            self.create_pose(5.344, -3.89, math.radians(90)),
            self.create_pose(3.0169, 1.043, math.radians(180)),
            self.create_pose(0.0, 0.0, 0.0),  # Return to base
        ]

        self.get_logger().info(f'Starting delivery route with {len(waypoints)} waypoints')

        for i, pose in enumerate(waypoints, start=1):
            self.get_logger().info(f'Navigating to waypoint {i}/{len(waypoints)}')
            self.navigator.goToPose(pose)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    # Optional: log progress
                    pass
                rclpy.spin_once(self, timeout_sec=0.2)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if dwell_sec > 0:
                    self.get_logger().info(f'Arrived at waypoint {i}. Waiting {dwell_sec} seconds...')
                    time.sleep(dwell_sec)
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'Waypoint {i} canceled; aborting route')
                return
            else:
                self.get_logger().error(f'Failed to reach waypoint {i}; aborting route')
                return

        self.get_logger().info('Delivery route completed successfully!')

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