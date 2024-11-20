#!/usr/bin/env python3

import rclpy
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class RightHandRuleExplorer(Node):
    def __init__(self):
        super().__init__('right_hand_rule_explorer')
        
        # Publisher to control the robot's movement
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for laser scan data
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.process_scan, qos_profile=QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        depth=10  # You can adjust this if needed
    ))
        
        # Initialize Twist message
        self.twist = Twist()
        
        # Wall-following parameters
        self.forward_speed = 0.2  # Forward movement speed
        self.turning_speed = 0.5  # Turning speed
        self.safe_distance = 0.5  # Desired distance from the wall

    def process_scan(self, scan):

        self.get_logger().info(len(scan.ranges))
        # Extract the relevant sections of the laser scan ranges
        # front_section = scan.ranges[0:30] + scan.ranges[-30:]  # Front of the robot
        # right_section = scan.ranges[270:300]  # Right side of the robot
        # left_section = scan.ranges[60:90]  # Left side of the robot

        # # Log the sections to check their values
        # self.get_logger().info(f"Front section: {front_section}")
        # self.get_logger().info(f"Right section: {right_section}")
        # self.get_logger().info(f"Left section: {left_section}")

        # # Helper function to filter out invalid ranges and return a safe minimum value
        # def safe_min(range_slice):
        #     valid_ranges = [r for r in range_slice if not math.isnan(r)]
        #     return min(valid_ranges) if valid_ranges else float('inf')

        # # Calculate distances with safe_min to avoid errors
        # front_distance = safe_min(front_section)
        # right_distance = safe_min(right_section)
        # left_distance = safe_min(left_section)

        # # 1. Obstacle directly in front, turn left
        # if front_distance < self.safe_distance:
        #     self.twist.linear.x = 0.0
        #     self.twist.angular.z = self.turning_speed  # Turn left
        # # 2. No wall on the right, turn right to follow the wall
        # elif right_distance > self.safe_distance:
        #     self.twist.linear.x = 0.0
        #     self.twist.angular.z = -self.turning_speed  # Turn right
        # # 3. Wall on the right, move forward
        # else:
        #     self.twist.linear.x = self.forward_speed
        #     self.twist.angular.z = 0.0

        # # Publish the twist message to command the robot
        # self.pub_cmd.publish(self.twist)

    def run(self):
        # Keep the node running to process callbacks
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    right_hand_rule_explorer = RightHandRuleExplorer()
    right_hand_rule_explorer.run()
    right_hand_rule_explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# def scan_callback(self, msg):
#     twist = Twist()
#     min_distance = 1.0  # Set your minimum distance threshold

#     # Calculate the index for -90 and 90 degrees
#     total_measurements = len(msg.ranges)
#     angle_min = msg.angle_min
#     angle_max = msg.angle_max
#     angle_increment = msg.angle_increment

#     # Calculate index for -90 degrees
#     left_angle = -90.0 * (3.14159265359 / 180.0)  # Convert to radians
#     left_index = int((left_angle - angle_min) / angle_increment)

#     # Calculate index for 90 degrees
#     right_angle = 90.0 * (3.14159265359 / 180.0)  # Convert to radians
#     right_index = int((right_angle - angle_min) / angle_increment)

#     # Ensure indices are within the range
#     if left_index < 0 or left_index >= total_measurements:
#         left_index = 0
#     if right_index < 0 or right_index >= total_measurements:
#         right_index = total_measurements - 1

#     front_distance = msg.ranges[0]
#     left_side_distance = msg.ranges[left_index]
#     right_side_distance = msg.ranges[right_index]

#     if front_distance > min_distance and left_side_distance > min_distance:
#         twist.linear.x = 0.2  # Move forward
#         twist.angular.z = 0.0
#     else:
#         twist.linear.x = 0.0  # Stop
#         twist.angular.z = 0.5  # Turn left

#     self.publisher.publish(twist)
