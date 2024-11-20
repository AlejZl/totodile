# import rclpy
# import math

# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan

# #70-130 front
# #10-70 right
# #130-190 left
# #180-10 back
# class RightHandRuleExplorer(Node):
#     def __init__(self):
#         super().__init__('right_hand_rule_explorer')
#         self.distance_threshold = 1.0  # Set this threshold as needed
#         self.subscription = self.create_subscription(
#             LaserScan,
#             'scan',
#             self.process_scan,
#             qos_profile=QoSProfile(
# 				reliability=ReliabilityPolicy.BEST_EFFORT,
# 				durability=DurabilityPolicy.VOLATILE,
# 				depth=10  # You can adjust this if needed
# 			))
#         self.pub = self.create_publisher(LaserScan,"/scan_filtered",10)
#         self.get_logger().info("Right Hand Rule Explorer node has started.")

#     def process_scan(self, scan):

#         laserscan = LaserScan()
#         scan_reading = scan.ranges
#         scan_intensities = scan.intensities
#         scan_length = len(scan_reading)
#         scanInt_lenght = len(scan_intensities)



#         nan_scan_values = []
#         nan_int_values = []

#         for i in range (scan_length):
#             if (scan_reading[i] < 0.3 and not math.isnan(scan_reading[i])): #change for less than 1 instead of NaN
#                 nan_scan_values.append(float(i))

#         # for i in range(scanInt_lenght):
#         #     if math.isnan(scan_intensities[i]):
#         #         nan_int_values.append(float(i))
#         nan_int_values = scan_reading[70:125]


#         laserscan.ranges = nan_scan_values
#         laserscan.intensities = scan_reading

#         self.pub.publish(laserscan)

#         self.get_logger().info("|LaserScanNaNPos| published successfull")
#         # # Define the sections based on index ranges
#         # front_section = scan.ranges[0:30] + scan.ranges[-30:]
#         # right_section = scan.ranges[270:300]
#         # left_section = scan.ranges[60:90]

#         # # Check for obstacles in each section by seeing if any value is below the threshold
#         # front_clear = all(distance > self.distance_threshold for distance in front_section if not self.is_nan(distance))
#         # right_clear = all(distance > self.distance_threshold for distance in right_section if not self.is_nan(distance))
#         # left_clear = all(distance > self.distance_threshold for distance in left_section if not self.is_nan(distance))

#         # # Log information for debugging
#         # self.get_logger().info(f"Front clear: {front_clear}\n Right clear: {right_clear}\n Left clear: {left_clear}")

#         # # Implement right-hand rule based on threshold checks
#         # if right_clear:
#         #     self.get_logger().info("Clear path to the right, turning right.")
#         #     # Add code here to turn right
#         # elif front_clear:
#         #     self.get_logger().info("Clear path forward, moving forward.")
#         #     # Add code here to move forward
#         # elif left_clear:
#         #     self.get_logger().info("Clear path to the left, turning left.")
#         #     # Add code here to turn left
#         # else:
#         #     self.get_logger().info("No clear path, stopping or turning around.")
#         #     # Add code here to stop or perform a turnaround

#     def is_nan(self, value):
#         # Check if a value is NaN
#         return value != value

#     def run(self):
#         rclpy.spin(self)

# def main(args=None):
#     rclpy.init(args=args)
#     right_hand_rule_explorer = RightHandRuleExplorer()
#     try:
#         right_hand_rule_explorer.run()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         right_hand_rule_explorer.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np

class LeftHandRuleExplorer(Node):
    def __init__(self):
        super().__init__('left_hand_rule_explorer')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 
                                                     qos_profile=QoSProfile(
				                                        reliability=ReliabilityPolicy.BEST_EFFORT,
				                                        durability=DurabilityPolicy.VOLATILE,
				                                        depth=10  # You can adjust this if needed
			))
        self.get_logger().info('LeftHandRuleExplorer Node has been started')

    def scan_callback(self, msg):
        twist = Twist()
        min_distance = 0.5  # Set your minimum distance threshold
        valid_distance_threshold = 0.3  # Threshold for valid distances

        # Define ranges for different directions
        front_distances = msg.ranges[70:130]
        left_distances = msg.ranges[130:180]

        # Filter out distances below the valid distance threshold
        valid_front_distances = [d for d in front_distances if d >= valid_distance_threshold]
        valid_left_distances = [d for d in left_distances if d >= valid_distance_threshold]

        # Calculate the proportion of valid distances above the minimum distance
        front_above_threshold = np.sum(np.array(valid_front_distances) > min_distance) / len(valid_front_distances)
        left_above_threshold = np.sum(np.array(valid_left_distances) > min_distance) / len(valid_left_distances)

        # Decision logic for left-hand rule exploration 
        if front_above_threshold >= 0.5 and left_above_threshold >= 0.5: 
            twist.linear.x = 0.15 # Move forward 
            twist.angular.z = 0.0 # No rotation 
        else: 
            twist.linear.x = 0.0 # Stop 
            twist.angular.z = 0.5 # Turn left

        self.get_logger().info(f"frnt avove tresh: {str(front_above_threshold >= 0.5)}")
        self.get_logger().info(f"lft avove tresh: {str(left_above_threshold >= 0.5)}")
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LeftHandRuleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
