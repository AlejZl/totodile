import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class RightHandRuleExplorer(Node):
    def __init__(self):
        super().__init__('right_hand_rule_explorer')
        self.distance_threshold = 1.0  # Set this threshold as needed
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.process_scan,
            qos_profile=QoSProfile(
				reliability=ReliabilityPolicy.BEST_EFFORT,
				durability=DurabilityPolicy.VOLATILE,
				depth=10  # You can adjust this if needed
			))
        self.pub = self.create_publisher(LaserScan,"/nan_on_scan",10)
        self.get_logger().info("Right Hand Rule Explorer node has started.")

    def process_scan(self, scan):

        laserscan = LaserScan()
        scan_reading = scan.ranges
        scan_length = len(scan_reading)


        nan_values = []

        for i in range (scan_length):
            if math.isnan(scan_reading[i]):
                nan_values.append(float(i))

        laserscan.ranges = nan_values
        self.pub.publish(laserscan)

        self.get_logger().info("|LaserScanNaNPos| published successfull")
        # # Define the sections based on index ranges
        # front_section = scan.ranges[0:30] + scan.ranges[-30:]
        # right_section = scan.ranges[270:300]
        # left_section = scan.ranges[60:90]

        # # Check for obstacles in each section by seeing if any value is below the threshold
        # front_clear = all(distance > self.distance_threshold for distance in front_section if not self.is_nan(distance))
        # right_clear = all(distance > self.distance_threshold for distance in right_section if not self.is_nan(distance))
        # left_clear = all(distance > self.distance_threshold for distance in left_section if not self.is_nan(distance))

        # # Log information for debugging
        # self.get_logger().info(f"Front clear: {front_clear}\n Right clear: {right_clear}\n Left clear: {left_clear}")

        # # Implement right-hand rule based on threshold checks
        # if right_clear:
        #     self.get_logger().info("Clear path to the right, turning right.")
        #     # Add code here to turn right
        # elif front_clear:
        #     self.get_logger().info("Clear path forward, moving forward.")
        #     # Add code here to move forward
        # elif left_clear:
        #     self.get_logger().info("Clear path to the left, turning left.")
        #     # Add code here to turn left
        # else:
        #     self.get_logger().info("No clear path, stopping or turning around.")
        #     # Add code here to stop or perform a turnaround

    def is_nan(self, value):
        # Check if a value is NaN
        return value != value

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    right_hand_rule_explorer = RightHandRuleExplorer()
    try:
        right_hand_rule_explorer.run()
    except KeyboardInterrupt:
        pass
    finally:
        right_hand_rule_explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
