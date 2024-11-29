import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32MultiArray, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Angle ranges for different views:
# 70°-130°: front
# 10°-70°: right
# 130°-190°: left
# 180°-10°: back

class LeftHandRuleExplorer(Node):
    def __init__(self):
        super().__init__("Explorer")

        # Publisher to send velocity commands to the robot
        self.motors = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions for detecting status (free or occupied) of different views
        self.front_view_info = self.create_subscription(String, '/front_view_info', self.front_view_callback, 10)
        self.left_view_info = self.create_subscription(String, '/left_view_info', self.left_view_callback, 10)
        self.right_view_info = self.create_subscription(String, '/right_view_info', self.right_view_callback, 10)

        # Subscription to the robot's IMU for orientation data
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Subscriptions to receive raw distance data from sensors
        self.front_value = self.create_subscription(Float32MultiArray, '/front_view', self.front_data_callback, 10)
        self.left_value = self.create_subscription(Float32MultiArray, '/left_view', self.left_data_callback, 10)
        self.right_value = self.create_subscription(Float32MultiArray, '/right_view', self.right_data_callback, 10)

        # Status dictionary to track the state of each direction (0 = free, 1 = occupied)
        self.status = {"front": 0, "left": 0, "right": 0}

        # Variables to manage rotation and wall-following behavior
        self.rotating = False
        self.followingWall = False
        self.currentYaw = None
        self.targetYaw = None

        # Reference values for rotation comparison
        self.rotRef = 0
        self.lastRef = 100

        self.get_logger().info('LeftHandRuleExplorer Node has been started')

    # Callback to update the robot's current yaw angle from the IMU data
    def imu_callback(self, msg: Imu):
        self.currentYaw = msg.orientation.z

    # Callbacks to update the status of different views based on String messages
    def front_view_callback(self, msg: String):
        self.status["front"] = 0 if msg.data == "Free" else 1
        self.update_explorer()

    def left_view_callback(self, msg: String):
        self.status["left"] = 0 if msg.data == "Free" else 1
        self.update_explorer()

    def right_view_callback(self, msg: String):
        self.status["right"] = 0 if msg.data == "Free" else 1
        self.update_explorer()

    # Callbacks to process raw sensor data and compute the mean distance for each view
    def front_data_callback(self, msg: Float32MultiArray):
        self.front_value = np.nan_to_num(np.mean(msg.data), nan=0.0)

    def left_data_callback(self, msg: Float32MultiArray):
        self.left_value = np.nan_to_num(np.mean(msg.data), nan=0.0)

    def right_data_callback(self, msg: Float32MultiArray):
        self.right_value = np.nan_to_num(np.mean(msg.data), nan=0.0)

    # Utility method to compute the mean of a list while handling NaN values
    def mean(self, data=list):
        return np.mean(np.nan_to_num(data))

    # Check if the rotation has completed based on sensor readings
    def rotation_finished(self):
        self.get_logger().info("ROTATION CHECKUP...")
        left_part = round(self.mean(self.left_value), 1)
        compare_part = round(self.mean(self.rotRef), 1)

        self.get_logger().info(f"left_current_value: {left_part}")
        self.get_logger().info(f"comp_current_value: {compare_part}")

        return not (left_part >= compare_part)

    # Main logic to update the robot's behavior based on current state and sensor data
    def update_explorer(self):
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0

        if self.rotating:  # If rotating, execute the rotation logic
            radPerSec = 0.4
            command.linear.x = 0.0

            if self.rotation_finished():
                self.rotating = False
                command.angular.z = 0.0
                self.followingWall = True
            else:
                command.angular.z = -radPerSec
                self.lastRef = self.mean(self.left_value)

        elif self.followingWall:  # If following the wall, continue wall-following behavior
            command.linear.x = 0.0
            command.angular.z = 0.0
            self.get_logger().info("FOLLOWING WALL PART")
            self.followingWall = False

        elif not bool(self.status["front"]):  # If front is free, move forward
            command.angular.z = 0.0
            command.linear.x = 0.07
            self.get_logger().info("FRONT IS FREE")

        else:  # Otherwise, prepare to rotate
            self.rotating = True
            self.targetYaw = self.currentYaw + 0.70

            if self.targetYaw > 1:
                self.targetYaw = 1.0 - (self.targetYaw - 1.0)

        # Publish the command to the motors
        self.motors.publish(command)

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
