import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32MultiArray, String


class LeftHandRuleExplorer(Node):
    def __init__(self):
        super().__init__("Explorer")
        self.motors = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        self.front_view_info = self.create_subscription(String, '/front_view_info', self.front_view_callback, 10)
        self.left_view_info = self.create_subscription(String, '/left_view_info', self.left_view_callback, 10)
        self.right_view_info = self.create_subscription(String, '/right_view_info', self.right_view_callback, 10)

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Status and control variables
        self.status = {"front": 0, "left": 0, "right": 0}  # 0 = free, 1 = occupied
        self.rotating = False
        self.followingWall = False
        self.target_yaw = None  # Desired yaw for rotation
        self.current_yaw = 0.0  # Current yaw from IMU

        self.get_logger().info('LeftHandRuleExplorer Node has been started')

    def front_view_callback(self, msg: String):
        self.status["front"] = 0 if msg.data == "Free" else 1
        self.update_explorer()

    def left_view_callback(self, msg: String):
        self.status["left"] = 0 if msg.data == "Free" else 1
        self.update_explorer()

    def right_view_callback(self, msg: String):
        self.status["right"] = 0 if msg.data == "Free" else 1
        self.update_explorer()

    def imu_callback(self, msg: Imu):
        # Extract yaw from quaternion
        q = msg.orientation
        self.current_yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def rotation_finished(self):
        # Check if the robot's yaw has reached the target yaw
        if self.target_yaw is None:
            return False

        yaw_diff = abs(self.current_yaw - self.target_yaw)
        self.get_logger().info(f"Current Yaw: {self.current_yaw:.2f}, Target Yaw: {self.target_yaw:.2f}, Diff: {yaw_diff:.2f}")
        return yaw_diff < 0.1  # Rotation considered finished if yaw difference is small

    def update_explorer(self):
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0

        if self.rotating:
            self.get_logger().info("ROTATING...")
            if self.rotation_finished():
                self.rotating = False
                self.target_yaw = None
                command.angular.z = 0.0
                #self.followingWall = True
            else:
                command.angular.z = -0.4  # Continue rotating
        elif self.followingWall:
            self.get_logger().info("FOLLOWING WALL...")
            command.linear.x = 0.0
            command.angular.z = 0.0
            # Add wall-following logic (e.g., PID control for keeping distance from the wall)
        elif self.status["front"] == 0:  # Front is free
            self.get_logger().info("MOVING FORWARD...")
            command.linear.x = 0.13
        else:  # Front is blocked
            self.get_logger().info("FRONT BLOCKED: STARTING ROTATION...")
            self.rotating = True
            self.target_yaw = (self.current_yaw - np.pi / 2) % (2 * np.pi)  # Rotate 90 degrees clockwise


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
