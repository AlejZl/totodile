import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class LeftHandRuleExplorer(Node):
    def __init__(self):
        super().__init__("Explorer")
        self.motors = self.create_publisher(Twist, '/cmd_vel', 10)

        self.front_view_info = self.create_subscription(String, '/front_view_info', self.front_view_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.status = {"front": 0}  # 0 = free, 1 = occupied
        self.rotating = False
        self.followingWall = False
        self.target_yaw = None
        self.current_yaw = None

        self.get_logger().info('LeftHandRuleExplorer Node has been started')

    def imu_callback(self, msg: Imu):
        # Extract the yaw from the normalized orientation.z
        self.current_yaw = msg.orientation.z

    def front_view_callback(self, msg: String):
        self.status["front"] = 0 if msg.data == "Free" else 1
        self.update_explorer()

    def normalize_yaw(self, yaw):
        """
        Normalize yaw to the range [-1, 1].
        """
        if yaw > 1:
            return yaw - 2
        elif yaw < -1:
            return yaw + 2
        return yaw

    def rotation_finished(self):
        if self.target_yaw is None or self.current_yaw is None:
            return False

        # Calculate the signed difference to handle wraparound
        diff = self.target_yaw - self.current_yaw
        diff = self.normalize_yaw(diff)  # Ensure diff remains in [-1, 1]

        self.get_logger().info(f"CURRENT YAW: {self.current_yaw}, TARGET YAW: {self.target_yaw}, DIFF: {diff}")
        return diff <= 0.0  # Rotation is finished when we overshoot or meet the target

    def update_explorer(self):
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0

        if self.rotating:
            rad_per_sec = 0.4
            self.get_logger().info("ROTATION CHECKUP...")

            if self.rotation_finished():
                self.rotating = False
                command.angular.z = 0.0
                self.followingWall = True
            else:
                command.angular.z = -rad_per_sec  # Rotate clockwise

        elif self.followingWall:
            self.get_logger().info("FOLLOWING WALL PART")
            # Implement wall-following logic if required
            pass

        elif not bool(self.status["front"]):  # Front is free
            command.angular.z = 0.0
            command.linear.x = 0.13
        else:  # Front is blocked
            self.get_logger().info("FRONT BLOCKED: STARTING ROTATION...")
            self.rotating = True

            # Calculate the target yaw
            self.target_yaw = self.normalize_yaw(self.current_yaw + 0.70)

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
