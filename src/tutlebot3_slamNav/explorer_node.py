
import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy




# #70-130 front
# #10-70 right
# #130-190 left
# #180-10 back

class LeftHandRuleExplorer(Node):
    def __init__(self):
        super().__init__("Explorer")
        self.motors = self.create_publisher(Twist,'/cmd_vel', 10)

        self.front_view_info = self.create_subscription(String,'/front_view_info',self.front_view_callback,10) 
        
        self.get_logger().info('LeftHandRuleExplorer Node has been started')

    def front_view_callback(self, msg):
        command = Twist()
        status = msg.data

        if status == "Free":
            command.linear.x = 0.01
        else:
            command.linear.x = 0.0
    

        self.motors.publish(command)
        self.get_logger().info("TWIST MSG PUBLISHED")


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
