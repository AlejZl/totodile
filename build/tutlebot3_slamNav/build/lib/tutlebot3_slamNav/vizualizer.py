import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray,String
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy




class Vizualizer(Node):

	def __init__(self):

		super().__init__('Vizualizer')

		self.front = self.create_publisher(Float32MultiArray, '/front_view',10)
		self.left = self.create_publisher(Float32MultiArray, '/left_view', 10)
		self.right = self.create_publisher(Float32MultiArray, '/right_view', 10)

		self.front_info = self.create_publisher(String, '/front_view_info',10)
		self.left_info = self.create_publisher(String, '/left_view_info', 10)
		self.right_info = self.create_publisher(String, '/right_view_info', 10)

		self.laser_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback,
											 qos_profile = QoSProfile(
												 reliability=ReliabilityPolicy.BEST_EFFORT,
												 durability=DurabilityPolicy.VOLATILE,
												 depth=10
											 ))
		
		self.minimum_value_treshold = 0.2
		
		self.get_logger().info("Vizualizer node started")
		
	def collide(self,data):
		
		data_lenght = len(data)
		prom = np.sum(data) / data_lenght

		if prom >= self.minimum_value_treshold:
			return False
		else:
			return True
		
	def scan_callback(self,laser_scan):

		front_view = Float32MultiArray()
		left_view = Float32MultiArray()
		right_view = Float32MultiArray()

		front_view.data = [float(val) for val in np.nan_to_num(laser_scan.ranges[70:130], nan=0)] 
		left_view.data = [float(val) for val in np.nan_to_num(laser_scan.ranges[130:190], nan=0)] 
		right_view.data = [float(val) for val in np.nan_to_num(laser_scan.ranges[10:69], nan=0)]

		if self.collide(front_view.data):
			self.front_info.publish(String(data="NotFree"))
		else:
			self.front_info.publish(String(data="Free"))
		
		if self.collide(left_view.data):
			self.left_info.publish(String(data="NotFree"))
		else:
			self.left_info.publish(String(data="Free"))

		if self.collide(right_view.data):
			self.right_info.publish(String(data="NotFree"))
		else:
			self.right_info.publish(String(data="Free"))


		self.front.publish(front_view)
		self.right.publish(right_view)
		self.left.publish(left_view)
		self.get_logger().info("VIEWS PUBLISHED")
	
	
def main(args=None):
    rclpy.init(args=args)
    node = Vizualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
