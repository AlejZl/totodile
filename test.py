import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray,String
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy

class test(Node):
    def __init__(self):
        super().__init__('tst node')
        self.laser_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback,
											 qos_profile = QoSProfile(
												 reliability=ReliabilityPolicy.BEST_EFFORT,
												 durability=DurabilityPolicy.VOLATILE,
												 depth=10
											 ))

    def scan_callback(self, msg=LaserScan):
        
		
        min_ang = msg.angle_min
        max_ang = msg.angle_max
        incremnt = msg.angle_increment
        self.get_logger().info(f"minimum angle scan: {min_ang}, max angle scan: {max_ang} and incre: {incremnt}")
        
def main(args=None):
    rclpy.init(args=args)
    node = test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
