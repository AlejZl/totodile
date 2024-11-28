#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class Float32MultiArrayPlotter(Node):
    def __init__(self):
        super().__init__('float32_multiarray_plotter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/left_view',  # Replace with your topic name
            self.listener_callback,
            10)
        self.data_buffer = []

    def listener_callback(self, msg):
        self.data_buffer = msg.data
        self.get_logger().info(f"Received data: {self.data_buffer}")

def main(args=None):
    rclpy.init(args=args)
    plotter_node = Float32MultiArrayPlotter()

    plt.ion()  # Interactive mode
    fig, ax = plt.subplots()
    line, = ax.plot([], [], '-o', label="Float32MultiArray Data")
    ax.legend()
    ax.set_title("Real-Time Float32MultiArray Data")
    ax.set_xlabel("Index")
    ax.set_ylabel("Value")

    try:
        while rclpy.ok():
            rclpy.spin_once(plotter_node, timeout_sec=0.1)
            data = plotter_node.data_buffer
            if data:
                line.set_xdata(range(len(data)))
                line.set_ydata(data)
                ax.relim()
                ax.autoscale_view()
                plt.pause(0.01)  # Briefly update plot
    except KeyboardInterrupt:
        pass
    finally:
        plotter_node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
