#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class YawPublisherNode(Node):
    def __init__(self):
        super().__init__('yaw_publisher')
        self.declare_parameter('enable_publish_log', False)
        self.enable_publish_log = self.get_parameter('enable_publish_log').value
        self.publisher_ = self.create_publisher(Float32, '/robot/yaw', 10)
        self.timer = self.create_timer(0.05, self.publish_yaw) # Publish at 20 Hz
        self.start_time = time.time()
        self.get_logger().info(
            f'Fake Yaw Publisher started. Publishing to /robot/yaw, '
            f'enable_publish_log={self.enable_publish_log}'
        )

    def publish_yaw(self):
        # Create a slowly rotating yaw angle for testing
        # It will complete a full circle every 30 seconds
        elapsed_time = time.time() - self.start_time
        angle_radians = (elapsed_time * (2 * math.pi / 30.0)) % (2 * math.pi)
        
        # Convert to degrees from 0-360
        angle_degrees = math.degrees(angle_radians)

        msg = Float32()
        msg.data = angle_degrees
        self.publisher_.publish(msg)
        if self.enable_publish_log:
            self.get_logger().info(f'Publishing Yaw: {angle_degrees:.2f} degrees')


def main(args=None):
    rclpy.init(args=args)
    node = YawPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
