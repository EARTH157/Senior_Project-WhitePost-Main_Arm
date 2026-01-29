#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Main_Processor(Node):
    def __init__(self):
        super().__init__('Main_processor_node')

        self.publisher1_ = self.create_publisher(Float32, 'joint1/set_target_angle', 10)
        self.publisher2_ = self.create_publisher(Float32, 'joint2/set_target_angle', 10)
        self.publisher3_ = self.create_publisher(Float32, 'joint3/set_target_angle', 10)
        
        self.publisher1_calibrate_ = self.create_publisher(Float32, 'joint1/calibrate', 10)
        self.publisher2_calibrate_ = self.create_publisher(Float32, 'joint2/calibrate', 10)
        self.publisher3_calibrate_ = self.create_publisher(Float32, 'joint3/calibrate', 10)

        self.subangle1_ = self.create_subscription(Float32, 'joint1/angle', self.angle_callback, 10)
        self.subangle2_ = self.create_subscription(Float32, 'joint2/angle', self.angle_callback, 10)
        self.subangle3_ = self.create_subscription(Float32, 'joint3/angle', self.angle_callback, 10)

def main(args=None):
    rclpy.init(args=args)
    node = Main_Processor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()