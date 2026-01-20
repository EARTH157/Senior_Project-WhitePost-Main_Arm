#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Joint1Processor(Node):
    def __init__(self):
        super().__init__('joint1_processor_node')
        #มุมที่เปลี่ยนแปลงจะถูกส่งไปที่ Driver ของ Joint 1

        # Publisher: ส่งคำสั่งมุมไปที่ Driver
        self.target_pub = self.create_publisher(Float32, 'joint1/set_target_angle', 10)
        
        # Subscriber: ฟังค่ามุมปัจจุบัน (เพื่อ Monitor)
        self.create_subscription(Float32, 'joint1/angle', self.angle_callback, 10)

        self.get_logger().info("🧠 Processor Ready. Sending commands to Joint 1 Driver.")
        
        # ตัวอย่าง: Timer ส่งคำสั่งทดสอบ (Optional: ลบออกได้ถ้าจะสั่งมือ)
        # self.create_timer(5.0, self.demo_sequence) 
        self.step_state = 0

    def angle_callback(self, msg):
        # รับรู้มุมปัจจุบัน แต่ยังไม่ได้ทำอะไรพิเศษ
        pass

    def send_angle(self, angle):
        msg = Float32()
        msg.data = float(angle)
        self.target_pub.publish(msg)
        self.get_logger().info(f"📤 Sent Command: Move to {angle:.2f} deg")

    # (ฟังก์ชันเสริม) ทดสอบส่งค่าไปมา
    def demo_sequence(self):
        targets = [90.0, 60.0, 30.0] # มุมที่จะวิ่งไป
        target = targets[self.step_state % len(targets)]
        self.send_angle(target)
        self.step_state += 1

def main(args=None):
    rclpy.init(args=args)
    node = Joint1Processor()
    
    # ถ้าอยากให้มันรอรับคำสั่งผ่าน Terminal (manual control)
    # สามารถใช้ ros2 topic pub หรือเขียน input() ตรงนี้ก็ได้
    # แต่วิธีมาตรฐานคือใช้ rclpy.spin แล้วรับ msg จาก Topic อื่น
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()