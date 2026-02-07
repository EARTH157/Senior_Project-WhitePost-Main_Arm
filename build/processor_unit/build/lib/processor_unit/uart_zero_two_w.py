import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialBridgePi5(Node):
    def __init__(self):
        super().__init__('serial_bridge_pi5')
        
        # ตั้งค่า UART
        self.serial_port = serial.Serial('/dev/ttyAMA1', 9600, timeout=0.1)
        
        # สร้าง Publisher เพื่อส่งข้อมูลที่ได้รับจาก UART เข้าสู่ระบบ ROS 2
        self.publisher_ = self.create_publisher(String, 'uart_rx', 10)
        
        # สร้าง Subscriber เพื่อรับข้อมูลจาก ROS 2 แล้วส่งออกทาง UART
        self.subscription = self.create_subscription(
            String,
            'uart_tx',
            self.uart_tx_callback,
            10)
        
        # สร้าง Timer เพื่อตรวจสอบข้อมูลขาเข้าจาก UART ทุกๆ 0.1 วินาที
        self.timer = self.create_timer(0.1, self.uart_rx_check)
        
        self.get_logger().info('Serial Bridge Pi 5 (UART 1) has started.')

    def uart_tx_callback(self, msg):
        """เมื่อมีข้อมูลเข้ามาที่ Topic 'uart_tx' ให้ส่งออกไปที่ UART"""
        self.serial_port.write((msg.data + '\n').encode('utf-8'))
        self.get_logger().info(f'Sent to UART: {msg.data}')

    def uart_rx_check(self):
        """ตรวจสอบข้อมูลที่ได้รับจาก UART"""
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Received from UART: {line}')
            except Exception as e:
                self.get_logger().error(f'Read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgePi5()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()