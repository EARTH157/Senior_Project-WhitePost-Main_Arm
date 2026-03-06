#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import Float32MultiArray, Float32

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')
        
        # 1. ตั้งค่าการเชื่อมต่อ Serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            self.get_logger().info(f"✅ เชื่อมต่อ ESP32 สำเร็จที่ {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"🛑 ไม่สามารถเชื่อมต่อพอร์ต Serial ได้: {e}")
            raise SystemExit

        # 2. สร้าง Publisher ส่งค่า Raw Sensor ไปให้ Joint 1, 2, 3
        self.pub_raw_j1 = self.create_publisher(Float32, '/sensor/as5600/joint1', 10)
        self.pub_raw_j2 = self.create_publisher(Float32, '/sensor/as5600/joint2', 10)
        self.pub_raw_j3 = self.create_publisher(Float32, '/sensor/as5600/joint3', 10)

        # 3. สร้าง Subscriber รับค่าสั่ง Servo จาก main_processor
        self.create_subscription(Float32MultiArray, '/servo/set_angle', self.cb_set_servo, 10)

        # 4. สร้าง Timer ไว้อ่านข้อมูลจาก ESP32 เร็วๆ (100Hz)
        self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        """ อ่านค่า JSON จาก ESP32 แล้วแตกส่งให้แต่ละ Joint """
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("{"):
                    data = json.loads(line)
                    if "as5600" in data:
                        angles = data["as5600"]
                        # สมมติว่า ESP32 ส่งมาเป็น Array: [Raw_J1, Raw_J2, Raw_J3]
                        if len(angles) >= 3:
                            self.pub_raw_j1.publish(Float32(data=float(angles[0])))
                            self.pub_raw_j2.publish(Float32(data=float(angles[1])))
                            self.pub_raw_j3.publish(Float32(data=float(angles[2])))
            except json.JSONDecodeError:
                pass
            except Exception as e:
                self.get_logger().warn(f"อ่าน Serial ผิดพลาด: {e}")

    def cb_set_servo(self, msg):
        """ รับคำสั่งจาก /servo/set_angle แล้วส่งไปให้ ESP32 """
        data = msg.data
        # รูปแบบเดิมจาก main_processor คือ [0.0, 0.0, angJ4, 1.0, angJ5]
        # โดย index 1,3 คือช่อง Servo (0, 1) และ index 2,4 คือมุม
        if len(data) >= 3 and len(data) % 2 == 1:
            for i in range(1, len(data), 2):
                channel = int(data[i])
                angle = int(data[i+1])
                
                # ส่งคำสั่งรูปแบบ S{channel}:{angle}\n
                command = f"S{channel}:{angle}\n"
                self.ser.write(command.encode('utf-8'))

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
