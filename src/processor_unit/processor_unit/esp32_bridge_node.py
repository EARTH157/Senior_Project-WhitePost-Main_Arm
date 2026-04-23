#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
import time
from std_msgs.msg import Float32MultiArray, Float32, String # 🌟 เพิ่ม String เข้ามา

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')
        
        # --- ตั้งค่าช่องต่างๆ ---
        self.MUX_CH_J1 = 7   
        self.MUX_CH_J2 = 1   
        self.MUX_CH_J3 = 0   
        self.MUX_CH_SERVO = 2     
        self.SERVO_PIN_J4 = 0     
        self.SERVO_PIN_J5 = 1  
        self.SERVO_PIN_EXTRA = 2 
        
        self.ENABLE_DEBUG = True 
        self.ser = None
        self.last_reconnect_time = 0.0
        self.connect_serial()

        # Publishers
        self.pub_raw_j1 = self.create_publisher(Float32, '/sensor/as5600/joint1', 10)
        self.pub_raw_j2 = self.create_publisher(Float32, '/sensor/as5600/joint2', 10)
        self.pub_raw_j3 = self.create_publisher(Float32, '/sensor/as5600/joint3', 10)

        # Subscriptions
        self.create_subscription(Float32MultiArray, '/servo/set_angle', self.cb_set_servo, 10)
        self.create_subscription(Float32, '/servo/set_angle_terminal', self.cb_set_servo_terminal, 10)
        
        # 🌟 [NEW] รับคำสั่งแบบ String สำหรับ Sequence (เช่น "activate")
        self.create_subscription(String, '/servo/command', self.cb_sequence_command, 10)

        self.create_timer(0.01, self.read_serial_data)

    def connect_serial(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            self.get_logger().info(f"✅ เชื่อมต่อ ESP32 สำเร็จที่ {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().warn(f"⏳ ไม่สามารถเชื่อมต่อ ESP32 ได้ ({e})")
            self.ser = None

    def send_servo_raw(self, pin, angle):
        """ ฟังก์ชันกลางสำหรับส่งคำสั่งไป ESP32 """
        if self.ser and self.ser.is_open:
            cmd = f"S{self.MUX_CH_SERVO}:{pin}:{int(angle)}\n"
            try:
                self.ser.write(cmd.encode('utf-8'))
                return True
            except:
                return False
        return False

    # 🌟 [NEW] ฟังก์ชันจัดการคำสั่ง activate
    def cb_sequence_command(self, msg):
        command = msg.data.lower().strip()
        
        if command == "activate":
            self.get_logger().info("🚀 [SEQUENCE] เริ่มทำงาน: activate")
            
            # 1. ไปที่ 0 องศา
            self.get_logger().info("Step 1: กำลังไปที่ 0 องศา...")
            self.send_servo_raw(self.SERVO_PIN_EXTRA, 0)
            
            # 2. รอ 2 วินาที
            time.sleep(0.5)
            
            # 3. ไปที่ 90 องศา
            self.get_logger().info("Step 2: กำลังไปที่ 90 องศา...")
            self.send_servo_raw(self.SERVO_PIN_EXTRA, 90)
            
            self.get_logger().info("✅ [SEQUENCE] ทำงานเสร็จสิ้น")
        else:
            self.get_logger().warn(f"❓ ไม่รู้จักคำสั่ง: {command}")

    def read_serial_data(self):
        if self.ser is None or not self.ser.is_open:
            current_time = time.time()
            if current_time - self.last_reconnect_time > 2.0:
                self.last_reconnect_time = current_time
                self.connect_serial()
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or not line.startswith("{"): return
                data = json.loads(line)
                angles = data.get("as5600", [])
                if len(angles) > max(self.MUX_CH_J1, self.MUX_CH_J2, self.MUX_CH_J3):
                    self.pub_raw_j1.publish(Float32(data=float(angles[self.MUX_CH_J1])))
                    self.pub_raw_j2.publish(Float32(data=float(angles[self.MUX_CH_J2])))
                    self.pub_raw_j3.publish(Float32(data=float(angles[self.MUX_CH_J3])))
        except:
            pass

    def cb_set_servo(self, msg):
        data = msg.data
        if len(data) >= 5:
            self.send_servo_raw(self.SERVO_PIN_J4, data[2])
            self.send_servo_raw(self.SERVO_PIN_J5, data[4])

    def cb_set_servo_terminal(self, msg):
        self.send_servo_raw(self.SERVO_PIN_EXTRA, msg.data)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser is not None:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()