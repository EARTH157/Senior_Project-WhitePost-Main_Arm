#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
import time
from std_msgs.msg import Float32MultiArray, Float32

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')
        
        # =========================================================
        # ⚙️ 1. ตั้งค่า MUX Channel สำหรับเซนเซอร์ AS5600
        # =========================================================
        self.MUX_CH_J1 = 7   
        self.MUX_CH_J2 = 1   
        self.MUX_CH_J3 = 0   
        
        # =========================================================
        # ⚙️ 2. ตั้งค่า MUX Channel และช่องเสียบสำหรับ Servo (PCA9685)
        # =========================================================
        self.MUX_CH_SERVO = 2     
        self.SERVO_PIN_J4 = 0     
        self.SERVO_PIN_J5 = 1     
        
        # =========================================================
        # ⚙️ 3. ตั้งค่าการแสดงผลข้อความ (Debug Output)
        # =========================================================
        self.ENABLE_DEBUG = False  
        self.DEBUG_INTERVAL = 50  
        
        # 🟢 ตัวแปรสำหรับ Auto-Reconnect
        self.ser = None
        self.last_reconnect_time = 0.0
        
        # พยายามเชื่อมต่อครั้งแรก
        self.connect_serial()

        # Publisher สำหรับส่งค่าดิบของ AS5600 ให้แต่ละ Joint
        self.pub_raw_j1 = self.create_publisher(Float32, '/sensor/as5600/joint1', 10)
        self.pub_raw_j2 = self.create_publisher(Float32, '/sensor/as5600/joint2', 10)
        self.pub_raw_j3 = self.create_publisher(Float32, '/sensor/as5600/joint3', 10)

        # รับคำสั่งขยับกลีบมือมาจาก main_processor 
        self.create_subscription(Float32MultiArray, '/servo/set_angle', self.cb_set_servo, 10)

        self.debug_counter = self.DEBUG_INTERVAL

        # วนลูปอ่านข้อมูล 100Hz
        self.create_timer(0.01, self.read_serial_data)

    def connect_serial(self):
        """ ฟังก์ชันพยายามเชื่อมต่อ USB """
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            self.get_logger().info(f"✅ เชื่อมต่อ ESP32 สำเร็จที่ {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().warn(f"⏳ ไม่สามารถเชื่อมต่อ ESP32 ได้ ({e}) กำลังพยายามเชื่อมต่อใหม่...", throttle_duration_sec=2.0)
            self.ser = None

    def read_serial_data(self):
        """ อ่านค่า Devices Map และ AS5600 จาก ESP32 """
        # 🟢 1. ถ้ายังเชื่อมต่อไม่ได้ ให้พยายามต่อใหม่ทุกๆ 2 วินาที
        if self.ser is None or not self.ser.is_open:
            current_time = time.time()
            if current_time - self.last_reconnect_time > 2.0:
                self.last_reconnect_time = current_time
                self.connect_serial()
            return

        # 🟢 2. อ่านข้อมูล พร้อมดักจับ Error กรณีสายหลุดกลางคัน
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or not line.startswith("{"):
                    return

                data = json.loads(line)
                devices = data.get("devices", {})
                angles = data.get("as5600", [])
                
                str_j1 = str(self.MUX_CH_J1)
                str_j2 = str(self.MUX_CH_J2)
                str_j3 = str(self.MUX_CH_J3)

                j1_ok = (devices.get(str_j1) == "as5600" and len(angles) > self.MUX_CH_J1)
                j2_ok = (devices.get(str_j2) == "as5600" and len(angles) > self.MUX_CH_J2)
                j3_ok = (devices.get(str_j3) == "as5600" and len(angles) > self.MUX_CH_J3)

                if not (j1_ok and j2_ok and j3_ok):
                    self.get_logger().error("🚨 SENSOR DROP DETECTED! สายเซ็นเซอร์ภายในหลุด! หยุดส่งพิกัดเพื่อล็อคแขนกล!", throttle_duration_sec=1.0)
                    return 

                self.pub_raw_j1.publish(Float32(data=float(angles[self.MUX_CH_J1])))
                self.pub_raw_j2.publish(Float32(data=float(angles[self.MUX_CH_J2])))
                self.pub_raw_j3.publish(Float32(data=float(angles[self.MUX_CH_J3])))

                if self.ENABLE_DEBUG:
                    self.debug_counter += 1
                    if self.debug_counter >= self.DEBUG_INTERVAL:
                        val_j1 = angles[self.MUX_CH_J1] if len(angles) > self.MUX_CH_J1 else 'N/A'
                        val_j2 = angles[self.MUX_CH_J2] if len(angles) > self.MUX_CH_J2 else 'N/A'
                        val_j3 = angles[self.MUX_CH_J3] if len(angles) > self.MUX_CH_J3 else 'N/A'
                        
                        msg = (
                            f"\n--- [ESP32 Sensor Status] ---\n"
                            f"📍 Joint 1 (Mux {self.MUX_CH_J1}): {val_j1} unit\n"
                            f"📍 Joint 2 (Mux {self.MUX_CH_J2}): {val_j2} unit\n"
                            f"📍 Joint 3 (Mux {self.MUX_CH_J3}): {val_j3} unit\n"
                            f"🔍 Mux Devices: {devices}\n"
                            f"-----------------------------"
                        )
                        self.get_logger().info(msg)
                        self.debug_counter = 0

        # 🟢 ดักจับ Error OSError กรณีสาย USB หลุด
        except (OSError, serial.SerialException) as e:
            self.get_logger().error(f"🚨 สาย USB ของ ESP32 หลุด! กำลังเข้าสู่โหมดรอการเชื่อมต่อใหม่... ({e})")
            if self.ser:
                self.ser.close()
            self.ser = None # รีเซ็ตพอร์ต เพื่อให้มันต่อใหม่ในลูปหน้า
            
        except json.JSONDecodeError:
            pass
        except Exception as e:
            pass

    def cb_set_servo(self, msg):
        """ Pi สั่งการให้ Servo ขยับ (ส่งคำสั่งไป ESP32) """
        if self.ser is None or not self.ser.is_open:
            return # ถ้าสายหลุดอยู่ ไม่ต้องทำอะไร
            
        data = msg.data
        if len(data) >= 5:
            angle_j4 = int(data[2])
            angle_j5 = int(data[4])
            
            cmd_j4 = f"S{self.MUX_CH_SERVO}:{self.SERVO_PIN_J4}:{angle_j4}\n"
            cmd_j5 = f"S{self.MUX_CH_SERVO}:{self.SERVO_PIN_J5}:{angle_j5}\n"
            
            try:
                self.ser.write(cmd_j4.encode('utf-8'))
                self.ser.write(cmd_j5.encode('utf-8'))
                
                if self.ENABLE_DEBUG:
                    self.get_logger().info(
                        f"📤 [CMD SERVO] สั่งงานไปที่ข้อมือ (J4): {angle_j4}° | กลีบมือ (J5): {angle_j5}°"
                    )
            except (OSError, serial.SerialException):
                self.get_logger().error("🚨 เขียนข้อมูลไป ESP32 ไม่ได้ สาย USB อาจจะหลุด!")
                self.ser.close()
                self.ser = None

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser is not None and self.ser.is_open:
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