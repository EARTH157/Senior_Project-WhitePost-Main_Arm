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
        
        # =========================================================
        # ⚙️ 1. ตั้งค่า MUX Channel สำหรับเซนเซอร์ AS5600
        # =========================================================
        self.MUX_CH_J1 = 7   # ให้ Joint 1 อ่านข้อมูลจาก MUX ช่อง 7
        self.MUX_CH_J2 = 1   # ให้ Joint 2 อ่านข้อมูลจาก MUX ช่อง 1
        self.MUX_CH_J3 = 0   # ให้ Joint 3 อ่านข้อมูลจาก MUX ช่อง 0
        
        # =========================================================
        # ⚙️ 2. ตั้งค่า MUX Channel และช่องเสียบสำหรับ Servo (PCA9685)
        # =========================================================
        self.MUX_CH_SERVO = 2     # ให้บอร์ด PCA9685 เสียบอยู่ที่ MUX ช่อง 2
        self.SERVO_PIN_J4 = 0     # Joint 4 (ข้อมือ) ต่อกับ Servo ขาที่ 0
        self.SERVO_PIN_J5 = 1     # Joint 5 (กลีบมือ) ต่อกับ Servo ขาที่ 1
        # =========================================================

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            self.get_logger().info(f"✅ เชื่อมต่อ ESP32 สำเร็จที่ {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().fatal(f"🛑 Error: {e}")
            raise SystemExit

        # Publisher สำหรับส่งค่าดิบของ AS5600 ให้แต่ละ Joint
        self.pub_raw_j1 = self.create_publisher(Float32, '/sensor/as5600/joint1', 10)
        self.pub_raw_j2 = self.create_publisher(Float32, '/sensor/as5600/joint2', 10)
        self.pub_raw_j3 = self.create_publisher(Float32, '/sensor/as5600/joint3', 10)

        # รับคำสั่งขยับกลีบมือมาจาก main_processor 
        self.create_subscription(Float32MultiArray, '/servo/set_angle', self.cb_set_servo, 10)

        # ตัวแปรสำหรับลดความถี่ในการ Print ลง Terminal
        self.debug_counter = 0

        # วนลูปอ่านข้อมูล 100Hz
        self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        """ อ่านค่า Devices Map และ AS5600 จาก ESP32 """
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or not line.startswith("{"):
                    return

                data = json.loads(line)
                devices = data.get("devices", {})
                angles = data.get("as5600", [])
                
                # แปลงตัวแปรช่องเป็น String เพื่อเช็คชนิดอุปกรณ์
                str_j1 = str(self.MUX_CH_J1)
                str_j2 = str(self.MUX_CH_J2)
                str_j3 = str(self.MUX_CH_J3)

                # 🚨 [SAFETY E-STOP] ตรวจสอบว่าเซ็นเซอร์ AS5600 ทั้ง 3 ตัวเชื่อมต่ออยู่ครบหรือไม่
                j1_ok = (devices.get(str_j1) == "as5600" and len(angles) > self.MUX_CH_J1)
                j2_ok = (devices.get(str_j2) == "as5600" and len(angles) > self.MUX_CH_J2)
                j3_ok = (devices.get(str_j3) == "as5600" and len(angles) > self.MUX_CH_J3)

                # ถ้ามีตัวใดตัวหนึ่งหายไป ให้หยุดส่งข้อมูลทั้งหมด!
                if not (j1_ok and j2_ok and j3_ok):
                    self.get_logger().error("🚨 SENSOR DROP DETECTED! สายเซ็นเซอร์หลุด! หยุดส่งพิกัดเพื่อล็อคแขนกล!", throttle_duration_sec=1.0)
                    return # ข้ามการทำงานไปเลย เพื่อให้ฝั่ง Joint เกิด Timeout พร้อมกัน

                # ตรวจสอบและ Publish ข้อมูล AS5600 (ตามปกติถ้าครบ 3 ตัว)
                self.pub_raw_j1.publish(Float32(data=float(angles[self.MUX_CH_J1])))
                self.pub_raw_j2.publish(Float32(data=float(angles[self.MUX_CH_J2])))
                self.pub_raw_j3.publish(Float32(data=float(angles[self.MUX_CH_J3])))

                # ---------------------------------------------------------
                # 🐛 SECTION DEBUG: Print โชว์สถานะทุกๆ 50 รอบ (ประมาณ 0.5 วิ)
                # ---------------------------------------------------------
                self.debug_counter += 1
                if self.debug_counter >= 50:
                    val_j1 = angles[self.MUX_CH_J1] if len(angles) > self.MUX_CH_J1 else 'N/A'
                    val_j2 = angles[self.MUX_CH_J2] if len(angles) > self.MUX_CH_J2 else 'N/A'
                    val_j3 = angles[self.MUX_CH_J3] if len(angles) > self.MUX_CH_J3 else 'N/A'
                    
                    self.get_logger().info(
                        f"📥 [อ่านค่า] J1(Ch{self.MUX_CH_J1}): {val_j1} | "
                        f"J2(Ch{self.MUX_CH_J2}): {val_j2} | "
                        f"J3(Ch{self.MUX_CH_J3}): {val_j3} | "
                        f"Mux Status: {devices}"
                    )
                    self.debug_counter = 0

            except json.JSONDecodeError:
                pass
            except Exception as e:
                pass

    def cb_set_servo(self, msg):
        """ Pi สั่งการให้ Servo ขยับ (ส่งคำสั่งไป ESP32) """
        data = msg.data
        
        # รูปแบบจาก main_processor ส่งมาคือ [mux_เดิม, ch1_เดิม, angle_j4, ch2_เดิม, angle_j5]
        # เราจะดึงแค่องศามาใช้ (Index 2 และ 4) แล้วให้มันวิ่งไปตามช่องที่เราตั้งค่าไว้ด้านบนแทน!
        if len(data) >= 5:
            angle_j4 = int(data[2])
            angle_j5 = int(data[4])
            
            # สร้างคำสั่งส่งให้ ESP32
            cmd_j4 = f"S{self.MUX_CH_SERVO}:{self.SERVO_PIN_J4}:{angle_j4}\n"
            cmd_j5 = f"S{self.MUX_CH_SERVO}:{self.SERVO_PIN_J5}:{angle_j5}\n"
            
            self.ser.write(cmd_j4.encode('utf-8'))
            self.ser.write(cmd_j5.encode('utf-8'))
            
            # ---------------------------------------------------------
            # 🐛 SECTION DEBUG: Print ทันทีเมื่อมีการสั่งงาน Servo
            # ---------------------------------------------------------
            self.get_logger().info(f"📤 [สั่ง Servo] -> J4(ข้อมือ): {cmd_j4.strip()} | J5(กลีบมือ): {cmd_j5.strip()}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
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