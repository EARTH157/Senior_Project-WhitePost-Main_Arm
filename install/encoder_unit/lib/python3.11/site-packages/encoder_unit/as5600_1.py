#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from smbus2 import SMBus

# ==========================================
# ส่วนตั้งค่า (แก้ไขตรงนี้ได้เลย)
# ==========================================
TARGET_CHANNEL = 2      # <--- ใส่เลขช่องที่ต้องการตรงนี้ (0 - 7)
MUX_ADDRESS    = 0x70   # ที่อยู่ I2C ของ Multiplexer
AS5600_ADDRESS = 0x36   # ที่อยู่ของ AS5600
I2C_BUS_ID     = 1      # Raspberry Pi ปกติใช้ Bus 1
PUBLISH_RATE   = 20.0   # ความถี่ในการอ่านค่า (Hz)
# ==========================================

class AS5600HardcodedNode(Node):
    def __init__(self):
        super().__init__('as5600_fixed_channel_node')

        self.target_channel = TARGET_CHANNEL
        self.mux_addr = MUX_ADDRESS
        self.as5600_addr = AS5600_ADDRESS

        # 1. เชื่อมต่อ I2C
        try:
            self.bus = SMBus(I2C_BUS_ID)
            self.get_logger().info(f"Connected to I2C Bus {I2C_BUS_ID}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to I2C: {e}")
            return

        # 2. สร้าง Publisher
        # Topic จะชื่อตามช่องที่เลือก เช่น /as5600_ch4/angle
        topic_name = f'as5600_ch{self.target_channel}/angle'
        self.publisher_ = self.create_publisher(Float32, topic_name, 10)

        # 3. สร้าง Timer
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)
        
        self.get_logger().info(f"Setup Complete: Reading MUX 0x{self.mux_addr:02X} -> Channel {self.target_channel}")

    def select_mux_channel(self):
        """สั่ง MUX ให้สลับไปช่องที่ตั้งค่าไว้"""
        try:
            # หลักการ: ส่ง byte ที่มีบิต 1 ตรงตำแหน่งช่องที่ต้องการ
            # เช่น ช่อง 0 = 0x01, ช่อง 1 = 0x02, ช่อง 7 = 0x80
            self.bus.write_byte(self.mux_addr, 1 << self.target_channel)
            return True
        except OSError as e:
            self.get_logger().warn(f"Mux Error (Ch {self.target_channel}): {e}")
            return False

    def read_raw_angle(self):
        """อ่านค่ามุมดิบ (0-4096)"""
        try:
            hi = self.bus.read_byte_data(self.as5600_addr, 0x0E) & 0x0F
            lo = self.bus.read_byte_data(self.as5600_addr, 0x0F)
            return (hi << 8) | lo
        except OSError as e:
            # บางทีอ่านพลาดเพราะสายหลวมหรือ Noise ก็จะข้ามไป
            # self.get_logger().warn(f"Read Error: {e}") 
            return None

    def timer_callback(self):
        # 1. ย้ำช่อง Mux ทุกครั้งก่อนอ่าน (เผื่อมีอุปกรณ์อื่นมาเปลี่ยนช่องไป)
        if not self.select_mux_channel():
            return

        # 2. อ่านค่าจาก AS5600
        raw_val = self.read_raw_angle()
        
        if raw_val is not None:
            # 3. แปลงเป็นองศา
            deg = raw_val * 360.0 / 4096.0
            
            # 4. ส่งค่าขึ้น ROS Topic
            msg = Float32()
            msg.data = deg
            self.publisher_.publish(msg)
            
            # (Optional) แสดงผลที่หน้าจอด้วย
            print(f"\rChannel {self.target_channel}: {deg:.2f} deg", end="")

    def __del__(self):
        if hasattr(self, 'bus'):
            self.bus.close()

def main(args=None):
    rclpy.init(args=args)
    node = AS5600HardcodedNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '_main_':
    main()
