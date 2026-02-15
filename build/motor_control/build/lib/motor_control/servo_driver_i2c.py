#!/usr/bin/env python3
import time
import math
import fcntl
from smbus2 import SMBus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

# ---------- Constants ----------
PCA_ADDR    = 0x40
MUX_ADDR    = 0x70
MODE1       = 0x00
PRESCALE    = 0xFE
LED0_ON_L   = 0x06

class ServoMuxNode(Node):
    def __init__(self):
        super().__init__('servo_mux_node')

        # [1] สร้างไฟล์ Lock (สำคัญ: ต้องสร้างก่อนทำ I2C)
        self.lock_file = open('/tmp/raspi_i2c_lock', 'w+')

        # --- Parameters ---
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('default_mux_channel', 0)
        
        self.bus_num = self.get_parameter('i2c_bus').value
        self.freq = self.get_parameter('frequency').value
        self.default_mux = self.get_parameter('default_mux_channel').value

        # --- State ---
        self.i2c = SMBus(self.bus_num)
        self.enabled = True
        self.current_mux = -1
        self.initialized_muxs = set() 

        self.get_logger().info(f"Servo Driver Ready on Bus {self.bus_num}, Default Mux: {self.default_mux}")
        
        # Init ช่อง Default
        self.activate_driver(self.default_mux)

        # --- Subscribers ---
        self.create_subscription(Float32MultiArray, '/servo/set_angle', self.cb_set_angle, 10)
        self.create_subscription(Bool, '/servo/enable', self.cb_enable, 10)

    # ---------- Low Level Hardware Control (With Lock) ----------
    def switch_mux(self, channel):
        """สับราง Mux แบบมี Lock"""
        if channel == self.current_mux:
            return
        
        # 🔒 LOCK
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        try:
            # ใช้ write_byte ธรรมดา เพราะเรามี Lock คุ้มกันแล้ว ไม่ต้อง Retry ซับซ้อน
            self.i2c.write_byte(MUX_ADDR, 1 << channel)
            self.current_mux = channel
        except Exception as e:
            self.get_logger().error(f"Mux Switch Error: {e}")
        finally:
            # 🔓 UNLOCK
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)

    def init_pca9685(self):
        """ตั้งค่า PCA9685 แบบมี Lock"""
        # 🔒 LOCK (ล็อคยาวจนกว่าจะตั้งค่าเสร็จ)
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        try:
            # ต้องสับ Mux อีกทีให้ชัวร์ว่าอยู่ใน Lock zone
            self.i2c.write_byte(MUX_ADDR, 1 << self.current_mux)
            
            # 1. Reset
            self.i2c.write_byte_data(PCA_ADDR, MODE1, 0x00)
            
            # 2. Set Frequency
            prescale_val = 25_000_000.0 / (4096.0 * self.freq) - 1.0
            prescale = int(round(prescale_val))
            
            oldmode = self.i2c.read_byte_data(PCA_ADDR, MODE1)
            newmode = (oldmode & 0x7F) | 0x10 
            
            self.i2c.write_byte_data(PCA_ADDR, MODE1, newmode) 
            self.i2c.write_byte_data(PCA_ADDR, PRESCALE, prescale) 
            self.i2c.write_byte_data(PCA_ADDR, MODE1, oldmode) 
            
            time.sleep(0.005)
            self.i2c.write_byte_data(PCA_ADDR, MODE1, oldmode | 0xA1) 
            
            self.get_logger().info(f"Initialized PCA9685 on Mux Ch {self.current_mux}")
        except Exception as e:
            self.get_logger().error(f"Failed to Init PCA: {e}")
        finally:
            # 🔓 UNLOCK
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)

    def activate_driver(self, mux_ch):
        self.switch_mux(mux_ch) # ตอนนี้ switch_mux มี lock แล้ว ปลอดภัย
        if mux_ch not in self.initialized_muxs:
            self.init_pca9685() # init ก็มี lock แล้ว ปลอดภัย
            self.initialized_muxs.add(mux_ch)

    def set_pwm(self, channel, on, off):
        """เขียนค่า PWM ลง Register แบบมี Lock"""
        # 🔒 LOCK
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        try:
            # --- เริ่มเขตห้ามแทรก ---
            # ต้องสับ Mux ใหม่ทุกครั้ง เพราะอาจโดน Joint อื่นแย่งไปแล้วระหว่างรอ Lock
            self.i2c.write_byte(MUX_ADDR, 1 << self.current_mux)
            
            # คำนวณค่าและส่งข้อมูล
            base_reg = LED0_ON_L + 4 * channel
            data = [
                on & 0xFF, (on >> 8) & 0x0F,
                off & 0xFF, (off >> 8) & 0x0F
            ]
            self.i2c.write_i2c_block_data(PCA_ADDR, base_reg, data)
            # --- จบเขตห้ามแทรก ---

        except Exception as e:
            self.get_logger().error(f"I2C Write Error: {e}")
        finally:
            # 🔓 UNLOCK
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)

    # ---------- Logic & Callbacks (ส่วนนี้เหมือนเดิม) ----------
    def angle_to_pulse(self, angle):
        min_pulse = 150 
        max_pulse = 600 
        angle = max(0.0, min(180.0, angle))
        pulse = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
        return int(pulse)

    def cb_enable(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            self.get_logger().info("Disabling Servos...")
            for m in list(self.initialized_muxs):
                self.switch_mux(m)
                for i in range(16):
                    self.set_pwm(i, 0, 0)
    
    def cb_set_angle(self, msg):
        if not self.enabled: return
        data = msg.data
        
        # กรณีส่งมาทีละ 1 ตัว (แบบเดิม) หรือแบบหลายๆ ตัว
        # รูปแบบใหม่ที่เราส่งมาคือ [Mux, Ch1, Ang1, Ch2, Ang2, ...]
        
        if len(data) >= 3 and len(data) % 2 == 1: 
            target_mux = int(data[0])
            self.activate_driver(target_mux)
            
            # วนลูปอ่านค่า (Channel, Angle) ทีละคู่
            for i in range(1, len(data), 2):
                servo_ch = int(data[i])
                angle = float(data[i+1])
                pulse = self.angle_to_pulse(angle)
                self.set_pwm(servo_ch, 0, pulse)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # เช็คก่อน shutdown เพื่อป้องกัน Error rcl_shutdown already called
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()