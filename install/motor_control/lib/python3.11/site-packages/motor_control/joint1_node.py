#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import RPi.GPIO as GPIO
import smbus2
import time
import fcntl
import sys
import threading
import json
import os

# ==========================================
# ⚙️ CONFIGURATION
# ==========================================
PIN_ENA = 26
PIN_DIR = 6
PIN_PUL = 5
PIN_LIMIT = 18
ENA_ACTIVE_HIGH = False 

# I2C Sensor (AS5600)
I2C_BUS_ID = 1
MUX_ADDR = 0x70
AS5600_ADDR = 0x36
MUX_CHANNEL = 1

# PID Parameters
KP = 1.0
KI = 0.1
KD = 0.01    

# Speed Settings
MIN_DELAY = 0.0005   
MAX_DELAY = 0.001  
PULSE_WIDTH = 0.00005

ANGLE_TOLERANCE = 0.15
WARN_DIFF_THRESHOLD = 0.5  # ยอมให้คลาดเคลื่อนได้ 0.5 องศา ถ้าเกินนี้ต้อง Homing ใหม่
STATE_FILE = "joint1_last_state.json" 

class Joint1Driver(Node):
    def __init__(self):
        super().__init__('joint1_driver_node')
        
        self.lock_file = open('/tmp/raspi_i2c_lock', 'w+')
        
        self.zero_offset = 0.0
        self.is_homed = False
        self.current_target = None 
        
        # PID Variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = time.time()

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_ENA, GPIO.OUT, initial=GPIO.HIGH) # ปิดไว้ก่อน
        GPIO.setup(PIN_DIR, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_PUL, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_LIMIT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Setup I2C
        try:
            self.bus = smbus2.SMBus(I2C_BUS_ID)
            if self.read_as5600() is None: raise Exception("Sensor Error")
            self.get_logger().info("✅ Sensor OK.")
        except Exception as e:
            self.get_logger().fatal(f"🛑 Sensor Failed: {e}")
            sys.exit(1)

        # ROS Setup
        self.create_subscription(Float32, 'joint1/set_target_angle', self.target_callback, 10)
        self.create_subscription(Bool, 'joint1/calibrate', self.calibrate_callback, 10)
        self.angle_pub = self.create_publisher(Float32, 'joint1/angle', 10)
        self.create_timer(0.5, self.report_status)

        # -----------------------------------------------------
        # 🔥 ตรวจสอบค่าเดิม (Auto-Resume Logic)
        # -----------------------------------------------------
        self.check_initial_position()

        # เริ่ม Thread ควบคุม
        self.running = True
        self.worker_thread = threading.Thread(target=self.control_loop_worker, daemon=True)
        self.worker_thread.start()

        if not self.is_homed:
            self.get_logger().info("⏳ Waiting for calibration command... (Topic: /joint1/calibrate)")

    # ---------------------------------------------------------
    # 💾 STATE CHECKING
    # ---------------------------------------------------------
    def check_initial_position(self):
        """
        เช็คตำแหน่ง:
        - ถ้าตำแหน่งเดิมปลอดภัย -> Load ค่า Offset -> is_homed = True เลย
        - ถ้าตำแหน่งเปลี่ยน -> is_homed = False -> รอ Calibrate
        """
        current_raw = self.read_as5600()
        if current_raw is None: return

        if os.path.exists(STATE_FILE):
            try:
                with open(STATE_FILE, 'r') as f:
                    data = json.load(f)
                    last_raw = data.get('last_raw_angle', -1)
                    saved_offset = data.get('zero_offset', 0.0) # โหลดค่า Offset เดิมด้วย
                
                if last_raw != -1:
                    # คำนวณความต่าง
                    diff = abs(current_raw - last_raw)
                    if diff > 180: diff = 360 - diff

                    if diff <= WARN_DIFF_THRESHOLD:
                        # ✅ เคสปลอดภัย: มุมไม่เปลี่ยน
                        self.get_logger().info(f"✅ Position Verified (Diff: {diff:.2f}°). Resuming...")
                        
                        # 1. คืนค่า Zero Offset เดิม
                        self.zero_offset = saved_offset
                        
                        # 2. ตั้งค่าสถานะเป็น Homed แล้ว
                        self.is_homed = True
                        
                        # 3. ตั้ง Target เป็นมุมปัจจุบันทันที (ให้ Motor Hold ตำแหน่งนี้)
                        current_angle = current_raw - self.zero_offset
                        self.current_target = current_angle
                        
                        # 4. จ่ายไฟเข้ามอเตอร์
                        self.enable_motor(True)
                        self.get_logger().info(f"🚀 System READY! Holding at {current_angle:.2f}°")
                        
                    else:
                        # ❌ เคสอันตราย: มุมเปลี่ยนไปเยอะ (อาจมีคนไปหมุนตอนปิดเครื่อง)
                        self.get_logger().warn(f"⚠️ MOVED WHILE OFF! (Diff: {diff:.2f}°)")
                        self.get_logger().warn("   Safety Lock: PLEASE RE-CALIBRATE.")
                        self.is_homed = False # บังคับ Calibrate ใหม่
            except Exception as e:
                self.get_logger().error(f"Failed to load state: {e}")
        else:
            self.get_logger().info("ℹ️ No previous state file found. Calibration required.")

    def save_current_state(self):
        """บันทึกค่า Sensor และ Offset ก่อนปิดโปรแกรม"""
        current_raw = self.read_as5600()
        if current_raw is not None:
            # บันทึกทั้ง Raw Angle และ Zero Offset
            data = {
                'last_raw_angle': current_raw,
                'zero_offset': self.zero_offset
            }
            try:
                with open(STATE_FILE, 'w') as f:
                    json.dump(data, f)
                self.get_logger().info(f"💾 State saved. Raw: {current_raw:.2f}, Offset: {self.zero_offset:.2f}")
            except Exception as e:
                self.get_logger().error(f"Failed to save state: {e}")

    # ---------------------------------------------------------
    # 📨 ROS CALLBACKS
    # ---------------------------------------------------------
    def calibrate_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("🔄 Manual Calibration Requested.")
            threading.Thread(target=self.sequence_homing_wrapper, daemon=True).start()

    def sequence_homing_wrapper(self):
        self.is_homed = False
        self.enable_motor(True)
        self.perform_homing_sequence()

    def target_callback(self, msg: Float32):
        if not self.is_homed:
            self.get_logger().warn("⚠️ Ignoring target: Need Calibration first!")
            return
            
        self.current_target = msg.data
        self.get_logger().info(f"🎯 Target Updated: {self.current_target:.2f}")
        
        # Reset PID parameters to prevent jump
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = time.time()

    # ---------------------------------------------------------
    # ⏱️ CONTROL LOOP (แก้ไขเพิ่ม Safety)
    # ---------------------------------------------------------
    def precise_delay(self, duration):
        start = time.perf_counter()
        while time.perf_counter() - start < duration: pass

    def step_pulse_single(self, direction_sign, delay):
        dir_val = GPIO.HIGH if direction_sign > 0 else GPIO.LOW
        GPIO.output(PIN_DIR, dir_val)
        GPIO.output(PIN_PUL, GPIO.HIGH)
        self.precise_delay(PULSE_WIDTH) 
        GPIO.output(PIN_PUL, GPIO.LOW)
        wait_time = delay - PULSE_WIDTH
        if wait_time < 0: wait_time = 0
        self.precise_delay(wait_time)

    def control_loop_worker(self):
        while self.running and rclpy.ok():
            # ถ้ายังไม่ Homed และไม่มี Target ให้รอ
            if not self.is_homed or self.current_target is None:
                time.sleep(0.1)
                continue

            current_angle = self.get_calibrated_angle()
            if current_angle is None: 
                time.sleep(0.01); continue

            error = self.current_target - current_angle
            
            # Deadband
            if abs(error) <= ANGLE_TOLERANCE:
                self.integral = 0.0
                self.prev_error = 0.0
                time.sleep(0.01)
                continue 

            # PID Logic
            now = time.time()
            dt = now - self.last_pid_time
            if dt == 0: dt = 0.001

            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            pid_output = (KP * error) + (KI * self.integral) + (KD * derivative)
            
            self.prev_error = error
            self.last_pid_time = now

            direction = 1 if pid_output > 0 else -1
            speed_mag = abs(pid_output)
            
            if speed_mag > 0.01:
                calc_delay = MAX_DELAY - (speed_mag / 500.0)
            else:
                calc_delay = MAX_DELAY
            
            if calc_delay < MIN_DELAY: calc_delay = MIN_DELAY
            if calc_delay > MAX_DELAY: calc_delay = MAX_DELAY

            # ------------------------------------------------------------------
            # 🛡️ SAFETY CHECK: LIMIT SWITCH PROTECTION
            # ------------------------------------------------------------------
            # สมมติฐาน: Homing วิ่งทิศ -1 เข้าหา Limit (ตามฟังก์ชัน homing sequence)
            # ดังนั้นถ้าชน Limit:
            #   - ห้ามวิ่ง -1 (เข้าหาสวิตช์)
            #   - ยอมให้วิ่ง +1 (ถอยออกจากสวิตช์)
            # ------------------------------------------------------------------
            if GPIO.input(PIN_LIMIT) == 0:  # 0 คือถูกกด (NC/NO แล้วแต่วงจร แต่ใน homing ใช้ 0=ชน)
                if direction == -1: 
                    # 🛑 กรณีพยายามวิ่งเข้าหา Limit ทั้งที่ชนอยู่ -> หยุดทันที!
                    self.integral = 0.0   # Reset Integral ไม่ให้แรงสะสม
                    self.prev_error = 0.0
                    
                    # Log เตือน (แบบไม่รัวเกินไป)
                    if int(time.time()) % 2 == 0: 
                        print(f"🛑 LIMIT HIT! Blocking MOVE IN (-). Angle: {current_angle:.2f}", end='\r')
                    
                    time.sleep(0.01)
                    continue  # 🚫 ข้ามการสั่ง Pulse รอบนี้ไปเลย
                
                else:
                    # ✅ กรณีวิ่ง +1 (ถอยออก) -> ยอมให้ทำได้
                    pass 

            # สั่ง Pulse เมื่อผ่านเงื่อนไขความปลอดภัย
            self.step_pulse_single(direction, calc_delay)

    # ---------------------------------------------------------
    # 📐 SENSOR
    # ---------------------------------------------------------
    def get_calibrated_angle(self):
        raw = self.read_as5600()
        if raw is not None:
            return raw - self.zero_offset
        return None

    # ---------------------------------------------------------
    # 📐 SENSOR (UPDATED WITH RETRY LOGIC)
    # ---------------------------------------------------------
    def read_as5600(self):
        real_angle = None
        
        # [3] เริ่มจองคิว (Lock)
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        try:
            # 1. เลือกช่อง MUX (Write)
            self.bus.write_byte(0x70, 1 << 1)
                
            # 2. อ่านค่า Angle High/Low (Read)
            hi = self.bus.read_byte_data(AS5600_ADDR, 0x0E) & 0x0F
            lo = self.bus.read_byte_data(AS5600_ADDR, 0x0F)
            current_raw = (hi << 8) | lo
                
            # ==========================================
            # ⚙️ คำนวณค่ามุม (Logic เดิมของคุณ)
            # ==========================================
            RAW_AT_0_DEG  = 585.0   
            RAW_AT_90_DEG = 1720.0   
                
            slope = (90.0 - 0.0) / (RAW_AT_90_DEG - RAW_AT_0_DEG)
            real_angle = slope * (current_raw - RAW_AT_0_DEG) + 0.0
    
        except Exception as e:
            self.get_logger().error(f"I2C Read Error: {e}")
            pass # หรือ Log error
        finally:
            # [4] คืนบัตรคิว (Unlock)
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)

        return real_angle

    def report_status(self):
        angle = self.get_calibrated_angle()
        if angle is not None and self.is_homed:
            msg = Float32()
            msg.data = angle
            self.angle_pub.publish(msg)
            tgt_str = f"{self.current_target:.2f}" if self.current_target is not None else "None"
            
            # เพิ่มสถานะ Limit ใน Print
            lim_status = "🛑HIT" if GPIO.input(PIN_LIMIT) == 0 else "OK"
            print(f"✅ Run | Tgt: {tgt_str} | Cur: {angle:.2f} | Lim: {lim_status}   ", end='\r')
            
        elif not self.is_homed:
            print(f"⚠️ Wait Calib | Raw: {self.read_as5600():.2f} ", end='\r')

    # ---------------------------------------------------------
    # 🏠 HOMING & CLEANUP
    # ---------------------------------------------------------
    def perform_homing_sequence(self):
        self.get_logger().info("🏠 Homing Started...")
        HOMING_DELAY = 0.001 
        
        # 1. ถอยถ้าชน
        if GPIO.input(PIN_LIMIT) == 0:
            for _ in range(500): self.step_pulse_single(1, HOMING_DELAY)
            
        # 2. วิ่งเข้าหา
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(-1, HOMING_DELAY)
            
        # 3. ถอยออก
        time.sleep(0.5)
        for _ in range(500): self.step_pulse_single(1, HOMING_DELAY)
        time.sleep(0.5)
        
        # 4. วิ่งเข้าหาช้าๆ
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(-1, HOMING_DELAY * 2)
            
        # 5. Set Zero
        val = self.read_as5600()
        if val: 
            self.zero_offset = val
            self.is_homed = True
            self.current_target = 0.0
            self.get_logger().info(f"✅ Homing Done. New Offset: {self.zero_offset:.2f}")
        else:
            self.get_logger().error("❌ Homing Failed: Sensor Error")

    def enable_motor(self, enable):
        GPIO.output(PIN_ENA, GPIO.HIGH if enable == ENA_ACTIVE_HIGH else GPIO.LOW)

    def destroy_node(self):
        self.get_logger().info("🛑 Shutting down...")
        self.save_current_state()
        
        self.running = False
        if hasattr(self, 'worker_thread'):
            self.worker_thread.join()
        self.enable_motor(False)
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = Joint1Driver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()