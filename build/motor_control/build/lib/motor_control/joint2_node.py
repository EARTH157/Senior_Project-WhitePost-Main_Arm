#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import RPi.GPIO as GPIO
import fcntl
import smbus2
import time
import sys
import threading
import json
import os

# ==========================================
# ⚙️ CONFIGURATION
# ==========================================
PIN_ENA = 11
PIN_DIR = 9
PIN_PUL = 10
PIN_LIMIT = 20
ENA_ACTIVE_HIGH = False 

# I2C Sensor (AS5600)
I2C_BUS_ID = 1
MUX_ADDR = 0x70
AS5600_ADDR = 0x36
MUX_CHANNEL = 2  # Joint 2 Channel

# PID Parameters
KP = 0.8   # เพิ่มเพื่อให้ตอบสนองแรงขึ้น (เดิม 0.50)
KI = 0.20   # ลดลงก่อน กันการแกว่งสะสม (เดิม 0.39)
KD = 0.01   # เพิ่มแรงต้านการแกว่ง (เดิม 0.01)   

# Speed Settings
# เพิ่ม Delay เพื่อให้หมุนช้าลง แต่แรงบิดจะเยอะขึ้น
MIN_DELAY = 0.0005   # จากเดิม 0.0003 (เร็วไป) -> ลองปรับเป็น 0.0010 หรือ 0.0020
MAX_DELAY = 0.001   # จากเดิม 0.0008 -> ปรับให้กว้างขึ้น
PULSE_WIDTH = 0.00005 # เพิ่มความกว้าง Pulse นิดหน่อยให้ Driver อ่านทันแน่นอน

ANGLE_TOLERANCE = 1.0
WARN_DIFF_THRESHOLD = 0.5  
STATE_FILE = "joint2_last_state.json" 

# 📐 LIMIT SETTINGS
MIN_ANGLE_LIMIT = 0.0    # องศาต่ำสุด
MAX_ANGLE_LIMIT = 180.0  # องศาสูงสุด (ตำแหน่ง Limit Switch)

class Joint2Driver(Node):
    def __init__(self):
        super().__init__('joint2_driver_node')
        
        self.lock_file = open('/tmp/raspi_i2c_lock', 'w+')
        
        self.zero_offset = 0.0
        self.is_homed = False
        self.current_target = None 
        
        # PID Variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = time.time()
        
        # 🔥 เพิ่มตัวแปรสำหรับทำ Acceleration Ramp (Soft Start)
        self.current_delay = MAX_DELAY

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # 🔥 แก้ตรงนี้: เปลี่ยน initial เป็น LOW เพื่อให้มอเตอร์มี Holding Torque ทันที
        GPIO.setup(PIN_ENA, GPIO.OUT, initial=GPIO.LOW) 
        
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
        self.create_subscription(Float32, 'joint2/set_target_angle', self.target_callback, 10)
        self.create_subscription(Bool, 'joint2/calibrate', self.calibrate_callback, 10)
        self.angle_pub = self.create_publisher(Float32, 'joint2/angle', 10)
        self.calibration_pub = self.create_publisher(Bool, 'joint2/calibrated', 10)
        self.create_timer(0.5, self.report_status)

        # Auto-Resume Logic
        self.check_initial_position()

        # ตัวแปรควบคุมความเร็วและทิศทางแบบใหม่
        self.target_hz = 0.0
        self.current_hz = 0.0
        self.motor_direction = 1
        
        # เริ่ม Thread ควบคุมแยกส่วน (Motor และ PID)
        self.running = True
        self.motor_thread = threading.Thread(target=self.motor_worker, daemon=True)
        self.pid_thread = threading.Thread(target=self.pid_worker, daemon=True)
        self.motor_thread.start()
        self.pid_thread.start()

        if not self.is_homed:
            self.get_logger().info("⏳ Waiting for calibration command... (Topic: /joint1/calibrate)")

    # ---------------------------------------------------------
    # 💾 STATE CHECKING
    # ---------------------------------------------------------
    def check_initial_position(self):
        current_raw = self.read_as5600()
        if current_raw is None: return

        if os.path.exists(STATE_FILE):
            try:
                with open(STATE_FILE, 'r') as f:
                    data = json.load(f)
                    last_raw = data.get('last_raw_angle', -1)
                    saved_offset = data.get('zero_offset', 0.0)
                
                if last_raw != -1:
                    diff = abs(current_raw - last_raw)
                    if diff > 180: diff = 360 - diff

                    if diff <= WARN_DIFF_THRESHOLD:
                        self.get_logger().info(f"✅ Position Verified (Diff: {diff:.2f}°). Resuming...")
                        self.zero_offset = saved_offset
                        self.is_homed = True
                        
                        current_angle = current_raw - self.zero_offset
                        self.current_target = current_angle
                        
                        self.enable_motor(True)
                        self.get_logger().info(f"🚀 System READY! Holding at {current_angle:.2f}°")
                    else:
                        # ❌ เคสอันตราย: มุมเปลี่ยนไปเยอะ (อาจมีคนไปหมุนตอนปิดเครื่อง)
                        self.get_logger().warn(f"⚠️ MOVED WHILE OFF! (Diff: {diff:.2f}°)")
                        self.get_logger().warn("   Safety Lock: PLEASE RE-CALIBRATE.")
                        calib_msg = Bool()
                        calib_msg.data = False
                        self.calibration_pub.publish(calib_msg)
                        self.is_homed = False # บังคับ Calibrate ใหม่
                        
                        # 🔥 เพิ่มตรงนี้: ถึงจะบังคับ Calibrate ใหม่ แต่ก็ต้องจ่ายไฟล็อกมอเตอร์กันร่วง!
                        self.enable_motor(True)
            except Exception as e:
                self.get_logger().error(f"Failed to load state: {e}")
        else:
            self.get_logger().info("ℹ️ No previous state file found. Calibration required.")

    def save_current_state(self):
        current_raw = self.read_as5600()
        if current_raw is not None:
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
        
        # 🔒 CLAMP TARGET: บังคับค่าให้อยู่ในช่วง 0 - 180 เสมอ
        raw_target = msg.data
        clamped_target = max(MIN_ANGLE_LIMIT, min(MAX_ANGLE_LIMIT, raw_target))
        
        if clamped_target != raw_target:
             self.get_logger().warn(f"⚠️ Target Out of Range ({raw_target}). Clamped to {clamped_target}")

        self.current_target = clamped_target
        self.get_logger().info(f"🎯 Target Updated: {self.current_target:.2f}")
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = time.time()

    # ---------------------------------------------------------
    # ⏱️ CONTROL LOOP (With Dual Safety)
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

    # ---------------------------------------------------------
    # 🏎️ MOTOR WORKER: รับหน้าที่หมุนมอเตอร์อย่างเดียวให้จังหวะแม่นยำ
    # ---------------------------------------------------------
    def motor_worker(self):
        while self.running and rclpy.ok():
            hz = self.current_hz
            
            # หากความถี่ต่ำมาก ให้ถือว่าหยุด (ประหยัด CPU)
            if hz < 50.0:
                time.sleep(0.005)
                continue
                
            # 🛡️ SAFETY CHECK: หยุดถ้าชน Limit Switch ในทิศวิ่งเข้า (-1)
            if GPIO.input(PIN_LIMIT) == 0 and self.motor_direction == 1:
                time.sleep(0.005)
                continue

            # แปลงความถี่ (Hz) เป็น Delay
            delay = 1.0 / hz
            self.step_pulse_single(self.motor_direction, delay)

    # ---------------------------------------------------------
    # 🧠 PID WORKER: คำนวณ Error และสร้างความเร่ง (Acceleration)
    # ---------------------------------------------------------
    def pid_worker(self):
        # --- ปรับจูนใหม่: เน้นแรงบิดสูง ช้าแต่มั่นคง ---
        MAX_HZ = 600.0        # ⬇️ ลดลงเยอะมาก! (อย่าเพิ่งเกิน 800) เพื่อให้มีแรงบิดยกแขน
        MIN_HZ = 50.0         # ⬇️ ออกตัวช้าๆ ให้แรงบิดช่วงเริ่มต้น (Holding Torque) ทำงานเต็มที่
        ACCEL_RATE = 40.0     # ⬇️ ค่อยๆ เร่งความเร็ว (Soft Start) ป้องกันการกระชากจนหลุดสเต็ป
        SPEED_MULTIPLIER = 20.0 # ⬇️ ลดความดุดันของ PID ลง
        
        while self.running and rclpy.ok():
            time.sleep(0.02) # รันที่ประมาณ 50Hz (ทุกๆ 20ms)

            if not self.is_homed or self.current_target is None:
                self.target_hz = 0.0
                self.current_hz = 0.0
                continue

            current_angle = self.get_calibrated_angle()
            if current_angle is None: continue

            error = self.current_target - current_angle

            # Deadband (ถึงเป้าหมายแล้ว)
            if abs(error) <= ANGLE_TOLERANCE:
                self.integral = 0.0
                self.prev_error = 0.0
                self.target_hz = 0.0
                
                # ค่อยๆ ลดความเร็วลงจนหยุด (Deceleration) ป้องกันกระตุกตอนจบ
                if self.current_hz > 0:
                    self.current_hz -= ACCEL_RATE * 2 
                    if self.current_hz < 0: self.current_hz = 0.0
                continue 

            # PID Logic
            now = time.time()
            dt = now - self.last_pid_time
            if dt <= 0: dt = 0.02

            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            pid_output = (KP * error) + (KI * self.integral) + (KD * derivative)
            
            self.prev_error = error
            self.last_pid_time = now

            # กำหนดทิศทางมอเตอร์
            self.motor_direction = 1 if pid_output > 0 else -1
            
            # แปลง PID Output เป็นเป้าหมายความถี่ (Target Hz)
            speed_mag = abs(pid_output)
            mapped_hz = MIN_HZ + (speed_mag * SPEED_MULTIPLIER) 
            if mapped_hz > MAX_HZ: mapped_hz = MAX_HZ
            
            self.target_hz = mapped_hz

            # 🚀 ทำ Acceleration Ramp: ปรับความเร็วปัจจุบันให้เข้าหาเป้าหมายอย่างนุ่มนวล
            if self.current_hz < self.target_hz:
                self.current_hz += ACCEL_RATE
                if self.current_hz > self.target_hz: self.current_hz = self.target_hz
            elif self.current_hz > self.target_hz:
                self.current_hz -= ACCEL_RATE
                if self.current_hz < self.target_hz: self.current_hz = self.target_hz

    # ---------------------------------------------------------
    # 📐 SENSOR
    # ---------------------------------------------------------
    def get_calibrated_angle(self):
        raw = self.read_as5600()
        if raw is not None:
            return raw - self.zero_offset
        return None

    def read_as5600(self):
        real_angle = None
        
        # [3] เริ่มจองคิว (Lock)
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        
        try:
            self.bus.write_byte(0x70, 1 << 2)
            hi = self.bus.read_byte_data(AS5600_ADDR, 0x0E) & 0x0F
            lo = self.bus.read_byte_data(AS5600_ADDR, 0x0F)
            current_raw = (hi << 8) | lo
            
            # ==========================================
            # ⚙️ 3-POINT CALIBRATION (Piecewise)
            # ==========================================
            # กำหนดจุดอ้างอิง 3 จุดตามที่คุณวัดจริง
            P1_RAW = 1871.0   # จุดที่ 0 องศา
            P2_RAW = 2715.0   # จุดที่ 90 องศา
            P3_RAW = 4065.0   # จุดที่ 180 องศา (Limit Switch)
            
            P1_ANG = 0.0
            P2_ANG = 90.0
            P3_ANG = 180.0
            
            real_angle = 0.0
            
            # 📐 คำนวณแบบแยกช่วง (Piecewise Interpolation)
            
            if current_raw <= P2_RAW:
                # --- [ช่วงที่ 1: 0 ถึง 90 องศา] ---
                # ใช้ความชันของช่วงแรก
                # สูตร: angle = 0 + (raw - 1883) * (90 / (2656-1883))
                slope1 = (P2_ANG - P1_ANG) / (P2_RAW - P1_RAW)
                real_angle = P1_ANG + slope1 * (current_raw - P1_RAW)
                
            else:
                # --- [ช่วงที่ 2: 90 ถึง 180 องศา] ---
                # ใช้ความชันของช่วงหลัง (ซึ่งชันน้อยกว่า)
                # สูตร: angle = 90 + (raw - 2656) * (90 / (4033-2656))
                slope2 = (P3_ANG - P2_ANG) / (P3_RAW - P2_RAW)
                real_angle = P2_ANG + slope2 * (current_raw - P2_RAW)

        except Exception as e:
            pass # หรือ Log error
        finally:
            # [4] คืนบัตรคิว (Unlock)
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)
            
        return real_angle # ส่งค่าที่คำนวณแล้วกลับไป

    def report_status(self):
        # -------------------------------------------------------------
        # 🔥 เพิ่มส่วนนี้: ส่งสถานะ Calibrate (True/False) ของ Joint 2 ออกไปทุกครั้ง
        # -------------------------------------------------------------
        cal_msg = Bool()
        cal_msg.data = self.is_homed
        self.calibration_pub.publish(cal_msg)
        # -------------------------------------------------------------

        angle = self.get_calibrated_angle()
        if angle is not None and self.is_homed:
            msg = Float32()
            msg.data = angle
            self.angle_pub.publish(msg)
            tgt_str = f"{self.current_target:.2f}" if self.current_target is not None else "None"
            
            # Status Flags
            lim_status = "HIT" if GPIO.input(PIN_LIMIT) == 0 else "OK"
            print(f"✅ Active | Tgt: {tgt_str} | Cur: {angle:.2f} | Switch: {lim_status} ", end='\r')
            
        elif not self.is_homed:
            raw = self.read_as5600()
            if raw is not None:
                print(f"⚠️ Wait Calib | Raw: {raw:.2f} ", end='\r')

    # ---------------------------------------------------------
    # 🏠 HOMING & CLEANUP
    # ---------------------------------------------------------
    def perform_homing_sequence(self):
        self.get_logger().info("🏠 Homing Started (Target: Limit=180°)...")
        HOMING_DELAY = 0.001 
        
        # Logic: วิ่งทิศ +1 ไปหา Switch ที่ตำแหน่ง 180 องศา
        
        # 1. ถ้าชนอยู่แล้ว ให้ถอยออกมา (ทิศ -1)
        if GPIO.input(PIN_LIMIT) == 0:
            time.sleep(1)
            self.get_logger().info("   - Already at limit, backing off...")
            for _ in range(500): self.step_pulse_single(-1, HOMING_DELAY)
            
        # 2. วิ่งเข้าหา (ทิศ 1)
        self.get_logger().info("   - Finding limit...")
        time.sleep(1)
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(1, HOMING_DELAY)
            
        # 3. ถอยออกนิดนึง (ทิศ -1)
        time.sleep(1.0)
        for _ in range(500): self.step_pulse_single(-1, HOMING_DELAY)
        time.sleep(1.0)
        
        # 4. วิ่งเข้าหาช้าๆ อีกรอบ (ทิศ 1)
        self.get_logger().info("   - Precision check...")
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(1, HOMING_DELAY * 2)
            
        # 5. Set 180 Degrees
        # คำนวณ: Target (180) = Raw - Offset
        # ดังนั้น: Offset = Raw - 180
        val = self.read_as5600()
        if val: 
            self.zero_offset = 0.0
            self.is_homed = True
            
            # เช็คความแม่นยำ (แค่ปริ้นดูเฉยๆ)
            current_check = self.get_calibrated_angle()
            self.get_logger().info(f"✅ Homing Done. Sensor reads: {current_check:.2f}° (Should be approx 180°)")
            
            # 🔥 Set Initial Target to 180 (Hold current position)
            self.current_target = 180.0
            
            self.get_logger().info(f"✅ Homing Done. Limit set to 180°. New Offset: {self.zero_offset:.2f}")
            self.get_logger().info("🚀 Holding Position at 180.0°")
        else:
            self.get_logger().error("❌ Homing Failed: Sensor Error")

    def enable_motor(self, enable):
        GPIO.output(PIN_ENA, GPIO.HIGH if enable == ENA_ACTIVE_HIGH else GPIO.LOW)

    def destroy_node(self):
        self.get_logger().info("🛑 Shutting down...")
        self.save_current_state()
        
        # ส่งสัญญาณให้ Threads ทั้งหมดหยุดทำงาน
        self.running = False
        
        # รอให้ Motor Thread ปิดตัวอย่างสมบูรณ์
        if hasattr(self, 'motor_thread'):
            self.motor_thread.join()
            
        # รอให้ PID Thread ปิดตัวอย่างสมบูรณ์
        if hasattr(self, 'pid_thread'):
            self.pid_thread.join()
            
        self.enable_motor(False)
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = Joint2Driver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()