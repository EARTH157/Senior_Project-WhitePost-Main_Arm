#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import RPi.GPIO as GPIO
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
PIN_LIMIT = 23
ENA_ACTIVE_HIGH = False 

# I2C Sensor (AS5600)
I2C_BUS_ID = 1
MUX_ADDR = 0x70
AS5600_ADDR = 0x36
MUX_CHANNEL = 2  # Joint 2 Channel

# PID Parameters
KP = 0.50
KI = 0.39
KD = 0.01      

# Speed Settings
MIN_DELAY = 0.0003   
MAX_DELAY = 0.0010   
PULSE_WIDTH = 0.00001 

ANGLE_TOLERANCE = 2.0
WARN_DIFF_THRESHOLD = 2.0  
STATE_FILE = "joint2_last_state.json" 

# 📐 LIMIT SETTINGS
MIN_ANGLE_LIMIT = 0.0    # องศาต่ำสุด
MAX_ANGLE_LIMIT = 180.0  # องศาสูงสุด (ตำแหน่ง Limit Switch)

class Joint2Driver(Node):
    def __init__(self):
        super().__init__('joint2_driver_node')
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
        GPIO.setup(PIN_ENA, GPIO.OUT, initial=GPIO.HIGH)
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
        self.create_timer(0.5, self.report_status)

        # Auto-Resume Logic
        self.check_initial_position()

        # เริ่ม Thread ควบคุม
        self.running = True
        self.worker_thread = threading.Thread(target=self.control_loop_worker, daemon=True)
        self.worker_thread.start()

        if not self.is_homed:
            self.get_logger().info("⏳ Waiting for calibration command... (Topic: /joint2/calibrate)")

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
                        self.get_logger().warn(f"⚠️ MOVED WHILE OFF! (Diff: {diff:.2f}°)")
                        self.is_homed = False 
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

    def control_loop_worker(self):
        while self.running and rclpy.ok():
            if not self.is_homed or self.current_target is None:
                time.sleep(0.1)
                continue

            current_angle = self.get_calibrated_angle()
            if current_angle is None: 
                time.sleep(0.01); continue

            error = self.current_target - current_angle
            
            if abs(error) <= ANGLE_TOLERANCE:
                self.integral = 0.0
                self.prev_error = 0.0
                time.sleep(0.01)
                continue 

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

            # ==========================================================
            # 🛡️ SAFETY PROTECTION (0 - 180 Degrees)
            # ==========================================================
            
            # 1. HARD LIMIT Protection (Switch is at 180)
            # ถ้าชนสวิตช์ (GPIO=0) ห้ามหมุนทิศ +1 (เพิ่มองศา)
            if GPIO.input(PIN_LIMIT) == 0:
                if direction == 1:
                    self.integral = 0
                    if int(time.time()) % 2 == 0:
                        print(f"🛑 LIMIT HIT (180°)! Blocking move UP. Angle: {current_angle:.2f}", end='\r')
                    time.sleep(0.01)
                    continue

            # 2. SOFT LIMIT Protection (Software Limit at 0)
            # ถ้าองศาปัจจุบัน <= 0 ห้ามหมุนทิศ -1 (ลดองศา)
            if current_angle <= MIN_ANGLE_LIMIT:
                if direction == -1:
                    self.integral = 0
                    if int(time.time()) % 2 == 0:
                        print(f"🛑 SOFT LIMIT (0°)! Blocking move DOWN. Angle: {current_angle:.2f}", end='\r')
                    time.sleep(0.01)
                    continue
            
            # ถ้าผ่าน Safety ทั้งหมด ให้สั่งหมุนได้
            self.step_pulse_single(direction, calc_delay)

    # ---------------------------------------------------------
    # 📐 SENSOR
    # ---------------------------------------------------------
    def get_calibrated_angle(self):
        raw = self.read_as5600()
        if raw is not None:
            return raw - self.zero_offset
        return None

    def read_as5600(self):
        try:
            self.bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)
            hi = self.bus.read_byte_data(AS5600_ADDR, 0x0E) & 0x0F
            lo = self.bus.read_byte_data(AS5600_ADDR, 0x0F)
            current_raw = (hi << 8) | lo
            
            # ==========================================
            # ⚙️ 3-POINT CALIBRATION (Piecewise)
            # ==========================================
            # กำหนดจุดอ้างอิง 3 จุดตามที่คุณวัดจริง
            P1_RAW = 1871.0   # จุดที่ 0 องศา
            P2_RAW = 2656.0   # จุดที่ 90 องศา
            P3_RAW = 4042.0   # จุดที่ 180 องศา (Limit Switch)
            
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

            return real_angle

        except Exception as e:
            return None

    def report_status(self):
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
            print(f"⚠️ Wait Calib | Raw: {self.read_as5600():.2f} ", end='\r')

    # ---------------------------------------------------------
    # 🏠 HOMING & CLEANUP
    # ---------------------------------------------------------
    def perform_homing_sequence(self):
        self.get_logger().info("🏠 Homing Started (Target: Limit=180°)...")
        HOMING_DELAY = 0.001 
        
        # Logic: วิ่งทิศ +1 ไปหา Switch ที่ตำแหน่ง 180 องศา
        
        # 1. ถ้าชนอยู่แล้ว ให้ถอยออกมา (ทิศ -1)
        if GPIO.input(PIN_LIMIT) == 0:
            self.get_logger().info("   - Already at limit, backing off...")
            for _ in range(500): self.step_pulse_single(-1, HOMING_DELAY)
            
        # 2. วิ่งเข้าหา (ทิศ 1)
        self.get_logger().info("   - Finding limit...")
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(1, HOMING_DELAY)
            
        # 3. ถอยออกนิดนึง (ทิศ -1)
        for _ in range(500): self.step_pulse_single(-1, HOMING_DELAY)
        time.sleep(0.5)
        
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
        
        self.running = False
        if hasattr(self, 'worker_thread'):
            self.worker_thread.join()
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