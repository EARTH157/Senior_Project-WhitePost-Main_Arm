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
# ⚙️ CONFIGURATION FOR JOINT 3 DRIVER
# ==========================================
PIN_ENA = 22
PIN_DIR = 27
PIN_PUL = 17
PIN_LIMIT = 24
ENA_ACTIVE_HIGH = False 

# I2C Sensor (AS5600)
I2C_BUS_ID = 1
MUX_ADDR = 0x70
AS5600_ADDR = 0x36
MUX_CHANNEL = 3 

# ต้องแก้โค๊ดที่ตรงนี้เพื่อเปลี่ยนทิศเซ็นเซอร์
# 🔥 SENSOR DIRECTION SETTING 🔥
# True = แปลง Raw มากให้เป็นมุมน้อย (กลับทิศ)
# False = ปกติ (Raw มาก = มุมมาก)
INVERT_SENSOR_DIR = True  

# PID Parameters
KP = 0.50
KI = 0.39
KD = 0.01      

# Speed Settings
MIN_DELAY = 0.0003   
MAX_DELAY = 0.0010   
PULSE_WIDTH = 0.00001 

ANGLE_TOLERANCE = 0.5 
WARN_DIFF_THRESHOLD = 2.0  
STATE_FILE = "joint3_last_state.json"

# 📐 LIMIT SETTINGS
MIN_ANGLE_LIMIT = 8.0    
MAX_ANGLE_LIMIT = 352.0  

class Joint3Driver(Node):
    def __init__(self):
        super().__init__('joint3_driver_node') 
        self.zero_offset = 0.0
        self.is_homed = False
        self.current_target = None 
        
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
        self.create_subscription(Float32, 'joint3/set_target_angle', self.target_callback, 10)
        self.create_subscription(Bool, 'joint3/calibrate', self.calibrate_callback, 10)
        self.angle_pub = self.create_publisher(Float32, 'joint3/angle', 10)
        self.create_timer(0.5, self.report_status)

        self.check_initial_position()

        self.running = True
        self.worker_thread = threading.Thread(target=self.control_loop_worker, daemon=True)
        self.worker_thread.start()

        if not self.is_homed:
            self.get_logger().info("⏳ Waiting for calibration command... (Topic: /joint3/calibrate)")

    # ---------------------------------------------------------
    # 🧮 MATH HELPER
    # ---------------------------------------------------------
    def normalize_angle(self, angle):
        return angle % 360.0

    def calculate_angle(self, raw):
        """ คำนวณมุมตามทิศทางที่ตั้งค่าไว้ """
        if INVERT_SENSOR_DIR:
            # สูตรกลับทิศ: Offset - Raw
            return self.normalize_angle(self.zero_offset - raw)
        else:
            # สูตรปกติ: Raw - Offset
            return self.normalize_angle(raw - self.zero_offset)

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
                        self.get_logger().info(f"✅ Position Verified. Resuming...")
                        self.zero_offset = saved_offset
                        self.is_homed = True
                        
                        current_angle = self.calculate_angle(current_raw)
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
            data = {'last_raw_angle': current_raw, 'zero_offset': self.zero_offset}
            try:
                with open(STATE_FILE, 'w') as f:
                    json.dump(data, f)
            except: pass

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
        
        raw_target = msg.data
        clamped_target = max(MIN_ANGLE_LIMIT, min(MAX_ANGLE_LIMIT, raw_target))
        self.current_target = clamped_target
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = time.time()

    # ---------------------------------------------------------
    # ⏱️ CONTROL LOOP
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
            
            if error > 180: error -= 360
            if error < -180: error += 360

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
            # 🛡️ SAFETY PROTECTION
            # ==========================================================
            
            # 🔥 1. HARD LIMIT (Switch = 8°)
            if GPIO.input(PIN_LIMIT) == 0:
                if direction == -1: 
                    self.integral = 0
                    if int(time.time()) % 2 == 0:
                        print(f"🛑 LIMIT HIT! Blocking move DOWN. Angle: {current_angle:.2f}", end='\r')
                    time.sleep(0.01)
                    continue

            # 🔥 2. SOFT LIMIT LOW
            if current_angle < (MIN_ANGLE_LIMIT - 0.5): 
                if direction == -1: 
                    self.integral = 0
                    if int(time.time()) % 2 == 0:
                        print(f"🛑 SOFT LIMIT LOW! Blocking move DOWN. Angle: {current_angle:.2f}", end='\r')
                    time.sleep(0.01)
                    continue

            # 🔥 3. SOFT LIMIT HIGH
            if current_angle >= MAX_ANGLE_LIMIT:
                if direction == 1: 
                    self.integral = 0
                    if int(time.time()) % 2 == 0:
                        print(f"🛑 SOFT LIMIT HIGH! Blocking move UP. Angle: {current_angle:.2f}", end='\r')
                    time.sleep(0.01)
                    continue
            
            self.step_pulse_single(direction, calc_delay)

    # ---------------------------------------------------------
    # 📐 SENSOR READING
    # ---------------------------------------------------------
    def get_calibrated_angle(self):
        raw = self.read_as5600()
        if raw is not None:
            return self.calculate_angle(raw)
        return None

    def read_as5600(self):
        try:
            self.bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)
            hi = self.bus.read_byte_data(AS5600_ADDR, 0x0E) & 0x0F
            lo = self.bus.read_byte_data(AS5600_ADDR, 0x0F)
            return ((hi << 8) | lo) * 360.0 / 4096.0
        except:
            return None

    def report_status(self):
        angle = self.get_calibrated_angle()
        if angle is not None and self.is_homed:
            msg = Float32()
            msg.data = angle
            self.angle_pub.publish(msg)
            tgt_str = f"{self.current_target:.2f}" if self.current_target is not None else "None"
            lim_status = "HIT" if GPIO.input(PIN_LIMIT) == 0 else "OK"
            print(f"✅ J3 Active | Tgt: {tgt_str} | Cur: {angle:.2f} | Switch: {lim_status} ", end='\r')
        elif not self.is_homed:
            print(f"⚠️ J3 Wait Calib | Raw: {self.read_as5600():.2f} ", end='\r')

    # ---------------------------------------------------------
    # 🏠 HOMING (Calculates Offset with INVERT Logic)
    # ---------------------------------------------------------
    def perform_homing_sequence(self):
        self.get_logger().info(f"🏠 Homing Started (Target: Limit={MIN_ANGLE_LIMIT}°)...")
        HOMING_DELAY = 0.001 
        
        # 1. ถอยถ้าชน
        if GPIO.input(PIN_LIMIT) == 0:
            for _ in range(500): self.step_pulse_single(1, HOMING_DELAY)
            
        # 2. วิ่งเข้าหา Limit (ทิศ -1)
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(-1, HOMING_DELAY)
            
        # 3. ถอยออกนิดนึง (ทิศ +1)
        for _ in range(500): self.step_pulse_single(1, HOMING_DELAY)
        time.sleep(0.5)
        
        # 4. วิ่งเข้าหาช้าๆ (ทิศ -1)
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(-1, HOMING_DELAY * 2)
            
        # 5. Set Offset
        val = self.read_as5600()
        if val is not None: 
            target_val = MIN_ANGLE_LIMIT # 8.0
            
            # 🔥 คำนวณ OFFSET ตามโหมด Invert 🔥
            if INVERT_SENSOR_DIR:
                # สูตรกลับทิศ: Target = Offset - Raw
                # ดังนั้น: Offset = Target + Raw
                self.zero_offset = self.normalize_angle(target_val + val)
            else:
                # สูตรปกติ: Target = Raw - Offset
                # ดังนั้น: Offset = Raw - Target
                self.zero_offset = self.normalize_angle(val - target_val)
            
            self.is_homed = True
            
            print(f"\n[DEBUG] Raw: {val:.2f} | Desired: {target_val} | Invert: {INVERT_SENSOR_DIR}")
            print(f"[DEBUG] Calc Offset: {self.zero_offset:.2f}")
            print(f"[DEBUG] Verify: {self.calculate_angle(val):.2f}\n")
            
            # ถอยออกมาที่ Safe Position
            safe_target = MIN_ANGLE_LIMIT + 1.0 
            self.current_target = safe_target
            
            self.get_logger().info(f"✅ Homing Done. Offset: {self.zero_offset:.2f}")
            
        else:
            self.get_logger().error("❌ Homing Failed: Sensor Error")

    def enable_motor(self, enable):
        GPIO.output(PIN_ENA, GPIO.HIGH if enable == ENA_ACTIVE_HIGH else GPIO.LOW)

    def destroy_node(self):
        self.get_logger().info("🛑 Shutting down...")
        self.save_current_state()
        self.running = False
        if hasattr(self, 'worker_thread'): self.worker_thread.join()
        self.enable_motor(False)
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = Joint3Driver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()