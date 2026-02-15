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
PIN_ENA = 22
PIN_DIR = 27
PIN_PUL = 17
PIN_LIMIT = 16
ENA_ACTIVE_HIGH = False 

# I2C Sensor (AS5600)
I2C_BUS_ID = 1
MUX_ADDR = 0x70
AS5600_ADDR = 0x36
MUX_CHANNEL = 3

# PID Parameters
KP = 1.0
KI = 0.2
KD = 0.1

# Speed Settings
MIN_DELAY = 0.0005   
MAX_DELAY = 0.001   
PULSE_WIDTH = 0.00005 

ANGLE_TOLERANCE = 2.5  
WARN_DIFF_THRESHOLD = 0.5  
STATE_FILE = "joint3_last_state.json" 

class Joint3Driver(Node):
    def __init__(self):
        super().__init__('joint3_driver_node')
        
        self.lock_file = open('/tmp/raspi_i2c_lock', 'w+')
        
        self.zero_offset = 0.0
        self.is_homed = False
        self.current_target = None 
        
        # PID Variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = time.time()
        self.current_delay = MAX_DELAY # 🔥 ตัวแปรสำหรับทำ Soft Start

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        # 🔥 เปลี่ยนเป็น LOW เพื่อล็อคมอเตอร์กันแขนตกตอนเริ่มต้น
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
        self.create_subscription(Float32, 'joint3/set_target_angle', self.target_callback, 10)
        self.create_subscription(Bool, 'joint3/calibrate', self.calibrate_callback, 10)
        self.angle_pub = self.create_publisher(Float32, 'joint3/angle', 10)
        self.calibration_pub = self.create_publisher(Bool, 'joint3/calibrated', 10)
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
            self.get_logger().info("⏳ Waiting for calibration command... (Topic: /joint3/calibrate)")

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
                        self.get_logger().warn("   Safety Lock: PLEASE RE-CALIBRATE.")
                        calib_msg = Bool()
                        calib_msg.data = False
                        self.calibration_pub.publish(calib_msg)
                        self.is_homed = False
                        # 🔥 จ่ายไฟล็อคมอเตอร์ แม้จะบังคับให้ calibrate ใหม่
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
            
        self.current_target = msg.data
        self.get_logger().info(f"🎯 Target Updated: {self.current_target:.2f}")
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = time.time()

    # ---------------------------------------------------------
    # ⏱️ CONTROL LOOP (แก้ไขเพิ่ม Safety + Soft Start + Anti-Windup)
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
            
            # 🔥 1. Anti-Windup
            self.integral = max(min(self.integral, 20.0), -20.0)

            derivative = (error - self.prev_error) / dt
            pid_output = (KP * error) + (KI * self.integral) + (KD * derivative)
            
            self.prev_error = error
            self.last_pid_time = now

            direction = 1 if pid_output > 0 else -1
            speed_mag = abs(pid_output)
            
            # --- คำนวณความเร็วเป้าหมาย ---
            if speed_mag > 0.01:
                target_delay = MAX_DELAY - (speed_mag / 500.0)
            else:
                target_delay = MAX_DELAY
            
            target_delay = max(MIN_DELAY, min(MAX_DELAY, target_delay))

            # --- 2. Acceleration Ramp (Soft Start) ---
            RAMP_FACTOR = 0.05 
            if getattr(self, 'current_delay', None) is None:
                self.current_delay = MAX_DELAY
                
            self.current_delay = (self.current_delay * (1.0 - RAMP_FACTOR)) + (target_delay * RAMP_FACTOR)

            # ------------------------------------------------------------------
            # 🛡️ SAFETY CHECK: LIMIT SWITCH PROTECTION
            # ------------------------------------------------------------------
            if GPIO.input(PIN_LIMIT) == 0:  
                # Joint 3 Homing วิ่งเข้าหา 0 (ทิศ -1)
                if direction == -1: 
                    self.integral = 0.0   
                    self.prev_error = 0.0
                    self.current_delay = MAX_DELAY 
                    
                    if int(time.time()) % 2 == 0: 
                        print(f"🛑 LIMIT HIT! Blocking MOVE IN (-). Angle: {current_angle:.2f}", end='\r')
                    
                    time.sleep(0.01)
                    continue  

            # --- 3. สั่ง Pulse ---
            if speed_mag > 0.01:
                self.step_pulse_single(direction, self.current_delay)

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
        
        fcntl.flock(self.lock_file, fcntl.LOCK_EX)
        
        try:
            self.bus.write_byte(0x70, 1 << 3)
            hi = self.bus.read_byte_data(AS5600_ADDR, 0x0E) & 0x0F
            lo = self.bus.read_byte_data(AS5600_ADDR, 0x0F)
            current_raw = (hi << 8) | lo
            
            # ==========================================
            # ⚙️ 2-POINT CALIBRATION FOR JOINT 3
            # ==========================================
            P1_RAW = 313.0
            P1_ANG = 8.0
            
            P2_RAW = 2300.0
            P2_ANG = 180.0
            
            slope = (P2_ANG - P1_ANG) / (P2_RAW - P1_RAW)
            real_angle = P1_ANG + slope * (current_raw - P1_RAW)
            
        except Exception as e:
            pass 
        finally:
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)
            
        return real_angle 

    def report_status(self):
        cal_msg = Bool()
        cal_msg.data = self.is_homed
        self.calibration_pub.publish(cal_msg)

        angle = self.get_calibrated_angle()
        if angle is not None and self.is_homed:
            msg = Float32()
            msg.data = angle
            self.angle_pub.publish(msg)
            tgt_str = f"{self.current_target:.2f}" if self.current_target is not None else "None"
            
            lim_status = "🛑HIT" if GPIO.input(PIN_LIMIT) == 0 else "OK"
            print(f"✅ Run | Tgt: {tgt_str} | Cur: {angle:.2f} | Lim: {lim_status}   ", end='\r')
            
        elif not self.is_homed:
            raw = self.read_as5600()
            if raw is not None:
                print(f"⚠️ Wait Calib | Raw: {raw:.2f} ", end='\r')

    # ---------------------------------------------------------
    # 🏠 HOMING & CLEANUP
    # ---------------------------------------------------------
    def perform_homing_sequence(self):
        self.get_logger().info("🏠 Homing Started...")
        HOMING_DELAY = 0.001 
        
        if GPIO.input(PIN_LIMIT) == 0:
            for _ in range(500): self.step_pulse_single(1, HOMING_DELAY)
            
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(-1, HOMING_DELAY)
            
        time.sleep(1.0)
        for _ in range(500): self.step_pulse_single(1, HOMING_DELAY)
        time.sleep(1.0)
        
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(-1, HOMING_DELAY * 2)
            
        val = self.read_as5600()
        if val: 
            self.zero_offset = 0.0
            self.is_homed = True
            self.current_target = 8.0 
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
    node = Joint3Driver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()