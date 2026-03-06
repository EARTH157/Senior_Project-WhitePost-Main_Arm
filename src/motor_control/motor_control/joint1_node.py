#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import RPi.GPIO as GPIO
import time
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
PIN_LIMIT = 23
ENA_ACTIVE_HIGH = False 

# PID Parameters
KP = 1.0
KI = 0.1
KD = 0.01    

# Speed Settings
PULSE_WIDTH = 0.00005

ANGLE_TOLERANCE = 0.5
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
        self.last_pid_time = time.time()  # 🔥 เติมบรรทัดนี้กลับเข้ามาครับ!

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        # 🔥 แก้เป็น LOW เพื่อล็อกมอเตอร์ทันทีที่เปิด ป้องกันการเคลื่อนที่เอง
        GPIO.setup(PIN_ENA, GPIO.OUT, initial=GPIO.LOW) 
        GPIO.setup(PIN_DIR, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_PUL, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_LIMIT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # (เพิ่มตรงที่เคยเป็น Setup I2C)
        self.latest_raw_sensor_val = None
        # สมมติเป็น joint1_node ก็ subscribe '/sensor/as5600/joint1' (แก้ชื่อเลขให้ตรงตามไฟล์)
        self.create_subscription(Float32, '/sensor/as5600/joint1', self.raw_sensor_callback, 10)

        # ROS Setup
        self.create_subscription(Float32, 'joint1/set_target_angle', self.target_callback, 10)
        self.create_subscription(Bool, 'joint1/calibrate', self.calibrate_callback, 10)
        self.angle_pub = self.create_publisher(Float32, 'joint1/angle', 10)
        self.calibration_pub = self.create_publisher(Bool, 'joint1/calibrated', 10)
        self.create_timer(0.5, self.report_status)

        # -----------------------------------------------------
        # 🔥 ตรวจสอบค่าเดิม (Auto-Resume Logic)
        # -----------------------------------------------------
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

    def raw_sensor_callback(self, msg: Float32):
        self.latest_raw_sensor_val = msg.data

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
                        # 🔥 เพิ่มให้ล็อกมอเตอร์ไว้กันเคลื่อนที่เพิ่ม
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
        #self.get_logger().info(f"🎯 Target Updated: {self.current_target:.2f}")
        
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
            if GPIO.input(PIN_LIMIT) == 0 and self.motor_direction == -1:
                time.sleep(0.005)
                continue

            # แปลงความถี่ (Hz) เป็น Delay
            delay = 1.0 / hz
            self.step_pulse_single(self.motor_direction, delay)

    # ---------------------------------------------------------
    # 🧠 PID WORKER: คำนวณ Error และสร้างความเร่ง (Acceleration)
    # ---------------------------------------------------------
    def pid_worker(self):
        # --- ปรับจูนใหม่สำหรับ TB6600 (Microstep 1/8 หรือ 1600 Pulse/Rev) ---
        MAX_HZ = 3500.0       # ⬆️ เพิ่มความเร็วสูงสุด (2000 Hz = วิ่งประมาณ 1.2 รอบ/วินาที)
        MIN_HZ = 200.0        # ⬆️ เพิ่มความเร็วต่ำสุด เลี้ยงรอบไว้ไม่ให้หยุดกระชาก
        ACCEL_RATE = 250.0    # ⬆️ เพิ่มอัตราเร่ง ให้ขยับเข้าหาเป้าหมายสมูทๆ
        SPEED_MULTIPLIER = 50.0 # ⬆️ เพิ่มตัวคูณให้ PID ตอบสนองไวขึ้น
        
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
        # ถ้าระบบยังไม่ได้รับค่าจาก ESP32 เลย ให้ return None
        if self.latest_raw_sensor_val is None:
            return None
            
        real_angle = None
        current_raw = self.latest_raw_sensor_val
        
        try:
            # ==========================================
            # ⚙️ โค้ดคำนวณ Calibration ของคุณใส่ไว้ตรงนี้เหมือนเดิม!
            # (ด้านล่างนี้เป็นสูตรเดิมของ Joint 1)
            # ==========================================
            RAW_AT_0_DEG  = 653.0   
            RAW_AT_90_DEG = 1760.0   
                
            slope = (90.0 - 0.0) / (RAW_AT_90_DEG - RAW_AT_0_DEG)
            real_angle = slope * (current_raw - RAW_AT_0_DEG) + 0.0
            
        except Exception as e:
            self.get_logger().error(f"Sensor Math Error: {e}")

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
            
        time.sleep(0.5)
        for _ in range(500): self.step_pulse_single(1, HOMING_DELAY)
        time.sleep(0.5)
        
        while GPIO.input(PIN_LIMIT) == 1: 
            self.step_pulse_single(-1, HOMING_DELAY * 2)
            
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
    node = Joint1Driver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
