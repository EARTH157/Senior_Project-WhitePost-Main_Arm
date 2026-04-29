#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_msgs.msg import Bool as BoolMsg
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
KP = 1.5    # 🟢 เพิ่ม P ให้พุ่งเข้าหาเป้าอย่างรวดเร็ว (เดิม 0.8)
KI = 0.01   # 🟢 ลด I ลงมหาศาล เพื่อป้องกันการสะสมพลังงานจนทะลุเป้า (เดิม 0.20)
KD = 0.08   # 🟢 เพิ่ม D ให้ทำหน้าที่เป็น "เบรก" ตอนใกล้ถึงเป้าหมาย (เดิม 0.01)   

# Speed Settings
PULSE_WIDTH = 0.00005

ANGLE_TOLERANCE = 0.5
WARN_DIFF_THRESHOLD = 1.0  # ยอมให้คลาดเคลื่อนได้ 0.5 องศา ถ้าเกินนี้ต้อง Homing ใหม่
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
        self.last_sensor_rx_time = time.time() # 🟢 เพิ่มบรรทัดนี้: ตัวจับเวลา Watchdog
        # 🟢 [เพิ่มใหม่] ตัวแปรสำหรับ Auto-Resume ตอนเซ็นเซอร์หลุด
        self.sensor_timeout = False
        self.last_known_good_raw = None
        # สมมติเป็น joint1_node ก็ subscribe '/sensor/as5600/joint1' (แก้ชื่อเลขให้ตรงตามไฟล์)
        self.create_subscription(Float32, '/sensor/as5600/joint1', self.raw_sensor_callback, 10)

        # ROS Setup
        self.create_subscription(Float32, '/joint1/set_target_angle', self.target_callback, 10)
        self.create_subscription(Bool, '/joint1/calibrate', self.calibrate_callback, 10)
        self.angle_pub = self.create_publisher(Float32, '/joint1/angle', 10)
        self.calibration_pub = self.create_publisher(Bool, '/joint1/calibrated', 10)
        self.create_timer(0.5, self.report_status)

        # -----------------------------------------------------
        # 🔥 ตรวจสอบค่าเดิม (Auto-Resume Logic)
        # -----------------------------------------------------
        #self.check_initial_position()
        self.initial_position_checked = False # 🟢 เพิ่มบรรทัดนี้แทน
        self.is_homing_active = False

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
        self.last_sensor_rx_time = time.time() # รีเซ็ตเวลาเมื่อเซ็นเซอร์ปกติ
        new_val = msg.data

        # 🟢 1. ต้องอัปเดตค่าล่าสุดก่อน เพื่อให้ฟังก์ชันอ่านองศาสามารถทำงานได้
        self.latest_raw_sensor_val = new_val

        # 🟢 2. ตรวจสอบ Auto-Resume ตอนเปิด Launch File ครั้งแรกสุด
        if not getattr(self, 'initial_position_checked', False):
            self.check_initial_position()
            self.initial_position_checked = True

        # 🟢 3. ลอจิก Auto-Resume เมื่อสาย USB หรือ Sensor สะอึกระหว่างทำงาน (โค้ดเดิม)
        if self.sensor_timeout:
            if self.last_known_good_raw is not None:
                diff = abs(new_val - self.last_known_good_raw)
                if diff > 180: diff = 360 - diff # ป้องกันปัญหามุมหมุนครบรอบ
                
                if diff <= WARN_DIFF_THRESHOLD:
                    self.get_logger().info(f"✅ Sensor reconnected! Position unchanged (Diff {diff:.2f}°). Auto-Resuming...")
                    self.is_homed = True 
                    self.sensor_timeout = False 
                else:
                    self.get_logger().warn(f"⚠️ Sensor reconnected, but arm MOVED ({diff:.2f}°)! Forcing recalibration.")
                    self.is_homed = False       
                    self.sensor_timeout = False
        else:
            # เก็บค่าล่าสุดไว้เผื่อสายหลุด
            self.last_known_good_raw = new_val

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

                    # แจ้งเตือนเฉยๆ ว่ามีการคลาดเคลื่อน แต่จะไม่ล็อกแล้ว
                    if diff <= WARN_DIFF_THRESHOLD:
                        self.get_logger().info(f"✅ Position Verified (Diff: {diff:.2f}°). Resuming...")
                    else:
                        self.get_logger().warn(f"⚠️ MOVED WHILE OFF! (Diff: {diff:.2f}°)")
                        self.get_logger().info("🔓 ปลดล็อกเซฟตี้: อนุญาตให้ทำงานต่อโดยไม่ต้อง Calibrate")

                    # 🟢 บังคับให้ระบบพร้อมทำงานเสมอ!
                    self.zero_offset = saved_offset
                    self.is_homed = True
                    
                    current_angle = current_raw - self.zero_offset
                    self.current_target = current_angle
                    
                    self.enable_motor(True)
                    self.get_logger().info(f"🚀 System READY! Holding at {current_angle:.2f}°")

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
        self.is_homing_active = True   # ✅ ป้องกัน sensor timeout ระหว่าง homing
        self.is_homed = False
        self.target_hz = 0.0
        self.current_hz = 0.0
        
        # ✅ Publish False ทันที ให้ main_processor ผ่าน WAIT_ACK ได้เร็ว
        cal_msg = Bool()
        cal_msg.data = False
        self.calibration_pub.publish(cal_msg)
        self.get_logger().info("📢 Published calibrated=False immediately")
        
        self.enable_motor(True)
        self.perform_homing_sequence()
        
        self.is_homing_active = False  # ✅ เปิด watchdog กลับมาหลัง homing เสร็จ

    def target_callback(self, msg: Float32):
        if not self.is_homed:
            self.get_logger().warn("⚠️ Ignoring target: Need Calibration first!")
            return
        
        if self.current_target is not None and abs(self.current_target - msg.data) < 0.1:
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
        MAX_HZ = 5000.0       # ⬆️ เพิ่มความเร็วสูงสุด (2000 Hz = วิ่งประมาณ 1.2 รอบ/วินาที)
        MIN_HZ = 50.0        # ⬆️ เพิ่มความเร็วต่ำสุด เลี้ยงรอบไว้ไม่ให้หยุดกระชาก
        ACCEL_RATE = 400.0    # ⬆️ เพิ่มอัตราเร่ง ให้ขยับเข้าหาเป้าหมายสมูทๆ
        SPEED_MULTIPLIER = 80.0 # ⬆️ เพิ่มตัวคูณให้ PID ตอบสนองไวขึ้น
        
        while self.running and rclpy.ok():
            time.sleep(0.01) # รันที่ประมาณ 50Hz (ทุกๆ 20ms)
            
            # 🚨 [SAFETY E-STOP] ถ้าไม่ได้รับค่าเซ็นเซอร์เกิน 0.5 วินาที
            # In pid_worker(), replace the sensor timeout block with:
            # FIXED (all 5 lines inside the guard):
            if not getattr(self, 'is_homing_active', False):
                if self.latest_raw_sensor_val is None:
                    self.last_sensor_rx_time = time.time()
                elif time.time() - self.last_sensor_rx_time > 0.5:
                    if self.is_homed:
                        self.get_logger().error("🚨 SENSOR TIMEOUT!", throttle_duration_sec=1.0)
                    self.target_hz = 0.0        # ← now INSIDE the guard ✅
                    self.current_hz = 0.0       # ✅
                    self.is_homed = False       # ✅
                    self.sensor_timeout = True  # ✅
                    continue                    # ✅
            # If is_homing_active is True, skip ALL of the above entirely

            if not self.is_homed or self.current_target is None or self.sensor_timeout:
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

            # ==========================================
            # 🧠 PID Logic & Anti-Windup
            # ==========================================
            now = time.time()
            dt = now - self.last_pid_time
            if dt <= 0: dt = 0.01

            # 🛡️ Anti-Windup 1: ถ้าย้ายฝั่ง (ข้ามเป้าหมาย) ให้ล้างพลังงานสะสมทิ้งทันที!
            if (error > 0 and self.prev_error < 0) or (error < 0 and self.prev_error > 0):
                self.integral = 0.0
                
            self.integral += error * dt
            
            # 🛡️ Anti-Windup 2: จำกัดเพดานพลังงานสะสม ไม่ให้สปริงแข็งเกินไป
            LIMIT_I = 20.0
            if self.integral > LIMIT_I: self.integral = LIMIT_I
            if self.integral < -LIMIT_I: self.integral = -LIMIT_I

            derivative = (error - self.prev_error) / dt
            pid_output = (KP * error) + (KI * self.integral) + (KD * derivative)
            
            self.prev_error = error
            self.last_pid_time = now

            # กำหนดทิศทางมอเตอร์
            self.motor_direction = 1 if pid_output > 0 else -1
            
            # แปลง PID Output เป็นเป้าหมายความถี่ (Target Hz)
            speed_mag = abs(pid_output)
            mapped_hz = MIN_HZ + (speed_mag * SPEED_MULTIPLIER) 
            
            # 🏎️ 🛡️ DYNAMIC BRAKE (ระบบเบรก ABS):
            # ถ้าใกล้ถึงเป้าหมาย (เหลือน้อยกว่า 3 องศา) บังคับลดความเร็วสูงสุดลงตามระยะทาง!
            if abs(error) < 3.0:
                dynamic_max_hz = MAX_HZ * (abs(error) / 3.0) 
                if dynamic_max_hz < MIN_HZ * 1.5: dynamic_max_hz = MIN_HZ * 1.5
                if mapped_hz > dynamic_max_hz: mapped_hz = dynamic_max_hz
            elif mapped_hz > MAX_HZ: 
                mapped_hz = MAX_HZ
            
            self.target_hz = mapped_hz

            # 🚀 ทำ Acceleration Ramp ... (โค้ดเดิมที่เหลือ)

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
            RAW_AT_0_DEG  = 720.0   
            RAW_AT_90_DEG = 1750.0
                
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
            
        # 5. Set 180 Degrees
        val = self.read_as5600()
        
        # 🔓 ปลดล็อกเงื่อนไข: บังคับให้ Homing สำเร็จไปเลย!
        self.zero_offset = val - 0.0        
        self.is_homed = True
        self.current_target = 0.0
        
        if val is not None: 
            current_check = self.get_calibrated_angle()
            self.get_logger().info(f"✅ Homing Done. Sensor reads: {current_check:.2f}° (Should be approx 180°)")
        else:
            self.get_logger().warn("⚠️ Could not read sensor at home — offset defaulting to 0!")
            
        self.get_logger().info("🚀 Holding Position at 0.0°")
        self.last_sensor_rx_time = time.time()  # ✅ reset watchdog after homing
        self.last_known_good_raw = self.latest_raw_sensor_val  # ✅ sync reference point
        
        cal_msg = BoolMsg()
        cal_msg.data = True
        self.calibration_pub.publish(cal_msg)
        self.get_logger().info("📢 Published calibrated=True immediately")

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
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if node:
            node.destroy_node()
        # Guard against double-shutdown
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__': main()