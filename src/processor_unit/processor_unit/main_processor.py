#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time

# Import message types
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, Bool, String

class Main_Processor(Node):
    def __init__(self):
        super().__init__('Main_processor_node')
        self.declare_parameter('simulation_mode', False)
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # --- Robot Parameters (หน่วย mm) ---
        self.l1 = 90.0   # Base height
        self.l2 = 575.0  # Link 2
        self.l3 = 485.0  # Link 3
        self.l4 = 75.0   # Wrist 1
        self.l5 = 115.0  # Wrist 2
        self.l_end = self.l4 + self.l5 # 190 mm
        
        # --- Trajectory Parameters (สำหรับเคลื่อนที่เส้นตรง) ---
        self.current_pos = None  
        self.target_pos = None   
        self.start_pos = None    
        
        # เก็บมุมข้อต่อปัจจุบัน และโหมดการเคลื่อนที่
        self.current_joints = None 
        self.start_joints = None
        self.target_joints = None
        self.move_mode = 'LINEAR' 
        
        self.is_moving = False   
        self.speed_mm_s = 50.0  
        self.speed_joint_deg_s = 60.0  
        self.timer_period = 0.01 
        self.step_total = 0
        self.step_current = 0
        self.delay_ticks = 0 
        self.speed_homing_deg_s = 80.0
        
        self.homing_phase = 0       
        self.final_home_pos = None  
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # --- Calibration State Machine Parameters ---
        self.cal_status = {1: None, 2: None, 3: None} 
        self.cal_state = 'WAIT_FOR_COMMAND'
        self.system_ready = False
        self.next_state_after_j1 = None
        self.next_state_after_j2 = None
        self.next_state_after_j3 = None
        
        # 🔥 เพิ่ม Flag สำหรับบังคับ Calibrate ใหม่
        self.force_cal = {1: False, 2: False, 3: False}
        
        if self.simulation_mode:
            self.get_logger().info("💻 ใช้งานโหมด SIMULATION: ข้ามการรอเซ็นเซอร์ฮาร์ดแวร์")
            self.system_ready = True
            self.cal_state = 'READY'
            self.current_angles = {1: 90.0, 2: 180.0, 3: 10.0}
            self.current_joints = [90.0, 180.0, 10.0, 90.0, 90.0]
            self.current_pos = self.calculate_forward_kinematics(90.0, 180.0, 10.0)
            self.get_logger().info("✅ Simulation Ready: รอรับพิกัดที่ /target_position")
        
        # --- ตัวแปรสำหรับระบบ Camera Tracking ---
        self.is_tracking_mode = False  
        self.tracking_kp_xy = 0.06
        self.tracking_kp_depth = 0.04
        self.tracking_max_step = 15.0  
        self.RADIUS_DEAD_ZONE = 10.0  
        self.lock_start_time = 0.0  
        self.wait_calib_start_time = 0.0
        self.settle_start_time = 0.0
        
        self.preset_x = 650.0
        self.preset_y = 300.0  
        self.preset_z = 550.0  
        
        self.preset_back_x = -500.0 
        self.preset_back_y = -500.0  
        self.preset_back_z = 750.0
        
        self.sub_command = self.create_subscription(String, '/robot_command', self.cb_robot_command, 10)
        self.pub_command = self.create_publisher(String, '/robot_command', 10)
        self.post_move_action = None # ตัวแปรจำสถานะว่าขยับเสร็จแล้วต้องทำอะไรต่อ
        
        self.button_is_lit = False
        self.sub_button_light = self.create_subscription(Bool, '/button_light_status', self.cb_button_light, 10)
        
        self.floor_level_changed = False
        self.sub_level_sensor = self.create_subscription(String, '/floor_level_status', self.cb_level_sensor, 10)
        
        self.press_sequence_state = 'IDLE' 
        self.state_start_time = 0.0
        self.pre_press_pos = None
        self.force_detected = False
        
        self.press_retry_count = 0 
        self.elevator_door_open = False
        self.check_door_start_time = 0.0
        self.sub_door = self.create_subscription(Bool, '/elevator_door_status', self.cb_door_status, 10)

        self.sub_force = self.create_subscription(String, '/uart_rx_zero_2w', self.cb_force_sensor, 10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_target_error = self.create_subscription(Point, '/target_error', self.cb_target_error, qos_profile)
        self.pub_tracking_ready = self.create_publisher(Bool, '/tracking_ready', 10)
        
        self.cal_timer = self.create_timer(0.1, self.calibration_routine)

        self.subscription = self.create_subscription(Point, '/target_position', self.listener_callback, 10)
        self.pub_active_cam = self.create_publisher(String, '/active_camera', 10)
        self.pub_target_label = self.create_publisher(String, '/set_target_label', 10)
        
        self.sub_cal1 = self.create_subscription(Bool, '/joint1/calibrated', self.cb_cal1, 10)
        self.sub_cal2 = self.create_subscription(Bool, '/joint2/calibrated', self.cb_cal2, 10)
        self.sub_cal3 = self.create_subscription(Bool, '/joint3/calibrated', self.cb_cal3, 10)

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.publisher1_ = self.create_publisher(Float32, '/joint1/set_target_angle', 10)
        self.publisher2_ = self.create_publisher(Float32, '/joint2/set_target_angle', 10)
        self.publisher3_ = self.create_publisher(Float32, '/joint3/set_target_angle', 10)
        self.publisher_servo_ = self.create_publisher(Float32MultiArray, '/servo/set_angle', 10)
        
        self.publisher1_calibrate_ = self.create_publisher(Bool, '/joint1/calibrate', 10)
        self.publisher2_calibrate_ = self.create_publisher(Bool, '/joint2/calibrate', 10)
        self.publisher3_calibrate_ = self.create_publisher(Bool, '/joint3/calibrate', 10)
        
        self.sub_angle1 = self.create_subscription(Float32, '/joint1/angle', self.cb_angle1, 10)
        self.sub_angle2 = self.create_subscription(Float32, '/joint2/angle', self.cb_angle2, 10)
        self.sub_angle3 = self.create_subscription(Float32, '/joint3/angle', self.cb_angle3, 10)
        
        self.current_angles = {1: None, 2: None, 3: None} 
        self.get_logger().info('IK Solver Node Started. Checking Calibration Status...')
        
    def cb_door_status(self, msg):
        self.elevator_door_open = msg.data
        
    def cb_button_light(self, msg):
        self.button_is_lit = msg.data
        
    def cb_level_sensor(self, msg):
        data = msg.data.strip().lower()
        if "change" in data or "ต่างระดับ" in data or data == "1":
            self.floor_level_changed = True

    def force_go_home_before_exit(self):
        self.get_logger().info("🛑 Ctrl+C Detected! กำลังสั่งกลับท่า Home ก่อนปิดระบบ...")
        safe_home_joints = [90.0, 45.0, 90.0, 90.0, 90.0]
        self.publish_joints(safe_home_joints)
        
    def cb_force_sensor(self, msg):
        data = msg.data.strip().lower()
        if "active" in data or data == "1":
            self.force_detected = True
        
    def cb_robot_command(self, msg):
        cmd = msg.data.strip().lower()
        
        # 🟢 เพิ่มการรับคำสั่ง "calibrate" (ยอมให้ทำงานได้แม้ system_ready จะเป็น False)
        if cmd == "calibrate": 
            self.execute_calibrate()
            return

        if not self.system_ready:
            self.get_logger().warn("⚠️ ยัง Calibrate ไม่เสร็จ ไม่สามารถรับคำสั่งได้!")
            return
            
        if cmd == "start": self.execute_go_start()
        elif cmd == "preset": self.execute_preset()
        elif cmd == "start_back": self.execute_go_start_back() 
        elif cmd == "preset_back": self.execute_preset_back()  
        elif cmd == "track": self.execute_track()
        elif cmd == "home": self.execute_go_home()
        else: self.get_logger().warn(f"❓ ไม่รู้จักคำสั่ง: {cmd}")

    def execute_calibrate(self):
        self.get_logger().info("🔄 [COMMAND] CALIBRATE: บังคับ Calibrate ใหม่ทั้งหมด...")
        self.system_ready = False
        self.force_cal = {1: True, 2: True, 3: True}
        self.cal_state = 'INIT_WAIT'
        self.is_moving = False
        self.is_tracking_mode = False
        self.press_sequence_state = 'IDLE'
        
        # ✅ ตั้ง Joint 4 = 180° ระหว่าง Calibrate
        safe_servo = Float32MultiArray()
        safe_servo.data = [0.0, 0.0, 180.0, 0.0, 90.0]  # index 2 = joint4
        self.publisher_servo_.publish(safe_servo)
        self.get_logger().info("🦾 Joint 4 → 180° (Calibration safe position)")

    def execute_go_start(self):
        self.post_move_action = None
        self.get_logger().info("▶️ [COMMAND] GO START: ไปท่าเตรียมพร้อม...")
        self.pub_active_cam.publish(String(data="front"))
        self.is_tracking_mode = False 
        target_start_joints = [90.0, 90.0, 8.0, 180.0, 90.0]
        target_xyz = self.calculate_forward_kinematics(target_start_joints[0], target_start_joints[1], target_start_joints[2])
        
        self.move_mode = 'JOINT'
        self.target_joints = target_start_joints
        self.start_joints = list(self.current_joints)
        max_diff = max([abs(t - s) for t, s in zip(self.target_joints, self.start_joints)])
        duration = max_diff / self.speed_joint_deg_s 
        if duration < 0.5: duration = 0.5
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.target_pos = target_xyz
        self.is_moving = True

    def execute_preset(self):
        self.post_move_action = None
        self.get_logger().info(f"🎯 [COMMAND] PRESET: กางแขนไปพิกัดเตรียมเล็งกล้อง...")
        self.pub_active_cam.publish(String(data="front")) 
        self.is_tracking_mode = False
        self.post_move_action = 'TRACK'
        
        try:
            target_jts = self.calculate_inverse_kinematics(self.preset_x, self.preset_y, self.preset_z)
        except ValueError as e:
            self.get_logger().error(f"❌ พิกัด Preset อยู่นอกระยะเอื้อม: {e}")
            return

        self.target_pos = [self.preset_x, self.preset_y, self.preset_z]
        self.start_pos = list(self.current_pos)
        self.target_joints = target_jts
        self.start_joints = list(self.current_joints)
        
        self.move_mode = 'JOINT'
        
        max_angle_diff = max([abs(t - c) for t, c in zip(self.target_joints, self.current_joints)])
        duration = max_angle_diff / self.speed_joint_deg_s
        if duration < 0.5: 
            duration = 0.5 
            
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.is_moving = True
        
    def execute_go_start_back(self):
        self.post_move_action = None
        self.get_logger().info("▶️ [COMMAND] GO START BACK: ไปท่าเตรียมพร้อม (ด้านหลัง)...")
        self.pub_active_cam.publish(String(data="back")) 
        self.is_tracking_mode = False 
        
        target_start_joints = [270.0, 90.0, 8.0, 180.0, 90.0]
        target_xyz = self.calculate_forward_kinematics(target_start_joints[0], target_start_joints[1], target_start_joints[2])
        
        self.move_mode = 'JOINT'
        self.target_joints = target_start_joints
        self.start_joints = list(self.current_joints)
        max_diff = max([abs(t - s) for t, s in zip(self.target_joints, self.start_joints)])
        duration = max_diff / self.speed_joint_deg_s 
        if duration < 0.5: duration = 0.5
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.target_pos = target_xyz
        self.is_moving = True

    def execute_preset_back(self):
        self.post_move_action = None
        self.get_logger().info(f"🎯 [COMMAND] PRESET BACK: กางแขนไปพิกัดเตรียมเล็งกล้อง (ด้านหลัง)...")
        self.pub_active_cam.publish(String(data="back")) 
        self.is_tracking_mode = False
        self.post_move_action = 'TRACK'
        
        try:
            target_jts = self.calculate_inverse_kinematics(self.preset_back_x, self.preset_back_y, self.preset_back_z)
        except ValueError as e:
            self.get_logger().error(f"❌ พิกัด Preset Back อยู่นอกระยะเอื้อม: {e}")
            return

        self.target_pos = [self.preset_back_x, self.preset_back_y, self.preset_back_z]
        self.start_pos = list(self.current_pos)
        self.target_joints = target_jts
        self.start_joints = list(self.current_joints)
        
        self.move_mode = 'JOINT'
        
        max_angle_diff = max([abs(t - c) for t, c in zip(self.target_joints, self.current_joints)])
        duration = max_angle_diff / self.speed_joint_deg_s
        if duration < 0.5: 
            duration = 0.5 
            
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.is_moving = True

    def execute_track(self):
        self.get_logger().info("👁️ [COMMAND] TRACK: ล็อกพิกัดปัจจุบันล่าสุด แล้วเริ่มรับค่าจากกล้อง...")
        
        self.press_sequence_state = 'IDLE' 
        self.is_moving = False 
        self.delay_ticks = 0
        self.press_retry_count = 0

        if self.current_pos is not None and self.current_joints is not None:
            self.start_pos = list(self.current_pos)
            self.target_pos = list(self.current_pos)
            
            self.start_joints = list(self.current_joints)
            self.target_joints = list(self.current_joints)
            
            self.step_total = 1
            self.step_current = 0
            self.get_logger().info(f"📍 บันทึกพิกัดล่าสุด: X={self.current_pos[0]:.1f}, Y={self.current_pos[1]:.1f}, Z={self.current_pos[2]:.1f}")

        self.is_tracking_mode = True
        self.lock_start_time = 0.0 
        self.pub_tracking_ready.publish(Bool(data=True))
        
    def execute_go_home(self):
        self.post_move_action = None
        self.get_logger().info("🏠 [COMMAND] GO HOME: เก็บแขนกลับท่า Home...")
        self.pub_active_cam.publish(String(data="none")) 
        self.is_tracking_mode = False 
        
        if self.current_joints is not None:
            if self.current_angles[1] is not None: 
                self.current_joints[0] = float(self.current_angles[1])
            if self.current_angles[2] is not None: 
                self.current_joints[1] = float(self.current_angles[2])
            if self.current_angles[3] is not None: 
                self.current_joints[2] = float(self.current_angles[3])
        
        # 📌 ท่า Home สุดท้าย (J1=90, J2=180, J3=8, J4=90, J5=90)
        home_joints = [90.0, 180.0, 8.0, 90.0, 90.0] 

        # 🟢 โค้ดส่วนที่หายไป คืนชีพกลับมาแล้ว!
        if self.current_joints is None:
            target_xyz = self.calculate_forward_kinematics(home_joints[0], home_joints[1], home_joints[2])
            self.publish_joints(home_joints)
            self.current_joints = home_joints
            self.current_pos = target_xyz
            return
            
        self.final_home_pos = self.calculate_forward_kinematics(home_joints[0], home_joints[1], home_joints[2])
        self.homing_phase = 1
        self.move_mode = 'JOINT'
        self.start_joints = list(self.current_joints)
        
        # 📌 เฟส 1: J1 ค้างไว้, J2->90, J3->8
        self.target_joints = [self.current_joints[0], 90.0, 8.0, 90.0, 90.0]

        # ==========================================
        # 🟢 โยนภาระให้ PID มอเตอร์จัดการเร่ง/เบรกเอง!
        # ==========================================
        self.step_total = 1
        self.step_current = 0
        self.is_moving = True
            
    def cb_target_error(self, msg):
        # 1. เช็คความพร้อมของระบบ
        if not self.system_ready or self.homing_phase != 0 or self.current_pos is None:
            return
            
        # 2. อนุญาตให้ทำงานเฉพาะตอน Track หรือ ตอนกำลังยื่นมือกด (PRESSING)
        if not (self.is_tracking_mode or self.press_sequence_state == 'PRESSING'):
            return

        # ==========================================
        # 🙈 3. จัดการจุดบอด (Blind Spot) เมื่อเป้าหลุดกล้อง
        # ==========================================
        if msg.x == 9999.0:
            if self.press_sequence_state == 'PRESSING':
                # กำลังแทงแขนเข้าไป แล้วกล้องมองไม่เห็นปุ่มแล้ว -> ไม่ต้องหยุด แทงต่อไปตามแนวเดิม!
                pass 
            elif self.is_moving:
                # ถ้าเพิ่งเล็งเป้าแล้วหลุด ให้หยุดรอเป้าใหม่
                self.get_logger().warn("🔍 เป้าหลุด! สั่งหุ่นยนต์หยุดนิ่งรอเป้าหมายใหม่...")
                self.is_moving = False  
                self.lock_start_time = 0.0  
                self.pub_tracking_ready.publish(Bool(data=True)) 
            return

        # ==========================================
        # 👀 4. คำนวณ Error และ Delta สำหรับการขยับ
        # ==========================================
        err_x = msg.x       
        err_depth = msg.y   
        err_y = msg.z       
        limit = 20.0 

        # คำนวณระยะการขยับ (Delta)
        delta_x = err_x * self.tracking_kp_xy      
        delta_y = err_depth * self.tracking_kp_depth 
        delta_z = err_y * self.tracking_kp_xy      

        # Limit step size ไม่ให้แขนกระชากแรงเกินไป
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        if distance > self.tracking_max_step:
            scale = self.tracking_max_step / distance
            delta_x *= scale
            delta_y *= scale
            delta_z *= scale

        # ==========================================
        # 🚀 5. โหมด Active Aiming: กำลังยื่นแขนไปกดปุ่ม
        # ==========================================
        if self.press_sequence_state == 'PRESSING':
            # ในโหมดนี้ หุ่นยนต์กำลังเดินหน้าตามแกน Y อยู่แล้วโดยอัตโนมัติ
            # เราจะไม่เรียก move_to_absolute() ซ้ำ เพื่อไม่ให้การเดินหน้าโดนรีเซ็ต
            
            # เราจะทำแค่ "เลื่อนจุดหมายปลายทาง" (Target Pos) ของแกน X (ซ้าย/ขวา) และ Z (ขึ้น/ลง)
            adjust_rate = 1.0  # สเกลการขยับเป้า (ถ้ามันแก้แรงไปให้ลดลงเหลือ 0.5 หรือ 0.3)
            
            # ปรับเป้าแกน X (ซ้าย/ขวา)
            if self.current_pos[1] >= 0:
                self.target_pos[0] += delta_x * adjust_rate
            else:
                self.target_pos[0] -= delta_x * adjust_rate
                
            # ปรับเป้าแกน Z (ขึ้น/ลง)
            self.target_pos[2] += delta_z * adjust_rate
            
            # 🟢 บอกให้กล้องส่งค่าเฟรมต่อไปมาได้เลย เพื่อชดเชยศูนย์อย่างต่อเนื่อง
            self.pub_tracking_ready.publish(Bool(data=True))
            return

        # ==========================================
        # 🎯 6. โหมด Tracking ปกติ: เล็งและล็อกเป้า 2 วินาที
        # ==========================================
        if abs(err_x) <= limit and abs(err_y) <= limit and abs(err_depth) <= self.RADIUS_DEAD_ZONE:
            self.is_moving = False  
            
            if self.lock_start_time == 0.0:
                self.lock_start_time = time.time()
                self.get_logger().info("🎯 [LOCKED] เป้าตรงกลาง! กำลังจ้องจับเวลา 2 วินาที...")
            else:
                elapsed_time = time.time() - self.lock_start_time
                if elapsed_time >= 2.0:
                    if self.press_sequence_state == 'IDLE':
                        self.get_logger().info("✅ นิ่งครบ 2 วินาทีแล้ว! เริ่มยื่นมือไปกดแบบ Dynamic (ยื่นไปแก้ศูนย์ไป)")
                        
                        # เปลี่ยนสถานะเป็นเริ่มกด
                        self.press_sequence_state = 'PRESSING'
                        self.pre_press_pos = list(self.current_pos)
                        self.force_detected = False
                        
                        # ลดความเร็วลงเพื่อให้ขยับแก้ศูนย์ตามกล้องได้ทันและไม่กระชาก
                        self.speed_mm_s = 5.0 
                        
                        # คำนวณเป้าหมาย Y ที่ต้องพุ่งไป
                        push_dir = 1.0 if self.current_pos[1] >= 0 else -1.0
                        target_y = self.current_pos[1] + (150.0 * push_dir)
                        
                        # สั่งเคลื่อนที่เข้าหาเป้าหมาย
                        self.move_to_absolute(self.current_pos[0], target_y, self.current_pos[2])
                        
                        self.lock_start_time = 0.0
            
            self.pub_tracking_ready.publish(Bool(data=True))
            return

        # หากเป้าขยับหลุด Deadzone ระหว่างนับถอยหลัง ให้รีเซ็ตเวลาใหม่
        if self.lock_start_time != 0.0:
            self.get_logger().warn("🏃 เป้าหมายขยับหนี! ยกเลิกการจับเวลา กลับไป Track ต่อ...")
            self.lock_start_time = 0.0 

        # ==========================================
        # 🕹️ 7. ขยับแขนตามกล้องในโหมดเล็ง (Tracking)
        # ==========================================
        if self.current_pos[1] >= 0:
            new_x = self.current_pos[0] + delta_x
            new_y = self.current_pos[1] + delta_y
        else:
            new_x = self.current_pos[0] - delta_x
            new_y = self.current_pos[1] - delta_y
            
        new_z = self.current_pos[2] + delta_z

        try:
            target_jts = self.calculate_inverse_kinematics(new_x, new_y, new_z)
        except ValueError as e:
            self.get_logger().warning(f"❌ เอื้อมไม่ถึง: {e}")
            self.pub_tracking_ready.publish(Bool(data=True)) 
            return

        self.move_mode = 'JOINT' 
        self.target_pos = [new_x, new_y, new_z]
        self.start_pos = list(self.current_pos)
        self.target_joints = target_jts
        self.start_joints = list(self.current_joints)
        
        max_angle_diff = max([abs(t - c) for t, c in zip(self.target_joints, self.current_joints)])
        duration = max_angle_diff / self.speed_joint_deg_s
        if duration < 0.03: 
             duration = 0.03
             
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.is_moving = True

    def cb_angle1(self, msg): self.current_angles[1] = msg.data
    def cb_angle2(self, msg): self.current_angles[2] = msg.data
    def cb_angle3(self, msg): self.current_angles[3] = msg.data
        
    def trigger_estop(self, joint_name):
        self.post_move_action = None
        self.get_logger().error(f"🚨 E-STOP TRIGGERED! {joint_name} เซ็นเซอร์หลุด! ยกเลิกภารกิจทั้งหมดทันที!")
        self.system_ready = False
        self.is_moving = False
        self.is_tracking_mode = False
        self.press_sequence_state = 'IDLE'
        self.homing_phase = 0

    def cb_cal1(self, msg): 
        self.cal_status[1] = msg.data
        if not msg.data and self.system_ready: self.trigger_estop("Joint 1")

    def cb_cal2(self, msg): 
        self.cal_status[2] = msg.data
        if not msg.data and self.system_ready: self.trigger_estop("Joint 2")

    def cb_cal3(self, msg): 
        self.cal_status[3] = msg.data
        if not msg.data and self.system_ready: self.trigger_estop("Joint 3")
    
    def move_to_absolute(self, x, y, z):
        try:
            target_jts = self.calculate_inverse_kinematics(x, y, z)
        except ValueError as e:
            self.get_logger().error(f"❌ เป้าหมายอยู่นอกระยะเอื้อม: {e}")
            self.press_sequence_state = 'IDLE' 
            return

        self.target_pos = [x, y, z]
        self.start_pos = list(self.current_pos)
        self.target_joints = target_jts
        self.start_joints = list(self.current_joints)
        
        distance = math.sqrt((x - self.start_pos[0])**2 + (y - self.start_pos[1])**2 + (z - self.start_pos[2])**2)
        
        self.move_mode = 'LINEAR'
        duration = distance / self.speed_mm_s
        if duration < 0.5: duration = 0.5 
        
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.is_moving = True

    def move_to_offset(self, dx, dy, dz):
        if self.current_pos is None: return
        self.move_to_absolute(self.current_pos[0] + dx, self.current_pos[1] + dy, self.current_pos[2] + dz)
        
    def process_button_press_sequence(self):
        if self.press_sequence_state == 'PRESSING':
            self.floor_level_changed = False 
            
            if self.force_detected:
                self.get_logger().info("🚨 Force Active! ชนแล้ว เริ่มเช็คสถานะและกำลังถอยกลับ...")
                self.is_moving = False 
                
                temp_speed = self.speed_mm_s
                self.speed_mm_s = 50.0 
                self.move_to_absolute(self.pre_press_pos[0], self.pre_press_pos[1], self.pre_press_pos[2])
                self.move_mode = 'JOINT'
                self.speed_mm_s = temp_speed
                
                self.check_door_start_time = time.time() 
                self.press_sequence_state = 'WAIT_FOR_RETRACT'
                
            elif not self.is_moving:
                self.get_logger().warn("⚠️ กดจนสุดแขนแล้วแต่ไม่เจอเซ็นเซอร์ ถอยกลับ...")
                temp_speed = self.speed_mm_s
                self.speed_mm_s = 50.0
                self.get_logger().warn("⚠️ ไม่เจอแรงต้านที่ปุ่ม ถอยกลับ...")
                self.move_to_absolute(self.pre_press_pos[0], self.pre_press_pos[1], self.pre_press_pos[2])
                self.move_mode = 'JOINT'
                self.press_sequence_state = 'WAIT_FOR_RETRACT'
                self.speed_mm_s = temp_speed
                
                self.check_door_start_time = time.time()
                self.press_sequence_state = 'WAIT_FOR_RETRACT'
                
        elif self.press_sequence_state == 'WAIT_FOR_RETRACT':
            is_front = self.pre_press_pos[1] >= 0
            
            if self.elevator_door_open or self.button_is_lit or (not is_front and self.floor_level_changed):
                if self.elevator_door_open:
                    self.get_logger().info("🚪 ภารกิจสำเร็จ: ประตูลิฟต์เปิดแล้ว!")
                elif self.button_is_lit:
                    self.get_logger().info("💡 ภารกิจสำเร็จ: ไฟปุ่มลิฟต์สว่างแล้ว!")
                elif not is_front and self.floor_level_changed:
                    self.get_logger().info("⬆️ ภารกิจสำเร็จ: ลิฟต์กำลังเคลื่อนที่ (ต่างระดับ)!")
                    
                self.pub_target_label.publish(String(data="none"))
                
                self.press_retry_count = 0
                self.floor_level_changed = False 
                
                if is_front:
                    self.press_sequence_state = 'IDLE'
                    self.execute_go_home()
                else:
                    self.get_logger().info("↩️ อยู่ด้านหลัง: กำลังไป Start Back -> Start -> Home...")
                    self.press_sequence_state = 'WAIT_START_BACK_THEN_START'
                    self.execute_go_start_back() 
                return

            if not self.is_moving:
                self.get_logger().info("✅ ถอยกลับถึงจุดปลอดภัยแล้ว! เฝ้าดูประตู/ไฟปุ่ม ต่อ...")
                self.press_sequence_state = 'CHECK_DOOR'
                self.check_door_start_time = time.time()
                
        elif self.press_sequence_state == 'CHECK_DOOR':
            is_front = self.pre_press_pos[1] >= 0
            
            if self.elevator_door_open or self.button_is_lit or (not is_front and self.floor_level_changed):
                if self.elevator_door_open:
                    self.get_logger().info("🚪 ภารกิจสำเร็จ: ประตูลิฟต์เปิดแล้ว!")
                elif self.button_is_lit:
                    self.get_logger().info("💡 ภารกิจสำเร็จ: ไฟปุ่มลิฟต์สว่างแล้ว!")
                elif not is_front and self.floor_level_changed:
                    self.get_logger().info("⬆️ ภารกิจสำเร็จ: ลิฟต์กำลังเคลื่อนที่ (ต่างระดับ)!")
                    
                self.pub_target_label.publish(String(data="none"))
                
                self.press_retry_count = 0
                self.floor_level_changed = False 
                
                if is_front:
                    self.press_sequence_state = 'IDLE'
                    self.execute_go_home()
                else:
                    self.get_logger().info("↩️ อยู่ด้านหลัง: กำลังไป Start Back -> Start -> Home...")
                    self.press_sequence_state = 'WAIT_START_BACK_THEN_START'
                    self.execute_go_start_back()
                return
                
            elapsed = time.time() - self.check_door_start_time
            
            if elapsed >= 5.0: 
                self.press_retry_count += 1
                if self.press_retry_count < 5:
                    self.get_logger().warn(f"⚠️ สถานะยังไม่เปลี่ยน! เตรียม Track ใหม่อีกครั้ง (รอบที่ {self.press_retry_count + 1}/5)")
                    self.press_sequence_state = 'WAIT_BEFORE_RETRACK'
                    self.retrack_start_time = time.time() 
                else:
                    self.get_logger().error("❌ กดครบ 5 ครั้งแล้ว! ยกเลิกภารกิจกลับท่าพัก...")
                    self.press_retry_count = 0
                    self.press_sequence_state = 'IDLE'
                    
                    if self.pre_press_pos[1] >= 0:
                        self.execute_go_start()
                    else:
                        self.execute_go_start_back()

        elif self.press_sequence_state == 'WAIT_BEFORE_RETRACK':
            elapsed = time.time() - self.retrack_start_time
            if elapsed >= 1.5: 
                self.get_logger().info("🔍 เล็งหาปุ่มลิฟต์เพื่อ Track ใหม่...")
                self.press_sequence_state = 'IDLE'      
                self.is_tracking_mode = True            
                self.lock_start_time = 0.0              
                self.pub_tracking_ready.publish(Bool(data=True)) 

        elif self.press_sequence_state == 'WAIT_START_BACK_THEN_START':
            if not self.is_moving:
                self.get_logger().info("✅ ถอยเข้า Start Back แล้ว! กำลังหมุนตัวกลับมาด้านหน้า (Start)...")
                self.press_sequence_state = 'WAIT_START_THEN_HOME' 
                self.execute_go_start()

        elif self.press_sequence_state == 'WAIT_START_THEN_HOME':
            if not self.is_moving:
                self.get_logger().info("✅ หมุนกลับมาด้านหน้าแล้ว! สั่งแขนพับเก็บเข้าท่า Home...")
                self.press_sequence_state = 'IDLE' 
                self.execute_go_home()
    
    # =================================================================
    # 🔄 CALIBRATION ROUTINE
    # =================================================================
    def calibration_routine(self):
        if self.system_ready: return
        
        if self.cal_state == 'WAIT_FOR_COMMAND':
            if self.cal_status[1] == True and self.cal_status[2] == True and self.cal_status[3] == True:
                self.cal_state = 'READY'
            return

        if self.cal_state == 'INIT_WAIT':
            if None not in self.cal_status.values():
                j2_angle = self.current_angles[2]
                if j2_angle is not None and 70.0 <= j2_angle <= 120.0:
                    self.get_logger().info(f">>> Joint 2 อยู่ที่ {j2_angle:.1f}° (ช่วง 70-120): ลำดับ J3 -> J1 -> J2")
                    self.cal_state = 'CHECK_J3'
                    self.next_state_after_j3 = 'CHECK_J1'
                    self.next_state_after_j1 = 'CHECK_J2'
                    self.next_state_after_j2 = 'READY'
                else:
                    self.get_logger().info(f">>> Joint 2 อยู่ที่ {j2_angle if j2_angle else 'Unknown'}°: ลำดับ J2 -> J1 -> J3")
                    self.cal_state = 'CHECK_J2'
                    self.next_state_after_j2 = 'CHECK_J1'
                    self.next_state_after_j1 = 'CHECK_J3'
                    self.next_state_after_j3 = 'READY'

        # ------------------- JOINT 3 -------------------
        elif self.cal_state == 'CHECK_J3':
            if self.cal_status[3] == True and not self.force_cal[3]:
                self.cal_state = self.next_state_after_j3
            else:
                self.publisher3_calibrate_.publish(Bool(data=True))
                self.force_cal[3] = False
                self.cal_state = 'WAIT_ACK_J3'
                self.wait_calib_start_time = time.time()

        elif self.cal_state == 'WAIT_ACK_J3':
            if self.cal_status[3] == False:
                self.cal_state = 'WAIT_CALIB_J3'
                self.wait_calib_start_time = time.time()
            elif time.time() - self.wait_calib_start_time > 5.0:
                self.get_logger().warn("⚠️ J3 ACK timeout — forcing WAIT_CALIB_J3")
                self.cal_state = 'WAIT_CALIB_J3'
                self.wait_calib_start_time = time.time()

        elif self.cal_state == 'WAIT_CALIB_J3':
            if self.cal_status[3] == True:
                self.get_logger().info("✅ Joint 3 Calibrate เสร็จสิ้น!")
                self.cal_state = self.next_state_after_j3
            elif time.time() - self.wait_calib_start_time > 30.0:
                self.get_logger().warn("⚠️ J3 homing timeout — forcing proceed")
                self.cal_state = self.next_state_after_j3

        # ------------------- JOINT 2 -------------------
        elif self.cal_state == 'CHECK_J2':
            if self.cal_status[2] == True and not self.force_cal[2]:
                self.publisher2_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J2'
            else:
                self.publisher2_calibrate_.publish(Bool(data=True))
                self.force_cal[2] = False
                self.cal_state = 'WAIT_ACK_J2'
                self.wait_calib_start_time = time.time()

        elif self.cal_state == 'WAIT_ACK_J2':
            if self.cal_status[2] == False:
                self.cal_state = 'WAIT_CALIB_J2'
                self.wait_calib_start_time = time.time()
            elif time.time() - self.wait_calib_start_time > 5.0:
                self.get_logger().warn("⚠️ J2 ACK timeout — forcing WAIT_CALIB_J2")
                self.cal_state = 'WAIT_CALIB_J2'
                self.wait_calib_start_time = time.time()

        elif self.cal_state == 'WAIT_CALIB_J2':
            if self.cal_status[2] == True:
                self.get_logger().info("✅ Joint 2 Homing เสร็จ! สั่งไป 90°")
                self.publisher2_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J2'
            elif time.time() - self.wait_calib_start_time > 30.0:
                self.get_logger().warn("⚠️ J2 homing timeout — forcing 90°")
                self.publisher2_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J2'

        elif self.cal_state == 'WAIT_ANGLE_J2':
            curr = self.current_angles[2]
            if curr is not None and abs(curr - 90.0) <= 5.0:
                self.get_logger().info("✅ Joint 2 ถึง 90° แล้ว! รอให้นิ่ง...")
                self.settle_start_time = time.time()
                self.cal_state = 'SETTLE_J2'

        elif self.cal_state == 'SETTLE_J2':
            if time.time() - self.settle_start_time >= 1.5:
                self.get_logger().info("✅ J2 นิ่งแล้ว เริ่ม J1")
                self.cal_state = self.next_state_after_j2

        # ------------------- JOINT 1 -------------------
        elif self.cal_state == 'CHECK_J1':
            if self.cal_status[1] == True and not self.force_cal[1]:
                self.publisher1_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J1'
            else:
                self.publisher1_calibrate_.publish(Bool(data=True))
                self.force_cal[1] = False
                self.cal_state = 'WAIT_ACK_J1'
                self.wait_calib_start_time = time.time()

        elif self.cal_state == 'WAIT_ACK_J1':
            if self.cal_status[1] == False:
                self.cal_state = 'WAIT_CALIB_J1'
                self.wait_calib_start_time = time.time()
            elif time.time() - self.wait_calib_start_time > 5.0:
                self.get_logger().warn("⚠️ J1 ACK timeout — forcing WAIT_CALIB_J1")
                self.cal_state = 'WAIT_CALIB_J1'
                self.wait_calib_start_time = time.time()

        elif self.cal_state == 'WAIT_CALIB_J1':
            if self.cal_status[1] == True:
                self.get_logger().info("✅ Joint 1 Homing เสร็จ! สั่งไป 90°")
                self.publisher1_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J1'
            elif time.time() - self.wait_calib_start_time > 30.0:
                self.get_logger().warn("⚠️ J1 homing timeout — forcing 90°")
                self.publisher1_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J1'

        elif self.cal_state == 'WAIT_ANGLE_J1':
            curr = self.current_angles[1]
            if curr is not None and abs(curr - 90.0) <= 5.0:
                self.get_logger().info("✅ Joint 1 ถึง 90° แล้ว! รอให้นิ่ง...")
                self.settle_start_time = time.time()
                self.cal_state = 'SETTLE_J1'

        elif self.cal_state == 'SETTLE_J1':
            if time.time() - self.settle_start_time >= 1.5:
                self.get_logger().info("✅ J1 นิ่งแล้ว เริ่ม J3")
                self.cal_state = self.next_state_after_j1

        # ------------------- READY -------------------
        elif self.cal_state == 'READY':
            self.get_logger().info("===============================")
            self.get_logger().info("🚀 Calibrate สำเร็จ! กำลังเซตท่าเริ่มต้น...")

            # ✅ คืน Joint 4 กลับ 90° หลัง Calibrate เสร็จ
            safe_servo = Float32MultiArray()
            safe_servo.data = [0.0, 0.0, 90.0, 0.0, 90.0]
            self.publisher_servo_.publish(safe_servo)
            self.get_logger().info("🦾 Joint 4 → 90° (Returning to normal)")

            init_joints = [90.0, 180.0, 10.0, 90.0, 90.0]
            init_xyz = self.calculate_forward_kinematics(init_joints[0], init_joints[1], init_joints[2])

            self.move_mode = 'JOINT'
            self.target_joints = init_joints

            start_j1 = self.current_angles[1] if self.current_angles[1] is not None else 90.0
            start_j2 = self.current_angles[2] if self.current_angles[2] is not None else 90.0
            start_j3 = self.current_angles[3] if self.current_angles[3] is not None else 8.0

            self.start_joints = [start_j1, start_j2, start_j3, 90.0, 90.0]

            duration = 2.0
            self.step_total = int(duration / self.timer_period)
            if self.step_total == 0: self.step_total = 1
            self.step_current = 0

            self.current_pos = init_xyz
            self.is_moving = True

            self.get_logger().info(f"📍 Set Home XYZ: {init_xyz}")
            self.get_logger().info("รอรับคำสั่งที่ Topic /robot_command (start, track, home, preset, calibrate) ...")
            self.get_logger().info("===============================")

            self.system_ready = True

    def listener_callback(self, msg):
        if not self.system_ready or self.homing_phase != 0: return
        target_x = msg.x
        target_y = msg.y
        target_z = msg.z
        if self.current_pos is None:
            self.current_pos = [target_x, target_y, target_z]
            try:
                joints = self.calculate_inverse_kinematics(target_x, target_y, target_z)
                self.current_joints = joints
                self.publish_joints(joints)
            except ValueError as e:
                self.get_logger().error(f'Error computing initial IK: {e}')
            return
        dx = target_x - self.current_pos[0]
        dy = target_y - self.current_pos[1]
        dz = target_z - self.current_pos[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance > 0:
            try:
                target_jts = self.calculate_inverse_kinematics(target_x, target_y, target_z)
            except ValueError as e:
                return 
            max_angle_diff = max([abs(t - c) for t, c in zip(target_jts, self.current_joints)])
            if distance > 150.0 or max_angle_diff > 45.0:
                self.move_mode = 'JOINT'
            else:
                self.move_mode = 'LINEAR'
            self.target_pos = [target_x, target_y, target_z]
            self.start_pos = list(self.current_pos)
            self.target_joints = target_jts
            self.start_joints = list(self.current_joints)
            if self.move_mode == 'JOINT':
                duration = max_angle_diff / self.speed_joint_deg_s
                if duration < (distance / self.speed_mm_s): duration = distance / self.speed_mm_s 
            else:
                duration = distance / self.speed_mm_s
            self.step_total = int(duration / self.timer_period)
            if self.step_total == 0: self.step_total = 1
            self.step_current = 0
            self.is_moving = True

    def timer_callback(self):
        if not self.system_ready:
            return
        
        # 🟢 [ส่วนที่เพิ่มใหม่] เช็คประตูลิฟต์เปิดระหว่างเตรียมการ (Preset -> Track)
        # ถ้าประตูเปิด และเรากำลังอยู่ในโหมดเตรียมตัว (Preset หรือ Tracking หรือ กำลังจะกด)
        if self.elevator_door_open:
            # ตรวจสอบว่าอยู่ในสถานะที่ควรยกเลิกหรือไม่:
            # 1. กำลังเคลื่อนที่ไปจุดเล็ง (post_move_action == 'TRACK')
            # 2. หรือ กำลังล็อกเป้าหมายอยู่ (is_tracking_mode)
            # 3. หรือ อยู่ในขั้นตอนการกดปุ่ม (press_sequence_state ไม่ใช่ IDLE)
            is_preparing = (hasattr(self, 'post_move_action') and self.post_move_action == 'TRACK')
            is_busy = self.is_tracking_mode or self.press_sequence_state != 'IDLE' or is_preparing

            # ถ้ายุ่งอยู่ และยังไม่ได้เริ่มกระบวนการกลับบ้าน (homing_phase == 0)
            if is_busy and self.homing_phase == 0:
                self.get_logger().info("🚪 ประตูเปิดเองระหว่างเตรียมการ! ยกเลิกภารกิจและกลับ Home ทันที")
                
                # ล้างสถานะการทำงานทั้งหมด
                self.is_tracking_mode = False
                self.press_sequence_state = 'IDLE'
                self.post_move_action = None
                self.is_moving = False # หยุดการเคลื่อนที่ปัจจุบัน
                
                # สั่งกลับบ้าน
                self.execute_go_home()
                return # ออกจาก callback ทันทีเพื่อไปเริ่ม homing ในรอบถัดไป
        
        if self.press_sequence_state != 'IDLE':
            self.process_button_press_sequence()

        if self.delay_ticks > 0:
            self.delay_ticks -= 1
            if self.delay_ticks == 0:
                self.get_logger().info("✅ พักเสร็จแล้ว! ส่งสัญญาณพร้อมรับค่าใหม่")
                self.pub_tracking_ready.publish(Bool(data=True))
            return 

        if not self.is_moving:
            return
        
        # ==============================================================
        # 🟢 [ด่านตรวจเส้นตรง] เช็คความล้าหลังของฮาร์ดแวร์ (Lag Check)
        # ==============================================================
        if hasattr(self, 'current_interp_joints') and self.step_current > 0:
            max_lag = 0.0
            
            # เช็คเฉพาะแกน 1, 2, 3 (ตัวกำหนดเส้นตรง)
            if self.current_angles[1] is not None:
                max_lag = max(max_lag, abs(self.current_angles[1] - self.current_interp_joints[0]))
            if self.current_angles[2] is not None:
                max_lag = max(max_lag, abs(self.current_angles[2] - self.current_interp_joints[1]))
            if self.current_angles[3] is not None:
                max_lag = max(max_lag, abs(self.current_angles[3] - self.current_interp_joints[2]))
            
            # 🎯 ระยะสายจูง: ถ้ายอมให้ฮาร์ดแวร์ตามหลังได้ไม่เกิน 3.0 องศา
            # ถ้ามอเตอร์ตามหลังเกิน 3 องศา ซอฟต์แวร์จะไม่บวก step ไปข้างหน้า (หยุดรอ)
            if max_lag > 3.0: 
                # ส่งพิกัดย่อยตัวเดิมซ้ำไปก่อน ให้มอเตอร์ตามให้ทัน
                self.publish_joints(self.current_interp_joints)
                return 
        # ==============================================================

        self.step_current += 1
        t = self.step_current / self.step_total 

        if t >= 1.0:
            t = 1.0
            
        if self.press_sequence_state != 'PRESSING':
            t = t * t * (3.0 - 2.0 * t)

        if self.move_mode == 'LINEAR':
            interp_x = self.start_pos[0] + (self.target_pos[0] - self.start_pos[0]) * t
            interp_y = self.start_pos[1] + (self.target_pos[1] - self.start_pos[1]) * t
            interp_z = self.start_pos[2] + (self.target_pos[2] - self.start_pos[2]) * t
            try:
                joints = self.calculate_inverse_kinematics(interp_x, interp_y, interp_z)
                self.publish_joints(joints)
                self.current_joints = joints
                self.current_pos = [interp_x, interp_y, interp_z] 
            except ValueError as e:
                self.is_moving = False
                self.homing_phase = 0
                return
        elif self.move_mode == 'JOINT':
            interp_joints = []
            for i in range(5):
                j_val = self.start_joints[i] + (self.target_joints[i] - self.start_joints[i]) * t
                interp_joints.append(j_val)
            self.publish_joints(interp_joints)
            self.current_joints = interp_joints
            
            if self.start_pos is not None and self.target_pos is not None:
                self.current_pos = [
                    self.start_pos[0] + (self.target_pos[0] - self.start_pos[0]) * t,
                    self.start_pos[1] + (self.target_pos[1] - self.start_pos[1]) * t,
                    self.start_pos[2] + (self.target_pos[2] - self.start_pos[2]) * t
                ]

        if t == 1.0:
            if self.homing_phase == 1:
                # 🛑 ด่านตรวจ J2, J3: รอให้แขนพับหลบให้เสร็จก่อน!
                curr_j2 = self.current_angles[2]
                curr_j3 = self.current_angles[3]
                
                if curr_j2 is None or abs(curr_j2 - 90.0) > 5.0 or curr_j3 is None or abs(curr_j3 - 8.0) > 5.0:
                    if self.step_current % 50 == 0:
                        j2_str = f"{curr_j2:.1f}" if curr_j2 else "N/A"
                        self.get_logger().info(f"⏳ รอแขนพับหลบ (เฟส 1)... (ปัจจุบัน J2:{j2_str}°)")
                    return # เด้งออกไปก่อน ห้ามไปเฟส 2 จนกว่าจะพับเสร็จ
                
                self.get_logger().info("🏠 เฟส 1 เสร็จสิ้น! เริ่มเฟส 2 (หมุนฐาน J1 เข้า 90 องศา)")
                self.homing_phase = 2
                self.start_joints = list(self.current_joints)
                
                self.target_joints = [90.0, 90.0, 8.0, self.current_joints[3], self.current_joints[4]]
                self.step_total = 1
                self.step_current = 0 
                return 

            elif self.homing_phase == 2:
                # 🛑 ด่านตรวจ J1: รอให้ฐานหมุนเข้าที่
                curr_j1 = self.current_angles[1]
                
                if curr_j1 is None or abs(curr_j1 - 90.0) > 3.0:
                    if self.step_current % 50 == 0: 
                        j1_str = f"{curr_j1:.1f}" if curr_j1 else "N/A"
                        self.get_logger().info(f"⏳ กำลังรอ J1 หมุนเข้าที่... (ปัจจุบัน: {j1_str}°)")
                    return 

                self.get_logger().info("🏠 เฟส 2 เสร็จสิ้น! เริ่มเฟส 3 (ดัน Joint 2 ไปที่ 180 องศา)")
                self.homing_phase = 3
                self.start_joints = list(self.current_joints)
                
                self.target_joints = [90.0, 180.0, 8.0, self.current_joints[3], self.current_joints[4]]
                self.step_total = 1
                self.step_current = 0 
                return 

            elif self.homing_phase == 3:
                # 🛑 ด่านตรวจ J2: รอให้ดันแขนเก็บสุด
                curr_j2 = self.current_angles[2]
                
                if curr_j2 is None or abs(curr_j2 - 180.0) > 3.0:
                    if self.step_current % 50 == 0: 
                        j2_str = f"{curr_j2:.1f}" if curr_j2 else "N/A"
                        self.get_logger().info(f"⏳ กำลังรอ J2 ดันเก็บสุด... (ปัจจุบัน: {j2_str}°)")
                    return 

                self.get_logger().info("✅ กลับถึงท่า Home เรียบร้อย! อัปเดตพิกัด XYZ พร้อมรับคำสั่งใหม่")
                self.current_pos = list(self.final_home_pos) 
                self.homing_phase = 0
                self.is_moving = False
                
            else:
                if self.target_pos is not None:
                    self.current_pos = list(self.target_pos)
                
                self.is_moving = False 
                
                if self.is_tracking_mode:
                    self.delay_ticks = 0 
                    self.pub_tracking_ready.publish(Bool(data=True))
                    
                if hasattr(self, 'post_move_action') and self.post_move_action == 'TRACK':
                    self.get_logger().info("✅ ถึงจุด Preset แล้ว! ส่งสัญญาณเริ่ม Track อัตโนมัติ...")
                    self.post_move_action = None
                    self.pub_command.publish(String(data="track"))
            
    def calculate_forward_kinematics(self, j1_deg, j2_deg, j3_deg):
        t1 = math.radians(j1_deg)
        t2 = math.radians(j2_deg)
        math_j3 = 180.0 - j3_deg
        t3 = math.radians(math_j3)
        beta = t3
        D_sq = self.l2**2 + self.l3**2 - 2 * self.l2 * self.l3 * math.cos(math.pi - beta)
        D = math.sqrt(abs(D_sq))
        if D == 0: D = 0.001
        cos_gamma = (self.l2**2 + D**2 - self.l3**2) / (2 * self.l2 * D)
        gamma = math.acos(max(min(cos_gamma, 1.0), -1.0))
        alpha = t2 - gamma
        z_wrist = D * math.sin(alpha)
        w_wrist = D * math.cos(alpha)
        z = z_wrist
        w_target = w_wrist + self.l_end
        x = w_target * math.cos(t1)
        if math.sin(t1) >= 0:
             y = w_target * math.sin(t1) + 190.0
        else:
             y = w_target * math.sin(t1) - 190.0
        return [x, y, z]
            
    def calculate_inverse_kinematics(self, x, y, z):
        def normalize_angle_positive(angle):
            return angle % (2 * math.pi)

        if y >= 0: theta1_raw = math.atan2(y-190.0, x)
        else: theta1_raw = math.atan2(y+190.0, x)
            
        theta1 = normalize_angle_positive(theta1_raw)
        if not (0 <= math.degrees(theta1) <= 360.0): 
            raise ValueError(f"Joint 1 Limit Exceeded: {math.degrees(theta1):.2f} deg")
        
        w_target = math.sqrt(x**2 + y**2)
        w_wrist = w_target - self.l_end
        z_wrist = z

        D_sq = w_wrist**2 + z_wrist**2
        D = math.sqrt(D_sq)

        if D > (self.l2 + self.l3):
            raise ValueError(f"Target Unreachable: Distance {D:.2f} > Max Reach")
        
        alpha = math.atan2(z_wrist, w_wrist)
        cos_gamma = (self.l2**2 + D_sq - self.l3**2) / (2 * self.l2 * D)
        gamma = math.acos(max(min(cos_gamma, 1.0), -1.0))

        cos_beta = (self.l2**2 + self.l3**2 - D_sq) / (2 * self.l2 * self.l3)
        beta = math.acos(max(min(cos_beta, 1.0), -1.0))

        theta2 = alpha - gamma
        theta2 = theta2 + (2*gamma)

        theta3 = math.pi - beta
        theta3 = -(theta3)
        theta3_raw = theta3
        theta3 = math.pi + theta3

        if -theta3_raw > theta2: 
            theta4_raw = -(theta2 + theta3_raw) + (math.pi / 2)
        elif -theta3_raw == theta2: 
            theta4_raw = math.pi / 2
        else:
            theta4_raw = (theta2 + theta3_raw)
            theta4_raw = (math.pi / 2) - theta4_raw

        if not self.simulation_mode:
            theta4_raw += (5 * (math.pi / 180.0))
            
        theta4 = normalize_angle_positive(theta4_raw)
        theta1_for_5 = theta1

        if y > 0:
            if x >= 0:
                if abs(math.degrees(theta1_for_5) - 90.0) < 0.01: theta1_for_5 = 0.0
                theta5_raw = (math.pi / 2) - theta1_for_5
                theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
                theta5_servo = theta5_raw + theta1_for_5 + theta5_phi
                if x == 0: theta5_servo = math.pi/2
                
                if not self.simulation_mode:
                    theta5_servo += (7 * (math.pi / 180.0))
                    
                theta5 = normalize_angle_positive(theta5_servo) 
            else:
                theta5_servo = math.pi - theta1_for_5
                if not self.simulation_mode:
                    theta5_servo += (7 * (math.pi / 180.0))
                theta5 = normalize_angle_positive(theta5_servo)
        else:
            theta5_raw = (math.pi / 2) - theta1_for_5
            theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
            theta5_servo = math.pi + (theta5_raw + theta1_for_5 + theta5_phi)
            
            if not self.simulation_mode:
                theta5_servo += (7 * (math.pi / 180.0))
                
            theta5 = normalize_angle_positive(theta5_servo)
        
        angle_joint1 = math.degrees(theta1)
        angle_joint2 = math.degrees(theta2)
        
        if 70.0 <= angle_joint1 <= 110.0:
            if not (0.0 <= angle_joint2 <= 150.0):
                raise ValueError(f"Safety Limit: เมื่อ J1={angle_joint1:.1f}°, J2 ต้องไม่เกิน 150° (คำนวณได้ {angle_joint2:.1f}°)")
        
        angle_joint3 = math.degrees(theta3)
        angle_joint4 = math.degrees(theta4)
        angle_joint5 = math.degrees(theta5)

        return [angle_joint1, angle_joint2, angle_joint3, angle_joint4, angle_joint5]
    
    def publish_joints(self, joints):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        msg.position = [math.radians(j) for j in joints] 
        self.publisher_.publish(msg)
        
        if not self.simulation_mode:
            msg_float = Float32()
            msg_float.data = float(joints[0])
            self.publisher1_.publish(msg_float) 
            msg_float.data = float(joints[1])
            self.publisher2_.publish(msg_float) 
            msg_float.data = float(joints[2])
            self.publisher3_.publish(msg_float) 

            msg_servo_combined = Float32MultiArray()
            msg_servo_combined.data = [0.0, 0.0, float(joints[3]), 1.0, float(joints[4])]
            self.publisher_servo_.publish(msg_servo_combined)

def main(args=None):
    rclpy.init(args=args)
    node = Main_Processor()

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

if __name__ == '__main__':
    main()