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
        self.current_pos = None  # เก็บตำแหน่ง [x, y, z] ปัจจุบัน
        self.target_pos = None   # เก็บตำแหน่งเป้าหมาย
        self.start_pos = None    # เก็บตำแหน่งเริ่มต้นของการเคลื่อนที่แต่ละรอบ 
        
        # เก็บมุมข้อต่อปัจจุบัน และโหมดการเคลื่อนที่
        self.current_joints = None 
        self.start_joints = None
        self.target_joints = None
        self.move_mode = 'LINEAR' # 'LINEAR' หรือ 'JOINT'
        
        self.is_moving = False   
        self.speed_mm_s = 50.0  
        self.speed_joint_deg_s = 60.0  
        self.timer_period = 0.02 
        self.step_total = 0
        self.step_current = 0
        self.delay_ticks = 0 
        
        # เพิ่ม 2 ตัวแปรนี้สำหรับการทำ Homing 2 จังหวะ
        self.homing_phase = 0       # 0=ปกติ, 1=กำลังหมุน J1, 2=กำลังพับ J2-J5
        self.final_home_pos = None  # เก็บพิกัด XYZ ปลายทาง
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # --- Calibration State Machine Parameters ---
        self.cal_status = {1: None, 2: None, 3: None} 
        self.cal_state = 'INIT_WAIT'
        self.system_ready = False
        self.next_state_after_j1 = None
        self.next_state_after_j2 = None
        
        if self.simulation_mode:
            self.get_logger().info("💻 ใช้งานโหมด SIMULATION: ข้ามการรอเซ็นเซอร์ฮาร์ดแวร์")
            self.system_ready = True
            self.cal_state = 'READY'
            # จำลองค่าเริ่มต้นให้ระบบ
            self.current_angles = {1: 90.0, 2: 180.0, 3: 10.0}
            self.current_joints = [90.0, 180.0, 10.0, 90.0, 90.0]
            self.current_pos = self.calculate_forward_kinematics(90.0, 180.0, 10.0)
            
            # 📍 เพิ่มบรรทัดนี้เพื่อหยุดตัวนับเวลา Calibrate ไม่ให้มันทำงานค้างไว้
            if hasattr(self, 'cal_timer'):
                self.cal_timer.cancel()
            
            self.get_logger().info("✅ Simulation Ready: รอรับพิกัดที่ /target_position")
        
        # --- ตัวแปรสำหรับระบบ Camera Tracking ---
        self.is_tracking_mode = False  
        self.tracking_kp_xy = 0.05  
        self.tracking_kp_depth = 0.2 
        self.tracking_max_step = 3.0   
        self.RADIUS_DEAD_ZONE = 40.0  # 👈 เพิ่มบรรทัดนี้ครับ! (ค่าเดียวกับในกล้อง)
        self.lock_start_time = 0.0  # 👈 [เพิ่มบรรทัดนี้] เอาไว้เก็บเวลาตอนเป้าเริ่มนิ่ง
        
        # 🔥 พิกัด PRESET เตรียมกดปุ่ม (แก้ระยะให้ตรงหน้าลิฟต์ได้เลย)
        self.preset_x = 300.0
        self.preset_y = 350.0  
        self.preset_z = 400.0  
        
        # 🟢 พิกัด PRESET เตรียมกดปุ่ม (ด้านหลัง Q3, Q4)
        # ตั้ง Y ติดลบเพื่อให้หันไปด้านหลัง (ลองปรับ X ตามที่ต้องการได้เลย)
        self.preset_back_x = -300.0 
        self.preset_back_y = -350.0  
        self.preset_back_z = 400.0
        
        # 🔥 ศูนย์กลางรับคำสั่งทั้งหมด
        self.sub_command = self.create_subscription(String, '/robot_command', self.cb_robot_command, 10)
        
        self.button_is_lit = False
        self.sub_button_light = self.create_subscription(Bool, '/button_light_status', self.cb_button_light, 10)
        
        self.floor_level_changed = False
        self.sub_level_sensor = self.create_subscription(String, '/floor_level_status', self.cb_level_sensor, 10)
        
        # --- ตัวแปรสำหรับภารกิจกดปุ่มอัตโนมัติ ---
        self.press_sequence_state = 'IDLE' 
        self.state_start_time = 0.0
        self.pre_press_pos = None
        self.force_detected = False
        
        # 🟢 [เพิ่มใหม่] ตัวแปรสำหรับการเช็คประตู
        self.press_retry_count = 0 
        self.elevator_door_open = False
        self.check_door_start_time = 0.0
        self.sub_door = self.create_subscription(Bool, '/elevator_door_status', self.cb_door_status, 10)

        # รับค่าจาก UART (Force Sensor)
        self.sub_force = self.create_subscription(String, '/uart_rx_zero_2w', self.cb_force_sensor, 10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # สร้าง Subscriber รับค่าจากกล้อง และสวิตช์เปิด/ปิด
        self.sub_target_error = self.create_subscription(Point, '/target_error', self.cb_target_error, qos_profile)
        
        # สร้าง Publisher ส่งสัญญาณบอกกล้องว่าหุ่นยนต์พร้อมรับค่าใหม่แล้ว
        self.pub_tracking_ready = self.create_publisher(Bool, '/tracking_ready', 10)
        
        self.cal_timer = self.create_timer(0.1, self.calibration_routine)

        # --- Subscribers ---
        self.subscription = self.create_subscription(Point, '/target_position', self.listener_callback, 10)
        
        self.pub_active_cam = self.create_publisher(String, '/active_camera', 10)
        
        self.sub_cal1 = self.create_subscription(Bool, '/joint1/calibrated', self.cb_cal1, 10)
        self.sub_cal2 = self.create_subscription(Bool, '/joint2/calibrated', self.cb_cal2, 10)
        self.sub_cal3 = self.create_subscription(Bool, '/joint3/calibrated', self.cb_cal3, 10)

        # --- Publishers ---
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.publisher1_ = self.create_publisher(Float32, '/joint1/set_target_angle', 10)
        self.publisher2_ = self.create_publisher(Float32, '/joint2/set_target_angle', 10)
        self.publisher3_ = self.create_publisher(Float32, '/joint3/set_target_angle', 10)
        self.publisher_servo_ = self.create_publisher(Float32MultiArray, '/servo/set_angle', 10)
        
        self.publisher1_calibrate_ = self.create_publisher(Bool, '/joint1/calibrate', 10)
        self.publisher2_calibrate_ = self.create_publisher(Bool, '/joint2/calibrate', 10)
        self.publisher3_calibrate_ = self.create_publisher(Bool, '/joint3/calibrate', 10)
        
        # เพิ่ม Subscribers สำหรับอ่านมุมปัจจุบันของแต่ละ Joint
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
        # อ่านข้อความที่ส่งมา (แปลงเป็นตัวพิมพ์เล็กเพื่อเทียบง่ายๆ)
        data = msg.data.strip().lower()
        
        # ⚠️ คุณสามารถเปลี่ยนคำว่า "changed" หรือ "1" ให้ตรงกับข้อความที่เซ็นเซอร์คุณส่งมาจริงๆ ได้เลยครับ
        if "change" in data or "ต่างระดับ" in data or data == "1":
            self.floor_level_changed = True

    def force_go_home_before_exit(self):
        self.get_logger().info("🛑 Ctrl+C Detected! กำลังสั่งกลับท่า Home ก่อนปิดระบบ...")
        safe_home_joints = [90.0, 45.0, 90.0, 90.0, 90.0]
        self.publish_joints(safe_home_joints)
        
    def cb_force_sensor(self, msg):
        # อ่านข้อความจาก UART (แปลงเป็นพิมพ์เล็กเพื่อเทียบง่ายๆ)
        data = msg.data.strip().lower()
        
        # ⚠️ แก้ไขคำว่า "active" ให้ตรงกับที่บอร์ดของคุณส่งมาจริงๆ เช่น "1", "hit", "active"
        if "active" in data or data == "1":
            self.force_detected = True
        
    def cb_robot_command(self, msg):
        if not self.system_ready:
            self.get_logger().warn("⚠️ ยัง Calibrate ไม่เสร็จ ไม่สามารถรับคำสั่งได้!")
            return
            
        cmd = msg.data.strip().lower()
        if cmd == "start": self.execute_go_start()
        elif cmd == "preset": self.execute_preset()
        elif cmd == "start_back": self.execute_go_start_back() # 🟢 เพิ่มใหม่
        elif cmd == "preset_back": self.execute_preset_back()  # 🟢 เพิ่มใหม่
        elif cmd == "track": self.execute_track()
        elif cmd == "home": self.execute_go_home()
        else: self.get_logger().warn(f"❓ ไม่รู้จักคำสั่ง: {cmd}")

    def execute_go_start(self):
        self.get_logger().info("▶️ [COMMAND] GO START: ไปท่าเตรียมพร้อม...")
        self.pub_active_cam.publish(String(data="front")) # 🟢 สั่งเปิดกล้องหน้า
        self.is_tracking_mode = False 
        target_start_joints = [90.0, 90.0, 8.0, 180.0, 90.0]
        target_xyz = self.calculate_forward_kinematics(target_start_joints[0], target_start_joints[1], target_start_joints[2])
        
        self.move_mode = 'JOINT'
        self.target_joints = target_start_joints
        self.start_joints = list(self.current_joints)
        max_diff = max([abs(t - s) for t, s in zip(self.target_joints, self.start_joints)])
        duration = max_diff / 30.0 
        if duration < 2.0: duration = 2.0 
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.target_pos = target_xyz
        self.is_moving = True

    def execute_preset(self):
        self.get_logger().info(f"🎯 [COMMAND] PRESET: กางแขนไปพิกัดเตรียมเล็งกล้อง...")
        self.pub_active_cam.publish(String(data="front")) # 🟢 สั่งเปิดกล้องหน้า
        self.is_tracking_mode = False
        
        # คำนวณองศาข้อต่อปลายทาง
        try:
            target_jts = self.calculate_inverse_kinematics(self.preset_x, self.preset_y, self.preset_z)
        except ValueError as e:
            self.get_logger().error(f"❌ พิกัด Preset อยู่นอกระยะเอื้อม: {e}")
            return

        # ตั้งค่าจุดเริ่มต้นและจุดหมาย
        self.target_pos = [self.preset_x, self.preset_y, self.preset_z]
        self.start_pos = list(self.current_pos)
        self.target_joints = target_jts
        self.start_joints = list(self.current_joints)
        
        # 🔥 กุญแจสำคัญ: เปลี่ยนจากโหมด LINEAR เป็น JOINT เพื่อป้องกันการวาร์ป/ตีลังกา
        self.move_mode = 'JOINT'
        
        # คำนวณเวลาเดินทางให้สมูทๆ ไม่กระชาก
        max_angle_diff = max([abs(t - c) for t, c in zip(self.target_joints, self.current_joints)])
        duration = max_angle_diff / self.speed_joint_deg_s
        if duration < 1.5: 
            duration = 1.5 # บังคับให้ใช้เวลาอย่างน้อย 1.5 วินาทีในการกางแขน
            
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.is_moving = True
        
    def execute_go_start_back(self):
        self.get_logger().info("▶️ [COMMAND] GO START BACK: ไปท่าเตรียมพร้อม (ด้านหลัง)...")
        self.pub_active_cam.publish(String(data="back")) # 🟢 สั่งเปิดกล้องหลัง
        self.is_tracking_mode = False 
        
        # 🟢 หมุน Joint 1 ไปที่ 270 องศา (หันไปทางด้านหลัง)
        target_start_joints = [270.0, 90.0, 8.0, 180.0, 90.0]
        target_xyz = self.calculate_forward_kinematics(target_start_joints[0], target_start_joints[1], target_start_joints[2])
        
        self.move_mode = 'JOINT'
        self.target_joints = target_start_joints
        self.start_joints = list(self.current_joints)
        max_diff = max([abs(t - s) for t, s in zip(self.target_joints, self.start_joints)])
        duration = max_diff / 30.0 
        if duration < 2.0: duration = 2.0 
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.target_pos = target_xyz
        self.is_moving = True

    def execute_preset_back(self):
        self.get_logger().info(f"🎯 [COMMAND] PRESET BACK: กางแขนไปพิกัดเตรียมเล็งกล้อง (ด้านหลัง)...")
        self.pub_active_cam.publish(String(data="back")) # 🟢 สั่งเปิดกล้องหลัง
        self.is_tracking_mode = False
        
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
        if duration < 1.5: 
            duration = 1.5 
            
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.is_moving = True

    def execute_track(self):
        self.get_logger().info("👁️ [COMMAND] TRACK: ล็อกพิกัดปัจจุบันล่าสุด แล้วเริ่มรับค่าจากกล้อง...")
        
        # 1. ยกเลิกภารกิจเก่า และหยุดการเคลื่อนที่ทั้งหมดทันที
        self.press_sequence_state = 'IDLE' 
        self.is_moving = False 
        self.delay_ticks = 0
        self.press_retry_count = 0

        # 2. 🌟 บันทึกพิกัดล่าสุดที่ซอฟต์แวร์จำได้ เพื่อใช้เป็นจุดสตาร์ท (สมูทที่สุด ไม่มีกระชาก)
        if self.current_pos is not None and self.current_joints is not None:
            self.start_pos = list(self.current_pos)
            self.target_pos = list(self.current_pos)
            
            self.start_joints = list(self.current_joints)
            self.target_joints = list(self.current_joints)
            
            # เคลียร์สเต็ปการเดินเก่าทิ้งทั้งหมด
            self.step_total = 1
            self.step_current = 0
            
            self.get_logger().info(f"📍 บันทึกพิกัดล่าสุด: X={self.current_pos[0]:.1f}, Y={self.current_pos[1]:.1f}, Z={self.current_pos[2]:.1f}")

        # 3. เปิดโหมดรับภาพ
        self.is_tracking_mode = True
        self.lock_start_time = 0.0 
        
        # 4. ยิงสัญญาณไฟเขียวบอกกล้องให้ส่ง Error รอบใหม่มาได้เลย
        self.pub_tracking_ready.publish(Bool(data=True))
        
    def execute_go_home(self):
        self.get_logger().info("🏠 [COMMAND] GO HOME: เก็บแขนกลับท่า Home...")
        self.pub_active_cam.publish(String(data="none")) # 🟢 สั่งปิดทุกกล้องตอนพัก ประหยัดแบต!
        self.is_tracking_mode = False 
        home_joints = [90.0, 180.0, 8.0, 90.0, 90.0] 
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
        self.target_joints = [self.current_joints[0], 90.0, 8.0, 90.0, 90.0]
        max_diff = max([abs(tj - cj) for tj, cj in zip(self.target_joints, self.current_joints)])
        duration = max_diff / 30.0  
        if duration < 1.0: duration = 1.0 
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.is_moving = True
            
    def cb_target_error(self, msg):
        # 🌟 1. ด่านตรวจจับแรกสุด! ถ้าระบบไม่พร้อม หรือไม่ได้เปิดโหมด Track ให้ "เมิน" ข้อความทุกอย่างทิ้งไปเลย
        if not self.is_tracking_mode or not self.system_ready or self.homing_phase != 0 or self.current_pos is None:
            return

        # 🌟 2. Print ข้อความเฉพาะตอนที่กำลัง Track อยู่ (แก้ปัญหาโชว์ 9999 ตอนกดเสร็จ)
        self.get_logger().info(f"📨 Msg received: {msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}")
        
        # 🔥 3. ถ้ารับรหัส 9999.0 แปลว่า "เป้าหลุดกล้อง" ระหว่าง Track
        if msg.x == 9999.0:
            if self.is_moving:
                self.get_logger().warn("🔍 เป้าหลุด! สั่งหุ่นยนต์หยุดนิ่งรอเป้าหมายใหม่...")
                self.is_moving = False  # สั่งเบรกหยุดนิ่งอยู่กับที่ทันที!
            self.lock_start_time = 0.0  # รีเซ็ตตัวนับเวลากดปุ่ม
            self.pub_tracking_ready.publish(Bool(data=True)) # บอกกล้องว่าพร้อมรับภาพเฟรมต่อไป
            return

        # 2. เช็คเงื่อนไขความพร้อม
        if not self.system_ready:
            self.get_logger().warn("⚠️ Ignore: System NOT Ready")
            return
        if not self.is_tracking_mode:
            return
        if self.current_pos is None:
            self.get_logger().warn("⚠️ Ignore: Current Pos is None")
            return
        if self.homing_phase != 0:
            self.get_logger().warn("⚠️ Ignore: Homing in progress")
            return

        err_x = msg.x       
        err_depth = msg.y   
        err_y = msg.z       
        
        limit = 20.0 # (ปรับความแคบ/กว้างของเป้าได้ตามต้องการ)
        
        # --- 1. ถ้าเป้าหมายเข้ามาอยู่ตรงกลาง ---
        if abs(err_x) <= limit and abs(err_y) <= limit and abs(err_depth) <= self.RADIUS_DEAD_ZONE:
            self.is_moving = False  # สั่งเบรกหยุดขยับ
            
            if self.lock_start_time == 0.0:
                self.lock_start_time = time.time()
                self.get_logger().info("🎯 [LOCKED] เป้าตรงกลาง! กำลังจ้องจับเวลา 2 วินาที...")
            else:
                elapsed_time = time.time() - self.lock_start_time
                if elapsed_time >= 2.0:
                    if self.press_sequence_state == 'IDLE':
                        self.get_logger().info("✅ นิ่งครบ 2 วินาทีแล้ว! มั่นใจ! ยื่นมือไปกดเลย!")
                        self.is_tracking_mode = False 
                        self.lock_start_time = 0.0 
                        
                        self.press_sequence_state = 'PRESSING'
                        self.pre_press_pos = list(self.current_pos)
                        self.force_detected = False
                        #self.press_retry_count = 0 
                        
                        temp_speed = self.speed_mm_s
                        self.speed_mm_s = 50.0 
                        
                        # 🟢 [แก้ไข] เช็คว่าถ้า Y ติดลบ (อยู่ด้านหลัง) ให้ยื่นดันไปทางทิศ -Y
                        push_dir = 1.0 if self.current_pos[1] >= 0 else -1.0
                        self.move_to_offset(0.0, 150.0 * push_dir, 0.0) 
                        
                        self.speed_mm_s = temp_speed
                        
            self.pub_tracking_ready.publish(Bool(data=True))
            return

        # --- 2. ถ้าเป้าหมายขยับหนีออกไปจากตรงกลาง ---
        # (ผมตัดคำว่า else ทิ้งไปเลย เพื่อป้องกันการย่อหน้าผิดพลาดครับ)
        if self.lock_start_time != 0.0:
            self.get_logger().warn("🏃 เป้าหมายขยับหนี! ยกเลิกการจับเวลา กลับไป Track ต่อ...")
            self.lock_start_time = 0.0 

        # คำนวณ Delta (พิกัดระยะก้าว)
        delta_x = err_x * self.tracking_kp_xy      
        delta_y = err_depth * self.tracking_kp_depth 
        delta_z = err_y * self.tracking_kp_xy      

        # จำกัดระยะก้าว (Safety)
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        if distance > self.tracking_max_step:
            scale = self.tracking_max_step / distance
            delta_x *= scale
            delta_y *= scale
            delta_z *= scale

        # 🟢 [แก้ไข] ตรวจสอบว่าหันหน้า (Y>=0) หรือ หันหลัง (Y<0) เพื่อแปลงแกนกล้องให้ตรงข้าม
        if self.current_pos[1] >= 0:
            new_x = self.current_pos[0] + delta_x
            new_y = self.current_pos[1] + delta_y
        else:
            new_x = self.current_pos[0] - delta_x
            new_y = self.current_pos[1] - delta_y
            
        new_z = self.current_pos[2] + delta_z

        self.get_logger().info(f"Target: X:{new_x:.1f}, Y:{new_y:.1f}, Z:{new_z:.1f}")

        self.get_logger().info(f"Target: X:{new_x:.1f}, Y:{new_y:.1f}, Z:{new_z:.1f} (Moved: {delta_x:.1f}, {delta_y:.1f}, {delta_z:.1f})")

        try:
            target_jts = self.calculate_inverse_kinematics(new_x, new_y, new_z)
        except ValueError as e:
            self.get_logger().warning(f"❌ เอื้อมไม่ถึง: {e}")
            self.pub_tracking_ready.publish(Bool(data=True)) 
            return

        # เตรียมตัวขยับ 1 รอบ
        self.move_mode = 'JOINT' 
        self.target_pos = [new_x, new_y, new_z]
        self.start_pos = list(self.current_pos)
        self.target_joints = target_jts
        self.start_joints = list(self.current_joints)
        
        # คำนวณเวลาเดินทางใน 1 รอบ 
        max_angle_diff = max([abs(t - c) for t, c in zip(self.target_joints, self.current_joints)])
        duration = max_angle_diff / self.speed_joint_deg_s
        if duration < 0.05: 
             duration = 0.05
             
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        
        # ล็อคธง! เริ่มเคลื่อนที่
        self.is_moving = True 
        self.get_logger().info(f"🎯 คำนวณเป้าหมายเสร็จแล้ว! กำลังเคลื่อนที่...")

    def cb_angle1(self, msg): self.current_angles[1] = msg.data
    def cb_angle2(self, msg): self.current_angles[2] = msg.data
    def cb_angle3(self, msg): self.current_angles[3] = msg.data
        
    def trigger_estop(self, joint_name):
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
        """ สั่งหุ่นยนต์ไปที่พิกัด x, y, z ทันที (โหมด LINEAR) """
        try:
            target_jts = self.calculate_inverse_kinematics(x, y, z)
        except ValueError as e:
            self.get_logger().error(f"❌ เป้าหมายอยู่นอกระยะเอื้อม: {e}")
            self.press_sequence_state = 'IDLE' # ยกเลิกภารกิจ
            return

        self.target_pos = [x, y, z]
        self.start_pos = list(self.current_pos)
        self.target_joints = target_jts
        self.start_joints = list(self.current_joints)
        
        distance = math.sqrt((x - self.start_pos[0])**2 + (y - self.start_pos[1])**2 + (z - self.start_pos[2])**2)
        
        self.move_mode = 'LINEAR'
        duration = distance / self.speed_mm_s
        if duration < 0.5: duration = 0.5 # ขั้นต่ำให้เวลาขยับ 0.5 วินาที จะได้สมูท
        
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        self.is_moving = True

    def move_to_offset(self, dx, dy, dz):
        """ สั่งหุ่นยนต์ขยับแบบอ้างอิงจากตำแหน่งปัจจุบัน """
        if self.current_pos is None: return
        self.move_to_absolute(self.current_pos[0] + dx, self.current_pos[1] + dy, self.current_pos[2] + dz)
        
    def process_button_press_sequence(self):
        # 1. จังหวะกำลังยื่นแขนไปกด
        if self.press_sequence_state == 'PRESSING':
            self.floor_level_changed = False 
            
            if self.force_detected:
                self.get_logger().info("🚨 Force Active! ชนแล้ว เริ่มเช็คสถานะและกำลังถอยกลับ...")
                self.is_moving = False 
                
                temp_speed = self.speed_mm_s
                self.speed_mm_s = 50.0 
                self.move_to_absolute(self.pre_press_pos[0], self.pre_press_pos[1], self.pre_press_pos[2])
                self.speed_mm_s = temp_speed
                
                self.check_door_start_time = time.time() 
                self.press_sequence_state = 'WAIT_FOR_RETRACT'
                
            elif not self.is_moving:
                self.get_logger().warn("⚠️ กดจนสุดแขนแล้วแต่ไม่เจอเซ็นเซอร์ ถอยกลับ...")
                temp_speed = self.speed_mm_s
                self.speed_mm_s = 50.0
                self.move_to_absolute(self.pre_press_pos[0], self.pre_press_pos[1], self.pre_press_pos[2])
                self.speed_mm_s = temp_speed
                
                self.check_door_start_time = time.time()
                self.press_sequence_state = 'WAIT_FOR_RETRACT'
                
        # 2. รอถอยกลับ
        elif self.press_sequence_state == 'WAIT_FOR_RETRACT':
            is_front = self.pre_press_pos[1] >= 0
            
            if self.elevator_door_open or self.button_is_lit or (not is_front and self.floor_level_changed):
                if self.elevator_door_open:
                    self.get_logger().info("🚪 ภารกิจสำเร็จ: ประตูลิฟต์เปิดแล้ว!")
                elif self.button_is_lit:
                    self.get_logger().info("💡 ภารกิจสำเร็จ: ไฟปุ่มลิฟต์สว่างแล้ว!")
                elif not is_front and self.floor_level_changed:
                    self.get_logger().info("⬆️ ภารกิจสำเร็จ: ลิฟต์กำลังเคลื่อนที่ (ต่างระดับ)!")
                
                self.press_retry_count = 0
                self.floor_level_changed = False 
                
                if is_front:
                    self.press_sequence_state = 'IDLE'
                    self.execute_go_home()
                else:
                    self.get_logger().info("↩️ อยู่ด้านหลัง: กำลังไป Start Back -> Start -> Home...")
                    self.press_sequence_state = 'WAIT_START_BACK_THEN_START' # 🟢 เปลี่ยนสเตท
                    self.execute_go_start_back() 
                return

            if not self.is_moving:
                self.get_logger().info("✅ ถอยกลับถึงจุดปลอดภัยแล้ว! เฝ้าดูประตู/ไฟปุ่ม ต่อ...")
                self.press_sequence_state = 'CHECK_DOOR'
                
        # 3. เฝ้าดูประตู/ไฟปุ่ม จนกว่าจะครบเวลาที่กำหนด
        elif self.press_sequence_state == 'CHECK_DOOR':
            is_front = self.pre_press_pos[1] >= 0
            
            if self.elevator_door_open or self.button_is_lit or (not is_front and self.floor_level_changed):
                if self.elevator_door_open:
                    self.get_logger().info("🚪 ภารกิจสำเร็จ: ประตูลิฟต์เปิดแล้ว!")
                elif self.button_is_lit:
                    self.get_logger().info("💡 ภารกิจสำเร็จ: ไฟปุ่มลิฟต์สว่างแล้ว!")
                elif not is_front and self.floor_level_changed:
                    self.get_logger().info("⬆️ ภารกิจสำเร็จ: ลิฟต์กำลังเคลื่อนที่ (ต่างระดับ)!")
                
                self.press_retry_count = 0
                self.floor_level_changed = False 
                
                if is_front:
                    self.press_sequence_state = 'IDLE'
                    self.execute_go_home()
                else:
                    self.get_logger().info("↩️ อยู่ด้านหลัง: กำลังไป Start Back -> Start -> Home...")
                    self.press_sequence_state = 'WAIT_START_BACK_THEN_START' # 🟢 เปลี่ยนสเตท
                    self.execute_go_start_back()
                return
                
            elapsed = time.time() - self.check_door_start_time
            
            if elapsed >= 1.5: 
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

        # 4. รอเวลาสักเล็กน้อย แล้วเปิดโหมด Track อีกรอบ
        elif self.press_sequence_state == 'WAIT_BEFORE_RETRACK':
            elapsed = time.time() - self.retrack_start_time
            if elapsed >= 1.5: 
                self.get_logger().info("🔍 เล็งหาปุ่มลิฟต์เพื่อ Track ใหม่...")
                self.press_sequence_state = 'IDLE'      
                self.is_tracking_mode = True            
                self.lock_start_time = 0.0              
                self.pub_tracking_ready.publish(Bool(data=True)) 

        # 5. 🟢 [ใหม่] รอให้กลับถึง Start Back เสร็จ แล้วสั่งหมุนมาท่า Start (หน้า)
        elif self.press_sequence_state == 'WAIT_START_BACK_THEN_START':
            if not self.is_moving:
                self.get_logger().info("✅ ถอยเข้า Start Back แล้ว! กำลังหมุนตัวกลับมาด้านหน้า (Start)...")
                self.press_sequence_state = 'WAIT_START_THEN_HOME' 
                self.execute_go_start()

        # 6. 🟢 [ใหม่] รอให้หมุนถึงท่า Start (หน้า) เสร็จ แล้วสั่งพับเข้า Home
        elif self.press_sequence_state == 'WAIT_START_THEN_HOME':
            if not self.is_moving:
                self.get_logger().info("✅ หมุนกลับมาด้านหน้าแล้ว! สั่งแขนพับเก็บเข้าท่า Home...")
                self.press_sequence_state = 'IDLE' # จบภารกิจ
                self.execute_go_home()
    
    def calibration_routine(self):
        if self.system_ready: return

        if self.cal_state == 'INIT_WAIT':
            if None not in self.cal_status.values():
                j2_angle = self.current_angles[2]
                
                # 🔥 เช็คเงื่อนไข: ถ้า J2 อ่านค่าได้ และมุมอยู่ระหว่าง 45 ถึง 135 องศา -> ทำ J1 ก่อน
                if j2_angle is not None and 45.0 <= j2_angle <= 135.0:
                    self.get_logger().info(f">>> Joint 2 อยู่ที่ {j2_angle:.1f}° (ปลอดภัย): เริ่ม Calibrate Joint 1 ก่อน")
                    self.cal_state = 'CHECK_J1'
                    self.next_state_after_j1 = 'CHECK_J2'
                    self.next_state_after_j2 = 'CHECK_J3'
                else:
                    # ถ้าอยู่นอกระยะ หรือยังอ่านค่าไม่ได้ -> ทำ J2 ก่อนเหมือนเดิม
                    if j2_angle is not None:
                        self.get_logger().info(f">>> Joint 2 อยู่ที่ {j2_angle:.1f}°: เริ่ม Calibrate Joint 2 ก่อนเพื่อความปลอดภัย")
                    else:
                        self.get_logger().info(">>> เริ่มต้น Calibrate: กำลังตรวจสอบ Joint 2 ก่อน")
                        
                    self.cal_state = 'CHECK_J2'
                    self.next_state_after_j2 = 'CHECK_J1'
                    self.next_state_after_j1 = 'CHECK_J3'

        elif self.cal_state == 'CHECK_J2':
            if self.cal_status[2] == True:
                self.publisher2_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J2'
            else:
                self.publisher2_calibrate_.publish(Bool(data=True))
                self.cal_state = 'WAIT_CALIB_J2'

        elif self.cal_state == 'WAIT_CALIB_J2':
            if self.cal_status[2] == True:
                self.publisher2_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J2'

        elif self.cal_state == 'WAIT_ANGLE_J2':
            curr = self.current_angles[2]
            if curr is not None and abs(curr - 90.0) <= 5.0:
                self.get_logger().info("✅ Joint 2 ถึงเป้าหมายแล้ว! -> ไปขั้นตอนต่อไป")
                # ขยับไป State ต่อไปตามที่ประเมินไว้ตอนแรก
                self.cal_state = self.next_state_after_j2

        elif self.cal_state == 'CHECK_J1':
            if self.cal_status[1] == True:
                self.publisher1_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J1'
            else:
                self.publisher1_calibrate_.publish(Bool(data=True))
                self.cal_state = 'WAIT_CALIB_J1'

        elif self.cal_state == 'WAIT_CALIB_J1':
            if self.cal_status[1] == True:
                self.publisher1_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J1'

        elif self.cal_state == 'WAIT_ANGLE_J1':
            curr = self.current_angles[1]
            if curr is not None and abs(curr - 90.0) <= 5.0:
                self.get_logger().info("✅ Joint 1 ถึงเป้าหมายแล้ว! -> ไปขั้นตอนต่อไป")
                # ขยับไป State ต่อไปตามที่ประเมินไว้ตอนแรก
                self.cal_state = self.next_state_after_j1
                
        elif self.cal_state == 'CHECK_J3':
            if self.cal_status[3] == True:
                self.cal_state = 'READY'
            else:
                self.publisher3_calibrate_.publish(Bool(data=True))
                self.cal_state = 'WAIT_CALIB_J3'

        elif self.cal_state == 'WAIT_CALIB_J3':
            if self.cal_status[3] == True:
                self.get_logger().info("✅ Joint 3 Calibrate เสร็จสิ้น!")
                self.cal_state = 'READY'

        elif self.cal_state == 'READY':
            self.get_logger().info("===============================")
            self.get_logger().info("🚀 Calibrate สำเร็จ! กำลังเซตท่าเริ่มต้น...")

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
            self.get_logger().info("รอรับคำสั่งพิกัดใหม่ที่ /target_position...")
            self.get_logger().info("===============================")

            self.system_ready = True
            self.cal_timer.cancel()

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

    # 🔥 โค้ดส่วนหลักที่แก้ไขเรื่องการนับ Delay 🔥
    def timer_callback(self):
        if not self.system_ready:
            return
        
        # 🔥 แทรกโค้ดตรงนี้: ให้รัน State Machine ภารกิจกดปุ่ม
        if self.press_sequence_state != 'IDLE':
            self.process_button_press_sequence()

        # 1. ถ้านับถอยหลังพักอยู่ ให้ทำงานส่วนนี้
        if self.delay_ticks > 0:
            self.delay_ticks -= 1
            if self.delay_ticks == 0:
                # พักเสร็จแล้ว ส่งค่าบอกกล้อง
                self.get_logger().info("✅ พักเสร็จแล้ว! ส่งสัญญาณพร้อมรับค่าใหม่")
                self.pub_tracking_ready.publish(Bool(data=True))
            return # ออกไปเลย ไม่ต้องไปขยับมอเตอร์

        # 2. ถ้าไม่ได้เคลื่อนที่ ก็ไม่ต้องทำอะไร
        if not self.is_moving:
            return

        # 3. กำลังเคลื่อนที่ คำนวณ t
        self.step_current += 1
        t = self.step_current / self.step_total 

        if t >= 1.0:
            t = 1.0

        # --- คำนวณและสั่งขยับ ---
        # --- คำนวณและสั่งขยับ ---
        if self.move_mode == 'LINEAR':
            interp_x = self.start_pos[0] + (self.target_pos[0] - self.start_pos[0]) * t
            interp_y = self.start_pos[1] + (self.target_pos[1] - self.start_pos[1]) * t
            interp_z = self.start_pos[2] + (self.target_pos[2] - self.start_pos[2]) * t
            try:
                joints = self.calculate_inverse_kinematics(interp_x, interp_y, interp_z)
                # 🔥 [ลบการตรวจจับ > 5.0 ทิ้ง] เพราะมันชอบไปสกัดขาหุ่นตอนวิ่งแนวตรงไกลๆ ทำให้แขนค้าง
                self.publish_joints(joints)
                self.current_joints = joints
                
                # ✅ [แก้ไขบั๊กสำคัญ!] ต้องบันทึกพิกัด XYZ ตามไปด้วยเสมอ ไม่งั้นหุ่นจะหลงทิศตอนถูกเบรก!
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
            
            # 🌟 [แก้ไขบั๊กพุ่งกระฉูด!]
            # ยกเลิกการใช้สมการ FK (calculate_forward_kinematics) มาอัปเดตพิกัด XYZ ชั่วคราว 
            # เพราะสมการ IK/FK ตีกันเอง ทำให้ค่า X, Y กระโดดข้ามพิกัด
            # เปลี่ยนมาใช้การคำนวณจุดต่อจุดอิงจาก target_pos โดยตรง หุ่นจะได้จำตำแหน่งตัวเองได้แม่นยำ 100%
            if self.start_pos is not None and self.target_pos is not None:
                self.current_pos = [
                    self.start_pos[0] + (self.target_pos[0] - self.start_pos[0]) * t,
                    self.start_pos[1] + (self.target_pos[1] - self.start_pos[1]) * t,
                    self.start_pos[2] + (self.target_pos[2] - self.start_pos[2]) * t
                ]

        # --- 2. การจัดการเมื่อเดินสุดทาง (t = 1.0) ---
        if t == 1.0:
            if self.homing_phase == 1:
                # จบเฟส 1: เริ่มเฟส 2 (หมุนฐาน J1 ไป 90 องศา)
                self.get_logger().info("🏠 เฟส 1 เสร็จสิ้น! เริ่มเฟส 2 (หมุนฐาน J1 เข้า 90 องศา)")
                self.homing_phase = 2
                self.start_joints = list(self.current_joints)
                
                # เป้าหมายเฟส 2: เปลี่ยนแค่ J1 เป็น 90 (ส่วน J4 ยังคงค้างชูไว้ที่ 180)
                self.target_joints = [
                    90.0, 
                    self.current_joints[1], 
                    self.current_joints[2], 
                    self.current_joints[3], 
                    self.current_joints[4]
                ]
                
                diff_j1 = abs(90.0 - self.current_joints[0])
                duration = diff_j1 / 30.0
                if duration < 1.0: duration = 1.0
                
                self.step_total = int(duration / self.timer_period)
                if self.step_total == 0: self.step_total = 1
                self.step_current = 0 
                return 

            elif self.homing_phase == 2:
                # เช็คให้แน่ใจว่า J1 หมุนมาถึง 90 องศาจริงๆ ก่อนจะพับแขนลง
                curr_j1 = self.current_angles[1] if hasattr(self, 'current_angles') else None
                if curr_j1 is not None and abs(curr_j1 - 90.0) > 2.0:
                    if self.step_current % 25 == 0: 
                        self.get_logger().info(f"⏳ กำลังรอ Joint 1 หมุนเข้าที่ 90°... (ปัจจุบัน: {curr_j1:.1f}°)")
                    return 

                # 🟢 จบเฟส 2: เริ่มเฟส 3 (พับ J2 ลงไปที่ 180 องศา และพับ J4 กลับลงไปที่ 90 องศา)
                self.get_logger().info("🏠 เฟส 2 เสร็จสิ้น! เริ่มเฟส 3 (พับแขนเก็บ: J2->180°, J4->90°)")
                self.homing_phase = 3
                self.start_joints = list(self.current_joints)
                
                # เป้าหมายเฟส 3: ท่า Home สุดท้าย 
                self.target_joints = [90.0, 180.0, 8.0, 90.0, 90.0]
                
                diff_j2 = abs(180.0 - self.current_joints[1])
                diff_j4 = abs(90.0 - self.current_joints[3])
                max_diff = max(diff_j2, diff_j4) # เอาจอยท์ที่หมุนไกลสุดมาคิดเวลา
                duration = max_diff / 30.0
                if duration < 1.5: duration = 1.5 # ให้เวลาพับลงนิ่มๆ
                
                self.step_total = int(duration / self.timer_period)
                if self.step_total == 0: self.step_total = 1
                self.step_current = 0 
                return 

            elif self.homing_phase == 3:
                # จบเฟส 3 (จบกระบวนการ Homing สมบูรณ์)
                self.get_logger().info("✅ กลับถึงท่า Home เรียบร้อย! อัปเดตพิกัด XYZ พร้อมรับคำสั่งใหม่")
                self.current_pos = list(self.final_home_pos) 
                self.homing_phase = 0
                self.is_moving = False
                
            else:
                # --- การทำงานเมื่อเดินถึงเป้าหมายของระบบ Tracking ปกติ ---
                if self.target_pos is not None:
                    self.current_pos = list(self.target_pos)
                
                self.is_moving = False 
                
                if self.is_tracking_mode:
                    # 🔥 [แก้] เอา delay ออก แล้วส่งสัญญาณให้กล้องยิงเฟรมถัดมาได้เลย!
                    self.delay_ticks = 0 
                    self.pub_tracking_ready.publish(Bool(data=True))
            
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

        # --- คำนวณ Theta 4 ---
        if -theta3_raw > theta2: 
            theta4_raw = -(theta2 + theta3_raw) + (math.pi / 2)
        elif -theta3_raw == theta2: 
            theta4_raw = math.pi / 2
        else:
            theta4_raw = (theta2 + theta3_raw)
            theta4_raw = (math.pi / 2) - theta4_raw

        # 🔥 เพิ่มเงื่อนไข Simulation สำหรับ Joint 4 (5 องศา)
        if not self.simulation_mode:
            theta4_raw += (5 * (math.pi / 180.0))
            
        theta4 = normalize_angle_positive(theta4_raw)
        theta1_for_5 = theta1

        # --- คำนวณ Theta 5 ---
        if y > 0:
            if x >= 0:
                if abs(math.degrees(theta1_for_5) - 90.0) < 0.01: theta1_for_5 = 0.0
                theta5_raw = (math.pi / 2) - theta1_for_5
                theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
                theta5_servo = theta5_raw + theta1_for_5 + theta5_phi
                if x == 0: theta5_servo = math.pi/2
                
                # 🔥 เพิ่มเงื่อนไข Simulation สำหรับ Joint 5 (7 องศา)
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
        
        # 🟢 1. กฎข้อที่ 1: จำกัดระยะการพับของ J2 เมื่อ J1 อยู่ระหว่าง 70 ถึง 110 องศา
        if 70.0 <= angle_joint1 <= 110.0:
            if not (0.0 <= angle_joint2 <= 150.0):
                # โยน Error ออกมาเพื่อสั่งเบรกหุ่นยนต์ทันทีถ้าคำนวณแล้วเกินระยะ
                raise ValueError(f"Safety Limit: เมื่อ J1={angle_joint1:.1f}°, J2 ต้องไม่เกิน 150° (คำนวณได้ {angle_joint2:.1f}°)")
        
        angle_joint3 = math.degrees(theta3)
        #angle_joint3 = abs(180.0 - math_angle_j3)
        
        angle_joint4 = math.degrees(theta4)
        angle_joint5 = math.degrees(theta5)

        return [angle_joint1, angle_joint2, angle_joint3, angle_joint4, angle_joint5]
    
    def publish_joints(self, joints):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        
        # 🔥 แปลงจาก องศา (Degree) เป็น เรเดียน (Radian) เพื่อให้ RViz แสดงผลได้ถูกต้อง
        msg.position = [math.radians(j) for j in joints] 
        self.publisher_.publish(msg)
        
        # 🔥 ถ้าเป็นโหมดซิม ไม่ต้องส่งข้อมูลความถี่สูงๆ ไปกวนระบบ Topic ของบอร์ดจริง
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
    except KeyboardInterrupt:
        node.force_go_home_before_exit()
        time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()