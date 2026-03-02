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
        self.speed_mm_s = 300.0  
        self.speed_joint_deg_s = 100.0  
        self.timer_period = 0.02 
        self.step_total = 0
        self.step_current = 0
        
        # 🔥 เพิ่มตัวแปรสำหรับนับรอบหน่วงเวลา
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
        
        # --- ตัวแปรสำหรับระบบ Camera Tracking ---
        self.is_tracking_mode = False  
        self.tracking_kp_xy = 0.05  
        self.tracking_kp_depth = 0.2 
        self.tracking_max_step = 3.0   
        
        # --- ตัวแปรสำหรับภารกิจกดปุ่มอัตโนมัติ ---
        self.press_sequence_state = 'IDLE' 
        self.state_start_time = 0.0
        self.pre_press_pos = None
        self.force_detected = False

        # รับค่าจาก UART (Force Sensor)
        self.sub_force = self.create_subscription(String, '/uart_rx_zero_2w', self.cb_force_sensor, 10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # สร้าง Subscriber รับค่าจากกล้อง และสวิตช์เปิด/ปิด
        self.sub_target_error = self.create_subscription(Point, '/target_error', self.cb_target_error, qos_profile)
        self.sub_tracking_switch = self.create_subscription(Bool, '/toggle_tracking', self.cb_toggle_tracking, 10)
        
        # เพิ่ม Subscriber รับคำสั่งกลับบ้าน
        self.sub_go_home = self.create_subscription(Bool, '/go_home', self.cb_go_home, 10)
        
        # 🔥 เพิ่ม Subscriber สำหรับรับคำสั่งไปท่าเริ่มต้น
        self.sub_go_start = self.create_subscription(Bool, '/go_start', self.cb_go_start, 10)
        
        # สร้าง Publisher ส่งสัญญาณบอกกล้องว่าหุ่นยนต์พร้อมรับค่าใหม่แล้ว
        self.pub_tracking_ready = self.create_publisher(Bool, '/tracking_ready', 10)
        
        self.cal_timer = self.create_timer(0.1, self.calibration_routine)

        # --- Subscribers ---
        self.subscription = self.create_subscription(Point, '/target_position', self.listener_callback, 10)
        
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
        
    def cb_go_start(self, msg):
        if not msg.data or not self.system_ready:
            return
            
        self.get_logger().info("▶️ ได้รับคำสั่ง GO START: กำลังไปท่าเริ่มต้น (J1=90, J2=90, J3=8, J4=180, J5=90)")
        self.is_tracking_mode = False # ปิดกล้องชั่วคราวเผื่อเปิดอยู่
        
        # ท่าเป้าหมายเริ่มต้น
        target_start_joints = [90.0, 90.0, 8.0, 180.0, 90.0]
        target_xyz = self.calculate_forward_kinematics(target_start_joints[0], target_start_joints[1], target_start_joints[2])
        
        self.move_mode = 'JOINT'
        self.target_joints = target_start_joints
        self.start_joints = list(self.current_joints)
        
        # คำนวณเวลาที่ใช้ (สมูทๆ)
        max_diff = max([abs(t - s) for t, s in zip(self.target_joints, self.start_joints)])
        duration = max_diff / 30.0 # ความเร็ว 30 องศา/วิ
        if duration < 2.0: duration = 2.0 # ขั้นต่ำ 2 วินาที
        
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        
        # อัปเดตพิกัดปลายทาง
        self.target_pos = target_xyz
        self.is_moving = True
        
    def cb_go_home(self, msg):
        if not msg.data or not self.system_ready:
            return
            
        self.get_logger().info("🏠 ได้รับคำสั่ง GO HOME: เริ่มเฟส 1 (ปรับ J2=90, J3=8, J4=90, J5=90)")
        self.is_tracking_mode = False # ปิดกล้อง
        
        # ท่า Home ปลายทางสุดท้าย (ใช้คำนวณ XYZ เก็บไว้)
        home_joints = [90.0, 180.0, 8.0, 90.0, 90.0] 
        
        # ถ้าเพิ่งเปิดเครื่องมายังไม่มีตำแหน่ง ให้วาร์ปไปเลย
        if self.current_joints is None:
            target_xyz = self.calculate_forward_kinematics(home_joints[0], home_joints[1], home_joints[2])
            self.publish_joints(home_joints)
            self.current_joints = home_joints
            self.current_pos = target_xyz
            return
            
        # คำนวณพิกัด XYZ ท่า Home เก็บไว้ใช้ตอนจบเฟสสุดท้าย
        self.final_home_pos = self.calculate_forward_kinematics(home_joints[0], home_joints[1], home_joints[2])
        
        # --- เตรียมตัวเข้าสู่ เฟส 1 ---
        self.homing_phase = 1
        self.move_mode = 'JOINT'
        self.start_joints = list(self.current_joints)
        
        # เป้าหมายเฟส 1: J1 ค้างไว้ที่เดิม, J2=90, J3=8, J4=90, J5=90
        self.target_joints = [
            self.current_joints[0], # J1 คงเดิม
            90.0,                   # J2
            8.0,                    # J3
            90.0,                   # J4
            90.0                    # J5
        ]
        
        # คำนวณเวลาที่ใช้สำหรับเฟส 1 (ดูจากองศาที่ต้องขยับเยอะสุด)
        max_diff = max([abs(tj - cj) for tj, cj in zip(self.target_joints, self.current_joints)])
        duration = max_diff / 30.0  # ความเร็ว 30 องศา/วิ (เปลี่ยนเป็น self.speed_joint_deg_s ก็ได้ครับ)
        if duration < 1.0: duration = 1.0 # ขั้นต่ำ 1 วินาที
            
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        
        self.step_current = 0
        self.is_moving = True
        
    def cb_toggle_tracking(self, msg):
        self.is_tracking_mode = msg.data
        if self.is_tracking_mode:
            self.is_moving = False 
            self.delay_ticks = 0 # รีเซ็ตดีเลย์
            self.get_logger().info("👁️ Tracking Mode: ON (กำลังล็อกเป้าหมายด้วยกล้อง...)")
            self.pub_tracking_ready.publish(Bool(data=True))
        else:
            self.get_logger().info("👁️ Tracking Mode: OFF (กลับสู่โหมดรับคำสั่งปกติ)")
            
    def cb_target_error(self, msg):
        # 1. เช็คก่อนเลยว่าข้อความเข้ามาถึงไหม 
        self.get_logger().info(f"📨 Msg received: {msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}")

        # 2. เช็คเงื่อนไข
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

        # 🔥 ถ้ากำลังพักหน่วงเวลา หรือกำลังเดินอยู่ ให้ Ignore
        if self.is_moving or self.delay_ticks > 0:
            self.get_logger().warn("⚠️ Ignore: Robot is moving or waiting")
            return

        err_x = msg.x       
        err_depth = msg.y   
        err_y = msg.z       

        # ถ้าเข้าเป้าแล้ว ให้บอกกล้อง และเริ่ม State Machine กดปุ่ม
        if err_depth == 0.0 and err_x == 0.0 and err_y == 0.0:
            if self.press_sequence_state == 'IDLE':
                self.get_logger().info("🎯 ล็อกเป้าสำเร็จ! เริ่มภารกิจกดปุ่ม...")
                self.is_tracking_mode = False # ปิดการล็อกเป้าจากกล้องชั่วคราว ไม่ให้กวนกัน
                self.press_sequence_state = 'DELAY_1'
                self.state_start_time = time.time()
                
            self.pub_tracking_ready.publish(Bool(data=True))
            return

        # คำนวณ Delta
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

        new_x = self.current_pos[0] + delta_x
        new_y = self.current_pos[1] + delta_y
        new_z = self.current_pos[2] + delta_z

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
        if duration < 0.2: 
             duration = 0.2
             
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        self.step_current = 0
        
        # ล็อคธง! เริ่มเคลื่อนที่
        self.is_moving = True 
        self.get_logger().info(f"🎯 คำนวณเป้าหมายเสร็จแล้ว! กำลังเคลื่อนที่...")

    def cb_angle1(self, msg): self.current_angles[1] = msg.data
    def cb_angle2(self, msg): self.current_angles[2] = msg.data
    def cb_angle3(self, msg): self.current_angles[3] = msg.data
        
    def cb_cal1(self, msg): self.cal_status[1] = msg.data
    def cb_cal2(self, msg): self.cal_status[2] = msg.data
    def cb_cal3(self, msg): self.cal_status[3] = msg.data
    
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
        current_time = time.time()
        
        if self.press_sequence_state == 'DELAY_1':
            if current_time - self.state_start_time >= 2.0:
                self.get_logger().info("⬆️ ชลอครบ 2 วินาที... ขยับแกน Z ขึ้น 2 cm (20 mm)")
                self.move_to_offset(0.0, 0.0, 20.0)
                self.press_sequence_state = 'WAIT_MOVE_Z'
                
        elif self.press_sequence_state == 'WAIT_MOVE_Z':
            if not self.is_moving: # รอจนกว่าจะเคลื่อนที่เสร็จ
                self.get_logger().info("⏳ ขยับขึ้นเสร็จแล้ว ชลอ 2 วินาที (DELAY_2)")
                self.state_start_time = time.time()
                self.press_sequence_state = 'DELAY_2'
                
        elif self.press_sequence_state == 'DELAY_2':
            if current_time - self.state_start_time >= 2.0:
                self.get_logger().info("👉 เริ่มกดปุ่ม! (เคลื่อนที่แกน Y จนกว่าจะเจอแรงต้าน)")
                self.pre_press_pos = list(self.current_pos) # จำตำแหน่งก่อนกดไว้
                self.force_detected = False # รีเซ็ตเซ็นเซอร์ก่อนเริ่มลุย
                # สั่งเดินหน้าล่วงหน้าไป 150 mm (ปรับระยะสุดแขนตามจริงได้ครับ)
                self.move_to_offset(0.0, 150.0, 0.0) 
                self.press_sequence_state = 'PRESSING'
                
        elif self.press_sequence_state == 'PRESSING':
            # เช็คว่าเซ็นเซอร์ชนหรือยัง (Interrupt!)
            if self.force_detected:
                self.get_logger().info("🚨 Force Active! ชนแล้ว กำลังถอยกลับ...")
                self.is_moving = False # ยกเลิกการเคลื่อนที่ปัจจุบันทันที
                self.move_to_absolute(self.pre_press_pos[0], self.pre_press_pos[1], self.pre_press_pos[2])
                self.press_sequence_state = 'RETRACTING'
                
            # ถ้าเดินจนสุดระยะที่เราเผื่อไว้ 150mm แต่ยังไม่ชน
            elif not self.is_moving:
                self.get_logger().warn("⚠️ กดจนสุดแขนแล้วแต่ไม่เจอเซ็นเซอร์ ถอยกลับ...")
                self.move_to_absolute(self.pre_press_pos[0], self.pre_press_pos[1], self.pre_press_pos[2])
                self.press_sequence_state = 'RETRACTING'
                
        elif self.press_sequence_state == 'RETRACTING':
            if not self.is_moving:
                self.get_logger().info("✅ ถอยกลับมาจุดเดิมเสร็จสิ้น! จบภารกิจ")
                self.press_sequence_state = 'IDLE'
                # ถ้าอยากให้พอกดเสร็จแล้ว กลับมารอรับเป้าหมายจากกล้องใหม่ ให้ Uncomment บรรทัดล่างครับ
                # self.is_tracking_mode = True
    
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
        if self.move_mode == 'LINEAR':
            interp_x = self.start_pos[0] + (self.target_pos[0] - self.start_pos[0]) * t
            interp_y = self.start_pos[1] + (self.target_pos[1] - self.start_pos[1]) * t
            interp_z = self.start_pos[2] + (self.target_pos[2] - self.start_pos[2]) * t
            try:
                joints = self.calculate_inverse_kinematics(interp_x, interp_y, interp_z)
                for i in range(5):
                    if abs(joints[i] - self.current_joints[i]) > 5.0:
                        self.is_moving = False
                        self.homing_phase = 0
                        return 
                self.publish_joints(joints)
                self.current_joints = joints
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

        # --- 2. การจัดการเมื่อเดินสุดทาง (t = 1.0) ---
        if t == 1.0:
            if self.homing_phase == 1:
                # จบเฟส 1: เริ่มเฟส 2 (หมุนฐาน J1 ไป 90 องศา)
                self.get_logger().info("🏠 เฟส 1 เสร็จสิ้น! เริ่มเฟส 2 (หมุนฐาน J1 ไป 90 องศา)")
                self.homing_phase = 2
                self.start_joints = list(self.current_joints)
                
                # เป้าหมายเฟส 2: เปลี่ยนแค่ J1 นอกนั้นค้างไว้
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
                # (เผื่อไว้) เช็คให้แน่ใจว่า J1 หมุนมาถึง 90 องศาจริงๆ ก่อนจะพับแขนลง
                curr_j1 = self.current_angles[1] if hasattr(self, 'current_angles') else None
                if curr_j1 is not None and abs(curr_j1 - 90.0) > 2.0:
                    if self.step_current % 25 == 0: 
                        self.get_logger().info(f"⏳ กำลังรอ Joint 1 หมุนเข้าที่ 90°... (ปัจจุบัน: {curr_j1:.1f}°)")
                    return 

                # จบเฟส 2: เริ่มเฟส 3 (พับ J2 ลงไปที่ 180 องศา)
                self.get_logger().info("🏠 เฟส 2 เสร็จสิ้น! เริ่มเฟส 3 (พับ Joint 2 ลงไปที่ 180 องศา)")
                self.homing_phase = 3
                self.start_joints = list(self.current_joints)
                
                # เป้าหมายเฟส 3: ท่า Home สุดท้าย [90, 180, 8, 90, 90]
                self.target_joints = [90.0, 180.0, 8.0, 90.0, 90.0]
                
                diff_j2 = abs(180.0 - self.current_joints[1])
                duration = diff_j2 / 30.0
                if duration < 1.5: duration = 1.5 # ให้เวลาพับลงนิ่มๆ หน่อย
                
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
                    # หน่วงเวลา 25 รอบ (0.5 วินาที) ก่อนรับเป้าหมายใหม่
                    self.delay_ticks = 25
            
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
        if not (0 <= math.degrees(theta1) <= 270.1): 
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

        if -theta3_raw > theta2: theta4_raw = -(theta2 + theta3_raw) + (math.pi / 2)  + (5 * (math.pi / 180.0))
        elif -theta3_raw == theta2: theta4_raw = math.pi / 2 + (5 * (math.pi / 180.0))
        else:
            theta4_raw = (theta2 + theta3_raw)
            theta4_raw = (math.pi / 2) - theta4_raw + (5 * (math.pi / 180.0))
            
        theta4 = normalize_angle_positive(theta4_raw)
        theta1_for_5 = theta1

        if y > 0:
            if x >= 0:
                if abs(math.degrees(theta1_for_5) - 90.0) < 0.01: theta1_for_5 = 0.0
                theta5_raw = (math.pi / 2) - theta1_for_5
                theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
                theta5_servo = theta5_raw + theta1_for_5 + theta5_phi + (7 * (math.pi / 180.0))
                if x == 0: theta5_servo = math.pi/2 + (7 * (math.pi / 180.0))
                theta5 = normalize_angle_positive(theta5_servo) 
            else:
                theta5_servo = math.pi - theta1_for_5 + (7 * (math.pi / 180.0))
                theta5 = normalize_angle_positive(theta5_servo)
        else:
            theta5_raw = (math.pi / 2) - theta1_for_5
            theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
            theta5_servo = math.pi + (theta5_raw + theta1_for_5 + theta5_phi) + (7 * (math.pi / 180.0))
            theta5 = normalize_angle_positive(theta5_servo)
        
        angle_joint1 = math.degrees(theta1)
        angle_joint2 = math.degrees(theta2)
        
        angle_joint3 = math.degrees(theta3)
        #angle_joint3 = abs(180.0 - math_angle_j3)
        
        angle_joint4 = math.degrees(theta4)
        angle_joint5 = math.degrees(theta5)

        return [angle_joint1, angle_joint2, angle_joint3, angle_joint4, angle_joint5]
    
    def publish_joints(self, joints):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        msg.position = joints
        self.publisher_.publish(msg)
        
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