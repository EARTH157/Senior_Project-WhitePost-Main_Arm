#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# Import message types
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, Bool

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
        self.is_moving = False   # สถานะว่าหุ่นยนต์กำลังเคลื่อนที่อยู่หรือไม่
        
        self.speed_mm_s = 50.0   # ความเร็วในการเคลื่อนที่แนวเส้นตรง (mm/sec)
        self.timer_period = 0.02 # ความถี่ 50Hz (ทำงานทุกๆ 0.02 วินาที)
        self.step_total = 0
        self.step_current = 0
        
        # --- Trajectory Parameters ---
        self.current_pos = None  
        self.target_pos = None   
        self.start_pos = None    
        
        # 🔥 สิ่งที่เพิ่มเข้ามา: เก็บมุมข้อต่อปัจจุบัน และโหมดการเคลื่อนที่
        self.current_joints = None 
        self.start_joints = None
        self.target_joints = None
        self.move_mode = 'LINEAR' # 'LINEAR' หรือ 'JOINT'
        
        self.is_moving = False   
        self.speed_mm_s = 50.0   
        self.timer_period = 0.02 
        self.step_total = 0
        self.step_current = 0
        # 🔥 เพิ่ม 2 ตัวแปรนี้สำหรับการทำ Homing 2 จังหวะ
        self.homing_phase = 0       # 0=ปกติ, 1=กำลังหมุน J1, 2=กำลังพับ J2-J5
        self.final_home_pos = None  # เก็บพิกัด XYZ ปลายทาง
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # --- Calibration State Machine Parameters ---
        self.cal_status = {1: None, 2: None, 3: None} 
        self.cal_state = 'INIT_WAIT'
        self.system_ready = False
        self.wait_time = None
        
        # --- ตัวแปรสำหรับระบบ Camera Tracking ---
        self.is_tracking_mode = False  # สวิตช์เปิด/ปิด Tracking
        self.tracking_kp_xy = 0.05     # ความไวแกนซ้ายขวา/บนล่าง (mm ต่อ 1 pixel) -> ค่อยๆ ปรับเพิ่มถ้ามันตามช้าไป
        self.tracking_kp_depth = 0.2   # ความไวแกนเดินหน้าถอยหลัง (mm ต่อ 1 หน่วยรัศมี)
        self.tracking_max_step = 3.0   # ลิมิตความเร็ว: ห้ามขยับเกิน 3 mm ต่อเฟรม (ป้องกันหุ่นกระชาก)

        # สร้าง Subscriber รับค่าจากกล้อง และสวิตช์เปิด/ปิด
        self.sub_target_error = self.create_subscription(Point, '/target_error', self.cb_target_error, 10)
        self.sub_tracking_switch = self.create_subscription(Bool, '/toggle_tracking', self.cb_toggle_tracking, 10)
        
        # 🔥 เพิ่ม Subscriber รับคำสั่งกลับบ้าน
        self.sub_go_home = self.create_subscription(Bool, '/go_home', self.cb_go_home, 10)
        
        self.cal_timer = self.create_timer(0.1, self.calibration_routine)

        # --- Subscribers ---
        self.subscription = self.create_subscription(Point, '/target_position', self.listener_callback, 10)
        
        self.sub_cal1 = self.create_subscription(Bool, '/joint1/calibrated', self.cb_cal1, 10)
        self.sub_cal2 = self.create_subscription(Bool, '/joint2/calibrated', self.cb_cal2, 10)
        self.sub_cal3 = self.create_subscription(Bool, '/joint3/calibrated', self.cb_cal3, 10)

        # --- Publishers ---
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.publisher1_ = self.create_publisher(Float32, 'joint1/set_target_angle', 10)
        self.publisher2_ = self.create_publisher(Float32, 'joint2/set_target_angle', 10)
        self.publisher3_ = self.create_publisher(Float32, 'joint3/set_target_angle', 10)
        self.publisher_servo_ = self.create_publisher(Float32MultiArray, '/servo/set_angle', 10)
        
        self.publisher1_calibrate_ = self.create_publisher(Bool, '/joint1/calibrate', 10)
        self.publisher2_calibrate_ = self.create_publisher(Bool, '/joint2/calibrate', 10)
        self.publisher3_calibrate_ = self.create_publisher(Bool, '/joint3/calibrate', 10)
        
        # เพิ่ม Subscribers สำหรับอ่านมุมปัจจุบันของแต่ละ Joint
        self.sub_angle1 = self.create_subscription(Float32, '/joint1/angle', self.cb_angle1, 10)
        self.sub_angle2 = self.create_subscription(Float32, '/joint2/angle', self.cb_angle2, 10)
        self.sub_angle3 = self.create_subscription(Float32, '/joint3/angle', self.cb_angle3, 10)
        
        self.current_angles = {1: None, 2: None, 3: None} # ตัวแปรเก็บมุมปัจจุบันของแต่ละ Joint
        
        self.get_logger().info('IK Solver Node Started. Checking Calibration Status...')
        
    def cb_go_home(self, msg):
        if not msg.data or not self.system_ready:
            return
            
        self.get_logger().info("🏠 ได้รับคำสั่ง GO HOME: เริ่มเฟส 1 (หมุนฐาน Joint 1 ไป 90 องศา)")
        self.is_tracking_mode = False # ปิดกล้อง
        
        home_joints = [90.0, 180.0, 8.0, 90.0, 90.0] 
        
        # ถ้าเพิ่งเปิดเครื่องมายังไม่มีตำแหน่ง ให้วาร์ปไปเลย
        if self.current_joints is None:
            target_xyz = self.calculate_forward_kinematics(home_joints[0], home_joints[1], home_joints[2])
            self.publish_joints(home_joints)
            self.current_joints = home_joints
            self.current_pos = target_xyz
            return
            
        # คำนวณพิกัด XYZ ท่า Home เก็บไว้ใช้ตอนจบเฟส 2
        self.final_home_pos = self.calculate_forward_kinematics(home_joints[0], home_joints[1], home_joints[2])
        
        # --- เตรียมตัวเข้าสู่ เฟส 1 ---
        self.homing_phase = 1
        self.move_mode = 'JOINT'
        self.start_joints = list(self.current_joints)
        
        # เป้าหมายเฟส 1: J1 ไป 90 ส่วน J2-J5 ให้ค้างไว้ที่มุมปัจจุบัน
        self.target_joints = [
            90.0, 
            self.current_joints[1], 
            self.current_joints[2], 
            self.current_joints[3], 
            self.current_joints[4]
        ]
        
        # คำนวณเวลาที่ใช้สำหรับเฟส 1 (อิงจากระยะทางที่ J1 ต้องหมุน)
        diff_j1 = abs(90.0 - self.current_joints[0])
        duration = diff_j1 / 30.0  # ความเร็ว 30 องศา/วิ
        if duration < 1.0: duration = 1.0 # ขั้นต่ำ 1 วินาที
            
        self.step_total = int(duration / self.timer_period)
        if self.step_total == 0: self.step_total = 1
        
        self.step_current = 0
        self.is_moving = True
        
    def cb_toggle_tracking(self, msg):
        self.is_tracking_mode = msg.data
        if self.is_tracking_mode:
            self.is_moving = False # ยกเลิกการเคลื่อนที่แบบเส้นตรง (MoveL) ถ้ารันอยู่
            self.get_logger().info("👁️ Tracking Mode: ON (กำลังล็อกเป้าหมายด้วยกล้อง...)")
        else:
            self.get_logger().info("👁️ Tracking Mode: OFF (กลับสู่โหมดรับคำสั่งปกติ)")
            
    def cb_target_error(self, msg):
        # 🔥 เพิ่มเช็ค self.homing_phase != 0
        if not self.system_ready or not self.is_tracking_mode or self.current_pos is None or self.homing_phase != 0:
            return

        err_x = msg.x      # ซ้าย/ขวา (Pixel)
        err_depth = msg.y  # ระยะห่าง (Radius Error)
        err_y = msg.z      # บน/ล่าง (Pixel)

        # 2. ถ้าหลุดเป้าหมาย (ส่งค่ามาเป็น 0 หมด หรือส่ง y=0 ตามโค้ดกล้อง) ให้หยุดนิ่ง
        if err_depth == 0.0 and err_x == 0.0 and err_y == 0.0:
            return

        # 3. แมปปิ้งแกนกล้อง -> เข้ากับแกนหุ่นยนต์ (Kinematics Mapping)
        # ⚠️ ข้อควรระวัง: คุณอาจต้องสลับเครื่องหมาย (+ เป็น -) ขึ้นอยู่กับทิศทางการติดกล้องบนปลายแขน!
        # สมมติฐาน: หุ่นยนต์ชี้หน้าไปทางแกน +X
        delta_x = err_depth * self.tracking_kp_depth  # ระยะห่าง -> เดินหน้า/ถอยหลัง (แกน X หุ่น)
        delta_y = -err_x * self.tracking_kp_xy        # ซ้าย/ขวา -> เลื่อนซ้าย/ขวา (แกน Y หุ่น)
        delta_z = err_y * self.tracking_kp_xy         # บน/ล่าง -> ยกขึ้น/ลง (แกน Z หุ่น)

        # 4. จำกัดความเร็ว (Safety Capping)
        # ป้องกันไม่ให้รับค่า Error ก้อนใหญ่แล้วหุ่นยนต์กระชาก
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        if distance > self.tracking_max_step:
            scale = self.tracking_max_step / distance
            delta_x *= scale
            delta_y *= scale
            delta_z *= scale

        # 5. คำนวณพิกัดใหม่
        new_x = self.current_pos[0] + delta_x
        new_y = self.current_pos[1] + delta_y
        new_z = self.current_pos[2] + delta_z

        # 6. ส่งคำนวณ IK และขยับข้อต่อทันที
        try:
            joints = self.calculate_inverse_kinematics(new_x, new_y, new_z)
            self.publish_joints(joints)
            
            # อัปเดตตำแหน่งปัจจุบันให้ระบบรู้
            self.current_pos = [new_x, new_y, new_z]
            
        except ValueError as e:
            # ถ้าระหว่างรักษาสมดุล แขนเอื้อมไปสุดลิมิต ก็ให้หยุดชั่วคราว ไม่ฝืน
            pass

    def cb_angle1(self, msg):
        self.current_angles[1] = msg.data

    def cb_angle2(self, msg):
        self.current_angles[2] = msg.data

    def cb_angle3(self, msg):
        self.current_angles[3] = msg.data
        
    # --- Callbacks สำหรับเช็คสถานะ Calibrate ---
    def cb_cal1(self, msg):
        self.cal_status[1] = msg.data

    def cb_cal2(self, msg):
        self.cal_status[2] = msg.data

    def cb_cal3(self, msg):
        self.cal_status[3] = msg.data

    # --- State Machine สำหรับการ Calibrate และขยับมุมอัตโนมัติ (J2 -> J1 -> J3) ---
    def calibration_routine(self):
        if self.system_ready:
            return 

        if self.cal_state == 'INIT_WAIT':
            if None not in self.cal_status.values():
                self.cal_state = 'CHECK_J2'
                self.get_logger().info("ได้รับข้อมูลสถานะจากทุก Joint แล้ว เริ่มตรวจสอบ Joint 2...")
            return

        # ---------------- ลำดับที่ 1: Joint 2 ----------------
        elif self.cal_state == 'CHECK_J2':
            if self.cal_status[2] == True:
                self.get_logger().info("Joint 2 Calibrated อยู่แล้ว สั่งขยับไปที่ 90 องศา...")
                self.publisher2_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J2'
            else:
                self.get_logger().info("กำลังสั่ง Calibrate Joint 2...")
                self.publisher2_calibrate_.publish(Bool(data=True))
                self.cal_state = 'WAIT_CALIB_J2'

        elif self.cal_state == 'WAIT_CALIB_J2':
            if self.cal_status[2] == True:
                self.get_logger().info("Joint 2 Calibrate เสร็จสิ้น! สั่งขยับไปที่ 90 องศา...")
                self.publisher2_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J2'

        elif self.cal_state == 'WAIT_ANGLE_J2':
            # รอจนกว่ามุมปัจจุบันจะขยับมาถึงช่วง 90 องศา (+/- 2.5 องศา)
            curr_angle = self.current_angles[2]
            if curr_angle is not None and abs(curr_angle - 90.0) <= 2.5:
                self.get_logger().info("✅ Joint 2 ถึงตำแหน่ง 90 องศาแล้ว ไปต่อที่ Joint 1...")
                self.cal_state = 'CHECK_J1'

        # ---------------- ลำดับที่ 2: Joint 1 ----------------
        elif self.cal_state == 'CHECK_J1':
            if self.cal_status[1] == True:
                self.get_logger().info("Joint 1 Calibrated อยู่แล้ว สั่งขยับไปที่ 90 องศา...")
                self.publisher1_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J1'
            else:
                self.get_logger().info("กำลังสั่ง Calibrate Joint 1...")
                self.publisher1_calibrate_.publish(Bool(data=True))
                self.cal_state = 'WAIT_CALIB_J1'

        elif self.cal_state == 'WAIT_CALIB_J1':
            if self.cal_status[1] == True:
                self.get_logger().info("Joint 1 Calibrate เสร็จสิ้น! สั่งขยับไปที่ 90 องศา...")
                self.publisher1_.publish(Float32(data=90.0))
                self.cal_state = 'WAIT_ANGLE_J1'

        elif self.cal_state == 'WAIT_ANGLE_J1':
            # รอจนกว่ามุมปัจจุบันจะขยับมาถึงช่วง 90 องศา (+/- 2.5 องศา)
            curr_angle = self.current_angles[1]
            if curr_angle is not None and abs(curr_angle - 90.0) <= 2.5:
                self.get_logger().info("✅ Joint 1 ถึงตำแหน่ง 90 องศาแล้ว ไปต่อที่ Joint 3...")
                self.cal_state = 'CHECK_J3'

        # ---------------- ลำดับที่ 3: Joint 3 ----------------
        elif self.cal_state == 'CHECK_J3':
            if self.cal_status[3] == True:
                self.get_logger().info("Joint 3 Calibrated อยู่แล้ว เสร็จสิ้นกระบวนการ!")
                self.cal_state = 'READY'
            else:
                self.get_logger().info("กำลังสั่ง Calibrate Joint 3...")
                self.publisher3_calibrate_.publish(Bool(data=True))
                self.cal_state = 'WAIT_CALIB_J3'

        elif self.cal_state == 'WAIT_CALIB_J3':
            if self.cal_status[3] == True:
                self.get_logger().info("✅ Joint 3 Calibrate เสร็จสิ้น! (ไม่ต้องขยับมุม)")
                self.cal_state = 'READY'

        # ---------------- เสร็จสิ้น ----------------
        elif self.cal_state == 'READY':
            self.get_logger().info("===============================")
            self.get_logger().info("🚀 ระบบพร้อมทำงานเต็มรูปแบบ!")
            self.get_logger().info("รอรับคำสั่งพิกัดใหม่ที่ /target_position...")
            self.get_logger().info("===============================")
            self.system_ready = True
            self.cal_timer.cancel()

    # --- ส่วนของการรับเป้าหมาย Cartesian (x, y, z) ---
    def listener_callback(self, msg):
        # 🔥 เพิ่มเช็ค self.homing_phase != 0
        if not self.system_ready or self.homing_phase != 0:
            self.get_logger().warning('ไม่สามารถรับพิกัดได้: ระบบยังไม่พร้อม หรือกำลังกลับบ้าน!')
            return

        target_x = msg.x
        target_y = msg.y
        target_z = msg.z

        # กรณีรับค่าครั้งแรกสุด วาปไปจุดนั้นและจำค่ามุมไว้
        if self.current_pos is None:
            self.current_pos = [target_x, target_y, target_z]
            try:
                joints = self.calculate_inverse_kinematics(target_x, target_y, target_z)
                self.current_joints = joints
                self.publish_joints(joints)
                self.get_logger().info(f'Initial Position Set: ({target_x}, {target_y}, {target_z})')
            except ValueError as e:
                self.get_logger().error(f'Error computing initial IK: {e}')
            return

        # คำนวณระยะทาง X, Y, Z
        dx = target_x - self.current_pos[0]
        dy = target_y - self.current_pos[1]
        dz = target_z - self.current_pos[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance > 0:
            try:
                # ลองคำนวณ IK ของจุดหมายปลายทางดูก่อน ว่าเอื้อมถึงไหม
                target_jts = self.calculate_inverse_kinematics(target_x, target_y, target_z)
            except ValueError as e:
                self.get_logger().error(f'Target Unreachable: {e}')
                return # เอื้อมไม่ถึง ยกเลิกคำสั่งเลย

            # 🧠 [DECISION MAKER] ตรวจสอบว่าควรใช้ MoveL หรือ MoveJ
            # หาว่ามี Joint ไหนที่ต้องหมุนเปลี่ยนองศาเยอะที่สุด
            max_angle_diff = max([abs(t - c) for t, c in zip(target_jts, self.current_joints)])
            
            # 📌 กฎการเลือก: ถ้าระยะทางเกิน 150mm หรือ ต้องหมุนข้อต่อใดๆ เกิน 45 องศา ให้ใช้ MoveJ (เพื่อความปลอดภัย)
            if distance > 150.0 or max_angle_diff > 45.0:
                self.move_mode = 'JOINT'
                self.get_logger().info(f"🤖 Auto-Mode: [MoveJ] (Max Joint Diff: {max_angle_diff:.1f}°) - ป้องกันการตีลังกา")
            else:
                self.move_mode = 'LINEAR'
                self.get_logger().info(f"🤖 Auto-Mode: [MoveL] (Distance: {distance:.1f}mm) - เคลื่อนที่เส้นตรง")

            self.target_pos = [target_x, target_y, target_z]
            self.start_pos = list(self.current_pos)
            self.target_joints = target_jts
            self.start_joints = list(self.current_joints)
            
            # คำนวณเวลาที่ใช้ (ถ้า MoveJ ให้ยึดเวลาจากองศาที่หมุนเยอะสุด / ถ้า MoveL ยึดจากระยะทาง)
            if self.move_mode == 'JOINT':
                duration = max_angle_diff / 30.0 # สมมติให้หมุนความเร็ว 30 องศา/วินาที
                if duration < (distance / self.speed_mm_s): 
                    duration = distance / self.speed_mm_s # เลือกเวลาที่นานกว่าเพื่อให้ Smooth
            else:
                duration = distance / self.speed_mm_s

            self.step_total = int(duration / self.timer_period)
            if self.step_total == 0: self.step_total = 1
                
            self.step_current = 0
            self.is_moving = True

    # --- ส่วนของการคำนวณและเดินเส้นตรง (Linear Interpolation) ---
    def timer_callback(self):
        if not self.is_moving or not self.system_ready:
            return

        self.step_current += 1
        t = self.step_current / self.step_total # t มีค่า 0.0 ถึง 1.0

        if t >= 1.0:
            t = 1.0

        # --- 1. คำนวณและสั่งขยับ (Linear หรือ Joint) ---
        if self.move_mode == 'LINEAR':
            interp_x = self.start_pos[0] + (self.target_pos[0] - self.start_pos[0]) * t
            interp_y = self.start_pos[1] + (self.target_pos[1] - self.start_pos[1]) * t
            interp_z = self.start_pos[2] + (self.target_pos[2] - self.start_pos[2]) * t

            try:
                joints = self.calculate_inverse_kinematics(interp_x, interp_y, interp_z)
                for i in range(5):
                    if abs(joints[i] - self.current_joints[i]) > 5.0:
                        self.get_logger().error(f"🛑 SINGULARITY DETECTED! Joint {i+1} jump. Stopping!")
                        self.is_moving = False
                        self.homing_phase = 0
                        return 

                self.publish_joints(joints)
                self.current_joints = joints
            except ValueError as e:
                self.get_logger().error(f'Trajectory IK Error: {e}')
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
                # ----------------------------------------------------
                # 🔥 เพิ่มการเช็คเซนเซอร์: รอให้ Joint 1 ถึง 90 องศาจริงๆ
                # ----------------------------------------------------
                # ดึงค่ามุมจริงของ Joint 1 จากเซนเซอร์
                curr_j1 = self.current_angles[1] if hasattr(self, 'current_angles') else None
                
                # ถ้าเซนเซอร์บอกว่ามุมยังห่างจาก 90 เกิน 2 องศา ให้รอไปก่อน!
                if curr_j1 is not None and abs(curr_j1 - 90.0) > 2.0:
                    if self.step_current % 25 == 0: # ปริ้นท์บอกทุกๆ 0.5 วินาที
                        self.get_logger().info(f"⏳ กำลังรอ Joint 1 หมุนเข้าที่ 90°... (ปัจจุบัน: {curr_j1:.1f}°)")
                    return # รีเทิร์นออกไปเพื่อหยุดรอ ปล่อยให้ระบบสั่ง hold ท่านี้ไว้
                
                # ----------------------------------------------------
                # ถ้าหลุดเงื่อนไขด้านบนมาได้ แปลว่า J1 ถึง 90 องศาแล้ว! ไปเฟส 2 ต่อ
                self.get_logger().info("🏠 เฟส 1 เสร็จสิ้น! เริ่มเฟส 2 (พับ Joint 2, 3, 4, 5 ลงพร้อมกัน)")
                self.homing_phase = 2
                
                self.start_joints = list(self.current_joints)
                
                # เป้าหมายเฟส 2: พับแขนทั้งหมดเก็บ (J4 = 90 ตามที่คุณขอไว้)
                self.target_joints = [90.0, 180.0, 8.0, 90.0, 90.0]
                
                # คำนวณเวลาสำหรับเฟส 2
                max_diff = max([abs(tj - cj) for tj, cj in zip(self.target_joints, self.current_joints)])
                duration = max_diff / 30.0
                if duration < 2.0: duration = 2.0
                
                self.step_total = int(duration / self.timer_period)
                if self.step_total == 0: self.step_total = 1
                self.step_current = 0 
                
                return # เริ่มขยับเฟส 2 ในรอบลูปถัดไป

            elif self.homing_phase == 2:
                # จบเฟส 2 (พับครบทุกตัว)
                self.get_logger().info("✅ กลับถึงท่า Home เรียบร้อย! อัปเดตพิกัด XYZ พร้อมรับคำสั่งใหม่")
                self.current_pos = list(self.final_home_pos) # อัปเดตพิกัดคาร์ทีเซียน
                self.homing_phase = 0
                self.is_moving = False
                
            else:
                # การสิ้นสุดของการเคลื่อนที่ปกติ (MoveL หรือ MoveJ ธรรมดา)
                if self.target_pos is not None:
                    self.current_pos = list(self.target_pos)
                self.is_moving = False

        # --- 3. แสดงผลความคืบหน้า ---
        if self.step_current % 20 == 0 or t == 1.0:
            if self.homing_phase > 0:
                pass # ซ่อนการปริ้นท์เปอร์เซ็นต์ตอน Homing จะได้ไม่รกจอ
            else:
                self.get_logger().info(f'[{self.move_mode}] Progress: {int(t*100)}%')
            
    def calculate_forward_kinematics(self, j1_deg, j2_deg, j3_deg):
        """ แปลงมุมองศากลับเป็นพิกัด X, Y, Z เพื่อไม่ให้ระบบคาร์ทีเซียนหลงทาง """
        t1 = math.radians(j1_deg)
        t2 = math.radians(j2_deg)
        t3 = math.radians(j3_deg)
        
        # 1. จาก IK: theta3 (beta) คือมุมภายในสามเหลี่ยม
        beta = t3
        
        # 2. หาระยะ D (จาก Joint 2 ไป Wrist) ด้วยกฎของโคไซน์
        D_sq = self.l2**2 + self.l3**2 - 2 * self.l2 * self.l3 * math.cos(math.pi - beta)
        D = math.sqrt(abs(D_sq))
        if D == 0: D = 0.001
        
        # 3. หามุม gamma
        cos_gamma = (self.l2**2 + D**2 - self.l3**2) / (2 * self.l2 * D)
        gamma = math.acos(max(min(cos_gamma, 1.0), -1.0))
        
        # 4. หามุม alpha (มุมเงยของแขน)
        alpha = t2 - gamma
        
        # 5. แตกเวกเตอร์หาแกน Z และระนาบ W
        z_wrist = D * math.sin(alpha)
        w_wrist = D * math.cos(alpha)
        
        z = z_wrist
        w_target = w_wrist + self.l_end
        
        # 6. แตกเวกเตอร์เข้าแกน X และ Y (พิจารณา Offset 190.0 ด้วย)
        x = w_target * math.cos(t1)
        
        if math.sin(t1) >= 0:
             y = w_target * math.sin(t1) + 190.0
        else:
             y = w_target * math.sin(t1) - 190.0
             
        return [x, y, z]
            
    # --- ส่วนการคำนวณ IK คณิตศาสตร์ของคุณ (ไม่เปลี่ยนแปลง) ---
    def calculate_inverse_kinematics(self, x, y, z):
        def normalize_angle_positive(angle):
            return angle % (2 * math.pi)

        if y >= 0:
            theta1_raw = math.atan2(y-190.0, x)
        else:
            theta1_raw = math.atan2(y+190.0, x)
            
        theta1 = normalize_angle_positive(theta1_raw)
        if not (0 <= math.degrees(theta1) <= 270.1): 
            raise ValueError(f"Joint 1 Limit Exceeded: {math.degrees(theta1):.2f} deg (Max 270)")
        
        w_target = math.sqrt(x**2 + y**2)
        w_wrist = w_target - self.l_end
        z_wrist = z

        D_sq = w_wrist**2 + z_wrist**2
        D = math.sqrt(D_sq)

        if D > (self.l2 + self.l3):
            raise ValueError(f"Target Unreachable: Distance {D:.2f} > Max Reach {self.l2 + self.l3}")
        
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
            
        theta4 = normalize_angle_positive(theta4_raw)

        theta1_for_5 = theta1
        if y > 0:
            if x >= 0:
                if abs(math.degrees(theta1_for_5) - 90.0) < 0.01:
                    theta1_for_5 = 0.0
                
                theta5_raw = (math.pi / 2) - theta1_for_5
                theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
                theta5_servo = theta5_raw + theta1_for_5 + theta5_phi + (5 * (math.pi / 180))
                
                if x == 0:
                    theta5_servo = math.pi/2 + (5 * (math.pi / 180))
                    
                theta5 = normalize_angle_positive(theta5_servo)
                if not (0 <= math.degrees(theta5) <= 180.1):
                    raise ValueError(f"Joint 5 Limit Exceeded: {math.degrees(theta5):.2f} deg (Max 180)")
                
            else:
                theta5_servo = math.pi - theta1_for_5 + (5 * (math.pi / 180))
                theta5 = normalize_angle_positive(theta5_servo)
                if not (0 <= math.degrees(theta5) <= 180.1):
                    raise ValueError(f"Joint 5 Limit Exceeded: {math.degrees(theta5):.2f} deg (Max 180)")
        else:
            theta5_raw = (math.pi / 2) - theta1_for_5
            theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
            theta5_servo = math.pi + (theta5_raw + theta1_for_5 + theta5_phi) + (5 * (math.pi / 180))
            theta5 = normalize_angle_positive(theta5_servo)
            if not (0 <= math.degrees(theta5) <= 180.1):
                raise ValueError(f"Joint 5 Limit Exceeded: {math.degrees(theta5):.2f} deg (Max 180)")
        
        angle_joint1 = math.degrees(theta1)
        angle_joint2 = math.degrees(theta2)
        angle_joint3 = math.degrees(theta3)
        angle_joint4 = math.degrees(theta4)
        angle_joint5 = math.degrees(theta5)

        return [angle_joint1, angle_joint2, angle_joint3, angle_joint4, angle_joint5]
    
    # --- ส่วนการส่งค่ามุมออกไป ---
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

        # 🔥 แก้ตรงนี้: รวบส่ง Servo รวดเดียวใน 1 Message
        # รูปแบบใหม่: [Target_Mux, Servo_Channel_1, Angle_1, Servo_Channel_2, Angle_2]
        msg_servo_combined = Float32MultiArray()
        # สมมติว่าทั้ง J4 และ J5 อยู่บน MUX Channel 0 เหมือนกัน (ถ้าคนละอันต้องบอกผมนะ)
        msg_servo_combined.data = [0.0, 0.0, float(joints[3]), 1.0, float(joints[4])]
        self.publisher_servo_.publish(msg_servo_combined)

def main(args=None):
    rclpy.init(args=args)
    node = Main_Processor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()