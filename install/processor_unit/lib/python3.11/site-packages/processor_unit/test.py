#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# Import message types
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

        # --- Robot Parameters (หน่วย mm) ---
        self.l1 = 90.0   # Base height
        self.l2 = 575.0  # Link 2
        self.l3 = 485.0  # Link 3
        self.l4 = 75.0   # Wrist 1
        self.l5 = 115.0  # Wrist 2
        self.l_end = self.l4 + self.l5 # 190 mm

        # --- Subscribers & Publishers ---
        # รับค่า x, y, z (mm)
        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self.listener_callback,
            10)
        
        # ส่งค่ามุม joint ออกไป (Radian)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        self.get_logger().info('IK Solver Node Started. Waiting for /target_position...')

    def listener_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z

        try:
            joints = self.calculate_inverse_kinematics(x, y, z)
            self.publish_joints(joints)
            self.get_logger().info(f'Input: ({x}, {y}, {z}) -> Joints: {[round(j,3) for j in joints]}')
        except ValueError as e:
            self.get_logger().error(f'Error computing IK: {e}')

    def calculate_inverse_kinematics(self, x, y, z):
        """
        คำนวณ IK สำหรับหุ่นยนต์ 5 แกน
        - แก้ไขปัญหา Angle Wrapping (มุมติดลบ)
        - เพิ่มการเช็ค Joint Limits
        """
        # --- 1. Helper Function: จัดการมุมให้อยู่ในช่วง 0 ถึง 360 องศา (0 - 2*PI) ---
        def normalize_angle_positive(angle):
            # ทำให้เป็นบวกเสมอ (0 ถึง 2PI)
            return angle % (2 * math.pi)

        # --- 2. Joint 1 (Base Rotation) ---
        # atan2 ปกติให้ค่า -PI ถึง +PI
        if y >= 0:
            theta1_raw = math.atan2(y-190.0, x)
        else:
            theta1_raw = math.atan2(y+190.0, x)
        # แปลงเป็น 0 ถึง 2PI (เพื่อให้เข้ากับ Range 0-270 ตามสเปค)
        theta1 = normalize_angle_positive(theta1_raw)
        #theta1 = theta1_raw
        if not (0 <= math.degrees(theta1) <= 270.1): # เผื่อ Error นิดหน่อย (tolerance)
            raise ValueError(f"Joint 1 Limit Exceeded: {math.degrees(theta1):.2f} deg (Max 270)")
        
        # --- 3. Wrist Position Calculation ---
        w_target = math.sqrt(x**2 + y**2)
        
        # ถอยระยะจากปลายแขน 190mm (l4+l5)
        w_wrist = w_target - self.l_end
        z_wrist = z

        # ระยะจาก Joint 2 ไป Wrist
        # คำนวณ D (ระยะจาก Joint 2 ไป Wrist)
        D_sq = w_wrist**2 + z_wrist**2
        D = math.sqrt(D_sq)

        # Check 1: แขนเอื้อมถึงหรือไม่?
        if D > (self.l2 + self.l3):
            raise ValueError(f"Target Unreachable: Distance {D:.2f} > Max Reach {self.l2 + self.l3}")
        
        # 1. Alpha: มุมเงยของเส้น D (เทียบกับแนวราบ) -> ถูกแล้ว
        alpha = math.atan2(z_wrist, w_wrist)

        # 2. Gamma: มุมภายในสามเหลี่ยมตรงหัวไหล่ (ระหว่าง L2 กับ D) -> แก้สูตรใหม่
        # สูตร: (L2^2 + D^2 - L3^2) / (2 * L2 * D)
        cos_gamma = (self.l2**2 + D_sq - self.l3**2) / (2 * self.l2 * D)
        gamma = math.acos(max(min(cos_gamma, 1.0), -1.0)) # Clamp กัน Error

        # 3. Beta: มุมภายในสามเหลี่ยมตรงข้อศอก (ระหว่าง L2 กับ L3) -> ถูกแล้ว
        cos_beta = (self.l2**2 + self.l3**2 - D_sq) / (2 * self.l2 * self.l3)
        beta = math.acos(max(min(cos_beta, 1.0), -1.0))

        # --- 5. Joint 2 (Shoulder) ---
        theta2 = alpha - gamma
        theta2 = theta2 + (2*gamma)
        # Check 2: Joint 2 เกิน Limit 180 องศาหรือไม่? (แก้ปัญหาเคสระยะประชิด)
        #theta2 = normalize_angle_positive(theta2)
        #if not (0 <= math.degrees(theta2) <= 180.1): # เผื่อ Error นิดหน่อย (tolerance)
        #    raise ValueError(f"Joint 2 Limit Exceeded: {math.degrees(theta2):.2f} deg (Max 180). Target is too close to base.")

        # --- 4. Joint 3 (Elbow) ---
        theta3 = math.pi - beta
        theta3 = -(theta3)
        theta3_raw = theta3
        theta3 = math.pi + theta3

        # --- 6. Joint 4 (Wrist Pitch) ---
        # สมการ: theta2 + theta3 + theta4 = 0 (เพื่อให้ขนานพื้น)
        if -theta3_raw > theta2:
            theta4_raw = -(theta2 + theta3_raw)
            theta4_raw = (math.pi / 2) + theta4_raw
        else:
            # มุม 2 มากกว่า มุม 3
            theta4_raw = (theta2 + theta3_raw)
            delta = theta2 - theta3_raw
            phi = (math.pi / 2) - delta
            theta4_raw = (theta4_raw + phi) - delta
        # แปลงให้เป็นบวก (แก้ปัญหาค่าติดลบ -230 -> +130)
        theta4 = normalize_angle_positive(theta4_raw)

        # --- 7. Joint 5 ---
        theta1_for_5 = theta1
        if y >= 0:
            if x >= 0:
                # เงื่อนไข: ถ้า x เป็นบวก
                
                # เช็คว่ามุมใกล้เคียง 90 องศาหรือไม่ (ใช้ abs < 0.01 แทน == เพื่อความชัวร์)
                if abs(math.degrees(theta1_for_5) - 90.0) < 0.01:
                    theta1_for_5 = 0.0
                
                # สมการที่คุณให้มา
                theta5_raw = (math.pi / 2) - theta1_for_5
                theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
                theta5_servo = theta5_raw + theta1_for_5 + theta5_phi
                # แปลงให้เป็นบวก
                theta5 = normalize_angle_positive(theta5_servo)
                if not (0 <= math.degrees(theta5) <= 180.1): # เผื่อ Error นิดหน่อย (tolerance)
                    raise ValueError(f"Joint 5 Limit Exceeded: {math.degrees(theta5):.2f} deg (Max 180)")
                
            else:
                # เงื่อนไข: ถ้า x ติดลบ
                theta5_servo = math.pi - theta1_for_5
                # แปลงให้เป็นบวก
                theta5 = normalize_angle_positive(theta5_servo)
                if not (0 <= math.degrees(theta5) <= 180.1): # เผื่อ Error นิดหน่อย (tolerance)
                    raise ValueError(f"Joint 5 Limit Exceeded: {math.degrees(theta5):.2f} deg (Max 180)")
        else:
            # เงื่อนไข: ถ้า y ติดลบ
            theta5_raw = (math.pi / 2) - theta1_for_5
            theta5_phi = math.pi - (theta5_raw + (2*theta1_for_5))
            theta5_servo = math.pi + (theta5_raw + theta1_for_5 + theta5_phi)
            #theta5_servo = ((1.5 * math.pi) + theta1_for_5) + (math.pi / 2)
            # แปลงให้เป็นบวก
            theta5 = normalize_angle_positive(theta5_servo)
            if not (0 <= math.degrees(theta5) <= 180.1): # เผื่อ Error นิดหน่อย (tolerance)
                raise ValueError(f"Joint 5 Limit Exceeded: {math.degrees(theta5):.2f} deg (Max 180)")
        
        angle_joint1 = math.degrees(theta1)
        angle_joint2 = math.degrees(theta2)
        angle_joint3 = math.degrees(theta3)
        angle_joint4 = math.degrees(theta4)
        angle_joint5 = math.degrees(theta5)

        return [angle_joint1, angle_joint2, angle_joint3, angle_joint4, angle_joint5]

    def publish_joints(self, joints):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # ตั้งชื่อ Joint ตามมาตรฐาน (ปรับได้ตาม URDF ของคุณ)
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        msg.position = joints
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()