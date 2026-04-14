import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image       
from cv_bridge import CvBridge

class AprilTagFrontNode(Node):
    def __init__(self):
        super().__init__('camera_front_node')
        
        # 🚩 สวิตช์โหมด: True = Gazebo, False = กล้องจริง (USB)
        self.declare_parameter('simulation_mode', False)
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        self.camera_ready = False
        self.camera_index = '/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0' # 🟢 เปลี่ยนพอร์ตกล้องหน้าของคุณที่นี่
        
        # ==========================================
        # 🎯 ตั้งค่าระบบตรวจจับ AprilTag (มาตรฐาน Tag36h11)
        # ==========================================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # รองรับ OpenCV เวอร์ชั่นใหม่
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None # ใช้ฟังก์ชันเก่าแทน

        # 🟢 ค่าที่ต้องจูน: ระยะห่างพิกเซลระหว่าง Tag 2 ตัวตอน "ประตูลิฟต์ปิดสนิท"
        self.door_closed_dist_px = 300.0 
        self.margin_px = 50.0# ระยะคลาดเคลื่อน (ถ้ากว้างกว่า 300+80 ถือว่าประตูเปิด)
        
        self.pub_door_status = self.create_publisher(Bool, '/elevator_door_status', 10)
        self.pub_image = self.create_publisher(Image, '/yolo/front/image_result', 10)
        
        self.bridge = CvBridge()
        self.is_active = False
        self.sub_active = self.create_subscription(String, '/active_camera', self.cb_active, 10)

        if self.simulation_mode:
            self.sub_image = self.create_subscription(Image, '/camera_front/image_raw', self.image_callback, 10)
        else:
            self.timer = self.create_timer(0.033, self.timer_callback)
            
        self.get_logger().info("✅ AprilTag FRONT Node Ready! (Low-CPU Mode)")

    def cb_active(self, msg):
        command = msg.data.strip().lower()
        if command == "front":
            if not self.is_active:
                self.is_active = True
                self.get_logger().info("🚀 [FRONT] Camera Active")
        elif command in ["none", "off", "back"]:
            if self.is_active:
                self.is_active = False
                if not self.simulation_mode and hasattr(self, 'cap'):
                    self.cap.release()
                    self.camera_ready = False
                cv2.destroyAllWindows()
                self.get_logger().info("💤 [FRONT] Standby")

    # ==================================================
    # 🧠 ลอจิกหลัก: ประมวลผลภาพเพื่อหา AprilTag
    # ==================================================
    def process_frame(self, frame):
        # แปลงภาพเป็นขาวดำเพื่อให้อ่าน Tag ได้เร็วและแม่นยำขึ้น
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.detector:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        # สมมติฐานเริ่มต้น: ถ้าไม่เห็นอะไรเลย = ประตูเปิด (เผื่อคนบัง)
        door_is_open = True 
        
        if ids is not None:
            # วาดกรอบสี่เหลี่ยมรอบ Tag ที่เจอ
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # ถ้าเจอ Tag 2 ตัวขึ้นไป (ติดที่ประตูซ้าย 1 ตัว ขวา 1 ตัว)
            if len(ids) >= 2:
                # คำนวณจุดกึ่งกลางของ Tag ทั้ง 2 ตัว
                c1 = np.mean(corners[0][0], axis=0)
                c2 = np.mean(corners[1][0], axis=0)
                
                # หาระยะห่าง (Pixel Distance) ระหว่าง Tag ทั้งสอง
                distance = np.linalg.norm(c1 - c2)
                
                # วาดเส้นเชื่อมระหว่าง Tag
                cv2.line(frame, (int(c1[0]), int(c1[1])), (int(c2[0]), int(c2[1])), (0, 255, 255), 2)
                cv2.putText(frame, f"Gap: {distance:.1f} px", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                # ลอจิกประตู: ถ้าระยะห่างมากกว่าตอนปิด + ระยะขอบเขต = ประตูเปิด
                if distance > (self.door_closed_dist_px + self.margin_px):
                    door_is_open = True
                    cv2.putText(frame, "STATUS: DOOR OPEN", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                else:
                    door_is_open = False
                    cv2.putText(frame, "STATUS: DOOR CLOSED", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            else:
                # ถ้าเห็นแค่ 1 ตัว (อาจจะประตูเปิดจนซ่อน Tag หรือโดนบัง)
                cv2.putText(frame, "STATUS: DOOR OPEN (Tag Lost)", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)

        # ส่งสถานะประตูไปให้ main_processor ตัดสินใจ
        self.pub_door_status.publish(Bool(data=door_is_open))
        
        # แปลงภาพส่งขึ้น RViz2
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub_image.publish(img_msg)
        
        cv2.imshow("AprilTag FRONT Test", frame)
        cv2.waitKey(1)

    # --------------------------------------------------
    def image_callback(self, msg):
        if not self.is_active: return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Sim Image Error: {e}")

    def timer_callback(self):
        if not self.is_active: return
        
        if not self.camera_ready:
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.camera_ready = True
            else:
                self.get_logger().warn("🚨 Camera Disconnected!")
            return

        ret, frame = self.cap.read()
        if ret:
            self.process_frame(frame)

    def destroy_node(self):
        if not self.simulation_mode and hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFrontNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()