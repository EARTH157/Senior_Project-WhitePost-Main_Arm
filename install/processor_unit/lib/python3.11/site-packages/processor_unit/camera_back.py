import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
import subprocess
import os
import threading
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image       
from cv_bridge import CvBridge
from ultralytics import YOLO

class AprilTagBackNode(Node):
    def __init__(self):
        super().__init__('camera_back_node')
        
        # 🚩 สวิตช์โหมด: True = Gazebo, False = กล้องจริง (USB)
        self.declare_parameter('simulation_mode', False)
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        self.camera_ready = False
        self.camera_index = '/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0' # 🟢 พอร์ตกล้องหลัง
        
        # ==========================================
        # 🔘 สวิตช์เปิด-ปิด ระบบ YOLO และเสียง
        # ==========================================
        self.enable_yolo_audio = False  # ค่าเริ่มต้นคือ "เปิด" ทำงาน
        self.sub_toggle_yolo = self.create_subscription(Bool, '/toggle_yolo_audio', self.cb_toggle_yolo, 10)

        # ==========================================
        # 🤖 โหลดโมเดล YOLO สำหรับจับคน
        # ==========================================
        if self.enable_yolo_audio:
            self.get_logger().info("⏳ กำลังโหลดโมเดล YOLOv8n (BACK)...")
            # 🟢 เปลี่ยน Path โมเดลให้ตรงกับเครื่องของคุณ
            self.yolo_model = YOLO('/home/raspi-earth/project_ws/src/processor_unit/processor_unit/yolov8n.pt') 
            self.get_logger().info("✅ โหลดโมเดล YOLO สำเร็จ!")

        # ==========================================
        # 🎯 ตั้งค่าระบบตรวจจับ AprilTag
        # ==========================================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        self.door_closed_dist_px = 360.0 
        self.margin_px = 20.0 
        
        self.pub_door_status = self.create_publisher(Bool, '/elevator_door_status', 10)
        self.pub_image = self.create_publisher(Image, '/yolo/back/image_result', 10)
        
        self.bridge = CvBridge()
        self.is_active = False
        self.sub_active = self.create_subscription(String, '/active_camera', self.cb_active, 10)

        # ==========================================
        # 🗣️ ตั้งค่าระบบเสียงขอความช่วยเหลือ
        # ==========================================
        self.audio_file_help = "/home/raspi-earth/project_ws/src/processor_unit/processor_unit/help_press.wav" # 🟢 เปลี่ยน Path เสียงให้ถูกต้อง
        self.last_audio_time = 0.0
        self.audio_cooldown = 10.0 # หน่วงเวลาพูด 10 วินาที
        
        self.frame_count = 0 

        if self.simulation_mode:
            self.sub_image = self.create_subscription(Image, '/camera_back/image_raw', self.image_callback, 10)
        else:
            self.timer = self.create_timer(0.033, self.timer_callback)
            
        self.get_logger().info("✅ AprilTag BACK Node Ready! (YOLO/Audio Toggle Enabled)")

    # --------------------------------------------------
    # 🔘 Callback สำหรับเปิด-ปิด YOLO และเสียง
    # --------------------------------------------------
    def cb_toggle_yolo(self, msg):
        self.enable_yolo_audio = msg.data
        status = "🟢 เปิดการทำงาน (ON)" if self.enable_yolo_audio else "🔴 ปิดการทำงาน (OFF)"
        self.get_logger().info(f"🔄 ปรับสถานะ YOLO และเสียง (BACK): {status}")

    # --------------------------------------------------
    # 🎯 Callback รับคำสั่งเปิด-ปิดกล้อง
    # --------------------------------------------------
    def cb_active(self, msg):
        command = msg.data.strip().lower()
        if command in ["back", "start_back", "preset_back"]:
            if not self.is_active:
                self.is_active = True
                self.get_logger().info("🚀 [BACK] Camera Active (Started)")
        elif command in ["none", "off", "stop", "front"]:
            if self.is_active:
                self.is_active = False
                if not self.simulation_mode and hasattr(self, 'cap'):
                    self.cap.release()
                    self.camera_ready = False
                cv2.destroyAllWindows()
                self.get_logger().info("💤 [BACK] Standby")

    def request_elevator_help(self):
        audio_thread = threading.Thread(target=self.play_audio_final_boss, args=(self.audio_file_help,))
        audio_thread.start()

    def play_audio_final_boss(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().error(f"❌ ไม่พบไฟล์เสียง: {file_path}")
            return
        try:
            subprocess.run(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", file_path])
        except Exception as e:
            self.get_logger().error(f"❌ เกิดข้อผิดพลาดในการเล่นเสียง: {e}")

    # ==================================================
    # 🧠 ลอจิกหลัก: ประมวลผลภาพ (YOLO + AprilTag)
    # ==================================================
    def process_frame(self, frame):
        self.frame_count += 1
        person_detected = False

        # --------------------------------------------------
        # 1. การประมวลผล YOLO (หาคน) - ทำงานเมื่อสวิตช์เปิดอยู่
        # --------------------------------------------------
        if self.enable_yolo_audio:
            # กล้องหลังรันทุกๆ 3 เฟรม เพื่อให้ตอบสนองเร็วขึ้นตอนอยู่หน้าลิฟต์
            if self.frame_count % 3 == 0:
                results = self.yolo_model(frame, classes=[0], verbose=False)
                
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        person_detected = True 
                        
                        x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                        cv2.putText(frame, "Person", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

                if person_detected:
                    current_time = time.time()
                    if (current_time - self.last_audio_time) >= self.audio_cooldown:
                        self.last_audio_time = current_time
                        self.get_logger().info("👤 พบคน! กำลังขอความช่วยเหลือให้กดลิฟต์ (BACK)...")
                        self.request_elevator_help()

        # --------------------------------------------------
        # 2. การประมวลผล AprilTag (ประเมินประตูลิฟต์) - ทำงานตลอด
        # --------------------------------------------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.detector:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        door_is_open = True 
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if len(ids) >= 2:
                c1 = np.mean(corners[0][0], axis=0)
                c2 = np.mean(corners[1][0], axis=0)
                distance = np.linalg.norm(c1 - c2)
                
                cv2.line(frame, (int(c1[0]), int(c1[1])), (int(c2[0]), int(c2[1])), (0, 255, 255), 2)
                cv2.putText(frame, f"Gap: {distance:.1f} px", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                if distance > (self.door_closed_dist_px + self.margin_px):
                    door_is_open = True
                    cv2.putText(frame, "STATUS: DOOR OPEN", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                else:
                    door_is_open = False
                    cv2.putText(frame, "STATUS: DOOR CLOSED", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            else:
                cv2.putText(frame, "STATUS: DOOR OPEN (Tag Lost)", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)

        self.pub_door_status.publish(Bool(data=door_is_open))
        
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub_image.publish(img_msg)
        
        cv2.imshow("AprilTag + YOLO BACK", frame)
        cv2.waitKey(1)

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
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
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
    node = AprilTagBackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()