import rclpy
from rclpy.node import Node
import cv2
import os
import time
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from std_msgs.msg import Bool, String

class YoloWebcamFrontNode(Node):
    def __init__(self):
        super().__init__('yolo_webcam_front_node')
        
        self.model_loaded = False
        self.camera_ready = False
        self.last_retry_time = 0.0
        
        # 🟢 ตัวแปรสำหรับคุม Skip Frame
        self.frame_count = 0
        self.skip_frames = 3 # ประมวลผล 1 เฟรม ข้าม 2 เฟรม (ลดภาระลง 66%)
        
        # เก็บ Path ไว้ใช้ตอนโหลดซ้ำ
        package_share_dir = get_package_share_directory('processor_unit')
        self.model_path = os.path.join(package_share_dir, 'elevator_door.pt')
        self.camera_index = '/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0'
        
        self.get_logger().info("Starting YOLO Node (FRONT). Checking AI and Camera...")

        self.pub_door_status = self.create_publisher(Bool, '/elevator_door_status', 10)
        self.is_active = False
        self.sub_active = self.create_subscription(String, '/active_camera', self.cb_active, 10)

        # ตั้งเวลาให้ทำงานถี่ขึ้นเพื่อเช็คทั้งภาพและการเชื่อมต่อ
        self.timer = self.create_timer(0.033, self.timer_callback)

    def cb_active(self, msg):
        command = msg.data.strip().lower()
        
        # 🟢 เปิดทำงานเมื่อได้รับคำสั่ง front
        if command == "front":
            if not self.is_active:
                self.is_active = True
                self.get_logger().info("🚀 [FRONT] Node Active")
                
        # 🔴 ปิดกล้องเมื่อได้รับคำสั่ง none (เช่น ตอนกดติดแล้ว หรือกลับ Home) เท่านั้น
        elif command in ["none", "off"]:
            if self.is_active:
                self.is_active = False
                if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
                    self.cap.release()
                self.camera_ready = False
                cv2.destroyAllWindows()
                self.get_logger().info("💤 [FRONT] Camera released and Node standby")

    def timer_callback(self):
        # 1. ถ้าไม่ได้ถูกสั่งให้ Active (is_active = False) ให้หยุดทำงานทันที ไม่ต้อง Reconnect
        if not self.is_active:
            return

        current_time = time.time()
        
        # 🟢 [ระบบ Auto-Reconnect] จะทำงานเฉพาะตอนที่ is_active เป็น True เท่านั้น
        if not self.model_loaded or not self.camera_ready:
            if current_time - self.last_retry_time > 2.0:
                self.last_retry_time = current_time
                
                if not self.model_loaded:
                    try:
                        self.model = YOLO(self.model_path)
                        self.model_loaded = True
                        self.get_logger().info("✅ YOLO FRONT loaded!")
                    except Exception as e:
                        self.get_logger().error(f"⏳ AI Load Failed: {e}")
                
                if self.model_loaded and not self.camera_ready:
                    # 🟢 บังคับใช้ MJPG เพื่อแก้ปัญหาสีชมพู/เขียว
                    self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                    if self.cap.isOpened():
                        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        self.camera_ready = True
                        self.get_logger().info("✅ Webcam FRONT connected (MJPG)!")
            return

        # อ่านภาพ (เมื่อพร้อมและ Active เท่านั้น)
        ret, frame = self.cap.read()
        
        if not ret: 
            self.get_logger().warn("🚨 FRONT Camera Disconnected!")
            self.camera_ready = False
            self.cap.release()
            return

        # --- ส่วน YOLO และ Skip Frame คงเดิม ---
        self.frame_count += 1
        if self.frame_count % self.skip_frames != 0:
            return

        try:
            results = self.model(frame, imgsz=320, verbose=False)
            # ... ส่วนวิเคราะห์และ publish ...
            cv2.imshow("FRONT Camera", results[0].plot())
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloWebcamFrontNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()