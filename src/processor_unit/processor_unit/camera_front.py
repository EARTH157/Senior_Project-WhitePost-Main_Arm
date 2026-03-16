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
        
        # เก็บ Path ไว้ใช้ตอนโหลดซ้ำ
        package_share_dir = get_package_share_directory('processor_unit')
        self.model_path = os.path.join(package_share_dir, 'elevator_door.pt')
        self.camera_index = 0 
        
        self.get_logger().info("Starting YOLO Node (FRONT). Checking AI and Camera...")

        self.pub_door_status = self.create_publisher(Bool, '/elevator_door_status', 10)
        self.is_active = False
        self.sub_active = self.create_subscription(String, '/active_camera', self.cb_active, 10)

        # ตั้งเวลาให้ทำงานถี่ขึ้นเพื่อเช็คทั้งภาพและการเชื่อมต่อ
        self.timer = self.create_timer(0.033, self.timer_callback)

    def cb_active(self, msg):
        self.is_active = (msg.data.strip().lower() == "front")

    def timer_callback(self):
        current_time = time.time()
        
        # 🟢 [ระบบ Auto-Reconnect] ถ้า AI หรือ กล้องยังไม่พร้อม ให้พยายามเชื่อมต่อใหม่ทุก 2 วินาที
        if not self.model_loaded or not self.camera_ready:
            if current_time - self.last_retry_time > 2.0:
                self.last_retry_time = current_time
                
                # 1. พยายามโหลด AI
                if not self.model_loaded:
                    try:
                        self.model = YOLO(self.model_path)
                        self.model_loaded = True
                        self.get_logger().info("✅ YOLO FRONT loaded successfully!")
                    except Exception as e:
                        self.get_logger().error(f"⏳ AI Load Failed. Retrying... ({e})")
                
                # 2. พยายามเปิดกล้อง (ทำหลังจาก AI โหลดเสร็จแล้วเท่านั้น)
                if self.model_loaded and not self.camera_ready:
                    self.cap = cv2.VideoCapture(self.camera_index)
                    if self.cap.isOpened():
                        self.camera_ready = True
                        self.get_logger().info("✅ Webcam FRONT connected!")
                    else:
                        self.get_logger().error("⏳ Camera FRONT not found. Retrying...")
            return # ข้ามการทำงานส่วนอื่นไปก่อนจนกว่าจะพร้อม

        # --- ส่วนการทำงานปกติเมื่อกล้องและ AI พร้อม ---
        if not self.is_active:
            self.cap.grab() 
            return

        ret, frame = self.cap.read()
        
        # 🟢 [ระบบป้องกันสายหลุด] ถ้าจู่ๆ อ่านภาพไม่ได้ (สายหลวม/หลุด) ให้รีเซ็ตสถานะกล้อง
        if not ret: 
            self.get_logger().warn("🚨 FRONT Camera Disconnected! Entering reconnect mode...")
            self.camera_ready = False
            self.cap.release()
            return

        try:
            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot()
            
            door_is_open = False
            for box in results[0].boxes:
                cls_name = self.model.names[int(box.cls[0])]
                if cls_name in ["elevator_door_open", "elevator_door_oprn"]:
                    door_is_open = True
                    break
            
            self.pub_door_status.publish(Bool(data=door_is_open))
            cv2.imshow("FRONT Camera", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

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