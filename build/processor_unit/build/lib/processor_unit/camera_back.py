import rclpy
from rclpy.node import Node
import cv2
import os
import time
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from std_msgs.msg import Bool, String

class YoloWebcamBackNode(Node):
    def __init__(self):
        super().__init__('yolo_webcam_back_node')
        
        self.model_loaded = False
        self.camera_ready = False
        self.last_retry_time = 0.0
        
        # 🟢 ตัวแปรสำหรับคุม Skip Frame
        self.frame_count = 0
        self.skip_frames = 3 # ประมวลผล 1 เฟรม ข้าม 2 เฟรม (ลดภาระลง 66%)
        
        package_share_dir = get_package_share_directory('processor_unit')
        self.model_path = os.path.join(package_share_dir, 'elevator_door.pt')
        self.camera_index = 2
        
        self.get_logger().info("Starting YOLO Node (BACK). Checking AI and Camera...")

        self.pub_door_status = self.create_publisher(Bool, '/elevator_door_status', 10)
        self.is_active = False
        self.sub_active = self.create_subscription(String, '/active_camera', self.cb_active, 10)

        self.timer = self.create_timer(0.033, self.timer_callback)

    def cb_active(self, msg):
        self.is_active = (msg.data.strip().lower() == "back")

    def timer_callback(self):
        current_time = time.time()
        
        # 🟢 [ระบบ Auto-Reconnect]
        if not self.model_loaded or not self.camera_ready:
            if current_time - self.last_retry_time > 2.0:
                self.last_retry_time = current_time
                
                if not self.model_loaded:
                    try:
                        self.model = YOLO(self.model_path)
                        self.model_loaded = True
                        self.get_logger().info("✅ YOLO BACK loaded successfully!")
                    except Exception as e:
                        self.get_logger().error(f"⏳ AI Load Failed. Retrying... ({e})")
                
                if self.model_loaded and not self.camera_ready:
                    self.cap = cv2.VideoCapture(self.camera_index)
                    if self.cap.isOpened():
                        # 🟢 ลดเหลือ 640x480 ทันทีที่เปิดกล้อง
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        
                        self.camera_ready = True
                        self.get_logger().info("✅ Webcam BACK connected (640x480)!")
                    else:
                        self.get_logger().error("⏳ Camera BACK not found. Retrying...")
            return

        if not self.is_active:
            self.cap.grab() 
            return

        ret, frame = self.cap.read()
        
        # 🟢 [ระบบป้องกันสายหลุด]
        if not ret: 
            self.get_logger().warn("🚨 BACK Camera Disconnected! Entering reconnect mode...")
            self.camera_ready = False
            self.cap.release()
            return

        # 🟢 [ระบบ Skip Frame]
        self.frame_count += 1
        if self.frame_count % self.skip_frames != 0:
            return 

        try:
            # 🟢 ลดขนาดภาพตอนรัน AI (imgsz=320)
            results = self.model(frame, imgsz=320, verbose=False)
            annotated_frame = results[0].plot()
            
            door_is_open = False
            for box in results[0].boxes:
                cls_name = self.model.names[int(box.cls[0])]
                if cls_name in ["elevator_door_open", "elevator_door_oprn"]:
                    door_is_open = True
                    break
            
            self.pub_door_status.publish(Bool(data=door_is_open))
            cv2.imshow("BACK Camera", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloWebcamBackNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()