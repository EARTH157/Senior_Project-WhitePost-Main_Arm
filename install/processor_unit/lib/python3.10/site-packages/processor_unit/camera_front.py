import rclpy
from rclpy.node import Node
import cv2
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from std_msgs.msg import Bool, String

class YoloWebcamFrontNode(Node):
    def __init__(self):
        super().__init__('yolo_webcam_front_node')
        
        self.get_logger().info("Loading YOLO model (FRONT)...")
        try:
            package_share_dir = get_package_share_directory('processor_unit')
            model_path = os.path.join(package_share_dir, 'elevator_door.pt')
            self.model = YOLO(model_path)
            self.get_logger().info("YOLO FRONT loaded!")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            return

        # ⚠️ อย่าลืมตั้ง Index ของกล้องหน้าให้ถูกต้อง
        self.camera_index = 0 
        self.cap = cv2.VideoCapture(self.camera_index)

        self.pub_door_status = self.create_publisher(Bool, '/elevator_door_status', 10)
        
        # 🟢 [เพิ่มใหม่] รับคำสั่งว่าให้ตื่นหรือหลับ (ค่าเริ่มต้นให้หลับไปก่อน)
        self.is_active = False
        self.sub_active = self.create_subscription(String, '/active_camera', self.cb_active, 10)

        self.timer = self.create_timer(0.033, self.timer_callback)

    def cb_active(self, msg):
        # ตื่นเฉพาะตอนที่ได้คำสั่ง "front"
        self.is_active = (msg.data.strip().lower() == "front")

    def timer_callback(self):
        if not self.is_active:
            # 🟢 [ทริคเด็ด] ใช้ cap.grab() แทน cap.read() 
            # มันจะดึงภาพทิ้งไปเฉยๆ โดยไม่เสียเวลาถอดรหัสภาพ เพื่อไม่ให้ภาพดีเลย์ตอนกลับมา active
            self.cap.grab()
            return

        ret, frame = self.cap.read()
        if not ret: return

        try:
            # รัน YOLO เฉพาะตอนที่ is_active เป็น True เท่านั้น
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
        if hasattr(self, 'cap') and self.cap.isOpened():
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