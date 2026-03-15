import rclpy
from rclpy.node import Node
import cv2
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from std_msgs.msg import Bool # <--- เพิ่มไลบรารีส่งข้อความ

class YoloWebcamTestNode(Node):
    def __init__(self):
        super().__init__('yolo_webcam_test_node')
        
        # 1. โหลดโมเดล YOLO
        self.get_logger().info("Loading YOLO model...")
        try:
            package_share_dir = get_package_share_directory('processor_unit')
            model_path = os.path.join(package_share_dir, 'elevator_door.pt')
            self.model = YOLO(model_path)
            self.get_logger().info("YOLO model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            return

        self.camera_index = 1
        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open USB webcam with index {self.camera_index}")
            return
        self.get_logger().info(f"Webcam (index {self.camera_index}) connected successfully!")

        # --- เพิ่ม Publisher สำหรับส่งสถานะประตูลิฟต์ ---
        self.pub_door_status = self.create_publisher(Bool, '/elevator_door_status', 10)

        timer_period = 0.033 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from webcam.")
            return

        try:
            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot()
            
            # --- เช็คว่าเจอ "elevator_door_open" ไหม ---
            door_is_open = False
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                # ⚠️ ตรวจสอบชื่อคลาสให้ตรงกับที่เทรนมา (ถ้าพิมพ์ผิดแก้ตรงนี้ได้เลย)
                if cls_name == "elevator_door_open" or cls_name == "elevator_door_oprn":
                    door_is_open = True
                    break
            
            # ส่งสถานะประตูให้ Main Processor
            self.pub_door_status.publish(Bool(data=door_is_open))
            
            cv2.imshow("YOLO USB Webcam Test", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Released webcam resource.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloWebcamTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO webcam test node.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()