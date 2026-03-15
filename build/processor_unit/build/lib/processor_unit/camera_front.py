import rclpy
from rclpy.node import Node
import cv2
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

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
            return # หยุดการทำงานหากโหลดโมเดลไม่สำเร็จ

        # 2. เชื่อมต่อกับ USB Webcam
        # ตัวแปร camera_index: ใส่เลข 0 สำหรับกล้องตัวแรกที่เชื่อมต่อกับคอมพิวเตอร์
        self.camera_index = 1
        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open USB webcam with index {self.camera_index}")
            return
        self.get_logger().info(f"Webcam (index {self.camera_index}) connected successfully!")

        # 3. สร้าง Timer เพื่อดึงภาพมาประมวลผลเป็นระยะๆ
        # ตัวแปร timer_period: 0.033 วินาที คือการทำงานประมาณ 30 FPS (1 / 30)
        timer_period = 0.033 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # อ่านภาพจากกล้อง
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error("Failed to read frame from webcam.")
            return

        try:
            # นำภาพไปประมวลผลด้วยโมเดล AI
            results = self.model(frame)
            
            # ดึงภาพที่ถูกวาดกรอบ (Bounding Box) และ Label เรียบร้อยแล้ว
            annotated_frame = results[0].plot()
            
            # แสดงผลภาพผ่านหน้าต่าง OpenCV
            cv2.imshow("YOLO USB Webcam Test", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def destroy_node(self):
        # คืนทรัพยากรกล้องให้ระบบเมื่อเราปิด Node
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