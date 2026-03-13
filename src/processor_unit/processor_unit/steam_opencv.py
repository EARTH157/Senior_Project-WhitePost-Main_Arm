import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point 

import socket
import cv2
import struct
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import math
import time
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ultralytics import YOLO

# --- ค่าคงที่ ---
RES_W, RES_H = 640, 480
CENTER_X = RES_W // 2
CENTER_Y = RES_H // 2
DEAD_ZONE = 40

def nothing(x):
    pass

class SocketTrackerNode(Node):
    def __init__(self):
        super().__init__('socket_tracker_node')
        
        # 🚩 สวิตช์โหมด: True = ใช้กล้องคอม, False = ใช้ Socket จากหุ่นจริง
        self.declare_parameter('simulation_mode', False)
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        self.get_logger().info("Loading YOLO model...")
        package_share_dir = get_package_share_directory('processor_unit')
        model_path = os.path.join(package_share_dir, 'elevator_btn_4_best.pt')
        self.model = YOLO(model_path)
        
        # --- เตรียมตัวแปรรับภาพ (Image Source) ---
        if self.simulation_mode:
            self.cap = cv2.VideoCapture(0) # เปิดกล้องคอม
            if not self.cap.isOpened():
                self.get_logger().error("❌ เปิดกล้องคอมไม่ได้!")
            self.get_logger().info("💻 Running in SIMULATION mode (Webcam)")
        else:
            # --- ยุบรวม Socket ไว้ตรงนี้ที่เดียว ---
            os.environ["QT_QPA_PLATFORM"] = "xcb"
            self.HOST = '0.0.0.0'
            self.PORT = 8000
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.s.bind((self.HOST, self.PORT))
            self.s.listen(5)
            self.get_logger().info(f"🚀 Waiting for connection on {self.HOST}:{self.PORT}...")
            self.conn, self.addr = self.s.accept()
            self.get_logger().info(f"✅ Connected by: {self.addr}")
            self.payload_size = struct.calcsize("Q") # 8 Bytes
            self.data_buffer = b""

        # --- ตัวแปรระบบ Tracking (ใช้ร่วมกันทั้ง 2 โหมด) ---
        self.last_x, self.last_y, self.last_r = CENTER_X, CENTER_Y, 0 
        self.miss_count = 0
        self.MAX_MISS_FRAMES = 10
        self.SMOOTH_FACTOR = 0.3
        self.target_label = "all"
        self.frame_counter = 0
        self.SKIP_FRAMES = 2 if not self.simulation_mode else 0 # ซิมในคอมให้ลื่นๆ ไปเลย
        self.TARGET_RADIUS = 80      
        self.RADIUS_DEAD_ZONE = 20   
        self.waiting_for_robot = False
        self.last_msg_time = 0  
        self.robot_tracking_state = False 

        # --- ตั้งค่า ROS 2 (Publishers/Subscribers) ---
        cv2.namedWindow("Tuner")
        cv2.createTrackbar("Confidence", "Tuner", 50, 100, nothing)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.error_pub = self.create_publisher(Point, '/target_error', qos_profile)
        self.toggle_pub = self.create_publisher(Bool, '/toggle_tracking', 10)
        self.fake_force_pub = self.create_publisher(String, '/uart_rx_zero_2w', 10)
        self.sub_ready = self.create_subscription(Bool, '/tracking_ready', self.cb_robot_ready, 10)
        self.sub_target = self.create_subscription(String, '/set_target_label', self.cb_set_target, 10)
        
        print("Controls: 't' = Toggle, 'q' = Quit")
        
    def cb_robot_ready(self, msg):
        if msg.data:
            self.waiting_for_robot = False

    def cb_set_target(self, msg):
        self.target_label = msg.data.strip()
        self.get_logger().info(f"🎯 Changed target to: {self.target_label}")

    def draw_text_bg(self, img, text, pos, font_scale=0.6, color=(255, 255, 255), thickness=2, bg_color=(0, 0, 0)):
        font = cv2.FONT_HERSHEY_SIMPLEX
        (w, h), _ = cv2.getTextSize(text, font, font_scale, thickness)
        x, y = pos
        cv2.rectangle(img, (x, y - h - 5), (x + w, y + 5), bg_color, -1) 
        cv2.putText(img, text, (x, y), font, font_scale, color, thickness)

    def simulate_control(self, error_x, error_y, error_r):
        msg_x = ""
        msg_y = ""
        msg_r = ""
        is_centered = True
        
        if abs(error_x) > DEAD_ZONE:
            is_centered = False
            msg_x = "RIGHT" if error_x > 0 else "LEFT"
                
        if abs(error_y) > DEAD_ZONE:
            is_centered = False
            msg_y = "UP" if error_y > 0 else "DOWN"

        if abs(error_r) > self.RADIUS_DEAD_ZONE:
            is_centered = False
            msg_r = "FORWARD" if error_r > 0 else "BACKWARD"

        if is_centered:
            return "[ TARGET LOCKED ]", (0, 255, 0)
        else:
            parts = [p for p in [msg_x, msg_y, msg_r] if p]
            return " + ".join(parts), (0, 0, 255)

    def run_loop(self):
        data = b""
        try:
            while rclpy.ok(): 
                rclpy.spin_once(self, timeout_sec=0.01)

                # --- 🚀 ส่วนดึงข้อมูลภาพ (Image Fetching) ---
                if self.simulation_mode:
                    # โหมด Simulation: อ่านจากกล้องคอมโดยตรง
                    ret, frame = self.cap.read()
                    if not ret:
                        continue
                    # กลับด้านภาพให้เหมือนกระจก (Flip) จะได้คุมง่ายขึ้นในคอม
                    frame = cv2.flip(frame, 1)
                else:
                    # โหมด Hardware: อ่านจาก Socket Buffer เหมือนเดิม
                    while len(data) < self.payload_size:
                        packet = self.conn.recv(4*1024)
                        if not packet: break
                        data += packet
                    
                    if not data: break
                    
                    packed_msg_size = data[:self.payload_size]
                    data = data[self.payload_size:]
                    msg_size = struct.unpack("Q", packed_msg_size)[0]
                    
                    while len(data) < msg_size:
                        data += self.conn.recv(4*1024)
                        
                    frame_data = data[:msg_size]
                    data = data[msg_size:]

                    # 🚀 ทริคที่ 1: เช็คเงื่อนไข Skip เฟรม "ก่อน" ถอดรหัสภาพ!
                    self.frame_counter += 1
                    if self.frame_counter % (self.SKIP_FRAMES + 1) != 0:
                        continue 

                    # ถอดรหัสภาพจาก Buffer
                    frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        # กล้องที่ติดบนหุ่นอาจจะวางกลับหัว (180 องศา)
                        frame = cv2.rotate(frame, cv2.ROTATE_180)

                # --- 🧠 ส่วนประมวลผล (Processing) ---
                # จากจุดนี้ไป ทั้ง 2 โหมดจะใช้ Logic เดียวกันทั้งหมด
                if frame is not None:
                    display_frame = frame.copy()

                    # วาดเส้นกึ่งกลางและ Dead Zone
                    cv2.line(display_frame, (CENTER_X, 0), (CENTER_X, RES_H), (255, 255, 0), 1)
                    cv2.line(display_frame, (0, CENTER_Y), (RES_W, CENTER_Y), (255, 255, 0), 1)
                    cv2.rectangle(display_frame, (CENTER_X-DEAD_ZONE, CENTER_Y-DEAD_ZONE),
                                (CENTER_X+DEAD_ZONE, CENTER_Y+DEAD_ZONE), (255, 255, 0), 1)
                    
                    self.draw_text_bg(display_frame, f"MODE: {'SIM' if self.simulation_mode else 'HW'} | TARGET: {self.target_label.upper()}", (10, 30), 0.6, (0, 255, 255))

                    conf_thresh = cv2.getTrackbarPos("Confidence", "Tuner") / 100.0
                    if conf_thresh == 0: conf_thresh = 0.01

                    # รัน YOLO
                    results = self.model.predict(source=frame, conf=conf_thresh, imgsz=320, verbose=False)
                    boxes = results[0].boxes

                    found_valid_target = False
                    error_x, error_y, current_r = 0.0, 0.0, 0.0 
                    
                    if len(boxes) > 0:
                        largest_box = None
                        max_radius = 0

                        for box in boxes:
                            conf = float(box.conf[0].cpu().numpy())
                            cls_id = int(box.cls[0].cpu().numpy())
                            cls_name = self.model.names[cls_id]
                            
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                            r = max(x2 - x1, y2 - y1) / 2.0 

                            if self.target_label.lower() != "all" and cls_name.lower() != self.target_label.lower():
                                cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), (100, 100, 100), 1)
                                continue

                            cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            if r > max_radius:
                                max_radius = r
                                largest_box = (cx, cy, r)

                        if largest_box is not None:
                            found_valid_target = True
                            self.miss_count = 0
                            raw_x, raw_y, raw_r = largest_box

                            # Smoothing
                            current_x = int((raw_x * self.SMOOTH_FACTOR) + (self.last_x * (1 - self.SMOOTH_FACTOR)))
                            current_y = int((raw_y * self.SMOOTH_FACTOR) + (self.last_y * (1 - self.SMOOTH_FACTOR)))
                            current_r = int((raw_r * self.SMOOTH_FACTOR) + (self.last_r * (1 - self.SMOOTH_FACTOR)))
                            self.last_x, self.last_y, self.last_r = current_x, current_y, current_r

                            cv2.arrowedLine(display_frame, (CENTER_X, CENTER_Y), (current_x, current_y), (0, 255, 255), 3)

                            # คำนวณ Error
                            error_x = float(current_x - CENTER_X)
                            error_y = float(CENTER_Y - current_y)
                            error_r = float(self.TARGET_RADIUS - current_r) 
                            
                            status_text, status_color = self.simulate_control(error_x, error_y, error_r)
                            self.draw_text_bg(display_frame, status_text, (CENTER_X - 100, RES_H - 50), 1.0, status_color)

                    # --- 📤 ส่วนการส่งข้อมูลไปหาหุ่นยนต์ (ROS 2 Message) ---
                    msg = Point()
                    msg.x = float(error_x)
                    msg.z = float(error_y) 
                    msg.y = float(error_r) if found_valid_target else 0.0

                    # (ส่วนจัดการ Timeout และการ Publish เหมือนเดิม)
                    current_time = time.time()
                    is_timeout = (self.waiting_for_robot and (current_time - self.last_msg_time > 5.0))

                    if self.robot_tracking_state and (not self.waiting_for_robot or is_timeout):
                        if is_timeout: self.waiting_for_robot = False
                        
                        # ตรวจสอบว่า Error เกิน Dead Zone หรือไม่
                        if (abs(error_x) > DEAD_ZONE or abs(error_y) > DEAD_ZONE or abs(error_r) > self.RADIUS_DEAD_ZONE):
                            self.error_pub.publish(msg)
                            self.last_msg_time = time.time()
                            self.waiting_for_robot = True 
                            self.get_logger().info(f"📤 Sent Error: X={msg.x:.1f}, Y={msg.y:.1f}, Z={msg.z:.1f}")

                    cv2.imshow("Robot View", display_frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'): break
                    elif key == ord('t'):
                        self.robot_tracking_state = not self.robot_tracking_state
                        self.toggle_pub.publish(Bool(data=self.robot_tracking_state))
                        if not self.robot_tracking_state: self.waiting_for_robot = False
                        self.get_logger().info(f"👉 Tracking: {'ON' if self.robot_tracking_state else 'OFF'}")
                    elif key == ord('f'):
                        self.fake_force_pub.publish(String(data="1"))
                        self.get_logger().info("💥 [SIMULATION] จำลองการชนเป้าหมาย!")
                else:
                    self.get_logger().warn("⚠️ Frame Decode Error")

        except Exception as e:
            self.get_logger().error(f"❌ Error in Loop: {e}")
        finally:
            if not self.simulation_mode:
                self.conn.close()
                self.s.close()
            else:
                self.cap.release()
            cv2.destroyAllWindows()
            
def main(args=None):
    rclpy.init(args=args)
    node = SocketTrackerNode()
    node.run_loop() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()