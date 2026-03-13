import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point 

import socket
import cv2
import struct
import numpy as np
import os
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
        
        self.get_logger().info("Loading YOLO model...")
        # 🔥 แก้พาธตรงนี้ให้ตรงกับของคุณ
        self.model = YOLO('/home/raspi-earth/project_ws/src/processor_unit/processor_unit/best.pt')
        
        # --- ตัวแปรระบบ Tracking ---
        self.last_x = CENTER_X
        self.last_y = CENTER_Y
        self.last_r = 0 
        self.miss_count = 0
        self.MAX_MISS_FRAMES = 10
        self.SMOOTH_FACTOR = 0.3
        
        # 🔥 [ใหม่] ชื่อเป้าหมายที่ต้องการเดินตาม (พิมพ์ตัวพิมพ์เล็กหรือใหญ่ก็ได้)
        # ถ้าต้องการให้ตามทุกอย่างแบบเดิม ให้ตั้งเป็น "all"
        self.target_label = "all"  # ลองเปลี่ยนเป็น "button" ดูครับ
        
        # ระบบจัดการ Skip Frame
        self.frame_counter = 0
        self.SKIP_FRAMES = 2 # ปรับเพิ่มเป็น 2 หรือ 3 ได้ถ้าภาพยังกระตุก

        # --- ควบคุมระยะห่าง (แกน Y) ---
        self.TARGET_RADIUS = 80      
        self.RADIUS_DEAD_ZONE = 20   
        
        self.waiting_for_robot = False
        self.last_msg_time = 0  
        
        # --- ตั้งค่า Socket ---
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
        
        # --- ตั้งค่า UI ของ OpenCV ---
        cv2.namedWindow("Tuner")
        cv2.createTrackbar("Confidence", "Tuner", 50, 100, nothing)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.error_pub = self.create_publisher(Point, '/target_error', qos_profile)
        self.toggle_pub = self.create_publisher(Bool, '/toggle_tracking', 10)
        
        self.robot_tracking_state = False 
        self.sub_ready = self.create_subscription(Bool, '/tracking_ready', self.cb_robot_ready, 10)
        
        # 🔥 [ใหม่] รับค่าคำสั่งเปลี่ยนชื่อเป้าหมายจากภายนอก
        self.sub_target = self.create_subscription(String, '/set_target_label', self.cb_set_target, 10)
        
        print("Controls:")
        print(" - 't': Toggle Tracking State (ON/OFF)")
        print(" - 'q': Quit")
        
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
                
                # ถ้ายังไม่ถึงรอบเฟรมที่ต้องการ ให้ข้ามไปเลย (ดึงข้อมูลทิ้งออกจาก Buffer เฉยๆ)
                if self.frame_counter % (self.SKIP_FRAMES + 1) != 0:
                    continue 

                # 🚀 ถอดรหัสภาพเฉพาะเฟรมที่เราจะเอาไปรัน YOLO จริงๆ (ประหยัด CPU มาก)
                frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)

                if frame is not None:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    display_frame = frame.copy()

                    cv2.line(display_frame, (CENTER_X, 0), (CENTER_X, RES_H), (255, 255, 0), 1)
                    cv2.line(display_frame, (0, CENTER_Y), (RES_W, CENTER_Y), (255, 255, 0), 1)
                    cv2.rectangle(display_frame, (CENTER_X-DEAD_ZONE, CENTER_Y-DEAD_ZONE),
                                (CENTER_X+DEAD_ZONE, CENTER_Y+DEAD_ZONE), (255, 255, 0), 1)
                    
                    # 🔥 วาดบอกว่าตอนนี้กำลังหาอะไรอยู่
                    self.draw_text_bg(display_frame, f"SEARCHING: {self.target_label.upper()}", (10, 30), 0.7, (0, 255, 255), 2, (50, 50, 50))

                    conf_thresh = cv2.getTrackbarPos("Confidence", "Tuner") / 100.0
                    if conf_thresh == 0: conf_thresh = 0.01

                    # รัน YOLO (ปรับ imgsz ลงมาเพื่อความลื่น)
                    results = self.model.predict(source=frame, conf=conf_thresh, imgsz=320, verbose=False)
                    boxes = results[0].boxes

                    found_valid_target = False
                    error_x, error_y, current_r = 0.0, 0.0, 0.0 
                    
                    if len(boxes) > 0:
                        largest_box = None
                        max_radius = 0

                        for box in boxes:
                            # ดึงข้อมูลชื่อและเปอร์เซ็นต์
                            conf = float(box.conf[0].cpu().numpy())
                            cls_id = int(box.cls[0].cpu().numpy())
                            cls_name = self.model.names[cls_id]
                            
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            cx = (x1 + x2) / 2.0
                            cy = (y1 + y2) / 2.0
                            w = x2 - x1
                            h = y2 - y1
                            r = max(w, h) / 2.0 

                            # 🔥 ตรวจสอบว่าตรงกับเป้าหมายที่ต้องการหาหรือไม่?
                            # ถ้าไม่ได้ตั้งเป็น "all" และชื่อไม่ตรงกับที่กรอกไว้ -> ข้ามไปเลย
                            if self.target_label.lower() != "all" and cls_name.lower() != self.target_label.lower():
                                # วาดเป็นกรอบสีเทาเพื่อให้รู้ว่า AI เจอ แต่ไม่ใช่เป้าหมาย
                                cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), (100, 100, 100), 1)
                                label = f"{cls_name} {conf:.2f} (Ignored)"
                                self.draw_text_bg(display_frame, label, (int(x1), int(y1) - 10), font_scale=0.4, color=(150, 150, 150), thickness=1, bg_color=(30, 30, 30))
                                continue

                            # ถ้าเป็นเป้าหมายที่ต้องการ ค่อยนำมาวาดและประมวลผลให้หุ่นยนต์
                            cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.circle(display_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                            label = f"{cls_name} {conf:.2f}"
                            self.draw_text_bg(display_frame, label, (int(x1), int(y1) - 10), font_scale=0.5, color=(0, 255, 0), thickness=1, bg_color=(50, 50, 50))

                            # เลือกเฉพาะเป้าหมายที่ใหญ่ที่สุดไปใช้
                            if r > max_radius:
                                max_radius = r
                                largest_box = (cx, cy, r)

                        if largest_box is not None:
                            found_valid_target = True
                            self.miss_count = 0
                            
                            raw_x, raw_y, raw_r = largest_box

                            current_x = int((raw_x * self.SMOOTH_FACTOR) + (self.last_x * (1 - self.SMOOTH_FACTOR)))
                            current_y = int((raw_y * self.SMOOTH_FACTOR) + (self.last_y * (1 - self.SMOOTH_FACTOR)))
                            current_r = int((raw_r * self.SMOOTH_FACTOR) + (self.last_r * (1 - self.SMOOTH_FACTOR)))
                            
                            self.last_x, self.last_y, self.last_r = current_x, current_y, current_r

                            cv2.arrowedLine(display_frame, (CENTER_X, CENTER_Y), (current_x, current_y), (0, 255, 255), 3, tipLength=0.2)

                            error_x = float(current_x - CENTER_X)
                            error_y = float(CENTER_Y - current_y)
                            error_r = float(self.TARGET_RADIUS - current_r) 
                            
                            status_text, status_color = self.simulate_control(error_x, error_y, error_r)
                            
                            self.draw_text_bg(display_frame, f"Diff X: {error_x:.1f}", (current_x + 10, current_y - 25), 0.6, (255, 255, 0))
                            self.draw_text_bg(display_frame, f"Diff Z: {error_y:.1f}", (current_x + 10, current_y), 0.6, (255, 255, 0))
                            self.draw_text_bg(display_frame, f"Diff R: {error_r:.1f}", (current_x + 10, current_y + 25), 0.6, (0, 255, 255))
                            
                            if self.waiting_for_robot and self.robot_tracking_state:
                                self.draw_text_bg(display_frame, "WAITING FOR ROBOT...", (CENTER_X - 120, RES_H - 80), 0.8, (0, 165, 255))
                            
                            self.draw_text_bg(display_frame, status_text, (CENTER_X - 100, RES_H - 50), 1.0, status_color)

                    msg = Point()
                    msg.x = float(error_x)
                    msg.z = float(error_y) 

                    if not found_valid_target:
                        self.miss_count += 1
                        if self.miss_count > self.MAX_MISS_FRAMES:
                            self.last_x, self.last_y, self.last_r = CENTER_X, CENTER_Y, self.TARGET_RADIUS
                            self.draw_text_bg(display_frame, "NO VALID TARGET", (CENTER_X - 100, RES_H - 50), 1.0, (100, 100, 100))
                            msg.y = 0.0 
                        else:
                            cv2.circle(display_frame, (int(self.last_x), int(self.last_y)), 5, (0, 255, 255), -1)
                            self.draw_text_bg(display_frame, "LOST signal... holding", (10, 60), 0.6, (0, 165, 255))
                            msg.y = float(self.TARGET_RADIUS - self.last_r) 
                    else:
                        msg.y = float(error_r) 
                        
                    current_time = time.time()
                    is_timeout = (self.waiting_for_robot and (current_time - self.last_msg_time > 5.0))

                    if self.robot_tracking_state and (not self.waiting_for_robot or is_timeout):
                        if is_timeout:
                            self.get_logger().warn("⏰ Timeout! ปลดล็อคส่งใหม่...")
                            self.waiting_for_robot = False
                            
                        needs_move = False
                        
                        if abs(error_x) < DEAD_ZONE:
                            msg.x = 0.0 
                        else:
                            needs_move = True 

                        if abs(error_y) < DEAD_ZONE: 
                            msg.z = 0.0
                        else:
                            needs_move = True

                        if abs(error_r) < self.RADIUS_DEAD_ZONE:
                            msg.y = 0.0
                        else:
                            needs_move = True

                        if needs_move:
                            self.error_pub.publish(msg)
                            self.last_msg_time = time.time()
                            self.waiting_for_robot = True 
                            self.get_logger().info(f"📤 Sent Error: X={msg.x:.1f}, Y={msg.y:.1f}, Z={msg.z:.1f}")

                    cv2.imshow("Robot View", display_frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('t'):
                        self.robot_tracking_state = not self.robot_tracking_state
                        msg_toggle = Bool()
                        msg_toggle.data = self.robot_tracking_state
                        self.toggle_pub.publish(msg_toggle)

                        if not self.robot_tracking_state:
                            self.waiting_for_robot = False

                        status_str = "ON" if self.robot_tracking_state else "OFF"
                        self.get_logger().info(f"👉 Sent Tracking Toggle: {status_str}")
                else:
                    self.get_logger().warn("⚠️ Frame Decode Error")

        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
        finally:
            self.conn.close()
            self.s.close()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = SocketTrackerNode()
    node.run_loop() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()