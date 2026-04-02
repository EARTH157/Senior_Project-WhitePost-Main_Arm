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

# 🌟 บังคับ X11 ทันทีที่ไฟล์ถูกโหลด (ก่อนคลาส Node)
os.environ["QT_QPA_PLATFORM"] = "xcb"

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
        
        # 🟢 ตัวแปรสำหรับ Auto Reconnect
        self.model_loaded = False
        self.camera_ready = False
        self.last_retry_time = 0.0
        
        self.is_active = False 
        self.was_active = False  # 🟢 เพิ่มตัวแปรเช็คสถานะหน้าต่าง
        self.cap = None
        self.s = None
        self.conn = None
        
        self.get_logger().info("Loading YOLO model...")
        package_share_dir = get_package_share_directory('processor_unit')
        
        self.model_path = os.path.join(package_share_dir, 'elevator_btn_4_best.pt') 
        
        try:
            self.model = YOLO(self.model_path)
            self.model_loaded = True 
        except Exception as e:
            self.get_logger().error(f"Init AI Failed: {e}")
        
        # --- เตรียมตัวแปรรับภาพ (Image Source) ---
        if self.simulation_mode:
            self.cap = cv2.VideoCapture(0) # เปิดกล้องคอม
            if not self.cap.isOpened():
                self.get_logger().error("❌ เปิดกล้องคอมไม่ได้!")
            self.get_logger().info("💻 Running in SIMULATION mode (Webcam)")
        else:
            self.HOST = '0.0.0.0'
            self.PORT = 8000
            self.get_logger().info(f"📡 Hardware mode selected. Waiting for activation to open port {self.PORT}.")
            self.payload_size = struct.calcsize("Q") # 8 Bytes
            self.data_buffer = b""

        # --- ตัวแปรระบบ Tracking ---
        self.last_x, self.last_y, self.last_r = CENTER_X, CENTER_Y, 0 
        self.miss_count = 0
        self.MAX_MISS_FRAMES = 10
        self.SMOOTH_FACTOR = 0.3
        self.target_label = "none"
        self.frame_counter = 0
        self.SKIP_FRAMES = 5 if not self.simulation_mode else 0 
        self.TARGET_RADIUS = 80      
        self.RADIUS_DEAD_ZONE = 20   
        self.waiting_for_robot = False
        self.last_msg_time = 0  
        self.robot_tracking_state = False 

        # --- ตั้งค่า ROS 2 (Publishers/Subscribers) ---
        cv2.namedWindow("Robot View", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Tuner", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Confidence", "Tuner", 50, 100, nothing)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.sub_active = self.create_subscription(String, '/active_camera', self.cb_active, 10)
        self.error_pub = self.create_publisher(Point, '/target_error', qos_profile)
        self.toggle_pub = self.create_publisher(Bool, '/toggle_tracking', 10)
        self.fake_force_pub = self.create_publisher(String, '/uart_rx_zero_2w', 10)
        self.sub_ready = self.create_subscription(Bool, '/tracking_ready', self.cb_robot_ready, 10)
        self.sub_target = self.create_subscription(String, '/set_target_label', self.cb_set_target, 10)
        self.cmd_pub = self.create_publisher(String, '/robot_command', 10)
        self.cmd_sub = self.create_subscription(String, '/robot_command', self.cb_robot_command, 10)
        self.pub_button_light = self.create_publisher(Bool, '/button_light_status', 10)
        
        print("Controls: 't' = Toggle, 'q' = Quit")

    # ==========================================
    # 📌 ส่วนฟังก์ชัน Callbacks และ Helper
    # ==========================================

    def cb_active(self, msg):
        command = msg.data.strip().lower()
        
        # 🟢 ให้ตื่นมารอทั้งคำสั่ง preset (front) และ preset_back (back)
        if command in ["front", "back", "tracker"]:
            if not self.is_active:
                self.is_active = True
                self.get_logger().info("🚀 [TRACKER] อุ่นเครื่องรับ Socket แล้ว!")
                
        # 🔴 ปิดกล้องหน้าต่างและการคำนวณ (แต่รักษาการเชื่อมต่อ Socket ไว้)
        elif command in ["none", "off"]:
            if self.is_active:
                self.is_active = False
                # 🟢 ไม่ปิด Socket และกล้องแล้ว! แค่แจ้งสถานะว่าเข้าสู่โหมด Standby
                self.get_logger().info("💤 [TRACKER] Standby (รักษาการเชื่อมต่อไว้แต่หยุดคำนวณ)")
        
    def cb_robot_ready(self, msg):
        if msg.data:
            self.waiting_for_robot = False

    def cb_set_target(self, msg):
        self.target_label = msg.data.strip()
        self.get_logger().info(f"🎯 Changed target to: {self.target_label}")
        
    def cb_robot_command(self, msg):
        cmd = msg.data.strip().lower()
        
        if cmd == "track":
            # 🟢 เปิดวาล์วให้ส่งค่า Error เพื่อขยับหุ่น (ทำได้ก็ต่อเมื่อตื่นอยู่เท่านั้น)
            if self.is_active:
                self.robot_tracking_state = True
                self.waiting_for_robot = False
                self.last_r = 0 
                self.get_logger().info("👀 [TRACKER] เปิดวาล์ว: เริ่มส่งพิกัดล็อกเป้า!")
                
        else:
            # 🔴 คำสั่งขยับอื่นๆ ให้ปิดวาล์วส่งพิกัดไว้ก่อน
            self.robot_tracking_state = False
            self.last_r = 0 
            self.get_logger().info(f"🛑 [TRACKER] ปิดวาล์วชั่วคราว รอหุ่นขยับ (Mode: {cmd.upper()})")

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

    # ==========================================
    # 📌 ส่วนฟังก์ชันลูปหลัก (Main Loop)
    # ==========================================

    def run_loop(self):
        data = b""
        try:
            while rclpy.ok(): 
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # 🟢 จัดการหน้าต่าง OpenCV อย่างปลอดภัยใน Main Thread
                if self.is_active and not getattr(self, 'was_active', False):
                    # ถ้าเพิ่งเปิด ให้สร้างหน้าต่าง
                    cv2.namedWindow("Robot View", cv2.WINDOW_AUTOSIZE)
                    cv2.namedWindow("Tuner", cv2.WINDOW_NORMAL)
                    cv2.createTrackbar("Confidence", "Tuner", 50, 100, nothing)
                    self.was_active = True
                
                elif not self.is_active and getattr(self, 'was_active', False):
                    # ถ้าเพิ่งปิด ให้ทำลายหน้าต่าง
                    cv2.destroyAllWindows()
                    self.was_active = False

                current_time = time.time()

                if not self.model_loaded:
                    if current_time - self.last_retry_time > 2.0:
                        self.last_retry_time = current_time
                        try:
                            self.get_logger().info("⏳ Loading YOLO model...")
                            self.model = YOLO(self.model_path)
                            self.model_loaded = True
                            self.get_logger().info("✅ YOLO model loaded!")
                        except Exception as e:
                            self.get_logger().error(f"AI Failed: {e}. Retrying...")
                    continue 

                if not self.camera_ready:
                    if current_time - self.last_retry_time > 2.0:
                        self.last_retry_time = current_time
                        if self.simulation_mode:
                            self.cap = cv2.VideoCapture(0)
                            if self.cap.isOpened():
                                self.camera_ready = True
                                self.get_logger().info("✅ Webcam connected!")
                            else:
                                self.get_logger().error("⏳ Webcam not found. Retrying...")
                        else:
                            try:
                                # 🟢 ส่วนที่แก้ไขกลับคืนมา เพื่อสร้าง Socket และรอการเชื่อมต่อ
                                if self.s is None:
                                    os.environ["QT_QPA_PLATFORM"] = "xcb"
                                    self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                                    self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                                    self.s.bind(('0.0.0.0', 8000))
                                    self.s.listen(5)
                                    self.s.settimeout(2.0)
                                
                                self.get_logger().info("🚀 Waiting for Socket connection...")
                                self.conn, self.addr = self.s.accept() # 🟢 ตัวแปร self.addr ถูกสร้างที่นี่
                                self.conn.settimeout(2.0)
                                self.camera_ready = True
                                self.payload_size = struct.calcsize("Q")
                                self.get_logger().info(f"✅ Connected by: {self.addr}")
                            except socket.timeout:
                                pass 
                            except Exception as e:
                                self.get_logger().error(f"Socket Error: {e}")
                    continue

                # --- 🚀 ส่วนดึงข้อมูลภาพ (Image Fetching) ---
                frame = None
                if self.simulation_mode:
                    ret, frame = self.cap.read()
                    if not ret:
                        self.get_logger().warn("🚨 Webcam Disconnected! Retrying...")
                        self.camera_ready = False
                        if self.cap: self.cap.release()
                        continue
                    frame = cv2.flip(frame, 1)
                else:
                    try:
                        while len(data) < self.payload_size:
                            packet = self.conn.recv(4*1024)
                            if not packet: raise ConnectionError("Stream ended")
                            data += packet
                        
                        packed_msg_size = data[:self.payload_size]
                        data = data[self.payload_size:]
                        msg_size = struct.unpack("Q", packed_msg_size)[0]
                        
                        while len(data) < msg_size:
                            packet = self.conn.recv(4*1024)
                            if not packet: raise ConnectionError("Stream ended")
                            data += packet
                            
                        frame_data = data[:msg_size]
                        data = data[msg_size:]

                        self.frame_counter += 1
                        if self.frame_counter % (self.SKIP_FRAMES + 1) != 0: continue 

                        frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
                        if frame is not None:
                            frame = cv2.rotate(frame, cv2.ROTATE_180)
                    except Exception as e:
                        self.get_logger().warn(f"🚨 Socket Disconnected! ({e}). Retrying...")
                        self.camera_ready = False
                        data = b""
                        if self.conn: self.conn.close()
                        continue
                    
                # --- 🧠 ส่วนประมวลผล (Processing) ---
                if frame is not None:
                    if not self.is_active:
                        continue
                    display_frame = frame.copy()

                    cv2.line(display_frame, (CENTER_X, 0), (CENTER_X, RES_H), (255, 255, 0), 1)
                    cv2.line(display_frame, (0, CENTER_Y), (RES_W, CENTER_Y), (255, 255, 0), 1)
                    cv2.rectangle(display_frame, (CENTER_X-DEAD_ZONE, CENTER_Y-DEAD_ZONE),
                                (CENTER_X+DEAD_ZONE, CENTER_Y+DEAD_ZONE), (255, 255, 0), 1)
                    
                    self.draw_text_bg(display_frame, f"MODE: {'SIM' if self.simulation_mode else 'HW'} | TARGET: {self.target_label.upper()}", (10, 30), 0.6, (0, 255, 255))

                    conf_thresh = cv2.getTrackbarPos("Confidence", "Tuner") / 100.0
                    if conf_thresh == 0: conf_thresh = 0.01

                    results = self.model.predict(source=frame, conf=conf_thresh, imgsz=320, verbose=False)
                    boxes = results[0].boxes

                    found_valid_target = False
                    error_x, error_y, error_r, current_r = 0.0, 0.0, 0.0, 0.0
                    
                    if len(boxes) > 0:
                        largest_box = None
                        max_radius = 0
                        
                        button_is_lit = False 

                        for box in boxes:
                            conf = float(box.conf[0].cpu().numpy())
                            cls_id = int(box.cls[0].cpu().numpy())
                            cls_name = self.model.names[cls_id]
                            
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                            r = max(x2 - x1, y2 - y1) / 2.0 

                            is_target = (self.target_label.lower() == "all" or cls_name.lower() == self.target_label.lower())
                            
                            if not is_target:
                                cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), (100, 100, 100), 1)
                                continue 
                            
                            cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(display_frame, cls_name, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                            roi = frame[int(y1):int(y2), int(x1):int(x2)]
                            if roi.size > 0:
                                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                                
                                lower_red1, upper_red1 = np.array([0, 0, 0]), np.array([50, 50, 255])
                                red_mask = cv2.inRange(hsv, lower_red1, upper_red1)
                                
                                cv2.imshow("Red Mask Debug", red_mask)
                                
                                red_pixels = cv2.countNonZero(red_mask)
                                box_area = roi.shape[0] * roi.shape[1]
                                
                                if box_area > 0 and (red_pixels / box_area) > 0.7:
                                    button_is_lit = True
                                    self.draw_text_bg(display_frame, "LIT!", (int(x1), int(y1)-25), 0.6, (0, 0, 255))
                                    cv2.rectangle(display_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)

                            if r > max_radius:
                                max_radius = r
                                largest_box = (cx, cy, r)

                        self.pub_button_light.publish(Bool(data=button_is_lit))

                        if largest_box is not None:
                            found_valid_target = True
                            raw_x, raw_y, raw_r = largest_box

                            if self.last_r == 0 or self.miss_count > 5:
                                self.last_x = raw_x
                                self.last_y = raw_y
                                self.last_r = raw_r
                                
                            self.miss_count = 0

                            current_x = int((raw_x * self.SMOOTH_FACTOR) + (self.last_x * (1 - self.SMOOTH_FACTOR)))
                            current_y = int((raw_y * self.SMOOTH_FACTOR) + (self.last_y * (1 - self.SMOOTH_FACTOR)))
                            current_r = int((raw_r * self.SMOOTH_FACTOR) + (self.last_r * (1 - self.SMOOTH_FACTOR)))
                            self.last_x, self.last_y, self.last_r = current_x, current_y, current_r

                            cv2.arrowedLine(display_frame, (CENTER_X, CENTER_Y), (current_x, current_y), (0, 255, 255), 3)

                            error_x = float(current_x - CENTER_X)
                            error_y = float(CENTER_Y - current_y)
                            error_r = float(self.TARGET_RADIUS - current_r) 
                            
                            status_text, status_color = self.simulate_control(error_x, error_y, error_r)
                            self.draw_text_bg(display_frame, status_text, (CENTER_X - 100, RES_H - 50), 1.0, status_color)
                    
                    if not found_valid_target:
                        self.miss_count += 1

                    # --- 📤 ส่วนการส่งข้อมูลไปหาหุ่นยนต์ (ROS 2 Message) ---
                    msg = Point()
                    if not found_valid_target:
                        msg.x = 9999.0
                        msg.y = 9999.0
                        msg.z = 9999.0
                        
                        if self.miss_count > self.MAX_MISS_FRAMES:
                            self.last_x, self.last_y, self.last_r = CENTER_X, CENTER_Y, self.TARGET_RADIUS
                            self.draw_text_bg(display_frame, "NO VALID TARGET", (CENTER_X - 100, RES_H - 50), 1.0, (100, 100, 100))
                        else:
                            cv2.circle(display_frame, (int(self.last_x), int(self.last_y)), 5, (0, 255, 255), -1)
                            self.draw_text_bg(display_frame, "LOST signal... holding", (10, 60), 0.6, (0, 165, 255))
                    else:
                        msg.x = float(error_x)
                        msg.z = float(error_y) 
                        msg.y = float(error_r) if found_valid_target else 0.0

                    current_time = time.time()
                    is_timeout = (self.waiting_for_robot and (current_time - self.last_msg_time > 5.0))

                    if self.robot_tracking_state and (not self.waiting_for_robot or is_timeout):
                        if is_timeout: self.waiting_for_robot = False
                        
                        self.error_pub.publish(msg)
                        self.last_msg_time = time.time()
                        self.waiting_for_robot = True 

                        if (abs(error_x) <= DEAD_ZONE and abs(error_y) <= DEAD_ZONE and abs(error_r) <= self.RADIUS_DEAD_ZONE):
                            msg.x = 0.0
                            msg.y = 0.0
                            msg.z = 0.0
                            self.get_logger().info("🎯 [TARGET LOCKED] Sending Stop/Press signal to Robot")
                        else:
                            self.get_logger().info(f"📤 Sent Error: X={msg.x:.1f}, Y={msg.y:.1f}, Z={msg.z:.1f}")
                    
                    cv2.imshow("Robot View", display_frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'): 
                        break
                    elif key == ord('s'):
                        self.cmd_pub.publish(String(data="start"))
                    elif key == ord('p'):
                        self.cmd_pub.publish(String(data="preset"))
                    elif key == ord('t'):
                        self.cmd_pub.publish(String(data="track"))
                        self.robot_tracking_state = True
                        self.waiting_for_robot = False
                        self.last_r = 0
                    elif key == ord('h'):
                        self.cmd_pub.publish(String(data="home"))
                    elif key == ord('f'):
                        self.fake_force_pub.publish(String(data="1"))
                else:
                    self.get_logger().warn("⚠️ Frame Decode Error")

        except Exception as e:
            self.get_logger().error(f"❌ Error in Loop: {e}")
        finally:
            if not self.simulation_mode and hasattr(self, 'conn') and self.conn:
                self.conn.close()
            if not self.simulation_mode and hasattr(self, 's') and self.s:
                self.s.close()
            if self.simulation_mode and hasattr(self, 'cap') and self.cap:
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