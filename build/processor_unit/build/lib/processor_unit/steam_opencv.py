import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point 

import socket
import cv2
import struct
import numpy as np
import os
import math
from std_msgs.msg import Bool

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
        
        # --- ตัวแปรระบบ Tracking ---
        self.last_x = CENTER_X
        self.last_y = CENTER_Y
        self.last_r = 0 # เพิ่มตัวแปรเก็บค่ารัศมีเฟรมก่อนหน้าเพื่อทำ Smoothing
        self.is_tracking = False
        self.miss_count = 0
        self.MAX_MISS_FRAMES = 10
        self.JUMP_THRESHOLD = 150
        self.SMOOTH_FACTOR = 0.3
        self.view_mode = 0 
        
        # 🔥 เพิ่มตัวแปรสำหรับควบคุมระยะห่าง (แกน Y)
        self.TARGET_RADIUS = 80      # ขนาดรัศมีเป้าหมาย (ยิ่งเยอะ แปลว่าให้หุ่นเข้าไปใกล้) ปรับค่าได้ตามต้องการ!
        self.RADIUS_DEAD_ZONE = 40   # ระยะยอมให้คลาดเคลื่อนได้ จะได้ไม่ขยับเดินหน้าถอยหลังสั่นๆ
        
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
        cv2.createTrackbar("Param1 (Edge)", "Tuner", 100, 200, nothing)
        cv2.createTrackbar("Param2 (Thresh)", "Tuner", 45, 150, nothing) 
        cv2.createTrackbar("Min Radius", "Tuner", 20, 100, nothing)
        cv2.createTrackbar("Max Radius", "Tuner", 150, 300, nothing)
        
        # 1. สร้าง ROS 2 Publisher
        # msg.x = Error แนวนอน, msg.y = Error แนวตั้ง, msg.z = รัศมีของวัตถุ (ความใกล้-ไกล)
        self.error_pub = self.create_publisher(Point, '/target_error', 10)

        # 🔥 เพิ่ม Publisher สำหรับสั่งเปิด/ปิดโหมด Tracking ของหุ่นยนต์
        self.toggle_pub = self.create_publisher(Bool, '/toggle_tracking', 10)
        self.robot_tracking_state = False # ตัวแปรเก็บสถานะการสั่งงาน
        
        print("Controls:")
        print(" - 'v': Toggle Edge View")
        print(" - 'r': Reset Tracking")
        print(" - 'q': Quit")

    def draw_text_bg(self, img, text, pos, font_scale=0.6, color=(255, 255, 255), thickness=2):
        font = cv2.FONT_HERSHEY_SIMPLEX
        (w, h), _ = cv2.getTextSize(text, font, font_scale, thickness)
        x, y = pos
        cv2.rectangle(img, (x, y - h - 5), (x + w, y + 5), (0, 0, 0), -1) 
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

        # เช็คระยะห่าง (Depth)
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

                # 1. รับ Header
                while len(data) < self.payload_size:
                    packet = self.conn.recv(4*1024)
                    if not packet: break
                    data += packet
                
                if not data: break
                
                packed_msg_size = data[:self.payload_size]
                data = data[self.payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]
                
                # 2. รับเนื้อภาพ
                while len(data) < msg_size:
                    data += self.conn.recv(4*1024)
                    
                frame_data = data[:msg_size]
                data = data[msg_size:]

                # 3. แปลงข้อมูลเป็นภาพ
                frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)

                if frame is not None:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)

                    p1 = max(1, cv2.getTrackbarPos("Param1 (Edge)", "Tuner"))
                    p2 = max(1, cv2.getTrackbarPos("Param2 (Thresh)", "Tuner"))
                    min_r = max(1, cv2.getTrackbarPos("Min Radius", "Tuner"))
                    max_r = cv2.getTrackbarPos("Max Radius", "Tuner")

                    circles = cv2.HoughCircles(
                        gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=RES_H/4,
                        param1=p1, param2=p2, minRadius=min_r, maxRadius=max_r
                    )

                    if self.view_mode == 1:
                        edges = cv2.Canny(gray_blurred, p1/2, p1)
                        display_frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                        self.draw_text_bg(display_frame, "MODE: EDGE VIEW", (10, RES_H-20), 0.7, (0, 255, 255))
                    else:
                        display_frame = frame.copy()

                    cv2.line(display_frame, (CENTER_X, 0), (CENTER_X, RES_H), (255, 255, 0), 1)
                    cv2.line(display_frame, (0, CENTER_Y), (RES_W, CENTER_Y), (255, 255, 0), 1)
                    cv2.rectangle(display_frame, (CENTER_X-DEAD_ZONE, CENTER_Y-DEAD_ZONE),
                                (CENTER_X+DEAD_ZONE, CENTER_Y+DEAD_ZONE), (255, 255, 0), 1)

                    found_valid_target = False
                    error_x, error_y, current_r = 0.0, 0.0, 0.0 # เพิ่ม current_r
                    
                    if circles is not None:
                        circles = np.float32(np.around(circles))
                        best_circle = None
                        min_dist_to_last = 99999
                        max_radius = 0

                        for i in circles[0, :]:
                            cx, cy, r = i[0], i[1], i[2]
                            
                            if self.view_mode == 0:
                                cv2.circle(display_frame, (int(cx), int(cy)), int(r), (100, 100, 100), 1)

                            dist_from_last = math.sqrt((int(cx) - int(self.last_x))**2 + (int(cy) - int(self.last_y))**2)

                            if self.is_tracking:
                                if dist_from_last < self.JUMP_THRESHOLD and dist_from_last < min_dist_to_last:
                                    min_dist_to_last = dist_from_last
                                    best_circle = i
                            else:
                                if r > max_radius:
                                    max_radius = r
                                    best_circle = i

                        if best_circle is not None:
                            found_valid_target = True
                            self.miss_count = 0
                            self.is_tracking = True
                            
                            raw_x, raw_y, raw_r = best_circle[0], best_circle[1], best_circle[2]

                            # Smoothing แกน X, Y และ R (ความกว้างของเป้า)
                            current_x = int((raw_x * self.SMOOTH_FACTOR) + (self.last_x * (1 - self.SMOOTH_FACTOR)))
                            current_y = int((raw_y * self.SMOOTH_FACTOR) + (self.last_y * (1 - self.SMOOTH_FACTOR)))
                            current_r = int((raw_r * self.SMOOTH_FACTOR) + (self.last_r * (1 - self.SMOOTH_FACTOR)))
                            
                            self.last_x, self.last_y, self.last_r = current_x, current_y, current_r

                            cv2.circle(display_frame, (current_x, current_y), int(current_r), (0, 255, 0), 3)
                            cv2.arrowedLine(display_frame, (CENTER_X, CENTER_Y), (current_x, current_y), (0, 255, 255), 3, tipLength=0.2)

                            # คำนวณ Error ทั้ง 3 แกน
                            error_x = float(current_x - CENTER_X)
                            error_y = float(CENTER_Y - current_y)
                            error_r = float(self.TARGET_RADIUS - current_r) # 🔥 คำนวณความผิดปกติของระยะห่าง
                            
                            status_text, status_color = self.simulate_control(error_x, error_y, error_r)
                            
                            # วาดบอกค่าบนหน้าจอ
                            self.draw_text_bg(display_frame, f"Diff X: {error_x:.1f}", (current_x + 10, current_y - 25), 0.6, (255, 255, 0))
                            self.draw_text_bg(display_frame, f"Diff Z(y): {error_y:.1f}", (current_x + 10, current_y), 0.6, (255, 255, 0))
                            self.draw_text_bg(display_frame, f"Diff Y(r): {error_r:.1f}", (current_x + 10, current_y + 25), 0.6, (0, 255, 255))
                            
                            self.draw_text_bg(display_frame, status_text, (CENTER_X - 100, RES_H - 50), 1.0, status_color)

                    # สร้าง Message และส่งข้อมูลผ่าน ROS 2 Topic
                    msg = Point()
                    msg.x = float(error_x) # แกน X: ขวา/ซ้าย
                    msg.z = float(error_y) # แกน Z: ขึ้น/ลง

                    if not found_valid_target:
                        self.miss_count += 1
                        if self.miss_count > self.MAX_MISS_FRAMES:
                            self.is_tracking = False
                            self.last_x, self.last_y, self.last_r = CENTER_X, CENTER_Y, self.TARGET_RADIUS
                            self.draw_text_bg(display_frame, "NO TARGET", (CENTER_X - 80, RES_H - 50), 1.0, (100, 100, 100))
                            msg.y = 0.0 # เป็น 0 เพื่อให้หุ่นหยุดเดินหน้า/ถอยหลัง
                        else:
                            cv2.circle(display_frame, (int(self.last_x), int(self.last_y)), 5, (0, 255, 255), -1)
                            self.draw_text_bg(display_frame, "LOST signal... holding", (10, 30), 0.6, (0, 165, 255))
                            msg.y = float(self.TARGET_RADIUS - self.last_r) # จำค่า Error ความลึกเดิมไว้
                    else:
                        msg.y = float(error_r) # 🔥 ส่งค่า Error ของระยะห่างไปที่แกน Y

                    # พับลิชข้อมูลออกไป!
                    self.error_pub.publish(msg)

                    cv2.imshow("Robot View", display_frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('v'):
                        self.view_mode = 1 - self.view_mode
                    elif key == ord('r'):
                        self.is_tracking = False
                        self.miss_count = self.MAX_MISS_FRAMES + 1
                    # 🔥 เพิ่มปุ่ม 't' เพื่อส่งคำสั่งไปให้ main_processor
                    elif key == ord('t'):
                        self.robot_tracking_state = not self.robot_tracking_state
                        msg_toggle = Bool()
                        msg_toggle.data = self.robot_tracking_state
                        self.toggle_pub.publish(msg_toggle)

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