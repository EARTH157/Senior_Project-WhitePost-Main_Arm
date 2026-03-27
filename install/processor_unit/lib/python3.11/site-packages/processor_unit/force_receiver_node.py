#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import time

class ForceReceiverNode(Node):
    def __init__(self):
        super().__init__('force_receiver_node')
        
        # สร้าง Publisher เพื่อส่งข้อมูลไปให้ Main_Processor
        self.publisher_ = self.create_publisher(String, '/uart_rx_zero_2w', 10)
        
        # สร้าง Thread แยกสำหรับรัน Socket Server จะได้ไม่กวนการทำงานของ ROS 2
        self.server_thread = threading.Thread(target=self.socket_server_loop)
        self.server_thread.daemon = True # ปิดโปรแกรมปุ๊บ Thread นี้ปิดตามทันที
        self.server_thread.start()
        
        self.get_logger().info("📡 Force Receiver Node Started: รอรับสัญญาณจาก Pi Zero ที่ Port 8001...")

    def socket_server_loop(self):
        HOST = '0.0.0.0'
        PORT = 8001
        
        while rclpy.ok():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind((HOST, PORT))
                    s.listen()
                    s.settimeout(1.0) # เช็ค timeout เป็นระยะเพื่อให้กด Ctrl+C ปิดโปรแกรมได้
                    
                    while rclpy.ok():
                        try:
                            conn, addr = s.accept()
                            self.get_logger().info(f"✅ เชื่อมต่อกับเซ็นเซอร์สำเร็จจาก IP: {addr}")
                            
                            with conn:
                                # ใช้ makefile เพื่อให้อ่านข้อมูลทีละบรรทัด (\n) ได้ง่ายขึ้น
                                file_conn = conn.makefile('r')
                                while rclpy.ok():
                                    line = file_conn.readline()
                                    if not line:
                                        self.get_logger().warn("🚨 สัญญาณเซ็นเซอร์ขาดหาย (Client Disconnected)")
                                        break # หลุดการเชื่อมต่อ ออกไปรอรับใหม่
                                    
                                    data = line.strip()
                                    if data == "1":
                                        # ถ้าได้รับค่า 1 (กดเซ็นเซอร์) ให้ Publish เข้า ROS 2
                                        msg = String()
                                        msg.data = "1"
                                        self.publisher_.publish(msg)
                                        self.get_logger().info("💥 เซ็นเซอร์ถูกกด! ส่งสัญญาณ '1' ไปที่ /uart_rx_zero_2w แล้ว")
                                        
                        except socket.timeout:
                            continue # ถ้ายังไม่มีใครเชื่อมต่อ ก็แค่รอรอบต่อไป
                        except Exception as inner_e:
                            self.get_logger().error(f"⚠️ ข้อผิดพลาดในการเชื่อมต่อ: {inner_e}")
                            
            except Exception as e:
                self.get_logger().error(f"❌ Server Error: {e} (กำลังเริ่มใหม่ใน 2 วินาที)")
                time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    node = ForceReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 ปิดระบบ Force Receiver")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()