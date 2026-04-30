#!/usr/bin/env python3
"""
📡 TCP Stream Viewer — กล้องหน้า (camera_front, port 9100)

วิธีใช้:
    python3 ws_viewer_front.py                    # เชื่อมต่อ localhost
    python3 ws_viewer_front.py --host 192.168.1.10 # เชื่อมต่อ IP อื่น

กด 'q' เพื่อปิดหน้าต่าง
"""
import cv2
import numpy as np
import socket
import struct
import argparse
import sys
import time


def recv_exact(sock, n):
    """รับข้อมูลจาก socket ให้ครบ n bytes"""
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def main():
    parser = argparse.ArgumentParser(description='📡 TCP Stream Viewer — กล้องหน้า')
    parser.add_argument('--host', type=str, default='localhost', help='IP ของ Raspberry Pi (default: localhost)')
    parser.add_argument('--port', type=int, default=9100, help='Port ของ stream server (default: 9100)')
    args = parser.parse_args()

    print(f"📡 กำลังเชื่อมต่อ {args.host}:{args.port} ...")
    
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((args.host, args.port))
            sock.settimeout(None)  # ถอด timeout หลัง connect สำเร็จ — รอรับข้อมูลไม่จำกัดเวลา
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"✅ เชื่อมต่อสำเร็จ! กด 'q' เพื่อปิด")
            
            while True:
                # อ่าน header 4 bytes (ขนาด JPEG)
                header = recv_exact(sock, 4)
                if header is None:
                    print("⚠️ การเชื่อมต่อหลุด! กำลังเชื่อมต่อใหม่...")
                    break
                
                frame_size = struct.unpack('>I', header)[0]
                
                # Keep-alive packet (length=0) — กล้องยังไม่ active, รอต่อ
                if frame_size == 0:
                    continue
                
                # อ่าน JPEG data
                jpeg_data = recv_exact(sock, frame_size)
                if jpeg_data is None:
                    print("⚠️ การเชื่อมต่อหลุด! กำลังเชื่อมต่อใหม่...")
                    break
                
                # Decode JPEG → OpenCV frame
                np_arr = np.frombuffer(jpeg_data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    cv2.imshow("Camera FRONT Stream", frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("👋 ปิดโปรแกรม")
                        sock.close()
                        cv2.destroyAllWindows()
                        cv2.waitKey(1)
                        return
                        
        except socket.timeout:
            print(f"⏳ หมดเวลาเชื่อมต่อ — ลองใหม่ใน 2 วินาที...")
        except ConnectionRefusedError:
            print(f"❌ ไม่สามารถเชื่อมต่อ {args.host}:{args.port} — ลองใหม่ใน 2 วินาที...")
        except KeyboardInterrupt:
            print("\n👋 ปิดโปรแกรม")
            cv2.destroyAllWindows()
            cv2.waitKey(1)
            sys.exit(0)
        except Exception as e:
            print(f"❌ Error: {e}")
        
        try:
            sock.close()
        except:
            pass
        
        time.sleep(2.0)


if __name__ == '__main__':
    main()
