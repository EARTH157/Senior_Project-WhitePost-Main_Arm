import serial
import json
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def send_servo_command(ser, channel, angle):
    """
    ฟังก์ชันสำหรับส่งคำสั่งไปที่ ESP32 เพื่อคุม PCA9685
    channel: 0 ถึง 15
    angle: 0 ถึง 180
    """
    if 0 <= channel <= 15 and 0 <= angle <= 180:
        command = f"S{channel}:{angle}\n"
        ser.write(command.encode('utf-8'))
        print(f"-> ส่งคำสั่ง: Servo ช่อง {channel} ไปที่ {angle} องศา")

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) # รอ ESP32 รีเซ็ตและพร้อมทำงาน
        
        # ตัวอย่างการสั่ง Servo ทันทีที่เชื่อมต่อเสร็จ
        send_servo_command(ser, 0, 90)   # สั่ง Servo ตัวที่ 0 ไปที่ 90 องศา
        send_servo_command(ser, 15, 180) # สั่ง Servo ตัวที่ 15 ไปที่ 180 องศา

        while True:
            # คอยอ่านค่า JSON จาก AS5600 ที่ ESP32 ส่งมา
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("{"):
                    try:
                        data = json.loads(line)
                        if "as5600" in data:
                            # นำข้อมูลดิบของเซนเซอร์ไปใช้งานต่อได้เลย
                            angles = data["as5600"]
                            # print(f"Raw Data: {angles}") 
                    except json.JSONDecodeError:
                        pass

    except Exception as e:
        print(f"เกิดข้อผิดพลาด: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
