import serial

# ตั้งค่าพอร์ตและ Baud Rate ให้ตรงกับ ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200)

print("กำลังอ่านข้อมูลจาก ESP32... (กด Ctrl+C เพื่อออก)")

try:
    while True:
        # อ่านข้อมูลและแสดงผลบนหน้าจอ
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
except KeyboardInterrupt:
    print("\nปิดการอ่านข้อมูล")
    ser.close()