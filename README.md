# 🦾 Senior Project: WhitePost - Main Arm Control

This repository contains the control system for a 5-DOF Robotic Arm and End Effector using ROS2, running on Raspberry Pi 5 (Arm Station) and Raspberry Pi Zero 2W (End Effector).

## 📌 Table of Contents
1. [Installation & Setup](#1-installation--setup)
2. [Remote Access (SSH)](#2-remote-access-ssh)
3. [Networking (Hotspot Configuration)](#3-networking-hotspot-configuration)
4. [ROS2 Execution](#4-ros2-execution)
5. [Control Interface (Topic Commands)](#5-control-interface-topic-commands)
6. [Hardware & Vision (Pi Zero)](#6-hardware--vision-pi-zero)
7. [Maintenance & Troubleshooting](#7-maintenance--troubleshooting)

---

## 1. Installation & Setup
Clone the repository to your local machine or Raspberry Pi:
```bash
git clone [https://github.com/EARTH157/Senior_Project-WhitePost-Main_Arm.git](https://github.com/EARTH157/Senior_Project-WhitePost-Main_Arm.git)
cd Senior_Project-WhitePost-Main_Arm

# Clone this repository

git clone https://github.com/EARTH157/Senior_Project-WhitePost-Main_Arm.git

# remote Raspberry Pi 5 in Arm Station

ssh raspi-earth@172.20.10.2

Password: spacebar and enter

# remote Raspberry Pi Zero 2W in End effector

ssh raspi-zero-two-w-earth@172.20.10.3
ssh raspi-zero-two-w-earth@10.42.0.142
ssh-keygen -R 10.42.0.142

Password: spacebar and enter

# Kill all Process

jobs -l

kill -9 

# Run Motor

ros2 run motor_control stepmotor1

ros2 run motor_control servodrive

ros2 run motor_control joint1_node

# Run step and angle

ros2 launch launch_project launch_project.py

ros2 topic pub -1 /stepper_joint1_cmd std_msgs/Int32 "{data: 500}"

ros2 topic pub --once /joint1/calibrate std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /joint1/set_target_angle std_msgs/msg/Float32 "{data: 45.0}"

ros2 topic pub /servo/set_angle std_msgs/msg/Float32MultiArray "{data: [0.0, 90.0]}" --once

ros2 topic pub --once /target_position geometry_msgs/msg/Point "{x: 0.0, y: 300.0, z: 200.0}"

ros2 topic pub --once /target_position geometry_msgs/msg/Point "{x: 600.0, y: 400.0, z: 450.0}"

ros2 topic pub --once /toggle_tracking std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /go_home std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /go_start std_msgs/msg/Bool "{data: true}"

ros2 topic echo

ros2 topic pub --once /set_target_label std_msgs/msg/String "{data: 'button'}"

# เลือก channel 0

i2cset -y 1 0x70 0x01

i2cdetect -y 1

source venv/bin/activate

# nothing at all

ros2 topic pub /servo/set_angle std_msgs/Float32MultiArray "data: [14, 45.0]" -1

python step1_rrtstar_drive_650610830.py --tracker stanley --rejoin-dist 0.25 --lookahead 0.22 --vmax 0.12 --path-mode mesh3d 
 --path-width-m 0.02   --path-thickness 0.002 --path-z 0.00005 --path-color 0,0,0 --face-now --face-mode tangent --show

# 1. สร้าง Hotspot (ชื่อ Robot-Master รหัส robot12345678) 2.4G
sudo nmcli device wifi hotspot ifname wlan0 con-name "Robot-Master-2.4G" ssid "Robot-Master-2.4G" password "robot12345678" band bg

# 1. สร้าง Hotspot (ชื่อ Robot-Master รหัส robot12345678) 5.0G
sudo nmcli device wifi hotspot ifname wlan0 con-name "Robot-Master-5.0G" ssid "Robot-Master-5.0G" password "robot12345678" band a channel 36

# 2. ตั้งค่าให้แชร์เน็ตจากสาย LAN ไปให้ Hotspot นี้ (สำคัญมาก)
sudo nmcli connection modify "Robot-Master" ipv4.method shared

# 3. สั่งเปิดใช้งานทันที
sudo nmcli connection up "Robot-Master"

sudo nmcli connection delete "Robot-Link"

# Pi Zero two w

sudo nmcli connection add type wifi con-name "Robot-Master" ssid "Robot-Master" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "robot12345678"

# ตั้งค่า Robot-Master ให้ Priority สูง (เช่น 100)
sudo nmcli connection modify "Robot-Master" connection.autoconnect-priority 100

# ตั้งค่า preconfigured ให้ Priority ต่ำกว่า (เช่น 0)
sudo nmcli connection modify "preconfigured" connection.autoconnect-priority 0

nmcli connection show

ip neigh

python3 opencv_raspi-zero.py

sudo pkill -f opencv_raspi-zero.py

rpicam-hello --list-cameras

sudo nano /etc/systemd/system/robot.service
sudo systemctl daemon-reload
sudo systemctl start robot.service
sudo systemctl restart robot.service
sudo systemctl status robot.service

ssh-keygen -R 172.20.10.2

sudo pkill -f python3

# 🚨 วิธีออกจากโปรแกรม screen (สำคัญมาก!):

screen /dev/ttyUSB0 115200

หลายคนเปิดแล้วหาทางออกไม่เจอ วิธีปิดคือต้องกดปุ่มบนคีย์บอร์ดตามลำดับนี้ครับ:

กดปุ่ม Ctrl และ a พร้อมกัน แล้วปล่อยมือ

พิมพ์ปุ่ม k (ตัวเค)

ระบบจะถามว่าต้องการปิดไหม (Kill window y/n?) ให้กด y เพื่อยืนยันครับ

ros2 launch whitepost_description display.launch.py

ros2 launch launch_project gazebo_launch.py

ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'start'}"

ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'preset'}"

ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'track'}"

ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'home'}"