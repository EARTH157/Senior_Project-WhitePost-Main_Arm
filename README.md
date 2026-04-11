# 🦾 Senior Project: WhitePost - Main Arm Control

This repository contains the control system for a 5-DOF Robotic Arm and End Effector using ROS2 (Humble). The system is designed to run on a **Raspberry Pi 5** (acting as the Arm Station/Master) and a **Raspberry Pi Zero 2W** (acting as the End Effector/Vision node).

## 📌 Table of Contents

- [🦾 Senior Project: WhitePost - Main Arm Control](#-senior-project-whitepost---main-arm-control)
  - [📌 Table of Contents](#-table-of-contents)
  - [1. Environment Setup](#1-environment-setup)
  - [2. Installation \& Development Workflow](#2-installation--development-workflow)
  - [3. Remote Access (SSH)](#3-remote-access-ssh)
  - [4. Networking (Hotspot Configuration)](#4-networking-hotspot-configuration)
  - [5. Running the System (ROS2)](#5-running-the-system-ros2)
  - [6. Control Interface (ROS2 Topics)](#6-control-interface-ros2-topics)
  - [7. Hardware \& Vision (Pi Zero)](#7-hardware--vision-pi-zero)
  - [8. Service Management (Auto-start)](#8-service-management-auto-start)
  - [9. Maintenance \& Troubleshooting](#9-maintenance--troubleshooting)

---

## 1. Environment Setup

Before running any ROS2 commands, ensure you source the ROS2 installation and your local workspace:

```bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source local workspace
source install/setup.bash
```

Tip: You can add these lines to your ~/.bashrc to load them automatically.

---

## 2. Installation & Development Workflow

**Initial Setup**

Clone the repository to your Raspberry Pi or local machine:

```bash
git clone [https://github.com/EARTH157/Senior_Project-WhitePost-Main_Arm.git](https://github.com/EARTH157/Senior_Project-WhitePost-Main_Arm.git)
cd Senior_Project-WhitePost-Main_Arm
```

**Updating the Workspace (Pulling latest changes)**

Before making any edits, always ensure your workspace is up to date to avoid conflicts:

```bash
git pull origin main
```

**Editing and Saving Changes (Git Workflow)**

When you pull the workspace, edit the files, and want to save your progress back to your GitHub repository, follow these standard steps:

1. Check your changes: See which files have been modified.

```bash
git status
```

2. Stage the changes: Add all modified files to be ready for a commit.

```bash
git add .
```

3. Commit the changes: Save the changes locally with a descriptive message.

```bash
git commit -m "Update: Describe what you changed or fixed"
```

4. Push the changes: Upload your final edits back to GitHub.

```bash
git push origin main
```

(Note: You may be prompted to enter your GitHub credentials or Personal Access Token when pushing.)

---

## 3. Remote Access (SSH)

**Raspberry Pi 5 (Arm Station / Master)**

```bash
ssh raspi-earth@172.20.10.2
```

**Raspberry Pi Zero 2W (End Effector)**

```bash
ssh raspi-zero-two-w-earth@172.20.10.3
# OR via Hotspot IP:
ssh raspi-zero-two-w-earth@10.42.0.142 
```

Note: If you encounter SSH key errors, clear the old key using: ```bash ssh-keygen -R 10.42.0.142 ``` or ```bash ssh-keygen -R 172.20.10.2 ```

Password for both devices: Press **Spacebar** then **Enter.**

---

## 4. Networking (Hotspot Configuration)

The Raspberry Pi 5 acts as a Wi-Fi Master. Use the following commands to configure the network via **nmcli.**

**On Raspberry Pi 5 (Create Hotspot)**

Create a 2.4GHz Hotspot:

```bash
sudo nmcli device wifi hotspot ifname wlan0 con-name "Robot-Master-2.4G" ssid "Robot-Master-2.4G" password "robot12345678" band bg
```

Alternatively, for a 5.0GHz Hotspot:

```bash
sudo nmcli device wifi hotspot ifname wlan0 con-name "Robot-Master-5.0G" ssid "Robot-Master-5.0G" password "robot12345678" band a channel 36
```

Share LAN connection to the Hotspot & Bring it UP:

```bash
sudo nmcli connection modify "Robot-Master-2.4G" ipv4.method shared
sudo nmcli connection up "Robot-Master-2.4G"
```

**On Raspberry Pi Zero 2W (Connect to Hotspot)**

Connect to the master's network:

```bash
sudo nmcli connection add type wifi con-name "Robot-Master" ssid "Robot-Master-2.4G" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "robot12345678"
```

Set network priority (ensure it connects to the robot network first):

```bash
sudo nmcli connection modify "Robot-Master" connection.autoconnect-priority 100
sudo nmcli connection modify "preconfigured" connection.autoconnect-priority 0
```

Check connections:

```bash
nmcli connection show
ip neigh
```

---

## 5. Running the System (ROS2)

**Launch Files**

Run the main project and physics simulation/visualization:

```bash
ros2 launch launch_project launch_project.py
ros2 launch launch_project gazebo_launch.py
ros2 launch whitepost_description display.launch.py
```

**Run Individual Motor Nodes**

```bash
# Run Stepper Motor 1
ros2 run motor_control stepmotor1

# Run Servo Drive
ros2 run motor_control servodrive

# Run Joint 1 Node
ros2 run motor_control joint1_node
```

---

## 6. Control Interface (ROS2 Topics)

Use these commands to publish data to ROS2 topics and control the arm.

**High-Level State Commands**

```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'start_back'}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'preset'}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'preset_back'}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'track'}"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'home'}"
ros2 topic pub /active_camera std_msgs/msg/String "{data: 'tracker'}" --once
ros2 topic pub /active_camera std_msgs/msg/String "{data: 'off_tracker'}" --once
ros2 topic pub /active_camera std_msgs/msg/String "{data: 'front'}" --once
ros2 topic pub /active_camera std_msgs/msg/String "{data: 'back'}" --once
ros2 topic pub /active_camera std_msgs/msg/String "{data: 'off'}" --once
```

**Direct Position & Angle Control**

```bash
# Set specific stepper command
ros2 topic pub -1 /stepper_joint1_cmd std_msgs/Int32 "{data: 500}"

# Set target angle for joint 1
ros2 topic pub --once /joint1/set_target_angle std_msgs/msg/Float32 "{data: 45.0}"

# Set multiple servo angles
ros2 topic pub /servo/set_angle std_msgs/msg/Float32MultiArray "{data: [0.0, 90.0]}" --once
ros2 topic pub /servo/set_angle std_msgs/Float32MultiArray "data: [14, 45.0]" -1

# Set Cartesian target position (X, Y, Z)
ros2 topic pub --once /target_position geometry_msgs/msg/Point "{x: 0.0, y: 300.0, z: 200.0}"
ros2 topic pub --once /target_position geometry_msgs/msg/Point "{x: 600.0, y: 400.0, z: 450.0}"
```

**System Flags & Tracking**

```bash
ros2 topic pub --once /joint1/calibrate std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /toggle_tracking std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /go_home std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /go_start std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /set_target_label std_msgs/msg/String "{data: 'btn_down'}"
```

---

## 7. Hardware & Vision (Pi Zero)

**I2C Configuration**

Select I2C Channel 0 and detect devices:

```bash
i2cset -y 1 0x70 0x01
i2cdetect -y 1
```

**Computer Vision (Camera & OpenCV)**

List available cameras:

```bash
rpicam-hello --list-cameras
```

Run OpenCV script on Pi Zero:

```bash
python3 opencv_raspi-zero.py
```

(To stop the script safely: sudo pkill -f opencv_raspi-zero.py)

---

## 8. Service Management (Auto-start)

The robot system is configured to run as a **systemd** service for automatic startup.

```bash
# Edit service configuration
sudo nano /etc/systemd/system/robot.service

# Reload daemon to apply changes
sudo systemctl daemon-reload

# Service controls
sudo systemctl start robot.service
sudo systemctl restart robot.service
sudo systemctl status robot.service
```

---

## 9. Maintenance & Troubleshooting

**Process Management**

List active background jobs and kill a specific process:

```bash
jobs -l
kill -9 <PID>
sudo pkill -f python3
```

**Serial Communication (Screen)**

To access the serial monitor via USB:

```bash
screen /dev/ttyUSB0 115200
```

**🚨 How to exit screen safely:**

1. Press **Ctrl + a** together, then release.

2. Press the **k** key.

3. The system will prompt **Kill window y/n?**. Press **y** to confirm and exit.

**RRT* Path Planning Script Example**

```bash
source venv/bin/activate
python step1_rrtstar_drive_650610830.py --tracker stanley --rejoin-dist 0.25 --lookahead 0.22 --vmax 0.12 --path-mode mesh3d --path-width-m 0.02 --path-thickness 0.002 --path-z 0.00005 --path-color 0,0,0 --face-now --face-mode tangent --show
```

/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0
/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0