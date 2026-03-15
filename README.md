🤖 WhitePost Senior Project: Control GuideThis document contains all essential commands for the WhitePost Robotic Arm project.🌐 1. Connection & Remote AccessUse 'spacebar' then 'Enter' as the password for all devices.Raspberry Pi 5 (Arm Station)# Connect to Pi 5
ssh raspi-earth@172.20.10.2

# If RSA key error occurs, reset it with:
ssh-keygen -R 172.20.10.2
Raspberry Pi Zero 2W (End Effector)# Connect to Pi Zero via Hotspot or Local IP
ssh raspi-zero-two-w-earth@172.20.10.3
ssh raspi-zero-two-w-earth@10.42.0.142

# If RSA key error occurs for specific IP:
ssh-keygen -R 10.42.0.142
📶 2. Network Setup (Hotspot)Use these commands on Pi 5 to manage the Robot-Master network.Create Hotspot (Pi 5)# Setup 2.4GHz Hotspot
sudo nmcli device wifi hotspot ifname wlan0 con-name "Robot-Master-2.4G" ssid "Robot-Master-2.4G" password "robot12345678" band bg

# Setup 5.0GHz Hotspot (Recommended for speed)
sudo nmcli device wifi hotspot ifname wlan0 con-name "Robot-Master-5.0G" ssid "Robot-Master-5.0G" password "robot12345678" band a channel 36

# Share internet from Ethernet (LAN) to Hotspot
sudo nmcli connection modify "Robot-Master-5.0G" ipv4.method shared
sudo nmcli connection up "Robot-Master-5.0G"
Client Setup (Pi Zero)# Add Master Hotspot to Pi Zero
sudo nmcli connection add type wifi con-name "Robot-Master" ssid "Robot-Master" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "robot12345678"

# Set connection priority (100 = Highest)
sudo nmcli connection modify "Robot-Master" connection.autoconnect-priority 100
sudo nmcli connection modify "preconfigured" connection.autoconnect-priority 0
🚀 3. ROS2 OperationsCommands to run and control the robotic arm.Execution# Option A: Run individual nodes
ros2 run motor_control stepmotor1
ros2 run motor_control servodrive
ros2 run motor_control joint1_node

# Option B: Run via Launch file (Simulation or Real)
ros2 launch launch_project launch_project.py
ros2 launch launch_project gazebo_launch.py
ros2 launch whitepost_description display.launch.py
Control Topics# Calibration & Homing
ros2 topic pub --once /joint1/calibrate std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /go_home std_msgs/msg/Bool "{data: true}"

# Positioning
ros2 topic pub --once /joint1/set_target_angle std_msgs/msg/Float32 "{data: 45.0}"
ros2 topic pub --once /target_position geometry_msgs/msg/Point "{x: 600.0, y: 400.0, z: 450.0}"

# End Effector & Tracking
ros2 topic pub /servo/set_angle std_msgs/msg/Float32MultiArray "{data: [0.0, 90.0]}" --once
ros2 topic pub --once /toggle_tracking std_msgs/msg/Bool "{data: true}"
🛠 4. System MaintenanceTroubleshooting and process management.Kill Stuck Processes# List background jobs
jobs -l

# Force kill all Python/ROS2 processes
sudo pkill -f python3
sudo pkill -f ros2
I2C & Sensors (Pi Zero)# Select I2C Channel 0 and scan
i2cset -y 1 0x70 0x01
i2cdetect -y 1

# Run CV script
source venv/bin/activate
python3 opencv_raspi-zero.py
⚠️ Screen Exit MethodIf stuck in 'screen /dev/ttyUSB0 115200':1. Press Ctrl + A2. Press K3. Press Y (to confirm)Maintained by: EARTH157