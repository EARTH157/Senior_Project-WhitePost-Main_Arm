# remote Raspberry Pi 5 in Arm Station

ssh raspi-earth@172.20.10.2

Password: spacebar and enter

# remote Raspberry Pi Zero 2W in End effector

ssh raspi-zero-two-w-earth@172.20.10.3

Password: spacebar and enter

# Kill all Process

jobs -l

kill -9 

# Run Motor

ros2 run motor_control stepmotor1

ros2 run motor_control servo_driver_i2c

ros2 run motor_control joint1_node

ros2 topic pub -1 /stepper_joint1_cmd std_msgs/Int32 "{data: 500}"

ros2 topic pub --once /joint1/calibrate std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /joint1/set_target_angle std_msgs/msg/Float32 "{data: 45.0}"

ros2 topic pub /servo/set_angle std_msgs/msg/Float32MultiArray "{data: [0.0, 90.0]}" --once

# เลือก channel 0

i2cset -y 1 0x70 0x01

i2cdetect -y 1

source venv/bin/activate

ros2 topic pub /servo/set_angle std_msgs/Float32MultiArray "data: [14, 45.0]" -1

python step1_rrtstar_drive_650610830.py --tracker stanley --rejoin-dist 0.25 --lookahead 0.22 --vmax 0.12 --path-mode mesh3d 
 --path-width-m 0.02   --path-thickness 0.002 --path-z 0.00005 --path-color 0,0,0 --face-now --face-mode tangent --show
