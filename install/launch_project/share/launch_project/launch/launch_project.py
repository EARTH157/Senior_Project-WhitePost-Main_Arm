import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        
        # --- กลุ่มที่ 1: การควบคุมมอเตอร์ฮาร์ดแวร์ (Motor Control) ---
        Node(
            package='motor_control',
            executable='joint1_node',
            name='joint1_node',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='joint2_node',
            name='joint2_node',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='joint3_node',
            name='joint3_node',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='servodrive',
            name='servodrive',
            output='screen'
        ),

        # --- กลุ่มที่ 2: สมองกลและเซ็นเซอร์ (Processor Unit) ---
        Node(
            package='processor_unit',
            executable='esp32_bridge_node',
            name='esp32_bridge_node',
            output='screen'
        ),
        Node(
            package='processor_unit',
            executable='main_processor',
            name='main_processor',
            parameters=[{'simulation_mode': False}], # 🟢 บังคับใช้ฮาร์ดแวร์จริง
            output='screen'
        ),
        Node(
            package='processor_unit',
            executable='steam_cam',
            name='steam_cam',
            parameters=[{'simulation_mode': False}], # 🟢 บังคับใช้กล้องจริง/Socket
            output='screen'
        ),
        
        # --- กลุ่มที่ 3: ระบบกล้องเช็คประตูลิฟต์ ---
        Node(
            package='processor_unit',
            executable='camera_front',
            name='camera_front',
            output='screen'
        ),
        # 🟢 เพิ่มกล้องหลังเข้าไปด้วยครับ
        Node(
            package='processor_unit',
            executable='camera_back',
            name='camera_back',
            output='screen'
        ),
    ])