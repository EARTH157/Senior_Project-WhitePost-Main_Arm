import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    display_launch_path = os.path.join(
        get_package_share_directory('whitepost_description'),
        'launch',
        'display.launch.py'
    )
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Set to "true" for Webcam, "false" for Socket'
    )
    
    sim_config = LaunchConfiguration('sim')

    return LaunchDescription([
        sim_mode_arg, # ใส่ Argument เข้าไปในระบบ

        # 1. Node สมองกล (Main Processor)
        Node(
            package='processor_unit',
            executable='main_processor',
            name='main_processor',
            parameters=[{'simulation_mode': sim_config}], # 👈 รับค่า sim
            output='screen'
        ),

        # 2. Node ระบบกล้อง (Steam Cam)
        Node(
            package='processor_unit',
            executable='steam_cam',
            name='socket_tracker_node',
            parameters=[{'simulation_mode': sim_config}], # 👈 รับค่า sim เหมือนกัน
            output='screen'
        ),

        # 2. หน้าต่างสไลเดอร์ GUI (โหมด "เต้นตาม" สมองกล)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'source_list': ['/joint_states']}] # 👈 เติมบรรทัดนี้ครับ!
        ),

        # 3. RViz และ Robot State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_path)
        )
    ])