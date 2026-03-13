import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'whitepost_description'
    
    # ⚠️ แก้ชื่อไฟล์ .urdf บรรทัดล่างนี้ให้ตรงกับชื่อไฟล์ในโฟลเดอร์ urdf/ ของคุณนะครับ
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'whitepost_description.urdf') 
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # ตัวอ่านโครงสร้างหุ่นยนต์
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # ตัวสร้างหน้าต่าง Slider สำหรับเลื่อนข้อต่อ (Joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # เปิดโปรแกรม RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])