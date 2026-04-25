import launch
import launch_ros.actions
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([

        Node(package='motor_control', executable='joint1_node', output='screen', respawn=True, respawn_delay=2.0),
        Node(package='motor_control', executable='joint2_node', output='screen', respawn=True, respawn_delay=2.0),
        Node(package='motor_control', executable='joint3_node', output='screen', respawn=True, respawn_delay=2.0),

        Node(package='processor_unit', executable='esp32_bridge_node', output='screen'),
        Node(package='processor_unit', executable='force_receiver_node', output='screen'),

        Node(
            package='processor_unit', executable='main_processor', output='screen',
            parameters=[{'simulation_mode': False}]
        ),
        Node(
            package='processor_unit', executable='steam_cam', output='screen',
            parameters=[{'simulation_mode': False}]
        ),

        # ✅ Add simulation_mode to camera nodes
        Node(
            package='processor_unit', executable='camera_front', output='screen',
            parameters=[{'simulation_mode': False}]
        ),
        Node(
            package='processor_unit', executable='camera_back', output='screen',
            parameters=[{'simulation_mode': False}]
        ),

        # ✅ Auto-send calibrate command 5 seconds after launch
        # giving ESP32 and joints time to boot first
        TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    package='ros2topic',
                    executable='pub',
                    arguments=[
                        '--once',
                        '/robot_command',
                        'std_msgs/msg/String',
                        '{data: calibrate}'
                    ]
                )
            ]
        ),
    ])