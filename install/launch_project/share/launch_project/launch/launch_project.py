import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='motor_control',
            executable='joint1_node',
            name='joint1_node',
        ),
        launch_ros.actions.Node(
            package='motor_control',
            executable='joint2_node',
            name='joint2_node',
        ),
        launch_ros.actions.Node(
            package='motor_control',
            executable='joint3_node',
            name='joint3_node',
        ),
        launch_ros.actions.Node(
            package='motor_control',
            executable='servodrive',
            name='servodrive',
        ),
        launch_ros.actions.Node(
            package='processor_unit',
            executable='main_processor',
            name='main_processor',
        ),
    ])