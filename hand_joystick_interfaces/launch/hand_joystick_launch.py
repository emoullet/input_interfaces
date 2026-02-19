from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('hand_joystick_interfaces'),
            'config',
            'default_parameters.yaml'
        ]),
        description='Path to the configuration file (2D mode)'
    )

    # Hand joystick interface node
    hand_joystick_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_node',
        name='hand_joystick_interface',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        hand_joystick_node
    ])
