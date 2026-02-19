from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    hand_joystick_share_dir = get_package_share_directory('hand_joystick_interfaces')
    mediapipe_share_dir = get_package_share_directory('mediapipe_mocap')
    
    # Config files
    hand_joystick_config = os.path.join(hand_joystick_share_dir, 'config', '3d_parameters.yaml')
    hand_landmarks_config = os.path.join(mediapipe_share_dir, 'config', 'hand_landmarks_node_depth_inference.yaml')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=hand_joystick_config,
        description='Path to hand joystick 3D configuration file'
    )

    # Hand landmarks node with depth inference (MediaPipe)
    hand_landmarks_node = Node(
        package='mediapipe_mocap',
        executable='hand_landmarks_node_depth_inference',
        name='hand_landmarks_depth_inference_node',
        output='screen',
        parameters=[hand_landmarks_config],
        remappings=[
            ('hand_landmarks_depth', '/hand_landmarks')
        ]
    )

    # Hand joystick interface node
    hand_joystick_3d_node = Node(
        package='hand_joystick_interfaces',
        executable='hand_joystick_node',
        name='hand_joystick_interface',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        hand_landmarks_node,
        hand_joystick_3d_node
    ])
