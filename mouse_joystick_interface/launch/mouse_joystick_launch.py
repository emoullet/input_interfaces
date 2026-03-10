from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        "host",
        default_value="127.0.0.1",
        description="HTTP bind address",
    )
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",
        description="HTTP bind port",
    )
    auto_open_browser_arg = DeclareLaunchArgument(
        "auto_open_browser",
        default_value="true",
        description="Open the joystick page in the default browser when node starts",
    )
    teleop_topic_arg = DeclareLaunchArgument(
        "teleop_topic",
        default_value="teleop_cmd",
        description="Teleop command topic name",
    )

    node = Node(
        package="mouse_joystick_interface",
        executable="mouse_joystick_server.py",
        name="mouse_joystick_interface",
        output="screen",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "auto_open_browser": LaunchConfiguration("auto_open_browser"),
                "teleop_topic": LaunchConfiguration("teleop_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            host_arg,
            port_arg,
            auto_open_browser_arg,
            teleop_topic_arg,
            node,
        ]
    )
