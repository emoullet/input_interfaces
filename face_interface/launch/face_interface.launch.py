# Copyright 2026 Etienne Moullet
# SPDX-License-Identifier: Apache-2.0

"""Launch PlayAbility, its virtual game controller, and the Cartesian mapper."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def _launch_nodes(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    start_playability = IfCondition(
        LaunchConfiguration('start_playability')).evaluate(context)
    start_joy = IfCondition(LaunchConfiguration('start_joy')).evaluate(context)
    start_mapper = IfCondition(LaunchConfiguration('start_mapper')).evaluate(context)
    actions = []

    if start_playability:
        actions.append(ExecuteProcess(
            cmd=['playability'],
            output='screen',
        ))

    if start_joy:
        with Path(config_file).open(encoding='utf-8') as stream:
            parameters = yaml.safe_load(stream)
        try:
            joy_topic = parameters['joystick_mapper']['ros__parameters']['joy_topic']
        except (KeyError, TypeError) as error:
            raise ValueError(
                f'{config_file}: expected joystick_mapper.ros__parameters.joy_topic'
            ) from error
        if not isinstance(joy_topic, str) or not joy_topic.strip():
            raise ValueError(f'{config_file}: joy_topic must be a non-empty string')

        driver = Node(
            package='joy', executable='game_controller_node',
            name='joy_playability', output='screen',
            parameters=[config_file], remappings=[('joy', joy_topic)],
        )
        actions.append(
            TimerAction(period=3.0, actions=[driver]) if start_playability else driver)

    if start_mapper:
        actions.append(Node(
            package='joystick_mapper', executable='joystick_mapper_node',
            name='joystick_mapper', output='screen', parameters=[config_file],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('face_interface'), 'config', 'explorer_face_params.yaml',
            ]),
            description='YAML parameters for joy_playability and joystick_mapper.',
        ),
        DeclareLaunchArgument(
            'start_playability', default_value='true',
            description='Start the Debian-installed PlayAbility executable.',
        ),
        DeclareLaunchArgument(
            'start_joy', default_value='true',
            description='Start the PlayAbility virtual game-controller driver.',
        ),
        DeclareLaunchArgument(
            'start_mapper', default_value='true',
            description='Start the mapper; disable when robot bringup already owns it.',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
