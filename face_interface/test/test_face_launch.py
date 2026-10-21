# Copyright 2026 Etienne Moullet
# SPDX-License-Identifier: Apache-2.0

"""Check installed face launch wiring without starting any processes."""

import importlib.util
from itertools import product
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


def _share():
    return Path(get_package_share_directory('face_interface'))


def _resolve(overrides=None):
    spec = importlib.util.spec_from_file_location(
        'face_launch', _share() / 'launch' / 'face_interface.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    actions = []
    for entity in description.entities:
        if isinstance(entity, OpaqueFunction):
            actions.extend(entity.execute(context) or [])
    return context, actions


def _nodes(actions):
    result = []
    for action in actions:
        if isinstance(action, Node):
            result.append(action)
        elif isinstance(action, TimerAction):
            result.extend(_nodes(action.actions))
    return result


def _parameter_files(node, context):
    # Jazzy has no public pre-execution accessor for normalized parameters.
    return list(evaluate_parameters(context, vars(node)['_Node__parameters']))


def _remappings(node, context):
    return [
        (perform_substitutions(context, source), perform_substitutions(context, target))
        for source, target in vars(node)['_Node__remappings']
    ]


def test_default_launch_uses_installed_assets_and_one_dead_zone():
    context, actions = _resolve()
    nodes = {node.node_executable: node for node in _nodes(actions)}
    assert set(nodes) == {'game_controller_node', 'joystick_mapper_node'}
    config_file = _share() / 'config' / 'explorer_face_params.yaml'
    assert (_share() / 'README.md').is_file()
    parameters = yaml.safe_load(config_file.read_text())
    driver = parameters['joy_playability']['ros__parameters']
    mapper = parameters['joystick_mapper']['ros__parameters']
    assert driver == {
        'device_name': 'Xbox 360 Controller', 'deadzone': 0.0, 'autorepeat_rate': 50.0,
    }
    assert mapper['joy_topic'] == '/joy/playability'
    assert mapper['output_topic'] == '/joystick_cartesian_command'
    assert mapper['deadzone'] == 0.05
    assert mapper['modes']['names'] == ['translation', 'rotation']
    assert mapper['local_mode_button_index'] == 11
    assert mapper['local_mode_button_mode'] == 'toggle'
    for button in ('jaco', 'snake', 'home', 'gripper'):
        assert mapper[button + '_button_index'] == -1
    assert _remappings(nodes['game_controller_node'], context) == [
        ('joy', mapper['joy_topic'])]
    for node in nodes.values():
        assert _parameter_files(node, context) == [config_file]
        assert Path(ExecutableInPackage(
            package=node.node_package, executable=node.node_executable).perform(context)).is_file()

    processes = [action for action in actions
                 if isinstance(action, ExecuteProcess) and not isinstance(action, Node)]
    assert len(processes) == 1
    assert [perform_substitutions(context, part) for part in processes[0].cmd] == [
        'flatpak', 'run', 'com.playability.playAbility']


@pytest.mark.parametrize('playability,joy,mapper', list(product([False, True], repeat=3)))
def test_startup_options_and_driver_delay(playability, joy, mapper):
    context, actions = _resolve({
        'start_playability': str(playability).lower(),
        'start_joy': str(joy).lower(),
        'start_mapper': str(mapper).lower(),
    })
    executables = [node.node_executable for node in _nodes(actions)]
    assert executables.count('game_controller_node') == int(joy)
    assert executables.count('joystick_mapper_node') == int(mapper)
    processes = [action for action in actions
                 if isinstance(action, ExecuteProcess) and not isinstance(action, Node)]
    assert len(processes) == int(playability)
    timers = [action for action in actions if isinstance(action, TimerAction)]
    assert len(timers) == int(playability and joy)
    if timers:
        assert timers[0].period == 3.0
        assert [node.node_executable for node in _nodes(timers[0].actions)] == [
            'game_controller_node']


def test_custom_configuration_connects_driver_and_mapper(tmp_path):
    config = yaml.safe_load((_share() / 'config' / 'explorer_face_params.yaml').read_text())
    config['joy_playability']['ros__parameters']['device_name'] = 'Custom Controller'
    config['joystick_mapper']['ros__parameters']['joy_topic'] = '/custom/face_joy'
    config['joystick_mapper']['ros__parameters']['output_topic'] = '/custom/command'
    path = tmp_path / 'face.yaml'
    path.write_text(yaml.safe_dump(config))
    context, actions = _resolve({'config_file': str(path), 'start_playability': 'false'})
    nodes = _nodes(actions)
    assert len(nodes) == 2
    for node in nodes:
        assert _parameter_files(node, context) == [path]
    driver = next(node for node in nodes if node.node_executable == 'game_controller_node')
    assert _remappings(driver, context) == [('joy', '/custom/face_joy')]


@pytest.mark.parametrize('topic', [None, '', 12])
def test_invalid_driver_topic_has_a_clear_error(tmp_path, topic):
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump({
        'joystick_mapper': {'ros__parameters': {'joy_topic': topic}},
    }))
    with pytest.raises(ValueError, match='joy_topic'):
        _resolve({'config_file': str(path)})
