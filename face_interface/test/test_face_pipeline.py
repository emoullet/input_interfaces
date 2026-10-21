# Copyright 2026 Etienne Moullet
# SPDX-License-Identifier: Apache-2.0

"""Exercise the installed face launch using synthetic Joy, without hardware."""

import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import yaml


class Pipeline:
    def __init__(self, node, executor, process, topic):
        self.node = node
        self.executor = executor
        self.process = process
        self.publisher = node.create_publisher(Joy, topic, 10)
        self.commands = []
        self.requests = []

    def wait(self, predicate, on_tick=None):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            assert self.process.poll() is None, 'Face launch exited early'
            if on_tick is not None:
                on_tick()
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Timed out waiting for face mapper output')

    def pump(self, seconds=0.1):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def message(self, axes, pressed=False, buttons=None):
        message = Joy()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.axes = [float(value) for value in axes]
        message.buttons = list(buttons) if buttons is not None else [0] * 12
        if pressed:
            message.buttons[11] = 1
        return message

    def send(self, axes, pressed=False, buttons=None):
        previous_count = len(self.commands)
        self.publisher.publish(self.message(axes, pressed, buttons))
        self.wait(lambda: len(self.commands) > previous_count)
        return self.commands[-1]


@pytest.fixture
def pipeline(tmp_path):
    # Private topics keep these tests away from any running robot stack.
    prefix = '/face_test_' + uuid.uuid4().hex
    share = Path(get_package_share_directory('face_interface'))
    document = yaml.safe_load((share / 'config' / 'explorer_face_params.yaml').read_text())
    params = document['joystick_mapper']['ros__parameters']
    params.update({
        'joy_topic': prefix + '/joy', 'output_topic': prefix + '/command',
        'mode_request_topic': prefix + '/mode_request',
        'gripper_command_topic': prefix + '/gripper',
    })
    config = tmp_path / 'face.yaml'
    config.write_text(yaml.safe_dump(document))
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('observer', namespace=prefix, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    process = None
    log = (tmp_path / 'face_launch.log').open('w+')
    try:
        process = subprocess.Popen([
            'ros2', 'launch', 'face_interface', 'face_interface.launch.py',
            'start_playability:=false', 'start_joy:=false', 'config_file:=' + str(config),
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        observer = Pipeline(node, executor, process, params['joy_topic'])
        subscriptions = [
            node.create_subscription(
                TwistStamped, params['output_topic'], observer.commands.append, 10),
            node.create_subscription(
                String, params['mode_request_topic'], observer.requests.append, 10),
        ]
        observer.wait(lambda: (
            observer.publisher.get_subscription_count() == 1
            and all(subscription.get_publisher_count() == 1 for subscription in subscriptions)
        ))
        # DDS graph discovery can precede reader/writer matching.
        observer.wait(lambda: bool(observer.commands), on_tick=lambda: (
            observer.publisher.publish(observer.message([0.0] * 4))))
        observer.pump()
        yield observer
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        log.seek(0)
        print(log.read())
        log.close()


def _assert_command(command, linear, angular):
    assert command.header.frame_id == 'base_link'
    assert command.header.stamp.sec > 0
    vector = command.twist.linear
    assert [vector.x, vector.y, vector.z] == pytest.approx(linear, abs=1e-6)
    vector = command.twist.angular
    assert [vector.x, vector.y, vector.z] == pytest.approx(angular, abs=1e-6)


def test_translation_mapping_dead_zone_saturation_and_missing_axes(pipeline):
    for axes, expected in (
        ([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ([0.049, -0.049, 0.0, 0.049], [0.0, 0.0, 0.0]),
        ([0.05, -0.05, 0.0, 0.05], [0.0, 0.0, 0.0]),
        ([0.525, -0.525, 1.0, 0.525], [-0.5, -0.5, 0.5]),
        ([-1.0, 1.0, 0.0, -1.0], [1.0, 1.0, -1.0]),
        ([-2.0, 2.0, 0.0, 2.0], [-1.0, 1.0, -1.0]),
        ([], [0.0, 0.0, 0.0]),
        ([1.0], [0.0, 0.0, 1.0]),
    ):
        _assert_command(pipeline.send(axes, buttons=[]), expected, [0.0] * 3)


def test_button_edges_switch_modes_without_geometric_requests(pipeline):
    axes = [0.525, -1.0, 1.0, -0.525]
    expected = [0.5, -1.0, 0.5]
    _assert_command(pipeline.send(axes), expected, [0.0] * 3)
    _assert_command(pipeline.send(axes, pressed=True), [0.0] * 3, expected)
    for _ in range(3):
        _assert_command(pipeline.send(axes, pressed=True), [0.0] * 3, expected)
    _assert_command(pipeline.send([0.0] * 4), [0.0] * 3, [0.0] * 3)
    _assert_command(pipeline.send(axes), [0.0] * 3, expected)
    _assert_command(pipeline.send(axes, pressed=True), expected, [0.0] * 3)
    # Other buttons are deliberately unassigned, including geometric controls.
    _assert_command(pipeline.send(axes, buttons=[1] * 11 + [0]), expected, [0.0] * 3)
    pipeline.pump()
    assert pipeline.requests == []
