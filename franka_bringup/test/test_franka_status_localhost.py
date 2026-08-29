# Copyright 2026 The multipanda_ros2 Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercises the read-only status collector over localhost ROS services."""

import threading

from controller_manager_msgs.msg import HardwareComponentState
from controller_manager_msgs.msg import HardwareInterface
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareComponents
from controller_manager_msgs.srv import ListHardwareInterfaces
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from franka_bringup import status
from lifecycle_msgs.msg import State
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


def _interface(name):
    return HardwareInterface(
        name=name, data_type='double', is_available=True, is_claimed=False)


def _diagnostic(arm_id):
    values = []
    for key in sorted(status.DIAGNOSTIC_KEYS):
        value = arm_id if key == 'arm_id' else '0'
        if key == 'hardware_lifecycle_id':
            value = str(State.PRIMARY_STATE_ACTIVE)
        elif key == 'hardware_lifecycle_label':
            value = 'active'
        values.append(KeyValue(key=key, value=value))
    return DiagnosticStatus(
        level=DiagnosticStatus.OK,
        name='franka_hardware_diagnostics: franka_hardware/{}'.format(arm_id),
        message='healthy',
        hardware_id=arm_id,
        values=values,
    )


class _ReadOnlyServer:
    def __init__(self, arm_ids=status.ARM_IDS):
        self.arm_ids = tuple(arm_ids)
        self.command_names, self.state_names = status.expected_hardware_interfaces_for(
            self.arm_ids)
        self.node = rclpy.create_node('franka_status_read_only_test_server')
        self.calls = []
        self.node.create_service(
            ListControllers, '/controller_manager/list_controllers', self._controllers)
        self.node.create_service(
            ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces',
            self._interfaces)
        self.node.create_service(
            ListHardwareComponents, '/controller_manager/list_hardware_components',
            self._components)
        self.publisher = self.node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.node.create_timer(0.02, self._publish)

    def _controllers(self, _request, response):
        self.calls.append('list_controllers')
        return response

    def _interfaces(self, _request, response):
        self.calls.append('list_hardware_interfaces')
        response.command_interfaces = [
            _interface(name) for name in sorted(self.command_names)]
        response.state_interfaces = [
            _interface(name) for name in sorted(self.state_names)]
        return response

    def _components(self, _request, response):
        self.calls.append('list_hardware_components')
        lifecycle = State(id=State.PRIMARY_STATE_ACTIVE, label='active')
        response.component = [HardwareComponentState(
            name=status.FRANKA_COMPONENT_NAME,
            type='system',
            class_type=status.FRANKA_COMPONENT_PLUGIN,
            plugin_name=status.FRANKA_COMPONENT_PLUGIN,
            state=lifecycle,
            command_interfaces=[
                _interface(name) for name in sorted(self.command_names)],
            state_interfaces=[
                _interface(name) for name in sorted(self.state_names)],
        )]
        return response

    def _publish(self):
        self.publisher.publish(DiagnosticArray(
            status=[_diagnostic(arm_id) for arm_id in self.arm_ids]))


def _collect_over_localhost(server_arm_ids, client_arm_mode):
    """Run one real collection over localhost DDS and return (result, error, server)."""
    rclpy.init()
    server = _ReadOnlyServer(server_arm_ids)
    client = status.FrankaStatusNode(client_arm_mode)
    executor = SingleThreadedExecutor()
    executor.add_node(server.node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    result = None
    error = None
    try:
        try:
            result = client.collect(5.0)
        except status.StatusError as caught:
            error = caught
    finally:
        client.destroy_node()
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        server.node.destroy_node()
        rclpy.shutdown()
    assert not thread.is_alive()
    return result, error, server


def test_status_collects_only_read_only_services_and_fresh_diagnostics():
    result, error, server = _collect_over_localhost(status.ARM_IDS, 'dual')
    assert error is None
    assert result['ok'] is True
    assert result['hardware']['command_interface_count'] == 86
    assert result['hardware']['state_interface_count'] == 110
    assert 'arm_mode' not in result
    assert sorted(server.calls) == [
        'list_controllers', 'list_hardware_components', 'list_hardware_interfaces']


@pytest.mark.parametrize('arm_id', status.ONE_ARM_MODE_IDS)
def test_one_arm_status_auto_detects_the_live_arm_over_localhost(arm_id):
    result, error, server = _collect_over_localhost((arm_id,), 'single')
    assert error is None
    assert result['ok'] is True
    assert result['arm_id'] == arm_id
    assert result['arm_mode'] == 'single'
    assert [entry['arm_id'] for entry in result['diagnostics']] == [arm_id]
    assert result['hardware']['command_interface_count'] == 43
    assert result['hardware']['state_interface_count'] == 55
    assert sorted(server.calls) == [
        'list_controllers', 'list_hardware_components', 'list_hardware_interfaces']


def test_one_arm_status_refuses_a_live_two_arm_stack_as_ambiguous():
    result, error, _server = _collect_over_localhost(status.ARM_IDS, 'single')
    assert result is None
    assert isinstance(error, status.StatusError)
    assert 'ambiguous' in str(error)


def test_dual_status_refuses_a_live_one_arm_stack():
    result, error, _server = _collect_over_localhost(('panda1',), 'dual')
    assert result is None
    assert isinstance(error, status.StatusError)
