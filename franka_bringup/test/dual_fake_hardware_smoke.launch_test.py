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
"""
Dual fake-hardware launch smoke test.

Brings up franka_bringup/launch/real/dual_franka.launch.py with mock hardware only and asserts
the dual-arm bringup safety criteria:

  * exactly one controller_manager node/hardware component starts,
  * exactly 14 panda1_/panda2_ joints are exported,
  * every joint has position, velocity and effort state interfaces,
  * every joint has position, velocity and effort command interfaces, all unclaimed,
  * joint_state_broadcaster is the only active controller and joint states publish without
    duplicate names,
  * no motion controller (dual_joint_impedance_example_controller /
    dual_joint_velocity_example_controller) is ever loaded or active.

Safety: this test hardcodes ``use_fake_hardware:=true`` and non-routable placeholder robot IPs.
It never passes a real robot IP and never enables rviz. With ``use_fake_hardware`` true, the real
``franka_hardware/FrankaMultiHardwareInterface`` plugin (the only code path that touches
libfranka/FCI) is never selected by
``franka_description/robots/real/dual_panda_arm.ros2_control.xacro`` - only
``mock_components/GenericSystem`` is loaded, which never opens a network connection. This test
additionally asserts the loaded plugin name at runtime as a belt-and-suspenders check.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    ListHardwareInterfaces,
)
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState

# Fixed fake-hardware launch inputs.
# Do not turn these into pytest/launch parameters - this test must never be able to select the
# real hardware plugin or contact a robot.
_FAKE_LAUNCH_ARGUMENTS = {
    'robot_ip_1': 'dont-care',
    'robot_ip_2': 'dont-care',
    'arm_id_1': 'panda1',
    'arm_id_2': 'panda2',
    'use_fake_hardware': 'true',
    'load_gripper_1': 'false',
    'load_gripper_2': 'false',
    'use_rviz': 'false',
}

_EXPECTED_JOINTS = {
    '{}_joint{}'.format(arm_id, i) for arm_id in ('panda1', 'panda2') for i in range(1, 8)
}
_EXPECTED_INTERFACE_KINDS = {'position', 'velocity', 'effort'}
_MOTION_CONTROLLER_NAMES = {
    'dual_joint_impedance_example_controller',
    'dual_joint_velocity_example_controller',
}

# Bounded so a stuck controller_manager cannot hang CI: overridden per-service-call below, this
# is the outer ceiling for the whole pre-shutdown test class.
_SERVICE_WAIT_TIMEOUT_SEC = 30.0
_SERVICE_CALL_TIMEOUT_SEC = 10.0
_JOINT_STATE_TIMEOUT_SEC = 20.0


@pytest.mark.launch_test
def generate_test_description():
    dual_franka_launch_file = os.path.join(
        get_package_share_directory('franka_bringup'), 'launch', 'real', 'dual_franka.launch.py')

    dual_franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_franka_launch_file),
        launch_arguments=_FAKE_LAUNCH_ARGUMENTS.items(),
    )

    return launch.LaunchDescription([
        dual_franka_launch,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestDualFakeHardwareSmoke(unittest.TestCase):
    """Runs concurrently with the launched stack; launch_testing tears it down afterwards."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('dual_fake_hardware_smoke_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_service(self, client, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                return True
        return False

    def _call(self, client, request, timeout_sec=_SERVICE_CALL_TIMEOUT_SEC):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(
            future.done(),
            '{} did not respond within {}s'.format(client.srv_name, timeout_sec))
        result = future.result()
        self.assertIsNotNone(result, '{} call raised: {}'.format(
            client.srv_name, future.exception()))
        return result

    def test_single_controller_manager_node(self):
        deadline = time.monotonic() + _SERVICE_WAIT_TIMEOUT_SEC
        node_names = []
        while time.monotonic() < deadline:
            node_names = [
                name for name, _ in self.node.get_node_names_and_namespaces()
                if name == 'controller_manager'
            ]
            if node_names:
                break
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.assertEqual(
            len(node_names), 1,
            'expected exactly one controller_manager node, found {}'.format(len(node_names)))

    def test_spawner_exits_cleanly(self, proc_info):
        proc_info.assertWaitForShutdown(process='spawner', timeout=10)

    def test_hardware_component_is_mock_and_active(self):
        client = self.node.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.assertTrue(
            self._wait_for_service(client, _SERVICE_WAIT_TIMEOUT_SEC),
            '/controller_manager/list_hardware_components did not become available')

        response = self._call(client, ListHardwareComponents.Request())
        self.assertEqual(
            len(response.component), 1,
            'expected exactly one hardware component, got {}'.format(
                [c.name for c in response.component]))

        component = response.component[0]
        # This is the load-bearing safety assertion: only the mock plugin may ever be loaded by
        # this test. If this ever reads 'franka_hardware/FrankaMultiHardwareInterface' the test
        # setup is broken and must be treated as a hard failure, not a flake.
        self.assertEqual(
            component.plugin_name, 'mock_components/GenericSystem',
            'real hardware plugin was loaded during a fake-hardware smoke test')
        self.assertEqual(component.state.label, 'active')

    def test_joint_interfaces(self):
        client = self.node.create_client(
            ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces')
        self.assertTrue(
            self._wait_for_service(client, _SERVICE_WAIT_TIMEOUT_SEC),
            '/controller_manager/list_hardware_interfaces did not become available')

        response = self._call(client, ListHardwareInterfaces.Request())

        state_kinds_by_joint = {}
        for iface in response.state_interfaces:
            joint, _, kind = iface.name.partition('/')
            state_kinds_by_joint.setdefault(joint, set()).add(kind)

        self.assertEqual(
            set(state_kinds_by_joint), _EXPECTED_JOINTS,
            'unexpected joint set in state interfaces: {}'.format(
                sorted(state_kinds_by_joint)))
        for joint, kinds in state_kinds_by_joint.items():
            self.assertEqual(
                kinds, _EXPECTED_INTERFACE_KINDS,
                '{} is missing state interface kinds: {}'.format(
                    joint, _EXPECTED_INTERFACE_KINDS - kinds))

        command_kinds_by_joint = {}
        for iface in response.command_interfaces:
            joint, _, kind = iface.name.partition('/')
            command_kinds_by_joint.setdefault(joint, set()).add(kind)
            self.assertTrue(iface.is_available, '{} command interface not available'.format(
                iface.name))
            self.assertFalse(
                iface.is_claimed,
                '{} command interface is claimed - a controller is active early'.format(
                    iface.name))

        self.assertEqual(
            set(command_kinds_by_joint), _EXPECTED_JOINTS,
            'unexpected joint set in command interfaces: {}'.format(
                sorted(command_kinds_by_joint)))
        for joint, kinds in command_kinds_by_joint.items():
            self.assertEqual(
                kinds, _EXPECTED_INTERFACE_KINDS,
                '{} is missing command interface kinds: {}'.format(
                    joint, _EXPECTED_INTERFACE_KINDS - kinds))

    def test_no_motion_controller_active(self):
        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(
            self._wait_for_service(client, _SERVICE_WAIT_TIMEOUT_SEC),
            '/controller_manager/list_controllers did not become available')

        response = self._call(client, ListControllers.Request())
        controllers_by_name = {c.name: c for c in response.controller}

        for motion_controller_name in _MOTION_CONTROLLER_NAMES:
            self.assertNotIn(
                motion_controller_name, controllers_by_name,
                '{} must never be loaded by the fake-hardware smoke test'.format(
                    motion_controller_name))

        self.assertIn('joint_state_broadcaster', controllers_by_name)
        self.assertEqual(controllers_by_name['joint_state_broadcaster'].state, 'active')

        for controller in response.controller:
            if controller.name == 'joint_state_broadcaster':
                continue
            self.assertNotEqual(
                controller.state, 'active',
                'unexpected active controller {} - only joint_state_broadcaster may be '
                'active in this smoke test'.format(controller.name))

    def test_joint_states_publish_without_duplicates(self):
        received = {}

        def _on_joint_state(msg):
            received['msg'] = msg

        subscription = self.node.create_subscription(
            JointState, '/franka/joint_states', _on_joint_state, 10)
        try:
            deadline = time.monotonic() + _JOINT_STATE_TIMEOUT_SEC
            while 'msg' not in received and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.5)
        finally:
            self.node.destroy_subscription(subscription)

        self.assertIn(
            'msg', received,
            'no message received on /franka/joint_states within {}s'.format(
                _JOINT_STATE_TIMEOUT_SEC))
        names = list(received['msg'].name)
        self.assertEqual(
            len(names), len(_EXPECTED_JOINTS),
            'expected {} joints on /franka/joint_states, got {}: {}'.format(
                len(_EXPECTED_JOINTS), len(names), names))
        self.assertEqual(
            len(names), len(set(names)),
            'duplicate joint names on /franka/joint_states: {}'.format(names))
        self.assertEqual(set(names), _EXPECTED_JOINTS)


@launch_testing.post_shutdown_test()
class TestDualFakeHardwareShutdown(unittest.TestCase):
    """Verifies that the controller manager completes its hardware shutdown path."""

    def test_controller_manager_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
