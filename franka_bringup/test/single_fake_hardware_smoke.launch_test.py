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
Offline smoke test for the single-arm GenericSystem launch path.

The launch inputs below are fixed test constants: fake hardware is always enabled, the robot
address is always a placeholder, and no gripper, RViz, or motion controller is started. Runtime
checks additionally require the loaded plugin to be ``mock_components/GenericSystem`` and expose
exactly the expected seven joints.
"""

from pathlib import Path
import sys
import time
import unittest

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


# franka.launch.py is loaded from the SOURCE tree below (it is deliberately not
# installed - see franka_bringup/CMakeLists.txt) via PythonLaunchDescriptionSource,
# which resolves it with importlib internally; that would otherwise write a
# launch/real/__pycache__/franka.launch.cpython-3xx.pyc next to the source file,
# and with --symlink-install that stray cache directory gets swept into the
# installed share/ tree, reintroducing the excluded file as a loadable .pyc. Must
# be set before the module is loaded (below), regardless of process.
sys.dont_write_bytecode = True

_SOURCE_ROOT = Path(__file__).resolve().parents[2]


# Do not parameterize these values. This test must have no input that can select real hardware or
# provide a robot address.
_FAKE_LAUNCH_ARGUMENTS = {
    'robot_ip': 'dont-care',
    'arm_id': 'panda',
    'use_fake_hardware': 'true',
    'fake_sensor_commands': 'false',
    'load_gripper': 'false',
    'use_rviz': 'false',
}
_EXPECTED_JOINTS = {'panda_joint{}'.format(index) for index in range(1, 8)}
_EXPECTED_INTERFACE_KINDS = {'position', 'velocity', 'effort'}
_SERVICE_WAIT_TIMEOUT_SEC = 30.0
_SERVICE_CALL_TIMEOUT_SEC = 10.0
_JOINT_STATE_TIMEOUT_SEC = 20.0


@pytest.mark.launch_test
def generate_test_description():
    # source-tree path because franka.launch.py is deliberately not installed during the MVP
    # (excluded real-IP surface; see franka_bringup/CMakeLists.txt).
    franka_launch_file = str(
        _SOURCE_ROOT / 'franka_bringup' / 'launch' / 'real' / 'franka.launch.py')
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(franka_launch_file),
        launch_arguments=_FAKE_LAUNCH_ARGUMENTS.items(),
    )
    return launch.LaunchDescription([
        franka_launch,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestSingleFakeHardwareSmoke(unittest.TestCase):
    """Inspect the running single-arm stack before launch_testing tears it down."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('single_fake_hardware_smoke_test_client')

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
            '{} did not respond within {}s'.format(client.srv_name, timeout_sec),
        )
        result = future.result()
        self.assertIsNotNone(
            result,
            '{} call raised: {}'.format(client.srv_name, future.exception()),
        )
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
            node_names,
            ['controller_manager'],
            'expected exactly one controller_manager node',
        )

    def test_spawner_exits_cleanly(self, proc_info):
        proc_info.assertWaitForShutdown(process='spawner', timeout=10)
        launch_testing.asserts.assertExitCodes(proc_info, process='spawner')

    def test_hardware_component_is_generic_system(self):
        client = self.node.create_client(
            ListHardwareComponents,
            '/controller_manager/list_hardware_components',
        )
        self.assertTrue(self._wait_for_service(client, _SERVICE_WAIT_TIMEOUT_SEC))

        response = self._call(client, ListHardwareComponents.Request())
        self.assertEqual(len(response.component), 1)
        component = response.component[0]
        self.assertEqual(
            component.plugin_name,
            'mock_components/GenericSystem',
            'the offline smoke test must never load the real Franka hardware plugin',
        )
        self.assertEqual(component.state.label, 'active')

    def test_exactly_seven_joint_interfaces_are_unclaimed(self):
        client = self.node.create_client(
            ListHardwareInterfaces,
            '/controller_manager/list_hardware_interfaces',
        )
        self.assertTrue(self._wait_for_service(client, _SERVICE_WAIT_TIMEOUT_SEC))

        response = self._call(client, ListHardwareInterfaces.Request())
        state_kinds_by_joint = {}
        for interface in response.state_interfaces:
            joint, separator, kind = interface.name.partition('/')
            self.assertEqual(separator, '/')
            state_kinds_by_joint.setdefault(joint, set()).add(kind)

        command_kinds_by_joint = {}
        for interface in response.command_interfaces:
            joint, separator, kind = interface.name.partition('/')
            self.assertEqual(separator, '/')
            command_kinds_by_joint.setdefault(joint, set()).add(kind)
            self.assertTrue(interface.is_available)
            self.assertFalse(interface.is_claimed)

        self.assertEqual(set(state_kinds_by_joint), _EXPECTED_JOINTS)
        self.assertEqual(set(command_kinds_by_joint), _EXPECTED_JOINTS)
        for joint in _EXPECTED_JOINTS:
            self.assertEqual(state_kinds_by_joint[joint], _EXPECTED_INTERFACE_KINDS)
            self.assertEqual(command_kinds_by_joint[joint], _EXPECTED_INTERFACE_KINDS)

    def test_no_motion_controller_is_loaded(self):
        client = self.node.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        self.assertTrue(self._wait_for_service(client, _SERVICE_WAIT_TIMEOUT_SEC))

        response = self._call(client, ListControllers.Request())
        controllers_by_name = {controller.name: controller for controller in response.controller}
        self.assertEqual(set(controllers_by_name), {'joint_state_broadcaster'})
        self.assertEqual(controllers_by_name['joint_state_broadcaster'].state, 'active')

    def test_expected_joint_states_publish_without_duplicates(self):
        received = {}

        def _on_joint_state(message):
            received['message'] = message

        subscription = self.node.create_subscription(
            JointState,
            '/franka/joint_states',
            _on_joint_state,
            10,
        )
        try:
            deadline = time.monotonic() + _JOINT_STATE_TIMEOUT_SEC
            while 'message' not in received and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.5)
        finally:
            self.node.destroy_subscription(subscription)

        self.assertIn('message', received)
        joint_names = list(received['message'].name)
        self.assertEqual(len(joint_names), 7)
        self.assertEqual(len(joint_names), len(set(joint_names)))
        self.assertEqual(set(joint_names), _EXPECTED_JOINTS)


@launch_testing.post_shutdown_test()
class TestSingleFakeHardwareShutdown(unittest.TestCase):
    """Verify that the long-running managed processes exit cleanly."""

    def test_managed_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process='ros2_control_node')
        launch_testing.asserts.assertExitCodes(proc_info, process='robot_state_publisher')
        # joint_state_publisher is deliberately excluded: Jazzy can terminate it during
        # launch shutdown before its rclpy context has completed initialization.
