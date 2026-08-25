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
Exercise the selected safe dual-arm controllers through Jazzy controller_manager.

The real dual launch is fixed to GenericSystem and placeholder addresses. The hold and bounded
impedance controllers can be loaded and configured here, but they must not activate: GenericSystem
deliberately does not export the production-only robot_state/robot_model pointer interfaces.
Treating scalar mock values as those pointers would be invalid. Positive activation is therefore
covered by production-interface synthetic C++ fixtures, not fabricated in this launch test.
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    ListHardwareComponents,
    ListHardwareInterfaces,
    LoadController,
    SwitchController,
    UnloadController,
)
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


_FAKE_LAUNCH_ARGUMENTS = {
    'robot_ip_1': 'non-routable-placeholder-1',
    'robot_ip_2': 'non-routable-placeholder-2',
    'arm_id_1': 'panda1',
    'arm_id_2': 'panda2',
    'use_fake_hardware': 'true',
    'fake_sensor_commands': 'false',
    'load_gripper_1': 'false',
    'load_gripper_2': 'false',
    'use_rviz': 'false',
}
_HOLD_CONTROLLER = 'dual_arm_joint_hold_controller'
_IMPEDANCE_CONTROLLER = 'dual_arm_joint_impedance_controller'
_VELOCITY_CONTROLLER = 'dual_arm_joint_velocity_controller'
_EXPECTED_JOINTS = [
    '{}_joint{}'.format(arm_id, joint)
    for arm_id in ('panda1', 'panda2')
    for joint in range(1, 8)
]
_ARM_JOINTS = {
    arm_id: ['{}_joint{}'.format(arm_id, joint) for joint in range(1, 8)]
    for arm_id in ('panda1', 'panda2')
}
_SERVICE_WAIT_TIMEOUT_SEC = 30.0
_SERVICE_CALL_TIMEOUT_SEC = 15.0
_OBSERVATION_TIMEOUT_SEC = 10.0


@pytest.mark.launch_test
def generate_test_description():
    dual_launch = os.path.join(
        get_package_share_directory('franka_bringup'),
        'launch',
        'real',
        'dual_franka.launch.py',
    )
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_launch),
            launch_arguments=_FAKE_LAUNCH_ARGUMENTS.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestDualSafeControllersFakeHardware(unittest.TestCase):
    """Drive real controller-manager lifecycle services against GenericSystem only."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('dual_safe_controllers_fake_hardware_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        # ReadyToTest can run before the launch-owned spawner has finished. Every case starts only
        # after the state-only default controller is active, so it cannot observe a transient empty
        # controller list or miss the joint-state publisher.
        self._wait_for_controller_state('joint_state_broadcaster', 'active')

    def _client(self, service_type, service_name):
        client = self.node.create_client(service_type, service_name)
        deadline = time.monotonic() + _SERVICE_WAIT_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.5):
                return client
        self.fail('{} did not become available'.format(service_name))

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=_SERVICE_CALL_TIMEOUT_SEC,
        )
        self.assertTrue(future.done(), '{} timed out'.format(client.srv_name))
        self.assertIsNone(future.exception(), '{} raised {}'.format(
            client.srv_name,
            future.exception(),
        ))
        self.assertIsNotNone(future.result())
        return future.result()

    def _controller_states(self):
        client = self._client(ListControllers, '/controller_manager/list_controllers')
        response = self._call(client, ListControllers.Request())
        return {controller.name: controller.state for controller in response.controller}

    def _wait_for_controller_state(self, controller_name, expected_state):
        deadline = time.monotonic() + _SERVICE_WAIT_TIMEOUT_SEC
        observed = None
        while time.monotonic() < deadline:
            observed = self._controller_states().get(controller_name)
            if observed == expected_state:
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail('{} did not reach {}; last state was {}'.format(
            controller_name,
            expected_state,
            observed,
        ))

    def _load(self, controller_name):
        client = self._client(LoadController, '/controller_manager/load_controller')
        request = LoadController.Request()
        request.name = controller_name
        self.assertTrue(self._call(client, request).ok)
        self._wait_for_controller_state(controller_name, 'unconfigured')

    def _configure(self, controller_name):
        client = self._client(
            ConfigureController,
            '/controller_manager/configure_controller',
        )
        request = ConfigureController.Request()
        request.name = controller_name
        return self._call(client, request).ok

    def _switch(self, activate=(), deactivate=(), expect_success=True):
        client = self._client(SwitchController, '/controller_manager/switch_controller')
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 10
        response = self._call(client, request)
        self.assertEqual(response.ok, expect_success, response.message)
        return response

    def _unload(self, controller_name):
        states = self._controller_states()
        if states.get(controller_name) == 'active':
            self._switch(deactivate=(controller_name,))
        if controller_name in states:
            client = self._client(UnloadController, '/controller_manager/unload_controller')
            request = UnloadController.Request()
            request.name = controller_name
            self.assertTrue(self._call(client, request).ok)

    def _set_parameters(self, controller_name, values):
        client = AsyncParameterClient(self.node, '/' + controller_name)
        self.assertTrue(
            client.wait_for_services(timeout_sec=_SERVICE_WAIT_TIMEOUT_SEC),
            '{} parameter services did not become available'.format(controller_name),
        )
        future = client.set_parameters([
            Parameter(name, value=value) for name, value in values.items()
        ])
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=_SERVICE_CALL_TIMEOUT_SEC,
        )
        self.assertTrue(future.done())
        self.assertIsNone(future.exception())
        self.assertTrue(
            all(result.successful for result in future.result().results),
            [result.reason for result in future.result().results],
        )

    def _valid_velocity_parameters(self):
        values = {
            'watchdog_timeout': 0.5,
            'max_header_age': 0.5,
            'future_tolerance': 0.1,
        }
        for arm_index, arm_id in enumerate(('panda1', 'panda2'), start=1):
            prefix = 'arm_{}.'.format(arm_index)
            values[prefix + 'arm_id'] = arm_id
            values[prefix + 'joint_names'] = _ARM_JOINTS[arm_id]
            values[prefix + 'max_velocity'] = [0.5] * 7
            values[prefix + 'max_acceleration'] = [4.0] * 7
        return values

    def _valid_hold_parameters(self):
        values = {}
        for arm_index, arm_id in enumerate(('panda1', 'panda2'), start=1):
            prefix = 'arm_{}.'.format(arm_index)
            values[prefix + 'arm_id'] = arm_id
            values[prefix + 'k_gains'] = [0.0] * 7
            values[prefix + 'd_gains'] = [0.0] * 7
            values[prefix + 'max_effort'] = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]
        return values

    def _valid_impedance_parameters(self):
        values = {
            'watchdog_timeout': 0.5,
            'max_header_age': 0.5,
            'future_tolerance': 0.1,
        }
        lower = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
        upper = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
        effort = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]
        for arm_index, arm_id in enumerate(('panda1', 'panda2'), start=1):
            prefix = 'arm_{}.'.format(arm_index)
            values[prefix + 'arm_id'] = arm_id
            values[prefix + 'joint_names'] = _ARM_JOINTS[arm_id]
            values[prefix + 'k_gains'] = [0.0] * 7
            values[prefix + 'd_gains'] = [0.0] * 7
            values[prefix + 'max_effort'] = effort
            values[prefix + 'position_lower'] = lower
            values[prefix + 'position_upper'] = upper
            values[prefix + 'max_target_velocity'] = [0.5] * 7
        return values

    def _claimed_interfaces(self, interface_kind):
        client = self._client(
            ListHardwareInterfaces,
            '/controller_manager/list_hardware_interfaces',
        )
        response = self._call(client, ListHardwareInterfaces.Request())
        return {
            interface.name
            for interface in response.command_interfaces
            if interface.name.endswith('/' + interface_kind) and interface.is_claimed
        }

    def _hardware_interface_names(self):
        client = self._client(
            ListHardwareInterfaces,
            '/controller_manager/list_hardware_interfaces',
        )
        response = self._call(client, ListHardwareInterfaces.Request())
        return (
            {interface.name for interface in response.command_interfaces},
            {interface.name for interface in response.state_interfaces},
        )

    def _set_enabled(self, arm_index, enabled):
        service_name = '/{}/arm_{}/enable'.format(_VELOCITY_CONTROLLER, arm_index)
        client = self._client(SetBool, service_name)
        request = SetBool.Request()
        request.data = enabled
        response = self._call(client, request)
        self.assertTrue(response.success, response.message)

    def _make_command(self, arm_id, velocities):
        message = JointJog()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.joint_names = list(reversed(_ARM_JOINTS[arm_id]))
        message.velocities = list(reversed(velocities))
        return message

    def _wait_for_velocities(self, expected, publishers_and_commands=()):
        received = {}

        def callback(message):
            received['velocities'] = dict(zip(message.name, message.velocity))

        subscription = self.node.create_subscription(
            JointState,
            '/franka/joint_states',
            callback,
            10,
        )
        try:
            deadline = time.monotonic() + _OBSERVATION_TIMEOUT_SEC
            next_publish = 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_publish:
                    for publisher, message in publishers_and_commands:
                        message.header.stamp = self.node.get_clock().now().to_msg()
                        publisher.publish(message)
                    next_publish = now + 0.05
                rclpy.spin_once(self.node, timeout_sec=0.05)
                velocities = received.get('velocities', {})
                if set(velocities) != set(_EXPECTED_JOINTS):
                    continue
                if all(
                    math.isfinite(velocities[joint]) and
                    math.isclose(velocities[joint], value, rel_tol=0.0, abs_tol=0.01)
                    for joint, value in expected.items()
                ):
                    return
            self.fail('velocity state did not reach {}; last value was {}'.format(
                expected,
                received.get('velocities'),
            ))
        finally:
            self.node.destroy_subscription(subscription)

    def test_no_motion_controller_is_loaded_or_active_by_default(self):
        hardware_client = self._client(
            ListHardwareComponents,
            '/controller_manager/list_hardware_components',
        )
        hardware = self._call(hardware_client, ListHardwareComponents.Request())
        self.assertEqual(len(hardware.component), 1)
        self.assertEqual(hardware.component[0].plugin_name, 'mock_components/GenericSystem')

        states = self._controller_states()
        self.assertEqual(states, {'joint_state_broadcaster': 'active'})

    def test_controller_configuration_rejects_missing_duplicate_and_malformed_parameters(self):
        scenarios = []
        scenarios.append(('missing required parameters', None))

        duplicate_ids = self._valid_velocity_parameters()
        duplicate_ids['arm_2.arm_id'] = 'panda1'
        scenarios.append(('duplicate arm IDs', duplicate_ids))

        malformed_joint = self._valid_velocity_parameters()
        malformed_joint['arm_2.joint_names'] = list(malformed_joint['arm_2.joint_names'])
        malformed_joint['arm_2.joint_names'][3] = 'bad/joint'
        scenarios.append(('malformed joint name', malformed_joint))

        for description, parameters in scenarios:
            with self.subTest(description=description):
                self._load(_VELOCITY_CONTROLLER)
                try:
                    if parameters is not None:
                        self._set_parameters(_VELOCITY_CONTROLLER, parameters)
                    self.assertFalse(self._configure(_VELOCITY_CONTROLLER))
                    self._wait_for_controller_state(_VELOCITY_CONTROLLER, 'unconfigured')
                    self.assertEqual(self._claimed_interfaces('velocity'), set())
                finally:
                    self._unload(_VELOCITY_CONTROLLER)

    def test_hold_configuration_succeeds_but_generic_system_activation_is_rejected(self):
        self._load(_HOLD_CONTROLLER)
        try:
            self._set_parameters(_HOLD_CONTROLLER, self._valid_hold_parameters())
            self.assertTrue(self._configure(_HOLD_CONTROLLER))
            self._wait_for_controller_state(_HOLD_CONTROLLER, 'inactive')

            command_interfaces, state_interfaces = self._hardware_interface_names()
            expected_effort_commands = {
                joint + '/effort' for joint in _EXPECTED_JOINTS
            }
            expected_joint_states = {
                joint + '/' + interface_kind
                for joint in _EXPECTED_JOINTS
                for interface_kind in ('position', 'velocity', 'effort')
            }
            self.assertTrue(expected_effort_commands.issubset(command_interfaces))
            self.assertTrue(expected_joint_states.issubset(state_interfaces))
            self.assertTrue(all(
                arm_id + '/robot_state' not in state_interfaces and
                arm_id + '/robot_model' not in state_interfaces
                for arm_id in ('panda1', 'panda2')
            ))

            # The joint interfaces exist, but GenericSystem has neither production pointer state.
            self._switch(activate=(_HOLD_CONTROLLER,), expect_success=False)
            self._wait_for_controller_state(_HOLD_CONTROLLER, 'inactive')
            self.assertEqual(self._claimed_interfaces('effort'), set())
        finally:
            self._unload(_HOLD_CONTROLLER)

    def test_impedance_configuration_succeeds_but_generic_activation_is_rejected(self):
        self._load(_IMPEDANCE_CONTROLLER)
        try:
            self._set_parameters(
                _IMPEDANCE_CONTROLLER,
                self._valid_impedance_parameters(),
            )
            self.assertTrue(self._configure(_IMPEDANCE_CONTROLLER))
            self._wait_for_controller_state(_IMPEDANCE_CONTROLLER, 'inactive')

            command_interfaces, state_interfaces = self._hardware_interface_names()
            self.assertTrue(
                {joint + '/effort' for joint in _EXPECTED_JOINTS}.issubset(
                    command_interfaces
                )
            )
            self.assertTrue(all(
                arm_id + '/robot_state' not in state_interfaces and
                arm_id + '/robot_model' not in state_interfaces
                for arm_id in ('panda1', 'panda2')
            ))

            self._switch(activate=(_IMPEDANCE_CONTROLLER,), expect_success=False)
            self._wait_for_controller_state(_IMPEDANCE_CONTROLLER, 'inactive')
            self.assertEqual(self._claimed_interfaces('effort'), set())
        finally:
            self._unload(_IMPEDANCE_CONTROLLER)

    def test_velocity_zero_start_independent_arms_both_arms_and_zero_return(self):
        self._load(_VELOCITY_CONTROLLER)
        publishers = []
        try:
            self._set_parameters(_VELOCITY_CONTROLLER, self._valid_velocity_parameters())
            self.assertTrue(self._configure(_VELOCITY_CONTROLLER))
            self._wait_for_controller_state(_VELOCITY_CONTROLLER, 'inactive')

            self._switch(activate=(_VELOCITY_CONTROLLER,))
            self._wait_for_controller_state(_VELOCITY_CONTROLLER, 'active')
            self.assertEqual(
                self._claimed_interfaces('velocity'),
                {joint + '/velocity' for joint in _EXPECTED_JOINTS},
            )
            zero = {joint: 0.0 for joint in _EXPECTED_JOINTS}
            self._wait_for_velocities(zero)

            for arm_index in (1, 2):
                publisher = self.node.create_publisher(
                    JointJog,
                    '/{}/arm_{}/joint_jog'.format(_VELOCITY_CONTROLLER, arm_index),
                    1,
                )
                deadline = time.monotonic() + _SERVICE_WAIT_TIMEOUT_SEC
                while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                self.assertGreater(publisher.get_subscription_count(), 0)
                publishers.append(publisher)

            arm_1_values = [0.08, -0.07, 0.06, -0.05, 0.04, -0.03, 0.02]
            arm_2_values = [-0.02, 0.03, -0.04, 0.05, -0.06, 0.07, -0.08]
            arm_1_command = self._make_command('panda1', arm_1_values)
            arm_2_command = self._make_command('panda2', arm_2_values)

            self._set_enabled(1, True)
            expected = dict(zero)
            expected.update(dict(zip(_ARM_JOINTS['panda1'], arm_1_values)))
            self._wait_for_velocities(expected, ((publishers[0], arm_1_command),))

            self._set_enabled(2, True)
            expected.update(dict(zip(_ARM_JOINTS['panda2'], arm_2_values)))
            self._wait_for_velocities(
                expected,
                ((publishers[0], arm_1_command), (publishers[1], arm_2_command)),
            )

            self._set_enabled(1, False)
            expected.update({joint: 0.0 for joint in _ARM_JOINTS['panda1']})
            self._wait_for_velocities(expected, ((publishers[1], arm_2_command),))

            # Leave arm 2 enabled and stop publishing. The watchdog is a direct-to-zero safety
            # stop, independent of an explicit disable request.
            self._wait_for_velocities(zero)

            self._switch(deactivate=(_VELOCITY_CONTROLLER,))
            self._wait_for_controller_state(_VELOCITY_CONTROLLER, 'inactive')
            self.assertEqual(self._claimed_interfaces('velocity'), set())
            self._wait_for_velocities(zero)

            # Reactivation must invalidate both old commands and begin from zero again.
            self._switch(activate=(_VELOCITY_CONTROLLER,))
            self._wait_for_controller_state(_VELOCITY_CONTROLLER, 'active')
            self._wait_for_velocities(zero)
            self._switch(deactivate=(_VELOCITY_CONTROLLER,))
            self._wait_for_controller_state(_VELOCITY_CONTROLLER, 'inactive')
            self._wait_for_velocities(zero)
        finally:
            for publisher in publishers:
                self.node.destroy_publisher(publisher)
            self._unload(_VELOCITY_CONTROLLER)


@launch_testing.post_shutdown_test()
class TestDualSafeControllersFakeHardwareShutdown(unittest.TestCase):
    """Verify that all managed long-running processes shut down."""

    def test_managed_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process='ros2_control_node')
        launch_testing.asserts.assertExitCodes(proc_info, process='robot_state_publisher')
        launch_testing.asserts.assertExitCodes(proc_info, process='joint_state_publisher')
        launch_testing.asserts.assertExitCodes(proc_info, process='spawner')
