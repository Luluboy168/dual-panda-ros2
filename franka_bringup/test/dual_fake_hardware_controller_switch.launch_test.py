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
Exercise the controller_manager load/configure/switch sequence with mock hardware only.

The two example controllers driven here are deliberately NOT registered by any shipped production
config any more: until 2026-08-29 ``config/real/dual_controllers.yaml`` registered and fully
parameterized them, which meant one ``ros2 control load_controller <name> --set-state active``
could put an unreviewed motion controller in command of both real arms (P0-B). This test therefore
supplies the registration and the parameters itself, out of band -- it sets ``<name>.type`` and
``<name>.params_file`` on the live controller_manager node (which runs with
``allow_undeclared_parameters``; see ControllerManager::load_controller, controller_manager.cpp
:1257 and :1321) and points the latter at a YAML file it writes into a temporary directory. The
switch-sequence coverage is unchanged; what is gone is the ability to do this from the shipped
configuration alone.
"""

import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    ListHardwareComponents,
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

_IMPEDANCE_CONTROLLER = 'dual_joint_impedance_example_controller'
_VELOCITY_CONTROLLER = 'dual_joint_velocity_example_controller'
_CONTROLLER_TYPES = {
    _IMPEDANCE_CONTROLLER: 'franka_example_controllers/MultiJointImpedanceExampleController',
    _VELOCITY_CONTROLLER: 'franka_example_controllers/DualJointVelocityExampleController',
}
# Test-owned, written to a temporary directory at run time. Deliberately not a file under
# franka_bringup/config: nothing that ships may carry motion gains for an unreviewed controller.
_CONTROLLER_PARAMETERS_YAML = """\
{impedance}:
  ros__parameters:
    arm_count: 2
    arm_1:
      arm_id: panda1
      k_gains: [24.0, 24.0, 24.0, 24.0, 10.0, 6.0, 2.0]
      d_gains: [2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.5]
    arm_2:
      arm_id: panda2
      k_gains: [24.0, 24.0, 24.0, 24.0, 10.0, 6.0, 2.0]
      d_gains: [2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.5]

{velocity}:
  ros__parameters:
    arm_1:
      arm_id: panda1
    arm_2:
      arm_id: panda2
""".format(impedance=_IMPEDANCE_CONTROLLER, velocity=_VELOCITY_CONTROLLER)
_SERVICE_WAIT_TIMEOUT_SEC = 30.0
_SERVICE_CALL_TIMEOUT_SEC = 15.0


@pytest.mark.launch_test
def generate_test_description():
    dual_franka_launch_file = os.path.join(
        get_package_share_directory('franka_bringup'),
        'launch',
        'real',
        'dual_franka.launch.py',
    )
    dual_franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dual_franka_launch_file),
        launch_arguments=_FAKE_LAUNCH_ARGUMENTS.items(),
    )
    return launch.LaunchDescription([
        dual_franka_launch,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestDualFakeHardwareControllerSwitch(unittest.TestCase):
    """Loads and switches both dual-arm MVP controllers without a robot connection."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('dual_fake_hardware_controller_switch_test_client')
        cls._parameters_dir = tempfile.TemporaryDirectory(
            prefix='dual_fake_hardware_controller_switch_')
        cls.parameters_file = os.path.join(cls._parameters_dir.name, 'example_controllers.yaml')
        with open(cls.parameters_file, 'w', encoding='utf-8') as handle:
            handle.write(_CONTROLLER_PARAMETERS_YAML)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls._parameters_dir.cleanup()

    def _register_controller_out_of_band(self, controller_name):
        """Supply the type and parameter file the shipped config deliberately no longer does."""
        client = AsyncParameterClient(self.node, '/controller_manager')
        self.assertTrue(
            client.wait_for_services(timeout_sec=_SERVICE_WAIT_TIMEOUT_SEC),
            'controller_manager parameter services did not become available',
        )
        future = client.set_parameters([
            Parameter(controller_name + '.type', value=_CONTROLLER_TYPES[controller_name]),
            Parameter(controller_name + '.params_file', value=self.parameters_file),
        ])
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=_SERVICE_CALL_TIMEOUT_SEC)
        self.assertTrue(future.done(), 'setting {} registration parameters timed out'.format(
            controller_name))
        self.assertIsNone(future.exception())
        self.assertTrue(
            all(result.successful for result in future.result().results),
            [result.reason for result in future.result().results],
        )

    def _client(self, service_type, service_name):
        client = self.node.create_client(service_type, service_name)
        deadline = time.monotonic() + _SERVICE_WAIT_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                return client
        self.fail('{} did not become available'.format(service_name))

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=_SERVICE_CALL_TIMEOUT_SEC)
        self.assertTrue(future.done(), '{} timed out'.format(client.srv_name))
        result = future.result()
        self.assertIsNotNone(
            result,
            '{} raised {}'.format(client.srv_name, future.exception()),
        )
        return result

    def _switch(self, client, activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 10
        response = self._call(client, request)
        self.assertTrue(response.ok, response.message)

    def _controller_states(self, client):
        response = self._call(client, ListControllers.Request())
        return {controller.name: controller.state for controller in response.controller}

    def _wait_for_controller_state(self, client, controller_name, expected_state):
        deadline = time.monotonic() + _SERVICE_WAIT_TIMEOUT_SEC
        observed_state = None
        while time.monotonic() < deadline:
            observed_state = self._controller_states(client).get(controller_name)
            if observed_state == expected_state:
                return
            time.sleep(0.05)
        self.fail(
            '{} did not reach state {}; last state was {}'.format(
                controller_name,
                expected_state,
                observed_state,
            )
        )

    def _wait_for_node_to_disappear(self, node_name):
        deadline = time.monotonic() + _SERVICE_WAIT_TIMEOUT_SEC
        absent_since = None
        while time.monotonic() < deadline:
            node_names = {name for name, _ in self.node.get_node_names_and_namespaces()}
            if node_name not in node_names:
                if absent_since is None:
                    absent_since = time.monotonic()
                elif time.monotonic() - absent_since >= 0.2:
                    return
            else:
                absent_since = None
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail('{} did not exit'.format(node_name))

    def test_controller_switch_sequence(self, proc_info):
        hardware_client = self._client(
            ListHardwareComponents,
            '/controller_manager/list_hardware_components',
        )
        hardware = self._call(hardware_client, ListHardwareComponents.Request())
        self.assertEqual(len(hardware.component), 1)
        self.assertEqual(hardware.component[0].plugin_name, 'mock_components/GenericSystem')

        load_client = self._client(LoadController, '/controller_manager/load_controller')
        configure_client = self._client(
            ConfigureController,
            '/controller_manager/configure_controller',
        )
        switch_client = self._client(SwitchController, '/controller_manager/switch_controller')
        list_client = self._client(ListControllers, '/controller_manager/list_controllers')
        unload_client = self._client(UnloadController, '/controller_manager/unload_controller')

        # The launch-owned spawner must finish before this test begins teardown.
        self._wait_for_controller_state(list_client, 'joint_state_broadcaster', 'active')
        self._wait_for_node_to_disappear('spawner_joint_state_broadcaster')
        launch_testing.asserts.assertExitCodes(proc_info, process='spawner')

        loaded = []
        try:
            for controller_name in (_IMPEDANCE_CONTROLLER, _VELOCITY_CONTROLLER):
                self._register_controller_out_of_band(controller_name)
                request = LoadController.Request()
                request.name = controller_name
                self.assertTrue(self._call(load_client, request).ok)
                loaded.append(controller_name)

                request = ConfigureController.Request()
                request.name = controller_name
                self.assertTrue(self._call(configure_client, request).ok)

            self._switch(switch_client, activate=(_IMPEDANCE_CONTROLLER,))
            states = self._controller_states(list_client)
            self.assertEqual(states[_IMPEDANCE_CONTROLLER], 'active')
            self.assertEqual(states[_VELOCITY_CONTROLLER], 'inactive')

            self._switch(
                switch_client,
                activate=(_VELOCITY_CONTROLLER,),
                deactivate=(_IMPEDANCE_CONTROLLER,),
            )
            states = self._controller_states(list_client)
            self.assertEqual(states[_IMPEDANCE_CONTROLLER], 'inactive')
            self.assertEqual(states[_VELOCITY_CONTROLLER], 'active')

            self._switch(switch_client, deactivate=(_VELOCITY_CONTROLLER,))
            states = self._controller_states(list_client)
            self.assertEqual(states[_IMPEDANCE_CONTROLLER], 'inactive')
            self.assertEqual(states[_VELOCITY_CONTROLLER], 'inactive')
        finally:
            states = self._controller_states(list_client)
            active = [name for name in loaded if states.get(name) == 'active']
            if active:
                self._switch(switch_client, deactivate=active)
            for controller_name in reversed(loaded):
                request = UnloadController.Request()
                request.name = controller_name
                self.assertTrue(self._call(unload_client, request).ok)


@launch_testing.post_shutdown_test()
class TestDualFakeHardwareControllerSwitchShutdown(unittest.TestCase):
    """Verifies clean shutdown after the fake controller switch sequence."""

    def test_all_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process='ros2_control_node')
        launch_testing.asserts.assertExitCodes(proc_info, process='robot_state_publisher')
        launch_testing.asserts.assertExitCodes(proc_info, process='joint_state_publisher')
