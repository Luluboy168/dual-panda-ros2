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

"""Executes only the fixed fake state-only operator wrapper."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareComponents
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    wrapper = os.path.join(
        get_package_share_directory('franka_bringup'), 'launch', 'operator',
        'fake_dual_state_only.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(wrapper)),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestOperatorFakeStateOnly(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('operator_fake_state_only_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, service_type, service_name):
        client = self.node.create_client(service_type, service_name)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=0.5):
            pass
        self.assertTrue(client.service_is_ready(), '{} unavailable'.format(service_name))
        future = client.call_async(service_type.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), '{} timed out'.format(service_name))
        self.assertIsNone(future.exception())
        return future.result()

    def test_only_mock_hardware_is_active(self):
        response = self._call(
            ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.assertEqual(len(response.component), 1)
        self.assertEqual(response.component[0].name, 'FrankaMultiHardwareInterface')
        self.assertEqual(response.component[0].plugin_name, 'mock_components/GenericSystem')
        self.assertEqual(response.component[0].state.label, 'active')

    def test_no_motion_controller_is_loaded_or_active(self):
        deadline = time.monotonic() + 10.0
        controllers = {}
        while time.monotonic() < deadline:
            response = self._call(ListControllers, '/controller_manager/list_controllers')
            controllers = {controller.name: controller for controller in response.controller}
            broadcaster = controllers.get('joint_state_broadcaster', None)
            if broadcaster is not None and broadcaster.state == 'active':
                break
            time.sleep(0.05)
        self.assertEqual(set(controllers), {'joint_state_broadcaster'})
        self.assertEqual(controllers['joint_state_broadcaster'].state, 'active')

    def test_spawner_exits_cleanly(self, proc_info):
        proc_info.assertWaitForShutdown(process='spawner', timeout=10)
        launch_testing.asserts.assertExitCodes(proc_info, process='spawner')


@launch_testing.post_shutdown_test()
class TestOperatorFakeStateOnlyShutdown(unittest.TestCase):
    def test_all_managed_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process='ros2_control_node')
        launch_testing.asserts.assertExitCodes(proc_info, process='robot_state_publisher')
        launch_testing.asserts.assertExitCodes(proc_info, process='joint_state_publisher')
