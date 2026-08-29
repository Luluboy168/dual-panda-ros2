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

"""Launch and exercise both services without robot-stack or hardware processes."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from franka_ik_interfaces.msg import IkRequest
from franka_ik_interfaces.msg import IkResult
from franka_ik_interfaces.srv import GetChainInfo
from franka_ik_interfaces.srv import SolveIk
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    """Include the installed, URDF-only production launch description."""
    launch_file = os.path.join(
        get_package_share_directory('franka_ik'),
        'launch',
        'franka_ik.launch.py',
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {}


class TestFrankaIkLaunchSmoke(unittest.TestCase):
    """Call chain-info and one deterministic IK solve through the launched node."""

    @classmethod
    def setUpClass(cls):
        """Create a test-only client node."""
        rclpy.init()
        cls.node = rclpy.create_node('franka_ik_launch_smoke_client')

    @classmethod
    def tearDownClass(cls):
        """Release all client-side ROS resources."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, service_type, service_name, request):
        client = self.node.create_client(service_type, service_name)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=0.25):
            pass
        self.assertTrue(client.service_is_ready(), '{} unavailable'.format(service_name))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), '{} timed out'.format(service_name))
        self.assertIsNone(future.exception())
        return future.result()

    def test_chain_info_and_solve_succeed(self):
        """Verify the public surface and one full FK-to-IK witness."""
        info = self._call(
            GetChainInfo,
            '/franka_ik_service/chain_info',
            GetChainInfo.Request(),
        )
        self.assertEqual(info.urdf_root_frame, 'base_link')
        self.assertEqual([chain.arm_id for chain in info.chains], ['panda1', 'panda2'])
        self.assertEqual(info.default_solver, IkRequest.SOLVER_NUMERIC)
        self.assertFalse(info.analytic_backend_available)

        request = SolveIk.Request()
        request.request.frame_id = 'panda1_link0'
        request.request.arm_id = 'panda1'
        request.request.tip_frame = IkRequest.TIP_FLANGE
        request.request.target_pose.position.x = -0.2679670099747049
        request.request.target_pose.position.y = -0.2545381187828218
        request.request.target_pose.position.z = 1.0288313706801246
        request.request.target_pose.orientation.x = 0.34343406548352096
        request.request.target_pose.orientation.y = -0.3411330101002692
        request.request.target_pose.orientation.z = -0.40399181856832234
        request.request.target_pose.orientation.w = 0.7761906483688463
        request.request.seed_positions = [
            -0.6122365415847599,
            0.46267116354603566,
            -2.370918110186299,
            -1.356083329330045,
            0.021120164739621305,
            3.176221140891596,
            -0.9144407438822955,
        ]
        request.request.redundancy_mode = IkRequest.REDUNDANCY_FROM_SEED
        request.request.max_solutions = 1
        request.request.solver = IkRequest.SOLVER_DEFAULT
        response = self._call(
            SolveIk,
            '/franka_ik_service/solve_ik',
            request,
        )
        self.assertEqual(response.result.result, IkResult.RESULT_SUCCESS, response.result.message)
        self.assertEqual(response.result.solver_used, IkRequest.SOLVER_NUMERIC)
        self.assertEqual(len(response.result.solutions), 1)
        self.assertLessEqual(response.result.solutions[0].position_error, 1.0e-4)
        self.assertLessEqual(response.result.solutions[0].orientation_error, 1.0e-3)


@launch_testing.post_shutdown_test()
class TestFrankaIkLaunchShutdown(unittest.TestCase):
    """Require the launched node to handle test shutdown cleanly."""

    def test_managed_processes_exit_zero(self, proc_info):
        """Assert every process started by the launch description exits successfully."""
        launch_testing.asserts.assertExitCodes(proc_info)
