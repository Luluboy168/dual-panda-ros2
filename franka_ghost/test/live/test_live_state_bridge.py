# [THROWAWAY] Session C live-state scaffold test; franka_web owns merged integration tests.
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

"""Exercise fake joint states through the loopback HTTP bridge on test domain 223."""

import http.client
import json
import os
from pathlib import Path
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from franka_ghost.dev_server import create_server
from franka_ghost.joint_source import CANONICAL_JOINT_NAMES
from franka_ghost.joint_source import require_ros_domain_id
from franka_ghost.joint_source import RosJointSource
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest


_TEST_DOMAIN_ID = '223'
_EXPECTED_JOINTS = set(CANONICAL_JOINT_NAMES)
_STATE_TIMEOUT_S = 30.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch only the reviewed fake state-only wrapper and the test fixture."""
    wrapper = Path(get_package_share_directory('franka_bringup')) / (
        'launch/operator/fake_dual_state_only.launch.py'
    )
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(wrapper)),
                launch_arguments={'use_rviz': 'false'}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {}


class TestLiveStateBridge(unittest.TestCase):
    """Run the real throwaway ROS source and HTTP bridge in this test process."""

    @classmethod
    def setUpClass(cls):
        cls.source = None
        cls.server = None
        cls.server_thread = None
        try:
            if os.environ.get('ROS_DOMAIN_ID') != _TEST_DOMAIN_ID:
                raise AssertionError('registered test must run on ROS_DOMAIN_ID=223')
            if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
                raise AssertionError('registered test must use LOCALHOST discovery')
            # The normal CLI remains locked to interactive domain 82. This narrow
            # injection exists only because registered tests must use their own ID.
            try:
                require_ros_domain_id()
            except RuntimeError:
                pass
            else:
                raise AssertionError('default ROS source guard unexpectedly accepted domain 223')

            cls.source = RosJointSource(
                topic='/franka/joint_states',
                required_domain_id=_TEST_DOMAIN_ID,
            )
            web_root = Path(get_package_share_directory('franka_ghost')) / 'web'
            cls.server = create_server(
                port=0,
                web_root=web_root,
                source=cls.source,
                rate_hz=20.0,
            )
            cls.server_thread = threading.Thread(
                target=cls.server.serve_forever,
                name='franka-ghost-live-test-http',
            )
            cls.server_thread.start()
        except BaseException:
            cls._close_bridge()
            raise

    @classmethod
    def tearDownClass(cls):
        cls._close_bridge()

    @classmethod
    def _close_bridge(cls):
        server = getattr(cls, 'server', None)
        thread = getattr(cls, 'server_thread', None)
        source = getattr(cls, 'source', None)
        if server is not None:
            if thread is not None and thread.is_alive():
                server.shutdown()
            server.server_close()
        elif source is not None:
            source.close()
        if thread is not None:
            thread.join(timeout=5.0)
            if thread.is_alive():
                raise AssertionError('loopback HTTP server thread did not stop')
        cls.server = None
        cls.server_thread = None
        cls.source = None

    @classmethod
    def _state_json(cls):
        connection = http.client.HTTPConnection(
            *cls.server.server_address,
            timeout=2.0,
        )
        try:
            connection.request('GET', '/state.json')
            response = connection.getresponse()
            raw = response.read()
            if response.status != 200:
                raise AssertionError(
                    f'/state.json returned HTTP {response.status}: {raw!r}'
                )
            return json.loads(raw)
        finally:
            connection.close()

    @classmethod
    def _wait_for_complete_state(cls):
        deadline = time.monotonic() + _STATE_TIMEOUT_S
        last_state = None
        while time.monotonic() < deadline:
            last_state = cls._state_json()
            if set(last_state['joints']) == _EXPECTED_JOINTS and not last_state['stale']:
                return last_state
            time.sleep(0.05)
        raise AssertionError(
            'state.json did not reach 14 fresh canonical joints within '
            f'{_STATE_TIMEOUT_S}s; last state={last_state!r}'
        )

    def test_live_fake_state_reaches_loopback_bridge(self):
        state = self._wait_for_complete_state()
        self.assertEqual(state['schema'], 'franka.ghost.state/1')
        self.assertEqual(state['source'], 'ros')
        self.assertEqual(set(state['joints']), _EXPECTED_JOINTS)
        self.assertEqual(len(state['joints']), 14)

    def test_domain_is_unambiguous_and_localhost_only(self):
        self.assertEqual(os.environ['ROS_DOMAIN_ID'], _TEST_DOMAIN_ID)
        self.assertEqual(os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
        deadline = time.monotonic() + _STATE_TIMEOUT_S
        publisher_count = 0
        while time.monotonic() < deadline:
            publisher_count = self.source._node.count_publishers('/franka/joint_states')
            if publisher_count:
                break
            time.sleep(0.05)
        self.assertEqual(
            publisher_count,
            1,
            'expected exactly one publisher in reserved domain 223; '
            f'found {publisher_count}',
        )

    def test_fake_positions_are_static_zeros_and_bridge_is_decimated(self):
        """Record the observed Jazzy GenericSystem behavior without changing core xacro."""
        first = self._wait_for_complete_state()
        start = time.monotonic()
        deadline = start + 1.0
        last = first
        while time.monotonic() < deadline:
            last = self._state_json()
            time.sleep(0.025)
        elapsed = time.monotonic() - start

        # On this installed Jazzy stack, GenericSystem does not consume the
        # xacro's legacy `initial_position` joint parameter. All fourteen fake
        # positions therefore remain exactly zero in state-only mode.
        self.assertTrue(all(value == 0.0 for value in first['joints'].values()))
        self.assertEqual(last['joints'], first['joints'])

        promoted_rate = (last['seq'] - first['seq']) / elapsed
        self.assertGreaterEqual(promoted_rate, 5.0)
        self.assertLessEqual(promoted_rate, 35.0)


@launch_testing.post_shutdown_test()
class TestLiveStateBridgeShutdown(unittest.TestCase):
    """Require clean shutdown from every process managed by the fake wrapper."""

    def test_managed_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process='spawner')
        launch_testing.asserts.assertExitCodes(proc_info, process='ros2_control_node')
        launch_testing.asserts.assertExitCodes(proc_info, process='robot_state_publisher')
        launch_testing.asserts.assertExitCodes(proc_info, process='joint_state_publisher')
