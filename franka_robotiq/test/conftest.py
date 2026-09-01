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

r"""
Shared pytest wiring for franka_robotiq.

THIS FILE MUST NOT IMPORT ROS AT MODULE LEVEL, and the reason is a gate rather
than a preference. pytest imports a directory's ``conftest.py`` for EVERY
collection rooted there, including the zero-ROS run that proves this package's
six core modules import with no ROS on the path at all::

    env -i HOME="$HOME" PATH=/usr/bin:/bin python3 -m pytest \\
        franka_robotiq/test/test_protocol.py \\
        franka_robotiq/test/test_units.py \\
        franka_robotiq/test/test_discovery.py -q

A module-level ``import rclpy`` here turns that gate into a collection error,
so the ``rclpy`` import lives inside the fixture body and a collection that
never asks for the fixture never imports ROS.

``franka_robotiq`` is an ``ament_python`` package and therefore has no
CMakeLists ``ENV`` block to pin a per-test ``ROS_DOMAIN_ID`` from. The domain
is pinned here instead, before anything can call ``rclpy.init()``.

Domain allocation (master table in ``franka_bringup/CMakeLists.txt``;
191-218 taken there, franka_web 219/220/224, franka_ik 221/222,
franka_ghost 223, hard ceiling 232 for ``rmw_fastrtps_cpp``)::

    225  franka_robotiq  test_node_contract.py + test_action_contract.py
    226  franka_robotiq  test_launch.py   (exported per subprocess, in that file)
    227  franka_web      e2e_fake_dual_gripper_row_test.py

228-232 stay free. Adding these three rows to the master table is a handoff to
a session permitted to edit reviewed core.
"""

import importlib.util
import os
import sys

import pytest

sys.dont_write_bytecode = True

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)

#: The domain this package's in-process live-node tests run on.
NODE_DOMAIN_ID = '225'

#: The domain ``test_launch.py`` exports for the subprocesses it starts.
LAUNCH_DOMAIN_ID = '226'

os.environ.setdefault('ROS_DOMAIN_ID', NODE_DOMAIN_ID)
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'SHM')

#: The six ROS-free modules the node consumes through the pinned seam.
CORE_MODULES = ('registers', 'protocol', 'units', 'driver', 'discovery', 'fake')

DRIVER_SKIP_REASON = (
    "franka_robotiq's protocol, driver, discovery, units, registers and fake "
    'modules are not importable in this workspace. These tests drive the real '
    'node against the protocol-faithful fake over a pty, so they run as soon '
    'as that half of the package is present.')


def driver_modules_present():
    """Return whether all six of the node's ROS-free dependencies import."""
    for name in CORE_MODULES:
        try:
            if importlib.util.find_spec('franka_robotiq.{}'.format(name)) is None:
                return False
        except (ImportError, ValueError):
            return False
    return True


needs_driver_modules = pytest.mark.skipif(
    not driver_modules_present(), reason=DRIVER_SKIP_REASON)


@pytest.fixture(scope='session')
def ros_context():
    """
    Initialise rclpy once for the whole session and shut it down after.

    The ``import rclpy`` is INSIDE this body on purpose: see the module
    docstring. A collection that never requests this fixture never imports
    ROS, which is what keeps the zero-ROS gate runnable.
    """
    import rclpy
    rclpy.init(args=None)
    try:
        yield rclpy
    finally:
        rclpy.try_shutdown()


@pytest.fixture
def gripper_cell(ros_context):
    """
    Build fake-backed nodes and one client node, spun on a shared executor.

    The factory it yields returns ``(node, client)``: a real
    :class:`~franka_robotiq.node.RobotiqNode` bound to a
    protocol-faithful fake gripper over a pty, and a plain node the test uses
    to subscribe, call services and send goals. Nothing here is mocked at the
    protocol level -- that is the whole point of the fake.
    """
    import threading

    from franka_robotiq.node import RobotiqNode
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    built = []
    clients = []
    executor = MultiThreadedExecutor(num_threads=6)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def build(**overrides):
        """Return one fake-backed node plus a client node for talking to it."""
        values = {'use_fake': True, 'poll_rate_hz': 50.0}
        values.update(overrides)
        node = RobotiqNode(parameter_overrides=[
            Parameter(name, value=value) for name, value in values.items()])
        client = Node('{}_client'.format(node.get_name()))
        built.append(node)
        clients.append(client)
        executor.add_node(node)
        executor.add_node(client)
        return node, client

    try:
        yield build
    finally:
        for node in built:
            node.shutdown()
        executor.shutdown()
        thread.join(timeout=10.0)
        for node in built + clients:
            node.destroy_node()


def wait_until(predicate, timeout_s=10.0, interval_s=0.02):
    """Poll ``predicate`` until it is truthy; return its last value."""
    import time

    deadline = time.monotonic() + timeout_s
    value = predicate()
    while not value and time.monotonic() < deadline:
        time.sleep(interval_s)
        value = predicate()
    return value
