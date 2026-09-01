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
The two launch files, the parameter precedence, and two tests about the tests.

Everything that touches a live ROS graph here runs in a SUBPROCESS on
``ROS_DOMAIN_ID`` 226 (``conftest.LAUNCH_DOMAIN_ID``), so it never shares a
graph with the in-process node tests on 225.
"""

import ast
import os
import subprocess
import sys
import time

from conftest import (
    CORE_MODULES, driver_modules_present, LAUNCH_DOMAIN_ID, needs_driver_modules)
import pytest

#: The repository root, two levels above this file.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

TEST_DIR = os.path.dirname(os.path.abspath(__file__))

_POLL_S = 0.5


def launch_environment(**extra):
    """Return an environment pinned to this file's own domain."""
    environment = dict(os.environ)
    environment.update({
        'ROS_DOMAIN_ID': LAUNCH_DOMAIN_ID,
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'FASTDDS_BUILTIN_TRANSPORTS': 'SHM',
        'PYTHONUNBUFFERED': '1',
    })
    environment.update(extra)
    return environment


def ros2(*arguments, timeout_s=30.0):
    """Run one ``ros2`` command on the launch domain and return its output."""
    completed = subprocess.run(
        ['ros2', *arguments], env=launch_environment(), timeout=timeout_s,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    return completed.returncode, completed.stdout.decode('utf-8', 'replace')


class Launch:
    """One ``ros2 launch`` subprocess, stopped on the way out."""

    def __init__(self, launch_file, *arguments):
        """Start the launch and remember its process."""
        self.process = subprocess.Popen(
            ['ros2', 'launch', 'franka_robotiq', launch_file, *arguments],
            env=launch_environment(), stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)

    def wait_for_nodes(self, wanted, timeout_s=60.0):
        """Poll ``ros2 node list`` until every wanted node name appears."""
        deadline = time.monotonic() + timeout_s
        seen = ''
        while time.monotonic() < deadline:
            _code, seen = ros2('node', 'list')
            if all(name in seen for name in wanted):
                return seen
            time.sleep(_POLL_S)
        raise AssertionError('nodes {} never appeared; last list:\n{}'.format(
            wanted, seen))

    def stop(self):
        """Stop the launch and reap it."""
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=10)


@pytest.fixture
def launched():
    """Yield a factory that starts one launch and always stops it."""
    started = []

    def start(launch_file, *arguments):
        """Start one launch file with these arguments."""
        handle = Launch(launch_file, *arguments)
        started.append(handle)
        return handle

    try:
        yield start
    finally:
        for handle in started:
            handle.stop()


# ----------------------------------------------------------------------
# The headline gate
# ----------------------------------------------------------------------


@needs_driver_modules
def test_the_dual_fake_launch_brings_up_both_nodes_and_the_whole_surface(launched):
    """
    The merge gate, in test form: both nodes, the whole surface, a stalled close.

    With no object the fake reaches the requested position, reports
    ``gOBJ = 0x03``, and the node correctly returns ``stalled: false`` -- so
    the seeded 30 mm object is what makes this gate's own assertion reachable.
    """
    handle = launched('dual_robotiq.launch.py', 'use_fake:=true',
                      'fake_object_mm:=30.0')
    handle.wait_for_nodes(('/panda1_robotiq', '/panda2_robotiq'))

    _code, services = ros2('service', 'list')
    for arm_id in ('panda1', 'panda2'):
        for name in ('open', 'close', 'stop', 'reactivate'):
            assert '/{}_robotiq/{}'.format(arm_id, name) in services

    _code, topics = ros2('topic', 'list')
    for arm_id in ('panda1', 'panda2'):
        assert '/{}_robotiq/status'.format(arm_id) in topics
        assert '/{}_robotiq/joint_states'.format(arm_id) in topics

    _code, actions = ros2('action', 'list')
    for arm_id in ('panda1', 'panda2'):
        assert '/{}_robotiq/gripper_action'.format(arm_id) in actions

    code, _output = ros2('service', 'call', '/panda1_robotiq/reactivate',
                         'std_srvs/srv/Trigger', timeout_s=60.0)
    assert code == 0
    code, output = ros2(
        'action', 'send_goal', '/panda1_robotiq/gripper_action',
        'control_msgs/action/GripperCommand',
        '{command: {position: 0.0, max_effort: 0.0}}', timeout_s=60.0)
    assert code == 0, output
    assert field_reads(output, 'reached_goal', True), output
    assert field_reads(output, 'stalled', True), output


def field_reads(output, field, value):
    """Return whether a ``ros2 action send_goal`` dump shows ``field=value``."""
    wanted = {'{}={}'.format(field, value), '{}:{}'.format(field, value),
              '{}:{}'.format(field, str(value).lower())}
    for line in output.splitlines():
        stripped = line.strip().replace(' ', '')
        if stripped in wanted:
            return True
    return False


@needs_driver_modules
def test_fake_object_mm_zero_gives_a_reached_goal_with_stalled_false(launched):
    """The other half of the fake-object argument: nothing in the way."""
    handle = launched('robotiq.launch.py', 'arm_id:=panda1', 'use_fake:=true',
                      'fake_object_mm:=0.0')
    handle.wait_for_nodes(('/panda1_robotiq',))
    code, _output = ros2('service', 'call', '/panda1_robotiq/reactivate',
                         'std_srvs/srv/Trigger', timeout_s=60.0)
    assert code == 0
    code, output = ros2(
        'action', 'send_goal', '/panda1_robotiq/gripper_action',
        'control_msgs/action/GripperCommand',
        '{command: {position: 0.0, max_effort: 0.0}}', timeout_s=60.0)
    assert code == 0, output
    assert field_reads(output, 'reached_goal', True), output
    assert field_reads(output, 'stalled', False), output


@needs_driver_modules
def test_the_single_launch_takes_arm_id_and_serial_id(launched):
    """The single launch names the node from arm_id and passes the binding."""
    handle = launched('robotiq.launch.py', 'arm_id:=panda2', 'use_fake:=true')
    handle.wait_for_nodes(('/panda2_robotiq',))
    code, output = ros2('param', 'get', '/panda2_robotiq', 'arm_id')
    assert code == 0
    assert 'panda2' in output


# ----------------------------------------------------------------------
# Parameter precedence, checked on the launch description itself
# ----------------------------------------------------------------------


def describe(launch_file, **arguments):
    """Return the parameter mapping one launch file builds for its Node."""
    import launch
    import launch_ros

    source = os.path.join(REPO_ROOT, 'franka_robotiq', 'launch', launch_file)
    description = launch.LaunchDescriptionSource(
        launch.launch_description_sources.PythonLaunchDescriptionSource(source))
    context = launch.LaunchContext()
    for name, value in arguments.items():
        context.launch_configurations[name] = value
    actions = description.get_launch_description().entities
    built = []
    for action in actions:
        if isinstance(action, launch.actions.OpaqueFunction):
            for entity in action.execute(context) or ():
                if isinstance(entity, launch_ros.actions.Node):
                    built.append(entity)
    return built


@needs_driver_modules
def test_a_launch_argument_overrides_the_params_file(tmp_path):
    """Node defaults < params_file < launch arguments, in that order."""
    params = tmp_path / 'params.yaml'
    params.write_text('/panda1_robotiq:\n  ros__parameters:\n'
                      '    serial_id: "usb-FROM-FILE-if00-port0"\n',
                      encoding='utf-8')
    nodes = describe('robotiq.launch.py', arm_id='panda1',
                     serial_id='usb-FROM-ARGUMENT-if00-port0', usb_path='',
                     use_fake='false', fake_object_mm='30.0',
                     params_file=str(params), node_name='')
    assert len(nodes) == 1
    parameters = nodes[0]._Node__parameters
    assert str(params) in [str(entry) for entry in parameters]
    overrides = [entry for entry in parameters if isinstance(entry, dict)][-1]
    assert overrides['serial_id'] == 'usb-FROM-ARGUMENT-if00-port0'
    assert parameters.index(str(params)) < parameters.index(overrides)


@needs_driver_modules
def test_an_empty_binding_argument_does_not_erase_a_params_file_binding(tmp_path):
    """An unset serial_id contributes NO key rather than an empty override."""
    params = tmp_path / 'params.yaml'
    params.write_text('/panda1_robotiq:\n  ros__parameters:\n'
                      '    serial_id: "usb-FROM-FILE-if00-port0"\n',
                      encoding='utf-8')
    nodes = describe('robotiq.launch.py', arm_id='panda1', serial_id='',
                     usb_path='', use_fake='false', fake_object_mm='30.0',
                     params_file=str(params), node_name='')
    overrides = [entry for entry in nodes[0]._Node__parameters
                 if isinstance(entry, dict)][-1]
    assert 'serial_id' not in overrides
    assert 'usb_path' not in overrides


@needs_driver_modules
def test_the_dual_launch_refuses_two_identical_serial_ids():
    """One adapter cannot drive two grippers, and the launch says so first."""
    from franka_robotiq import discovery

    with pytest.raises(discovery.BindingError):
        describe('dual_robotiq.launch.py', use_fake='false',
                 fake_object_mm='30.0', params_file='',
                 panda1_serial_id='usb-SAME-if00-port0',
                 panda2_serial_id='usb-SAME-if00-port0',
                 panda1_usb_path='', panda2_usb_path='')


# ----------------------------------------------------------------------
# Two tests about the tests themselves
# ----------------------------------------------------------------------


def test_conftest_imports_no_ros_at_module_level():
    """
    ``conftest.py`` must import no ROS at module level, and here is why.

    pytest imports a directory's conftest for EVERY collection rooted there,
    including the zero-ROS run below. A module-level ``import rclpy`` turns
    that gate into a collection error rather than a green run.
    """
    source = open(os.path.join(TEST_DIR, 'conftest.py'), encoding='utf-8').read()
    tree = ast.parse(source)
    banned = ('rclpy', 'rcl_interfaces', 'sensor_msgs', 'diagnostic_msgs',
              'control_msgs', 'std_srvs', 'launch', 'launch_ros')
    for statement in tree.body:
        names = []
        if isinstance(statement, ast.Import):
            names = [alias.name for alias in statement.names]
        elif isinstance(statement, ast.ImportFrom):
            names = [statement.module or '']
        for name in names:
            root = name.split('.')[0]
            assert root not in banned, (
                'conftest.py imports {} at module level'.format(name))


def test_the_core_modules_import_with_no_ros_on_the_path():
    """
    The zero-ROS gate, run as a subprocess so it has a home in the suite.

    This is the run that PROVES the zero-ROS-imports rule instead of
    asserting it.
    """
    if not driver_modules_present():
        pytest.skip('the six core modules are not present in this workspace')
    files = [os.path.join(TEST_DIR, 'test_{}.py'.format(name))
             for name in ('protocol', 'units', 'discovery')]
    missing = [path for path in files if not os.path.isfile(path)]
    if missing:
        pytest.skip('the core-module tests are not present: {}'.format(missing))
    completed = subprocess.run(
        ['env', '-i', 'HOME={}'.format(os.environ.get('HOME', '/tmp')),
         'PATH=/usr/bin:/bin', sys.executable, '-m', 'pytest', *files, '-q'],
        cwd=TEST_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        check=False, timeout=300)
    assert completed.returncode == 0, completed.stdout.decode('utf-8', 'replace')


# ----------------------------------------------------------------------
# The cross-part install layout (seam 1)
# ----------------------------------------------------------------------


def test_the_installed_layout_carries_one_file_from_each_part():
    """
    One installed file per part, with the documentation half honestly guarded.

    The documentation and udev directories are written separately and land
    after this package's build files do, so their ``glob()`` rows install
    nothing until then. The assertion is not weakened, only deferred to the
    merge where it can pass.
    """
    from ament_index_python.packages import get_package_share_directory

    try:
        share = get_package_share_directory('franka_robotiq')
    except Exception:  # noqa: BLE001 - not installed is a skip, not a failure
        pytest.skip('franka_robotiq is not installed; build and source first')
        return
    package_root = os.path.join(REPO_ROOT, 'franka_robotiq')

    # The driver half: one of the six ROS-free modules, importable.
    if driver_modules_present():
        import franka_robotiq.protocol as installed_protocol
        assert os.path.isfile(installed_protocol.__file__)
    else:
        pytest.skip('the driver modules have not landed yet')
        return

    # This half: the launch file an operator actually types.
    assert os.path.isfile(os.path.join(share, 'launch', 'dual_robotiq.launch.py'))

    # The documentation half, deferred to the merge where it can pass.
    if not os.path.isdir(os.path.join(package_root, 'doc')):
        pytest.skip('the package documentation has not landed yet')
        return
    assert os.path.isfile(os.path.join(share, 'doc', 'MOUNTING.md'))


def test_the_node_consumes_every_module_the_seam_names():
    """The six ROS-free modules are named the same everywhere in this package."""
    source = open(os.path.join(REPO_ROOT, 'franka_robotiq', 'franka_robotiq',
                               'node.py'), encoding='utf-8').read()
    for name in CORE_MODULES:
        assert name in source, 'node.py never mentions {}'.format(name)
