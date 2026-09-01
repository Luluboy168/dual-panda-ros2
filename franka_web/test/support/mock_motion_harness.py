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
The Stage 2 motion rig: the real server stack against a mock controller.

:class:`MockMotionHarness` assembles, in one process, everything a motion
session needs except the one thing fake hardware cannot provide -- the
impedance controller itself:

* the **real** ``fake_dual_state_only`` stack, as a ``ros2 launch`` child
  spawned through the shipped :class:`franka_web.launcher.ChildProcess` with
  ``PR_SET_PDEATHSIG = SIGINT``. This is the ONLY launch file this package is
  ever permitted to execute, and no robot address is passed to anything here;
* :class:`support.mock_impedance_controller.MockImpedanceController`, the
  line-by-line port of ``ArmImpedanceTargetInbox::accept`` wrapped in a node
  named ``dual_arm_joint_impedance_controller``, on its own executor thread;
* the **real** server stack: ``FrankaWebBridge`` on its own executor thread
  (exactly as ``server.py`` runs it), ``OperatorLock``, ``Broker``,
  ``ProfileStore``, ``SessionSupervisor``, the 20 Hz jog callback, and a real
  ``http_api`` server on a free loopback port.

Everything the tests drive goes over that HTTP surface and is measured at the
mock, never from the server's own bookkeeping.

Why this file exists at all (plan sections 0.1, 0.2, 8 Stage 2)
--------------------------------------------------------------
There is no fake guarded-motion launch, and the impedance controller cannot be
activated on ``mock_components/GenericSystem`` -- proven as an executable fact
by the layer-3 test in ``e2e_fake_motion_mock_test.py``. So the motion
subsystem is exercised against a mock that answers with the controller's own
rules, on a real ROS graph, with real DDS between the server and the mock.

The four seams, and why each one is a seam
------------------------------------------
The frozen ``PROFILES`` table is **not** touched. Everything below is injected
through ``SessionSupervisor``'s own constructor arguments:

``spawn``
    Faked. The stack is already up (this harness brought it up); letting the
    supervisor spawn ``production_dual_guarded_motion.launch.py`` is exactly
    what must never happen. The fake child stays alive so fault rule F7 sees a
    healthy launch.
``argv_builder``
    Returns an inert placeholder argv. ``('both', 'motion')`` maps to a
    production launch that needs two robot addresses; since ``spawn`` is fake
    the argv is never executed, and this one is deliberately not a launch
    command line and carries no address.
``recording_factory``
    :class:`support.fake_launcher.FakeRecording`. The recorder is covered
    end-to-end by the domain-219 Stage 1 e2e (real ``franka_record``, real
    bag, real seal); this rig is about the motion subsystem, and a second
    real recorder would only add a bag to clean up.
``preflight_runner``
    A scripted PASS. The RT preflight inspects the host's kernel, not the
    session; it has its own unit tests, and a machine without an RT kernel
    must not turn this into a red motion test.

And two on the bridge (:class:`MotionE2EBridge`), each stated where it lives.

What is deliberately NOT faked
------------------------------
The jog stream, the enable/disable service calls, the error-recovery service
call, the ``/diagnostics`` fault input, ``/franka/joint_states``, the hardware
component, the operator lock, the gains upload and validation, the HTTP layer,
the SSE fan-out, and every state transition. All of those run for real.

The mock stack's pose, and why a controller is loaded to move it
----------------------------------------------------------------
``mock_components/GenericSystem`` starts every joint at exactly ``0.0``. A
real Panda cannot be there: the reviewed limit policy
(``franka_example_controllers/config/panda_joint_limits_v1.yaml``) puts joint 4
in ``[-3.0718, -0.0698]``, so **no validator-accepted fence can contain the
mock's boot pose**, and every motion session would be refused by the plan's
section 5.4 fence-vs-pose precondition -- correctly, but for a reason that has
nothing to do with the motion subsystem.

So the harness loads one stock ``position_controllers/JointGroupPositionController``
onto the fake stack and commands the arms into the Panda home pose
:data:`HOME_POSE`. Every fence in this rig is then a real fence around a real,
reachable pose, and the section 5.4 precondition is exercised for what it is
rather than short-circuited. The controller claims only ``position`` command
interfaces, so it does not collide with the impedance controller's ``effort``
claim in the layer-3 activation test.

Manual run (the browser rig, domain 80)
---------------------------------------
From the package root, with the workspace sourced, and with these four
variables exported: ``PYTHONPATH=$PWD/test:$PYTHONPATH`` -- **appended**, so
``support.*`` resolves without hiding the sourced ROS packages --
``ROS_DOMAIN_ID=80``, ``FASTDDS_BUILTIN_TRANSPORTS=SHM`` and
``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST``::

    python3 test/support/mock_motion_harness.py --port 8781

It prints the URL, the uploaded gains sha256 and the arms/mode to start, then
runs until Ctrl-C and tears the whole tree down on the way out.
"""

from collections import deque
import http.client
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from franka_msgs.msg import FrankaState
from franka_web import config, defaults, health
from franka_web.gains import ProfileStore
from franka_web.http_api import App, build_server
from franka_web.launcher import ChildProcess
from franka_web.lock import OperatorLock
from franka_web.logbus import LogBus
from franka_web.ros_bridge import FrankaWebBridge
from franka_web.server import _LOG_EVENTS_PER_TICK, _PRODUCTION_QUEUE_DEPTH
from franka_web.session import SessionSupervisor
from franka_web.sse import Broker
import jsonschema
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from support.fake_launcher import FakeChild, FakePreflightResult, FakeRecording
from support.mock_impedance_controller import ArmSlot, MockImpedanceController
import yaml

#: The only launch file this package may ever execute (plan section 0.1).
LAUNCH_FILE = 'fake_dual_state_only.launch.py'

#: The one jog-capable reviewed controller, and the mock's node name.
CONTROLLER_NAME = 'dual_arm_joint_impedance_controller'

#: Both arms of the dual fake stack, in slot order (slot 1 first).
ARM_IDS = ('panda1', 'panda2')

#: The Panda home pose, the pose the harness drives the mock hardware to.
#: Every joint is comfortably inside the reviewed policy limits, which the
#: mock's own all-zeros boot pose is not (joint 4 in [-3.0718, -0.0698]).
HOME_POSE = (
    0.0,
    -math.pi / 4.0,
    0.0,
    -3.0 * math.pi / 4.0,
    0.0,
    math.pi / 2.0,
    math.pi / 4.0,
)

#: Half-width of the generated fence at every joint but one.
WIDE_MARGIN = 0.35

#: One joint gets a deliberately tight fence so the clamp test needs two jog
#: presses rather than eleven. 0.05 rad is one and a half JOG_STEP_RAD steps.
TIGHT_JOINT_INDEX = 6
TIGHT_MARGIN = 0.05

#: The stock controller the harness uses to place the mock hardware.
POSE_SETTER_NAME = 'mock_pose_setter'
POSE_SETTER_TYPE = 'position_controllers/JointGroupPositionController'

#: The normative frame schema handed to Session C.
SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'state_frame_schema.json')

#: Cadence of the graph publishers this harness stands in for.
GRAPH_PUBLISH_HZ = 10.0

#: How many published frames are kept for the schema assertion.
FRAME_HISTORY = 4000

#: Substrings identifying a process a run of this harness may have created.
#: ``franka_joint_state_publisher`` is matched by ``joint_state_publisher``.
PROCESS_MARKERS = (
    'ros2 launch',
    'ros2_control_node',
    'robot_state_publisher',
    'joint_state_publisher',
    'controller_manager spawner',
)

#: RFC 5737 documentation addresses. The rig's argv seam is inert and its
#: only launch is the fake dual stack, so these never reach anything.
DOC_IP_1 = '192.0.2.11'
DOC_IP_2 = '192.0.2.12'

_LATEST_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)

_POLL_S = 0.05
_HEARTBEAT_INTERVAL_S = 2.0

# Test-only settling policy for a motionless GenericSystem pose. These values
# exist solely to exercise the complete activation-settling protocol; they are
# not robot limits and must never be copied into a production default. Given
# here in the SI the rig thinks in, and converted to the DEGREES the
# configuration file speaks by `_settling_document`.
_SYNTHETIC_SETTLING_RAD = {
    'drift_limit_rad': 0.02,
    'span_limit_rad': 0.001,
    'velocity_limit_rad_s': 0.05,
    'fence_margin_rad': 0.005,
    'stable_window_s': 1.0,
    'min_samples': 4,
    'timeout_s': 8.0,
}


# ----------------------------------------------------------------------
# /proc helpers (never pgrep: /proc/<pid>/comm truncates at 15 characters)
# ----------------------------------------------------------------------


def read_cmdline(pid):
    """Return a process's argv as one space-joined string, or None if it is gone."""
    try:
        with open('/proc/{}/cmdline'.format(pid), 'rb') as handle:
            raw = handle.read()
    except OSError:
        return None
    return raw.decode('utf-8', 'replace').replace('\x00', ' ').strip()


def read_domain_id(pid):
    """Return a process's ROS_DOMAIN_ID from /proc, or None when unreadable."""
    try:
        with open('/proc/{}/environ'.format(pid), 'rb') as handle:
            raw = handle.read()
    except OSError:
        return None
    for entry in raw.split(b'\x00'):
        if entry.startswith(b'ROS_DOMAIN_ID='):
            return entry.split(b'=', 1)[1].decode('utf-8', 'replace')
    return None


def read_parent_pid(pid):
    """Return a process's parent pid from ``/proc/<pid>/stat``, or None."""
    try:
        with open('/proc/{}/stat'.format(pid), 'rb') as handle:
            raw = handle.read()
    except OSError:
        return None
    # comm sits in parentheses and may itself contain spaces and parentheses,
    # so the fields are counted from the LAST ')' rather than split naively.
    try:
        fields = raw[raw.rindex(b')') + 1:].split()
        return int(fields[1])
    except (IndexError, ValueError):
        return None


def alive(pid):
    """Return whether ``/proc`` still has an entry for ``pid``."""
    return os.path.exists('/proc/{}'.format(pid))


def own_lineage():
    """Return this process's pid and every ancestor pid, so a scan can skip them."""
    lineage = set()
    pid = os.getpid()
    while pid and pid not in lineage:
        lineage.add(pid)
        pid = read_parent_pid(pid)
    return lineage


def scan_processes(lineage, domain_id):
    """
    Return ``{pid: cmdline}`` for every matching process on ``domain_id``.

    This process and its ancestors are skipped (a shell command line easily
    contains one of the markers), and so is any process whose readable
    ``ROS_DOMAIN_ID`` is somebody else's -- another workspace's stack on
    another domain is none of this rig's business.
    """
    found = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in lineage:
            continue
        line = read_cmdline(pid)
        if not line or not any(marker in line for marker in PROCESS_MARKERS):
            continue
        if read_domain_id(pid) not in (None, str(domain_id)):
            continue
        found[pid] = line
    return found


def describe(processes):
    """Render a ``{pid: cmdline}`` mapping as readable lines."""
    return '\n'.join(
        '  {} {}'.format(pid, (line or '')[:160])
        for pid, line in sorted(processes.items()))


def reap(pids):
    """SIGKILL every pid that is still alive; return the ones that needed it."""
    reaped = {}
    for pid, line in pids.items():
        if not alive(pid):
            continue
        reaped[pid] = line
        try:
            os.kill(pid, signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and any(alive(pid) for pid in reaped):
        time.sleep(0.2)
    return reaped


# ----------------------------------------------------------------------
# Schema, SSE and socket helpers
# ----------------------------------------------------------------------


def load_validator():
    """Build a validator for the normative state-frame schema."""
    with open(SCHEMA_PATH, encoding='utf-8') as handle:
        schema = json.load(handle)
    validator_class = jsonschema.validators.validator_for(schema)
    validator_class.check_schema(schema)
    return validator_class(schema)


def schema_errors(validator, frame):
    """Return every schema violation of ``frame`` as readable lines."""
    problems = sorted(validator.iter_errors(frame), key=str)
    return '\n'.join(
        '  {}: {}'.format(
            '/'.join(str(part) for part in error.absolute_path) or '<root>',
            error.message)
        for error in problems)


def parse_sse(payload):
    """
    Split a raw SSE byte stream into ``(event_name, data_object)`` pairs.

    Only the LAST block may be a partial frame (the read window closed
    mid-write); anything else that fails to parse is a real wire-format
    violation and is raised rather than skipped.
    """
    records = []
    blocks = payload.split(b'\n\n')
    for index, block in enumerate(blocks):
        name = None
        data = []
        for line in block.split(b'\n'):
            if line.startswith(b'event: '):
                name = line[len(b'event: '):].decode('utf-8')
            elif line.startswith(b'data: '):
                data.append(line[len(b'data: '):].decode('utf-8'))
        if name is None or not data:
            continue
        try:
            records.append((name, json.loads('\n'.join(data))))
        except ValueError:
            if index == len(blocks) - 1:
                continue
            raise
    return records


def collect_sse(port, window_s):
    """
    Read ``GET /api/state/stream`` with a raw socket for ``window_s`` seconds.

    A raw socket rather than ``http.client`` because the stream has neither a
    Content-Length nor a chunked encoding: it is bytes until the connection
    closes. Returns ``(head_text, records)``.
    """
    request = (
        'GET /api/state/stream HTTP/1.1\r\n'
        'Host: 127.0.0.1:{}\r\n'
        'Accept: text/event-stream\r\n'
        'Connection: close\r\n'
        '\r\n').format(port).encode('ascii')
    buffered = b''
    sock = socket.create_connection(('127.0.0.1', port), timeout=10.0)
    try:
        sock.sendall(request)
        sock.settimeout(0.5)
        deadline = time.monotonic() + window_s
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break
            buffered += chunk
    finally:
        sock.close()
    head, separator, body = buffered.partition(b'\r\n\r\n')
    if not separator:
        raise AssertionError('the SSE response never produced a complete header block')
    return head.decode('utf-8', 'replace'), parse_sse(body)


def free_port():
    """Bind port 0 on loopback, release it, and return the number the kernel chose."""
    sock = socket.socket()
    try:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]
    finally:
        sock.close()


def wait_until(predicate, timeout_s, poll_s=_POLL_S):
    """Poll ``predicate`` until it returns something truthy; None on timeout."""
    deadline = time.monotonic() + timeout_s
    while True:
        value = predicate()
        if value:
            return value
        if time.monotonic() >= deadline:
            return None
        time.sleep(poll_s)


# ----------------------------------------------------------------------
# The generated gains config
# ----------------------------------------------------------------------


def fence_around(pose=HOME_POSE, margins=None):
    """
    Return ``(lower, upper)`` fence arrays centred on ``pose``.

    ``margins`` defaults to :data:`WIDE_MARGIN` at every joint except
    :data:`TIGHT_JOINT_INDEX`, which gets :data:`TIGHT_MARGIN`.
    """
    if margins is None:
        margins = [WIDE_MARGIN] * defaults.JOINT_COUNT
        margins[TIGHT_JOINT_INDEX] = TIGHT_MARGIN
    lower = tuple(float(value) - float(margin) for value, margin in zip(pose, margins))
    upper = tuple(float(value) + float(margin) for value, margin in zip(pose, margins))
    return lower, upper


def uniform_fences(pose=HOME_POSE, margins=None):
    """Return the same :func:`fence_around` fence for both arms."""
    lower, upper = fence_around(pose, margins)
    return {arm_id: (lower, upper) for arm_id in ARM_IDS}


# ----------------------------------------------------------------------
# The two ROS pieces the harness owns
# ----------------------------------------------------------------------


class MotionE2EBridge(FrankaWebBridge):
    """
    ``FrankaWebBridge`` with the lifecycle answers a fake stack cannot give.

    These overrides are stated here rather than hidden in the harness, because
    each is a claim about what is real and what is not:

    :meth:`controller_states` and :meth:`query_controller_states`
        The mock impedance controller is an ordinary rclpy node, not a
        ``controller_manager`` controller, so it can never appear in
        ``list_controllers`` -- and the two ``franka_*_robot_state_broadcaster``
        plus two ``franka_*_robot_model_broadcaster`` instances cannot activate
        on mock hardware at all (they need Franka semantic interfaces, which
        ``mock_components/GenericSystem`` does not export; the layer-3 test
        pins exactly that failure for the impedance controller). Readiness
        (plan section 3.5) and fault rule F5 both ask this method whether the
        session's controller is active; for the MOCK the truthful answer is
        yes, so those five names are reported active. Everything else in the
        map is the live graph. :meth:`raw_controller_states` returns the
        unedited map, which is what the layer-3 test asserts against.

    :meth:`call_switch_activate` and :meth:`call_switch_deactivate`
        Scripted ``{'ok': True}`` with matching synthetic lifecycle changes.
        The recovery sequence deactivates then restores the session controller;
        on this stack those real controller-manager calls would operate on the
        REAL controller, which is the layer-3 test's subject, not this one's.
        Every call is recorded so the e2e can assert exact recovery ordering.

    ``impedance_gate``
        The mock impedance node exists for the whole life of the rig, but the
        REAL launch does not work that way: its ``spawner --switch-asap``
        activates the controller AFTER the stack is up, which is exactly what
        lets the server capture a pre-activation baseline. A permanently
        "active" mock would make every Motion session fail closed at the
        baseline step -- correctly, and uselessly. The gate is a callable the
        harness installs that answers "has the baseline been captured yet?",
        so the rig reproduces the real ordering rather than fighting it.
    """

    #: Controllers whose lifecycle the mock reports on the operator's behalf.
    SYNTHETIC_CONTROLLERS = (
        CONTROLLER_NAME,
        'franka_panda1_robot_state_broadcaster',
        'franka_panda2_robot_state_broadcaster',
        'franka_panda1_robot_model_broadcaster',
        'franka_panda2_robot_model_broadcaster',
    )

    def __init__(self):
        """Build the real bridge and the recorder for scripted switch calls."""
        super().__init__()
        self.switch_activate_calls = []
        self.switch_deactivate_calls = []
        self.impedance_gate = None
        self._synthetic_states = {
            name: 'active' for name in self.SYNTHETIC_CONTROLLERS}

    def _visible_states(self):
        """Return the synthetic map, hiding the impedance controller pre-baseline."""
        states = dict(self._synthetic_states)
        gate = self.impedance_gate
        if gate is not None and not gate():
            states.pop(CONTROLLER_NAME, None)
        return states

    def raw_controller_states(self):
        """Return the controller map exactly as the live graph reports it."""
        return FrankaWebBridge.controller_states(self)

    def controller_states(self):
        """Return the live map plus the controllers the mock stands in for."""
        states = FrankaWebBridge.controller_states(self)
        states.update(self._visible_states())
        return states

    def query_controller_states(self, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """Refresh the live map, then add the mock-owned synthetic controllers."""
        states = FrankaWebBridge.query_controller_states(self, timeout_s)
        if states is None:
            return None
        states.update(self._visible_states())
        return states

    def call_switch_activate(self, controllers, timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """Answer the section 7.3 re-activation step without touching the CM."""
        self.switch_activate_calls.append(tuple(controllers))
        for controller in controllers:
            if controller in self._synthetic_states:
                self._synthetic_states[controller] = 'active'
        return {'ok': True}

    def call_switch_deactivate(self, controllers,
                               timeout_s=defaults.SERVICE_CALL_TIMEOUT_S):
        """Model the recovery's fail-closed synthetic-controller deactivation."""
        self.switch_deactivate_calls.append(tuple(controllers))
        for controller in controllers:
            if controller in self._synthetic_states:
                self._synthetic_states[controller] = 'inactive'
        return {'ok': True}


class MotionE2ETools(Node):
    """
    The rest of the graph a fake dual stack does not bring: state and pose.

    Two publishers stand in for nodes the fake stack cannot run, and both
    publish only values that mean "healthy", so the fault rules they feed
    (F2/F3/F4) stay quiet unless a test deliberately fires one:

    * ``/franka_<arm>_robot_state_broadcaster/robot_state`` --
      ``franka_msgs/FrankaState`` with ``robot_mode = IDLE``, a success rate of
      1.0 and no errors. Real messages over real DDS into the bridge's real
      subscription, so the health projection and F2/F3/F4 run on wire data.
    * ``/<pose setter>/commands`` -- the position command that places the mock
      hardware (see the module docstring).

    The ``/franka/joint_states`` subscription is created only while a caller is
    waiting for a commanded pose to arrive and destroyed again afterwards: it
    is a 1 kHz stream, and a second permanent 1 kHz subscriber in this process
    would show up as jitter in the bridge's own 20 Hz jog timer, which is one
    of the things this rig measures.
    """

    def __init__(self, arm_ids=ARM_IDS, pose_setter=POSE_SETTER_NAME):
        """Create the stand-in publishers, the CM client and the joint slot."""
        super().__init__('franka_web_motion_e2e_tools')
        self._arm_ids = tuple(arm_ids)
        self._lock = threading.Lock()
        self._joint = None
        self._joint_subscription = None
        self._pose_command = self.create_publisher(
            Float64MultiArray, '/{}/commands'.format(pose_setter), 10)
        self._robot_state_publishers = {
            arm_id: self.create_publisher(
                FrankaState,
                '/franka_{}_robot_state_broadcaster/robot_state'.format(arm_id),
                _LATEST_QOS)
            for arm_id in self._arm_ids
        }
        self._robot_state_messages = {
            arm_id: self._healthy_robot_state() for arm_id in self._arm_ids}
        self._list_controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers')

    @staticmethod
    def _healthy_robot_state():
        """Build the one FrankaState this rig ever publishes: a healthy arm."""
        message = FrankaState()
        message.robot_mode = FrankaState.ROBOT_MODE_IDLE
        message.control_command_success_rate = 1.0
        # current_errors / last_motion_errors default to all-false, which is
        # what "no errors" means to fault rule F3 and to health.true_error_names.
        return message

    def publish_robot_states(self):
        """Publish one healthy FrankaState per arm (the broadcasters' stand-in)."""
        for arm_id, message in self._robot_state_messages.items():
            self._robot_state_publishers[arm_id].publish(message)

    def publish_pose_command(self, positions):
        """Command the pose-setter controller with 14 joint positions."""
        message = Float64MultiArray()
        message.data = [float(value) for value in positions]
        self._pose_command.publish(message)

    def open_joint_stream(self):
        """Subscribe to ``/franka/joint_states`` for the duration of one wait."""
        with self._lock:
            if self._joint_subscription is not None:
                return
            self._joint = None
        subscription = self.create_subscription(
            JointState, '/franka/joint_states', self._on_joint, _LATEST_QOS)
        with self._lock:
            self._joint_subscription = subscription

    def close_joint_stream(self):
        """Drop the temporary ``/franka/joint_states`` subscription."""
        with self._lock:
            subscription = self._joint_subscription
            self._joint_subscription = None
        if subscription is not None:
            self.destroy_subscription(subscription)

    def latest_joint_state(self):
        """Return the newest ``JointState`` seen while the stream was open."""
        with self._lock:
            return self._joint

    def _on_joint(self, message):
        """Store the newest joint sample, replacing the previous one."""
        with self._lock:
            self._joint = message

    def controller_states(self, timeout_s=15.0):
        """Return ``{controller: lifecycle}`` straight from ``list_controllers``."""
        if not self._list_controllers.wait_for_service(timeout_sec=timeout_s):
            return None
        done = threading.Event()
        future = self._list_controllers.call_async(ListControllers.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            future.cancel()
            return None
        response = future.result()
        if response is None:
            return None
        return {entry.name: entry.state for entry in response.controller}


# ----------------------------------------------------------------------
# The harness
# ----------------------------------------------------------------------


class MockMotionHarness:
    """
    One assembled motion rig: fake stack, mock controller, real server.

    Construct with a private ``root`` directory, call :meth:`start`, drive it
    over HTTP, then call :meth:`close`. :meth:`close` is idempotent and is
    safe to call after a failed :meth:`start`.
    """

    def __init__(self, root, domain_id=None, port=None, bind='127.0.0.1',
                 fences=None):
        """Record the rig's parameters; nothing is created until :meth:`start`."""
        self.root = str(root)
        self.domain_id = int(
            domain_id if domain_id is not None else os.environ.get('ROS_DOMAIN_ID', '0'))
        self.bind = bind
        self.port = int(port) if port else free_port()
        self.fences = fences or uniform_fences()

        self.state_dir = os.path.join(self.root, 'state')
        self.recording_root = os.path.join(self.root, 'recordings')
        self.ros_home = os.path.join(self.root, 'ros_home')
        self.ros_log_dir = os.path.join(self.root, 'ros_log')
        self.launch_log_path = os.path.join(self.root, 'launch.log')
        self.pose_setter_path = os.path.join(self.root, 'pose_setter.yaml')

        self.settings = None
        self.launch = None
        self.bridge = None
        self.mock = None
        self.tools = None
        self.lock = None
        self.broker = None
        self.profile_store = None
        self.logs = None
        self.supervisor = None
        self.httpd = None
        self.token = None
        self.claim_id = None
        self.config_path = None
        self.frames = deque(maxlen=FRAME_HISTORY)
        self.tick_errors = []
        self.spawned = []

        self._shutdown = threading.Event()
        self._threads = []
        self._executors = []
        self._nodes = []
        self._closed = False
        self._diagnostics = {
            arm_id: (0, 'franka_web motion e2e: nominal') for arm_id in ARM_IDS}
        self._diagnostics_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Bring-up
    # ------------------------------------------------------------------

    def start(self, timeout_s=120.0):
        """Bring the whole rig up to "server listening, stack placed"."""
        self._make_directories()
        self.config_path = self._write_config_file()
        self.settings = config.load(
            self.config_path, environ={'HOME': self.root}, make_dirs=True)
        # The file speaks DEGREES, so a radian fence does not survive the
        # round trip bit-for-bit. Adopt the LOADED bounds as the rig's own,
        # so the mock's slots, the jog model's clamp and every assertion
        # here compare the same doubles -- which is exactly what the
        # controller's own `accept` does on the wire.
        self.fences = {
            arm_id: (tuple(self.settings.profile(arm_id).position_lower_rad),
                     tuple(self.settings.profile(arm_id).position_upper_rad))
            for arm_id in ARM_IDS}
        self._spawn_launch()
        self._start_ros()
        self._wait_for_stack(timeout_s)
        self._place_mock_hardware()
        self._build_server_stack()
        self._start_threads()
        self._wait_until_listening()
        return self

    def _make_directories(self):
        """Create the rig's private directories with the modes config requires."""
        for path in (self.root, self.state_dir, self.recording_root,
                     self.ros_home, self.ros_log_dir):
            os.makedirs(path, mode=0o700, exist_ok=True)
            os.chmod(path, 0o700)

    def _write_config_file(self):
        """
        Write this rig's own ``config.yaml`` and return its path.

        The rig's fence is installed through ``fence.<arm>`` -- v2's only
        route to bounds tighter than the factory limits, and the reason those
        optional keys are load-bearing for this suite.
        """
        settling = {}
        for key, value in _SYNTHETIC_SETTLING_RAD.items():
            if key in ('stable_window_s', 'timeout_s', 'min_samples'):
                settling[key] = value
            elif key.endswith('_rad_s'):
                settling[key[:-len('_rad_s')] + '_deg_s'] = math.degrees(value)
            else:
                settling[key[:-len('_rad')] + '_deg'] = math.degrees(value)
        document = {
            'bind': self.bind,
            'port': self.port,
            'ros_domain_id': self.domain_id,
            'robots': {'panda1': {'ip': DOC_IP_1}, 'panda2': {'ip': DOC_IP_2}},
            'directories': {'state': self.state_dir,
                            'recordings': self.recording_root},
            'settling': settling,
            'fence': {
                arm_id: {
                    'enabled': True,
                    'lower_deg': [math.degrees(value) for value in lower],
                    'upper_deg': [math.degrees(value) for value in upper],
                }
                for arm_id, (lower, upper) in self.fences.items()},
        }
        path = os.path.join(self.root, 'config.yaml')
        with open(path, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(document, handle, default_flow_style=False,
                           sort_keys=True)
        return path

    def child_environment(self):
        """Build the environment every child process of this rig inherits."""
        environment = dict(os.environ)
        environment['ROS_DOMAIN_ID'] = str(self.domain_id)
        environment['ROS_HOME'] = self.ros_home
        environment['ROS_LOG_DIR'] = self.ros_log_dir
        environment['PYTHONUNBUFFERED'] = '1'
        return environment

    def _spawn_launch(self):
        """Start the one permitted launch through the shipped supervisor."""
        argv = ('ros2', 'launch', 'franka_bringup', LAUNCH_FILE, 'use_rviz:=false')
        # SIGINT, not SIGTERM: `ros2 launch` tears its whole tree down only on
        # SIGINT (launcher.py, finding D-E2E-1 of the Stage 1 e2e).
        self.launch = ChildProcess.spawn(
            argv, self.child_environment(), 'launch',
            parent_death_signal=signal.SIGINT)

    def _start_ros(self):
        """Create the three nodes and spin each on its own executor thread."""
        if not rclpy.ok():
            rclpy.init()
        self.bridge = MotionE2EBridge()
        self.tools = MotionE2ETools()
        slots = [
            ArmSlot(arm_id, self.fences[arm_id][0], self.fences[arm_id][1])
            for arm_id in ARM_IDS
        ]
        self.mock = MockImpedanceController(slots, measured_provider=self._measured)
        # Three executors, three reasons: the bridge gets the server's own
        # single-threaded executor (same shape as server.py, so the 20 Hz jog
        # timer competes with the 1 kHz joint stream exactly as it does in
        # production); the mock gets its own, because the real controller is a
        # separate process and its receive timestamps are what this rig
        # measures; the tools node gets its own so its bursts never land in
        # either of those two.
        for node in (self.bridge, self.mock, self.tools):
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            thread = threading.Thread(
                target=executor.spin, name='ros-' + node.get_name(), daemon=True)
            self._executors.append(executor)
            self._nodes.append(node)
            self._threads.append(thread)
            thread.start()

    def _measured(self, arm_id):
        """Return the arm's measured pose for the mock's own bookkeeping."""
        sample = self.bridge.joint_sample() if self.bridge is not None else None
        if sample is not None:
            joints = health.extract_joints(arm_id, sample[1])
            if joints['complete']:
                return tuple(joints['positions'])
        return HOME_POSE

    def _wait_for_stack(self, timeout_s):
        """Wait until the fake stack's joint_state_broadcaster is active."""
        def ready():
            if not self.launch.alive():
                return False
            states = self.tools.controller_states(timeout_s=5.0)
            return bool(states and states.get('joint_state_broadcaster') == 'active')
        if wait_until(ready, timeout_s, poll_s=0.5) is None:
            raise AssertionError(
                'the fake dual stack never activated joint_state_broadcaster '
                'within {:.0f} s; launch output tail:\n{}'.format(
                    timeout_s, '\n'.join(self.launch.output_tail(30))))

    # ------------------------------------------------------------------
    # Placing the mock hardware (see the module docstring for why)
    # ------------------------------------------------------------------

    def _place_mock_hardware(self, timeout_s=90.0):
        """Load the pose-setter controller and drive both arms to the home pose."""
        self._write_pose_setter_config()
        result = subprocess.run(
            ['ros2', 'run', 'controller_manager', 'spawner', POSE_SETTER_NAME,
             '--param-file', self.pose_setter_path,
             '--controller-manager', '/controller_manager',
             '--controller-manager-timeout', '30'],
            env=self.child_environment(), cwd=self.root,
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, timeout=timeout_s, check=False)
        if result.returncode != 0:
            raise AssertionError(
                'the pose-setter controller could not be spawned (rc {}):\n{}'.format(
                    result.returncode, result.stdout))
        self.set_mock_pose({arm_id: HOME_POSE for arm_id in ARM_IDS})

    def _write_pose_setter_config(self):
        """Write the pose setter's parameter file (type plus its 14 joints)."""
        names = [
            '{}_joint{}'.format(arm_id, index)
            for arm_id in ARM_IDS
            for index in range(1, defaults.JOINT_COUNT + 1)
        ]
        lines = [
            '/{}:'.format(POSE_SETTER_NAME),
            '  ros__parameters:',
            '    type: {}'.format(POSE_SETTER_TYPE),
            '    joints:',
        ]
        lines.extend('      - {}'.format(name) for name in names)
        with open(self.pose_setter_path, 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(lines) + '\n')

    def set_mock_pose(self, poses, timeout_s=30.0, tolerance=1e-6):
        """
        Command the mock hardware to ``poses`` and wait until it reports them.

        ``poses`` maps arm id to seven positions. The command is re-published
        on every poll so a publisher that has not finished matching the
        controller's subscription cannot lose the only copy.
        """
        flat = [float(value) for arm_id in ARM_IDS for value in poses[arm_id]]
        self.tools.open_joint_stream()
        try:
            def arrived():
                self.tools.publish_pose_command(flat)
                message = self.tools.latest_joint_state()
                if message is None:
                    return False
                for arm_id in ARM_IDS:
                    joints = health.extract_joints(arm_id, message)
                    if not joints['complete']:
                        return False
                    for measured, wanted in zip(joints['positions'], poses[arm_id]):
                        if abs(measured - float(wanted)) > tolerance:
                            return False
                return True
            if wait_until(arrived, timeout_s, poll_s=0.1) is None:
                raise AssertionError(
                    'the mock hardware did not reach the commanded pose within '
                    '{:.0f} s; last sample: {}'.format(
                        timeout_s, self.tools.latest_joint_state()))
        finally:
            self.tools.close_joint_stream()

    # ------------------------------------------------------------------
    # The real server stack
    # ------------------------------------------------------------------

    def _build_server_stack(self):
        """Wire the shipped server objects, with the four documented seams."""
        self.lock = OperatorLock()
        # The PRODUCTION depth, not the constructor's bare default: a rig at
        # depth 4 would not exercise what the server actually runs.
        self.broker = Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)
        self.logs = LogBus()
        self.profile_store = ProfileStore(self.settings.state_dir)
        self.supervisor = SessionSupervisor(
            self.settings, self.bridge, self.lock, self.broker,
            spawn=self._fake_spawn,
            recording_factory=FakeRecording,
            preflight_runner=self._scripted_preflight,
            argv_builder=self._inert_argv,
            profile_store=self.profile_store,
            log_bus=self.logs)
        self.bridge.set_jog_callback(self.supervisor.jog_stream_tick)
        # Reproduce the launch's own ordering: the impedance controller
        # becomes visible only once the pre-activation baseline is captured,
        # which is what `spawner --switch-asap` does on the real stack.
        self.bridge.impedance_gate = self._baseline_captured
        static_root = os.path.join(
            get_package_share_directory('franka_web'), 'static')
        app = App(settings=self.settings, supervisor=self.supervisor,
                  lock=self.lock, broker=self.broker, static_root=static_root,
                  profile_store=self.profile_store, log_bus=self.logs)
        self.httpd = build_server(app)
        self.httpd.daemon_threads = True

    def _baseline_captured(self):
        """Return whether the session has captured its pre-activation baseline."""
        supervisor = self.supervisor
        if supervisor is None:
            return False
        if getattr(supervisor, '_baseline_captured', False):
            return True
        # A recovery re-activates the controller explicitly, so once a session
        # has been running the gate stays open for the rest of its life.
        return supervisor.state in ('settling', 'running', 'fault')

    def _fake_spawn(self, argv, env, name, **options):
        """Answer the supervisor's spawn seam without starting anything."""
        self.spawned.append({'argv': tuple(argv), 'name': name,
                             'options': dict(options)})
        return FakeChild(name=name)

    @staticmethod
    def _scripted_preflight(settings, mode):
        """Return a passing RT preflight; the real one inspects the host kernel."""
        return FakePreflightResult(
            overall='PASS', passed=True, blocking=mode in ('watch', 'motion'))

    @staticmethod
    def _inert_argv(arms, mode, settings, *, controller_param_file=None):
        """
        Answer the argv seam with something that is not a launch command line.

        ``spawn`` is faked, so this is never executed. It deliberately is not
        a ``ros2 launch`` argv: the production guarded-motion launch that
        ``('both', 'motion')`` really maps to must never be constructible
        from this rig.
        """
        return ('franka-web-mock-motion-e2e', str(arms), str(mode),
                str(controller_param_file))

    def _start_threads(self):
        """Start the HTTP server, the supervisor loop and the two pumps."""
        for target, name in (
                (self.httpd.serve_forever, 'http'),
                (self._supervisor_loop, 'supervisor'),
                (self._frame_pump, 'pump'),
                (self._graph_loop, 'graph')):
            thread = threading.Thread(target=target, name=name, daemon=True)
            self._threads.append(thread)
            thread.start()

    def _supervisor_loop(self):
        """
        Run the shipped ``run_forever`` loop on a thread of its own.

        ``server.py`` runs this on the MAIN thread because ``PR_SET_PDEATHSIG``
        fires when the spawning THREAD dies. Here it is safe on a worker: the
        spawn seam is faked, so this loop never forks anything. The one real
        child of this rig -- the ``ros2 launch`` -- was spawned by
        :meth:`start` on the caller's thread and is stopped by :meth:`close`
        on the caller's thread.
        """
        self.supervisor.run_forever(self._shutdown)

    def _frame_pump(self):
        """Mirror ``server.py``'s pump: 5 Hz state frames and pings."""
        from franka_web.session import rfc3339
        interval = 1.0 / defaults.STATE_FRAME_HZ
        next_ping = time.monotonic()
        while not self._shutdown.is_set():
            try:
                # Mirror server.py exactly: log events first (debug filtered,
                # newest N only), then the state frame -- so `logs.last_seq`
                # is never ahead of the last published `log` event.
                pending = [line for line in self.logs.drain_pending()
                           if line.level != 'debug']
                for line in pending[-_LOG_EVENTS_PER_TICK:]:
                    self.broker.publish('log', line.event())
                frame = self.supervisor.frame()
                self.frames.append(frame)
                self.broker.publish('state', frame)
                now = time.monotonic()
                if now >= next_ping:
                    self.broker.publish(
                        'ping', {'schema_version': defaults.SCHEMA_VERSION,
                                 't': rfc3339()})
                    next_ping = now + defaults.SSE_PING_INTERVAL_S
            except Exception as error:            # noqa: BLE001 - see tick_errors
                self.tick_errors.append('frame pump: {}'.format(error))
            self._shutdown.wait(interval)

    def _graph_loop(self):
        """Publish the FrankaState and the diagnostics the fake stack cannot."""
        interval = 1.0 / GRAPH_PUBLISH_HZ
        while not self._shutdown.is_set():
            try:
                self.tools.publish_robot_states()
                with self._diagnostics_lock:
                    entries = dict(self._diagnostics)
                for arm_id, (level, message) in entries.items():
                    self.mock.publish_synthetic_diagnostic(
                        level, message, arm_ids=[arm_id])
            except Exception as error:            # noqa: BLE001 - see tick_errors
                # A node torn down underneath this thread during close() must
                # never turn teardown into an exception nobody catches.
                self.tick_errors.append('graph loop: {}'.format(error))
            self._shutdown.wait(interval)

    def set_diagnostic(self, arm_id, level, message):
        """Set the level the mock publishes for ``arm_id`` from now on."""
        with self._diagnostics_lock:
            self._diagnostics[arm_id] = (int(level), str(message))

    def _wait_until_listening(self, timeout_s=30.0):
        """Poll the listen port until the HTTP server accepts a connection."""
        def listening():
            try:
                socket.create_connection((self.bind, self.port), 0.5).close()
                return True
            except OSError:
                return False
        if wait_until(listening, timeout_s, poll_s=0.1) is None:
            raise AssertionError(
                'the harness HTTP server never listened on {}:{}'.format(
                    self.bind, self.port))

    # ------------------------------------------------------------------
    # HTTP surface
    # ------------------------------------------------------------------

    def request(self, method, path, body=None, raw=None, content_type=None,
                token=True, timeout_s=30.0):
        """
        Send one request and return ``(status, decoded_json)``.

        ``token=True`` sends the harness's current operator token, ``False``
        sends none, and a string sends exactly that token.
        """
        headers = {'Host': '127.0.0.1:{}'.format(self.port)}
        payload = raw
        if body is not None:
            payload = json.dumps(body).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        if content_type is not None:
            headers['Content-Type'] = content_type
        chosen = self.token if token is True else (token or None)
        if chosen:
            headers['X-Operator-Token'] = chosen
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=timeout_s)
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            body_bytes = response.read()
            status = response.status
        finally:
            connection.close()
        try:
            return status, json.loads(body_bytes.decode('utf-8'))
        except ValueError:
            raise AssertionError('{} {} answered non-JSON: {!r}'.format(
                method, path, body_bytes[:200])) from None

    def claim(self):
        """
        Claim the single-operator lock and remember the token.

        Settles for one frame-pump period afterwards: the pump forces every
        enable off whenever it observes an unheld lock, and an observation
        already in flight when the claim lands would otherwise be able to
        disable an arm the caller enables immediately after.
        """
        status, payload = self.request('POST', '/api/operator/claim', token=False)
        assert status == 200, 'claim answered {}: {}'.format(status, payload)
        self.token = payload['token']
        self.claim_id = payload['claim_id']
        time.sleep(2.0 / defaults.STATE_FRAME_HZ)
        return self.token

    def heartbeat(self):
        """Refresh the operator lock; returns the monotonic time of the refresh."""
        moment = time.monotonic()
        status, payload = self.request('POST', '/api/operator/heartbeat')
        assert status == 200, 'heartbeat answered {}: {}'.format(status, payload)
        return moment

    def release(self):
        """Release the operator lock; returns the monotonic time of the release."""
        status, payload = self.request('POST', '/api/operator/release')
        assert status == 200, 'release answered {}: {}'.format(status, payload)
        moment = time.monotonic()
        self.token = None
        return moment

    def state(self):
        """Return the current section 6.11 frame from ``GET /api/state``."""
        status, payload = self.request('GET', '/api/state', token=False)
        assert status == 200, 'GET /api/state answered {}: {}'.format(status, payload)
        return payload['state']

    def settle(self, seconds):
        """Wait ``seconds`` while keeping the operator lock alive; return elapsed."""
        started = time.monotonic()
        deadline = started + seconds
        next_beat = started
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_beat:
                self.heartbeat()
                next_beat = now + _HEARTBEAT_INTERVAL_S
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        return time.monotonic() - started

    def wait_for_session_state(self, wanted, timeout_s, heartbeat=True):
        """Poll ``GET /api/state`` until ``session.state`` is ``wanted``."""
        deadline = time.monotonic() + timeout_s
        seen = []
        while True:
            frame = self.state()
            current = frame['session']['state']
            if not seen or seen[-1] != current:
                seen.append(current)
            if current == wanted:
                return frame
            if time.monotonic() >= deadline:
                raise AssertionError(
                    'the session never reached {!r} within {:.0f} s; states seen: '
                    '{}\nlast error: {}'.format(
                        wanted, timeout_s, ' -> '.join(seen),
                        frame['session']['last_error']))
            if heartbeat and self.token:
                self.heartbeat()
            time.sleep(0.2)

    def wait_for_frame(self, predicate, timeout_s, description, heartbeat=True):
        """Poll ``GET /api/state`` until ``predicate(frame)`` holds."""
        deadline = time.monotonic() + timeout_s
        frame = None
        while True:
            frame = self.state()
            if predicate(frame):
                return frame
            if time.monotonic() >= deadline:
                raise AssertionError(
                    '{} did not hold within {:.0f} s; last frame:\n{}'.format(
                        description, timeout_s,
                        json.dumps(frame, sort_keys=True)[:2000]))
            if heartbeat and self.token:
                self.heartbeat()
            time.sleep(0.2)

    # ------------------------------------------------------------------
    # Session helpers
    # ------------------------------------------------------------------

    def start_session(self, arms='both', mode='simulate', expect=202):
        """POST ``/api/session/start`` and return ``(status, payload)``."""
        body = {'arms': arms, 'mode': mode}
        status, payload = self.request('POST', '/api/session/start', body=body)
        if expect is not None:
            assert status == expect, (
                'POST /api/session/start answered {} (expected {}): {}'.format(
                    status, expect, payload))
        return status, payload

    def stop_session(self, timeout_s=45.0):
        """POST ``/api/session/stop`` and wait for ``stopped``."""
        status, payload = self.request('POST', '/api/session/stop')
        assert status == 202, 'stop answered {}: {}'.format(status, payload)
        return self.wait_for_session_state('stopped', timeout_s)

    def begin_motion_session(self, timeout_s=90.0):
        """
        Start Motion and stop at its command-closed ``settling`` state.

        There is no attestation to arrange first: Motion is one go, and the
        pre-activation baseline is captured inside the session.
        """
        self.start_session(arms='both', mode='motion')
        return self.wait_for_session_state('settling', timeout_s)

    def publish_external_targets(self, arm_id, hz=20.0, seconds=3.0):
        """
        Publish JointTrajectory targets on one arm's topic, as an operator would.

        This is the "External" half of the source switch: the messages go out
        over the real graph, on the real topic, from a node the server knows
        nothing about, and the server's counting subscription is what has to
        see them.
        """
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        slot = self.slot(arm_id)
        topic = '/{}/arm_{}/joint_target'.format(CONTROLLER_NAME, slot)
        publisher = self.tools.create_publisher(JointTrajectory, topic, 50)
        names = health.joint_names_for(arm_id)
        measured = self._measured(arm_id) or HOME_POSE
        period = 1.0 / float(hz)
        deadline = time.monotonic() + float(seconds)
        published = 0
        try:
            while time.monotonic() < deadline:
                message = JointTrajectory()
                message.header.stamp = self.tools.get_clock().now().to_msg()
                message.joint_names = list(names)
                point = JointTrajectoryPoint()
                point.positions = [float(value) for value in measured]
                point.time_from_start.sec = 0
                point.time_from_start.nanosec = 0
                message.points = [point]
                publisher.publish(message)
                published += 1
                time.sleep(period)
        finally:
            self.tools.destroy_publisher(publisher)
        return published

    def materialized_profile_path(self):
        """
        Render and store the profile a Motion session would hand the launch.

        The rig's argv seam is inert, so nothing else in it ever produces a
        real parameter file; the layer-3 case needs one to hand the REAL
        controller.
        """
        profiles = {arm_id: self.settings.profile(arm_id) for arm_id in ARM_IDS}
        return self.profile_store.materialize('both', profiles).path

    def source(self, arm_id, source, expect=200):
        """POST ``/api/arm/{arm_id}/source`` and return ``(status, payload)``."""
        status, payload = self.request(
            'POST', '/api/arm/{}/source'.format(arm_id),
            body={'source': source}, token=self.token)
        if expect is not None:
            assert status == expect, (
                'source switch answered {} (expected {}): {}'.format(
                    status, expect, payload))
        return status, payload

    def hold_lock(self):
        """Claim the operator lock unless this rig already holds it."""
        if self.token is not None:
            status, _payload = self.request(
                'POST', '/api/operator/heartbeat', token=self.token)
            if status == 200:
                return self.token
        return self.claim()

    def takeover(self):
        """POST ``/api/operator/takeover`` and adopt the successor claim."""
        status, payload = self.request('POST', '/api/operator/takeover')
        assert status == 200, 'takeover answered {}: {}'.format(status, payload)
        self.token = payload['token']
        self.claim_id = payload['claim_id']
        return payload

    def settling_evidence(self):
        """Return test-only evidence from the current activation gate."""
        with self.supervisor._state_lock:
            gate = self.supervisor._activation_gate
            if gate is None:
                return None
            evidence = gate.frame()
            evidence.update({
                'barrier_ns': gate.barrier_ns,
                'stable_since_ns': gate.stable_since_ns,
                'last_sample_ns': gate.last_sample_ns,
            })
            return evidence

    def finish_motion_settling(self, timeout_s=90.0):
        """Wait for distinct stable samples to open the Motion command surface."""
        frame = self.wait_for_session_state('running', timeout_s)
        evidence = self.settling_evidence()
        if evidence is None or evidence['status'] != 'ready':
            raise AssertionError(
                'Motion reached running without ready settling evidence: {!r}'.format(
                    evidence))
        if (evidence['stable_samples'] < evidence['required_samples']
                or evidence['stable_for_s'] < evidence['required_stable_s']
                or evidence['stable_since_ns'] is None
                or evidence['last_sample_ns'] is None
                or not (evidence['barrier_ns'] < evidence['stable_since_ns']
                        < evidence['last_sample_ns'])):
            raise AssertionError(
                'Motion did not use distinct fresh stable samples: {!r}'.format(
                    evidence))
        return frame

    def start_motion_session(self, gains_sha256, timeout_s=90.0):
        """Start Motion, observe ``settling``, then wait for ``running``."""
        self.begin_motion_session(gains_sha256, timeout_s)
        return self.finish_motion_settling(timeout_s)

    def enable(self, arm_id, enabled, expect=200):
        """POST ``/api/arm/<arm>/enable`` and return ``(status, payload)``."""
        status, payload = self.request(
            'POST', '/api/arm/{}/enable'.format(arm_id), body={'enabled': bool(enabled)})
        if expect is not None:
            assert status == expect, (
                'enable({}, {}) answered {} (expected {}): {}'.format(
                    arm_id, enabled, status, expect, payload))
        return status, payload

    def jog(self, arm_id, joint_index, direction, expect=200):
        """POST ``/api/arm/<arm>/jog`` and return ``(status, payload)``."""
        status, payload = self.request(
            'POST', '/api/arm/{}/jog'.format(arm_id),
            body={'joint_index': int(joint_index), 'direction': int(direction)})
        if expect is not None:
            assert status == expect, (
                'jog({}, {}, {}) answered {} (expected {}): {}'.format(
                    arm_id, joint_index, direction, status, expect, payload))
        return status, payload

    def recover(self, expect=200):
        """POST ``/api/session/recover`` and return ``(status, payload)``."""
        status, payload = self.request('POST', '/api/session/recover')
        if expect is not None:
            assert status == expect, (
                'recover answered {} (expected {}): {}'.format(
                    status, expect, payload))
        return status, payload

    # ------------------------------------------------------------------
    # Mock-side observation
    # ------------------------------------------------------------------

    def slot(self, arm_id):
        """Return the mock's 1-based slot number for ``arm_id``."""
        return self.mock.slot_number(arm_id)

    def last_receive_s(self, arm_id):
        """Return the monotonic time of the mock's newest message, or None."""
        log = self.mock.received(self.slot(arm_id))
        return log[-1][0] / 1e9 if log else None

    def message_count(self, arm_id):
        """Return how many targets the mock has recorded for ``arm_id``."""
        return len(self.mock.received(self.slot(arm_id)))

    def buffered_target(self, arm_id):
        """Return the mock inbox's currently buffered target positions."""
        return self.mock.inbox(self.slot(arm_id)).buffered_target().positions

    def wait_for_targets(self, arm_id, count, timeout_s=5.0):
        """Wait until ``count`` more targets have reached the mock for ``arm_id``."""
        start = self.message_count(arm_id)
        return wait_until(
            lambda: self.message_count(arm_id) >= start + count, timeout_s, 0.01)

    # ------------------------------------------------------------------
    # Teardown
    # ------------------------------------------------------------------

    def close(self):
        """Tear the whole rig down; idempotent, and safe after a failed start."""
        if self._closed:
            return
        self._closed = True
        self._shutdown.set()
        if self.httpd is not None:
            try:
                self.httpd.shutdown()
            except Exception:                     # noqa: BLE001 - teardown only
                pass
        if self.broker is not None:
            self.broker.close_all()
        for thread in self._threads:
            if thread.name in ('supervisor', 'pump', 'graph'):
                thread.join(timeout=60.0)
        if self.launch is not None:
            try:
                if self.launch.alive():
                    self.launch.stop(defaults.STOP_SIGINT_WAIT_S,
                                     defaults.STOP_SIGTERM_WAIT_S,
                                     defaults.STOP_SIGKILL_WAIT_S)
            except Exception:                     # noqa: BLE001 - teardown only
                pass
            self._write_launch_log()
        for executor in self._executors:
            try:
                executor.shutdown(timeout_sec=2.0)
            except Exception:                     # noqa: BLE001 - teardown only
                pass
        for thread in self._threads:
            thread.join(timeout=10.0)
        for node in self._nodes:
            try:
                node.destroy_node()
            except Exception:                     # noqa: BLE001 - teardown only
                pass
        if self.httpd is not None:
            try:
                self.httpd.server_close()
            except Exception:                     # noqa: BLE001 - teardown only
                pass
        try:
            rclpy.try_shutdown()
        except Exception:                         # noqa: BLE001 - teardown only
            pass

    def _write_launch_log(self):
        """Save the launch child's output tail for post-mortems."""
        try:
            with open(self.launch_log_path, 'w', encoding='utf-8') as handle:
                handle.write('\n'.join(self.launch.output_tail(500)))
        except OSError:
            pass

    def launch_tail(self, limit=40):
        """Return the tail of the launch child's merged output."""
        if self.launch is None:
            return '<no launch child>'
        return '\n'.join(self.launch.output_tail(limit)) or '<no launch output>'


# ----------------------------------------------------------------------
# Manual run (the browser rig)
# ----------------------------------------------------------------------


def _announce(line):
    """Print one line of the manual rig's banner, unbuffered."""
    print(line, flush=True)


def main(argv=None):
    """Bring the rig up, print how to reach it, and run until interrupted."""
    import argparse
    import tempfile

    parser = argparse.ArgumentParser(
        prog='mock_motion_harness',
        description='Bring up the franka_web motion rig against a mock '
                    'impedance controller on the fake dual stack.')
    parser.add_argument('--port', type=int, default=None,
                        help='listen port (default: a free one)')
    parser.add_argument('--root', default=None,
                        help='private working directory (default: a temp dir)')
    arguments = parser.parse_args(argv)

    root = arguments.root or tempfile.mkdtemp(prefix='franka-web-motion-rig-')
    harness = MockMotionHarness(root, port=arguments.port)
    finished = threading.Event()
    try:
        harness.start()
        # ``rclpy.init`` (inside start()) claims SIGINT and SIGTERM for itself:
        # SIGINT still chains through to KeyboardInterrupt, but SIGTERM only
        # shuts the rclpy context down and leaves this process -- and the whole
        # robot stack under it -- running. Take both signals back now that
        # rclpy has had its turn, so `kill` and Ctrl-C both reach close().
        signal.signal(signal.SIGINT, lambda *_a: finished.set())
        signal.signal(signal.SIGTERM, lambda *_a: finished.set())
        # The gains upload is a mutating route, so it needs the operator lock;
        # the lock is handed straight back so the browser can claim it.
        _announce('franka_web motion rig on http://{}:{} (domain {})'.format(
            harness.bind, harness.port, harness.domain_id))
        _announce('  working directory : {}'.format(root))
        _announce('  configuration     : {}'.format(harness.config_path))
        _announce('  controller        : {}'.format(CONTROLLER_NAME))
        _announce('  start with        : arms=both mode=motion (Motion is one '
                  'go; the profile comes from the configuration above)')
        _announce('  {}'.format(defaults.STOP_ADVISORY))
        _announce('Ctrl-C (or SIGTERM) to tear everything down.')
        finished.wait()
    except KeyboardInterrupt:
        pass
    finally:
        _announce('tearing the rig down...')
        harness.close()
        _announce('the rig is down; nothing was left running.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
