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
The one ROS node of this package: a per-arm Robotiq 2F-85 driver.

This is the only module here that imports ROS. It owns no protocol, no
framing, no checksum and no unit arithmetic: every byte-level fact comes from
``franka_robotiq.protocol`` and ``franka_robotiq.driver``, every millimetre
conversion from ``franka_robotiq.units``, every device-name decision from
``franka_robotiq.discovery``, and the fault table from
``franka_robotiq.registers``.

UNITS, AND ONE DELIBERATE DIVERGENCE FROM franka_gripper
    ``~/gripper_action`` is ``control_msgs/action/GripperCommand``.
    ``goal.command.position``, ``feedback.position`` and ``result.position``
    are **half-width in metres throughout** -- one finger's opening, ``0.0``
    fully closed and ``0.0425`` fully open.

    ``franka_gripper`` reads its *goal* as half-width but reports *feedback
    and result* as FULL width. That asymmetry is not repeated here: a
    goal -> result round trip is consistent in this package and is not in
    ``franka_gripper``. ``franka_robotiq/doc/UNITS.md`` states this in
    a box.

    ``goal.command.max_effort`` is newtons of grip force; ``0.0`` or a
    negative value means "use the configured ``force_n``", which is what a
    bare ``ros2 action send_goal`` with an unset field sends.
    ``result.effort`` and ``feedback.effort`` are the **commanded** force.
    The gripper does not measure fingertip force.

ONE SERIAL LINK, ONE POLL LOOP
    A single ``poll_rate_hz`` timer owns the port. It applies at most one
    pending command (one write, and only when the command changed), reads
    status once, advances the goal state machine and publishes both topics
    from that one snapshot, so the two topics can never disagree. Service and
    action callbacks never touch the port: they queue an intent and wait on a
    ``threading.Event`` the timer sets.
"""

import math
import sys
import threading
import time

from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from franka_robotiq import discovery, driver, protocol, registers, units
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

#: The node's name is ``<arm_id>`` plus this suffix, and the launch files pass
#: the same spelling as the node's ``name=``.
NODE_NAME_SUFFIX = '_robotiq'

#: The cell's two arms. Used for one thing only: naming the OTHER arm in the
#: anti-swap refusal, which reads much better as "panda2's adapter" than as
#: "the other arm".
ARM_IDS = ('panda1', 'panda2')

DEFAULT_ARM_ID = 'panda1'

#: The joint names default; they deliberately do NOT collide with
#: franka_gripper's ``<arm_id>_finger_joint1/2``, so both nodes can run.
JOINT_NAME_TEMPLATE = ('{arm_id}_robotiq_finger_joint1',
                       '{arm_id}_robotiq_finger_joint2')

#: The thirteen ``~/status`` keys, always all present, always in this order.
#: An earlier revision of the plan counted twelve by folding ``speed_mm_s``
#: and ``force_n`` into one row; the key that fell out was ``current_ma``,
#: and it is load-bearing -- it is what makes the zeroed ``JointState.effort``
#: honest rather than a hole.
STATUS_KEYS = ('width_mm', 'requested_width_mm', 'object', 'activated',
               'moving', 'fault_code', 'fault_name', 'fault_class',
               'current_ma', 'speed_mm_s', 'force_n', 'port', 'link')

#: The closed object-detection set of the contract's section 3.6.
OBJECT_NONE = 'none'
OBJECT_OPENED = 'opened_on_object'
OBJECT_CLOSED = 'closed_on_object'
OBJECT_AT_POSITION = 'at_position'
OBJECT_UNKNOWN = 'unknown'

#: ``fault_class`` is a closed set of FOUR on the wire. ``registers.fault()``
#: returns ``klass='unknown'`` for a code outside the table -- an internal
#: value of that lookup, deliberately distinct so this node can tell
#: "unrecognised" from "recognised and benign". It is mapped to ``major``
#: here, at the composition site, because an unrecognised fault is not a safe
#: fault and the operator's correct action is the same as for a major one.
FAULT_CLASSES = ('none', 'priority', 'minor', 'major')
_INTERNAL_UNKNOWN_CLASS = 'unknown'

#: Only ``gSTA == 0x03`` is "activation completed".
_ACTIVATED = 0x03

#: Log a down link at most this often while it stays down.
_LINK_DOWN_LOG_INTERVAL_S = 30.0

#: Inclusive numeric bounds, one row per numeric parameter. Same bounds and
#: same sentence shape as the web config loader, so an operator meets one
#: style whichever door they came in.
_RANGES = {
    'speed_mm_s': (units.SPEED_RANGE_MM_S[0], units.SPEED_RANGE_MM_S[1], 'mm/s'),
    'force_n': (units.FORCE_RANGE_N[0], units.FORCE_RANGE_N[1], 'N'),
    'open_width_mm': (0.0, units.STROKE_MM, 'mm'),
    'close_width_mm': (0.0, units.STROKE_MM, 'mm'),
    'poll_rate_hz': (1.0, 100.0, 'Hz'),
    'motion_timeout_s': (0.5, 30.0, 's'),
    'activation_timeout_s': (1.0, 60.0, 's'),
    'reconnect_interval_s': (0.5, 30.0, 's'),
}

#: The parameters a running node accepts a set for.
RUNTIME_PARAMETERS = ('speed_mm_s', 'force_n', 'open_width_mm',
                      'close_width_mm', 'auto_activate')

#: Every other parameter, with the reason its value is fixed at startup. The
#: sentence names the parameter and says where it IS set.
_STARTUP_ONLY = {
    'poll_rate_hz': ('changing the poll rate on a live serial link would resize '
                     'the command window under a running goal'),
    'motion_timeout_s': ('the goal deadline is armed when a goal is accepted, so '
                         'a new value would not apply to the goal you are watching'),
    'activation_timeout_s': ('the activation bound is read when the activation '
                             'sequence starts'),
    'reconnect_interval_s': 'the reconnect cadence is armed when the link goes down',
    'joint_names': ('the published joint names are a contract with whatever is '
                    'consuming ~/joint_states'),
    'arm_id': 'the arm a node drives is its identity, not a setting',
    'use_fake': 'the port is opened once, at startup',
    'fake_object_mm': 'the fake gripper is seeded once, at startup',
    'by_id_root': 'the device-name roots are read once, at startup',
    'by_path_root': 'the device-name roots are read once, at startup',
    'sysfs_root': 'the device-name roots are read once, at startup',
}

#: serial_id and usb_path get their own, stronger refusal.
_BINDING_PARAMETERS = ('serial_id', 'usb_path')

_BINDING_REFUSAL = (
    '{} cannot be changed on a running node. Re-binding a live gripper to a '
    'different adapter through a parameter set is exactly the wrong-arm '
    'command the binding rule exists to prevent. Stop the node, change the '
    'launch argument, and start it again. See '
    'franka_robotiq/doc/SERIAL_BINDING.md.')

_STARTUP_REFUSAL = (
    '{} is read once at startup. Set it in the launch file or the parameters '
    'file and restart the node; {}.')


def node_name_for(arm_id):
    """Return the node name one arm's driver runs under."""
    return '{}{}'.format(arm_id, NODE_NAME_SUFFIX)


def other_arm(arm_id):
    """Return the cell's OTHER arm id, or a neutral phrase for a strange one."""
    for candidate in ARM_IDS:
        if candidate != arm_id:
            return candidate
    return 'the other arm'


def range_refusal(name, value, minimum, maximum, unit):
    """Return the one refusal sentence a numeric parameter out of range gets."""
    return 'expected a value in {:g} <= x <= {:g} {}, found {:g}.'.format(
        minimum, maximum, unit, value)


def arm_id_from_ros_args(argv):
    """
    Return the ``arm_id`` given as ``-p arm_id:=<value>`` on a command line.

    The launch files pass the node's name as well as this parameter, so this
    only matters for a hand-run ``ros2 run franka_robotiq robotiq_node
    --ros-args -p arm_id:=panda2``, where nothing else could tell the node
    what to call itself before ``rclpy`` has built it.
    """
    argv = list(argv or [])
    wanted = ('-p', '--param')
    for index, entry in enumerate(argv):
        if entry in wanted and index + 1 < len(argv):
            assignment = argv[index + 1]
        elif entry.startswith('-p') and ':=' in entry and entry not in wanted:
            assignment = entry[len('-p'):]
        else:
            continue
        name, separator, value = assignment.partition(':=')
        if separator and name.strip() == 'arm_id' and value.strip():
            return value.strip()
    return None


class _Request:
    """One queued serial intent with a synchronous reply slot."""

    def __init__(self, kind, payload=None):
        """Store the intent and arm its unset reply."""
        self.kind = kind
        self.payload = payload
        self.done = threading.Event()
        self.success = False
        self.message = ''

    def answer(self, success, message):
        """Answer the waiting service callback exactly once."""
        self.success = bool(success)
        self.message = message
        self.done.set()


class _Goal:
    """The bookkeeping of the one goal a node may have in flight."""

    def __init__(self, goal_handle, count, half_width_m, effort_n, deadline_mono):
        """Record what was accepted, and how long it has to finish."""
        self.goal_handle = goal_handle
        self.count = count
        self.half_width_m = half_width_m
        self.effort_n = effort_n
        self.deadline_mono = deadline_mono
        self.done = threading.Event()
        self.outcome = None          # 'succeeded' | 'aborted' | 'canceled'
        self.reached_goal = False
        self.stalled = False
        self.message = ''
        self.written = False

    def finish(self, outcome, *, reached_goal=False, stalled=False, message=''):
        """Settle the goal exactly once and release its execute callback."""
        if self.outcome is not None:
            return
        self.outcome = outcome
        self.reached_goal = reached_goal
        self.stalled = stalled
        self.message = message
        self.done.set()


class RobotiqNode(Node):
    """One arm's Robotiq 2F-85 driver: the whole contract section 3 surface."""

    def __init__(self, *, node_name=None, **node_kwargs):
        """
        Build the node, resolve the port, and start the one poll timer.

        The node's name is ``<arm_id>_robotiq``. It arrives either as
        ``node_name`` (the launch files pass it, and so does ``main()`` after
        reading ``-p arm_id:=``) or from an ``arm_id`` in
        ``parameter_overrides`` (which is how a test builds one directly).
        """
        overrides = node_kwargs.get('parameter_overrides') or ()
        seed = DEFAULT_ARM_ID
        for override in overrides:
            if getattr(override, 'name', None) == 'arm_id' and override.value:
                seed = str(override.value)
        super().__init__(node_name or node_name_for(seed), **node_kwargs)

        self._declare_parameters(seed)
        self._arm_id = str(self.get_parameter('arm_id').value)
        self._serial_id = str(self.get_parameter('serial_id').value)
        self._usb_path = str(self.get_parameter('usb_path').value)
        self._by_id_root = str(self.get_parameter('by_id_root').value)
        self._by_path_root = str(self.get_parameter('by_path_root').value)
        self._sysfs_root = str(self.get_parameter('sysfs_root').value)
        self._use_fake = bool(self.get_parameter('use_fake').value)
        self._fake_object_mm = float(self.get_parameter('fake_object_mm').value)
        self._poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self._motion_timeout_s = float(self.get_parameter('motion_timeout_s').value)
        self._activation_timeout_s = float(
            self.get_parameter('activation_timeout_s').value)
        self._reconnect_interval_s = float(
            self.get_parameter('reconnect_interval_s').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._check_startup_parameters()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._port_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._pending = None
        self._inflight = None
        self._goal = None
        self._goal_admitted = False
        self._gripper = None
        self._fake = None
        self._port = ''
        self._link_up = False
        # Auto-activation is a RECONNECT rule, never a startup one, so it
        # needs to know whether this node ever had a working link. An adapter
        # that appears three seconds late is a slow startup, not a gripper
        # that lost power, and it must not be swept through a calibration
        # motion nobody asked for.
        self._ever_connected = False
        self._binding_refusal = None
        self._last_status = None
        self._last_written = None
        self._next_reconnect_mono = 0.0
        self._last_down_log_mono = None
        self._shutdown_done = False

        self._joint_publisher = self.create_publisher(
            JointState, '~/joint_states',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        # TRANSIENT_LOCAL so a late joiner -- the web server, `ros2 topic
        # echo`, an operator script starting mid-session -- is filled at once
        # instead of waiting a poll.
        self._status_publisher = self.create_publisher(
            DiagnosticStatus, '~/status',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._requests = ReentrantCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()
        for name in ('open', 'close', 'stop', 'reactivate'):
            self.create_service(Trigger, '~/' + name,
                                getattr(self, '_srv_' + name),
                                callback_group=self._requests)
        self._action_server = ActionServer(
            self, GripperCommand, '~/gripper_action',
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted,
            execute_callback=self._execute_goal,
            cancel_callback=self._cancel_callback,
            callback_group=self._requests)

        self._open_port()
        self._timer = self.create_timer(1.0 / self._poll_rate_hz, self._tick,
                                        callback_group=self._timer_group)

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self, seed_arm_id):
        """
        Declare every parameter of contract section 5.4, and three more.

        ``enabled`` is deliberately ABSENT: a node that is running is
        enabled, and a parameter whose only correct value is ``true`` is a
        fake knob. ``by_path_root`` and ``sysfs_root`` are the siblings of
        ``by_id_root`` -- test-only overrides, undocumented for operators,
        without which the by-path binding and the anti-swap re-verification
        could not be exercised at all. ``fake_object_mm`` is fake-mode only
        and is ignored when ``use_fake`` is false.
        """
        self.declare_parameter('arm_id', seed_arm_id)
        self.declare_parameter('serial_id', '')
        self.declare_parameter('usb_path', '')
        self.declare_parameter('by_id_root', discovery.DEFAULT_BY_ID_ROOT)
        self.declare_parameter('by_path_root', discovery.DEFAULT_BY_PATH_ROOT)
        self.declare_parameter('sysfs_root', '/sys')
        self.declare_parameter('use_fake', False)
        self.declare_parameter('fake_object_mm', 30.0)
        self.declare_parameter('speed_mm_s', 85.0)
        self.declare_parameter('force_n', 74.0)
        self.declare_parameter('open_width_mm', units.STROKE_MM)
        self.declare_parameter('close_width_mm', 0.0)
        self.declare_parameter('auto_activate', True)
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('motion_timeout_s', 5.0)
        self.declare_parameter('activation_timeout_s', 10.0)
        self.declare_parameter('reconnect_interval_s', 2.0)
        self.declare_parameter(
            'joint_names',
            [name.format(arm_id=seed_arm_id) for name in JOINT_NAME_TEMPLATE])

    def _check_startup_parameters(self):
        """Refuse to run on a parameter set the running node would refuse."""
        for name in _RANGES:
            problem = self._range_problem(name, float(self.get_parameter(name).value))
            if problem is not None:
                raise ValueError('{}: {}'.format(name, problem))
        problem = self._joint_names_problem(self._joint_names)
        if problem is not None:
            raise ValueError('joint_names: {}'.format(problem))
        problem = self._width_order_problem(
            float(self.get_parameter('open_width_mm').value),
            float(self.get_parameter('close_width_mm').value))
        if problem is not None:
            raise ValueError('close_width_mm: {}'.format(problem))

    @staticmethod
    def _range_problem(name, value):
        """Return the refusal sentence for an out-of-range number, or None."""
        minimum, maximum, unit = _RANGES[name]
        if not math.isfinite(value) or not minimum <= value <= maximum:
            return range_refusal(name, value, minimum, maximum, unit)
        return None

    @staticmethod
    def _joint_names_problem(names):
        """Return the refusal sentence for a bad joint-name list, or None."""
        names = list(names or [])
        if len(names) != len(JOINT_NAME_TEMPLATE):
            return ('expected exactly two joint names, one per finger, found '
                    '{}.'.format(len(names)))
        if any(not str(name).strip() for name in names):
            return 'expected two non-empty joint names; one of them is empty.'
        if names[0] == names[1]:
            return ('expected two DISTINCT joint names, found "{}" twice; a '
                    'JointState with a repeated name resolves to its first '
                    'entry and the second finger disappears.'.format(names[0]))
        return None

    @staticmethod
    def _width_order_problem(open_width_mm, close_width_mm):
        """Return the refusal sentence when close is not below open, or None."""
        if close_width_mm >= open_width_mm:
            return ('expected a value below open_width_mm ({:g} mm), found '
                    '{:g}.'.format(open_width_mm, close_width_mm))
        return None

    def _on_set_parameters(self, parameters):
        """Accept the five runtime parameters and teach about every other."""
        pending = {parameter.name: parameter.value for parameter in parameters}
        for parameter in parameters:
            name = parameter.name
            if name in _BINDING_PARAMETERS:
                return SetParametersResult(
                    successful=False, reason=_BINDING_REFUSAL.format(name))
            if name in _STARTUP_ONLY:
                return SetParametersResult(
                    successful=False,
                    reason=_STARTUP_REFUSAL.format(name, _STARTUP_ONLY[name]))
            if name in _RANGES:
                problem = self._range_problem(name, float(parameter.value))
                if problem is not None:
                    return SetParametersResult(
                        successful=False,
                        reason='{}: {}'.format(name, problem))
        open_width_mm = float(pending.get(
            'open_width_mm', self.get_parameter('open_width_mm').value))
        close_width_mm = float(pending.get(
            'close_width_mm', self.get_parameter('close_width_mm').value))
        problem = self._width_order_problem(open_width_mm, close_width_mm)
        if problem is not None:
            return SetParametersResult(
                successful=False, reason='close_width_mm: {}'.format(problem))
        return SetParametersResult(successful=True)

    @property
    def speed_mm_s(self):
        """Return the speed setting in force right now."""
        return float(self.get_parameter('speed_mm_s').value)

    @property
    def force_n(self):
        """Return the grip-force setting in force right now."""
        return float(self.get_parameter('force_n').value)

    @property
    def auto_activate(self):
        """Return whether a reconnect may re-activate a gripper that lost power."""
        return bool(self.get_parameter('auto_activate').value)

    # ------------------------------------------------------------------
    # Startup and the link
    # ------------------------------------------------------------------

    def _open_port(self):
        """
        Resolve the bound device and open it; a failure is not fatal.

        A node that exits on a missing adapter gives the operator nothing to
        read and nothing to watch, so every failure here leaves the node
        RUNNING, publishing ``link: down``, and logging once.
        """
        if self._use_fake:
            from franka_robotiq import fake
            self._fake = fake.FakeGripper()
            self._fake.start()
            self._port = self._fake.port
            # Fake mode only. A value at or below zero means "no object",
            # which is the documented way to remove one. The default 30 mm is
            # what makes `ros2 action send_goal ... position: 0.0` stop short
            # and report stalled: true -- the demonstration of the whole
            # device that the merge order asks for.
            self._fake.set_object(
                self._fake_object_mm if self._fake_object_mm > 0.0 else None)
            self._connect()
            return
        try:
            discovery.check_binding(self._arm_id, self._serial_id, self._usb_path)
        except discovery.BindingError as error:
            # A binding that cannot be right is never retried: retrying would
            # print the same sentence every reconnect_interval_s forever.
            self._binding_refusal = str(error)
            self.get_logger().error(self._binding_refusal)
            return
        self._connect()

    def _connect(self):
        """Resolve, open, re-verify the adapter identity, and read once."""
        try:
            path = self._resolve()
        except discovery.BindingError as error:
            # Section 4.3 rule 1: say what is NOT there, list what IS, and
            # never open the one that was found. The message belongs to
            # discovery.py, which is why it is asked for rather than composed.
            self.get_logger().debug('{}: {}'.format(self._arm_id, error))
            self._log_link_down(self._missing_adapter_sentence())
            return False
        gripper = driver.RobotiqGripper(path)
        try:
            gripper.open()
        except driver.LinkDownError as error:
            self._log_link_down('{}: {}'.format(self._arm_id, error))
            return False
        if not self._verify_identity(gripper, path):
            return False
        with self._port_lock:
            self._gripper = gripper
            self._port = path
            self._link_up = True
            self._last_written = None
        self._ever_connected = True
        return True

    def _resolve(self):
        """Return the resolved device path for whichever binding is set."""
        if self._use_fake:
            return self._port
        if self._serial_id:
            return discovery.resolve(self._serial_id, root=self._by_id_root)
        return discovery.resolve_by_path(self._usb_path, root=self._by_path_root)

    def _verify_identity(self, gripper, path):
        """
        Read the adapter's serial back from sysfs and refuse a mismatch.

        This is the belt-and-braces half of the anti-swap rule: the by-id
        symlink tree is built by udev and a stale or hand-made symlink can
        lie. It runs on EVERY (re)connect, before any write. When the check
        is unavailable -- a by-path binding, or an adapter that reports no
        serial -- the node says so in one line rather than pretending it
        verified something it did not.
        """
        if self._use_fake or not self._serial_id:
            self.get_logger().info(
                discovery.identity_unavailable_message(self._arm_id))
            return True
        try:
            reported = discovery.adapter_serial_from_sysfs(
                path, sysfs_root=self._sysfs_root)
        except OSError:
            reported = None
        if reported is None:
            self.get_logger().info(
                discovery.identity_unavailable_message(self._arm_id))
            return True
        if discovery.identity_matches(self._serial_id, reported):
            return True
        gripper.close()
        self.get_logger().error(discovery.identity_mismatch_message(
            self._arm_id, self._serial_id, reported, other_arm(self._arm_id)))
        return False

    def _log_link_down(self, sentence):
        """Log one link-down sentence, at most one per thirty seconds."""
        now = time.monotonic()
        if (self._last_down_log_mono is not None
                and now - self._last_down_log_mono < _LINK_DOWN_LOG_INTERVAL_S):
            return
        self._last_down_log_mono = now
        self.get_logger().error(sentence)

    def _missing_adapter_sentence(self):
        """Return the section 4.3 rule-1 refusal for the configured binding."""
        if self._serial_id:
            return discovery.missing_adapter_message(
                self._arm_id, self._serial_id, root=self._by_id_root,
                key='serial_id')
        return discovery.missing_adapter_message(
            self._arm_id, self._usb_path, root=self._by_path_root,
            key='usb_path')

    def _declare_link_down(self, error):
        """React to the driver's link-down verdict; the port is already shut."""
        with self._port_lock:
            self._link_up = False
            self._gripper = None
            self._last_written = None
        self._next_reconnect_mono = time.monotonic() + self._reconnect_interval_s
        self._last_down_log_mono = None
        self._log_link_down(
            'Lost the serial link to the {} gripper: {}. Retrying every '
            '{:g} s.'.format(self._arm_id, error, self._reconnect_interval_s))
        with self._goal_lock:
            goal = self._goal
        if goal is not None:
            goal.finish('aborted', message=(
                'Lost the serial link to the {} gripper while '
                'moving.'.format(self._arm_id)))
        self._answer_inflight(False, 'the serial link went down')
        self._publish(None)

    def _try_reconnect(self):
        """Retry resolve + open + read at the configured cadence."""
        if self._binding_refusal is not None:
            return
        now = time.monotonic()
        if now < self._next_reconnect_mono:
            return
        self._next_reconnect_mono = now + self._reconnect_interval_s
        # Read BEFORE the connect attempt: _connect() sets the flag, and an
        # adapter that only appears now is a slow startup rather than a
        # gripper that lost power.
        had_link = self._ever_connected
        if not self._connect():
            self._publish(None)
            return
        try:
            status = self._gripper.read_status()
        except (protocol.ProtocolError, driver.RobotiqError) as error:
            self._log_link_down('{}: {}'.format(self._arm_id, error))
            with self._port_lock:
                self._link_up = False
                self._gripper = None
            self._publish(None)
            return
        self._last_down_log_mono = None
        if status.g_sta == _ACTIVATED:
            self.get_logger().info(
                'Serial link to the {} gripper is back.'.format(self._arm_id))
        elif self.auto_activate and self._goal is None and had_link:
            # THE ONLY place auto_activate grants motion, and it needs a link
            # that was once up: activation re-calibrates the fingers, so it is
            # a motion, and it is never a startup side effect. A node that
            # starts against an un-activated gripper publishes the WARN of
            # section 3.4 and waits for a human to call ~/reactivate.
            if self._run_activation():
                self.get_logger().info(
                    'The {} gripper lost power and was '
                    're-activated.'.format(self._arm_id))
            status = self._last_status or status
        else:
            self.get_logger().warn(
                'The {} gripper is back but is not activated. Call /{}/reactivate '
                'to run the calibration.'.format(self._arm_id, self.get_name()))
        self._publish(status)

    def _run_activation(self):
        """Run the activation sequence; return whether it completed."""
        try:
            self._gripper.activate(timeout_s=self._activation_timeout_s)
        except driver.ActivationTimeout:
            return False
        except (protocol.ProtocolError, driver.RobotiqError):
            return False
        self._last_written = None
        return True

    # ------------------------------------------------------------------
    # The poll loop
    # ------------------------------------------------------------------

    def _tick(self):
        """Own the port for one cycle: write at most once, read once, publish."""
        if self._binding_refusal is not None:
            self._publish(None)
            return
        if not self._link_up:
            self._try_reconnect()
            return
        request = self._take_pending()
        self._inflight = request
        failure = None
        try:
            if request is not None:
                failure = self._apply(request)
            status = self._gripper.read_status()
        except (protocol.ProtocolError, driver.TransientReadError) as error:
            # Count NOTHING here: driver.FAILURE_LIMIT is the driver's and the
            # driver is already counting. Stay up, keep polling, say so only
            # at debug.
            self.get_logger().debug('{} gripper transaction failed: {}'.format(
                self._arm_id, error))
            self._answer_inflight(False, 'the gripper did not answer; retrying')
            return
        except driver.LinkDownError as error:
            # The FAILURE_LIMIT'th consecutive failure, or an open() that
            # could not open. The port is ALREADY closed and .connected is
            # already False; this is the single event that means "link down".
            self._declare_link_down(error)
            return
        self._answer_inflight(failure is None,
                              failure or self._applied_message(request))
        self._last_status = status
        self._advance_goal(status)
        self._publish(status)

    def _take_pending(self):
        """Take at most one queued intent, under the queue lock."""
        with self._pending_lock:
            request = self._pending
            self._pending = None
        return request

    def _answer_inflight(self, success, message):
        """Answer the intent this cycle carried, if any."""
        request = self._inflight
        self._inflight = None
        if request is not None:
            request.answer(success, message)

    def _applied_message(self, request):
        """Return the sentence a successfully applied intent answers with."""
        if request is None:
            return ''
        if request.kind == 'stop':
            return 'The {} gripper is stopped; the fingers hold where they ' \
                   'are.'.format(self._arm_id)
        if request.kind == 'reactivate':
            return 'The {} gripper is activated.'.format(self._arm_id)
        return 'Commanded the {} gripper to {:g} mm.'.format(
            self._arm_id, units.count_to_width_mm(request.payload[0]))

    def _apply(self, request):
        """
        Put one intent on the wire; write once, and only when it changed.

        Returns ``None`` when the intent was applied, or the refusal sentence
        the waiting caller should be answered with.
        """
        if request.kind == 'stop':
            self._gripper.stop()
            self._last_written = None
            return None
        if request.kind == 'reactivate':
            try:
                self._gripper.activate(timeout_s=self._activation_timeout_s)
            except driver.ActivationTimeout:
                # The driver's message names the key; the operator sentence
                # around it is this node's to compose.
                return ('The {} gripper did not finish activating within '
                        'activation_timeout_s ({:g} s). Check that the gripper '
                        'is powered and try /{}/reactivate again.'.format(
                            self._arm_id, self._activation_timeout_s,
                            self.get_name()))
            self._last_written = None
            return None
        command = request.payload
        if command != self._last_written:
            self._gripper.go_to(*command)
            self._last_written = command
        with self._goal_lock:
            goal = self._goal
        if goal is not None and goal.count == command[0]:
            # The request is on the wire whether this tick wrote it or a
            # previous one did; either way the goal may now read gOBJ.
            goal.written = True
        return None

    def _enqueue(self, request):
        """Queue one intent, replacing any older unapplied one."""
        with self._pending_lock:
            stale = self._pending
            self._pending = request
        if stale is not None:
            stale.answer(False, 'superseded by a newer command')

    def _command_for(self, width_mm, force_n=None):
        """Return the (position, speed, force) counts for one target width."""
        return (units.width_mm_to_count(width_mm),
                units.speed_mm_s_to_count(self.speed_mm_s),
                units.force_n_to_count(self.force_n if force_n is None else force_n))

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self, status):
        """Publish both topics from one snapshot, or the down shape."""
        if status is not None:
            # The half-width conversion lives in units.py and is CONSUMED
            # here, never re-derived: this file carries no millimetre
            # arithmetic at all, and a grep proves it.
            half_m = units.half_width_m_from_count(status.g_po)
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self._joint_names)
            message.position = [half_m, half_m]
            # velocity and effort are zero because the protocol reports
            # neither. gCU is a MOTOR CURRENT, not a fingertip force, and a
            # converted guess in an `effort` field is a fabricated number in a
            # field consumers trust. The real current is `current_ma` on
            # ~/status.
            message.velocity = [0.0, 0.0]
            message.effort = [0.0, 0.0]
            self._joint_publisher.publish(message)
        # While the link is down ~/joint_states is not published at all: a
        # stale width is worse than none. ~/status is published every tick.
        self._status_publisher.publish(self._status_message(status))

    def _status_message(self, status):
        """Compose the thirteen-key DiagnosticStatus of contract section 3.4."""
        values = self._status_values(status)
        level, sentence = self._level_and_message(status, values)
        message = DiagnosticStatus()
        message.name = '{} Robotiq 2F-85'.format(self._arm_id)
        # The anti-swap evidence, on the wire, where an operator can read it.
        message.hardware_id = self._port
        message.level = level
        message.message = sentence
        message.values = [KeyValue(key=key, value=values[key])
                          for key in STATUS_KEYS]
        return message

    def _status_values(self, status):
        """
        Return the thirteen values; a genuinely unknown one is the empty string.

        ``current_ma`` is published HERE and is deliberately absent from the
        web state frame: a motor current is a driver diagnostic rather than
        something a console row can act on. It is what makes the zeroed
        ``JointState.effort`` of section 3.3 an honest omission.
        """
        values = {key: '' for key in STATUS_KEYS}
        values['speed_mm_s'] = '{:g}'.format(self.speed_mm_s)
        values['force_n'] = '{:g}'.format(self.force_n)
        values['port'] = self._port
        values['link'] = 'up' if status is not None else 'down'
        if status is None:
            return values
        fault = registers.fault(status.g_flt)
        values['width_mm'] = '{:.1f}'.format(units.count_to_width_mm(status.g_po))
        values['requested_width_mm'] = '{:.1f}'.format(
            units.count_to_width_mm(status.g_pr))
        values['object'] = self._object_state(status)
        values['activated'] = 'true' if status.g_sta == _ACTIVATED else 'false'
        values['moving'] = 'true' if (status.g_gto == 1 and status.g_obj == 0) \
            else 'false'
        values['fault_code'] = '0x{:02X}'.format(status.g_flt)
        values['fault_name'] = fault.name
        values['fault_class'] = self._fault_class(fault)
        values['current_ma'] = '{:d}'.format(int(units.current_ma(status.g_cu)))
        return values

    @staticmethod
    def _fault_class(fault):
        """Map the lookup's class onto the four values the wire may carry."""
        if fault.klass == _INTERNAL_UNKNOWN_CLASS:
            # Contract section 3.4: an unrecognised fault is not a safe fault,
            # and the operator's correct action is the same as for a major
            # one. `unknown` never reaches ~/status, the frame or the schema.
            return 'major'
        return fault.klass if fault.klass in FAULT_CLASSES else 'major'

    @staticmethod
    def _object_state(status):
        """Return the section 3.6 object state; ``unknown`` whenever gGTO is 0."""
        if status.g_gto == 0:
            # gOBJ is MEANINGLESS while the gripper is not going, and is
            # reported as unknown, never as none.
            return OBJECT_UNKNOWN
        return {0x00: OBJECT_NONE,
                0x01: OBJECT_OPENED,
                0x02: OBJECT_CLOSED,
                0x03: OBJECT_AT_POSITION}.get(status.g_obj, OBJECT_UNKNOWN)

    def _level_and_message(self, status, values):
        """Evaluate the section 3.4 level table top to bottom, first match."""
        if status is None:
            if self._binding_refusal is not None:
                return DiagnosticStatus.ERROR, self._binding_refusal
            return DiagnosticStatus.ERROR, (
                'No serial link to the {} gripper. Check the USB cable; the '
                'driver retries every {:g} s.'.format(
                    self._arm_id, self._reconnect_interval_s))
        fault = registers.fault(status.g_flt)
        klass = self._fault_class(fault)
        if status.g_flt:
            if klass == 'major':
                headline = fault.meaning or 'Gripper fault {} ({})'.format(
                    values['fault_code'], fault.name)
                return DiagnosticStatus.ERROR, (
                    '{}. Call /{}/reactivate to reset the gripper.'.format(
                        headline, self.get_name()))
            if klass == 'minor':
                return DiagnosticStatus.WARN, (
                    'Gripper is too hot. It resumes by itself once it cools down.')
            return DiagnosticStatus.WARN, (
                'Gripper is not activated yet. Call /{}/reactivate.'.format(
                    self.get_name()))
        if status.g_sta != _ACTIVATED:
            return DiagnosticStatus.WARN, (
                'Gripper is not activated. Call /{}/reactivate to run the '
                'calibration.'.format(self.get_name()))
        width_mm = units.count_to_width_mm(status.g_po)
        if values['object'] in (OBJECT_CLOSED, OBJECT_OPENED):
            return DiagnosticStatus.OK, 'Holding an object at {:.1f} mm.'.format(
                width_mm)
        if width_mm <= units.WIDTH_EPSILON_MM:
            return DiagnosticStatus.OK, 'Closed.'
        return DiagnosticStatus.OK, 'Open {:.1f} mm.'.format(width_mm)

    # ------------------------------------------------------------------
    # The goal state machine
    # ------------------------------------------------------------------

    def _goal_callback(self, goal):
        """Admit or refuse one goal; pure, fast, and never touching the port."""
        with self._goal_lock:
            if self._goal is not None or self._goal_admitted:
                # NO PREEMPTION, deliberately: one serial link means one
                # physical motion, and silent preemption would let two callers
                # fight over one device with no trace.
                self.get_logger().warn(
                    'Another gripper goal is running; cancel it first.')
                return GoalResponse.REJECT
            # Gate on the METRES, before converting. units.py CLAMPS and
            # node.py GATES: a range check on the converted count can never
            # fire, because the conversion clamps into 0..COUNT_MAX -- so a
            # goal of 0.05 m would be ACCEPTED and drive the fingers fully
            # open. HALF_WIDTH_MAX_M is what keeps this file free of 0.0425,
            # of 85 and of any arithmetic.
            position = goal.command.position
            if (not math.isfinite(position)
                    or not 0.0 <= position <= units.HALF_WIDTH_MAX_M):
                self.get_logger().warn(
                    'Requested {:.4f} m per finger; the 2F-85 opens to {:.4f} m '
                    'per finger ({:.0f} mm total). Command refused.'.format(
                        position, units.HALF_WIDTH_MAX_M, units.STROKE_MM))
                return GoalResponse.REJECT
            self._goal_admitted = True
        return GoalResponse.ACCEPT_AND_EXECUTE

    def _handle_accepted(self, goal_handle):
        """Register the accepted goal, then hand it to the execute callback."""
        command = goal_handle.request.command
        half_width_m = float(command.position)
        count = units.count_from_half_width_m(half_width_m)
        effort_n = (self.force_n if float(command.max_effort) <= 0.0
                    else float(command.max_effort))
        goal = _Goal(goal_handle, count, half_width_m, effort_n,
                     time.monotonic() + self._motion_timeout_s)
        with self._goal_lock:
            self._goal = goal
        status = self._last_status
        if not self._link_up:
            goal.finish('aborted', message=(
                'Lost the serial link to the {} gripper while '
                'moving.'.format(self._arm_id)))
        elif status is not None and status.g_sta != _ACTIVATED:
            # Never auto-activate inside a goal: activation is a motion and
            # must not be a surprise side effect.
            goal.finish('aborted', message=(
                'The gripper is not activated. Call /{}/reactivate '
                'first.'.format(self.get_name())))
        elif status is not None and self._already_there(status, half_width_m):
            goal.finish('succeeded', reached_goal=True, stalled=False,
                        message='Already within {:g} mm of the target; nothing '
                                'was commanded.'.format(units.WIDTH_EPSILON_MM))
        else:
            self._enqueue(_Request('goto', self._command_for(
                units.count_to_width_mm(count), effort_n)))
        goal_handle.execute()

    @staticmethod
    def _already_there(status, half_width_m):
        """Return whether the fingers are inside the "already there" band."""
        current_mm = units.count_to_width_mm(status.g_po)
        target_mm = units.count_to_width_mm(
            units.count_from_half_width_m(half_width_m))
        return abs(current_mm - target_mm) < units.WIDTH_EPSILON_MM

    def _cancel_callback(self, goal_handle):
        """Accept every cancel: the fingers hold, and are never auto-opened."""
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        """Wait for the poll loop's verdict and answer the action client."""
        with self._goal_lock:
            goal = self._goal
        if goal is None or goal.goal_handle is not goal_handle:
            goal_handle.abort()
            return self._goal_result(None)
        goal.done.wait()
        with self._goal_lock:
            self._goal = None
            self._goal_admitted = False
        if goal.outcome == 'succeeded':
            goal_handle.succeed()
        elif goal.outcome == 'canceled' and goal_handle.is_cancel_requested:
            # canceled() is only legal once the action server has moved the
            # goal into CANCELING, which is exactly when a cancel was asked
            # for. Every other ending -- a stop service, a shutdown -- is an
            # abort that says so in its message.
            goal_handle.canceled()
        else:
            goal_handle.abort()
        if goal.message:
            level = (self.get_logger().info if goal.outcome == 'succeeded'
                     else self.get_logger().warn)
            level(goal.message)
        return self._goal_result(goal)

    def _goal_result(self, goal):
        """Build the result message, in half-width metres and commanded newtons."""
        result = GripperCommand.Result()
        status = self._last_status
        result.position = (units.half_width_m_from_count(status.g_po)
                           if status is not None else 0.0)
        result.effort = goal.effort_n if goal is not None else 0.0
        result.stalled = bool(goal.stalled) if goal is not None else False
        result.reached_goal = bool(goal.reached_goal) if goal is not None else False
        return result

    def _advance_goal(self, status):
        """Advance the one active goal against this tick's status snapshot."""
        with self._goal_lock:
            goal = self._goal
        if goal is None or goal.outcome is not None:
            return
        if goal.goal_handle.is_cancel_requested:
            self._enqueue(_Request('stop'))
            goal.finish('canceled', message=(
                'The {} gripper goal was cancelled; the fingers hold where '
                'they are.'.format(self._arm_id)))
            return
        if status.g_flt:
            fault = registers.fault(status.g_flt)
            goal.finish('aborted', message=(
                '{}. Call /{}/reactivate to reset the gripper.'.format(
                    fault.meaning or 'Gripper fault 0x{:02X}'.format(status.g_flt),
                    self.get_name())))
            return
        if status.g_sta != _ACTIVATED:
            goal.finish('aborted', message=(
                'The gripper is not activated. Call /{}/reactivate '
                'first.'.format(self.get_name())))
            return
        self._publish_feedback(goal, status)
        if not goal.written or status.g_pr != goal.count:
            # The gripper has not echoed the request yet, so gOBJ still
            # describes the PREVIOUS motion and must not be read as this
            # goal's outcome.
            if time.monotonic() > goal.deadline_mono:
                self._fail_on_timeout(goal)
            return
        if status.g_obj in (0x01, 0x02):
            # An object stopped the fingers short. That is SUCCESS, and it is
            # the whole point of the device.
            goal.finish('succeeded', reached_goal=True, stalled=True,
                        message='The {} gripper closed on an object at '
                                '{:.1f} mm.'.format(
                                    self._arm_id,
                                    units.count_to_width_mm(status.g_po)))
            return
        if status.g_obj == 0x03:
            goal.finish('succeeded', reached_goal=True, stalled=False,
                        message='The {} gripper reached {:.1f} mm.'.format(
                            self._arm_id, units.count_to_width_mm(status.g_po)))
            return
        if time.monotonic() > goal.deadline_mono:
            self._fail_on_timeout(goal)

    def _fail_on_timeout(self, goal):
        """Abort a goal that outlived motion_timeout_s, naming the key."""
        goal.finish('aborted', message=(
            'The {} gripper goal did not finish within motion_timeout_s '
            '({:g} s).'.format(self._arm_id, self._motion_timeout_s)))

    def _publish_feedback(self, goal, status):
        """Publish one feedback sample, in the same units as the goal."""
        feedback = GripperCommand.Feedback()
        feedback.position = units.half_width_m_from_count(status.g_po)
        feedback.effort = goal.effort_n
        feedback.stalled = status.g_gto == 1 and status.g_obj in (0x01, 0x02)
        feedback.reached_goal = status.g_gto == 1 and status.g_obj == 0x03
        goal.goal_handle.publish_feedback(feedback)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _wait_for(self, request, timeout_s):
        """Queue an intent and wait for the poll loop to apply it."""
        self._enqueue(request)
        if not request.done.wait(timeout_s):
            return False, ('the {} gripper driver did not apply the command '
                           'within {:g} s'.format(self._arm_id, timeout_s))
        return request.success, request.message

    def _apply_timeout_s(self):
        """Return how long a queued write may take to reach the wire."""
        return max(1.0, 5.0 / self._poll_rate_hz)

    def _motion_guard(self):
        """Return the refusal sentence for open/close, or None."""
        if self._binding_refusal is not None:
            return self._binding_refusal
        if not self._link_up:
            return 'No serial link to the {} gripper.'.format(self._arm_id)
        with self._goal_lock:
            busy = self._goal is not None or self._goal_admitted
        if busy:
            # Section 3.5's no-preemption rule exists so two callers cannot
            # fight over one serial link and one physical motion. A service
            # that silently overrode a running goal would be that same
            # collision through a different door.
            return 'Another gripper goal is running; cancel it first.'
        status = self._last_status
        if status is None:
            return 'No news from the {} gripper yet.'.format(self._arm_id)
        if status.g_sta != _ACTIVATED:
            return ('The gripper is not activated. Call /{}/reactivate '
                    'first.'.format(self.get_name()))
        if status.g_flt:
            fault = registers.fault(status.g_flt)
            return '{}. Call /{}/reactivate to reset the gripper.'.format(
                fault.meaning or 'Gripper fault 0x{:02X}'.format(status.g_flt),
                self.get_name())
        return None

    def _srv_open(self, request, response):
        """``~/open``: go to ``open_width_mm`` at the configured speed and force."""
        return self._go_to_configured_width('open_width_mm', response)

    def _srv_close(self, request, response):
        """``~/close``: go to ``close_width_mm`` at the configured speed and force."""
        return self._go_to_configured_width('close_width_mm', response)

    def _go_to_configured_width(self, parameter_name, response):
        """Command one of the two configured widths, or refuse with a sentence."""
        refusal = self._motion_guard()
        if refusal is not None:
            response.success = False
            response.message = refusal
            return response
        width_mm = float(self.get_parameter(parameter_name).value)
        response.success, response.message = self._wait_for(
            _Request('goto', self._command_for(width_mm)), self._apply_timeout_s())
        return response

    def _srv_stop(self, request, response):
        """``~/stop``: clear rGTO. Never refused -- stop must always be pressable."""
        with self._goal_lock:
            goal = self._goal
        if goal is not None:
            goal.finish('aborted', message=(
                'The {} gripper goal was stopped; the fingers hold where they '
                'are.'.format(self._arm_id)))
        if not self._link_up:
            response.success = False
            response.message = 'No serial link to the {} gripper.'.format(self._arm_id)
            return response
        response.success, response.message = self._wait_for(
            _Request('stop'), self._apply_timeout_s())
        return response

    def _srv_reactivate(self, request, response):
        """
        ``~/reactivate``: run the rACT cycle. The one service that takes seconds.

        Trigger.success IS the outcome, so this call returns only when the
        activation cycle finishes, bounded by ``activation_timeout_s``. A
        caller that must not block for that long -- the web server is one --
        dispatches it asynchronously and reads the result off ``~/status``.
        """
        if not self._link_up:
            response.success = False
            response.message = 'No serial link to the {} gripper.'.format(self._arm_id)
            return response
        with self._goal_lock:
            busy = self._goal is not None or self._goal_admitted
        if busy:
            response.success = False
            response.message = 'Another gripper goal is running; cancel it first.'
            return response
        success, message = self._wait_for(
            _Request('reactivate'), self._activation_timeout_s + self._apply_timeout_s())
        if not success and not message:
            message = ('The {} gripper did not finish activating within '
                       'activation_timeout_s ({:g} s).'.format(
                           self._arm_id, self._activation_timeout_s))
        response.success = success
        response.message = message
        return response

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self):
        """
        Cancel any goal, clear rGTO, release the port. Idempotent.

        The fingers hold where they are: the gripper is NOT opened on
        shutdown, and a held workpiece stays held.
        """
        if self._shutdown_done:
            return
        self._shutdown_done = True
        try:
            self._timer.cancel()
        except Exception:  # noqa: BLE001 - finish every teardown
            pass
        with self._goal_lock:
            goal = self._goal
        if goal is not None:
            goal.finish('aborted', message='The node is shutting down; the '
                                           'fingers hold where they are.')
        with self._port_lock:
            gripper = self._gripper
            self._gripper = None
            self._link_up = False
        if gripper is not None:
            try:
                gripper.stop()
            except Exception:  # noqa: BLE001 - finish every teardown
                pass
            try:
                gripper.close()
            except Exception:  # noqa: BLE001 - finish every teardown
                pass
        if self._fake is not None:
            try:
                self._fake.stop()
            except Exception:  # noqa: BLE001 - finish every teardown
                pass
            self._fake = None


def main(argv=None):
    """Run one gripper node until it is interrupted or signalled."""
    rclpy.init(args=argv)
    arm_id = arm_id_from_ros_args(argv if argv is not None else sys.argv)
    node = RobotiqNode(node_name=node_name_for(arm_id) if arm_id else None)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # `ros2 launch` teardown sends SIGINT and escalates to SIGTERM, and an
        # operator's `kill` or a systemd unit sends SIGTERM directly. The port
        # must be released on either.
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
