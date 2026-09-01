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
A port of the impedance controller's target inbox, and a mock controller node.

:class:`ArmImpedanceTargetInbox` is a line-by-line port of the C++
``franka_example_controllers::ArmImpedanceTargetInbox`` (``accept``,
``setEnabled``, ``readFresh``, ``invalidate``) in
``franka_example_controllers/src/dual_arm_joint_impedance_controller.cpp``.
Same rules, same order, same boundary operators: the fence rejects only
``position < lower`` or ``position > upper`` (so both bounds are inclusive),
the two header-age checks reject only strictly beyond their window, and
``read_fresh`` demands ``steady_receive_ns > enabled_since`` *and*
``header_ns > enabled_ros_epoch`` (both strict). Every rejection invalidates
the previously buffered target -- the watchdog-freeze hazard of plan section
10.4, not a dropped frame.

``ControllerInactive`` is deliberately not an inbox rule. The C++ returns it
from ``DualArmJointImpedanceControllerCore::acceptTarget`` *before* the inbox
is reached, so it leaves the buffered target intact; :func:`accept_target`
ports that gate, including the C++'s reuse of ``DuplicateOrUnknownJoint`` as
the answer for an out-of-range arm index. ``accept(..., controller_active=
False)`` is the same gate, offered on the inbox for callers that hold one.

:class:`MockImpedanceController` wires the port into an rclpy node that
impersonates the controller on a fake-hardware graph: node name
``dual_arm_joint_impedance_controller``, per slot ``~/arm_<n>/enable``
(``std_srvs/SetBool``, answering with the controller's own reply strings) and
``~/arm_<n>/joint_target`` (``trajectory_msgs/JointTrajectory``, ``QoS(1)``
reliable/volatile). Nothing here touches hardware and no value in this module
is a robot address. The node class exists for the Stage 2 e2e; the unit tests
import the port and never instantiate it.
"""

import enum
import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from franka_msgs.srv import ErrorRecovery
from franka_web import defaults
from franka_web.health import canonical_diagnostic_name
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

NANOSECONDS_PER_SECOND = 1000000000

#: The node name the controller has in both dual and one-arm mode (plan 0.6).
CONTROLLER_NODE_NAME = 'dual_arm_joint_impedance_controller'

#: ``future_tolerance`` is a controller parameter with no counterpart in
#: ``franka_web.config``; plan section 0.6 pins the reviewed value at 0.1 s.
#: It is NOT ``defaults.REVIEWED_TIMING_S['watchdog_timeout']`` -- the numbers coincide, the
#: meanings do not -- so it is spelled out here rather than borrowed.
DEFAULT_FUTURE_TOLERANCE_S = 0.1

# The controller's own SetBool reply strings (dual_arm_joint_impedance_
# controller.cpp, the enable-service lambda). The web layer passes the enabled
# one straight through to the operator (plan section 6.13), so a drift here
# would be a drift in what the UI says.
ENABLE_REJECTED_MESSAGE = 'rejected because the controller is not stably active'
ENABLE_ENABLED_MESSAGE = (
    'enabled; measured target retained while awaiting a fresh valid target')
ENABLE_DISABLED_MESSAGE = 'disabled; measured position sampled on the next update'

#: ``ErrorRecovery`` replies this, with ``success=False``, when there is no
#: fault to clear. Informational, never an error (plan sections 0.8, 10.5).
NO_ERRORS_MESSAGE = 'No errors'

#: ``rclcpp::QoS(1).reliable().durability_volatile()`` on the target topic.
TARGET_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)

#: What ``FrankaWebBridge`` subscribes ``/diagnostics`` with.
DIAGNOSTICS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)


class JointTargetValidationResult(enum.Enum):
    """
    The C++ ``JointTargetValidationResult`` enumerators, in declaration order.

    Sixteen members: ``Accepted``, ``ControllerInactive``, and the fourteen
    rejection reasons. ``test_jog_message_contract.py`` scrapes the enum out of
    ``dual_arm_joint_impedance_controller_core.hpp`` and pins this list against
    it, so a future change to the C++ enum fails this package's tests instead
    of silently drifting.
    """

    Accepted = enum.auto()
    ControllerInactive = enum.auto()
    InvalidFrame = enum.auto()
    InvalidStamp = enum.auto()
    HeaderTooOld = enum.auto()
    HeaderTooFarInFuture = enum.auto()
    InvalidNameCount = enum.auto()
    InvalidPointCount = enum.auto()
    InvalidPositionCount = enum.auto()
    VelocityCommandNotAllowed = enum.auto()
    AccelerationCommandNotAllowed = enum.auto()
    EffortCommandNotAllowed = enum.auto()
    InvalidTimeFromStart = enum.auto()
    DuplicateOrUnknownJoint = enum.auto()
    NonfinitePosition = enum.auto()
    PositionLimitExceeded = enum.auto()


#: The fourteen results that are rejections: every member except the two that
#: are not (``Accepted``, and the pre-inbox ``ControllerInactive`` gate).
REJECTION_RESULTS = tuple(
    result for result in JointTargetValidationResult
    if result not in (JointTargetValidationResult.Accepted,
                      JointTargetValidationResult.ControllerInactive))


def seconds_to_nanoseconds(seconds):
    """Convert a policy window in seconds to whole nanoseconds."""
    return int(round(float(seconds) * NANOSECONDS_PER_SECOND))


def canonical_joint_names(arm_id):
    """Return ``<arm_id>_joint1`` .. ``_joint7``, the only names accepted."""
    return tuple(
        '{}_joint{}'.format(arm_id, index + 1) for index in range(defaults.JOINT_COUNT))


class BufferedImpedanceTarget:
    """
    The inbox's single buffered target (the C++ ``BufferedImpedanceTarget``).

    A default-constructed value is exactly what ``invalidate()`` writes: not
    valid, no positions, only the steady timestamp of the event that dropped
    it.
    """

    __slots__ = ('positions', 'header_ns', 'steady_receive_ns', 'valid')

    def __init__(self, positions=None, header_ns=0, steady_receive_ns=0, valid=False):
        """Store one candidate target and the two timestamps it is judged by."""
        self.positions = (
            (0.0,) * defaults.JOINT_COUNT if positions is None else tuple(positions))
        self.header_ns = int(header_ns)
        self.steady_receive_ns = int(steady_receive_ns)
        self.valid = bool(valid)


class ArmImpedanceTargetInbox:
    """
    Port of ``ArmImpedanceTargetInbox``: validate, buffer, and gate on epochs.

    One instance models one arm slot of the controller. The C++ reaches this
    object from the ROS topic-callback thread and reads it from the control
    loop; the lock here stands in for its atomics and realtime buffer, so the
    mock node can be driven from an executor thread while a test reads it.
    """

    def __init__(self, joint_names, position_lower, position_upper,
                 max_header_age_s=defaults.REVIEWED_TIMING_S['max_header_age'],
                 future_tolerance_s=DEFAULT_FUTURE_TOLERANCE_S):
        """Configure the arm's joint names and fence (the C++ ``configure``)."""
        self._joint_names = tuple(str(name) for name in joint_names)
        self._position_lower = tuple(float(value) for value in position_lower)
        self._position_upper = tuple(float(value) for value in position_upper)
        if not (len(self._joint_names) == len(self._position_lower)
                == len(self._position_upper) == defaults.JOINT_COUNT):
            raise ValueError(
                'an impedance arm carries exactly {} joints, names and both '
                'fence bounds'.format(defaults.JOINT_COUNT))
        self._max_header_age_ns = seconds_to_nanoseconds(max_header_age_s)
        self._future_tolerance_ns = seconds_to_nanoseconds(future_tolerance_s)
        self._lock = threading.Lock()
        self._enabled = False
        self._enabled_since_steady_ns = 0
        self._enabled_ros_epoch_ns = 0
        self._enable_generation = 0
        self._target = BufferedImpedanceTarget()
        self._results = []

    # ------------------------------------------------------------------
    # Configuration, readable by tests
    # ------------------------------------------------------------------

    @property
    def joint_names(self):
        """Return the configured joint names, in configured order."""
        return self._joint_names

    @property
    def position_lower(self):
        """Return the per-joint lower fence bound."""
        return self._position_lower

    @property
    def position_upper(self):
        """Return the per-joint upper fence bound."""
        return self._position_upper

    @property
    def max_header_age_ns(self):
        """Return the accepted header age window, in nanoseconds."""
        return self._max_header_age_ns

    @property
    def future_tolerance_ns(self):
        """Return the accepted header future window, in nanoseconds."""
        return self._future_tolerance_ns

    # ------------------------------------------------------------------
    # Enable epochs
    # ------------------------------------------------------------------

    def set_enabled(self, enabled, ros_now_ns, steady_now_ns):
        """
        Port ``setEnabled``: bump the generation, invalidate, record epochs.

        The C++ signature is ``(enabled, steady_now_ns, ros_now_ns)``; this port
        takes the two clocks in the order every other entry point here does
        (ros first, steady second) -- the only intentional shape difference.

        Both epochs are recorded on *every* call, enable or disable, and the
        previously buffered target is dropped either way: after this returns,
        only a target received later on the steady clock *and* stamped later on
        the ROS clock can ever be read fresh.
        """
        with self._lock:
            # Odd generations denote an in-progress update, exactly as in C++.
            self._enable_generation += 1
            self._enabled = False
            self._invalidate_locked(steady_now_ns)
            self._enabled_since_steady_ns = int(steady_now_ns)
            self._enabled_ros_epoch_ns = int(ros_now_ns)
            self._enabled = bool(enabled)
            self._enable_generation += 1

    @property
    def enabled(self):
        """Return whether the arm is currently enabled."""
        with self._lock:
            return self._enabled

    @property
    def enable_generation(self):
        """Return the enable generation counter (even means settled)."""
        with self._lock:
            return self._enable_generation

    @property
    def enabled_since_steady_ns(self):
        """Return the steady-clock epoch of the last enable/disable."""
        with self._lock:
            return self._enabled_since_steady_ns

    @property
    def enabled_ros_epoch_ns(self):
        """Return the ROS-clock epoch of the last enable/disable."""
        with self._lock:
            return self._enabled_ros_epoch_ns

    # ------------------------------------------------------------------
    # The validator itself
    # ------------------------------------------------------------------

    def accept(self, message, ros_now_ns, steady_now_ns, controller_active=True):
        """
        Port ``accept``: validate one message, buffer it, or reject and drop.

        ``steady_now_ns`` is the steady-clock receive time (the C++
        ``steady_receive_ns``). ``controller_active=False`` reproduces the
        caller-side gate of ``acceptTarget``: ``ControllerInactive`` comes back
        before any rule runs and, uniquely among the results, leaves the
        buffered target untouched.

        Every other non-``Accepted`` result goes through the C++ ``reject``
        lambda, which invalidates first and answers second.
        """
        with self._lock:
            if not controller_active:
                return self._record_locked(JointTargetValidationResult.ControllerInactive)
            return self._record_locked(
                self._accept_locked(message, int(ros_now_ns), int(steady_now_ns)))

    def _accept_locked(self, message, ros_now_ns, steady_receive_ns):
        """Run the C++ rules in the C++ order; the caller holds the lock."""
        def reject(result):
            self._invalidate_locked(steady_receive_ns)
            return result

        if message.header.frame_id:
            return reject(JointTargetValidationResult.InvalidFrame)
        stamp = message.header.stamp
        if (stamp.sec < 0 or stamp.nanosec >= NANOSECONDS_PER_SECOND
                or (stamp.sec == 0 and stamp.nanosec == 0)):
            return reject(JointTargetValidationResult.InvalidStamp)
        header_ns = int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)
        header_age_ns = ros_now_ns - header_ns
        if header_age_ns > self._max_header_age_ns:
            return reject(JointTargetValidationResult.HeaderTooOld)
        if -header_age_ns > self._future_tolerance_ns:
            return reject(JointTargetValidationResult.HeaderTooFarInFuture)
        if len(message.joint_names) != defaults.JOINT_COUNT:
            return reject(JointTargetValidationResult.InvalidNameCount)
        if len(message.points) != 1:
            return reject(JointTargetValidationResult.InvalidPointCount)

        point = message.points[0]
        if len(point.positions) != defaults.JOINT_COUNT:
            return reject(JointTargetValidationResult.InvalidPositionCount)
        if len(point.velocities):
            return reject(JointTargetValidationResult.VelocityCommandNotAllowed)
        if len(point.accelerations):
            return reject(JointTargetValidationResult.AccelerationCommandNotAllowed)
        if len(point.effort):
            return reject(JointTargetValidationResult.EffortCommandNotAllowed)
        if point.time_from_start.sec != 0 or point.time_from_start.nanosec != 0:
            return reject(JointTargetValidationResult.InvalidTimeFromStart)

        positions = [0.0] * defaults.JOINT_COUNT
        matched = [False] * defaults.JOINT_COUNT
        for message_index in range(defaults.JOINT_COUNT):
            configured_index = None
            for joint in range(defaults.JOINT_COUNT):
                if message.joint_names[message_index] == self._joint_names[joint]:
                    configured_index = joint
                    break
            if configured_index is None or matched[configured_index]:
                return reject(JointTargetValidationResult.DuplicateOrUnknownJoint)
            position = float(point.positions[message_index])
            if not math.isfinite(position):
                return reject(JointTargetValidationResult.NonfinitePosition)
            # Inclusive on both bounds: only strictly outside is a rejection.
            if (position < self._position_lower[configured_index]
                    or position > self._position_upper[configured_index]):
                return reject(JointTargetValidationResult.PositionLimitExceeded)
            matched[configured_index] = True
            positions[configured_index] = position

        self._target = BufferedImpedanceTarget(
            positions=positions, header_ns=header_ns,
            steady_receive_ns=steady_receive_ns, valid=True)
        return JointTargetValidationResult.Accepted

    def _invalidate_locked(self, steady_receive_ns):
        """Port ``invalidate``: drop the buffer, keeping only the timestamp."""
        self._target = BufferedImpedanceTarget(steady_receive_ns=steady_receive_ns)

    def _record_locked(self, result):
        """Append ``result`` to the log and return it unchanged."""
        self._results.append(result)
        return result

    # ------------------------------------------------------------------
    # The realtime side
    # ------------------------------------------------------------------

    def read_fresh(self, steady_now_ns=None, watchdog_ns=None):
        """
        Port ``readFresh``: return the usable target, or ``None``.

        A buffered target survives only if the arm is enabled, the target is
        valid, it was received strictly after the steady enable epoch, and it
        is stamped strictly after the ROS enable epoch -- so a target sent
        before an enable is never used, and after a re-enable the first
        stale-stamped target is ignored.

        ``steady_now_ns`` is optional here (it is not in C++) so a test can ask
        the epoch question alone. When it is given, the C++'s two clock checks
        also run: a receipt from the future is refused, and a receipt older
        than ``watchdog_ns`` (default ``defaults.REVIEWED_TIMING_S['watchdog_timeout']``) is refused
        -- that is the watchdog freeze.
        """
        if watchdog_ns is None:
            watchdog_ns = seconds_to_nanoseconds(defaults.REVIEWED_TIMING_S['watchdog_timeout'])
        with self._lock:
            if not self._enabled:
                return None
            target = self._target
            if (not target.valid
                    or target.steady_receive_ns <= self._enabled_since_steady_ns
                    or target.header_ns <= self._enabled_ros_epoch_ns):
                return None
            if steady_now_ns is not None:
                steady_now_ns = int(steady_now_ns)
                if (target.steady_receive_ns > steady_now_ns
                        or steady_now_ns - target.steady_receive_ns > watchdog_ns):
                    return None
            return target.positions

    def buffered_target(self):
        """Return the buffered target itself (the C++ ``nonRealtimeTarget``)."""
        with self._lock:
            return self._target

    # ------------------------------------------------------------------
    # Result log (test-only; the wire says nothing about a rejection)
    # ------------------------------------------------------------------

    @property
    def last_result(self):
        """Return the most recent :class:`JointTargetValidationResult`, or None."""
        with self._lock:
            return self._results[-1] if self._results else None

    def results(self):
        """Return every result so far, oldest first."""
        with self._lock:
            return tuple(self._results)

    def result_counts(self):
        """Return ``{result: count}`` over the whole log."""
        with self._lock:
            counts = {}
            for result in self._results:
                counts[result] = counts.get(result, 0) + 1
            return counts

    def clear_results(self):
        """Empty the result log (the buffered target is untouched)."""
        with self._lock:
            self._results = []


def accept_target(inboxes, arm, message, ros_now_ns, steady_now_ns,
                  controller_active=True):
    """
    Port ``DualArmJointImpedanceControllerCore::acceptTarget``.

    The activity gate answers ``ControllerInactive`` before the inbox is
    reached, so an inactive controller does not invalidate anything. An
    out-of-range arm index answers ``DuplicateOrUnknownJoint`` -- the C++
    genuinely reuses that enumerator for it, and it likewise never reaches an
    inbox.
    """
    if not controller_active:
        return JointTargetValidationResult.ControllerInactive
    if arm < 0 or arm >= len(inboxes):
        return JointTargetValidationResult.DuplicateOrUnknownJoint
    return inboxes[arm].accept(message, ros_now_ns, steady_now_ns)


class ArmSlot:
    """One mock controller slot: an arm id, its joint names, and its fence."""

    def __init__(self, arm_id, position_lower, position_upper, joint_names=None):
        """Build a slot; ``joint_names`` defaults to the canonical seven."""
        self.arm_id = str(arm_id)
        self.joint_names = tuple(
            canonical_joint_names(self.arm_id) if joint_names is None else joint_names)
        self.position_lower = tuple(float(value) for value in position_lower)
        self.position_upper = tuple(float(value) for value in position_upper)


class MockImpedanceController(Node):
    """
    An rclpy node that impersonates ``dual_arm_joint_impedance_controller``.

    Defined for the Stage 2 e2e (real DDS, fake hardware, no real controller):
    unit tests import the port above and never construct this. Slots are
    numbered from 1, matching ``~/arm_<n>/...``; in one-arm mode there is one
    slot and it is still ``arm_1``, whichever arm id it carries (plan 0.6).
    """

    def __init__(self, slots, measured_provider=None, node_name=CONTROLLER_NODE_NAME,
                 max_header_age_s=defaults.REVIEWED_TIMING_S['max_header_age'],
                 future_tolerance_s=DEFAULT_FUTURE_TOLERANCE_S, **node_kwargs):
        """Create the node, one inbox per slot, and every slot's endpoints."""
        slots = tuple(slots)
        if not slots:
            # Refused before the node exists, so a bad call leaves nothing to
            # destroy and no half-built participant on the graph.
            raise ValueError('a mock impedance controller needs at least one arm slot')
        arm_ids = [slot.arm_id for slot in slots]
        if len(set(arm_ids)) != len(arm_ids):
            # onConfigure() refuses two arms with one id; so does the mock,
            # before two slots could claim one error-recovery service name.
            raise ValueError('every arm slot needs a distinct arm id')
        super().__init__(node_name, **node_kwargs)
        self._slots = slots
        self._measured_provider = measured_provider
        self._lock = threading.Lock()
        self._controller_active = True
        self._enable_success = True
        self._inboxes = []
        self._received = []
        self._internal_targets = []
        self._recovery_replies = {}
        self._recovery_calls = {}
        # Not ``_subscriptions``/``_services``: rclpy's Node keeps its own
        # registries under exactly those names, and shadowing them breaks the
        # node's teardown and the executor's view of it.
        self._target_subscriptions = []
        self._slot_services = []
        self._diagnostics = self.create_publisher(
            DiagnosticArray, '/diagnostics', DIAGNOSTICS_QOS)
        for index, slot in enumerate(self._slots):
            self._inboxes.append(ArmImpedanceTargetInbox(
                slot.joint_names, slot.position_lower, slot.position_upper,
                max_header_age_s=max_header_age_s,
                future_tolerance_s=future_tolerance_s))
            self._received.append([])
            self._internal_targets.append(self.measured(slot.arm_id))
            self._recovery_replies[slot.arm_id] = (False, NO_ERRORS_MESSAGE)
            self._recovery_calls[slot.arm_id] = 0
            number = index + 1
            self._target_subscriptions.append(self.create_subscription(
                JointTrajectory, '~/arm_{}/joint_target'.format(number),
                self._target_callback(index), TARGET_QOS))
            self._slot_services.append(self.create_service(
                SetBool, '~/arm_{}/enable'.format(number),
                self._enable_callback(index)))
            self._slot_services.append(self.create_service(
                ErrorRecovery,
                '/{}_error_recovery_service_server/error_recovery'.format(slot.arm_id),
                self._recovery_callback(slot.arm_id)))

    # ------------------------------------------------------------------
    # Slot access
    # ------------------------------------------------------------------

    @property
    def slots(self):
        """Return the configured slots, in slot order (slot 1 first)."""
        return self._slots

    def slot_number(self, arm_id):
        """Return the 1-based slot number carrying ``arm_id``."""
        for index, slot in enumerate(self._slots):
            if slot.arm_id == arm_id:
                return index + 1
        raise KeyError(arm_id)

    def inbox(self, number):
        """Return the inbox of slot ``number`` (1-based)."""
        return self._inboxes[number - 1]

    def measured(self, arm_id):
        """
        Return the arm's measured joint positions.

        With no provider the fence midpoint is used: a pose that is inside the
        fence by construction, which is what the enable precondition of plan
        section 6.13 requires of a measured sample.
        """
        if self._measured_provider is not None:
            return tuple(float(value) for value in self._measured_provider(arm_id))
        slot = self._slots[self.slot_number(arm_id) - 1]
        return tuple(
            0.5 * (lower + upper)
            for lower, upper in zip(slot.position_lower, slot.position_upper))

    def internal_target(self, number):
        """Return the slot's held internal target (re-seeded at every enable)."""
        with self._lock:
            return self._internal_targets[number - 1]

    # ------------------------------------------------------------------
    # Scripting hooks
    # ------------------------------------------------------------------

    def set_controller_active(self, active):
        """Set whether the controller counts as stably active (the epoch gate)."""
        with self._lock:
            self._controller_active = bool(active)

    def set_enable_success(self, success):
        """Set whether the enable service accepts calls at all."""
        with self._lock:
            self._enable_success = bool(success)

    def set_error_recovery_reply(self, arm_id, success, error=''):
        """Script one arm's ``ErrorRecovery`` reply (default: ``No errors``)."""
        with self._lock:
            self._recovery_replies[arm_id] = (bool(success), str(error))

    def error_recovery_calls(self, arm_id):
        """Return how many ``ErrorRecovery`` calls this arm has answered."""
        with self._lock:
            return self._recovery_calls[arm_id]

    # ------------------------------------------------------------------
    # Endpoints
    # ------------------------------------------------------------------

    def _target_callback(self, index):
        """Build the ``~/arm_<n>/joint_target`` callback for one slot."""
        def _receive(message):
            ros_now_ns = self.get_clock().now().nanoseconds
            steady_ns = time.monotonic_ns()
            with self._lock:
                active = self._controller_active
            result = accept_target(
                self._inboxes, index, message, ros_now_ns, steady_ns,
                controller_active=active)
            with self._lock:
                self._received[index].append((steady_ns, result))
        return _receive

    def _enable_callback(self, index):
        """Build the ``~/arm_<n>/enable`` handler for one slot."""
        def _enable(request, response):
            ros_now_ns = self.get_clock().now().nanoseconds
            steady_ns = time.monotonic_ns()
            with self._lock:
                accepted = self._controller_active and self._enable_success
            if not accepted:
                response.success = False
                response.message = ENABLE_REJECTED_MESSAGE
                return response
            self._inboxes[index].set_enabled(request.data, ros_now_ns, steady_ns)
            # The control loop re-seeds the internal target from the measured
            # pose on every enable-generation change; the mock does the same so
            # the e2e can see that the server must re-seed too (plan 0.6).
            measured = self.measured(self._slots[index].arm_id)
            with self._lock:
                self._internal_targets[index] = measured
            response.success = True
            response.message = (
                ENABLE_ENABLED_MESSAGE if request.data else ENABLE_DISABLED_MESSAGE)
            return response
        return _enable

    def _recovery_callback(self, arm_id):
        """Build the per-arm ``ErrorRecovery`` handler."""
        def _recover(request, response):
            del request
            with self._lock:
                self._recovery_calls[arm_id] += 1
                success, error = self._recovery_replies[arm_id]
            response.success = success
            response.error = error
            return response
        return _recover

    def publish_synthetic_diagnostic(self, level, message, arm_ids=None, hardware_id=''):
        """
        Publish one ``/diagnostics`` status per arm under the canonical name.

        The name comes from ``franka_web.health.canonical_diagnostic_name`` --
        the same function the bridge filters on -- so a synthetic ERROR here
        exercises the real fault path. ``level`` may be an int or the
        ``DiagnosticStatus`` byte constant.
        """
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        if arm_ids is None:
            arm_ids = [slot.arm_id for slot in self._slots]
        for arm_id in arm_ids:
            status = DiagnosticStatus()
            status.level = level if isinstance(level, bytes) else bytes([int(level) & 0xFF])
            status.name = canonical_diagnostic_name(arm_id)
            status.message = str(message)
            status.hardware_id = str(hardware_id)
            array.status.append(status)
        self._diagnostics.publish(array)
        return array

    # ------------------------------------------------------------------
    # Observations
    # ------------------------------------------------------------------

    def received(self, number):
        """Return this slot's ``(steady_ns, result)`` log, oldest first."""
        with self._lock:
            return tuple(self._received[number - 1])

    def reset_observations(self):
        """Drop every recorded message and every inbox result log."""
        with self._lock:
            self._received = [[] for _ in self._slots]
        for inbox in self._inboxes:
            inbox.clear_results()

    def stats(self, number=None):
        """
        Return message statistics, aggregated or for one slot.

        Keys: ``messages``, ``accepted``, ``rejected``, ``inactive``,
        ``results`` (per-result-name counts), ``gaps_s`` (inter-arrival gaps,
        chronological) and ``max_gap_s``. The aggregate also carries
        ``per_arm``, keyed by slot number. Gaps are per slot: the e2e's 20 Hz
        and 0.08 s worst-gap assertions are per publisher.
        """
        with self._lock:
            if number is not None:
                return self._stats_locked(self._received[number - 1])
            aggregate = self._stats_locked(
                [entry for log in self._received for entry in log], gaps=False)
            aggregate['per_arm'] = {
                index + 1: self._stats_locked(log)
                for index, log in enumerate(self._received)}
            return aggregate

    @staticmethod
    def _stats_locked(log, gaps=True):
        """Summarize one ``(steady_ns, result)`` log; the caller holds the lock."""
        counts = {}
        for _, result in log:
            counts[result.name] = counts.get(result.name, 0) + 1
        gap_values = []
        if gaps:
            stamps = [steady_ns for steady_ns, _ in log]
            gap_values = [
                (later - earlier) / NANOSECONDS_PER_SECOND
                for earlier, later in zip(stamps, stamps[1:])]
        return {
            'messages': len(log),
            'accepted': counts.get(JointTargetValidationResult.Accepted.name, 0),
            'rejected': sum(
                counts.get(result.name, 0) for result in REJECTION_RESULTS),
            'inactive': counts.get(
                JointTargetValidationResult.ControllerInactive.name, 0),
            'results': counts,
            'gaps_s': gap_values,
            'max_gap_s': max(gap_values) if gap_values else None,
        }
