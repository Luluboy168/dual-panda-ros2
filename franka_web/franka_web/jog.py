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
The held jog target and the one message shape the controller accepts (5.2).

One :class:`JogTargetModel` per enabled arm. It owns exactly one piece of
state -- the 7-element target the arm is being commanded to -- and three ways
to touch it: :meth:`seed` from the measured pose, :meth:`step` one joint by
one fixed increment, and :meth:`invalidate` to forget it. :meth:`message`
renders the current target as the ``trajectory_msgs/JointTrajectory`` that
``dual_arm_joint_impedance_controller`` accepts.

The module is pure. No node, no clock, no I/O, no rclpy: the caller supplies
the stamp from its own node clock, and the only ROS names imported are the
message classes the returned message is built from.

Three contract rules from plan section 0.6 are load-bearing here.

A rejected message freezes the arm
    ``ArmImpedanceTargetInbox::accept`` calls ``invalidate()`` on **every**
    rejection, so a malformed target does not cost one frame -- it drops the
    buffered target and the arm holds its last internal target under the
    watchdog until a valid one arrives. Rejections are silent on the wire:
    nothing comes back to the publisher. That is why :meth:`message` builds a
    fixed shape rather than a configurable one, why the empty
    ``velocities``/``accelerations``/``effort`` sequences and the zero
    ``time_from_start`` are not options, and why the joint names and the stamp
    are checked here, where the caller still gets an exception, instead of
    being discovered as silence on the topic (standing hazard 4).

Every enable re-seeds
    On any enable/disable generation change the controller sets
    ``next_targets = positions``. A target held across a disable is stale the
    moment the arm is re-enabled, and publishing it would command an
    unintended slew-limited move. The enable path calls :meth:`seed` with the
    freshly measured pose every time; :meth:`invalidate` is the disable/fault
    side of the same rule.

The fence is the controller's own limit
    ``accept`` rejects a position outside the configured
    ``[position_lower, position_upper]`` with ``PositionLimitExceeded`` -- the
    same freeze. :meth:`step` therefore clamps to the fence rather than
    emitting the out-of-range value, and *reports* the clamp in
    ``StepResult.clamped`` so the UI can flash the joint instead of silently
    swallowing the button press.

The step magnitude is ``config.JOG_STEP_RAD`` and the API's ``direction`` means
exactly plus or minus one of them (plan section 6.13): there is deliberately no
caller-chosen magnitude, so this model has no code path that produces one.

Concurrency: the held target is a tuple, replaced by a single attribute rebind.
The HTTP worker thread that calls :meth:`step` and the ROS timer thread that
calls :meth:`message` therefore never see a torn target -- a reader gets either
the whole previous tuple or the whole new one -- and no lock is needed.
"""

from dataclasses import dataclass
import math
import numbers

from builtin_interfaces.msg import Duration, Time
from franka_web import config
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#: The two accepted values of ``step``'s ``direction`` argument.
DIRECTIONS = (-1, 1)


class JogError(ValueError):
    """
    A jog operation the model refuses to perform.

    Every refusal names what was wrong, and which joints: an operator reading
    a ``pose_outside_fence`` response needs the joint, not just the verdict.
    A refused call never changes the held target.
    """


@dataclass(frozen=True)
class StepResult:
    """
    The outcome of one :meth:`JogTargetModel.step`, as API section 6.13 sends it.

    ``target`` is the whole new 7-element target (not just the joint that
    moved) and ``clamped`` is the parallel 7-element mask, ``True`` at a joint
    whose motion the fence actually cut short. A press that only moved the
    target part-way to the requested step is reported, never swallowed.
    """

    target: tuple
    clamped: tuple


def _is_number(value):
    """Return True for a real number that is not a bool."""
    return isinstance(value, numbers.Real) and not isinstance(value, bool)


def _is_integer(value):
    """Return True for an integer that is not a bool (``True`` is not ``+1``)."""
    return isinstance(value, numbers.Integral) and not isinstance(value, bool)


def _as_seven(values, what):
    """Return ``values`` as a list of exactly ``config.JOINT_COUNT`` items."""
    if isinstance(values, (str, bytes, bytearray)):
        raise JogError('{} must be a sequence of {} entries, not a string'.format(
            what, config.JOINT_COUNT))
    try:
        items = list(values)
    except TypeError:
        raise JogError('{} must be a sequence of {} entries, got {}'.format(
            what, config.JOINT_COUNT, type(values).__name__)) from None
    if len(items) != config.JOINT_COUNT:
        raise JogError('{} must have exactly {} entries, got {}'.format(
            what, config.JOINT_COUNT, len(items)))
    return items


def _as_seven_floats(values, what, joint_names):
    """
    Return ``values`` as a list of seven finite floats, naming every offender.

    Both problems a number can have -- not being a number at all, and not
    being finite -- are collected across all seven joints and reported in one
    exception, so the operator fixes one message rather than seven.
    """
    items = _as_seven(values, what)
    problems = []
    numbers_out = []
    for index, value in enumerate(items):
        if not _is_number(value):
            problems.append('{} is not a number ({!r})'.format(joint_names[index], value))
            continue
        number = float(value)
        if not math.isfinite(number):
            problems.append('{} is not finite ({!r})'.format(joint_names[index], value))
            continue
        numbers_out.append(number)
    if problems:
        raise JogError('{}: {}'.format(what, '; '.join(problems)))
    return numbers_out


class JogTargetModel:
    """
    One arm's held jog target, fenced, clamped and renderable as a message.

    Construct one per arm from that arm's validated fence (the
    ``position_lower`` / ``position_upper`` arrays of the gains file the
    controller was launched with -- the same numbers ``accept`` checks
    against, or the model would clamp to a boundary the controller rejects).
    The model starts unseeded: :attr:`seeded` is ``False`` and :attr:`target`
    is ``None`` until the enable path seeds it from the measured pose.
    """

    def __init__(self, arm_id, fence_lower, fence_upper, step_rad=config.JOG_STEP_RAD):
        """
        Build an unseeded model for ``arm_id`` over the given fence.

        ``fence_lower`` and ``fence_upper`` are 7-sequences of finite numbers
        with ``fence_lower[i] < fence_upper[i]`` for every joint; anything
        else raises :class:`JogError` naming the joints at fault. ``step_rad``
        defaults to the one fixed UI step, ``config.JOG_STEP_RAD``, and must
        be positive and finite.
        """
        if not isinstance(arm_id, str) or not arm_id:
            raise JogError('arm_id must be a non-empty string, got {!r}'.format(arm_id))
        self._arm_id = arm_id
        self._joint_names = tuple(
            '{}_joint{}'.format(arm_id, index)
            for index in range(1, config.JOINT_COUNT + 1))
        lower = _as_seven_floats(fence_lower, 'fence_lower', self._joint_names)
        upper = _as_seven_floats(fence_upper, 'fence_upper', self._joint_names)
        inverted = [
            '{} [{!r}, {!r}]'.format(self._joint_names[index], lower[index], upper[index])
            for index in range(config.JOINT_COUNT)
            if not lower[index] < upper[index]]
        if inverted:
            raise JogError(
                'fence_lower must be strictly below fence_upper at every joint: {}'.format(
                    '; '.join(inverted)))
        if not _is_number(step_rad):
            raise JogError('step_rad must be a number, got {!r}'.format(step_rad))
        step = float(step_rad)
        if not math.isfinite(step) or step <= 0.0:
            raise JogError(
                'step_rad must be a positive, finite number of radians, got {!r}'.format(step_rad))
        self._fence_lower = tuple(lower)
        self._fence_upper = tuple(upper)
        self._step_rad = step
        self._target = None

    @property
    def arm_id(self):
        """Return the arm this model commands."""
        return self._arm_id

    @property
    def joint_names(self):
        """Return the canonical ``<arm_id>_joint1..7`` names, in controller order."""
        return self._joint_names

    @property
    def fence_lower(self):
        """Return the per-joint lower fence, as a 7-tuple of floats."""
        return self._fence_lower

    @property
    def fence_upper(self):
        """Return the per-joint upper fence, as a 7-tuple of floats."""
        return self._fence_upper

    @property
    def step_rad(self):
        """Return the fixed magnitude of one jog step, in radians."""
        return self._step_rad

    @property
    def seeded(self):
        """Return True once a measured pose has been accepted and not invalidated."""
        return self._target is not None

    @property
    def target(self):
        """Return the held 7-tuple target, or None while unseeded."""
        return self._target

    def seed(self, measured):
        """
        Adopt the measured pose as the held target; call this at every enable.

        ``measured`` is a 7-sequence of finite numbers, every one of them
        inside its joint's fence -- the plan section 5.4 precondition, which
        exists because a fence that does not contain the arm's actual pose
        commands motion the instant the controller is enabled. A value
        exactly on a fence boundary is inside it, matching ``accept``'s own
        ``position < lower || position > upper`` test.

        Re-seeding is not just allowed but required: the controller resets
        its internal target to the measured positions on every enable
        generation change (plan section 0.6), so a target carried across a
        disable would command an unintended move.

        Raises :class:`JogError` naming every offending joint, and leaves the
        previously held target untouched when it does -- a refused seed is
        never a partial mutation.
        """
        values = _as_seven_floats(measured, 'the measured pose', self._joint_names)
        outside = []
        for index, value in enumerate(values):
            lower = self._fence_lower[index]
            upper = self._fence_upper[index]
            if value < lower or value > upper:
                outside.append('{} = {!r} is outside [{!r}, {!r}]'.format(
                    self._joint_names[index], value, lower, upper))
        if outside:
            raise JogError('the measured pose is outside the fence: {}'.format('; '.join(outside)))
        self._target = tuple(values)

    def invalidate(self):
        """
        Forget the held target; call this on disable and on fault.

        Idempotent. Afterwards :meth:`step` and :meth:`message` both refuse,
        which is the point: nothing may publish a target that predates the
        next enable.
        """
        self._target = None

    def step(self, joint_index, direction):
        """
        Move one joint by one fixed step, clamped to that joint's fence.

        ``joint_index`` is ``0..6`` and ``direction`` is exactly ``-1`` or
        ``+1`` -- integers, and ``True``/``False`` are not integers here: the
        API deliberately has no caller-chosen magnitude (plan section 6.13),
        so ``direction`` selects a sign and nothing else.

        Returns the :class:`StepResult` API section 6.13 sends back, and
        updates the held target to its ``target``. Both arguments are
        validated before the seed is checked and before anything is mutated,
        so a refused step leaves the model exactly as it was.
        """
        if not _is_integer(joint_index) or not 0 <= int(joint_index) < config.JOINT_COUNT:
            raise JogError('joint_index must be an integer in 0..{}, got {!r}'.format(
                config.JOINT_COUNT - 1, joint_index))
        if not _is_integer(direction) or int(direction) not in DIRECTIONS:
            raise JogError('direction must be exactly -1 or +1, got {!r}'.format(direction))
        if self._target is None:
            raise JogError(
                'cannot jog {}: the target is not seeded (seed it from the measured pose '
                'at every enable)'.format(self._arm_id))
        index = int(joint_index)
        lower = self._fence_lower[index]
        upper = self._fence_upper[index]
        moved = self._target[index] + int(direction) * self._step_rad
        clamped = False
        if moved < lower:
            moved = lower
            clamped = True
        elif moved > upper:
            moved = upper
            clamped = True
        updated = list(self._target)
        updated[index] = moved
        self._target = tuple(updated)
        mask = tuple(
            clamped if joint == index else False for joint in range(config.JOINT_COUNT))
        return StepResult(target=self._target, clamped=mask)

    def message(self, stamp, joint_names):
        """
        Render the held target as the message plan section 5.2 specifies.

        ``stamp`` is a ``builtin_interfaces.msg.Time`` -- the caller's own
        node clock reading (``node.get_clock().now().to_msg()``), because
        freshness is measured against the controller's clock and only the
        caller has one. Its *shape* is still checked here: ``accept`` rejects
        a zero stamp, a negative ``sec`` or a ``nanosec`` at or above 1e9 with
        ``InvalidStamp``, and a rejection freezes the arm silently, so a dead
        or unset clock is refused loudly at the source instead.

        ``joint_names`` must equal :attr:`joint_names` exactly, in order: the
        controller pins its configured names to ``<arm_id>_joint1..7`` and
        rejects anything else with ``DuplicateOrUnknownJoint``. Passing them
        in rather than using the model's own copy is what lets the caller's
        names and the model's names be compared at all.

        Every other rule of plan section 0.6 is satisfied by construction:
        empty ``frame_id``, exactly seven names, exactly one point with seven
        finite in-fence positions, empty ``velocities``, ``accelerations`` and
        ``effort``, and a zero ``time_from_start``. The two rules this model
        cannot satisfy alone are the caller's: the stamp must be no more than
        ``config.MAX_HEADER_AGE_S`` old when it arrives, and it must be later
        than the enable epoch.
        """
        if not isinstance(stamp, Time):
            raise JogError(
                'stamp must be a builtin_interfaces.msg.Time from the node clock '
                '(Clock.now().to_msg()), got {}'.format(type(stamp).__name__))
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
        if sec < 0 or not 0 <= nanosec < 1000000000 or (sec == 0 and nanosec == 0):
            raise JogError(
                'stamp must be a non-zero clock reading with sec >= 0 and nanosec < 1e9, '
                'got sec={}, nanosec={}'.format(sec, nanosec))
        names = tuple(_as_seven(joint_names, 'joint_names'))
        if names != self._joint_names:
            raise JogError('joint_names must be exactly {}, got {}'.format(
                list(self._joint_names), list(names)))
        if self._target is None:
            raise JogError(
                'cannot build a target message for {}: the target is not seeded'.format(
                    self._arm_id))
        message = JointTrajectory()
        message.header.stamp = stamp
        message.header.frame_id = ''
        message.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = list(self._target)
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = Duration(sec=0, nanosec=0)
        message.points = [point]
        return message
