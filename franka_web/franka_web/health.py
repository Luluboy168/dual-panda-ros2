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
Per-arm health projection: raw ROS samples in, the plan's arm frame out.

This module is the only place that turns ``sensor_msgs/JointState``,
``franka_msgs/FrankaState`` and ``diagnostic_msgs/DiagnosticStatus`` into the
per-arm object of the `state` frame (plan section 6.11). It is pure: no node,
no clock, no I/O. Every caller passes ``(mono_ns, msg)`` samples and the
monotonic "now" it wants the ages computed against, so the whole projection is
testable with an explicit clock.

Three contract rules from the plan are load-bearing here, and each one is a
rule this module exists to enforce in exactly one place:

Joints are matched BY NAME
    ``/franka/joint_states`` carries **14** names in dual mode, in whatever
    order the broadcaster assembled them. Index arithmetic on that message
    silently mixes the two arms. :func:`extract_joints` resolves
    ``<arm_id>_joint1..7`` by name; a name that is absent yields ``None`` at
    that index and sets ``positions_stale`` (frame rule 2).

Degradation is explicit, never faked
    In a fake-hardware session there is no ``FrankaState`` and no Franka
    diagnostic. The frame says so -- ``available: False`` with every inner
    field ``None`` and ``values: {}`` -- rather than emitting plausible zeros
    that a consumer would read as a healthy real arm (frame rule 4).

The ``Errors`` field count is never hardcoded
    ``franka_msgs/Errors`` carries 41 ``bool`` fields today; the plan's prose
    says "~37", and the message is upstream's to change.
    :func:`true_error_names` reads the field list from
    ``get_fields_and_field_types()`` at call time, so a message revision
    changes the output and breaks nothing (correction D8).

``diagnostic.values`` is passed through verbatim as ``str -> str``: all keys,
unparsed, numeric-looking values left as strings (frame rule 5). Nothing here
parses, filters or renames a diagnostic key -- the canonical key set lives in
``franka_bringup.status`` and validating against it is that tool's job.
"""

import math

from franka_bringup import status as bringup_status
from franka_web import config

# Frame rule 6: the closed label set for franka_msgs/FrankaState.robot_mode.
ROBOT_MODE_LABELS = {
    0: 'other',
    1: 'idle',
    2: 'move',
    3: 'guiding',
    4: 'reflex',
    5: 'user_stopped',
    6: 'automatic_error_recovery',
}

# An unrecognized robot_mode integer keeps its number and reports the neutral
# label; ROBOT_MODE_OTHER is what the driver itself uses for "none of these".
_UNKNOWN_ROBOT_MODE_LABEL = 'other'

_SIMULATED_LINE = 'simulated hardware; no Franka diagnostics'
_NO_JOINTS_LINE = 'no joint states for this arm yet'
_INCOMPLETE_LINE = 'joint states are missing joints for this arm'
_STALE_LINE = 'joint states are stale'
_NO_DIAGNOSTIC_LINE = 'no Franka diagnostics for this arm'
_BLANK_DIAGNOSTIC_LINE = 'diagnostic reported no message'


def joint_names_for(arm_id):
    """
    Return this arm's canonical joint names, ``joint1`` first.

    These names are the frame's ordering key: every 7-element array in the
    per-arm frame is ordered to match this tuple (frame rule 2).
    """
    return tuple(
        '{}_joint{}'.format(arm_id, index) for index in range(1, config.JOINT_COUNT + 1))


def canonical_diagnostic_name(arm_id):
    """
    Return the canonical Franka diagnostic status name for ``arm_id``.

    Delegates to ``franka_bringup.status`` rather than repeating the format
    string: the name is a contract with the diagnostics node, and one
    definition of it in the workspace is the point.
    """
    return bringup_status.canonical_diagnostic_name(arm_id)


def true_error_names(errors_msg):
    """
    Return the names of the ``franka_msgs/Errors`` bool fields that are True.

    The field list comes from ``get_fields_and_field_types()`` at call time --
    never a hardcoded name list and never a hardcoded count (correction D8) --
    so the result follows the installed message definition. Order is field
    order, which is ``.msg`` declaration order. ``None`` yields ``[]``.
    """
    if errors_msg is None:
        return []
    names = []
    for name, field_type in errors_msg.get_fields_and_field_types().items():
        if field_type != 'boolean':
            continue
        if getattr(errors_msg, name, False):
            names.append(name)
    return names


def extract_joints(arm_id, joint_state_msg):
    """
    Resolve this arm's seven joints out of a ``JointState`` BY NAME.

    ``joint_state_msg`` may carry 14 names in dual mode, in any order, and may
    be ``None`` or empty. A canonical name that is absent -- or present but
    without a value in the requested column -- yields ``None`` at that index.
    A duplicated name resolves to its FIRST occurrence: a second entry for the
    same joint is a publisher bug, and silently preferring the later one would
    hide it behind whichever value happened to arrive last.

    ``complete`` reports whether all seven POSITIONS resolved. Velocity and
    effort are legitimately empty on some publishers, so their absence does not
    make the sample incomplete and never marks the arm stale.
    """
    names = joint_names_for(arm_id)
    index_by_name = {}
    for index, name in enumerate(_sequence(joint_state_msg, 'name')):
        if name not in index_by_name:
            index_by_name[name] = index
    positions = _column(joint_state_msg, 'position', names, index_by_name)
    return {
        'joint_names': list(names),
        'positions': positions,
        'velocities': _column(joint_state_msg, 'velocity', names, index_by_name),
        'efforts': _column(joint_state_msg, 'effort', names, index_by_name),
        'complete': all(value is not None for value in positions),
    }


def project_arm(arm_id, now_mono_ns, joint_sample, robot_state_sample, diagnostic_sample,
                stale_after_s=config.JOINT_STATE_STALE_FAULT_S):
    """
    Project one arm's latest samples into the plan's per-arm frame.

    Each sample is a ``(mono_ns, msg)`` tuple or ``None`` for "never seen";
    ages are computed against ``now_mono_ns``, which the caller supplies, so
    this function never reads a clock. The returned dict is the section 6.11
    per-arm object MINUS its ``motion`` key, which the supervisor owns and
    adds -- motion state is not derivable from these three messages.

    ``positions_age_s`` is ``None`` only when no joint sample was ever seen.
    ``positions_stale`` is True when the sample is missing, incomplete, or
    older than ``stale_after_s`` (fault rule F6's window by default).
    """
    joint_ns, joint_msg = _split_sample(joint_sample)
    joints = extract_joints(arm_id, joint_msg)
    positions = joints['positions']
    seen = joint_ns is not None
    positions_age_s = _age_s(now_mono_ns, joint_ns) if seen else None
    positions_stale = (
        not seen or not joints['complete'] or positions_age_s > float(stale_after_s))
    has_position = any(value is not None for value in positions)

    robot_state = _project_robot_state(now_mono_ns, robot_state_sample)
    diagnostic = _project_diagnostic(now_mono_ns, diagnostic_sample)
    return {
        'arm_id': arm_id,
        'status': _status_for(diagnostic['level'], has_position, positions_stale),
        'status_line': _status_line(
            diagnostic, robot_state, has_position, joints['complete'], positions_stale),
        'joint_names': joints['joint_names'],
        'positions': positions,
        'velocities': joints['velocities'],
        'efforts': joints['efforts'],
        'positions_age_s': positions_age_s,
        'positions_stale': positions_stale,
        'robot_state': robot_state,
        'diagnostic': diagnostic,
    }


def _sequence(message, attribute):
    """Return a message's sequence field, or ``()`` when absent (never truth-tested)."""
    if message is None:
        return ()
    values = getattr(message, attribute, None)
    return () if values is None else values


def _finite_or_none(value):
    """
    Return ``float(value)`` when finite, else ``None``.

    A NaN or infinity must never reach a frame: Python's json module would
    emit a bare ``NaN`` token, which is not JSON, and the browser's
    ``JSON.parse`` would reject every subsequent frame (review finding R3).
    A non-finite sample is bad data and is reported as absent.
    """
    number = float(value)
    return number if math.isfinite(number) else None


def _column(message, attribute, names, index_by_name):
    """Return one seven-slot column of a ``JointState``, ``None`` where unresolved."""
    values = _sequence(message, attribute)
    column = []
    for name in names:
        index = index_by_name.get(name)
        column.append(
            None if index is None or index >= len(values)
            else _finite_or_none(values[index]))
    return column


def _split_sample(sample):
    """Split a ``(mono_ns, msg)`` sample; ``(None, None)`` means never seen."""
    if sample is None:
        return None, None
    mono_ns, message = sample
    if message is None:
        return None, None
    return mono_ns, message


def _age_s(now_mono_ns, sample_mono_ns):
    """
    Return a sample's age in seconds, clamped at zero.

    A sample stamped fractionally after the frame's ``now`` (two threads
    reading the same monotonic clock) must not surface as a negative age.
    """
    return max(0.0, (int(now_mono_ns) - int(sample_mono_ns)) / 1e9)


def _diagnostic_level(value):
    """
    Normalize a ``DiagnosticStatus.level`` to an int, or ``None`` if unreadable.

    rclpy hands the ``octet`` field over as a one-byte ``bytes`` object, not an
    int; ``franka_bringup.status`` normalizes the same way.
    """
    if isinstance(value, (bytes, bytearray)):
        return value[0] if len(value) == 1 else None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _level_label(level):
    """Map a diagnostic level to the frame's label: 0 ok, 1 warn, >= 2 error."""
    if level >= 2:
        return 'error'
    return 'warn' if level == 1 else 'ok'


def _project_robot_state(now_mono_ns, sample):
    """Project the ``FrankaState`` sample, or the explicit unavailable shape."""
    mono_ns, message = _split_sample(sample)
    if message is None:
        return {
            'available': False,
            'age_s': None,
            'control_command_success_rate': None,
            'robot_mode': None,
            'robot_mode_label': None,
            'current_errors': None,
            'last_motion_errors': None,
        }
    robot_mode = int(message.robot_mode)
    return {
        'available': True,
        'age_s': _age_s(now_mono_ns, mono_ns),
        'control_command_success_rate': _finite_or_none(
            message.control_command_success_rate),
        'robot_mode': robot_mode,
        'robot_mode_label': ROBOT_MODE_LABELS.get(robot_mode, _UNKNOWN_ROBOT_MODE_LABEL),
        'current_errors': true_error_names(message.current_errors),
        'last_motion_errors': true_error_names(message.last_motion_errors),
    }


def _project_diagnostic(now_mono_ns, sample):
    """Project the ``DiagnosticStatus`` sample, or the explicit unavailable shape."""
    mono_ns, status = _split_sample(sample)
    level = None if status is None else _diagnostic_level(status.level)
    if level is None:
        # No sample, or a level this code cannot classify: either way there is
        # nothing trustworthy to colour the tile with, so say so.
        return {
            'available': False,
            'level': None,
            'level_label': None,
            'message': None,
            'age_s': None,
            'values': {},
        }
    values = {}
    for entry in _sequence(status, 'values'):
        key = str(entry.key)
        if key not in values:
            values[key] = str(entry.value)
    return {
        'available': True,
        'level': level,
        'level_label': _level_label(level),
        'message': str(status.message),
        'age_s': _age_s(now_mono_ns, mono_ns),
        'values': values,
    }


def _status_for(level, has_position, positions_stale):
    """
    Classify the arm: ``error``, ``warn``, ``ok`` or ``unknown``.

    A diagnostic outranks the joint stream in both directions -- level >= 2 is
    the same ``diagnostic_error`` rule ``franka_status`` applies, and level 1
    is a warning even while joints flow. ``unknown`` sits below those two and
    above the staleness warning: with no joint data at all there is nothing to
    warn ABOUT, and calling that ``warn`` would be indistinguishable from an
    arm whose samples are merely late.
    """
    if level is not None:
        if level >= 2:
            return 'error'
        if level == 1:
            return 'warn'
    if not has_position:
        return 'unknown'
    return 'warn' if positions_stale else 'ok'


def _status_line(diagnostic, robot_state, has_position, complete, positions_stale):
    """
    Compose the arm's one short human sentence (frame rule 3).

    The canonical diagnostic message wins whenever there is one: it is the
    operator-facing string the diagnostics node already composed. Otherwise
    the most specific thing known about the joint stream is said, and only a
    fully healthy fake-hardware arm gets the "simulated hardware" line.
    """
    if diagnostic['available'] and diagnostic['message']:
        return diagnostic['message']
    if not has_position:
        return _NO_JOINTS_LINE
    if not complete:
        return _INCOMPLETE_LINE
    if positions_stale:
        return _STALE_LINE
    if diagnostic['available']:
        return _BLANK_DIAGNOSTIC_LINE
    if not robot_state['available']:
        return _SIMULATED_LINE
    return _NO_DIAGNOSTIC_LINE
