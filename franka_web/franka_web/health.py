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
from franka_web import defaults

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
        '{}_joint{}'.format(arm_id, index) for index in range(1, defaults.JOINT_COUNT + 1))


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
                stale_after_s=defaults.JOINT_STATE_STALE_FAULT_S):
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


#: The section 3.6 object-detection set, verbatim.
GRIPPER_OBJECT_STATES = ('none', 'closed_on_object', 'opened_on_object',
                         'at_position', 'unknown')

#: The closed four-value fault class. The driver's own lookup has a fifth,
#: internal ``unknown``; the node maps it to ``major`` before publishing, so
#: nothing here ever sees it.
GRIPPER_FAULT_CLASSES = ('none', 'priority', 'minor', 'major')

_GRIPPER_NO_NEWS = 'No news from the {} gripper.'
_GRIPPER_UNCONFIGURED = 'No gripper is configured for {}.'
# The gripper nodes are standing nodes this server never launches, so "never
# seen" is an operator action, not a fault -- and the sentence teaches the one
# command that fixes it.
_GRIPPER_NO_NODE = ('No gripper node is running for {arm_id}. Start it with '
                    '"ros2 launch {package} {launch_file}".')

#: The measured keys, all of them null in the three degraded shapes.
_GRIPPER_MEASURED_KEYS = ('width_mm', 'requested_width_mm', 'object', 'moving',
                          'activated', 'fault_code', 'fault_name',
                          'fault_class', 'speed_mm_s', 'force_n', 'port')


def project_gripper(arm_id, now_mono_ns, status_sample, *, configured,
                    busy=False, stale_after_s=defaults.GRIPPER_STATUS_STALE_S):
    """
    Project one arm's ``~/status`` sample into the frame's ``gripper`` block.

    ``status_sample`` is a ``(mono_ns, DiagnosticStatus)`` tuple or ``None``.
    Four shapes, and only four:

    * not configured -- every measured key null, ``available`` false,
      ``level`` 'unknown', and a sentence saying so;
    * configured, never seen -- the same nulls with the "start it with
      ros2 launch ..." sentence, because nobody but the operator starts these
      nodes and the page must say so;
    * configured, seen but older than ``stale_after_s`` -- the same nulls with
      'No news from the panda1 gripper.'.  A stale width is worse than none,
      which is the same rule the driver applies when it stops publishing joint
      states on a dead link;
    * fresh -- the parsed values, with ``status_line`` taken VERBATIM from the
      node's own ``DiagnosticStatus.message``.

    ``level`` is 'unknown' in the first three shapes, decided BEFORE the
    sample is consulted, and 'unknown' again for a fresh sample whose level is
    unreadable; ``_level_label`` is never called with anything but an int,
    because it is ``level >= 2`` and ``None >= 2`` raises inside the 5 Hz
    frame pump.

    A ``values`` entry that is missing or unparseable yields ``None`` for that
    key. Nothing here guesses, and nothing here re-derives a sentence the node
    already composed.

    ``busy`` is the OR of the caller's in-flight flag and the parsed
    ``moving``, and it is computed HERE so the frame has exactly one
    authority for the key. A server-side flag alone misses a goal an
    operator's own node sent -- an expected second commander, not an edge
    case -- and ``moving`` alone drops the row's buttons back to enabled in
    the window between dispatch and the node's next sample.
    """
    caller_busy = bool(busy)
    if not configured:
        return _gripper_shape(_GRIPPER_UNCONFIGURED.format(arm_id),
                              configured=False, busy=caller_busy)
    mono_ns, status = _split_sample(status_sample)
    if status is None:
        return _gripper_shape(_GRIPPER_NO_NODE.format(
            arm_id=arm_id, package=defaults.GRIPPER_LAUNCH_PACKAGE,
            launch_file=defaults.GRIPPER_DUAL_LAUNCH_FILE),
            configured=True, busy=caller_busy)
    if _age_s(now_mono_ns, mono_ns) > float(stale_after_s):
        return _gripper_shape(_GRIPPER_NO_NEWS.format(arm_id),
                              configured=True, busy=caller_busy)
    values = {}
    for entry in _sequence(status, 'values'):
        key = str(entry.key)
        if key not in values:
            values[key] = str(entry.value)
    level = _diagnostic_level(status.level)
    moving = _gripper_bool(values.get('moving'))
    block = _gripper_shape('', configured=True, busy=caller_busy)
    block.update({
        'available': values.get('link') == 'up',
        'width_mm': _gripper_float(values.get('width_mm')),
        'requested_width_mm': _gripper_float(values.get('requested_width_mm')),
        'object': _gripper_enum(values.get('object'), GRIPPER_OBJECT_STATES,
                                'unknown'),
        'moving': moving,
        'activated': _gripper_bool(values.get('activated')),
        'fault_code': _gripper_fault_code(values.get('fault_code')),
        'fault_name': values.get('fault_name') or None,
        'fault_class': _gripper_enum(values.get('fault_class'),
                                     GRIPPER_FAULT_CLASSES, None),
        # VERBATIM: the node composed it, the page renders it, nobody in
        # between rewrites it.
        'status_line': str(status.message),
        'level': 'unknown' if level is None else _level_label(level),
        # The NODE's live settings, which is what makes them -- and not this
        # server's copy of the config file -- the honest thing to render.
        'speed_mm_s': _gripper_float(values.get('speed_mm_s')),
        'force_n': _gripper_float(values.get('force_n')),
        'port': str(status.hardware_id) or values.get('port') or None,
        # `moving` unparseable contributes nothing, so an unreadable sample
        # can never manufacture a busy row.
        'busy': caller_busy or moving is True,
    })
    return block


def _gripper_shape(status_line, *, configured, busy):
    """Return the degraded block: every measurement null, and a sentence."""
    block = {'configured': bool(configured), 'available': False,
             'status_line': status_line, 'level': 'unknown', 'busy': bool(busy)}
    for key in _GRIPPER_MEASURED_KEYS:
        block[key] = None
    return block


def _gripper_float(text):
    """Return a finite float from a ``values`` entry, else None."""
    if text is None:
        return None
    try:
        return _finite_or_none(float(text))
    except (TypeError, ValueError):
        return None


def _gripper_bool(text):
    """Return True/False for the node's own spelling, else None."""
    if text == 'true':
        return True
    if text == 'false':
        return False
    return None


def _gripper_enum(text, allowed, fallback):
    """Return the value when it is in the closed set, else the fallback."""
    if text in allowed:
        return text
    return fallback


def _gripper_fault_code(text):
    """Return a fault code from ``0x..`` or decimal text, else None."""
    if not text:
        return None
    try:
        if text.lower().startswith('0x'):
            return int(text, 16)
        return int(text)
    except (TypeError, ValueError):
        return None


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
