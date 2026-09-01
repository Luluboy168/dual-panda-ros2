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
Tests for franka_web.health: the per-arm projection of the state frame.

Real message types are constructed here (no DDS, no node, no rclpy.init) so the
by-name extraction, the Errors introspection and the diagnostic passthrough are
tested against the definitions actually installed in the workspace rather than
against a hand-written stand-in that could drift from them.
"""

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from franka_bringup.status import canonical_diagnostic_name as bringup_name, DIAGNOSTIC_KEYS
from franka_msgs.msg import Errors, FrankaState
from franka_web import defaults, health
import pytest
from sensor_msgs.msg import JointState
from support.fake_clock import FakeClock

ARM_1 = 'panda1'
ARM_2 = 'panda2'

# A fixed, deliberately non-obvious interleaving of the two arms' 14 joints:
# index arithmetic on this message resolves the wrong arm for every slot.
SHUFFLED_ORDER = (
    (ARM_2, 4), (ARM_1, 7), (ARM_1, 2), (ARM_2, 1), (ARM_1, 5), (ARM_2, 7), (ARM_1, 1),
    (ARM_2, 3), (ARM_1, 6), (ARM_2, 6), (ARM_1, 3), (ARM_2, 2), (ARM_1, 4), (ARM_2, 5),
)


def name_of(arm_id, joint):
    """Return the canonical joint name, e.g. ``panda1_joint3``."""
    return '{}_joint{}'.format(arm_id, joint)


def marker(arm_id, joint, column):
    """Return a value that encodes exactly which arm/joint/column it came from."""
    arm_offset = 100.0 if arm_id == ARM_1 else 200.0
    column_offset = {'position': 0.0, 'velocity': 0.01, 'effort': 0.02}[column]
    return arm_offset + joint + column_offset


def joint_state(order, columns=('position', 'velocity', 'effort')):
    """Build a JointState whose names follow ``order`` and whose values are markers."""
    message = JointState()
    message.name = [name_of(arm_id, joint) for arm_id, joint in order]
    for column in ('position', 'velocity', 'effort'):
        if column in columns:
            setattr(message, column,
                    [marker(arm_id, joint, column) for arm_id, joint in order])
    return message


def diagnostic_status(arm_id, level=0, message='backend state is healthy', values=None):
    """Build one canonical per-arm DiagnosticStatus."""
    status = DiagnosticStatus()
    status.name = health.canonical_diagnostic_name(arm_id)
    status.hardware_id = arm_id
    status.level = bytes([level])
    status.message = message
    if values:
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    return status


def robot_state(robot_mode=2, ccsr=0.998, current=(), last_motion=()):
    """Build a FrankaState with the Health-card fields set."""
    message = FrankaState()
    message.robot_mode = robot_mode
    message.control_command_success_rate = ccsr
    message.current_errors = errors_with(current)
    message.last_motion_errors = errors_with(last_motion)
    return message


def errors_with(names):
    """Build a franka_msgs/Errors with exactly ``names`` set True."""
    message = Errors()
    for name in names:
        setattr(message, name, True)
    return message


def canonical_values(arm_id):
    """Build a full 32-key diagnostic value map, values left deliberately unparsed."""
    values = {key: 'value-of-{}'.format(key) for key in sorted(DIAGNOSTIC_KEYS)}
    values['arm_id'] = arm_id
    values['state_age_ms'] = 'not_applicable'
    values['accepted_state_samples'] = '42000'
    values['stopped'] = 'false'
    return values


@pytest.fixture()
def clock():
    """Return a FakeClock; tests derive every mono_ns from it."""
    return FakeClock()


class TestJointNames:
    """Joint names are the frame's ordering key and come from JOINT_COUNT."""

    def test_seven_names_in_order(self):
        """joint_names_for yields joint1..joint7 for the arm, in that order."""
        assert health.joint_names_for(ARM_1) == (
            'panda1_joint1', 'panda1_joint2', 'panda1_joint3', 'panda1_joint4',
            'panda1_joint5', 'panda1_joint6', 'panda1_joint7')

    def test_length_tracks_the_config_constant(self):
        """The count is defaults.JOINT_COUNT, not a literal seven in this module."""
        assert len(health.joint_names_for(ARM_2)) == defaults.JOINT_COUNT

    def test_canonical_diagnostic_name_delegates(self):
        """The diagnostic name is franka_bringup's, not a second format string."""
        for arm_id in (ARM_1, ARM_2):
            assert health.canonical_diagnostic_name(arm_id) == bringup_name(arm_id)
        assert health.canonical_diagnostic_name(ARM_1) == (
            'franka_hardware_diagnostics: franka_hardware/panda1')


class TestExtractJointsByName:
    """A 14-name dual JointState must be resolved by name, never by index."""

    def test_shuffled_dual_message_resolves_both_arms(self):
        """Every slot carries its own arm's marker despite the interleaved order."""
        message = joint_state(SHUFFLED_ORDER)
        for arm_id in (ARM_1, ARM_2):
            extracted = health.extract_joints(arm_id, message)
            assert extracted['joint_names'] == [
                name_of(arm_id, joint) for joint in range(1, 8)]
            assert extracted['positions'] == [
                marker(arm_id, joint, 'position') for joint in range(1, 8)]
            assert extracted['velocities'] == [
                marker(arm_id, joint, 'velocity') for joint in range(1, 8)]
            assert extracted['efforts'] == [
                marker(arm_id, joint, 'effort') for joint in range(1, 8)]
            assert extracted['complete'] is True

    def test_index_order_would_have_been_wrong(self):
        """Guard the guard: the fixture really does defeat positional reads."""
        message = joint_state(SHUFFLED_ORDER)
        assert list(message.name[:7]) != [name_of(ARM_1, joint) for joint in range(1, 8)]

    def test_single_arm_seven_name_message(self):
        """A one-arm bringup publishes seven names and resolves identically."""
        order = tuple((ARM_1, joint) for joint in range(1, 8))
        extracted = health.extract_joints(ARM_1, joint_state(order))
        assert extracted['positions'] == [
            marker(ARM_1, joint, 'position') for joint in range(1, 8)]
        assert extracted['complete'] is True

    def test_missing_name_yields_none_at_that_index(self):
        """An absent joint is None in place, and the sample is not complete."""
        order = tuple(entry for entry in SHUFFLED_ORDER if entry != (ARM_1, 4))
        extracted = health.extract_joints(ARM_1, joint_state(order))
        assert extracted['positions'][3] is None
        assert extracted['velocities'][3] is None
        assert extracted['efforts'][3] is None
        assert extracted['positions'][2] == marker(ARM_1, 3, 'position')
        assert extracted['positions'][4] == marker(ARM_1, 5, 'position')
        assert extracted['complete'] is False

    def test_duplicate_name_takes_the_first_occurrence(self):
        """A duplicated joint resolves to the first entry, not the last."""
        message = JointState()
        message.name = [name_of(ARM_1, joint) for joint in range(1, 8)] + [name_of(ARM_1, 1)]
        message.position = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 99.0]
        extracted = health.extract_joints(ARM_1, message)
        assert extracted['positions'][0] == 1.0
        assert 99.0 not in extracted['positions']

    def test_other_arms_names_are_ignored(self):
        """A message carrying only the other arm resolves to all None."""
        order = tuple((ARM_2, joint) for joint in range(1, 8))
        extracted = health.extract_joints(ARM_1, joint_state(order))
        assert extracted['positions'] == [None] * 7
        assert extracted['complete'] is False

    def test_empty_and_none_messages(self):
        """An empty or missing JointState yields all None in every column."""
        for message in (None, JointState()):
            extracted = health.extract_joints(ARM_1, message)
            assert extracted['positions'] == [None] * 7
            assert extracted['velocities'] == [None] * 7
            assert extracted['efforts'] == [None] * 7
            assert extracted['complete'] is False

    def test_missing_columns_do_not_break_completeness(self):
        """Empty velocity/effort arrays are normal and never mark the arm stale."""
        message = joint_state(SHUFFLED_ORDER, columns=('position',))
        extracted = health.extract_joints(ARM_1, message)
        assert extracted['complete'] is True
        assert extracted['velocities'] == [None] * 7
        assert extracted['efforts'] == [None] * 7

    def test_short_position_array_yields_none(self):
        """A name present with no matching value slot resolves to None."""
        message = JointState()
        message.name = [name_of(ARM_1, joint) for joint in range(1, 8)]
        message.position = [1.0, 2.0, 3.0]
        extracted = health.extract_joints(ARM_1, message)
        assert extracted['positions'] == [1.0, 2.0, 3.0, None, None, None, None]
        assert extracted['complete'] is False


class TestTrueErrorNames:
    """Errors names come from introspection; the count is never hardcoded."""

    def test_errors_message_has_fortyone_bool_fields(self):
        """Pins correction D8: the count is read, not assumed (41 today)."""
        types = Errors().get_fields_and_field_types()
        bool_fields = [name for name, kind in types.items() if kind == 'boolean']
        assert len(bool_fields) == 41
        assert len(bool_fields) == len(types)

    def test_exact_names_of_the_true_fields(self):
        """Only the fields set True come back, under their real field names."""
        message = errors_with(
            ('joint_reflex', 'cartesian_reflex', 'power_limit_violation'))
        assert health.true_error_names(message) == [
            'joint_reflex', 'cartesian_reflex', 'power_limit_violation']

    def test_order_is_field_order_not_call_order(self):
        """The result follows .msg declaration order regardless of how it was set."""
        types = Errors().get_fields_and_field_types()
        declared = [name for name, kind in types.items() if kind == 'boolean']
        chosen = [declared[30], declared[1], declared[17]]
        message = errors_with(chosen)
        assert health.true_error_names(message) == [
            declared[1], declared[17], declared[30]]

    def test_all_fields_true_returns_every_field(self):
        """Every bool field can be reported; nothing is filtered out."""
        types = Errors().get_fields_and_field_types()
        declared = [name for name, kind in types.items() if kind == 'boolean']
        assert health.true_error_names(errors_with(declared)) == declared

    def test_empty_and_none(self):
        """A clean Errors message and None both yield the empty list."""
        assert health.true_error_names(Errors()) == []
        assert health.true_error_names(None) == []


class TestRobotStateProjection:
    """The FrankaState half of the frame, including every robot_mode label."""

    @pytest.mark.parametrize('mode,label', [
        (0, 'other'),
        (1, 'idle'),
        (2, 'move'),
        (3, 'guiding'),
        (4, 'reflex'),
        (5, 'user_stopped'),
        (6, 'automatic_error_recovery'),
    ])
    def test_every_label(self, clock, mode, label):
        """All seven documented robot_mode values map to their frame label."""
        frame = health.project_arm(
            ARM_1, clock.monotonic_ns(), None,
            (clock.monotonic_ns(), robot_state(robot_mode=mode)), None)
        assert frame['robot_state']['robot_mode'] == mode
        assert frame['robot_state']['robot_mode_label'] == label

    def test_labels_constant_matches_the_projection(self):
        """ROBOT_MODE_LABELS is the closed set of frame rule 6."""
        assert set(health.ROBOT_MODE_LABELS) == set(range(7))
        assert sorted(health.ROBOT_MODE_LABELS.values()) == sorted([
            'other', 'idle', 'move', 'guiding', 'reflex', 'user_stopped',
            'automatic_error_recovery'])

    def test_unknown_mode_keeps_the_number_and_reports_other(self, clock):
        """An unrecognized mode is not invented away: the integer survives."""
        frame = health.project_arm(
            ARM_1, clock.monotonic_ns(), None,
            (clock.monotonic_ns(), robot_state(robot_mode=250)), None)
        assert frame['robot_state']['robot_mode'] == 250
        assert frame['robot_state']['robot_mode_label'] == 'other'

    def test_errors_and_ccsr_and_age(self, clock):
        """ccsr, both error lists and the sample age are carried through."""
        sample_ns = clock.monotonic_ns()
        clock.advance(0.041)
        frame = health.project_arm(
            ARM_1, clock.monotonic_ns(), None,
            (sample_ns, robot_state(
                ccsr=0.9375, current=('instability_detected',),
                last_motion=('joint_reflex', 'tau_j_range_violation'))),
            None)
        state = frame['robot_state']
        assert state['available'] is True
        assert state['control_command_success_rate'] == pytest.approx(0.9375)
        assert state['current_errors'] == ['instability_detected']
        assert state['last_motion_errors'] == ['joint_reflex', 'tau_j_range_violation']
        assert state['age_s'] == pytest.approx(0.041, abs=1e-6)


class TestDiagnosticProjection:
    """The diagnostic half: levels, labels, and verbatim value passthrough."""

    @pytest.mark.parametrize('level,label,status', [
        (0, 'ok', 'ok'),
        (1, 'warn', 'warn'),
        (2, 'error', 'error'),
        (3, 'error', 'error'),
    ])
    def test_level_labels_and_status(self, clock, level, label, status):
        """0/1/>=2 map to ok/warn/error, and level >= 2 forces arm status error."""
        now = clock.monotonic_ns()
        frame = health.project_arm(
            ARM_1, now, (now, joint_state(SHUFFLED_ORDER)), None,
            (now, diagnostic_status(ARM_1, level=level, message='m')))
        assert frame['diagnostic']['level'] == level
        assert frame['diagnostic']['level_label'] == label
        assert frame['status'] == status

    def test_values_pass_through_verbatim(self, clock):
        """All 32 canonical keys survive unparsed, as strings, unrenamed."""
        now = clock.monotonic_ns()
        values = canonical_values(ARM_1)
        frame = health.project_arm(
            ARM_1, now, None, None,
            (now, diagnostic_status(ARM_1, values=values)))
        projected = frame['diagnostic']['values']
        assert projected == values
        assert len(projected) == 32
        assert projected['state_age_ms'] == 'not_applicable'
        assert isinstance(projected['state_age_ms'], str)
        assert projected['accepted_state_samples'] == '42000'
        assert isinstance(projected['accepted_state_samples'], str)
        assert projected['stopped'] == 'false'
        assert isinstance(projected['stopped'], str)

    def test_unknown_keys_are_not_filtered(self, clock):
        """A key this code has never heard of still reaches the browser."""
        now = clock.monotonic_ns()
        values = canonical_values(ARM_1)
        values['a_future_key'] = 'a future value'
        frame = health.project_arm(
            ARM_1, now, None, None, (now, diagnostic_status(ARM_1, values=values)))
        assert frame['diagnostic']['values']['a_future_key'] == 'a future value'

    def test_message_and_age(self, clock):
        """The canonical message is carried through with the sample's age."""
        sample_ns = clock.monotonic_ns()
        clock.advance(0.6)
        frame = health.project_arm(
            ARM_1, clock.monotonic_ns(), None, None,
            (sample_ns, diagnostic_status(ARM_1, message='backend state is healthy')))
        assert frame['diagnostic']['message'] == 'backend state is healthy'
        assert frame['diagnostic']['age_s'] == pytest.approx(0.6, abs=1e-6)

    def test_integer_level_is_accepted(self, clock):
        """A level handed over as a plain int normalizes like the octet does."""
        now = clock.monotonic_ns()
        status = diagnostic_status(ARM_1, level=1)
        status.level = 1
        frame = health.project_arm(ARM_1, now, None, None, (now, status))
        assert frame['diagnostic']['level'] == 1
        assert frame['diagnostic']['level_label'] == 'warn'


class TestFakeModeDegradation:
    """Frame rule 4: fake hardware degrades explicitly, never with zeros."""

    def test_available_false_everywhere_and_no_faked_values(self, clock):
        """Joints flow; robot_state and diagnostic say unavailable, field by field."""
        now = clock.monotonic_ns()
        frame = health.project_arm(ARM_1, now, (now, joint_state(SHUFFLED_ORDER)), None, None)
        assert frame['robot_state'] == {
            'available': False,
            'age_s': None,
            'control_command_success_rate': None,
            'robot_mode': None,
            'robot_mode_label': None,
            'current_errors': None,
            'last_motion_errors': None,
        }
        assert frame['diagnostic'] == {
            'available': False,
            'level': None,
            'level_label': None,
            'message': None,
            'age_s': None,
            'values': {},
        }
        assert frame['status'] == 'ok'
        assert frame['status_line'] == 'simulated hardware; no Franka diagnostics'

    def test_no_zero_is_ever_substituted(self, clock):
        """No inner field is 0, 0.0, '' or [] -- a consumer cannot misread it as real."""
        now = clock.monotonic_ns()
        frame = health.project_arm(ARM_1, now, (now, joint_state(SHUFFLED_ORDER)), None, None)
        for value in frame['robot_state'].values():
            assert value is None or value is False
        for key, value in frame['diagnostic'].items():
            if key == 'values':
                assert value == {}
            else:
                assert value is None or value is False

    def test_the_frame_has_exactly_the_contract_keys(self, clock):
        """project_arm returns the section 6.11 arm object minus 'motion'."""
        now = clock.monotonic_ns()
        frame = health.project_arm(ARM_1, now, (now, joint_state(SHUFFLED_ORDER)), None, None)
        assert set(frame) == {
            'arm_id', 'status', 'status_line', 'joint_names', 'positions', 'velocities',
            'efforts', 'positions_age_s', 'positions_stale', 'robot_state', 'diagnostic',
        }
        assert 'motion' not in frame
        assert frame['arm_id'] == ARM_1
        assert frame['joint_names'] == [name_of(ARM_1, joint) for joint in range(1, 8)]


class TestStaleness:
    """Ages come from the caller's mono_ns; the window is F6's by default."""

    def test_fresh_sample_is_not_stale(self, clock):
        """A sample well inside the window is fresh and reports its age."""
        sample_ns = clock.monotonic_ns()
        clock.advance(0.031)
        frame = health.project_arm(
            ARM_1, clock.monotonic_ns(), (sample_ns, joint_state(SHUFFLED_ORDER)), None, None)
        assert frame['positions_age_s'] == pytest.approx(0.031, abs=1e-6)
        assert frame['positions_stale'] is False
        assert frame['status'] == 'ok'

    def test_exactly_at_the_threshold_is_not_stale(self, clock):
        """The boundary is inclusive: age == stale_after_s is still fresh."""
        sample_ns = clock.monotonic_ns()
        clock.advance(defaults.JOINT_STATE_STALE_FAULT_S)
        frame = health.project_arm(
            ARM_1, clock.monotonic_ns(), (sample_ns, joint_state(SHUFFLED_ORDER)), None, None)
        assert frame['positions_age_s'] == pytest.approx(defaults.JOINT_STATE_STALE_FAULT_S)
        assert frame['positions_stale'] is False

    def test_past_the_threshold_is_stale_and_warns(self, clock):
        """Beyond the window the arm is stale and warns without any diagnostic."""
        sample_ns = clock.monotonic_ns()
        clock.advance(defaults.JOINT_STATE_STALE_FAULT_S + 0.5)
        frame = health.project_arm(
            ARM_1, clock.monotonic_ns(), (sample_ns, joint_state(SHUFFLED_ORDER)), None, None)
        assert frame['positions_stale'] is True
        assert frame['status'] == 'warn'
        assert frame['status_line'] == 'joint states are stale'

    def test_custom_window(self, clock):
        """stale_after_s is honoured, so callers can tighten the window."""
        sample_ns = clock.monotonic_ns()
        clock.advance(0.25)
        frame = health.project_arm(
            ARM_1, clock.monotonic_ns(), (sample_ns, joint_state(SHUFFLED_ORDER)), None, None,
            stale_after_s=0.2)
        assert frame['positions_stale'] is True

    def test_never_seen_has_no_age_and_is_stale(self, clock):
        """positions_age_s is None only when no sample was ever received."""
        frame = health.project_arm(ARM_1, clock.monotonic_ns(), None, None, None)
        assert frame['positions_age_s'] is None
        assert frame['positions_stale'] is True
        assert frame['positions'] == [None] * 7

    def test_incomplete_sample_is_stale_even_when_fresh(self, clock):
        """A missing joint name sets positions_stale (frame rule 2)."""
        now = clock.monotonic_ns()
        order = tuple(entry for entry in SHUFFLED_ORDER if entry != (ARM_1, 6))
        frame = health.project_arm(ARM_1, now, (now, joint_state(order)), None, None)
        assert frame['positions_age_s'] == pytest.approx(0.0)
        assert frame['positions_stale'] is True
        assert frame['positions'][5] is None
        assert frame['status'] == 'warn'

    def test_a_sample_from_the_future_never_reports_a_negative_age(self, clock):
        """Two threads reading one clock must not produce a negative age."""
        now = clock.monotonic_ns()
        frame = health.project_arm(
            ARM_1, now, (now + 5_000_000, joint_state(SHUFFLED_ORDER)), None, None)
        assert frame['positions_age_s'] == 0.0
        assert frame['positions_stale'] is False


class TestStatusMatrix:
    """The status / status_line ladder, case by case."""

    def _frame(self, clock, joints=True, fresh=True, complete=True,
               state=False, level=None, message='backend state is healthy'):
        """Project one arm from a described situation."""
        now = clock.monotonic_ns()
        joint_sample = None
        if joints:
            order = SHUFFLED_ORDER
            if not complete:
                order = tuple(entry for entry in order if entry != (ARM_1, 2))
            age_ns = 0 if fresh else int((defaults.JOINT_STATE_STALE_FAULT_S + 1.0) * 1e9)
            joint_sample = (now - age_ns, joint_state(order))
        state_sample = (now, robot_state()) if state else None
        diagnostic_sample = (
            None if level is None
            else (now, diagnostic_status(ARM_1, level=level, message=message)))
        return health.project_arm(
            ARM_1, now, joint_sample, state_sample, diagnostic_sample)

    def test_nothing_at_all_is_unknown(self, clock):
        """No joints, no state, no diagnostic: the arm is unknown, not warn."""
        frame = self._frame(clock, joints=False)
        assert frame['status'] == 'unknown'
        assert frame['status_line'] == 'no joint states for this arm yet'

    def test_no_joints_but_an_ok_diagnostic_is_still_unknown(self, clock):
        """A healthy diagnostic does not manufacture joint data."""
        frame = self._frame(clock, joints=False, level=0)
        assert frame['status'] == 'unknown'
        assert frame['status_line'] == 'backend state is healthy'

    def test_no_joints_with_a_warn_diagnostic_warns(self, clock):
        """A level-1 diagnostic outranks 'unknown': the arm is known to be unwell."""
        frame = self._frame(clock, joints=False, level=1, message='backend queue is saturated')
        assert frame['status'] == 'warn'
        assert frame['status_line'] == 'backend queue is saturated'

    def test_error_diagnostic_wins_over_healthy_joints(self, clock):
        """Level >= 2 is franka_status's diagnostic_error rule, applied here."""
        frame = self._frame(clock, level=2, message='global fault latched', state=True)
        assert frame['status'] == 'error'
        assert frame['status_line'] == 'global fault latched'

    def test_error_diagnostic_wins_over_stale_joints(self, clock):
        """Stale joints do not downgrade an error to a warning."""
        frame = self._frame(clock, fresh=False, level=2, message='backend worker fault')
        assert frame['status'] == 'error'
        assert frame['status_line'] == 'backend worker fault'

    def test_healthy_production_arm(self, clock):
        """Fresh joints, a FrankaState and an OK diagnostic: ok with its message."""
        frame = self._frame(clock, state=True, level=0)
        assert frame['status'] == 'ok'
        assert frame['status_line'] == 'backend state is healthy'
        assert frame['robot_state']['available'] is True
        assert frame['diagnostic']['available'] is True

    def test_stale_joints_warn_when_the_diagnostic_is_ok(self, clock):
        """A stale stream warns even while the diagnostic still reads OK."""
        frame = self._frame(clock, fresh=False, state=True, level=0)
        assert frame['status'] == 'warn'
        assert frame['status_line'] == 'backend state is healthy'

    def test_incomplete_joints_warn_and_say_so(self, clock):
        """With no diagnostic, the composed line names the incompleteness."""
        frame = self._frame(clock, complete=False)
        assert frame['status'] == 'warn'
        assert frame['status_line'] == 'joint states are missing joints for this arm'

    def test_simulated_line_only_when_both_are_unavailable(self, clock):
        """The simulated line is reserved for the fake-hardware shape."""
        assert self._frame(clock)['status_line'] == (
            'simulated hardware; no Franka diagnostics')
        assert self._frame(clock, state=True)['status_line'] == (
            'no Franka diagnostics for this arm')

    def test_blank_diagnostic_message_falls_back(self, clock):
        """An available diagnostic with an empty message still yields a sentence."""
        frame = self._frame(clock, level=0, message='')
        assert frame['status'] == 'ok'
        assert frame['status_line'] == 'diagnostic reported no message'

    def test_status_is_always_in_the_closed_set(self, clock):
        """Frame rule 3: status is one of ok/warn/error/unknown, always."""
        allowed = {'ok', 'warn', 'error', 'unknown'}
        for joints in (True, False):
            for fresh in (True, False):
                for complete in (True, False):
                    for level in (None, 0, 1, 2):
                        frame = self._frame(
                            clock, joints=joints, fresh=fresh, complete=complete, level=level)
                        assert frame['status'] in allowed
                        assert isinstance(frame['status_line'], str)
                        assert frame['status_line']


def test_project_arm_key_set_is_unchanged_from_v1():
    """
    The per-arm projection's key set is frozen, and a literal pins it.

    Everything `project_arm` returns reaches the state frame verbatim -- the
    session supervisor only ADDS a `motion` block beside it -- so a key
    quietly added, renamed or dropped here would reshape the console's
    contract without anything else noticing.
    """
    projection = health.project_arm(
        ARM_1, 0, None, None, None)
    assert set(projection) == {
        'arm_id', 'status', 'status_line', 'joint_names', 'positions',
        'velocities', 'efforts', 'positions_age_s', 'positions_stale',
        'robot_state', 'diagnostic',
    }


# ----------------------------------------------------------------------
# The gripper block
# ----------------------------------------------------------------------

#: A fresh, healthy sample: all thirteen keys the node always publishes.
HEALTHY_GRIPPER = {
    'width_mm': '84.7',
    'requested_width_mm': '85.0',
    'object': 'at_position',
    'activated': 'true',
    'moving': 'false',
    'fault_code': '0x00',
    'fault_name': 'no_fault',
    'fault_class': 'none',
    'current_ma': '120',
    'speed_mm_s': '85.0',
    'force_n': '74.0',
    'port': 'usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0',
    'link': 'up',
}

#: The sixteen keys of the frame block, and the four that are never null.
GRIPPER_FRAME_KEYS = (
    'configured', 'available', 'width_mm', 'requested_width_mm', 'object',
    'moving', 'activated', 'fault_code', 'fault_name', 'fault_class',
    'status_line', 'level', 'speed_mm_s', 'force_n', 'port', 'busy')


def gripper_status(values=None, *, message='Open 84.7 mm.', level=0,
                   hardware_id=None):
    """Build one gripper DiagnosticStatus with the given values."""
    merged = dict(HEALTHY_GRIPPER)
    if values is not None:
        merged.update(values)
    status = DiagnosticStatus()
    status.name = 'panda1 Robotiq 2F-85'
    status.hardware_id = (merged['port'] if hardware_id is None else hardware_id)
    status.level = level
    status.message = message
    status.values = [KeyValue(key=key, value=value)
                     for key, value in merged.items()]
    return status


class TestGripperProjection:
    """project_gripper: four shapes, one parse table, and no guessing."""

    def project(self, sample, *, configured=True, busy=False, now_ns=1_000_000_000,
                stale_after_s=defaults.GRIPPER_STATUS_STALE_S):
        """Project one sample at a fixed clock."""
        return health.project_gripper(
            ARM_1, now_ns, sample, configured=configured, busy=busy,
            stale_after_s=stale_after_s)

    def fresh(self, values=None, **kwargs):
        """Project a sample stamped at the projection clock."""
        return self.project((1_000_000_000, gripper_status(values, **kwargs)))

    def test_an_unconfigured_arm_reports_the_contract_shape(self):
        """Not configured: every measurement null, and a sentence saying so."""
        block = self.project(None, configured=False)
        assert block['configured'] is False
        assert block['available'] is False
        assert block['level'] == 'unknown'
        assert block['status_line'] == 'No gripper is configured for panda1.'
        assert block['width_mm'] is None

    def test_a_never_seen_status_says_no_node_is_running_and_names_the_launch_command(
            self):
        """Nobody but the operator starts these nodes, and the page says so."""
        block = self.project(None, configured=True)
        assert block['configured'] is True
        assert block['available'] is False
        assert block['status_line'] == (
            'No gripper node is running for panda1. Start it with '
            '"ros2 launch {} {}".'.format(defaults.GRIPPER_LAUNCH_PACKAGE,
                                          defaults.GRIPPER_DUAL_LAUNCH_FILE))

    def test_a_stale_status_reports_no_news_and_nulls_every_measurement(self):
        """A stale width is worse than none."""
        old = 1_000_000_000 - int(defaults.GRIPPER_STATUS_STALE_S * 1e9) - 1
        block = self.project((old, gripper_status()))
        assert block['status_line'] == 'No news from the panda1 gripper.'
        assert block['available'] is False
        assert block['level'] == 'unknown'
        for key in ('width_mm', 'object', 'activated', 'force_n', 'port'):
            assert block[key] is None

    def test_an_unreadable_level_projects_unknown_and_never_raises(self):
        """_level_label is never reached with anything but an int."""
        block = self.fresh(level=object())
        assert block['level'] == 'unknown'
        assert block['status_line'] == 'Open 84.7 mm.'
        for shape in (self.project(None, configured=False),
                      self.project(None, configured=True)):
            assert shape['level'] == 'unknown'

    def test_a_fresh_status_parses_every_contracted_key(self):
        """The healthy sample lands in the frame with its types."""
        block = self.fresh()
        assert block['available'] is True
        assert block['width_mm'] == pytest.approx(84.7)
        assert block['requested_width_mm'] == pytest.approx(85.0)
        assert block['object'] == 'at_position'
        assert block['moving'] is False
        assert block['activated'] is True
        assert block['fault_code'] == 0
        assert block['fault_name'] == 'no_fault'
        assert block['fault_class'] == 'none'
        assert block['speed_mm_s'] == pytest.approx(85.0)
        assert block['force_n'] == pytest.approx(74.0)
        assert block['port'] == HEALTHY_GRIPPER['port']
        assert block['level'] == 'ok'

    def test_the_status_line_is_the_nodes_own_message_verbatim(self):
        """One owner, no drift: the node composed it, the page renders it."""
        block = self.fresh(message='Holding an object at 32.1 mm.')
        assert block['status_line'] == 'Holding an object at 32.1 mm.'

    @pytest.mark.parametrize('key', ['width_mm', 'requested_width_mm', 'moving',
                                     'activated', 'fault_code', 'force_n'])
    def test_a_missing_or_unparseable_value_yields_null_not_a_guess(self, key):
        """An unreadable entry is absent, never a plausible zero."""
        assert self.fresh({key: 'nonsense'})[key] is None

    def test_an_unknown_object_value_falls_back_to_unknown(self):
        """The object enum is closed; anything else reads as unknown."""
        assert self.fresh({'object': 'gripping_hard'})['object'] == 'unknown'
        assert self.fresh({'fault_class': 'unknown'})['fault_class'] is None

    def test_a_hex_and_a_decimal_fault_code_both_parse(self):
        """0x0c and 12 are the same fault."""
        assert self.fresh({'fault_code': '0x0C'})['fault_code'] == 12
        assert self.fresh({'fault_code': '12'})['fault_code'] == 12

    def test_the_level_label_uses_the_shared_diagnostic_normalizer(self):
        """Including the one-byte bytes form rclpy delivers."""
        assert self.fresh(level=1)['level'] == 'warn'
        assert self.fresh(level=2)['level'] == 'error'
        assert self.fresh(level=b'\x02')['level'] == 'error'

    def test_link_down_makes_available_false_even_on_a_fresh_sample(self):
        """The link decides availability: node up AND serial link up."""
        block = self.fresh({'link': 'down'}, level=2,
                           message='No serial link to the panda1 gripper.')
        assert block['available'] is False
        assert block['level'] == 'error'
        assert block['status_line'] == 'No serial link to the panda1 gripper.'

    def test_busy_is_true_when_the_node_reports_moving_with_no_request_of_ours(self):
        """An operator's own script sent the goal; the buttons must still drop."""
        block = self.fresh({'moving': 'true'})
        assert block['moving'] is True
        assert block['busy'] is True

    def test_busy_is_false_when_moving_is_unparseable_and_we_sent_nothing(self):
        """None is not True: an unreadable sample cannot manufacture a busy row."""
        block = self.fresh({'moving': 'maybe'})
        assert block['moving'] is None
        assert block['busy'] is False

    def test_busy_is_true_while_a_request_of_ours_is_in_flight(self):
        """The caller's flag is the other half of the same OR."""
        block = self.project((1_000_000_000, gripper_status()), busy=True)
        assert block['busy'] is True

    def test_the_port_falls_back_to_the_values_entry(self):
        """hardware_id is the anti-swap evidence; values['port'] is the fallback."""
        assert self.fresh(hardware_id='')['port'] == HEALTHY_GRIPPER['port']

    def test_every_key_is_present_in_all_three_shapes(self):
        """A frame that forgets a key is a contract break, not an omission."""
        shapes = [self.project(None, configured=False),
                  self.project(None, configured=True),
                  self.fresh()]
        for shape in shapes:
            assert sorted(shape) == sorted(GRIPPER_FRAME_KEYS)
            assert isinstance(shape['configured'], bool)
            assert isinstance(shape['available'], bool)
            assert isinstance(shape['busy'], bool)
            assert shape['status_line']
            assert shape['level'] in ('ok', 'warn', 'error', 'unknown')

    def test_current_ma_is_never_a_frame_key(self):
        """A motor current is a driver diagnostic, not something a row can act on."""
        assert 'current_ma' not in self.fresh()
