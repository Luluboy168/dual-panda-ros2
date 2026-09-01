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
The jog message contract: what ``franka_web`` sends, judged by the controller.

Layer 1 of the three honest Stage 2 layers (plan section 8). Every message
``JogTargetModel`` builds is fed to ``ArmImpedanceTargetInbox``, the Python
port of the C++ ``ArmImpedanceTargetInbox::accept``, and every one of the
fourteen rejection reasons is produced deliberately by a hand-mutated message
-- paired, case by case, with an assertion that the server's own construction
path cannot produce that mutation.

Two facts make this the load-bearing test and not a formality:

* A rejection is **silent on the wire**. The controller answers a target with
  nothing at all; the publisher learns nothing. The only observable is what
  this file measures on the far side -- the buffered target going away.
* A rejection **invalidates the previously buffered target** (plan section
  10.4). One malformed message does not drop one frame, it freezes the arm at
  its last internal target until a fresh valid one arrives. Every rejection
  case below therefore also asserts the buffer is gone afterwards.

``ControllerInactive`` is the one result that behaves differently, and the one
correction the citation audit made to the plan's table: it is answered by
``acceptTarget`` before the inbox is reached, so it leaves the buffer intact
and never runs a rule. The enum itself is pinned against the C++ header by
:func:`test_the_port_pins_the_cpp_enum`, so a future change to the sixteen
enumerators fails here instead of drifting silently.

No node, no ``rclpy.init``, no DDS: only message objects and the pure port.
"""

import collections
import math
from pathlib import Path
import re

from builtin_interfaces.msg import Duration, Time
from franka_web import defaults
from franka_web.jog import JogError, JogTargetModel
import pytest
from support.mock_impedance_controller import (
    accept_target, ArmImpedanceTargetInbox, canonical_joint_names,
    DEFAULT_FUTURE_TOLERANCE_S, JointTargetValidationResult, REJECTION_RESULTS,
    seconds_to_nanoseconds)
from trajectory_msgs.msg import JointTrajectoryPoint

RESULT = JointTargetValidationResult

ARM_ID = 'panda1'
JOINT_NAMES = canonical_joint_names(ARM_ID)

# A per-joint fence with a different width and offset on every joint, so a
# joint mix-up in either direction lands outside a bound rather than passing.
# Joint 4 is one-sided negative, as the canonical Panda limits make it.
FENCE_LOWER = (-2.75, -1.60, -2.75, -3.00, -2.75, 0.10, -2.75)
FENCE_UPPER = (2.75, 1.60, 2.75, -0.10, 2.75, 3.60, 2.75)

# A measured pose strictly inside every bound, with room for a jog step either
# way on every joint (the plan section 5.4 precondition for enabling at all).
MEASURED = (0.00, -0.40, 0.10, -1.50, -0.20, 1.60, 0.70)

# Fixed clock readings: a plain ROS time near 2026 and a plain steady time.
# Nothing here is measured from a real clock -- every window in this file is
# arithmetic on these two constants.
ROS_NOW_NS = 1780000000 * 1000000000
STEADY_NS = 5 * 1000000000
ENABLE_ROS_NS = ROS_NOW_NS - 100000000       # enabled 0.1 s ago on both clocks
ENABLE_STEADY_NS = STEADY_NS - 100000000

MAX_AGE_NS = seconds_to_nanoseconds(defaults.REVIEWED_TIMING_S['max_header_age'])
FUTURE_NS = seconds_to_nanoseconds(DEFAULT_FUTURE_TOLERANCE_S)
WATCHDOG_NS = seconds_to_nanoseconds(defaults.REVIEWED_TIMING_S['watchdog_timeout'])
MILLISECOND_NS = 1000000

# Small enough to be a hair outside a bound, enormous next to the ~4.4e-16
# double spacing at these magnitudes: an epsilon that genuinely crosses.
EPSILON = 1e-9

CORE_HEADER = (
    Path(__file__).resolve().parents[2] / 'franka_example_controllers' / 'src'
    / 'dual_arm_joint_impedance_controller_core.hpp')

ENUM_PATTERN = re.compile(
    r'enum\s+class\s+JointTargetValidationResult[^{]*\{(?P<body>[^}]*)\}', re.DOTALL)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def stamp_of(nanoseconds):
    """Return the ``builtin_interfaces`` Time for a clock reading in ns."""
    return Time(sec=int(nanoseconds // 1000000000),
                nanosec=int(nanoseconds % 1000000000))


GOLDEN_STAMP = stamp_of(ROS_NOW_NS)


def make_inbox():
    """Return a fresh, disabled inbox configured with this arm's fence."""
    return ArmImpedanceTargetInbox(JOINT_NAMES, FENCE_LOWER, FENCE_UPPER)


def enabled_inbox():
    """Return an inbox enabled 0.1 s ago on both clocks."""
    inbox = make_inbox()
    inbox.set_enabled(True, ENABLE_ROS_NS, ENABLE_STEADY_NS)
    return inbox


def seeded_model(measured=MEASURED):
    """Return a JogTargetModel seeded with ``measured``, as enable does."""
    model = JogTargetModel(ARM_ID, FENCE_LOWER, FENCE_UPPER,
                           step_rad=defaults.JOG_STEP_RAD)
    model.seed(list(measured))
    return model


def golden(model=None, nanoseconds=ROS_NOW_NS):
    """Return one pristine message, exactly as the server would build it."""
    if model is None:
        model = seeded_model()
    return model.message(stamp_of(nanoseconds), list(JOINT_NAMES))


def positions_of(message):
    """Return the message's seven commanded positions as a tuple."""
    return tuple(message.points[0].positions)


def set_position(message, index, value):
    """Overwrite one commanded position in place."""
    positions = list(message.points[0].positions)
    positions[index] = value
    message.points[0].positions = positions


def set_name(message, index, value):
    """Overwrite one joint name in place."""
    names = list(message.joint_names)
    names[index] = value
    message.joint_names = names


def cpp_enum_member_names():
    """Scrape the JointTargetValidationResult enumerators out of the header."""
    text = CORE_HEADER.read_text(encoding='utf-8')
    match = ENUM_PATTERN.search(text)
    assert match is not None, (
        'no JointTargetValidationResult enum class found in {}'.format(CORE_HEADER))
    body = re.sub(r'/\*.*?\*/', '', match.group('body'), flags=re.DOTALL)
    body = re.sub(r'//[^\n]*', '', body)
    names = []
    for entry in body.split(','):
        entry = entry.split('=')[0].strip()
        if entry:
            names.append(entry)
    return names


# ---------------------------------------------------------------------------
# The port is pinned to the C++ enum
# ---------------------------------------------------------------------------


def test_the_port_pins_the_cpp_enum():
    """The ported enum equals the C++ enumerators, in declaration order."""
    assert CORE_HEADER.is_file(), (
        'the controller header must be readable from the test file: {}'.format(CORE_HEADER))
    names = cpp_enum_member_names()
    assert names == [member.name for member in JointTargetValidationResult]


def test_the_enum_has_sixteen_members_two_of_which_are_not_rejections():
    """Sixteen enumerators: Accepted, ControllerInactive, and 14 rejections."""
    names = cpp_enum_member_names()
    assert len(names) == 16
    assert names[0] == 'Accepted'
    assert names[1] == 'ControllerInactive'
    assert len(REJECTION_RESULTS) == 14
    assert RESULT.Accepted not in REJECTION_RESULTS
    assert RESULT.ControllerInactive not in REJECTION_RESULTS


# ---------------------------------------------------------------------------
# The golden path
# ---------------------------------------------------------------------------


def test_the_golden_message_is_accepted_and_read_back_exactly():
    """A seeded model's message is accepted and read fresh, value for value."""
    inbox = enabled_inbox()
    message = golden()
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS) == tuple(MEASURED)


def test_the_golden_message_satisfies_every_rule_by_construction():
    """Every field the controller inspects is already what it demands."""
    message = golden()
    point = message.points[0]
    assert message.header.frame_id == ''
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (
        GOLDEN_STAMP.sec, GOLDEN_STAMP.nanosec)
    assert message.header.stamp.sec > 0
    assert message.header.stamp.nanosec < 1000000000
    assert list(message.joint_names) == list(JOINT_NAMES)
    assert len(set(message.joint_names)) == defaults.JOINT_COUNT
    assert len(message.points) == 1
    assert len(point.positions) == defaults.JOINT_COUNT
    assert all(math.isfinite(value) for value in point.positions)
    assert len(point.velocities) == 0
    assert len(point.accelerations) == 0
    assert len(point.effort) == 0
    assert (point.time_from_start.sec, point.time_from_start.nanosec) == (0, 0)


def test_every_jog_of_every_joint_in_both_directions_is_accepted():
    """Seven joints, both directions: no message the UI can produce is refused."""
    inbox = enabled_inbox()
    model = seeded_model()
    steady = STEADY_NS
    for index in range(defaults.JOINT_COUNT):
        for direction in (1, -1, -1, 1):
            model.step(index, direction)
            steady += MILLISECOND_NS
            message = golden(model)
            assert inbox.accept(message, ROS_NOW_NS, steady) is RESULT.Accepted
            assert inbox.read_fresh(steady) == pytest.approx(positions_of(message))
    assert set(inbox.results()) == {RESULT.Accepted}


def test_one_step_moves_exactly_one_jog_step_and_is_still_accepted():
    """The held target moves by defaults.JOG_STEP_RAD and stays acceptable."""
    inbox = enabled_inbox()
    model = seeded_model()
    model.step(3, -1)
    message = golden(model)
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    expected = list(MEASURED)
    expected[3] = MEASURED[3] - defaults.JOG_STEP_RAD
    assert inbox.read_fresh(STEADY_NS) == pytest.approx(expected)


def test_a_target_clamped_onto_a_fence_bound_is_still_accepted():
    """Clamping lands exactly on the bound, which ``accept`` treats as inside."""
    inbox = enabled_inbox()
    model = seeded_model()
    for _ in range(200):
        model.step(3, -1)
    message = golden(model)
    assert positions_of(message)[3] == pytest.approx(FENCE_LOWER[3])
    assert positions_of(message)[3] >= FENCE_LOWER[3]
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    for _ in range(400):
        model.step(3, 1)
    message = golden(model)
    assert positions_of(message)[3] == pytest.approx(FENCE_UPPER[3])
    assert positions_of(message)[3] <= FENCE_UPPER[3]
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS + MILLISECOND_NS) is RESULT.Accepted


# ---------------------------------------------------------------------------
# The fourteen rejections, each produced deliberately
# ---------------------------------------------------------------------------

Case = collections.namedtuple('Case', 'name expected mutate invariant')


def set_frame_id(message):
    """Name a frame -- the controller demands an empty frame_id."""
    message.header.frame_id = 'panda_link0'


def zero_the_stamp(message):
    """Stamp the message with the zero time (an unset clock)."""
    message.header.stamp = Time(sec=0, nanosec=0)


def negative_stamp_seconds(message):
    """Stamp the message with a negative seconds field."""
    message.header.stamp = Time(sec=-1, nanosec=500000)


def overflowing_stamp_nanoseconds(message):
    """Stamp the message with nanosec at 1e9, one past the legal range."""
    message.header.stamp = Time(sec=GOLDEN_STAMP.sec, nanosec=1000000000)


def stale_stamp(message):
    """Stamp the message one nanosecond older than max_header_age."""
    message.header.stamp = stamp_of(ROS_NOW_NS - MAX_AGE_NS - 1)


def future_stamp(message):
    """Stamp the message one nanosecond further ahead than the tolerance."""
    message.header.stamp = stamp_of(ROS_NOW_NS + FUTURE_NS + 1)


def six_names(message):
    """Drop a joint name: six names instead of seven."""
    message.joint_names = list(JOINT_NAMES[:6])


def eight_names(message):
    """Add a joint name: eight names instead of seven."""
    message.joint_names = list(JOINT_NAMES) + [JOINT_NAMES[0]]


def no_points(message):
    """Send a trajectory with no points at all."""
    message.points = []


def two_points(message):
    """Send two points -- a trajectory, which this topic is not."""
    message.points = [message.points[0], JointTrajectoryPoint()]


def six_positions(message):
    """Drop a position: six values for seven joints."""
    message.points[0].positions = list(positions_of(message))[:6]


def eight_positions(message):
    """Add a position: eight values for seven joints."""
    positions = list(positions_of(message))
    message.points[0].positions = positions + [positions[-1]]


def with_velocities(message):
    """Attach a velocity command -- even a zero one is refused."""
    message.points[0].velocities = [0.0]


def with_accelerations(message):
    """Attach an acceleration command -- even a zero one is refused."""
    message.points[0].accelerations = [0.0]


def with_effort(message):
    """Attach an effort command -- even a zero one is refused."""
    message.points[0].effort = [0.0]


def time_from_start_seconds(message):
    """Ask for the point one second from now."""
    message.points[0].time_from_start = Duration(sec=1, nanosec=0)


def time_from_start_nanoseconds(message):
    """Ask for the point one nanosecond from now."""
    message.points[0].time_from_start = Duration(sec=0, nanosec=1)


def unknown_name(message):
    """Name a joint of the other arm."""
    set_name(message, 3, 'panda2_joint4')


def duplicated_name(message):
    """Name joint 1 twice, leaving joint 6 unnamed."""
    set_name(message, 5, JOINT_NAMES[0])


def nan_position(message):
    """Command a NaN position."""
    set_position(message, 2, float('nan'))


def infinite_position(message):
    """Command an infinite position."""
    set_position(message, 4, float('inf'))


def below_lower_fence(message):
    """Command one joint an epsilon below its lower bound."""
    set_position(message, 1, FENCE_LOWER[1] - EPSILON)


def above_upper_fence(message):
    """Command one joint an epsilon above its upper bound."""
    set_position(message, 5, FENCE_UPPER[5] + EPSILON)


def unknown_name_and_nan_at_one_joint(message):
    """Break the name rule and the finiteness rule at the SAME index."""
    set_name(message, 2, 'panda2_joint3')
    set_position(message, 2, float('nan'))


def frame_id_is_empty(message):
    """Assert the server never names a frame."""
    assert message.header.frame_id == ''


def the_stamp_is_a_legal_clock_reading(message):
    """Assert the stamp is non-zero, non-negative and under 1e9 nanoseconds."""
    stamp = message.header.stamp
    assert (stamp.sec, stamp.nanosec) != (0, 0)
    assert stamp.sec >= 0
    assert 0 <= stamp.nanosec < 1000000000


def the_stamp_is_the_reading_it_was_given(message):
    """
    Assert the server stamps with the clock reading handed to it, verbatim.

    Age is therefore a property of the caller's live node clock, not something
    the model can fabricate: it cannot age a stamp or place one in the future.
    """
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (
        GOLDEN_STAMP.sec, GOLDEN_STAMP.nanosec)


def there_are_seven_names(message):
    """Assert the server always names exactly seven joints."""
    assert len(message.joint_names) == defaults.JOINT_COUNT


def there_is_one_point(message):
    """Assert the server always sends exactly one point."""
    assert len(message.points) == 1


def there_are_seven_positions(message):
    """Assert the server always commands exactly seven positions."""
    assert len(message.points[0].positions) == defaults.JOINT_COUNT


def velocities_are_empty(message):
    """Assert the server never commands a velocity."""
    assert len(message.points[0].velocities) == 0


def accelerations_are_empty(message):
    """Assert the server never commands an acceleration."""
    assert len(message.points[0].accelerations) == 0


def effort_is_empty(message):
    """Assert the server never commands an effort."""
    assert len(message.points[0].effort) == 0


def time_from_start_is_zero(message):
    """Assert the server always asks for the point now."""
    assert (message.points[0].time_from_start.sec,
            message.points[0].time_from_start.nanosec) == (0, 0)


def the_names_are_this_arm_s_seven(message):
    """Assert the server names this arm's canonical joints, once each."""
    assert list(message.joint_names) == list(JOINT_NAMES)
    assert len(set(message.joint_names)) == defaults.JOINT_COUNT


def the_positions_are_finite(message):
    """Assert the server never commands a non-finite position."""
    assert all(math.isfinite(value) for value in message.points[0].positions)


def the_positions_are_inside_the_fence(message):
    """Assert every commanded position is inside the fence the model clamps to."""
    for index, value in enumerate(message.points[0].positions):
        assert FENCE_LOWER[index] <= value <= FENCE_UPPER[index]


MUTATIONS = (
    Case('frame_id', RESULT.InvalidFrame, set_frame_id, frame_id_is_empty),
    Case('zero_stamp', RESULT.InvalidStamp, zero_the_stamp,
         the_stamp_is_a_legal_clock_reading),
    Case('negative_stamp_sec', RESULT.InvalidStamp, negative_stamp_seconds,
         the_stamp_is_a_legal_clock_reading),
    Case('stamp_nanosec_1e9', RESULT.InvalidStamp, overflowing_stamp_nanoseconds,
         the_stamp_is_a_legal_clock_reading),
    Case('stamp_older_than_max_age', RESULT.HeaderTooOld, stale_stamp,
         the_stamp_is_the_reading_it_was_given),
    Case('stamp_beyond_future_tolerance', RESULT.HeaderTooFarInFuture, future_stamp,
         the_stamp_is_the_reading_it_was_given),
    Case('six_names', RESULT.InvalidNameCount, six_names, there_are_seven_names),
    Case('eight_names', RESULT.InvalidNameCount, eight_names, there_are_seven_names),
    Case('zero_points', RESULT.InvalidPointCount, no_points, there_is_one_point),
    Case('two_points', RESULT.InvalidPointCount, two_points, there_is_one_point),
    Case('six_positions', RESULT.InvalidPositionCount, six_positions,
         there_are_seven_positions),
    Case('eight_positions', RESULT.InvalidPositionCount, eight_positions,
         there_are_seven_positions),
    Case('velocities', RESULT.VelocityCommandNotAllowed, with_velocities,
         velocities_are_empty),
    Case('accelerations', RESULT.AccelerationCommandNotAllowed, with_accelerations,
         accelerations_are_empty),
    Case('effort', RESULT.EffortCommandNotAllowed, with_effort, effort_is_empty),
    Case('time_from_start_sec', RESULT.InvalidTimeFromStart, time_from_start_seconds,
         time_from_start_is_zero),
    Case('time_from_start_nanosec', RESULT.InvalidTimeFromStart,
         time_from_start_nanoseconds, time_from_start_is_zero),
    Case('unknown_joint', RESULT.DuplicateOrUnknownJoint, unknown_name,
         the_names_are_this_arm_s_seven),
    Case('duplicated_joint', RESULT.DuplicateOrUnknownJoint, duplicated_name,
         the_names_are_this_arm_s_seven),
    Case('nan_position', RESULT.NonfinitePosition, nan_position, the_positions_are_finite),
    Case('infinite_position', RESULT.NonfinitePosition, infinite_position,
         the_positions_are_finite),
    Case('below_lower_fence', RESULT.PositionLimitExceeded, below_lower_fence,
         the_positions_are_inside_the_fence),
    Case('above_upper_fence', RESULT.PositionLimitExceeded, above_upper_fence,
         the_positions_are_inside_the_fence),
)

MUTATION_IDS = tuple(case.name for case in MUTATIONS)


def test_the_mutation_table_covers_every_rejection_reason():
    """All fourteen rejections are produced deliberately by the table above."""
    assert {case.expected for case in MUTATIONS} == set(REJECTION_RESULTS)
    assert len(MUTATION_IDS) == len(set(MUTATION_IDS))


@pytest.mark.parametrize('case', MUTATIONS, ids=MUTATION_IDS)
def test_a_mutated_message_is_rejected_for_its_own_reason(case):
    """Each hand-mutated message draws exactly the rejection it was built for."""
    inbox = enabled_inbox()
    message = golden()
    case.invariant(message)
    case.mutate(message)
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is case.expected
    assert inbox.last_result is case.expected


@pytest.mark.parametrize('case', MUTATIONS, ids=MUTATION_IDS)
def test_the_server_cannot_build_that_mutation(case):
    """The invariant the mutation breaks holds on the server's own message."""
    case.invariant(golden())


@pytest.mark.parametrize('case', MUTATIONS, ids=MUTATION_IDS)
def test_every_rejection_invalidates_the_buffered_target(case):
    """A rejection is a freeze, not a dropped frame: the held target is gone."""
    inbox = enabled_inbox()
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS) == tuple(MEASURED)
    message = golden()
    case.mutate(message)
    later = STEADY_NS + MILLISECOND_NS
    assert inbox.accept(message, ROS_NOW_NS, later) is case.expected
    assert inbox.read_fresh(later) is None
    assert inbox.buffered_target().valid is False


def test_the_model_refuses_to_build_what_accept_would_reject():
    """
    The model raises rather than emit a message it cannot defend.

    The three ``InvalidStamp`` shapes and a wrong joint-name list are refused
    at the source, loudly, instead of becoming a silent freeze on the wire.
    """
    model = seeded_model()
    for stamp in (Time(sec=0, nanosec=0), Time(sec=-1, nanosec=5),
                  Time(sec=GOLDEN_STAMP.sec, nanosec=1000000000)):
        with pytest.raises(JogError):
            model.message(stamp, list(JOINT_NAMES))
    with pytest.raises(JogError):
        model.message(GOLDEN_STAMP, list(JOINT_NAMES[:6]))
    with pytest.raises(JogError):
        model.message(GOLDEN_STAMP, list(JOINT_NAMES) + [JOINT_NAMES[0]])
    with pytest.raises(JogError):
        model.message(GOLDEN_STAMP, [JOINT_NAMES[0]] + list(JOINT_NAMES[1:6]) + ['panda2_joint7'])


def test_the_model_refuses_a_measured_pose_outside_the_fence():
    """Seeding outside the fence is refused, so no message can start there."""
    model = JogTargetModel(ARM_ID, FENCE_LOWER, FENCE_UPPER,
                           step_rad=defaults.JOG_STEP_RAD)
    outside = list(MEASURED)
    outside[3] = FENCE_UPPER[3] + EPSILON
    with pytest.raises(JogError):
        model.seed(outside)


# ---------------------------------------------------------------------------
# Rule order (the C++ returns the FIRST rule that fires)
# ---------------------------------------------------------------------------


ORDERINGS = (
    ('frame_id_before_everything', (set_frame_id, zero_the_stamp, no_points),
     RESULT.InvalidFrame),
    ('stamp_shape_before_age', (zero_the_stamp,), RESULT.InvalidStamp),
    ('age_before_name_count', (stale_stamp, six_names), RESULT.HeaderTooOld),
    ('future_before_name_count', (future_stamp, six_names),
     RESULT.HeaderTooFarInFuture),
    ('name_count_before_point_count', (six_names, no_points), RESULT.InvalidNameCount),
    ('point_count_before_position_count', (two_points, six_positions),
     RESULT.InvalidPointCount),
    ('position_count_before_velocities', (six_positions, with_velocities),
     RESULT.InvalidPositionCount),
    ('velocities_before_accelerations', (with_velocities, with_accelerations, with_effort),
     RESULT.VelocityCommandNotAllowed),
    ('accelerations_before_effort', (with_accelerations, with_effort),
     RESULT.AccelerationCommandNotAllowed),
    ('effort_before_time_from_start', (with_effort, time_from_start_seconds),
     RESULT.EffortCommandNotAllowed),
    ('time_from_start_before_the_joint_loop', (time_from_start_nanoseconds, unknown_name),
     RESULT.InvalidTimeFromStart),
    ('name_lookup_before_finiteness', (unknown_name_and_nan_at_one_joint,),
     RESULT.DuplicateOrUnknownJoint),
    # An infinite position is BOTH non-finite and outside the fence, at one
    # index: the finiteness check runs first, so this is NonfinitePosition.
    ('finiteness_before_the_fence', (infinite_position,), RESULT.NonfinitePosition),
    # Across indices the loop reports whichever joint it reaches first, not
    # whichever rule sits earlier: joint 3's NaN outranks joint 4's bad name.
    ('the_earliest_joint_index_wins', (nan_position, unknown_name),
     RESULT.NonfinitePosition),
)


@pytest.mark.parametrize(
    'mutations,expected', [case[1:] for case in ORDERINGS],
    ids=[case[0] for case in ORDERINGS])
def test_the_first_rule_that_fires_is_the_one_reported(mutations, expected):
    """A message breaking several rules draws the earliest rule's reason."""
    inbox = enabled_inbox()
    message = golden()
    for mutate in mutations:
        mutate(message)
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is expected


def test_a_zero_stamp_is_a_stamp_fault_not_an_age_fault():
    """The zero stamp is ancient, and still reported as InvalidStamp."""
    inbox = enabled_inbox()
    message = golden()
    zero_the_stamp(message)
    assert ROS_NOW_NS > MAX_AGE_NS  # the zero stamp really is older than the window
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is RESULT.InvalidStamp


# ---------------------------------------------------------------------------
# Boundary semantics, as the C++ comparison operators have them
# ---------------------------------------------------------------------------


def test_the_header_age_window_is_inclusive():
    """``age > max_header_age`` rejects: exactly at the window is accepted."""
    inbox = enabled_inbox()
    at_window = golden(nanoseconds=ROS_NOW_NS - MAX_AGE_NS)
    assert inbox.accept(at_window, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    beyond = golden(nanoseconds=ROS_NOW_NS - MAX_AGE_NS - 1)
    assert inbox.accept(beyond, ROS_NOW_NS, STEADY_NS) is RESULT.HeaderTooOld


def test_the_future_tolerance_window_is_inclusive():
    """``-age > future_tolerance`` rejects: exactly at the window is accepted."""
    inbox = enabled_inbox()
    at_window = golden(nanoseconds=ROS_NOW_NS + FUTURE_NS)
    assert inbox.accept(at_window, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    beyond = golden(nanoseconds=ROS_NOW_NS + FUTURE_NS + 1)
    assert inbox.accept(beyond, ROS_NOW_NS, STEADY_NS) is RESULT.HeaderTooFarInFuture


def test_the_largest_legal_nanosec_is_accepted():
    """``nanosec >= 1e9`` rejects, so 999999999 is a legal stamp."""
    inbox = enabled_inbox()
    message = golden()
    message.header.stamp = Time(sec=GOLDEN_STAMP.sec, nanosec=999999999)
    ros_now = int(GOLDEN_STAMP.sec) * 1000000000 + 999999999
    assert inbox.accept(message, ros_now, STEADY_NS) is RESULT.Accepted


@pytest.mark.parametrize('index', range(defaults.JOINT_COUNT))
def test_both_fence_bounds_are_inclusive(index):
    """``position < lower || position > upper`` rejects: the bounds are in."""
    for bound, epsilon_sign in ((FENCE_LOWER[index], -1.0), (FENCE_UPPER[index], 1.0)):
        inbox = enabled_inbox()
        on_bound = golden()
        set_position(on_bound, index, bound)
        assert inbox.accept(on_bound, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
        assert inbox.read_fresh(STEADY_NS)[index] == pytest.approx(bound)
        outside = golden()
        set_position(outside, index, bound + epsilon_sign * EPSILON)
        later = STEADY_NS + MILLISECOND_NS
        assert inbox.accept(outside, ROS_NOW_NS, later) is RESULT.PositionLimitExceeded
        assert inbox.read_fresh(later) is None


def test_the_fence_is_checked_per_joint_not_over_the_whole_vector():
    """A value legal for one joint is still refused on a narrower joint."""
    inbox = enabled_inbox()
    message = golden()
    # 2.0 rad sits inside joint 1's fence and far outside joint 2's.
    set_position(message, 1, 2.0)
    assert FENCE_LOWER[0] <= 2.0 <= FENCE_UPPER[0]
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is RESULT.PositionLimitExceeded


def test_positions_are_reordered_into_configured_joint_order():
    """
    The buffered target follows the CONFIGURED order, not the message order.

    The C++ writes ``target.positions[configured_index]``, so a message that
    lists the joints in another order still lands each value on its own joint.
    """
    inbox = enabled_inbox()
    message = golden()
    reversed_names = list(reversed(JOINT_NAMES))
    reversed_positions = list(reversed(MEASURED))
    message.joint_names = reversed_names
    message.points[0].positions = reversed_positions
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS) == tuple(MEASURED)


# ---------------------------------------------------------------------------
# Freshness and the enable epochs
# ---------------------------------------------------------------------------


def test_a_target_accepted_before_the_enable_is_never_read_fresh():
    """Accepting is not enabling: a pre-enable target is dropped by the enable."""
    inbox = make_inbox()
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.buffered_target().valid is True
    assert inbox.read_fresh(STEADY_NS) is None  # not enabled yet
    inbox.set_enabled(True, ROS_NOW_NS + MILLISECOND_NS, STEADY_NS + MILLISECOND_NS)
    assert inbox.read_fresh(STEADY_NS + 2 * MILLISECOND_NS) is None
    assert inbox.buffered_target().valid is False


def test_after_a_re_enable_the_first_stale_stamped_target_is_ignored():
    """A target stamped before the new ROS epoch is accepted but never used."""
    inbox = enabled_inbox()
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS) is not None

    re_ros = ROS_NOW_NS + 100000000
    re_steady = STEADY_NS + 100000000
    inbox.set_enabled(True, re_ros, re_steady)

    stale = golden(nanoseconds=re_ros - 10 * MILLISECOND_NS)
    assert inbox.accept(stale, re_ros, re_steady + MILLISECOND_NS) is RESULT.Accepted
    assert inbox.read_fresh(re_steady + MILLISECOND_NS) is None

    fresh = golden(nanoseconds=re_ros + MILLISECOND_NS)
    assert inbox.accept(fresh, re_ros + MILLISECOND_NS,
                        re_steady + 2 * MILLISECOND_NS) is RESULT.Accepted
    assert inbox.read_fresh(re_steady + 2 * MILLISECOND_NS) == tuple(MEASURED)


def test_the_ros_epoch_comparison_is_strict():
    """``header_ns > enabled_ros_epoch_ns``: equal is not later enough."""
    inbox = make_inbox()
    inbox.set_enabled(True, ROS_NOW_NS, ENABLE_STEADY_NS)
    at_epoch = golden(nanoseconds=ROS_NOW_NS)
    assert inbox.accept(at_epoch, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS) is None
    after_epoch = golden(nanoseconds=ROS_NOW_NS + 1)
    assert inbox.accept(after_epoch, ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS) == tuple(MEASURED)


def test_the_steady_epoch_comparison_is_strict():
    """``steady_receive_ns > enabled_since_ns``: equal is not later enough."""
    inbox = make_inbox()
    inbox.set_enabled(True, ENABLE_ROS_NS, STEADY_NS)
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS) is None
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS + 1) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS + 1) == tuple(MEASURED)


def test_a_disable_drops_the_held_target_and_stops_every_read():
    """Disabling invalidates the buffer, and re-enabling does not resurrect it."""
    inbox = enabled_inbox()
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS) is not None
    inbox.set_enabled(False, ROS_NOW_NS + 1, STEADY_NS + 1)
    assert inbox.enabled is False
    assert inbox.read_fresh(STEADY_NS + 2) is None
    inbox.set_enabled(True, ROS_NOW_NS + 2, STEADY_NS + 2)
    assert inbox.enabled is True
    assert inbox.read_fresh(STEADY_NS + 3) is None


def test_every_enable_call_advances_the_generation_by_two():
    """Each settled enable/disable is an even generation, as the C++ has it."""
    inbox = make_inbox()
    assert inbox.enable_generation == 0
    inbox.set_enabled(True, ROS_NOW_NS, STEADY_NS)
    assert inbox.enable_generation == 2
    assert inbox.enabled_ros_epoch_ns == ROS_NOW_NS
    assert inbox.enabled_since_steady_ns == STEADY_NS
    inbox.set_enabled(False, ROS_NOW_NS + 1, STEADY_NS + 1)
    assert inbox.enable_generation == 4
    # The epochs move on a disable too, so a target buffered during the
    # disabled window can never be read after the next enable either.
    assert inbox.enabled_ros_epoch_ns == ROS_NOW_NS + 1
    assert inbox.enabled_since_steady_ns == STEADY_NS + 1


def test_the_watchdog_window_is_inclusive_and_the_future_is_refused():
    """A receipt exactly one watchdog old is still fresh; older is the freeze."""
    inbox = enabled_inbox()
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert inbox.read_fresh(STEADY_NS + WATCHDOG_NS) == tuple(MEASURED)
    assert inbox.read_fresh(STEADY_NS + WATCHDOG_NS + 1) is None
    assert inbox.read_fresh(STEADY_NS - 1) is None
    # The epoch answer is unchanged by the clock: the target is still buffered.
    assert inbox.read_fresh() == tuple(MEASURED)


# ---------------------------------------------------------------------------
# ControllerInactive: the one result that is not a rejection
# ---------------------------------------------------------------------------


def test_controller_inactive_leaves_the_buffered_target_alone():
    """The pre-inbox gate answers without touching what is already held."""
    inbox = enabled_inbox()
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    held = inbox.read_fresh(STEADY_NS)
    assert held == tuple(MEASURED)
    later = STEADY_NS + MILLISECOND_NS
    assert inbox.accept(golden(), ROS_NOW_NS, later,
                        controller_active=False) is RESULT.ControllerInactive
    assert inbox.read_fresh(later) == held
    assert inbox.buffered_target().valid is True


@pytest.mark.parametrize('case', MUTATIONS, ids=MUTATION_IDS)
def test_controller_inactive_is_answered_before_any_rule_runs(case):
    """Even a message breaking a rule gets ControllerInactive, and freezes nothing."""
    inbox = enabled_inbox()
    assert inbox.accept(golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    message = golden()
    case.mutate(message)
    later = STEADY_NS + MILLISECOND_NS
    assert inbox.accept(message, ROS_NOW_NS, later,
                        controller_active=False) is RESULT.ControllerInactive
    assert inbox.read_fresh(later) == tuple(MEASURED)


def test_accept_target_gates_before_the_inbox_and_reuses_one_enumerator():
    """
    ``acceptTarget``: inactive first, then a bad arm index, then the inbox.

    An out-of-range arm index answers ``DuplicateOrUnknownJoint`` -- the C++
    genuinely reuses that enumerator -- and, like the inactive gate, never
    reaches an inbox, so neither can invalidate anything.
    """
    inboxes = [enabled_inbox(), enabled_inbox()]
    assert accept_target(inboxes, 0, golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted
    assert accept_target(inboxes, 1, golden(), ROS_NOW_NS, STEADY_NS) is RESULT.Accepted

    later = STEADY_NS + MILLISECOND_NS
    assert accept_target(inboxes, 0, golden(), ROS_NOW_NS, later,
                         controller_active=False) is RESULT.ControllerInactive
    assert accept_target(inboxes, 2, golden(), ROS_NOW_NS,
                         later) is RESULT.DuplicateOrUnknownJoint
    assert accept_target(inboxes, -1, golden(), ROS_NOW_NS,
                         later) is RESULT.DuplicateOrUnknownJoint

    for inbox in inboxes:
        assert inbox.results() == (RESULT.Accepted,)
        assert inbox.read_fresh(later) == tuple(MEASURED)


def test_the_result_log_is_the_only_place_a_rejection_is_visible():
    """
    Nothing about a rejection reaches the wire; the log is a test-only view.

    The controller answers a target with no message, no service reply and no
    diagnostic. That is why the server has to be correct by construction and
    why this file exists.
    """
    inbox = enabled_inbox()
    message = golden()
    set_frame_id(message)
    assert inbox.accept(message, ROS_NOW_NS, STEADY_NS) is RESULT.InvalidFrame
    assert inbox.results() == (RESULT.InvalidFrame,)
    assert inbox.result_counts() == {RESULT.InvalidFrame: 1}
    inbox.clear_results()
    assert inbox.results() == ()
    assert inbox.last_result is None
    # Clearing the log does not restore what the rejection destroyed.
    assert inbox.read_fresh(STEADY_NS) is None
