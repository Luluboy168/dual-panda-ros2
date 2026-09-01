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
Tests for franka_web.jog: the held target, the fence clamp and the message.

The claims under test are the ones that keep the arm moving. Plan section 0.6
says the impedance controller answers every malformed target by dropping the
one it had buffered, silently, which freezes the arm under its watchdog -- so
these tests assert the message field by field rather than assert that one gets
built, they assert that a value outside the fence is clamped and *reported*
rather than published, and they assert that the model refuses every input the
controller would have refused: a stale seed, a wrong name list, a dead clock.

Nothing here touches ROS beyond constructing message objects: no init, no node,
no clock. The stamp is a plain ``builtin_interfaces.msg.Time`` the test builds.
"""

import dataclasses
import math

from builtin_interfaces.msg import Duration, Time
from franka_web import defaults
from franka_web.jog import JogError, JogTargetModel, StepResult
import pytest

ARM_ID = 'panda1'

# A per-joint fence with seven different widths and an entirely negative joint
# 4, so an off-by-one that mixes two joints up cannot pass unnoticed.
FENCE_LOWER = (-2.80, -1.70, -2.80, -3.00, -2.80, -0.01, -2.80)
FENCE_UPPER = (2.80, 1.70, 2.80, -0.10, 2.80, 3.70, 2.80)

# A pose comfortably inside every joint's fence: at least one step from each
# boundary, so a plain step never clamps from here.
POSE = (0.0, -0.30, 0.10, -1.60, -0.20, 1.80, 0.70)

JOINT_NAMES = tuple('panda1_joint{}'.format(index) for index in range(1, 8))
JOINTS = tuple(range(defaults.JOINT_COUNT))
STEP = defaults.JOG_STEP_RAD


def make_model(**kwargs):
    """Return a model over the test fence, with any argument overridden."""
    arguments = {
        'arm_id': ARM_ID,
        'fence_lower': FENCE_LOWER,
        'fence_upper': FENCE_UPPER,
        # Required, never defaulted: the step comes from the configuration,
        # and no caller may build a model carrying a baked-in one by accident.
        'step_rad': defaults.JOG_STEP_RAD,
    }
    arguments.update(kwargs)
    return JogTargetModel(**arguments)


def pose_with(index, value):
    """Return POSE with one joint replaced, leaving the rest inside the fence."""
    values = list(POSE)
    values[index] = value
    return tuple(values)


@pytest.fixture()
def model():
    """Return an unseeded model over the test fence."""
    return make_model()


@pytest.fixture()
def seeded(model):
    """Return a model already seeded at POSE."""
    model.seed(POSE)
    return model


@pytest.fixture()
def stamp():
    """Return a plausible non-zero node-clock stamp."""
    return Time(sec=1717171717, nanosec=250000000)


class TestConstruction:
    """The fence and the step are validated once, at construction."""

    def test_a_fresh_model_is_unseeded(self, model):
        """Nothing is held until the enable path seeds a measured pose."""
        assert model.seeded is False
        assert model.target is None

    def test_it_reports_its_arm_and_canonical_joint_names(self, model):
        """The names are the controller's pinned ``<arm_id>_jointN`` order."""
        assert model.arm_id == ARM_ID
        assert model.joint_names == JOINT_NAMES
        assert len(model.joint_names) == defaults.JOINT_COUNT

    def test_the_default_step_is_the_configured_one(self, model):
        """The 2 degree step comes from config, never a literal in the model."""
        assert model.step_rad == defaults.JOG_STEP_RAD

    def test_it_keeps_the_fence_as_seven_floats_each(self, model):
        """The fence is exposed for the UI's per-joint margin bars."""
        assert model.fence_lower == FENCE_LOWER
        assert model.fence_upper == FENCE_UPPER
        assert len(model.fence_lower) == defaults.JOINT_COUNT
        assert len(model.fence_upper) == defaults.JOINT_COUNT
        assert all(isinstance(value, float) for value in model.fence_lower)
        assert all(isinstance(value, float) for value in model.fence_upper)

    def test_integer_fence_values_are_accepted_as_floats(self):
        """A fence written as ints in YAML is still a fence."""
        built = make_model(fence_lower=[-3] * 7, fence_upper=[3] * 7)
        assert built.fence_lower == (-3.0,) * 7
        assert all(isinstance(value, float) for value in built.fence_upper)

    def test_a_custom_step_is_honoured(self):
        """A caller may pin a different step; the model does not overrule it."""
        built = make_model(step_rad=0.01)
        assert built.step_rad == 0.01

    @pytest.mark.parametrize('arm_id', ['', None, 7, b'panda1', ('panda1',)])
    def test_a_bad_arm_id_is_refused(self, arm_id):
        """The arm id builds the joint names, so it must be a real name."""
        with pytest.raises(JogError, match='arm_id'):
            make_model(arm_id=arm_id)

    @pytest.mark.parametrize('length', [0, 1, 6, 8, 14])
    def test_a_fence_of_the_wrong_length_is_refused(self, length):
        """Seven joints, seven bounds -- 14 (both arms) included."""
        with pytest.raises(JogError, match='exactly 7 entries'):
            make_model(fence_lower=[-1.0] * length)
        with pytest.raises(JogError, match='exactly 7 entries'):
            make_model(fence_upper=[1.0] * length)

    @pytest.mark.parametrize('bad', ['abcdefg', 3.0, None, 7])
    def test_a_fence_that_is_not_a_sequence_is_refused(self, bad):
        """A string of seven characters is not a fence either."""
        with pytest.raises(JogError, match='fence_lower'):
            make_model(fence_lower=bad)

    @pytest.mark.parametrize('index', JOINTS)
    @pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
    def test_a_nonfinite_fence_bound_is_refused_by_name(self, index, bad):
        """A NaN bound would make every comparison false; it is named and refused."""
        lower = list(FENCE_LOWER)
        lower[index] = bad
        with pytest.raises(JogError, match='panda1_joint{} is not finite'.format(index + 1)):
            make_model(fence_lower=lower)
        upper = list(FENCE_UPPER)
        upper[index] = bad
        with pytest.raises(JogError, match='panda1_joint{} is not finite'.format(index + 1)):
            make_model(fence_upper=upper)

    @pytest.mark.parametrize('bad', [None, 'x', True, False, [1.0]])
    def test_a_non_numeric_fence_bound_is_refused_by_name(self, bad):
        """``True`` is not a bound, however happily Python would add it."""
        lower = list(FENCE_LOWER)
        lower[3] = bad
        with pytest.raises(JogError, match='panda1_joint4 is not a number'):
            make_model(fence_lower=lower)

    @pytest.mark.parametrize('index', JOINTS)
    def test_an_inverted_or_empty_fence_is_refused_by_name(self, index):
        """``lower < upper`` strictly: a zero-width fence has no inside."""
        equal = list(FENCE_LOWER)
        equal[index] = FENCE_UPPER[index]
        with pytest.raises(JogError, match='panda1_joint{} '.format(index + 1)):
            make_model(fence_lower=equal)
        inverted = list(FENCE_LOWER)
        inverted[index] = FENCE_UPPER[index] + 1.0
        with pytest.raises(JogError, match='strictly below'):
            make_model(fence_lower=inverted)

    def test_every_inverted_joint_is_named_at_once(self):
        """One exception lists all the offenders, not just the first."""
        with pytest.raises(JogError) as caught:
            make_model(fence_lower=list(FENCE_UPPER))
        text = str(caught.value)
        for name in JOINT_NAMES:
            assert name in text

    @pytest.mark.parametrize('bad', [0.0, -0.1, float('nan'), float('inf')])
    def test_a_non_positive_or_nonfinite_step_is_refused(self, bad):
        """A zero step is a button that does nothing; an infinite one is worse."""
        with pytest.raises(JogError, match='step_rad'):
            make_model(step_rad=bad)

    @pytest.mark.parametrize('bad', [None, '0.03', True])
    def test_a_non_numeric_step_is_refused(self, bad):
        """``True`` would silently become a one-radian step."""
        with pytest.raises(JogError, match='step_rad must be a number'):
            make_model(step_rad=bad)


class TestSeed:
    """seed() is the plan section 5.4 fence-vs-pose check, in code."""

    def test_seeding_holds_the_measured_pose(self, model):
        """The held target starts life as the pose, exactly."""
        model.seed(POSE)
        assert model.seeded is True
        assert model.target == POSE
        assert len(model.target) == defaults.JOINT_COUNT
        assert isinstance(model.target, tuple)
        assert all(isinstance(value, float) for value in model.target)

    def test_integer_measurements_become_floats(self):
        """A pose of ints is held as floats, so the message carries doubles."""
        built = make_model(fence_lower=[-3.0] * 7, fence_upper=[3.0] * 7)
        built.seed([0] * 7)
        assert built.target == (0.0,) * 7
        assert all(isinstance(value, float) for value in built.target)

    @pytest.mark.parametrize('index', JOINTS)
    def test_a_pose_on_either_boundary_is_inside_the_fence(self, model, index):
        """``accept`` rejects only ``< lower`` or ``> upper``; the edge is in."""
        model.seed(pose_with(index, FENCE_LOWER[index]))
        assert model.target[index] == FENCE_LOWER[index]
        model.seed(pose_with(index, FENCE_UPPER[index]))
        assert model.target[index] == FENCE_UPPER[index]

    @pytest.mark.parametrize('index', JOINTS)
    def test_a_pose_below_the_lower_fence_is_refused_by_name(self, model, index):
        """Below the fence on any one joint refuses the whole seed."""
        outside = pose_with(index, FENCE_LOWER[index] - 1e-6)
        with pytest.raises(JogError, match='panda1_joint{} '.format(index + 1)) as caught:
            model.seed(outside)
        assert 'outside the fence' in str(caught.value)
        assert model.seeded is False

    @pytest.mark.parametrize('index', JOINTS)
    def test_a_pose_above_the_upper_fence_is_refused_by_name(self, model, index):
        """And above it, per joint, per side -- both boundaries are tested."""
        outside = pose_with(index, FENCE_UPPER[index] + 1e-6)
        with pytest.raises(JogError, match='panda1_joint{} '.format(index + 1)):
            model.seed(outside)
        assert model.seeded is False

    def test_every_out_of_fence_joint_is_named_at_once(self, model):
        """The operator gets the whole picture, not the first bad joint."""
        outside = tuple(upper + 1.0 for upper in FENCE_UPPER)
        with pytest.raises(JogError) as caught:
            model.seed(outside)
        text = str(caught.value)
        for name in JOINT_NAMES:
            assert name in text

    @pytest.mark.parametrize('index', JOINTS)
    @pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
    def test_a_nonfinite_measurement_is_refused_by_name(self, model, index, bad):
        """``NonfinitePosition`` is a rejection; a NaN never reaches the wire."""
        with pytest.raises(JogError, match='panda1_joint{} is not finite'.format(index + 1)):
            model.seed(pose_with(index, bad))
        assert model.seeded is False

    @pytest.mark.parametrize('bad', [None, 'x', True, False])
    def test_a_non_numeric_measurement_is_refused_by_name(self, model, bad):
        """A missing joint arriving as ``None`` is refused, not coerced."""
        with pytest.raises(JogError, match='panda1_joint5 is not a number'):
            model.seed(pose_with(4, bad))
        assert model.seeded is False

    @pytest.mark.parametrize('length', [0, 6, 8, 14])
    def test_a_measurement_of_the_wrong_length_is_refused(self, model, length):
        """14 is what ``/franka/joint_states`` carries in dual mode."""
        with pytest.raises(JogError, match='exactly 7 entries'):
            model.seed([0.0] * length)
        assert model.seeded is False

    def test_a_measurement_that_is_not_a_sequence_is_refused(self, model):
        """A bare float is not a pose."""
        with pytest.raises(JogError, match='measured pose'):
            model.seed(0.0)
        assert model.seeded is False

    def test_reseeding_replaces_the_whole_target(self, seeded):
        """Every enable re-seeds (plan section 0.6): the old target is gone."""
        seeded.step(0, 1)
        other = tuple(value + 0.05 for value in POSE)
        seeded.seed(other)
        assert seeded.target == other

    def test_a_refused_seed_leaves_the_held_target_untouched(self, seeded):
        """A refusal is never a partial mutation."""
        with pytest.raises(JogError):
            seeded.seed(pose_with(2, FENCE_UPPER[2] + 1.0))
        assert seeded.target == POSE
        assert seeded.seeded is True


class TestInvalidate:
    """invalidate() is the disable / fault side of the enable re-seed rule."""

    def test_invalidate_forgets_the_seed(self, seeded):
        """Nothing may publish a target that predates the next enable."""
        seeded.invalidate()
        assert seeded.seeded is False
        assert seeded.target is None

    def test_invalidate_is_idempotent(self, model):
        """Calling it on an unseeded model is a no-op, not an error."""
        model.invalidate()
        model.invalidate()
        assert model.seeded is False

    def test_the_model_is_usable_again_after_a_reseed(self, seeded):
        """Disable then enable: invalidate, then seed, then jog."""
        seeded.invalidate()
        seeded.seed(POSE)
        assert seeded.step(0, 1).target[0] == POSE[0] + STEP


class TestStep:
    """One button press moves one joint by exactly one step, or reports a clamp."""

    @pytest.mark.parametrize('index', JOINTS)
    @pytest.mark.parametrize('direction', [-1, 1])
    def test_it_moves_exactly_one_configured_step(self, seeded, index, direction):
        """The magnitude is ``defaults.JOG_STEP_RAD``, to the last bit."""
        result = seeded.step(index, direction)
        assert result.target[index] == POSE[index] + direction * STEP

    @pytest.mark.parametrize('index', JOINTS)
    def test_it_moves_only_that_joint(self, seeded, index):
        """Every other joint is bit-for-bit the value it already held."""
        result = seeded.step(index, 1)
        for joint in JOINTS:
            if joint != index:
                assert result.target[joint] == POSE[joint]

    def test_the_result_is_a_frozen_seven_by_seven_pair(self, seeded):
        """The result is the API section 6.13 body: two frozen 7-element tuples."""
        result = seeded.step(3, -1)
        assert isinstance(result, StepResult)
        assert isinstance(result.target, tuple)
        assert isinstance(result.clamped, tuple)
        assert len(result.target) == defaults.JOINT_COUNT
        assert len(result.clamped) == defaults.JOINT_COUNT
        assert all(isinstance(value, float) for value in result.target)
        assert all(isinstance(value, bool) for value in result.clamped)
        with pytest.raises(dataclasses.FrozenInstanceError):
            result.target = POSE

    def test_the_held_target_is_updated(self, seeded):
        """The model holds what it just returned; the next step builds on it."""
        result = seeded.step(1, 1)
        assert seeded.target == result.target
        again = seeded.step(1, 1)
        assert again.target[1] == result.target[1] + STEP

    def test_an_unclamped_step_reports_no_clamp(self, seeded):
        """Well inside the fence, nothing is flashed."""
        result = seeded.step(5, 1)
        assert result.clamped == (False,) * defaults.JOINT_COUNT

    @pytest.mark.parametrize('index', JOINTS)
    def test_it_clamps_to_the_upper_fence_and_says_so(self, model, index):
        """The clamp lands exactly on the bound the controller accepts."""
        model.seed(pose_with(index, FENCE_UPPER[index] - STEP / 2.0))
        result = model.step(index, 1)
        assert result.target[index] == FENCE_UPPER[index]
        assert result.clamped[index] is True
        assert sum(result.clamped) == 1

    @pytest.mark.parametrize('index', JOINTS)
    def test_it_clamps_to_the_lower_fence_and_says_so(self, model, index):
        """Both directions clamp, and the mask names the joint that clamped."""
        model.seed(pose_with(index, FENCE_LOWER[index] + STEP / 2.0))
        result = model.step(index, -1)
        assert result.target[index] == FENCE_LOWER[index]
        assert result.clamped[index] is True
        assert sum(result.clamped) == 1

    @pytest.mark.parametrize('index', JOINTS)
    def test_stepping_onto_the_boundary_is_not_a_clamp(self, model, index):
        """The clamp mask means "the fence cut this short", nothing looser."""
        model.seed(pose_with(index, FENCE_UPPER[index] - STEP))
        result = model.step(index, 1)
        assert result.clamped == (False,) * defaults.JOINT_COUNT
        assert result.target[index] == FENCE_UPPER[index] - STEP + STEP

    def test_pressing_on_at_the_boundary_keeps_clamping(self, model):
        """A held-down button parks on the fence and keeps reporting it."""
        model.seed(pose_with(3, FENCE_UPPER[3]))
        for _ in range(5):
            result = model.step(3, 1)
            assert result.target[3] == FENCE_UPPER[3]
            assert result.clamped[3] is True

    def test_a_clamped_joint_can_still_move_back(self, model):
        """The clamp is a wall, not a latch."""
        model.seed(pose_with(0, FENCE_UPPER[0]))
        model.step(0, 1)
        result = model.step(0, -1)
        assert result.clamped == (False,) * defaults.JOINT_COUNT
        assert result.target[0] == FENCE_UPPER[0] - STEP

    def test_the_target_never_leaves_the_fence(self, model):
        """A long random-ish walk stays inside every bound, every time."""
        model.seed(POSE)
        for round_index in range(200):
            index = round_index % defaults.JOINT_COUNT
            direction = 1 if (round_index // defaults.JOINT_COUNT) % 3 else -1
            result = model.step(index, direction)
            for joint in JOINTS:
                assert FENCE_LOWER[joint] <= result.target[joint] <= FENCE_UPPER[joint]

    @pytest.mark.parametrize(
        'bad', [-1, 7, 8, 100, True, False, 3.0, 0.0, None, '3', [3]])
    def test_a_bad_joint_index_is_refused(self, seeded, bad):
        """``True`` is not joint 1 and ``3.0`` is not joint 3."""
        with pytest.raises(JogError, match='joint_index'):
            seeded.step(bad, 1)
        assert seeded.target == POSE

    @pytest.mark.parametrize(
        'bad', [0, 2, -2, 1.0, -1.0, True, False, None, '1', '+1', [1]])
    def test_a_bad_direction_is_refused(self, seeded, bad):
        """The API's ``direction`` is a sign, never a magnitude."""
        with pytest.raises(JogError, match='direction'):
            seeded.step(0, bad)
        assert seeded.target == POSE

    def test_stepping_unseeded_is_refused(self, model):
        """No seed, no target: there is nothing to step from."""
        with pytest.raises(JogError, match='not seeded'):
            model.step(0, 1)

    def test_stepping_after_invalidate_is_refused(self, seeded):
        """Disable clears the seed, and the next press is refused until enable."""
        seeded.invalidate()
        with pytest.raises(JogError, match='not seeded'):
            seeded.step(0, 1)

    def test_arguments_are_validated_before_the_seed(self, model):
        """A bad index on an unseeded model reports the index, deterministically."""
        with pytest.raises(JogError, match='joint_index'):
            model.step(99, 1)
        with pytest.raises(JogError, match='direction'):
            model.step(0, 0)


class TestDeterminism:
    """The same presses from the same seed always give the same floats."""

    def test_replaying_a_sequence_gives_identical_targets(self):
        """Two models, one script, bit-for-bit equal targets at every step."""
        script = [(0, 1), (0, 1), (3, -1), (6, 1), (3, -1), (1, -1), (6, 1), (5, 1)]
        first = make_model()
        second = make_model()
        first.seed(POSE)
        second.seed(POSE)
        for index, direction in script:
            left = first.step(index, direction)
            right = second.step(index, direction)
            assert left.target == right.target
            assert left.clamped == right.clamped
        assert first.target == second.target

    def test_a_reseed_restarts_the_same_arithmetic(self, model):
        """Re-seeding to the same pose reproduces the same trajectory exactly."""
        model.seed(POSE)
        first = [model.step(2, 1).target for _ in range(20)]
        model.seed(POSE)
        second = [model.step(2, 1).target for _ in range(20)]
        assert first == second

    def test_repeated_steps_are_plain_ieee_addition(self, model):
        """No accumulator, no drift of its own: the target is the running sum."""
        model.seed(POSE)
        expected = POSE[4]
        for _ in range(10):
            expected = expected + STEP
            assert model.step(4, 1).target[4] == expected

    def test_a_clamped_step_is_exactly_the_bound(self, model):
        """The clamped value is the fence number itself, not a nearby float."""
        model.seed(pose_with(6, FENCE_UPPER[6] - 1e-9))
        result = model.step(6, 1)
        assert result.target[6] == FENCE_UPPER[6]
        assert repr(result.target[6]) == repr(FENCE_UPPER[6])


class TestMessage:
    """Every field of the message plan section 5.2 pins, asserted one by one."""

    def test_the_header_carries_the_given_stamp_and_no_frame(self, seeded, stamp):
        """``frame_id`` must be empty or ``accept`` returns ``InvalidFrame``."""
        message = seeded.message(stamp, JOINT_NAMES)
        assert message.header.frame_id == ''
        assert message.header.stamp.sec == stamp.sec
        assert message.header.stamp.nanosec == stamp.nanosec

    def test_the_joint_names_are_the_seven_canonical_ones(self, seeded, stamp):
        """Seven names, in order, or ``InvalidNameCount``/``DuplicateOrUnknownJoint``."""
        message = seeded.message(stamp, JOINT_NAMES)
        assert message.joint_names == list(JOINT_NAMES)
        assert len(message.joint_names) == defaults.JOINT_COUNT

    def test_there_is_exactly_one_point(self, seeded, stamp):
        """``points.size() == 1`` or ``InvalidPointCount``."""
        message = seeded.message(stamp, JOINT_NAMES)
        assert len(message.points) == 1

    def test_the_positions_are_the_held_target(self, seeded, stamp):
        """Seven finite in-fence doubles, in the joint-name order."""
        seeded.step(2, 1)
        message = seeded.message(stamp, JOINT_NAMES)
        positions = list(message.points[0].positions)
        assert positions == list(seeded.target)
        assert len(positions) == defaults.JOINT_COUNT
        assert all(math.isfinite(value) for value in positions)

    @pytest.mark.parametrize('field', ['velocities', 'accelerations', 'effort'])
    def test_the_command_fields_are_empty(self, seeded, stamp, field):
        """Each non-empty one is its own rejection, and its own arm freeze."""
        point = seeded.message(stamp, JOINT_NAMES).points[0]
        values = getattr(point, field)
        assert len(values) == 0
        assert list(values) == []

    def test_the_time_from_start_is_zero(self, seeded, stamp):
        """Both halves zero, or ``InvalidTimeFromStart``."""
        point = seeded.message(stamp, JOINT_NAMES).points[0]
        assert point.time_from_start.sec == 0
        assert point.time_from_start.nanosec == 0
        assert point.time_from_start == Duration(sec=0, nanosec=0)

    def test_a_tuple_of_names_is_accepted(self, seeded, stamp):
        """The names may arrive as any 7-sequence; they are copied into a list."""
        message = seeded.message(stamp, list(JOINT_NAMES))
        assert isinstance(message.joint_names, list)
        assert message.joint_names == list(JOINT_NAMES)

    def test_each_call_builds_an_independent_message(self, seeded, stamp):
        """The timer publishes 20 of these a second; none may share state."""
        first = seeded.message(stamp, JOINT_NAMES)
        second = seeded.message(stamp, JOINT_NAMES)
        assert first is not second
        assert first.points[0] is not second.points[0]
        first.points[0].positions = [9.0] * 7
        assert list(second.points[0].positions) == list(seeded.target)

    def test_the_message_tracks_later_steps(self, seeded, stamp):
        """What is published is always the current held target."""
        before = list(seeded.message(stamp, JOINT_NAMES).points[0].positions)
        seeded.step(0, 1)
        after = list(seeded.message(stamp, JOINT_NAMES).points[0].positions)
        assert after[0] == before[0] + STEP
        assert after[1:] == before[1:]

    def test_a_clamped_target_is_still_inside_the_fence_on_the_wire(self, model, stamp):
        """``PositionLimitExceeded`` is unreachable from a clamped target."""
        model.seed(pose_with(3, FENCE_UPPER[3] - STEP / 2.0))
        model.step(3, 1)
        positions = list(model.message(stamp, JOINT_NAMES).points[0].positions)
        for joint in JOINTS:
            assert FENCE_LOWER[joint] <= positions[joint] <= FENCE_UPPER[joint]

    @pytest.mark.parametrize('names', [
        JOINT_NAMES[:6],
        JOINT_NAMES + ('panda1_joint8',),
        tuple(reversed(JOINT_NAMES)),
        tuple('panda2_joint{}'.format(index) for index in range(1, 8)),
        tuple('joint{}'.format(index) for index in range(1, 8)),
        tuple('Panda1_Joint{}'.format(index) for index in range(1, 8)),
        ('panda1_joint1',) * 7,
        (),
    ])
    def test_wrong_joint_names_are_refused(self, seeded, stamp, names):
        """Short, long, reordered, other-arm, duplicated -- every one refused."""
        with pytest.raises(JogError):
            seeded.message(stamp, names)

    def test_a_name_list_that_is_not_a_sequence_is_refused(self, seeded, stamp):
        """A bare string is not a name list, however long it is."""
        with pytest.raises(JogError, match='joint_names'):
            seeded.message(stamp, 'panda1_joint1')

    @pytest.mark.parametrize('bad', [0, 1.5, None, 'now', Duration(sec=1, nanosec=0)])
    def test_a_stamp_that_is_not_a_time_is_refused(self, seeded, bad):
        """``rclpy.time.Time`` is not the message type; ``.to_msg()`` is."""
        with pytest.raises(JogError, match='builtin_interfaces.msg.Time'):
            seeded.message(bad, JOINT_NAMES)

    @pytest.mark.parametrize('bad', [
        Time(sec=0, nanosec=0),
        Time(sec=-1, nanosec=0),
        Time(sec=-5, nanosec=250000000),
        Time(sec=1, nanosec=1000000000),
        Time(sec=1, nanosec=4000000000),
    ])
    def test_a_malformed_stamp_is_refused(self, seeded, bad):
        """A dead clock reads zero; ``InvalidStamp`` would freeze the arm silently."""
        with pytest.raises(JogError, match='stamp must be a non-zero clock reading'):
            seeded.message(bad, JOINT_NAMES)

    def test_the_smallest_valid_stamp_is_accepted(self, seeded):
        """One nanosecond past the epoch is non-zero, and that is the only rule."""
        message = seeded.message(Time(sec=0, nanosec=1), JOINT_NAMES)
        assert message.header.stamp.nanosec == 1

    def test_building_a_message_unseeded_is_refused(self, model, stamp):
        """Nothing is published between disable and the next enable."""
        with pytest.raises(JogError, match='not seeded'):
            model.message(stamp, JOINT_NAMES)

    def test_building_a_message_after_invalidate_is_refused(self, seeded, stamp):
        """The fault path calls invalidate; the timer must then refuse."""
        seeded.invalidate()
        with pytest.raises(JogError, match='not seeded'):
            seeded.message(stamp, JOINT_NAMES)


class TestSecondArm:
    """The model is per arm, and the names follow the arm it was built for."""

    def test_the_second_arm_gets_its_own_names(self, stamp):
        """One-arm mode may carry panda2 in the ``arm_1`` slot (plan section 0.6)."""
        other = JogTargetModel('panda2', FENCE_LOWER, FENCE_UPPER,
                               step_rad=defaults.JOG_STEP_RAD)
        other.seed(POSE)
        expected = ['panda2_joint{}'.format(index) for index in range(1, 8)]
        assert list(other.joint_names) == expected
        assert other.message(stamp, expected).joint_names == expected

    def test_one_arms_names_are_refused_by_the_other(self, seeded, stamp):
        """Crossing the arms is ``DuplicateOrUnknownJoint``, so it is refused here."""
        with pytest.raises(JogError, match='joint_names must be exactly'):
            seeded.message(stamp, ['panda2_joint{}'.format(index) for index in range(1, 8)])

    def test_two_models_hold_independent_targets(self):
        """Two arms, two targets: stepping one never moves the other."""
        left = JogTargetModel('panda1', FENCE_LOWER, FENCE_UPPER,
                              step_rad=defaults.JOG_STEP_RAD)
        right = JogTargetModel('panda2', FENCE_LOWER, FENCE_UPPER,
                               step_rad=defaults.JOG_STEP_RAD)
        left.seed(POSE)
        right.seed(POSE)
        left.step(0, 1)
        assert right.target == POSE
        assert left.target != right.target


class TestConfiguredStep:
    """The step magnitude is the caller's, never a module default."""

    def test_the_step_magnitude_comes_from_the_caller_not_a_module_default(self):
        """
        ``step_rad`` is REQUIRED, which is what makes it structurally config-sourced.

        The only production caller is the session's start path, which passes
        ``settings.jog_step_rad``; there is deliberately no way to build a
        model carrying the baked-in default by accident.
        """
        with pytest.raises(TypeError):
            JogTargetModel(ARM_ID, FENCE_LOWER, FENCE_UPPER)
        model = make_model(step_rad=0.01)
        model.seed(POSE)
        assert model.step_rad == 0.01
        result = model.step(0, 1)
        assert result.target[0] == pytest.approx(POSE[0] + 0.01)
