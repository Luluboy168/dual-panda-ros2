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
The Apply decision module, driven with a stub cell model and no ROS at all.

``travel.py`` is the half of Apply that decides WHETHER and WHERE. Everything
below runs it directly: no session, no bridge, no HTTP, no robot, no ROS
client library. What it proves is the shape of the executed path (one scalar
for all seven joints, the last waypoint exact, every waypoint in the fence),
the order and the content of the refusal ladder, and -- the case this file
exists for -- that the check is handed the MEASURED pose as its first
waypoint, so the segment the arm closes at the start of a travel is checked
rather than assumed.
"""

import inspect
import math

from franka_web import defaults, ghost, travel
from franka_web.workspace import WorkspaceModelError
import pytest
from support.fake_checker import CheckResult, Contact, FakeCellModel

#: A comfortable pose, well inside the fence at every joint.
HOME = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)

#: The shipped per-joint target-rate limit of both profiles.
VELOCITY = (0.1,) * defaults.JOINT_COUNT

#: What the CONTROLLER can ramp through in one stream period, per joint.
#: Deliberately NOT computed from APPLY_SPEED_FRACTION: a budget derived from
#: the very constant under test would move with it, and a fraction raised past
#: 1.0 would pass its own assertion.
CONTROLLER_STEP = VELOCITY[0] / defaults.JOG_STREAM_HZ

DEG = math.pi / 180.0

#: "argument not given", so a test can pass ``None`` as a real value.
UNSET = object()


def fence(margin=1.0):
    """Return a symmetric fence ``margin`` radians either side of HOME."""
    return ([value - margin for value in HOME],
            [value + margin for value in HOME])


def moved(pose, joint=3, delta=0.2):
    """Return ``pose`` with one joint displaced."""
    out = list(pose)
    out[joint] += delta
    return tuple(out)


def plan(model=None, q_held=HOME, q_measured=UNSET, q_goal=UNSET,
         velocity=VELOCITY, fence_pair=None, co_arm_id='panda2',
         co_arm_q=HOME, enable_epoch=1, cancel_gen=2, arm_id='panda1'):
    """Call ``plan_travel`` with sensible defaults for one panda1 travel."""
    lower, upper = fence_pair or fence()
    return travel.plan_travel(
        arm_id=arm_id,
        q_held=q_held,
        q_measured=HOME if q_measured is UNSET else q_measured,
        q_goal=moved(HOME) if q_goal is UNSET else q_goal,
        fence_lower=lower, fence_upper=upper,
        max_target_velocity=velocity,
        model=model if model is not None else FakeCellModel(),
        co_arm_id=co_arm_id, co_arm_q=co_arm_q,
        enable_epoch=enable_epoch, cancel_gen=cancel_gen)


def refusal(**kwargs):
    """Return the TravelError ``plan_travel`` raises for these arguments."""
    with pytest.raises(travel.TravelError) as excinfo:
        plan(**kwargs)
    return excinfo.value


def contact(kind='cross_arm', a='panda1_link5_v1', b='panda2_link6_v1',
            distance=0.022, required=0.030, arm_id='panda1'):
    """Return one Contact with the model's six frozen fields."""
    return Contact(kind=kind, a=a, b=b, distance=distance, required=required,
                   arm_id=arm_id)


# ----------------------------------------------------------------------
# T01-T04 -- the shape of the executed path
# ----------------------------------------------------------------------


class TestTheExecutedPath:
    """The waypoints are the checked line, at a speed the controller tracks."""

    def test_no_waypoint_step_exceeds_the_per_joint_speed_budget(self):
        """
        T01. Every joint, every step, under the profile's own rate limit.

        The budget is per TICK: the profile's rate limit is radians per
        second, the stream runs at JOG_STREAM_HZ, and 20% headroom is kept so
        that one late tick cannot make the controller's per-joint slew limiter
        clamp some joints and not others -- which would take the executed path
        off the line check_path approved.
        """
        assert defaults.APPLY_SPEED_FRACTION < 1.0, (
            'the headroom is the whole point: at 1.0 a single late tick makes '
            "the controller's per-joint slew limiter clamp some joints and "
            'not others, and the executed path leaves the checked line')
        travel_plan = plan(q_goal=moved(HOME, joint=0, delta=0.9))
        previous = travel_plan.q0
        biggest = 0.0
        for index in range(1, travel_plan.steps_total + 1):
            point = travel_plan.waypoint(index)
            for joint in range(defaults.JOINT_COUNT):
                step = abs(point[joint] - previous[joint])
                biggest = max(biggest, step)
                assert step < CONTROLLER_STEP, (
                    'step {} joint {} moved {} rad; the controller can ramp '
                    '{} in one stream period'.format(
                        index, joint, step, CONTROLLER_STEP))
            previous = point
        assert biggest > 0.0, 'nothing moved, so this proves nothing'

    def test_the_last_waypoint_is_the_goal_exactly(self):
        """
        T02. ``==``, not ``approx``.

        Float error must never leave the arm a micro-radian short of the pose
        the operator drew, so the final waypoint is the goal by identity
        rather than by arithmetic.
        """
        goal = moved(HOME, joint=6, delta=0.37)
        travel_plan = plan(q_goal=goal)
        assert travel_plan.waypoint(travel_plan.steps_total) == goal

    def test_every_waypoint_is_one_scalar_along_the_line(self):
        """
        T03. One parameter drives all seven joints.

        This is what makes the executed path the CHECKED path: check_path
        resamples between consecutive waypoints along the straight line, and
        seven independent ramps would execute a different curve from the one
        that was approved.
        """
        goal = tuple(value + 0.11 * (index + 1)
                     for index, value in enumerate(HOME))
        travel_plan = plan(q_goal=goal)
        for index in range(1, travel_plan.steps_total + 1):
            point = travel_plan.waypoint(index)
            scalars = [(point[j] - travel_plan.q0[j])
                       / (travel_plan.q1[j] - travel_plan.q0[j])
                       for j in range(defaults.JOINT_COUNT)]
            assert max(scalars) - min(scalars) < 1e-12, scalars

    def test_every_waypoint_is_inside_a_fence_both_endpoints_touch(self):
        """
        T04. The convexity lemma, at the hardest case.

        The fence is a box of seven independent intervals, so a convex
        combination of two in-fence poses is in-fence. Both endpoints are put
        exactly ON the boundary here, which is where a bug would show.
        """
        lower = [value - 0.5 for value in HOME]
        upper = [value + 0.5 for value in HOME]
        travel_plan = plan(q_held=tuple(lower), q_measured=tuple(lower),
                           q_goal=tuple(upper), fence_pair=(lower, upper))
        for index in range(1, travel_plan.steps_total + 1):
            point = travel_plan.waypoint(index)
            for joint in range(defaults.JOINT_COUNT):
                assert lower[joint] <= point[joint] <= upper[joint]


# ----------------------------------------------------------------------
# T05-T08, T15d -- the refusal ladder, in its specified order
# ----------------------------------------------------------------------


class TestTheRefusalLadder:
    """Each gate refuses with its own code, and nothing is planned."""

    def test_a_goal_outside_the_fence_is_refused_before_any_check(self):
        """T05. ``pose_outside_fence``, and the checker is never called."""
        model = FakeCellModel()
        error = refusal(model=model, q_goal=moved(HOME, joint=0, delta=2.0))
        assert error.code == 'apply_refused'
        assert error.reason_code == 'pose_outside_fence'
        assert 'J1' in error.detail
        assert model.paths == [], 'a refused goal still cost a path check'

    def test_a_held_target_that_has_stopped_describing_the_arm_is_refused(self):
        """
        T06. The staleness gate, at 2.0 degrees, per joint.

        An arm being pushed by hand, still finishing a jog, or fighting an
        obstruction disagrees with its held target by more than one jog step,
        and in every one of those cases the operator should be told to wait
        rather than handed a travel planned from a stale premise.
        """
        error = refusal(q_measured=moved(HOME, joint=1, delta=2.1 * DEG))
        assert error.reason_code == 'not_settled'
        assert error.detail == (
            'Panda 1 is still catching up to its last command. Wait a moment '
            'and press Apply again.')
        assert plan(q_measured=moved(HOME, joint=1, delta=1.9 * DEG)) is not None

    def test_a_travel_shorter_than_half_a_degree_is_nothing_to_apply(self):
        """T07. Below the floor the ghost is where the arm already is."""
        assert refusal(q_goal=moved(HOME, delta=0.4 * DEG)).reason_code == 'no_travel'
        assert plan(q_goal=moved(HOME, delta=0.6 * DEG)) is not None

    def test_a_travel_past_the_duration_cap_is_refused_at_the_boundary(self):
        """
        T08. Exactly the cap is planned; one step past it is not.

        A move that takes two minutes is not a pose Apply, it is a program,
        and the console does not run programs. The velocity here is scripted
        small so that the DURATION cap binds before the path-length cap does;
        both answer ``too_far`` and their sentences say which one spoke.
        """
        slow = (0.001,) * defaults.JOINT_COUNT
        ceiling = int(defaults.APPLY_MAX_DURATION_S * defaults.JOG_STREAM_HZ)
        span = (ceiling * defaults.APPLY_SPEED_FRACTION * slow[0]
                / defaults.JOG_STREAM_HZ)
        exact = plan(velocity=slow,
                     q_goal=moved(HOME, delta=span * (1 - 1e-12)))
        assert exact.steps_total == ceiling
        error = refusal(velocity=slow,
                        q_goal=moved(HOME, delta=span * (1 + 1e-9)))
        assert error.reason_code == 'too_far'
        assert error.detail.startswith('That travel would take ')

    def test_the_path_budget_counts_both_checked_segments(self):
        """
        T15d. The start gap is part of the path, so it is part of the bound.

        The bound exists to cap check_path's resampled sample count, and the
        call is handed three waypoints, so a bound that covered only the
        commanded segment would under-count every Apply.
        """
        wide = ([value - 10.0 for value in HOME],
                [value + 10.0 for value in HOME])
        per_joint = defaults.APPLY_MAX_PATH_RAD / defaults.JOINT_COUNT - 0.001
        goal = tuple(value + per_joint for value in HOME)
        assert plan(q_goal=goal, fence_pair=wide) is not None
        gap = moved(HOME, joint=0, delta=defaults.APPLY_START_ALIGN_RAD)
        error = refusal(q_measured=gap, q_goal=goal, fence_pair=wide)
        assert error.reason_code == 'too_far'
        assert error.detail == travel.TOO_LONG_PATH


# ----------------------------------------------------------------------
# T09-T12 -- what the check says, and how it is rendered
# ----------------------------------------------------------------------


class TestTheCheckerVerdict:
    """A refused path carries its witness; a raising checker never allows."""

    def test_a_fouled_path_refuses_with_the_whole_witness(self):
        """T09. The sentence, the links, the clearance and the model triple."""
        model = FakeCellModel(path_result=CheckResult(
            ok=False, min_clearance=-0.008, contacts=(contact(),),
            sample_index=48, samples_evaluated=143))
        error = refusal(model=model)
        assert error.code == 'apply_refused'
        assert error.reason_code == 'contact'
        assert error.payload['offending_links'] == ['panda1_link5',
                                                    'panda2_link6']
        assert error.payload['min_clearance'] == pytest.approx(-0.008)
        assert error.payload['sample_index'] == 48
        assert error.payload['samples_evaluated'] == 143
        assert error.payload['model_id'] == 'fake_cell'
        assert error.payload['model_revision'] == 1
        assert error.payload['model_sha256'] == '0' * 64

    @pytest.mark.parametrize('raised', [
        WorkspaceModelError('the cell model refuses that question'),
        RuntimeError('something nobody predicted'),
    ])
    def test_a_checker_that_raises_never_produces_a_plan(self, raised):
        """
        T10. Both families of failure refuse, and neither ever allows.

        A checker that raises is the case a mutation would most easily turn
        into an allow: the exception is caught for exactly one reason, and
        that reason is to answer ``apply_refused``.
        """
        error = refusal(model=FakeCellModel(path_raises=raised))
        assert error.code == 'apply_refused'
        assert error.reason_code == 'checker_error'
        assert error.detail == str(raised)

    @pytest.mark.parametrize('kind,a,b', [
        ('self', 'panda1_link5_v1', 'panda1_link2_v1'),
        ('cross_arm', 'panda1_link5_v1', 'panda2_link6_v1'),
        ('containment', 'panda1_link3_v1', 'work_area.z_min'),
        ('environment', 'panda1_link4_v1', 'pedestal_1'),
        ('keep_out', 'panda1_link6_v1', 'operator_zone'),
        ('joint_limit', 'panda1_joint4', ''),
    ])
    def test_the_sentence_is_the_ghost_modules_own(self, kind, a, b):
        """
        T11. Byte for byte: this module authored no second table.

        The six contact sentences have exactly one author. A refusal here adds
        only the prefix that says WHERE along the way the problem is.
        """
        one = contact(kind=kind, a=a, b=b)
        model = FakeCellModel(path_result=CheckResult(
            ok=False, min_clearance=-0.008, contacts=(one,),
            sample_index=0, samples_evaluated=143))
        assert refusal(model=model).detail == ghost.verdict_sentence(one)

    def test_the_display_names_match_the_ghost_module(self):
        """
        The two arm display names are a duplicate, and it is checked.

        ``joint_names_for`` is spelled twice for the same reason, with the
        same kind of test beside it: a second spelling is acceptable only when
        something compares the two.
        """
        for arm_id, shown in travel.DISPLAY.items():
            one = contact(kind='self', a=arm_id + '_link5_v1',
                          b=arm_id + '_link2_v1', arm_id=arm_id)
            assert ghost.verdict_sentence(one).startswith(shown + "'s ")

    @pytest.mark.parametrize('index,total,expected', [
        (0, 143, ''),
        (None, 143, ''),
        (48, 143, 'About 34% of the way there: '),
        (142, 143, 'At the pose you drew: '),
        (1, 1, ''),
    ])
    def test_the_prefix_places_the_refusal_on_the_path(self, index, total,
                                                       expected):
        """
        T12. A refusal an operator cannot place on the path is unactionable.

        Index 0 is the start and gets no prefix; the last sample is the pose
        the operator drew and says so; everything between is a percentage.
        """
        assert travel.refusal_prefix(index, total) == expected


# ----------------------------------------------------------------------
# T13 -- the per-tick stop guards
# ----------------------------------------------------------------------


class TestStopReason:
    """Three conditions, in one order, over floats and nothing else."""

    def travelling(self, co_arm_id='panda2'):
        """Return a live plan to hand the guards."""
        return plan(co_arm_id=co_arm_id,
                    co_arm_q=None if co_arm_id is None else HOME)

    def ask(self, plan_in, co_arm_q_now=HOME, co_arm_fresh=True,
            q_measured=HOME, q_target=HOME):
        """Run the guards once."""
        return travel.stop_reason(
            plan=plan_in, co_arm_q_now=co_arm_q_now,
            co_arm_fresh=co_arm_fresh, q_measured=q_measured,
            q_target=q_target)

    def test_a_clear_tick_keeps_going(self):
        """Nothing wrong, nothing said."""
        assert self.ask(self.travelling()) is None

    def test_a_co_arm_that_drifts_past_one_degree_stops_the_travel(self):
        """
        T13. One resampling step of the check that approved the path.

        Cancel, not re-check: re-deriving authority mid-motion from a pose
        that is itself moving is the kind of cleverness that hides a bug.
        """
        live = self.travelling()
        assert self.ask(live, co_arm_q_now=moved(HOME, delta=0.9 * DEG)) is None
        sentence = self.ask(live, co_arm_q_now=moved(HOME, delta=1.1 * DEG))
        assert sentence == (
            'Panda 2 moved while Panda 1 was travelling, so the checked path '
            'no longer describes the cell. Hold the other arm still and press '
            'Apply again.')

    def test_a_co_arm_that_stops_being_reported_stops_the_travel(self):
        """T13. Staleness is judged before drift, because drift needs a pose."""
        live = self.travelling()
        assert 'stopped being reported' in self.ask(live, co_arm_fresh=False)
        assert 'stopped being reported' in self.ask(live, co_arm_q_now=None)

    def test_an_arm_that_does_not_follow_its_target_stops_the_travel(self):
        """
        T13. The lag brake, at 12 degrees, before the torque ceilings bind.

        The ceilings are the real limit; this is an earlier one that can
        explain itself in words.
        """
        live = self.travelling()
        assert self.ask(live, q_target=moved(HOME, delta=11.9 * DEG)) is None
        assert self.ask(live, q_target=moved(HOME, delta=12.1 * DEG)) == (
            'Panda 1 is not following its commanded pose — something may be '
            'in its way. The travel stopped where it is.')

    def test_the_co_arm_is_judged_before_this_arms_lag(self):
        """T13. The order is specified, so it is asserted."""
        live = self.travelling()
        sentence = self.ask(live, co_arm_fresh=False,
                            q_target=moved(HOME, delta=20 * DEG))
        assert 'Panda 2' in sentence

    def test_an_unreported_own_pose_skips_the_lag_test_rather_than_guessing(self):
        """
        A missing sample is fault rule F6's business, not a guess here.

        In a dual session it cannot arise -- the co-arm's freshness IS this
        sample's freshness -- and in a single-arm session the joint-state
        staleness fault clears every travel a second later.
        """
        live = self.travelling(co_arm_id=None)
        assert self.ask(live, q_measured=None) is None


# ----------------------------------------------------------------------
# T14, T15, T15b, T15c -- the multi-arm shape of the check
# ----------------------------------------------------------------------


class TestTheCheckedPath:
    """The call itself: three waypoints, both arms, the measured pose first."""

    def test_the_check_is_handed_measured_then_held_then_goal(self):
        """
        T15b. The tripwire for the two-waypoint mistake.

        The travel is commanded from the HELD target while the arm physically
        sits at the MEASURED pose, and the segment between them is one the arm
        closes under impedance the moment the travel starts. A two-waypoint
        call would assume it clear rather than check it.
        """
        model = FakeCellModel()
        held = moved(HOME, joint=2, delta=1.5 * DEG)
        goal = moved(HOME, joint=5, delta=0.3)
        plan(model=model, q_held=held, q_measured=HOME, q_goal=goal,
             co_arm_q=moved(HOME, joint=0, delta=0.4))
        assert len(model.paths) == 1
        # False, and deliberately: the model stops at the first violating
        # sample when this is True and then reports samples_evaluated as the
        # count it got through, so every refusal would read "At the pose you
        # drew". A clear path of the same length evaluates every sample
        # anyway, so the worst case costs the same either way.
        assert model.path_flags == [False]
        recorded = model.paths[0]
        assert [point['panda1'] for point in recorded] == [HOME, held, goal]
        other = moved(HOME, joint=0, delta=0.4)
        assert [point['panda2'] for point in recorded] == [other, other, other]

    def test_a_foul_on_the_start_segment_alone_refuses_the_whole_travel(self):
        """
        T15c. Clear at both ends and fouled between is still a refusal.

        This is the case a two-endpoint check cannot see, and it is the reason
        the measured pose is a waypoint rather than an assumption.
        """
        def verdict(waypoints):
            """Approve nothing on the way in from the measured pose."""
            return CheckResult(ok=False, min_clearance=-0.004,
                               contacts=(contact(),), sample_index=2,
                               samples_evaluated=143)
        model = FakeCellModel(path_verdict=verdict)
        error = refusal(model=model,
                        q_held=moved(HOME, joint=2, delta=1.5 * DEG))
        assert error.reason_code == 'contact'
        assert error.detail.startswith('About 1% of the way there: ')

    def test_a_single_arm_model_plans_with_no_co_arm_at_all(self):
        """T14. One key in the mapping, and nothing refused on its account."""
        model = FakeCellModel(arms=('panda1',))
        travel_plan = plan(model=model, co_arm_id=None, co_arm_q=None)
        assert travel_plan.co_arm_id is None
        assert travel_plan.co_arm_q is None
        assert all(set(point) == {'panda1'} for point in model.paths[0])

    def test_a_co_arm_the_model_wants_but_cannot_see_refuses(self):
        """T15. The model's own rule: refuse rather than assume a pose."""
        error = refusal(co_arm_q=None)
        assert error.reason_code == 'co_arm_unknown'
        assert error.detail == (
            "Panda 2's pose is not being reported, so the two-arm check could "
            'not run.')

    def test_a_co_arm_the_model_does_not_describe_is_left_out(self):
        """A single-arm model in a dual session checks the arm it knows."""
        model = FakeCellModel(arms=('panda1',))
        travel_plan = plan(model=model, co_arm_id='panda2', co_arm_q=HOME)
        assert travel_plan.co_arm_id is None
        assert all(set(point) == {'panda1'} for point in model.paths[0])


# ----------------------------------------------------------------------
# T15e -- the plan is a stamped fact, and this module reads no live state
# ----------------------------------------------------------------------


class TestThePlanIsPure:
    """The two integers pass through unchanged, and nothing else is read."""

    def test_the_plan_carries_the_epoch_and_generation_it_was_handed(self):
        """
        T15e. The tick proves authority with these two; nothing may edit them.

        They are read by the CALLER under the supervisor's own lock and passed
        in, so the plan records the authorisation it was built under rather
        than one it went and looked up for itself.
        """
        travel_plan = plan(enable_epoch=17, cancel_gen=42)
        assert travel_plan.enable_epoch == 17
        assert travel_plan.cancel_gen == 42
        assert travel_plan.advanced().enable_epoch == 17
        assert travel_plan.advanced().cancel_gen == 42

    def test_plan_travel_takes_no_live_state_at_all(self):
        """
        T15e. There is no session, bridge or supervisor to hand it.

        A stub that raises on every attribute cannot be passed to a function
        that has no parameter to pass it to, so the property is asserted on
        the signature: every input is a value, and the only object is the
        model.
        """
        parameters = set(inspect.signature(travel.plan_travel).parameters)
        assert parameters == {
            'arm_id', 'q_held', 'q_measured', 'q_goal', 'fence_lower',
            'fence_upper', 'max_target_velocity', 'model', 'co_arm_id',
            'co_arm_q', 'enable_epoch', 'cancel_gen', 'stream_hz'}

    def test_a_plan_advances_one_step_at_a_time_and_then_is_done(self):
        """``live`` and ``fraction`` are what the frame and the tick read."""
        travel_plan = plan(q_goal=moved(HOME, delta=0.04))
        assert travel_plan.live is True
        assert travel_plan.fraction == 0.0
        for _ in range(travel_plan.steps_total):
            assert travel_plan.live is True
            travel_plan = travel_plan.advanced()
        assert travel_plan.live is False
        assert travel_plan.fraction == 1.0

    def test_a_waypoint_outside_the_plan_is_an_error_not_a_guess(self):
        """Off the end of a finite sequence is a bug, and says so."""
        travel_plan = plan()
        with pytest.raises(IndexError):
            travel_plan.waypoint(0)
        with pytest.raises(IndexError):
            travel_plan.waypoint(travel_plan.steps_total + 1)


class TestMalformedGoals:
    """Seven finite floats, or ``invalid_json`` naming the field."""

    @pytest.mark.parametrize('goal', [
        None, 'abcdefg', [0.0] * 6, [0.0] * 8,
        [0.0, 0.0, 0.0, float('nan'), 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, float('inf'), 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 'x', 0.0, 0.0, 0.0],
        [True, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ])
    def test_a_malformed_goal_is_invalid_json(self, goal):
        """A bool is not a number here, and a string is not a sequence."""
        model = FakeCellModel()
        error = refusal(model=model, q_goal=goal)
        assert error.code == 'invalid_json'
        assert "'positions'" in error.detail
        assert model.paths == []
