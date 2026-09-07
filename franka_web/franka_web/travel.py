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
Apply: deciding WHETHER a ghost pose may be travelled to, and WHERE next.

One Apply is one bounded joint-space travel of ONE arm, from where it is now
to the pose the operator drew, along the straight line between them in joint
space, at a speed under the profile's own ``max_target_velocity``, streamed as
ordinary joint targets through the ordinary 20 Hz producer, after the whole
line has been approved by ``CellModel.check_path``.

The split this module exists to make legible:

    ``travel.py`` decides whether and where; ``session.py`` and ``jog.py``
    put the bytes on the wire, under the guards they already enforce.

So this module is PURE. It has no ROS client library, no ROS message type, no
session, no bridge, no HTTP, no clock and no lock. It cannot send anything to a
robot and it cannot read live state: everything it judges is handed to it, and
the two integers it stamps into a plan (``enable_epoch`` and ``cancel_gen``)
are read by the caller under the supervisor's own lock so that the 20 Hz tick
can later prove the plan is still the one the operator authorised.

It adds NO torque primitive. A travel changes the value of the held jog target
between ticks; every byte that reaches the controller is built by the same
``JogTargetModel.message`` from the same held target under the same four gates
as every byte a jog produces.

The sentence table is not duplicated here. A refusal that names a contact is
rendered by ``ghost.verdict_sentence``, which is the one author of those six
sentences; this module only prefixes it with where along the way the problem
is. The two display names below ARE a second spelling of a mapping ghost.py
holds privately, in the same way ``joint_names_for`` is spelled twice --
:func:`test_travel.test_the_display_names_match_the_ghost_module` asserts the
two agree.
"""

from dataclasses import dataclass, replace
import math

from franka_web import defaults
from franka_web.ghost import offending_links_for, verdict_sentence

#: The console's display name for each arm. Not a teaching sentence: the
#: sentences themselves are ghost.py's, and this module never writes one that
#: ghost.py could have written.
DISPLAY = {'panda1': 'Panda 1', 'panda2': 'Panda 2'}

#: The refusals the alignment, travel-length and co-arm gates raise, verbatim.
NOT_SETTLED = ('{arm} is still catching up to its last command. Wait a moment '
               'and press Apply again.')
CO_ARM_UNKNOWN = ("{other}'s pose is not being reported, so the two-arm check "
                  'could not run.')
NO_TRAVEL = ('The ghost is where {arm} already is, so there is nothing to '
             'apply.')
TOO_FAR = ('That travel would take {seconds:.0f} s. Move the ghost closer, '
           'apply, and drag again from there.')
TOO_LONG_PATH = ('That travel is too long to check in one go. Move the ghost '
                 'closer, apply, and drag again from there.')
POSE_OUTSIDE_FENCE = '{arm} cannot be commanded there: {joints}.'
STOPPED_WHILE_CHECKING = ('The arm was stopped while the path was being '
                          'checked; press Apply again.')

#: The three sentences :func:`stop_reason` can return, verbatim.
STOP_CO_ARM_STALE = ("{other}'s pose stopped being reported while {arm} was "
                     'travelling, so the checked path no longer describes the '
                     'cell. The travel stopped where it is.')
STOP_CO_ARM_DRIFT = ('{other} moved while {arm} was travelling, so the checked '
                     'path no longer describes the cell. Hold the other arm '
                     'still and press Apply again.')
STOP_LAG = ('{arm} is not following its commanded pose — something may be '
            'in its way. The travel stopped where it is.')

#: The prefix a refusal carries when the check says WHERE along the way the
#: problem is. Index 0 is the start, so it gets none.
PREFIX_AT_GOAL = 'At the pose you drew: '
PREFIX_PART_WAY = 'About {pct}% of the way there: '


class TravelError(Exception):
    """
    A refusal, carrying the closed-set code, the sentence and the witness.

    ``code`` is one of the API's closed error codes; ``reason_code`` is the
    machine token beside the human sentence, so the frontend never
    string-matches a sentence to choose a behaviour.
    """

    def __init__(self, code, reason_code, detail, payload=None):
        """Store the code, the machine token, the sentence and its evidence."""
        super().__init__(detail)
        self.code = code
        self.reason_code = reason_code
        self.detail = detail
        self.payload = dict(payload or {})
        if reason_code is not None:
            self.payload.setdefault('reason_code', reason_code)


@dataclass(frozen=True)
class TravelPlan:
    """
    One approved travel: a finite sequence of waypoints on a checked line.

    Frozen, and advanced by building a new one, so the 20 Hz tick can prove
    with an identity comparison that nothing cleared this plan while it was
    computing the next step (G3 plan section 3.1, gate 8).

    A plan is NOT a permission. Every gate that authorised it -- the session
    state, the enable flag, the source, the operator lock, the enable epoch and
    the cancel generation -- is re-read on every single tick immediately
    before the next target goes on the wire. The plan only says where the next
    point on the approved line is.
    """

    arm_id: str
    q0: tuple                    # 7, the held target at plan time
    q1: tuple                    # 7, the goal
    steps_total: int
    step: int                    # advanced steps so far
    enable_epoch: int            # gate 5: must still match _enable_epoch[arm]
    cancel_gen: int              # gate 6: must still match _cancel_gen[arm]
    co_arm_id: str = None
    co_arm_q: tuple = None       # the pose check_path was given
    checked: dict = None         # samples_evaluated, min_clearance, model triple

    @property
    def live(self):
        """Return whether this travel still has a waypoint to command."""
        return self.step < self.steps_total

    @property
    def fraction(self):
        """Return how far along the line the commanded target has gone, 0..1."""
        return self.step / float(self.steps_total)

    @property
    def seconds_remaining(self):
        """Return the nominal time left at the stream rate; an ESTIMATE."""
        return (self.steps_total - self.step) / defaults.JOG_STREAM_HZ

    def waypoint(self, index):
        """
        Return waypoint ``index`` (1..steps_total); the last is ``q1`` exactly.

        One scalar parameter drives all seven joints, so every waypoint is a
        convex combination of ``q0`` and ``q1`` and the executed path is the
        checked path. The last waypoint is ``q1`` by identity rather than by
        arithmetic, so float error can never leave the arm a micro-radian short
        of the pose the operator drew.
        """
        if not 1 <= index <= self.steps_total:
            raise IndexError('waypoint {} is outside 1..{}'.format(
                index, self.steps_total))
        if index == self.steps_total:
            return tuple(self.q1)
        scale = index / float(self.steps_total)
        return tuple(start + (goal - start) * scale
                     for start, goal in zip(self.q0, self.q1))

    def advanced(self):
        """Return the same plan with ``step`` incremented by one."""
        return replace(self, step=self.step + 1)


def _display(arm_id):
    """Return the console's display name for an arm id."""
    return DISPLAY.get(arm_id, arm_id)


def _is_number(value):
    """Return True for a real number that is not a bool."""
    return not isinstance(value, bool) and isinstance(value, (int, float))


def _seven_floats(values, what):
    """Return ``values`` as seven finite floats, or raise ``invalid_json``."""
    if isinstance(values, (str, bytes, bytearray)) or values is None:
        raise TravelError('invalid_json', None, '{} must be {} numbers'.format(
            what, defaults.JOINT_COUNT))
    try:
        items = list(values)
    except TypeError:
        raise TravelError('invalid_json', None, '{} must be {} numbers'.format(
            what, defaults.JOINT_COUNT)) from None
    if len(items) != defaults.JOINT_COUNT:
        raise TravelError('invalid_json', None, '{} must be {} numbers'.format(
            what, defaults.JOINT_COUNT))
    out = []
    for value in items:
        if not _is_number(value) or not math.isfinite(float(value)):
            raise TravelError(
                'invalid_json', None,
                '{} must be {} finite numbers'.format(
                    what, defaults.JOINT_COUNT))
        out.append(float(value))
    return tuple(out)


def path_fraction(sample_index, samples_evaluated):
    """
    Return where along a checked path one sample sits, 0..1, or None.

    One arithmetic behind both halves of a placed refusal: the percentage the
    operator reads and the waypoint the witness is taken from are computed
    from this single number, so the two cannot drift apart.
    """
    if (sample_index is None or samples_evaluated is None
            or samples_evaluated < 2):
        return None
    if sample_index <= 0:
        return 0.0
    if sample_index >= samples_evaluated - 1:
        return 1.0
    return sample_index / float(samples_evaluated - 1)


def refusal_prefix(sample_index, samples_evaluated):
    """
    Return the words that say WHERE along the way a refusal happens.

    Empty at the start of the path, ``At the pose you drew: `` at its end, and
    a rounded percentage anywhere between. A refusal an operator cannot place
    on the path is a refusal they cannot act on.
    """
    fraction = path_fraction(sample_index, samples_evaluated)
    if fraction is None or fraction <= 0.0:
        return ''
    if fraction >= 1.0:
        return PREFIX_AT_GOAL
    return PREFIX_PART_WAY.format(pct=int(round(100.0 * fraction)))


def _lerp(start, goal, scale):
    """
    Return the point ``scale`` of the way from ``start`` to ``goal``.

    Spelled exactly as the model's own resampler spells it -- ``a + (b - a) *
    s`` -- so a waypoint rebuilt here is the same IEEE double the model
    evaluated rather than a value very close to it.
    """
    return tuple(begin + (end - begin) * scale
                 for begin, end in zip(start, goal))


def _span(start, goal):
    """Return the max-norm distance between two joint vectors."""
    return max(abs(end - begin) for begin, end in zip(start, goal))


def _estimated_point(waypoints, fraction):
    """
    Return the point ``fraction`` along a polyline, by max-norm arc length.

    The fallback for when the model's own resampling cannot be recovered.
    Max-norm arc length is the metric the resampler itself spaces samples in,
    so this lands within about one resampling step of the sample an index
    names -- close, and never claimed to be more than close: the caller checks
    the point it gets back and withdraws the placement if it does not violate.
    """
    spans = [_span(start, end) for start, end in zip(waypoints, waypoints[1:])]
    total = sum(spans)
    if total <= 0.0:
        return tuple(waypoints[-1])
    remaining = fraction * total
    for start, end, span in zip(waypoints, waypoints[1:], spans):
        if span <= 0.0:
            continue
        if remaining <= span:
            return _lerp(start, end, remaining / span)
        remaining -= span
    return tuple(waypoints[-1])


def _segment_samples(check, point_of, start, end):
    """
    Return how many samples the model resamples ONE segment into, or None.

    A path is resampled per SEGMENT, each with its own count, so nothing about
    the whole path's sample total says where its first segment ends in
    sample-index space. Asking is the only honest way to find out, and the
    question is cheap: the first segment spans at most ``APPLY_START_ALIGN_RAD``
    per joint, so it is three or four samples.
    """
    try:
        head = check([point_of(start), point_of(end)])
    except Exception:                     # noqa: BLE001 - never an allow
        return None
    total = getattr(head, 'samples_evaluated', None)
    if isinstance(total, bool) or not isinstance(total, int) or total < 2:
        return None
    return total - 1


def _witness_at(check, point_of, waypoints, sample_index, samples_evaluated):
    """
    Return the model's verdict AT the first violating sample, or None.

    The whole-path call PLACES a violation -- ``sample_index`` of
    ``samples_evaluated`` -- but the contacts it carries are every violating
    contact from the whole path, re-sorted globally, so ``contacts[0]`` is the
    worst contact ANYWHERE on the path rather than the worst one there. Its
    ``min_clearance`` is likewise path-wide. Rendering the two together makes
    a sentence whose halves are each true and whose whole is false: "about 15%
    of the way there, 67 mm past the boundary", when at 15% the arm is 0.9 mm
    past it and the 67 mm contact is a different pair much further along.

    So the sample the prefix names is rebuilt and checked on its own. The
    rebuild is EXACT when the segment split can be recovered -- the same
    convex combination, spelled the same way, of the same two waypoints -- and
    an estimate otherwise; either way the point is put back to the model, so
    the sentence, the clearance and the offending links all come from one
    checked configuration.

    None when the rebuilt point does not violate after all, or the model would
    not answer. The caller then withdraws the placement instead of lending it
    a witness from somewhere else.
    """
    fraction = path_fraction(sample_index, samples_evaluated)
    if fraction is None:
        return None
    measured, held, goal = waypoints
    head = _segment_samples(check, point_of, measured, held)
    if head is not None and 1 <= head <= samples_evaluated - 2:
        if sample_index <= head:
            point = _lerp(measured, held, sample_index / float(head))
        else:
            point = _lerp(held, goal, (sample_index - head)
                          / float(samples_evaluated - 1 - head))
    else:
        point = _estimated_point(waypoints, fraction)
    try:
        at_point = check([point_of(point)])
    except Exception:                     # noqa: BLE001 - never an allow
        return None
    return at_point if (not at_point.ok and at_point.contacts) else None


def _model_triple(result):
    """Return the model identity triple a CheckResult carries, or nulls."""
    return {
        'model_id': getattr(result, 'model_id', None),
        'model_revision': getattr(result, 'model_revision', None),
        'model_sha256': getattr(result, 'model_sha256', None),
    }


def _finite_or_none(value):
    """Return a float the JSON envelope can carry, or None."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _steps_total(q0, q1, max_target_velocity, stream_hz):
    """
    Return the number of 20 Hz steps the speed budget allows for this line.

    ``N = max(1, ceil(stream_hz * max_j(d_j / v_j)))`` with
    ``v_j = APPLY_SPEED_FRACTION * max_target_velocity[j]``, so the per-tick
    per-joint increment ``d_j / N`` satisfies ``d_j * stream_hz / N <= v_j``
    for every joint, by construction. One scalar N for all seven joints is what
    makes the executed path the checked line.
    """
    worst = 0.0
    for index in range(defaults.JOINT_COUNT):
        budget = defaults.APPLY_SPEED_FRACTION * float(max_target_velocity[index])
        if not math.isfinite(budget) or budget <= 0.0:
            raise TravelError(
                'apply_refused', 'checker_error',
                'this arm has no usable speed limit, so no travel can be '
                'planned')
        worst = max(worst, abs(q1[index] - q0[index]) / budget)
    return max(1, int(math.ceil(stream_hz * worst - 1e-12)))


def plan_travel(*, arm_id, q_held, q_measured, q_goal, fence_lower, fence_upper,
                max_target_velocity, model, co_arm_id, co_arm_q,
                enable_epoch, cancel_gen, stream_hz=defaults.JOG_STREAM_HZ):
    """
    Validate, check and build one travel, or raise :class:`TravelError`.

    In this order, and the order is the specification:

      1. ``q_goal`` is seven finite floats                 -> invalid_json
      2. ``q_goal`` is inside the fence                    -> pose_outside_fence
      3. ``q_held`` aligns with ``q_measured`` within
         APPLY_START_ALIGN_RAD, per joint                  -> not_settled
      4. max_j |q_goal - q_held| >= APPLY_MIN_TRAVEL_RAD   -> no_travel
      5. sum_j |q_goal - q_held| + sum_j |q_held - q_measured|
                                  <= APPLY_MAX_PATH_RAD    -> too_far
      6. steps_total from the speed budget;
         steps_total <= APPLY_MAX_DURATION_S * stream_hz   -> too_far
      7. the co-arm pose is present when the model wants
         two arms                                          -> co_arm_unknown
      8. ``model.check_path([measured, held, goal], first_violation=False)``
         raises  -> apply_refused / checker_error
         not ok  -> apply_refused / contact, with the witness taken from the
                    sample the result PLACES the violation at, re-checked on
                    its own so the sentence and the location agree
      9. build the :class:`TravelPlan`

    Step 5 counts BOTH segments because both are checked: the bound exists to
    cap the resampled sample count, so it must cover every waypoint pair handed
    to ``check_path``, not only the one the arm is commanded along. Step 3 caps
    the extra segment at ``7 * APPLY_START_ALIGN_RAD``, so this is a correction
    of at most 0.24 rad on a 7.0 rad budget.

    Three waypoints, not two, and the first is the MEASURED pose. The travel is
    commanded from ``q_held``; the arm is physically at ``q_measured``, and the
    segment between them is one the arm closes under impedance the moment the
    travel starts. A two-waypoint call would assume it clear rather than check
    it. The alignment gate above is a separate, STALENESS guard -- it asks
    whether the held target still describes this arm -- and the safety of the
    start does not depend on its value.

    ``cancel_gen`` and ``enable_epoch`` are read by the CALLER under
    ``_state_lock`` and passed in. This function never reads live session state:
    it is pure, and the two integers it stamps into the plan are what let the
    tick prove the plan is still the one the operator authorised.

    ``model`` is a loaded ``CellModel``; obtaining it is the caller's job, and a
    caller that has none must refuse with ``apply_unavailable`` BEFORE calling
    here. This function never decides that a missing checker is acceptable,
    because it never sees one.

    Steps 1-7 run BEFORE the check so a malformed or pointless request costs no
    check time, and step 8 is the last thing that can refuse -- so if it passes,
    the plan is buildable and ``plan_travel`` cannot fail afterwards.
    """
    who = _display(arm_id)
    goal = _seven_floats(q_goal, "'positions'")
    held = _seven_floats(q_held, 'the held target')
    measured = _seven_floats(q_measured, 'the measured pose')

    outside = [
        'J{} = {:.1f}° is outside [{:.1f}°, {:.1f}°]'.format(
            index + 1, math.degrees(goal[index]),
            math.degrees(float(fence_lower[index])),
            math.degrees(float(fence_upper[index])))
        for index in range(defaults.JOINT_COUNT)
        if (goal[index] < float(fence_lower[index])
            or goal[index] > float(fence_upper[index]))]
    if outside:
        raise TravelError('apply_refused', 'pose_outside_fence',
                          POSE_OUTSIDE_FENCE.format(
                              arm=who, joints='; '.join(outside)))

    if max(abs(held[i] - measured[i])
           for i in range(defaults.JOINT_COUNT)) > defaults.APPLY_START_ALIGN_RAD:
        raise TravelError('apply_refused', 'not_settled',
                          NOT_SETTLED.format(arm=who))

    if max(abs(goal[i] - held[i])
           for i in range(defaults.JOINT_COUNT)) < defaults.APPLY_MIN_TRAVEL_RAD:
        raise TravelError('apply_refused', 'no_travel', NO_TRAVEL.format(arm=who))

    excursion = sum(abs(goal[i] - held[i]) + abs(held[i] - measured[i])
                    for i in range(defaults.JOINT_COUNT))
    if excursion > defaults.APPLY_MAX_PATH_RAD:
        raise TravelError('apply_refused', 'too_far', TOO_LONG_PATH)

    steps_total = _steps_total(held, goal, max_target_velocity, stream_hz)
    ceiling = int(defaults.APPLY_MAX_DURATION_S * stream_hz)
    if steps_total > ceiling:
        raise TravelError('apply_refused', 'too_far',
                          TOO_FAR.format(seconds=steps_total / stream_hz))

    wanted = tuple(model.arm_ids())
    other = co_arm_id if co_arm_id in wanted else None
    if other is not None and co_arm_q is None:
        raise TravelError('apply_refused', 'co_arm_unknown',
                          CO_ARM_UNKNOWN.format(other=_display(other)))
    co_arm = None if other is None else _seven_floats(
        co_arm_q, "the other arm's measured pose")

    def _point(values):
        """Return one full multi-arm waypoint mapping."""
        point = {arm_id: tuple(values)}
        if other is not None:
            point[other] = tuple(co_arm)
        return point

    def _check(points):
        """Ask the model about one waypoint list; the package's ONE call."""
        return model.check_path(points, first_violation=False)

    # WHY first_violation=False, which is not the cheaper option. The model
    # stops at the first violating sample when it is True and then reports
    # `samples_evaluated` as the number it got through -- so `sample_index`
    # would ALWAYS equal `samples_evaluated - 1`, and a refusal would always
    # read "At the pose you drew", including for a foul a fifth of the way
    # along. Evaluating the whole path is what makes the "where on the way"
    # sentence a fact rather than a guess, and it costs nothing at the bound
    # this module already enforces: a CLEAR path of the same length evaluates
    # every sample anyway, so the worst case is unchanged. `contacts[0]` is
    # still the most-violating contact, but across the WHOLE path rather than
    # within one sample -- which is why the refusal below re-checks the sample
    # the placement names instead of quoting this call's contacts[0].
    try:
        result = _check([_point(measured), _point(held), _point(goal)])
    except Exception as error:            # noqa: BLE001 - never an allow
        raise TravelError('apply_refused', 'checker_error', str(error)) from None

    payload = _model_triple(result)
    payload['min_clearance'] = _finite_or_none(
        getattr(result, 'min_clearance', None))
    payload['sample_index'] = getattr(result, 'sample_index', None)
    payload['samples_evaluated'] = getattr(result, 'samples_evaluated', None)
    if not result.ok:
        # The location, the magnitude, the witness and the tint must all
        # describe ONE point, or the refusal is a false compound statement
        # about the one thing the operator has to act on. See _witness_at.
        at_point = _witness_at(_check, _point, (measured, held, goal),
                               payload['sample_index'],
                               payload['samples_evaluated'])
        if at_point is None:
            # The rebuilt point did not violate, so the placement cannot be
            # made good. Withdraw it -- from the sentence AND from the payload
            # in one move -- rather than pair it with a witness from
            # elsewhere: an unplaced refusal is weaker than a placed one and
            # better than a wrong one.
            payload['sample_index'] = None
            contacts = tuple(result.contacts)
        else:
            contacts = tuple(at_point.contacts)
            payload['min_clearance'] = _finite_or_none(
                getattr(at_point, 'min_clearance', None))
        payload['offending_links'] = offending_links_for(contacts)
        prefix = refusal_prefix(payload['sample_index'],
                                payload['samples_evaluated'])
        raise TravelError('apply_refused', 'contact',
                          prefix + verdict_sentence(contacts[0]), payload)

    checked = _model_triple(result)
    checked['samples_evaluated'] = getattr(result, 'samples_evaluated', None)
    checked['min_clearance'] = _finite_or_none(
        getattr(result, 'min_clearance', None))
    return TravelPlan(
        arm_id=arm_id, q0=held, q1=goal, steps_total=steps_total, step=0,
        enable_epoch=enable_epoch, cancel_gen=cancel_gen,
        co_arm_id=other, co_arm_q=co_arm, checked=checked)


def stop_reason(*, plan, co_arm_q_now, co_arm_fresh, q_measured, q_target):
    """
    Return a stop sentence, or None to keep going. Called every tick.

    Three conditions, in this order: the co-arm went stale; the co-arm drifted
    past ``APPLY_CO_ARM_DRIFT_RAD``; the arm lags its own target past
    ``APPLY_LAG_LIMIT_RAD``. Pure comparisons over floats -- this runs on the
    20 Hz timer thread and must cost nothing.

    Cancel, not re-check. A re-check would be re-deriving authority mid-motion
    from a pose that is itself moving. Cancelling is honest, is stop-and-hold,
    costs the operator one button press to re-plan against the new reality, and
    makes the invariant trivially checkable: *the pair-pose the model approved
    is the pair-pose the travel executes, to within 1 degree.*

    ``q_measured`` may be None when this arm's own sample is missing or torn.
    The lag test is then skipped rather than guessed at: a joint stream that
    has stopped is fault rule F6's business, and that fault clears every travel
    a second later through ``_transition('fault')``. In a dual session it never
    arises, because the co-arm's freshness IS this sample's freshness.
    """
    who = _display(plan.arm_id)
    if plan.co_arm_id is not None:
        other = _display(plan.co_arm_id)
        if not co_arm_fresh or co_arm_q_now is None:
            return STOP_CO_ARM_STALE.format(arm=who, other=other)
        drift = max(abs(float(now) - float(then))
                    for now, then in zip(co_arm_q_now, plan.co_arm_q))
        if drift > defaults.APPLY_CO_ARM_DRIFT_RAD:
            return STOP_CO_ARM_DRIFT.format(arm=who, other=other)
    if q_measured is None or q_target is None:
        return None
    lag = max(abs(float(measured) - float(target))
              for measured, target in zip(q_measured, q_target))
    if lag > defaults.APPLY_LAG_LIMIT_RAD:
        return STOP_LAG.format(arm=who)
    return None


__all__ = ['path_fraction', 'plan_travel', 'refusal_prefix', 'stop_reason',
           'TravelError', 'TravelPlan']
