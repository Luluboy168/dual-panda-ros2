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

"""Pure offline tests for the fail-closed activation-settling policy and gate."""

from dataclasses import replace
import math

from franka_web.settling import (
    ActivationSampleCapture,
    ActivationSettlingGate,
    ActivationSettlingPolicy,
    POLICY_SEMANTICS,
)
import pytest


# These deliberately varied values are synthetic test data, not proposed robot
# limits.  Production has no default policy and must receive separately reviewed
# values through the environment.
WATCH_DELTA = (0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17)
POSITION_SPAN = (0.0011, 0.0012, 0.0013, 0.0014, 0.0015, 0.0016, 0.0017)
ABS_VELOCITY = (0.021, 0.022, 0.023, 0.024, 0.025, 0.026, 0.027)
FENCE_MARGIN = (0.031, 0.032, 0.033, 0.034, 0.035, 0.036, 0.037)


def policy(**changes):
    """Build one wholly synthetic policy, optionally replacing named fields."""
    values = {
        'max_watch_delta_rad': WATCH_DELTA,
        'max_position_span_rad': POSITION_SPAN,
        'max_abs_velocity_rad_s': ABS_VELOCITY,
        'min_fence_margin_rad': FENCE_MARGIN,
        'stable_window_s': 0.7,
        'min_sample_count': 5,
        'timeout_s': 4.0,
    }
    values.update(changes)
    return ActivationSettlingPolicy(**values)


def gate_for(policy_value=None, arm_ids=('panda1',), *, baseline=None,
             lower=None, upper=None, started_ns=0, barrier_ns=10):
    """Build a gate around synthetic poses and broad synthetic fences."""
    policy_value = policy_value or policy()
    baseline = tuple(baseline or [0.0] * 7)
    lower = tuple(lower or [-1.0] * 7)
    upper = tuple(upper or [1.0] * 7)
    return ActivationSettlingGate(
        policy_value,
        arm_ids,
        {arm: baseline for arm in arm_ids},
        {arm: (lower, upper) for arm in arm_ids},
        started_mono_ns=started_ns,
        barrier_ns=barrier_ns,
        sample_max_age_s=0.2,
    )


def joints(position=None, velocity=None):
    """Return one complete synthetic seven-joint observation."""
    return {
        'positions': list(position or [0.0] * 7),
        'velocities': list(velocity or [0.0] * 7),
    }


def observe(gate, receipt_ns, *, now_ns=None, by_arm=None,
            position=None, velocity=None):
    """Submit one sample, defaulting to the gate's selected arms."""
    if by_arm is None:
        sample = joints(position=position, velocity=velocity)
        by_arm = {arm: sample for arm in gate.arm_ids}
    return gate.observe(
        receipt_ns,
        receipt_ns if now_ns is None else now_ns,
        by_arm,
    )


class TestActivationSampleCapture:
    """Callback-rate extrema cannot disappear behind the latest-value cache."""

    def test_excursion_and_return_are_preserved_in_one_bounded_snapshot(self):
        """A safe latest sample cannot erase an earlier observed excursion."""
        capture = ActivationSampleCapture(('panda1',), generation=7)
        moved = [0.0] * 7
        moved[1] = 0.25
        capture.add(100, {'panda1': joints(position=moved, velocity=[0.4] * 7)})
        capture.add(200, {'panda1': joints()})
        value = capture.drain()
        assert value['generation'] == 7
        assert value['sample_count'] == 2
        assert value['arms']['panda1']['max_position_rad'][1] == 0.25
        assert value['arms']['panda1']['latest_position_rad'][1] == 0.0
        assert value['arms']['panda1']['max_abs_velocity_rad_s'] == \
            pytest.approx([0.4] * 7)

    def test_drain_resets_only_the_interval_and_malformed_data_is_sticky(self):
        """Memory stays bounded while generation identity survives every drain."""
        capture = ActivationSampleCapture(('panda1',), generation=3)
        capture.add(100, {'panda1': {'positions': [0.0] * 7, 'velocities': []}})
        first = capture.drain()
        second = capture.drain()
        assert first['malformed'] is True
        assert first['sample_count'] == 1
        assert second == {
            'generation': 3,
            'arm_ids': ['panda1'],
            'sample_count': 0,
            'first_receipt_ns': None,
            'last_receipt_ns': None,
            'malformed': False,
            'arms': {},
        }

    def test_gate_rejects_captured_excursion_even_after_return(self):
        """The hard Watch displacement applies to every callback, not one endpoint."""
        value = gate_for()
        capture = ActivationSampleCapture(('panda1',), generation=1)
        moved = [0.0] * 7
        moved[0] = WATCH_DELTA[0] + 0.001
        capture.add(100, {'panda1': joints(position=moved)})
        capture.add(200, {'panda1': joints()})
        verdict = value.observe_capture(capture.drain())
        assert verdict.status == 'failed'
        assert verdict.code == 'activation_settling_limit'

    @pytest.mark.parametrize('sign', [1, -1], ids=['upper', 'lower'])
    def test_captured_extremum_entering_the_reserved_margin_fails_hard(
            self, sign):
        """
        The reserved margin binds callback extrema, not just polled samples.

        `observe` has two dedicated tests for this pair of checks; the
        between-poll capture path had none, so the excursion could be judged
        only by the drift bound that happens to subsume it today -- and the
        operator would be told the wrong thing about their own configuration.
        """
        value = gate_for(
            policy(max_watch_delta_rad=tuple(
                1.0 - entry for entry in FENCE_MARGIN)),
            lower=(-1.0,) * 7, upper=(1.0,) * 7)
        moved = [0.0] * 7
        moved[3] = sign * (1.0 - FENCE_MARGIN[3] + 0.0001)
        capture = ActivationSampleCapture(('panda1',), generation=1)
        capture.add(100, {'panda1': joints(position=moved)})
        capture.add(200, {'panda1': joints()})

        verdict = value.observe_capture(capture.drain())

        assert verdict.status == 'failed'
        assert verdict.code == 'activation_settling_limit'
        assert 'panda1 joint4' in verdict.detail
        assert 'reserved fence margin' in verdict.detail

    def test_captured_extremum_at_the_exact_reserved_margin_is_inclusive(self):
        """An exact reviewed margin is accepted on the capture path too."""
        value = gate_for(
            policy(max_watch_delta_rad=tuple(
                1.0 - entry for entry in FENCE_MARGIN)),
            lower=(-1.0,) * 7, upper=(1.0,) * 7)
        moved = [0.0] * 7
        moved[3] = 1.0 - FENCE_MARGIN[3]
        capture = ActivationSampleCapture(('panda1',), generation=1)
        capture.add(100, {'panda1': joints(position=moved)})
        assert value.observe_capture(capture.drain()).status == 'settling'

    def test_gate_rejects_capture_generation_change(self):
        """One activation gate accepts exactly one continuous capture identity."""
        value = gate_for()
        first = ActivationSampleCapture(('panda1',), generation=1)
        assert value.observe_capture(first.drain()).status == 'settling'
        replacement = ActivationSampleCapture(('panda1',), generation=2)
        verdict = value.observe_capture(replacement.drain())
        assert verdict.status == 'failed'
        assert verdict.code == 'activation_settling_limit'

    def test_gate_accepts_repeated_drains_from_same_capture_generation(self):
        """Draining one continuously armed capture preserves its identity."""
        value = gate_for()
        capture = ActivationSampleCapture(('panda1',), generation=4)
        assert value.observe_capture(capture.drain()).status == 'settling'
        capture.add(100, {'panda1': joints()})
        assert value.observe_capture(capture.drain()).status == 'settling'
        assert value.capture_generation == 4

    def test_captured_velocity_between_polls_resets_a_ready_window(self):
        """A high-velocity callback cannot hide behind a later quiet sample."""
        value = gate_for(policy(stable_window_s=0.1, min_sample_count=2))
        observe(value, 100_000_000)
        assert observe(value, 200_000_000).status == 'ready'
        capture = ActivationSampleCapture(('panda1',), generation=1)
        capture.add(210_000_000, {
            'panda1': joints(velocity=[ABS_VELOCITY[0] + 0.001] * 7)})
        capture.add(220_000_000, {'panda1': joints()})
        verdict = value.observe_capture(capture.drain())
        assert verdict.status == 'settling'
        assert value.stable_sample_count == 0

    def test_single_shifted_final_callback_joins_the_existing_stable_span(self):
        """A one-sample capture is compared with earlier stable-window extrema."""
        value = gate_for(policy(
            max_position_span_rad=(0.01,) * 7,
            stable_window_s=0.1,
            min_sample_count=2))
        observe(value, 100_000_000)
        assert observe(value, 200_000_000).status == 'ready'
        shifted = [0.0] * 7
        shifted[0] = 0.02
        capture = ActivationSampleCapture(('panda1',), generation=1)
        capture.add(210_000_000, {'panda1': joints(position=shifted)})
        verdict = value.observe_capture(capture.drain())
        assert verdict.status == 'settling'
        assert value.stable_sample_count == 0

    def test_finalizer_rechecks_inclusive_timeout_after_ready_poll(self):
        """Preemption after a ready sample cannot open Running past the deadline."""
        value = gate_for(policy(
            stable_window_s=0.1, min_sample_count=2, timeout_s=0.3))
        observe(value, 100_000_000)
        assert observe(value, 200_000_000).status == 'ready'
        capture = ActivationSampleCapture(('panda1',), generation=1)
        verdict = value.finalize_capture(capture.drain(), 300_000_000)
        assert verdict.status == 'failed'
        assert verdict.code == 'activation_settling_timeout'

    def test_hard_final_extrema_keep_priority_at_the_timeout_boundary(self):
        """A captured envelope violation remains the more specific final verdict."""
        value = gate_for(policy(
            stable_window_s=0.1, min_sample_count=2, timeout_s=0.3))
        observe(value, 100_000_000)
        assert observe(value, 200_000_000).status == 'ready'
        moved = [0.0] * 7
        moved[0] = WATCH_DELTA[0] + 0.001
        capture = ActivationSampleCapture(('panda1',), generation=1)
        capture.add(250_000_000, {'panda1': joints(position=moved)})
        verdict = value.finalize_capture(capture.drain(), 300_000_000)
        assert verdict.status == 'failed'
        assert verdict.code == 'activation_settling_limit'


class TestActivationSettlingPolicy:
    """The policy is immutable, normalized, and content addressed."""

    def test_normalizes_sequences_and_public_payload_is_a_copy(self):
        """Mutable inputs become tuples and public JSON receives fresh lists."""
        value = policy(max_watch_delta_rad=list(WATCH_DELTA))
        assert value.max_watch_delta_rad == WATCH_DELTA
        assert isinstance(value.max_watch_delta_rad, tuple)
        payload = value.public_payload()
        payload['max_watch_delta_rad'][0] = 99.0
        assert value.max_watch_delta_rad[0] == WATCH_DELTA[0]
        assert payload['sha256'] == value.sha256

    @pytest.mark.parametrize('field', ['stable_window_s', 'timeout_s'])
    def test_boolean_duration_is_rejected(self, field):
        """Direct construction is as strict as text environment parsing."""
        values = {field: True}
        with pytest.raises(ValueError):
            policy(**values)

    def test_digest_has_frozen_known_answer_and_semantics(self):
        """Canonical hexadecimal floats make the evidence digest reproducible."""
        value = policy()
        assert POLICY_SEMANTICS == 'franka_web.activation_settling/v1'
        assert value.sha256 == (
            'aff1aa29347e61d4d7a7973b501e1379453ad18b6dbb3a3c92fe910c10811b61')
        assert value.canonical_payload()['semantics'] == POLICY_SEMANTICS
        assert all(text.startswith('0x')
                   for text in value.canonical_payload()['max_watch_delta_rad'])

    def test_numerically_equivalent_inputs_have_the_same_digest(self):
        """List/tuple, integer/float, and equivalent spelling do not alter identity."""
        original = policy()
        equivalent = ActivationSettlingPolicy(
            max_watch_delta_rad=[float(str(value)) for value in WATCH_DELTA],
            max_position_span_rad=list(POSITION_SPAN),
            max_abs_velocity_rad_s=list(ABS_VELOCITY),
            min_fence_margin_rad=list(FENCE_MARGIN),
            stable_window_s=float('0.70'),
            min_sample_count=5,
            timeout_s=4,
        )
        assert equivalent.sha256 == original.sha256

    @pytest.mark.parametrize('field_name,replacement', [
        ('max_watch_delta_rad', WATCH_DELTA[:-1] + (0.18,)),
        ('max_position_span_rad', POSITION_SPAN[:-1] + (0.0018,)),
        ('max_abs_velocity_rad_s', ABS_VELOCITY[:-1] + (0.028,)),
        ('min_fence_margin_rad', FENCE_MARGIN[:-1] + (0.038,)),
        ('stable_window_s', 0.8),
        ('min_sample_count', 6),
        ('timeout_s', 4.1),
    ])
    def test_every_policy_field_changes_the_digest(self, field_name, replacement):
        """No reviewed policy quantity is omitted from the content identity."""
        original = policy()
        assert replace(original, **{field_name: replacement}).sha256 != original.sha256

    @pytest.mark.parametrize('field_name', [
        'max_watch_delta_rad',
        'max_position_span_rad',
        'max_abs_velocity_rad_s',
    ])
    @pytest.mark.parametrize('bad_vector', [
        (0.1,) * 6,
        (0.1,) * 8,
        (0.1, 0.1, 0.1, 0.0, 0.1, 0.1, 0.1),
        (0.1, 0.1, 0.1, -0.1, 0.1, 0.1, 0.1),
        (0.1, 0.1, 0.1, math.nan, 0.1, 0.1, 0.1),
        (0.1, 0.1, 0.1, math.inf, 0.1, 0.1, 0.1),
    ])
    def test_positive_vectors_reject_wrong_shape_or_value(
            self, field_name, bad_vector):
        """Every non-margin vector is exactly seven finite positive values."""
        with pytest.raises(ValueError):
            policy(**{field_name: bad_vector})

    @pytest.mark.parametrize('bad_vector', [
        (0.1,) * 6,
        (0.1,) * 8,
        (0.1, 0.1, 0.1, -0.1, 0.1, 0.1, 0.1),
        (0.1, 0.1, 0.1, math.nan, 0.1, 0.1, 0.1),
        (0.1, 0.1, 0.1, math.inf, 0.1, 0.1, 0.1),
    ])
    def test_margin_vector_rejects_wrong_shape_negative_or_nonfinite(self, bad_vector):
        """The margin alone permits zero; its remaining contract stays strict."""
        with pytest.raises(ValueError):
            policy(min_fence_margin_rad=bad_vector)

    def test_zero_fence_margin_is_explicitly_supported(self):
        """Zero margin is a valid reviewed choice rather than an unset sentinel."""
        value = policy(min_fence_margin_rad=(0.0,) * 7)
        assert value.min_fence_margin_rad == (0.0,) * 7

    @pytest.mark.parametrize('changes', [
        {'stable_window_s': 0.0},
        {'stable_window_s': math.nan},
        {'stable_window_s': math.inf},
        {'timeout_s': 0.7},
        {'timeout_s': 0.6},
        {'timeout_s': math.nan},
        {'timeout_s': math.inf},
        {'min_sample_count': True},
        {'min_sample_count': 1},
        {'min_sample_count': 2.5},
        {'min_sample_count': '2'},
    ])
    def test_scalar_contract_rejects_ambiguous_or_invalid_values(self, changes):
        """Direct construction is as strict as environment construction."""
        with pytest.raises((TypeError, ValueError)):
            policy(**changes)


class TestBaselineEnvelope:
    """The full allowed activation envelope must fit inside the inner fence."""

    def test_valid_baseline_envelope_for_both_arms(self):
        """A centered pose with ample margin passes for every selected arm."""
        value = policy()
        ActivationSettlingGate.validate_baseline(
            value,
            ('panda1', 'panda2'),
            {'panda1': (0.0,) * 7, 'panda2': (0.1,) * 7},
            {'panda1': ((-1.0,) * 7, (1.0,) * 7),
             'panda2': ((-1.0,) * 7, (1.0,) * 7)},
        )

    @pytest.mark.parametrize('baseline_value', [0.90, -0.90])
    def test_insufficient_upper_or_lower_envelope_is_refused(self, baseline_value):
        """Both directions reserve max delta plus the reviewed margin."""
        with pytest.raises(ValueError, match='panda2 joint4'):
            ActivationSettlingGate.validate_baseline(
                policy(),
                ('panda2',),
                {'panda2': (0.0, 0.0, 0.0, baseline_value, 0.0, 0.0, 0.0)},
                {'panda2': ((-1.0,) * 7, (1.0,) * 7)},
            )

    @pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
    def test_nonfinite_baseline_is_refused(self, bad):
        """A corrupt Watch baseline can never silently pass comparisons."""
        baseline = [0.0] * 7
        baseline[2] = bad
        with pytest.raises(ValueError, match='panda1.*finite'):
            ActivationSettlingGate.validate_baseline(
                policy(), ('panda1',), {'panda1': baseline},
                {'panda1': ((-1.0,) * 7, (1.0,) * 7)})

    @pytest.mark.parametrize('bound,bad', [('lower', math.nan), ('upper', math.inf)])
    def test_nonfinite_fence_is_refused(self, bound, bad):
        """Nonfinite uploaded bounds fail closed before activation."""
        lower = [-1.0] * 7
        upper = [1.0] * 7
        (lower if bound == 'lower' else upper)[4] = bad
        with pytest.raises(ValueError, match='panda1.*finite'):
            ActivationSettlingGate.validate_baseline(
                policy(), ('panda1',), {'panda1': (0.0,) * 7},
                {'panda1': (lower, upper)})


class TestObservationFreshness:
    """Only fresh, distinct, post-activation observations contribute evidence."""

    def test_prebarrier_duplicate_and_stale_samples_do_not_count(self):
        """Cached or repeated data cannot manufacture the required sample count."""
        value = gate_for()
        assert observe(value, 10).status == 'settling'  # equal to barrier
        assert observe(value, 11).status == 'settling'
        assert observe(value, 11).status == 'settling'  # duplicate
        assert observe(value, 12, now_ns=300_000_013).status == 'settling'  # stale
        assert value.sample_count == 1
        assert value.stable_sample_count == 1

    def test_future_sample_fails_hard(self):
        """A future receipt timestamp is corrupt evidence, not a wait condition."""
        value = gate_for()
        result = observe(value, 100, now_ns=99)
        assert result.status == 'failed'
        assert result.code == 'activation_settling_limit'
        assert 'future-dated' in result.detail

    @pytest.mark.parametrize('bad_joints,fragment', [
        (None, 'no complete'),
        ({'positions': [0.0] * 6, 'velocities': [0.0] * 7}, '7 finite'),
        ({'positions': [0.0] * 7, 'velocities': [0.0] * 6}, '7 finite'),
        ({'positions': [0.0] * 6 + [math.nan], 'velocities': [0.0] * 7},
         '7 finite'),
        ({'positions': [0.0] * 7, 'velocities': [0.0] * 6 + [math.inf]},
         '7 finite'),
    ])
    def test_missing_incomplete_or_nonfinite_sample_fails_hard(
            self, bad_joints, fragment):
        """Shape/value uncertainty cannot authorize torque-active operation."""
        value = gate_for()
        result = observe(value, 100, by_arm={'panda1': bad_joints})
        assert result.status == 'failed'
        assert result.code == 'activation_settling_limit'
        assert fragment in result.detail

    def test_dual_sample_requires_both_arms(self):
        """One healthy arm cannot stand in for a missing sibling observation."""
        value = gate_for(arm_ids=('panda1', 'panda2'))
        result = observe(value, 100, by_arm={'panda1': joints()})
        assert result.status == 'failed'
        assert 'panda2' in result.detail


class TestHardLimits:
    """Excursion and fence violations immediately and permanently close the gate."""

    def test_watch_delta_violation_names_exact_arm_and_joint(self):
        """The hard transition envelope is measured from the Watch baseline."""
        value = gate_for()
        positions = [0.0] * 7
        positions[1] = WATCH_DELTA[1] + 0.0001
        result = observe(value, 100, position=positions)
        assert result.status == 'failed'
        assert result.code == 'activation_settling_limit'
        assert 'panda1 joint2' in result.detail
        assert 'Watch-to-Motion' in result.detail

    def test_exact_watch_delta_is_inclusive(self):
        """Equality with the reviewed maximum does not fail due to boundary drift."""
        value = gate_for()
        positions = [0.0] * 7
        positions[1] = WATCH_DELTA[1]
        assert observe(value, 100, position=positions).status == 'settling'

    def test_leaving_outer_fence_fails_hard(self):
        """The physical fence remains authoritative independently of delta."""
        value = gate_for(
            policy(max_watch_delta_rad=(0.9,) * 7),
            lower=(-1.0,) * 7, upper=(1.0,) * 7)
        positions = [0.0] * 7
        positions[4] = 1.0001
        result = observe(value, 100, position=positions)
        assert result.status == 'failed'
        assert 'left the reviewed fence' in result.detail

    def test_entering_reserved_inner_margin_fails_hard(self):
        """Remaining inside the outer fence is insufficient once margin is reserved."""
        value = gate_for(
            policy(max_watch_delta_rad=tuple(
                1.0 - value for value in FENCE_MARGIN)),
            lower=(-1.0,) * 7, upper=(1.0,) * 7)
        positions = [0.0] * 7
        positions[3] = 1.0 - FENCE_MARGIN[3] + 0.0001
        result = observe(value, 100, position=positions)
        assert result.status == 'failed'
        assert 'reserved fence margin' in result.detail

    def test_exact_inner_margin_is_inclusive(self):
        """An exact reviewed margin is accepted."""
        value = gate_for(
            policy(max_watch_delta_rad=tuple(
                1.0 - value for value in FENCE_MARGIN)),
            lower=(-1.0,) * 7, upper=(1.0,) * 7)
        positions = [0.0] * 7
        positions[3] = 1.0 - FENCE_MARGIN[3]
        assert observe(value, 100, position=positions).status == 'settling'

    def test_failure_verdict_is_sticky(self):
        """Later good data cannot reopen a gate that crossed a hard limit."""
        value = gate_for()
        positions = [0.0] * 7
        positions[0] = WATCH_DELTA[0] + 0.1
        first = observe(value, 100, position=positions)
        later = observe(value, 200, position=[0.0] * 7)
        assert later == first
        assert value.sample_count == 0


class TestStableWindow:
    """Readiness requires velocity, position span, duration, and sample count together."""

    def test_ready_requires_both_minimum_count_and_elapsed_window(self):
        """Neither a rapid burst nor two far-apart samples can independently pass."""
        count_limited = gate_for(policy(stable_window_s=0.2, min_sample_count=3))
        assert observe(count_limited, 100_000_000).status == 'settling'
        assert observe(count_limited, 200_000_000).status == 'settling'
        assert observe(count_limited, 300_000_000).status == 'ready'

        time_limited = gate_for(policy(stable_window_s=0.2, min_sample_count=4))
        assert observe(time_limited, 100_000_000).status == 'settling'
        assert observe(time_limited, 300_000_000).status == 'settling'
        assert time_limited.stable_sample_count == 2

        duration_limited = gate_for(policy(stable_window_s=0.2, min_sample_count=3))
        assert observe(duration_limited, 100_000_000).status == 'settling'
        assert observe(duration_limited, 110_000_000).status == 'settling'
        assert observe(duration_limited, 120_000_000).status == 'settling'
        assert duration_limited.stable_sample_count == 3

    def test_high_velocity_resets_the_complete_stable_window(self):
        """A single moving sample discards prior stable dwell and count."""
        value = gate_for(policy(stable_window_s=0.2, min_sample_count=3))
        observe(value, 100_000_000)
        observe(value, 200_000_000)
        velocities = [0.0] * 7
        velocities[5] = ABS_VELOCITY[5] + 0.0001
        assert observe(value, 250_000_000, velocity=velocities).status == 'settling'
        assert value.stable_sample_count == 0
        assert value.stable_since_ns is None
        assert observe(value, 300_000_000).status == 'settling'
        assert observe(value, 400_000_000).status == 'settling'
        assert observe(value, 500_000_000).status == 'ready'

    def test_exact_velocity_limit_is_stable(self):
        """Equality with a reviewed velocity maximum remains eligible."""
        value = gate_for(policy(stable_window_s=0.1, min_sample_count=2))
        velocities = list(ABS_VELOCITY)
        assert observe(value, 100_000_000, velocity=velocities).status == 'settling'
        assert observe(value, 200_000_000, velocity=velocities).status == 'ready'

    def test_sub_nanosecond_window_rounds_up(self):
        """Runtime uses the same conservative duration rounding as config."""
        value = gate_for(policy(
            stable_window_s=0.1000000001, min_sample_count=2))
        assert observe(value, 100_000_000).status == 'settling'
        assert observe(value, 200_000_000).status == 'settling'
        assert observe(value, 200_000_001).status == 'ready'

    def test_position_span_violation_restarts_at_the_current_sample(self):
        """Excess peak-to-peak motion cannot retain earlier dwell evidence."""
        value = gate_for(policy(stable_window_s=0.2, min_sample_count=3))
        observe(value, 100_000_000)
        positions = [0.0] * 7
        positions[2] = POSITION_SPAN[2] + 0.0001
        assert observe(value, 200_000_000, position=positions).status == 'settling'
        assert value.stable_since_ns == 200_000_000
        assert value.stable_sample_count == 1
        assert observe(value, 300_000_000, position=positions).status == 'settling'
        assert observe(value, 400_000_000, position=positions).status == 'ready'

    def test_exact_position_span_is_eligible(self):
        """Equality with the reviewed peak-to-peak span is accepted."""
        value = gate_for(policy(stable_window_s=0.1, min_sample_count=2))
        observe(value, 100_000_000)
        positions = [0.0] * 7
        positions[0] = POSITION_SPAN[0]
        assert observe(value, 200_000_000, position=positions).status == 'ready'
        assert value.current['panda1']['position_span_rad'][0] == pytest.approx(
            POSITION_SPAN[0])

    def test_ready_verdict_is_sticky(self):
        """The gate is one-shot; subsequent observations cannot alter accepted evidence."""
        value = gate_for(policy(stable_window_s=0.1, min_sample_count=2))
        observe(value, 100_000_000)
        ready = observe(value, 200_000_000)
        later = observe(value, 300_000_000, velocity=[99.0] * 7)
        assert ready.status == 'ready'
        assert later.status == 'ready'
        assert value.sample_count == 2


class TestTimeoutAndEvidence:
    """The total transition is bounded and exposes compact durable evidence."""

    def test_timeout_is_inclusive_and_sticky(self):
        """The exact reviewed deadline closes the gate; later data cannot reopen it."""
        value = gate_for(policy(timeout_s=2.0), started_ns=100)
        assert value.timed_out(2_000_000_099).status == 'settling'
        timed_out = value.timed_out(2_000_000_100)
        assert timed_out.status == 'failed'
        assert timed_out.code == 'activation_settling_timeout'
        assert value.timed_out(3_000_000_000) == timed_out
        assert observe(value, 3_000_000_001).status == 'failed'

    def test_sub_nanosecond_timeout_rounds_up(self):
        """A fractional-nanosecond deadline never fires earlier than reviewed."""
        value = gate_for(policy(timeout_s=2.0000000001), started_ns=100)
        assert value.timed_out(2_000_000_100).status == 'settling'
        assert value.timed_out(2_000_000_101).status == 'failed'

    def test_late_good_observation_cannot_beat_the_deadline_check(self):
        """A ready-looking sample after timeout fails without a prior timeout poll."""
        value = gate_for(policy(
            stable_window_s=0.1, min_sample_count=2, timeout_s=0.3))
        assert observe(value, 100_000_000).status == 'settling'
        result = observe(value, 400_000_000)
        assert result.status == 'failed'
        assert result.code == 'activation_settling_timeout'
        assert value.sample_count == 1

    def test_ready_gate_does_not_later_time_out(self):
        """A completed one-shot gate retains its accepted status past the deadline."""
        value = gate_for(policy(stable_window_s=0.1, min_sample_count=2, timeout_s=1.0))
        observe(value, 100_000_000)
        observe(value, 200_000_000)
        assert value.timed_out(2_000_000_000).status == 'ready'

    def test_frame_contains_policy_and_progress_but_no_pose_or_address(self):
        """Public progress is bounded and binds the exact reviewed policy."""
        value = gate_for(policy(stable_window_s=0.2, min_sample_count=3))
        positions = [0.0] * 7
        positions[6] = 0.05
        observe(value, 100_000_000, position=positions)
        frame = value.frame()
        assert frame == {
            'status': 'settling',
            'policy_sha256': value.policy.sha256,
            'samples': 1,
            'transition_samples': 0,
            'stable_samples': 1,
            'stable_for_s': 0.0,
            'required_stable_s': 0.2,
            'required_samples': 3,
        }
        rendered = repr(frame)
        assert 'panda1' not in rendered
        assert 'positions' not in rendered

    def test_current_evidence_tracks_peak_delta_and_margin(self):
        """Internal evidence retains the worst observed delta, not merely the latest."""
        value = gate_for()
        first = [0.0] * 7
        first[0] = 0.08
        second = [0.0] * 7
        second[0] = 0.02
        observe(value, 100_000_000, position=first)
        observe(value, 200_000_000, position=second)
        evidence = value.current['panda1']
        assert evidence['delta_rad'][0] == pytest.approx(0.02)
        assert evidence['max_abs_delta_rad'][0] == pytest.approx(0.08)
        assert evidence['lower_margin_rad'][0] == pytest.approx(1.02)
        assert evidence['upper_margin_rad'][0] == pytest.approx(0.98)
