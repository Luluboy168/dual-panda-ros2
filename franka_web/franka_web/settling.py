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

"""Pure, fail-closed verification of the Watch-to-Motion activation transition."""

from dataclasses import dataclass, field
import hashlib
import json
import math


JOINT_COUNT = 7
POLICY_SEMANTICS = 'franka_web.activation_settling/v1'


def _finite_tuple(name, values, *, allow_zero=False):
    """Normalize one seven-joint positive vector or raise ``ValueError``."""
    if isinstance(values, (str, bytes)):
        raise ValueError('{} must contain exactly 7 values'.format(name))
    try:
        raw = tuple(values)
    except TypeError:
        raise ValueError('{} must contain exactly 7 values'.format(name)) from None
    if any(isinstance(value, bool) for value in raw):
        raise ValueError('{} values must be numeric, not boolean'.format(name))
    try:
        vector = tuple(float(value) for value in raw)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('{} values must be finite numbers'.format(name)) from None
    if len(vector) != JOINT_COUNT:
        raise ValueError('{} must contain exactly 7 values'.format(name))
    for value in vector:
        if not math.isfinite(value) or value < 0.0 or (value == 0.0 and not allow_zero):
            relation = 'non-negative' if allow_zero else 'positive'
            raise ValueError('{} values must be finite and {}'.format(name, relation))
    return vector


def _finite_joint_sample(values):
    """Return seven finite floats, or ``None`` for malformed sample data."""
    try:
        if values is None or len(values) != JOINT_COUNT:
            return None
        if any(isinstance(value, bool) for value in values):
            return None
    except TypeError:
        return None
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(value) for value in result) else None


@dataclass(frozen=True)
class ActivationSettlingPolicy:
    """Prospectively reviewed quantitative rules for one activation transition."""

    max_watch_delta_rad: tuple
    max_position_span_rad: tuple
    max_abs_velocity_rad_s: tuple
    min_fence_margin_rad: tuple
    stable_window_s: float
    min_sample_count: int
    timeout_s: float
    sha256: str = field(init=False)

    def __post_init__(self):
        """Normalize values and derive an unambiguous content digest."""
        object.__setattr__(self, 'max_watch_delta_rad', _finite_tuple(
            'max_watch_delta_rad', self.max_watch_delta_rad))
        object.__setattr__(self, 'max_position_span_rad', _finite_tuple(
            'max_position_span_rad', self.max_position_span_rad))
        object.__setattr__(self, 'max_abs_velocity_rad_s', _finite_tuple(
            'max_abs_velocity_rad_s', self.max_abs_velocity_rad_s))
        object.__setattr__(self, 'min_fence_margin_rad', _finite_tuple(
            'min_fence_margin_rad', self.min_fence_margin_rad, allow_zero=True))
        if isinstance(self.stable_window_s, bool):
            raise ValueError('stable_window_s must be finite and positive')
        if isinstance(self.timeout_s, bool):
            raise ValueError(
                'timeout_s must be finite and greater than stable_window_s')
        stable = float(self.stable_window_s)
        timeout = float(self.timeout_s)
        try:
            samples = int(self.min_sample_count)
        except (TypeError, ValueError, OverflowError):
            raise ValueError(
                'min_sample_count must be an integer of at least 2') from None
        if not math.isfinite(stable) or stable <= 0.0:
            raise ValueError('stable_window_s must be finite and positive')
        if not math.isfinite(timeout) or timeout <= stable:
            raise ValueError('timeout_s must be finite and greater than stable_window_s')
        if (isinstance(self.min_sample_count, bool)
                or self.min_sample_count != samples or samples < 2):
            raise ValueError('min_sample_count must be an integer of at least 2')
        object.__setattr__(self, 'stable_window_s', stable)
        object.__setattr__(self, 'timeout_s', timeout)
        object.__setattr__(self, 'min_sample_count', samples)
        canonical = json.dumps(
            self.canonical_payload(), sort_keys=True, separators=(',', ':')).encode('utf-8')
        object.__setattr__(self, 'sha256', hashlib.sha256(canonical).hexdigest())

    def canonical_payload(self):
        """Return exact normalized values; hexadecimal floats make the digest stable."""
        return {
            'semantics': POLICY_SEMANTICS,
            'max_watch_delta_rad': [value.hex() for value in self.max_watch_delta_rad],
            'max_position_span_rad': [value.hex()
                                      for value in self.max_position_span_rad],
            'max_abs_velocity_rad_s': [value.hex()
                                       for value in self.max_abs_velocity_rad_s],
            'min_fence_margin_rad': [value.hex()
                                     for value in self.min_fence_margin_rad],
            'stable_window_s': self.stable_window_s.hex(),
            'min_sample_count': self.min_sample_count,
            'timeout_s': self.timeout_s.hex(),
        }

    def public_payload(self):
        """Return the reviewed numeric policy in JSON-number form for operator evidence."""
        return {
            'sha256': self.sha256,
            'max_watch_delta_rad': list(self.max_watch_delta_rad),
            'max_position_span_rad': list(self.max_position_span_rad),
            'max_abs_velocity_rad_s': list(self.max_abs_velocity_rad_s),
            'min_fence_margin_rad': list(self.min_fence_margin_rad),
            'stable_window_s': self.stable_window_s,
            'min_sample_count': self.min_sample_count,
            'timeout_s': self.timeout_s,
        }


@dataclass(frozen=True)
class SettlingObservation:
    """One verdict returned by :class:`ActivationSettlingGate`."""

    status: str
    code: str = None
    detail: str = None


class ActivationSampleCapture:
    """Bounded extrema retained across every observed activation callback."""

    def __init__(self, arm_ids, generation):
        """Start one empty capture generation for a fixed, nonempty arm set."""
        self.arm_ids = tuple(arm_ids)
        if not self.arm_ids or len(set(self.arm_ids)) != len(self.arm_ids):
            raise ValueError('capture arm_ids must be nonempty and unique')
        self.generation = int(generation)
        self._reset_interval()

    def add(self, receipt_ns, joints_by_arm):
        """Fold one callback into fixed-size extrema; malformed data is sticky."""
        receipt_ns = int(receipt_ns)
        self.sample_count += 1
        if self.first_receipt_ns is None:
            self.first_receipt_ns = receipt_ns
        self.last_receipt_ns = receipt_ns
        for arm in self.arm_ids:
            joints = (joints_by_arm.get(arm)
                      if isinstance(joints_by_arm, dict) else None)
            positions = (_finite_joint_sample(joints.get('positions'))
                         if isinstance(joints, dict) else None)
            velocities = (_finite_joint_sample(joints.get('velocities'))
                          if isinstance(joints, dict) else None)
            if positions is None or velocities is None:
                self.malformed = True
                continue
            previous = self.arms.get(arm)
            if previous is None:
                minimum = list(positions)
                maximum = list(positions)
                max_velocity = [abs(value) for value in velocities]
            else:
                minimum = [min(old, value) for old, value
                           in zip(previous['min_position_rad'], positions)]
                maximum = [max(old, value) for old, value
                           in zip(previous['max_position_rad'], positions)]
                max_velocity = [max(old, abs(value)) for old, value
                                in zip(previous['max_abs_velocity_rad_s'], velocities)]
            self.arms[arm] = {
                'min_position_rad': minimum,
                'max_position_rad': maximum,
                'max_abs_velocity_rad_s': max_velocity,
                'latest_position_rad': list(positions),
                'latest_velocity_rad_s': list(velocities),
            }

    def drain(self):
        """Return and clear one bounded interval without ending the generation."""
        snapshot = {
            'generation': self.generation,
            'arm_ids': list(self.arm_ids),
            'sample_count': self.sample_count,
            'first_receipt_ns': self.first_receipt_ns,
            'last_receipt_ns': self.last_receipt_ns,
            'malformed': self.malformed,
            'arms': {
                arm: {name: list(values) for name, values in metrics.items()}
                for arm, metrics in self.arms.items()
            },
        }
        self._reset_interval()
        return snapshot

    def _reset_interval(self):
        self.sample_count = 0
        self.first_receipt_ns = None
        self.last_receipt_ns = None
        self.malformed = False
        self.arms = {}


class ActivationSettlingGate:
    """Track fresh samples until every selected arm is stably inside its envelope."""

    def __init__(self, policy, arm_ids, baselines, fences, *,
                 started_mono_ns, barrier_ns, sample_max_age_s):
        self.policy = policy
        self.arm_ids = tuple(arm_ids)
        if not self.arm_ids or len(set(self.arm_ids)) != len(self.arm_ids):
            raise ValueError('arm_ids must be nonempty and unique')
        self.validate_baseline(policy, self.arm_ids, baselines, fences)
        self.baselines = {
            arm: _finite_joint_sample(baselines[arm]) for arm in self.arm_ids}
        self.fences = {
            arm: (_finite_joint_sample(fences[arm][0]),
                  _finite_joint_sample(fences[arm][1]))
            for arm in self.arm_ids
        }
        self.started_mono_ns = int(started_mono_ns)
        self.barrier_ns = int(barrier_ns)
        sample_max_age_s = float(sample_max_age_s)
        if not math.isfinite(sample_max_age_s) or sample_max_age_s <= 0.0:
            raise ValueError('sample_max_age_s must be finite and positive')
        self.sample_max_age_ns = int(sample_max_age_s * 1e9)
        self.last_sample_ns = None
        self.sample_count = 0
        self.stable_sample_count = 0
        self.stable_since_ns = None
        self._stable_min = {}
        self._stable_max = {}
        self.current = {}
        self.max_abs_delta = {arm: [0.0] * JOINT_COUNT for arm in self.arm_ids}
        self.max_abs_velocity = {
            arm: [0.0] * JOINT_COUNT for arm in self.arm_ids}
        self.transition_sample_count = 0
        self.capture_generation = None
        self.status = 'settling'
        self.failure_code = None
        self.failure_detail = None

    @staticmethod
    def validate_baseline(policy, arm_ids, baselines, fences):
        """Prove the full reviewed transition envelope fits before torque activation."""
        for arm in arm_ids:
            try:
                baseline = _finite_joint_sample(baselines[arm])
                lower, upper = fences[arm]
                lower = _finite_joint_sample(lower)
                upper = _finite_joint_sample(upper)
            except (KeyError, TypeError, ValueError):
                raise ValueError(
                    '{} has no complete finite activation baseline/fence'.format(
                        arm)) from None
            if baseline is None or lower is None or upper is None:
                raise ValueError(
                    '{} has no complete finite activation baseline/fence'.format(arm))
            for index, value in enumerate(baseline):
                if lower[index] > upper[index]:
                    raise ValueError(
                        '{} joint{} has an inverted activation fence'.format(
                            arm, index + 1))
                low = float(lower[index]) + policy.min_fence_margin_rad[index]
                high = float(upper[index]) - policy.min_fence_margin_rad[index]
                delta = policy.max_watch_delta_rad[index]
                if value - delta < low or value + delta > high:
                    raise ValueError(
                        '{} joint{} has insufficient reviewed fence margin for the '
                        'activation envelope'.format(arm, index + 1))

    def observe(self, receipt_ns, now_mono_ns, joints_by_arm):
        """Evaluate one distinct post-activation JointState receipt."""
        if self.status != 'settling':
            return SettlingObservation(
                self.status, self.failure_code, self.failure_detail)
        receipt_ns = int(receipt_ns)
        now_mono_ns = int(now_mono_ns)
        timeout = self.timed_out(now_mono_ns)
        if timeout.status == 'failed':
            return timeout
        if receipt_ns <= self.barrier_ns:
            return SettlingObservation('settling')
        if self.last_sample_ns is not None and receipt_ns <= self.last_sample_ns:
            return SettlingObservation('settling')
        if receipt_ns > now_mono_ns:
            return self._fail(
                'activation_settling_limit',
                'a future-dated joint sample cannot verify activation settling')
        if now_mono_ns - receipt_ns > self.sample_max_age_ns:
            return SettlingObservation('settling')

        prepared = {}
        for arm in self.arm_ids:
            joints = joints_by_arm.get(arm)
            if joints is None:
                return self._fail(
                    'activation_settling_limit',
                    '{} has no complete activation joint sample'.format(arm))
            positions = _finite_joint_sample(joints.get('positions'))
            velocities = _finite_joint_sample(joints.get('velocities'))
            if positions is None or velocities is None:
                return self._fail(
                    'activation_settling_limit',
                    '{} activation sample does not carry 7 finite positions '
                    'and velocities'.format(arm))
            prepared[arm] = (positions, velocities)

        stable = True
        for arm, (positions, velocities) in prepared.items():
            lower, upper = self.fences[arm]
            baseline = self.baselines[arm]
            delta_values = []
            lower_margins = []
            upper_margins = []
            for index, position in enumerate(positions):
                delta = position - baseline[index]
                abs_delta = abs(delta)
                self.max_abs_delta[arm][index] = max(
                    self.max_abs_delta[arm][index], abs_delta)
                self.max_abs_velocity[arm][index] = max(
                    self.max_abs_velocity[arm][index], abs(velocities[index]))
                delta_values.append(delta)
                lower_margin = position - lower[index]
                upper_margin = upper[index] - position
                lower_margins.append(lower_margin)
                upper_margins.append(upper_margin)
                if position < lower[index] or position > upper[index]:
                    return self._fail(
                        'activation_settling_limit',
                        '{} joint{} left the reviewed fence during activation'.format(
                            arm, index + 1))
                if (lower_margin < self.policy.min_fence_margin_rad[index]
                        or upper_margin < self.policy.min_fence_margin_rad[index]):
                    return self._fail(
                        'activation_settling_limit',
                        '{} joint{} entered the reserved fence margin during '
                        'activation'.format(arm, index + 1))
                if abs_delta > self.policy.max_watch_delta_rad[index]:
                    return self._fail(
                        'activation_settling_limit',
                        '{} joint{} exceeded the reviewed Watch-to-Motion '
                        'activation displacement'.format(arm, index + 1))
                if abs(velocities[index]) > self.policy.max_abs_velocity_rad_s[index]:
                    stable = False
            self.current[arm] = {
                'delta_rad': delta_values,
                'max_abs_delta_rad': list(self.max_abs_delta[arm]),
                'abs_velocity_rad_s': [abs(value) for value in velocities],
                'max_abs_velocity_rad_s': list(self.max_abs_velocity[arm]),
                'lower_margin_rad': lower_margins,
                'upper_margin_rad': upper_margins,
                'position_span_rad': [0.0] * JOINT_COUNT,
            }

        self.last_sample_ns = receipt_ns
        self.sample_count += 1
        if not stable:
            self._reset_stable_window()
            return SettlingObservation('settling')

        candidate_min = {}
        candidate_max = {}
        for arm, (positions, _velocities) in prepared.items():
            if self.stable_since_ns is None:
                candidate_min[arm] = list(positions)
                candidate_max[arm] = list(positions)
            else:
                candidate_min[arm] = [min(old, value) for old, value
                                      in zip(self._stable_min[arm], positions)]
                candidate_max[arm] = [max(old, value) for old, value
                                      in zip(self._stable_max[arm], positions)]
            spans = [high - low for low, high
                     in zip(candidate_min[arm], candidate_max[arm])]
            self.current[arm]['position_span_rad'] = spans
            if any(span > limit for span, limit
                   in zip(spans, self.policy.max_position_span_rad)):
                self._start_stable_window(receipt_ns, prepared)
                return SettlingObservation('settling')

        if self.stable_since_ns is None:
            self._start_stable_window(receipt_ns, prepared)
        else:
            self._stable_min = candidate_min
            self._stable_max = candidate_max
            self.stable_sample_count += 1
        stable_ns = receipt_ns - self.stable_since_ns
        if (self.stable_sample_count >= self.policy.min_sample_count
                and stable_ns >= math.ceil(
                    self.policy.stable_window_s * 1e9)):
            self.status = 'ready'
            return SettlingObservation('ready')
        return SettlingObservation('settling')

    def observe_capture(self, capture):
        """Apply every callback hidden between supervisor polls to the gate."""
        if self.status == 'failed':
            return SettlingObservation(
                self.status, self.failure_code, self.failure_detail)
        if not isinstance(capture, dict):
            return self._fail(
                'activation_settling_limit',
                'the activation transition capture is unavailable')
        try:
            generation = int(capture['generation'])
            arm_ids = tuple(capture['arm_ids'])
            sample_count = int(capture['sample_count'])
        except (KeyError, TypeError, ValueError, OverflowError):
            return self._fail(
                'activation_settling_limit',
                'the activation transition capture is malformed')
        if self.capture_generation is None:
            self.capture_generation = generation
        if (generation != self.capture_generation or arm_ids != self.arm_ids
                or isinstance(capture.get('sample_count'), bool)
                or sample_count < 0
                or capture.get('sample_count') != sample_count):
            return self._fail(
                'activation_settling_limit',
                'the activation transition capture identity is invalid')
        if capture.get('malformed'):
            return self._fail(
                'activation_settling_limit',
                'an activation callback lacked complete finite joint data')
        if sample_count == 0:
            return SettlingObservation(self.status)
        try:
            first_ns = int(capture['first_receipt_ns'])
            last_ns = int(capture['last_receipt_ns'])
            captured_arms = capture['arms']
        except (KeyError, TypeError, ValueError, OverflowError):
            return self._fail(
                'activation_settling_limit',
                'the activation transition capture timestamps are invalid')
        if first_ns > last_ns:
            return self._fail(
                'activation_settling_limit',
                'the activation transition capture timestamps are inverted')

        unstable = False
        captured_stable_min = {}
        captured_stable_max = {}
        for arm in self.arm_ids:
            metrics = captured_arms.get(arm) if isinstance(captured_arms, dict) else None
            if not isinstance(metrics, dict):
                return self._fail(
                    'activation_settling_limit',
                    '{} is absent from the activation transition capture'.format(arm))
            minimum = _finite_joint_sample(metrics.get('min_position_rad'))
            maximum = _finite_joint_sample(metrics.get('max_position_rad'))
            velocities = _finite_joint_sample(
                metrics.get('max_abs_velocity_rad_s'))
            latest = _finite_joint_sample(metrics.get('latest_position_rad'))
            latest_velocity = _finite_joint_sample(
                metrics.get('latest_velocity_rad_s'))
            if any(value is None for value in (
                    minimum, maximum, velocities, latest, latest_velocity)):
                return self._fail(
                    'activation_settling_limit',
                    '{} has malformed activation transition extrema'.format(arm))
            if any(value < 0.0 for value in velocities):
                return self._fail(
                    'activation_settling_limit',
                    '{} has negative captured absolute velocity'.format(arm))
            lower, upper = self.fences[arm]
            baseline = self.baselines[arm]
            if self.stable_since_ns is None:
                candidate_min = list(minimum)
                candidate_max = list(maximum)
            else:
                try:
                    candidate_min = [min(old, value) for old, value
                                     in zip(self._stable_min[arm], minimum)]
                    candidate_max = [max(old, value) for old, value
                                     in zip(self._stable_max[arm], maximum)]
                except KeyError:
                    return self._fail(
                        'activation_settling_limit',
                        '{} has no stable-window extrema'.format(arm))
            captured_stable_min[arm] = candidate_min
            captured_stable_max[arm] = candidate_max
            stable_spans = [high - low for low, high
                            in zip(candidate_min, candidate_max)]
            for index, (low_seen, high_seen) in enumerate(zip(minimum, maximum)):
                if low_seen > high_seen:
                    return self._fail(
                        'activation_settling_limit',
                        '{} joint{} has inverted captured extrema'.format(
                            arm, index + 1))
                maximum_delta = max(
                    abs(low_seen - baseline[index]),
                    abs(high_seen - baseline[index]))
                self.max_abs_delta[arm][index] = max(
                    self.max_abs_delta[arm][index], maximum_delta)
                self.max_abs_velocity[arm][index] = max(
                    self.max_abs_velocity[arm][index], velocities[index])
                low_margin = low_seen - lower[index]
                high_margin = upper[index] - high_seen
                if low_seen < lower[index] or high_seen > upper[index]:
                    return self._fail(
                        'activation_settling_limit',
                        '{} joint{} left the reviewed fence during activation'.format(
                            arm, index + 1))
                if (low_margin < self.policy.min_fence_margin_rad[index]
                        or high_margin < self.policy.min_fence_margin_rad[index]):
                    return self._fail(
                        'activation_settling_limit',
                        '{} joint{} entered the reserved fence margin during '
                        'activation'.format(arm, index + 1))
                if maximum_delta > self.policy.max_watch_delta_rad[index]:
                    return self._fail(
                        'activation_settling_limit',
                        '{} joint{} exceeded the reviewed Watch-to-Motion '
                        'activation displacement'.format(arm, index + 1))
                if velocities[index] > self.policy.max_abs_velocity_rad_s[index]:
                    unstable = True
                if stable_spans[index] > self.policy.max_position_span_rad[index]:
                    unstable = True
            self.current[arm] = {
                'delta_rad': [value - base for value, base
                              in zip(latest, baseline)],
                'max_abs_delta_rad': list(self.max_abs_delta[arm]),
                'abs_velocity_rad_s': [abs(value) for value in latest_velocity],
                'max_abs_velocity_rad_s': list(self.max_abs_velocity[arm]),
                'lower_margin_rad': [value - low for value, low
                                     in zip(latest, lower)],
                'upper_margin_rad': [high - value for high, value
                                     in zip(upper, latest)],
                'position_span_rad': stable_spans,
            }
        self.transition_sample_count += sample_count
        if unstable:
            self.status = 'settling'
            self._reset_stable_window()
        elif self.stable_since_ns is not None:
            # Captured callbacks do not earn sample-count/time credit, but
            # every one remains part of the current stability envelope.
            self._stable_min = captured_stable_min
            self._stable_max = captured_stable_max
        return SettlingObservation(self.status)

    def timed_out(self, now_mono_ns):
        """Return a failure once the reviewed total settling time expires."""
        if self.status == 'settling' and self.deadline_reached(now_mono_ns):
            return self._fail(
                'activation_settling_timeout',
                'activation did not become stable within the reviewed timeout')
        return SettlingObservation(self.status, self.failure_code, self.failure_detail)

    def deadline_reached(self, now_mono_ns):
        """Return whether the inclusive activation deadline has arrived."""
        return (int(now_mono_ns) - self.started_mono_ns
                >= math.ceil(self.policy.timeout_s * 1e9))

    def finalize_capture(self, capture, now_mono_ns):
        """Apply final extrema, then enforce the inclusive deadline even if ready."""
        verdict = self.observe_capture(capture)
        if verdict.status == 'failed':
            return verdict
        if self.deadline_reached(now_mono_ns):
            return self._fail(
                'activation_settling_timeout',
                'activation did not become stable within the reviewed timeout')
        return verdict

    def stable_for_s(self):
        """Return the duration covered by the current accepted sample window."""
        if self.stable_since_ns is None or self.last_sample_ns is None:
            return 0.0
        return max(0.0, (self.last_sample_ns - self.stable_since_ns) / 1e9)

    def frame(self):
        """Return bounded, address-free activation evidence for the state stream."""
        return {
            'status': self.status,
            'policy_sha256': self.policy.sha256,
            'samples': self.sample_count,
            'transition_samples': self.transition_sample_count,
            'stable_samples': self.stable_sample_count,
            'stable_for_s': round(self.stable_for_s(), 3),
            'required_stable_s': self.policy.stable_window_s,
            'required_samples': self.policy.min_sample_count,
        }

    def _reset_stable_window(self):
        self.stable_since_ns = None
        self.stable_sample_count = 0
        self._stable_min = {}
        self._stable_max = {}

    def _start_stable_window(self, receipt_ns, prepared):
        self.stable_since_ns = receipt_ns
        self.stable_sample_count = 1
        self._stable_min = {
            arm: list(prepared[arm][0]) for arm in self.arm_ids}
        self._stable_max = {
            arm: list(prepared[arm][0]) for arm in self.arm_ids}

    def _fail(self, code, detail):
        self.status = 'failed'
        self.failure_code = code
        self.failure_detail = detail
        return SettlingObservation('failed', code, detail)
