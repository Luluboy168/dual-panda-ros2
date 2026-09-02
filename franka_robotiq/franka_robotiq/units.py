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
The only file in this package that converts between counts and real units.

User-facing units are millimetres, newtons and millimetres per second. The
gripper speaks counts, 0 to 255. Every conversion between the two lives here,
so the one place a millimetre error can be introduced is one file with one
test.

**Two of these conversions are honest guesses, and this module says so rather
than presenting them as exact.**

    ``speed -> rSP`` and ``force -> rFR`` are linear interpolations between
    documented endpoints, not manufacturer curves. Robotiq publishes only the
    endpoints (S1 section 4.3: ``0x00`` minimum, ``0xFF`` maximum) and the
    ranges (S1 section 6.2 and the product sheet: 20-150 mm/s, 20-235 N).
    Linearity between them is assumed. S1 section 4.5.1's measured force table
    is the authority for what a given fingertip/payload pair actually
    produces, and force repeatability is plus or minus 10 percent. Do not
    present these as exact.

The position mapping is different in kind: S1 section 4.3 states ``0x00`` is
fully open and ``0xFF`` fully closed, describes the relationship as
quasi-linear, and states that activation re-calibrates the endpoints to
whatever fingertips are fitted -- so the stops always mean the real mechanical
stops. This module implements the mapping the manual describes.

One count is 85 / 255 = 0.333 mm. S1 section 6.2 and the product sheet both
print "0.4 mm" as the position resolution, but 255 counts at 0.4 mm would be
102 mm against an 85 mm stroke: 0.4 mm is a nominal fingertip figure, not the
scale factor. Nobody should "correct" the map to 0.4 mm per count.

**This module CLAMPS; it does not GATE.** An out-of-range width is clamped
into 0..255 and no exception is raised, because these functions are the
defensive floor under every caller, not a policy layer. Refusing an
out-of-stroke *goal*, with the sentence an operator reads, is the node's job
and the node does it on the metres it was handed, before any conversion. Only
non-finite input raises, because a silently clamped NaN becomes count 0 --
a full-open command produced by a typo.

Imports: ``math`` only. This module never imports ``registers``: units are
physics, registers are wire, and they meet in the driver and the node.
"""

import math

#: S1 section 6.2 / product sheet: the 2F-85 opens 0..85 mm.
STROKE_MM = 85.0
COUNT_MAX = 255
#: INTERPOLATION, NOT DOCUMENTED (see the module docstring).
SPEED_RANGE_MM_S = (20.0, 150.0)
#: INTERPOLATION, NOT DOCUMENTED (see the module docstring).
FORCE_RANGE_N = (20.0, 235.0)
#: S1 section 4.4: gCU is "approximately 10 x value" in mA.
CURRENT_MA_PER_COUNT = 10.0
#: The "already there" band for a position goal. One count is 85/255 =
#: 0.333 mm, so 0.2 mm is 0.6 of a count: a goal inside this band cannot
#: change rPR, and a serial write would be a no-op.
WIDTH_EPSILON_MM = 0.2


def _reject_non_finite(name, value):
    """Raise ValueError on NaN or infinity, naming the argument."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('{} must be a finite number, not {!r}'.format(name, value))
    return value


def _clamp_count(value):
    """Clamp a computed count into 0..COUNT_MAX, half-up rounded."""
    count = int(math.floor(value + 0.5))
    if count < 0:
        return 0
    if count > COUNT_MAX:
        return COUNT_MAX
    return count


def count_to_width_mm(count, *, stroke_mm=STROKE_MM):
    """
    Count to finger opening in mm: ``stroke * (255 - count) / 255``.

    S1 section 4.3: ``0x00`` is fully open, ``0xFF`` fully closed.
    """
    count = _reject_non_finite('count', count)
    return stroke_mm * (COUNT_MAX - count) / COUNT_MAX


def width_mm_to_count(width_mm, *, stroke_mm=STROKE_MM):
    """
    Finger opening in mm to count, clamped into 0..255.

    Half-up rounding, not Python's banker's rounding: ``round()`` would make
    this non-monotone at exact halves and break the count round trip.
    """
    width_mm = _reject_non_finite('width_mm', width_mm)
    return _clamp_count(COUNT_MAX * (stroke_mm - width_mm) / stroke_mm)


def speed_mm_s_to_count(speed_mm_s):
    """Finger speed in mm/s to rSP. INTERPOLATION, NOT DOCUMENTED."""
    speed_mm_s = _reject_non_finite('speed_mm_s', speed_mm_s)
    low, high = SPEED_RANGE_MM_S
    return _clamp_count(COUNT_MAX * (speed_mm_s - low) / (high - low))


def count_to_speed_mm_s(count):
    """Convert rSP to finger speed in mm/s. INTERPOLATION, NOT DOCUMENTED."""
    count = _reject_non_finite('count', count)
    low, high = SPEED_RANGE_MM_S
    return low + (high - low) * count / COUNT_MAX


def force_n_to_count(force_n):
    """Grip force in newtons to rFR. INTERPOLATION, NOT DOCUMENTED."""
    force_n = _reject_non_finite('force_n', force_n)
    low, high = FORCE_RANGE_N
    return _clamp_count(COUNT_MAX * (force_n - low) / (high - low))


def count_to_force_n(count):
    """Convert rFR to grip force in newtons. INTERPOLATION, NOT DOCUMENTED."""
    count = _reject_non_finite('count', count)
    low, high = FORCE_RANGE_N
    return low + (high - low) * count / COUNT_MAX


def current_ma(g_cu):
    """Convert gCU to motor current in mA. S1 4.4 calls this approximate."""
    g_cu = _reject_non_finite('g_cu', g_cu)
    return int(round(CURRENT_MA_PER_COUNT * g_cu))


def half_width_m_from_count(count, *, stroke_mm=STROKE_MM):
    """
    Count to ONE finger's opening in metres -- half the total width.

    Half-width throughout is this package's convention: goal, feedback and
    result all use it, which is a deliberate divergence from the Franka Hand
    driver's goal-versus-result asymmetry.
    """
    return count_to_width_mm(count, stroke_mm=stroke_mm) / 2000.0


def count_from_half_width_m(half_width_m, *, stroke_mm=STROKE_MM):
    """
    One finger's opening in metres to a count, clamped into 0..255.

    Clamps rather than raising on an out-of-range width: see the module
    docstring. Non-finite input raises ``ValueError``.
    """
    half_width_m = _reject_non_finite('half_width_m', half_width_m)
    return width_mm_to_count(half_width_m * 2000.0, stroke_mm=stroke_mm)


#: The largest half-width the 2F-85 can be commanded to, in metres: exactly
#: ``half_width_m_from_count(0)``. This is the ADMISSION BOUND the node gates a
#: goal against BEFORE converting it, which is why it is published rather than
#: left for a caller to compute -- a check on the converted count can never
#: fire, because this module clamps.
HALF_WIDTH_MAX_M = half_width_m_from_count(0)
