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

"""Round trips, endpoints, clamping and the half-width convention."""

import math

from franka_robotiq import units

import pytest


def test_count_zero_is_fully_open_and_255_is_fully_closed():
    """S1 section 4.3: 0x00 is fully open, 0xFF fully closed."""
    assert units.count_to_width_mm(0) == 85.0
    assert units.count_to_width_mm(units.COUNT_MAX) == 0.0


def test_every_count_round_trips_through_width():
    """All 256 counts survive count -> width -> count exactly."""
    for count in range(units.COUNT_MAX + 1):
        assert units.width_mm_to_count(units.count_to_width_mm(count)) == count


def test_one_count_is_a_third_of_a_millimetre_not_four_tenths():
    """
    85 mm over 255 counts is 0.333 mm, not the nominal 0.4 mm figure.

    Both the manual and the product sheet print 0.4 mm as the position
    resolution, but 255 counts at 0.4 mm would be 102 mm against an 85 mm
    stroke. 0.4 mm is a nominal fingertip figure, not the scale factor, and
    nobody should "correct" the map to it.
    """
    step = units.count_to_width_mm(0) - units.count_to_width_mm(1)
    assert step == pytest.approx(85.0 / 255.0)
    assert step == pytest.approx(0.3333, abs=1e-4)
    assert step != pytest.approx(0.4, abs=1e-3)


def test_width_epsilon_is_under_one_count():
    """The "already there" band is 0.6 of a count, so it cannot change rPR."""
    one_count_mm = units.STROKE_MM / units.COUNT_MAX
    assert units.WIDTH_EPSILON_MM < one_count_mm
    assert units.WIDTH_EPSILON_MM / one_count_mm == pytest.approx(0.6, abs=0.01)


def test_default_speed_maps_to_128():
    """85 mm/s, the configured default, is rSP 128."""
    assert units.speed_mm_s_to_count(85.0) == 128


def test_default_force_maps_to_64():
    """74 N, the configured default, is rFR 64: mid low-torque band."""
    assert units.force_n_to_count(74.0) == 64


def test_force_midpoint_lands_exactly_on_the_high_torque_boundary():
    """
    127.5 N maps to rFR 128, which is why 74 N is the default.

    S1 section 4.5.1 puts 1..127 in the low-torque, re-grasp-on band and
    128..255 in the high-torque one. The numeric midpoint of the force range
    sits exactly on that boundary, so it is the wrong default for a cell whose
    gripping payloads are unknown.
    """
    assert units.force_n_to_count(127.5) == 128
    assert units.force_n_to_count(74.0) < 128


def test_speed_and_force_round_trip_within_one_count():
    """The interpolations are stable in both directions."""
    for count in range(units.COUNT_MAX + 1):
        speed = units.count_to_speed_mm_s(count)
        assert abs(units.speed_mm_s_to_count(speed) - count) <= 1
        force = units.count_to_force_n(count)
        assert abs(units.force_n_to_count(force) - count) <= 1


def test_speed_and_force_endpoints_are_the_documented_ranges():
    """The endpoints are the manual's and the product sheet's, unaltered."""
    assert units.SPEED_RANGE_MM_S == (20.0, 150.0)
    assert units.FORCE_RANGE_N == (20.0, 235.0)
    assert units.count_to_speed_mm_s(0) == 20.0
    assert units.count_to_speed_mm_s(units.COUNT_MAX) == 150.0
    assert units.count_to_force_n(0) == 20.0
    assert units.count_to_force_n(units.COUNT_MAX) == 235.0


def test_values_outside_the_range_clamp():
    """
    Out-of-range values clamp into 0..255 and never raise.

    This is load-bearing for another part. The node refuses an out-of-stroke
    goal on the metres it was handed, BEFORE converting, precisely because
    this function clamps: a range check on the converted count could never
    fire, so it would be a rejection path that does not reject. A builder who
    changes this to raise breaks that admission check silently.
    """
    assert units.width_mm_to_count(-10.0) == units.COUNT_MAX
    assert units.width_mm_to_count(1000.0) == 0
    assert units.speed_mm_s_to_count(-5.0) == 0
    assert units.speed_mm_s_to_count(5000.0) == units.COUNT_MAX
    assert units.force_n_to_count(0.0) == 0
    assert units.force_n_to_count(5000.0) == units.COUNT_MAX
    assert units.count_from_half_width_m(0.05) == 0
    assert units.count_from_half_width_m(-1.0) == units.COUNT_MAX


@pytest.mark.parametrize('value', [float('nan'), float('inf'),
                                   float('-inf')])
def test_non_finite_input_raises(value):
    """A silently clamped NaN would be a full-open command from a typo."""
    for function in (units.width_mm_to_count, units.speed_mm_s_to_count,
                     units.force_n_to_count, units.count_from_half_width_m):
        with pytest.raises(ValueError):
            function(value)


def test_half_width_is_metres_and_open_is_0_0425():
    """Half-width in metres throughout: goal, feedback and result alike."""
    assert units.half_width_m_from_count(0) == 0.0425
    assert units.half_width_m_from_count(units.COUNT_MAX) == 0.0
    assert units.half_width_m_from_count(0) * 2000.0 == units.STROKE_MM


def test_half_width_max_m_is_the_open_half_width():
    """
    The admission bound is the function's own value, not a second literal.

    The node gates a goal against this constant, so it is pinned against
    ``half_width_m_from_count(0)`` rather than against a number alone: a
    stroke change must move both or fail here.
    """
    assert units.HALF_WIDTH_MAX_M == units.half_width_m_from_count(0)
    assert units.HALF_WIDTH_MAX_M == 0.0425


def test_half_width_round_trips_with_count():
    """All 256 counts survive count -> half-width -> count exactly."""
    for count in range(units.COUNT_MAX + 1):
        half = units.half_width_m_from_count(count)
        assert units.count_from_half_width_m(half) == count


def test_current_is_ten_times_the_count():
    """S1 section 4.4's own approximation, and it is labelled approximate."""
    assert units.current_ma(12) == 120
    assert units.current_ma(0) == 0
    assert units.current_ma(255) == 2550


def test_conversions_follow_the_stroke_rather_than_a_baked_in_constant():
    """The mapping is a function of the stroke, which the keyword proves."""
    assert units.count_to_width_mm(0, stroke_mm=140.0) == 140.0
    assert units.width_mm_to_count(70.0, stroke_mm=140.0) == 128
    assert units.half_width_m_from_count(0, stroke_mm=140.0) == 0.07


def test_module_docstring_labels_the_two_interpolations():
    """The honesty label is a deliverable, so it is tested like one."""
    text = ' '.join(units.__doc__.lower().split())
    assert 'interpolation' in text
    assert 'not manufacturer curves' in text
    assert 'linearity between them is assumed' in text
    assert 'do not present these as exact' in text


def test_rounding_is_half_up_not_bankers():
    """
    Half-up, not Python's round(), which would break monotonicity.

    A stroke of 255 mm makes one count exactly one millimetre, so the exact
    halves can be hit without floating-point noise: 254.5 mm is count 0.5 and
    253.5 mm is count 1.5. Banker's rounding would send both to an even
    count -- 0 and 2 -- which is not monotone in the width.
    """
    assert round(0.5) == 0 and round(1.5) == 2      # the behaviour avoided
    assert units.width_mm_to_count(254.5, stroke_mm=255.0) == 1
    assert units.width_mm_to_count(253.5, stroke_mm=255.0) == 2
    assert math.isclose(units.count_to_width_mm(128), 85.0 * 127 / 255)


def test_the_count_of_a_width_never_decreases_as_the_width_shrinks():
    """Monotonicity over the whole stroke, in 0.05 mm steps."""
    previous = -1
    for step in range(0, 1701):
        count = units.width_mm_to_count(85.0 - step * 0.05)
        assert count >= previous
        previous = count
