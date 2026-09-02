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

"""T6 and T8: the margin arithmetic, on both sides of the boundary."""

import math

from conftest import load_mutated, READY

import numpy as np


# At the ready pose two clearances are exact consequences of the description,
# with no forward kinematics beyond the first joint:
#   link2_v0 lies at cell z = 0.333 with r = 0.09, so its floor clearance is
#     0.333 - 0.09 = 0.243000 m;
#   link2_v0 against link2_v0 across the arms is 2*(0.50 - 0.06) - 0.18
#     = 0.700000 m, a structural tie with link4_v0.
# Both radii are 0.06 + safety_distance, so a checker that applied the built-in
# 30 mm a second time would read 0.213 and 0.580 and fail both sides below.
FLOOR_CLEARANCE = 0.333 - 0.09
CROSS_ARM_CLEARANCE = 2.0 * (0.5 - 0.06) - 0.18
# At the same pose panda1's link2_v0 and link4_v0 reach y = 0.56 with r = 0.09
# and link6_v0 reaches y = 0.57 with r = 0.08, so three volumes tie exactly at
# 1.00 - 0.56 - 0.09 = 0.350000 against the y_max face.
Y_MAX_CLEARANCE = 1.0 - 0.56 - 0.09
MARGIN = 0.03
SWEPT_PATH_EXTRA = 0.01
ONE_MILLIMETRE = 0.001
BOTH_READY = {'panda1': list(READY), 'panda2': list(READY)}


def _with_margin(tmp_path, key, value):
    def mutate(document):
        document['margins'][key] = value
    return load_mutated(tmp_path, mutate)


def _with_y_max(tmp_path, value):
    def mutate(document):
        document['allowed_volume']['y_max'] = value
    return load_mutated(tmp_path, mutate)


def test_containment_margin_one_millimetre_inside_the_boundary_passes(tmp_path):
    """The y_max face, moved until panda1 is a millimetre outside its margin."""
    model = _with_y_max(tmp_path, 1.0 - (Y_MAX_CLEARANCE - MARGIN - ONE_MILLIMETRE))
    result = model.check_configuration(BOTH_READY)
    assert result.ok


def test_containment_margin_one_millimetre_outside_the_boundary_fails(tmp_path):
    model = _with_y_max(tmp_path, 1.0 - (Y_MAX_CLEARANCE - MARGIN + ONE_MILLIMETRE))
    result = model.check_configuration(BOTH_READY)
    assert not result.ok
    contacts = [contact for contact in result.contacts
                if contact.kind == 'containment']
    assert {contact.b for contact in contacts} == {'work_area.y_max'}
    # The three-way tie the ready pose produces on that face.
    assert {contact.a for contact in contacts} == {
        'panda1_link2_v0', 'panda1_link4_v0', 'panda1_link6_v0'}


def test_the_jog_fence_applies_the_swept_extra_check_configuration_does_not(tmp_path):
    """
    Section 6.4: `check_jog` is swept too, so it is stricter than a static check.

    The y_max face is moved until panda1's ready pose clears it by 0.035 m,
    which is outside the 0.030 m containment margin and inside the swept
    0.040 m.  A `check_jog` that dropped `swept_path_extra` would loosen the one
    fence the jog console actually calls, and the existing swept assertion is on
    `check_path`, which that regression leaves untouched.
    """
    clearance = MARGIN + 0.005
    model = _with_y_max(tmp_path, 0.65 + clearance)
    assert model.check_configuration(BOTH_READY).ok
    result = model.check_jog('panda1', BOTH_READY, 0, 0.0)
    assert not result.allowed
    assert not result.clamped
    assert result.limiting.kind == 'containment'
    assert result.limiting.b == 'work_area.y_max'
    assert abs(result.limiting.required - (MARGIN + SWEPT_PATH_EXTRA)) < 1e-12
    assert abs(result.limiting.distance - clearance) < 1e-9


def _with_wide_box_and_margin(tmp_path, value):
    def mutate(document):
        document['allowed_volume'].update({'x_min': -5.0, 'x_max': 5.0,
                                           'y_min': -5.0, 'y_max': 5.0})
        document['margins']['environment'] = value
    return load_mutated(tmp_path, mutate)


def test_the_table_top_binds_at_exactly_the_derived_floor_clearance(tmp_path):
    """
    The floor, with the lateral faces moved out of the way.

    link2_v0's capsule lies at cell z = 0.333 with r = 0.09 = 0.06 + the built-in
    30 mm inflation, so its floor clearance is 0.243000 m exactly.  A checker
    that applied the inflation a second time would read 0.213 and fail the first
    half of this test.
    """
    passing = _with_wide_box_and_margin(tmp_path / 'inside',
                                        FLOOR_CLEARANCE - ONE_MILLIMETRE)
    assert passing.check_configuration(BOTH_READY).ok
    failing = _with_wide_box_and_margin(tmp_path / 'outside',
                                        FLOOR_CLEARANCE + ONE_MILLIMETRE)
    result = failing.check_configuration(BOTH_READY)
    assert not result.ok
    contacts = [contact for contact in result.contacts
                if contact.kind == 'containment']
    assert {contact.b for contact in contacts} == {'work_area.z_min'}
    assert {contact.a for contact in contacts} == {'panda1_link2_v0',
                                                   'panda2_link2_v0'}


def test_cross_arm_margin_one_millimetre_inside_the_boundary_passes(tmp_path):
    model = _with_margin(tmp_path, 'cross_arm', CROSS_ARM_CLEARANCE - ONE_MILLIMETRE)
    assert model.check_configuration(BOTH_READY).ok


def test_cross_arm_margin_one_millimetre_outside_the_boundary_fails(tmp_path):
    model = _with_margin(tmp_path, 'cross_arm', CROSS_ARM_CLEARANCE + ONE_MILLIMETRE)
    result = model.check_configuration(BOTH_READY)
    assert not result.ok
    pairs = {(contact.a, contact.b) for contact in result.contacts
             if contact.kind == 'cross_arm'}
    assert pairs == {('panda1_link2_v0', 'panda2_link2_v0'),
                     ('panda1_link4_v0', 'panda2_link4_v0')}


def test_a_pair_exactly_on_its_margin_passes(tmp_path):
    """Equality is not a violation: the comparison is strict."""
    model = _with_margin(tmp_path, 'cross_arm', CROSS_ARM_CLEARANCE)
    result = model.check_configuration(BOTH_READY)
    assert result.ok
    assert abs(result.min_clearance) < 1e-12


def _random_configurations(count, seed=20260901):
    generator = np.random.default_rng(seed)
    lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    for _ in range(count):
        values = generator.uniform(lower, upper, size=(2, 7))
        yield {'panda1': list(values[0]), 'panda2': list(values[1])}


def test_increasing_a_margin_never_turns_a_refusal_into_an_approval(tmp_path):
    """T8: the margin plumbing has no sign errors and no per-kind mix-ups."""
    base = load_mutated(tmp_path / 'base')

    def widen(document):
        for key in ('self_collision', 'cross_arm', 'environment', 'keep_out'):
            document['margins'][key] = document['margins'][key] + 0.01
    wider = load_mutated(tmp_path / 'wide', widen)
    for configuration in _random_configurations(1000):
        first = base.check_configuration(configuration)
        second = wider.check_configuration(configuration)
        assert not (second.ok and not first.ok)
        if math.isfinite(first.min_clearance) and math.isfinite(second.min_clearance):
            assert second.min_clearance <= first.min_clearance + 1e-12
