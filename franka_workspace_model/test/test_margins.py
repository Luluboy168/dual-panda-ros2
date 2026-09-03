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


# THESE CONSTANTS ARE MEASUREMENTS NOW, NOT ARITHMETIC ON RADII, and that is
# the whole difference the mesh fence makes.  They used to be derivable from
# the description with no forward kinematics beyond the first joint - link2_v0
# at cell z = 0.333 with r = 0.09 gives a floor clearance of 0.243000 m
# exactly - because a capsule's extent IS its axis plus its radius.  A body
# has no radius; its extent is where the casting actually reaches, so each
# number below is the measured extreme of the placed vertices of the body that
# binds, at the ready pose, in the cell frame.
#
# The old numbers are kept in this comment because their DIFFERENCE is the
# point.  The floor clearance moves from 0.243000 m to 0.277988 m: link2's
# casting sits 35 mm higher above the table than its padded capsule claimed.
# The cross-arm clearance moves from 0.700000 m to 0.741123 m for the same
# reason, and the pair that binds changes with it - link1 against link2 rather
# than link2 against link2, because the capsule tie was an artefact of equal
# radii and the castings are not equal.
#
# Each is the exact float the fence reports, quoted to full precision, so that
# the "exactly on the margin" test below is exactly on it.
FLOOR_CLEARANCE = 0.2779880781978916          # panda1_link2_st / panda2_link2_st
CROSS_ARM_CLEARANCE = 0.7411234109851764      # panda1_link1_st / panda2_link2_st
# On the y_max face the three-way capsule tie is also gone: link5's third
# collision piece binds alone, at a max y of 0.6299349000000828.
Y_MAX_CLEARANCE = 0.3700650999999172          # panda1_link5_collision_2_st
Y_MAX_BINDING_BODY = 'panda1_link5_collision_2_st'
Y_MAX_BINDING_REACH = 0.6299349000000828
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
    # The body that binds names itself.  There is no longer a three-way tie:
    # that was a property of three capsules with the same radius, not of three
    # castings.
    assert Y_MAX_BINDING_BODY in {contact.a for contact in contacts}


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
    model = _with_y_max(tmp_path, Y_MAX_BINDING_REACH + clearance)
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

    link2's CASTING reaches down to cell z = 0.277988 m at the ready pose, on
    both arms, which is 35 mm higher than the 0.243000 m its padded capsule
    claimed - the capsule carried 30 mm of inflation plus a radius that bounds
    the whole link rather than the part nearest the floor.  A checker that
    applied the inflation a second time, or that measured the capsule instead
    of the body, fails one half of this test or the other.
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
    # A genuine two-way tie, and this one survives: the two arms are mirror
    # images about the cell origin, so their link2 castings reach the same
    # height.
    assert {contact.a for contact in contacts} == {'panda1_link2_st',
                                                   'panda2_link2_st'}


def test_cross_arm_margin_one_millimetre_inside_the_boundary_passes(tmp_path):
    model = _with_margin(tmp_path, 'cross_arm', CROSS_ARM_CLEARANCE - ONE_MILLIMETRE)
    assert model.check_configuration(BOTH_READY).ok


def test_cross_arm_margin_one_millimetre_outside_the_boundary_fails(tmp_path):
    model = _with_margin(tmp_path, 'cross_arm', CROSS_ARM_CLEARANCE + ONE_MILLIMETRE)
    result = model.check_configuration(BOTH_READY)
    assert not result.ok
    pairs = {(contact.a, contact.b) for contact in result.contacts
             if contact.kind == 'cross_arm'}
    assert ('panda1_link1_st', 'panda2_link2_st') in pairs


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
        # The half that is about SAFETY, and it is unconditional.
        assert not (second.ok and not first.ok)
        # The half that is about the reported number, and it is conditional -
        # deliberately, and here is why.  On a PASSING configuration the
        # reported minimum includes each CULLED pair's certified lower bound,
        # which is conservative; widening a margin evaluates more pairs, and a
        # pair that flips from culled to evaluated stops contributing its bound
        # and starts contributing its exact distance, which is larger.  So the
        # reported minimum can rise when a margin widens, and that is an
        # artefact of the conservatism changing rather than a sign error.
        # Whenever either model actually reports a contact the minimum is
        # EXACT - the violating pair is never culled - and monotonicity holds
        # again.  See doc/CONTRACT.md on min_clearance.
        if first.ok and second.ok:
            continue
        if math.isfinite(first.min_clearance) and math.isfinite(second.min_clearance):
            assert second.min_clearance <= first.min_clearance + 1e-12
