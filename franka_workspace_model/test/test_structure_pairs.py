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
Step 2b: the pedestal against each arm, the fifth checking step.

The two arms are bolted to one 0.1 m cube at the cell origin, and that cube is
the only structure either arm can hit that is not part of an arm.  The step is
eighteen volume pairs and it is live: an arm reaching across the table centre
and down goes through it.  Under the single profile the step does not exist, and
test_single_profile.py pins that; this file pins the dual profile, both the
shape of the pair list and one pose that actually lands in it.

The attribution rule is section 6.8's: `a` is the base_link volume and `b` is
the arm volume, always in that order, with `arm_id` naming the arm the volume
belongs to.  A checker that reported the pair the other way round would tell an
operator the pedestal moved.
"""

from conftest import READY

import numpy as np


# Derived independently of the checker: panda1's shoulder swung toward the
# table centre with the forearm folded down puts the wrist over the pedestal.
# The mirrored joint map M(q) = (-q1, q2, -q3, q4, -q5, q6, -q7) does the same
# thing on panda2, which is what makes the arm_id half of the assertion real.
OVER_THE_PEDESTAL = [-1.65, 1.25, 0.0, -1.65, 0.0, 1.5708, 0.7854]
MIRROR_JOINTS = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
PEDESTAL_VOLUME = 'base_link_v0'
SELF_COLLISION_MARGIN = 0.02


def _mirrored(values):
    return list(np.array(values) * MIRROR_JOINTS)


def test_the_pedestal_step_is_eighteen_pairs(cell_model):
    """Ten volumes an arm, less link0, which CONTRACT A disables against the base."""
    pairs = cell_model._structure_pairs
    assert len(pairs) == 18
    assert len({(first, second) for first, second, _ in pairs}) == 18


def test_every_pedestal_pair_names_the_pedestal_first(cell_model):
    """Section 6.8's order, on the pair list rather than on one contact."""
    for first, second, arm_id in cell_model._structure_pairs:
        assert first == PEDESTAL_VOLUME
        assert second.startswith(arm_id + '_')
        assert arm_id in cell_model.arm_ids()


def test_the_pedestal_step_omits_the_two_disabled_mounting_links(cell_model):
    """base_link against each link0 is disabled: they are bolted together."""
    second_volumes = {second for _, second, _ in cell_model._structure_pairs}
    for arm_id in cell_model.arm_ids():
        assert '{}_link0_v0'.format(arm_id) not in second_volumes
    for arm_id in cell_model.arm_ids():
        assert '{}_link6_v0'.format(arm_id) in second_volumes


def test_the_pedestal_step_covers_both_arms_equally(cell_model):
    counts = {}
    for _, _, arm_id in cell_model._structure_pairs:
        counts[arm_id] = counts.get(arm_id, 0) + 1
    assert counts == {arm_id: 9 for arm_id in cell_model.arm_ids()}


def test_an_arm_over_the_pedestal_produces_a_self_contact_against_it(cell_model):
    """The positive case: the step is not merely present, it fires."""
    result = cell_model.check_configuration(
        {'panda1': list(OVER_THE_PEDESTAL), 'panda2': list(READY)})
    assert not result.ok
    pedestal = [contact for contact in result.contacts
                if contact.a == PEDESTAL_VOLUME]
    assert pedestal, [(c.kind, c.a, c.b) for c in result.contacts]
    witness = min(pedestal, key=lambda contact: contact.distance)
    assert witness.kind == 'self'
    assert witness.b == 'panda1_link6_v0'
    assert witness.arm_id == 'panda1'
    assert witness.required == SELF_COLLISION_MARGIN
    assert witness.distance < -0.1


def test_the_same_pose_on_the_other_arm_is_attributed_to_the_other_arm(cell_model):
    """The arm_id half: a mirrored pose must name panda2, not panda1."""
    result = cell_model.check_configuration(
        {'panda1': list(READY), 'panda2': _mirrored(OVER_THE_PEDESTAL)})
    pedestal = [contact for contact in result.contacts
                if contact.a == PEDESTAL_VOLUME]
    assert pedestal
    for contact in pedestal:
        assert contact.kind == 'self'
        assert contact.arm_id == 'panda2'
        assert contact.b.startswith('panda2_')


def test_the_pedestal_is_never_reported_as_the_second_of_the_pair(cell_model):
    """Swapping a and b would tell the operator the pedestal moved."""
    for configuration in (
            {'panda1': list(OVER_THE_PEDESTAL), 'panda2': list(READY)},
            {'panda1': list(READY), 'panda2': _mirrored(OVER_THE_PEDESTAL)},
            {'panda1': list(OVER_THE_PEDESTAL),
             'panda2': _mirrored(OVER_THE_PEDESTAL)}):
        contacts = cell_model.check_configuration(configuration).contacts
        assert any(contact.a == PEDESTAL_VOLUME for contact in contacts)
        for contact in contacts:
            assert contact.b != PEDESTAL_VOLUME
