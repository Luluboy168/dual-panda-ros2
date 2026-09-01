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
T7: the mirror invariant, stated as the invariant that actually holds.

The two arms are mounted at y = +-0.50 with rpy = 0 0 0, so the cell is
geometrically symmetric under the reflection y -> -y combined with the joint map
M(q) = (-q1, q2, -q3, q4, -q5, q6, -q7).  Under that map every LINK ORIGIN of
panda2 is the exact reflection of panda1's, to machine precision, and that is
the structural invariant which catches transform bugs no hand-built case would.

It does NOT extend to every collision volume, and the difference is not a
tolerance question.  Three of the ten volumes per arm are chiral in their own
link frame - link5_v1 sits 0.08 m off the link axis, link8_v0 sits at
(0.0424, 0.0424), and link6_v0's capsule is not symmetric about its link origin
(it runs from +0.01 to -0.07) - so a reflected configuration is not a reflected
collision model, and mirrored contact sets and equal min_clearance values are
NOT available.  This test pins both halves: the seven volumes that do mirror,
and the three that provably do not.
"""

from conftest import READY

import numpy as np

import pytest


MIRROR_JOINTS = np.array([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
REFLECTION = np.diag([1.0, -1.0, 1.0])
MIRRORING_VOLUMES = ('link0_v0', 'link1_v0', 'link2_v0', 'link3_v0', 'link4_v0',
                     'link5_v0', 'link7_v0')
CHIRAL_VOLUMES = ('link5_v1', 'link6_v0', 'link8_v0')


def _configurations(count=25, seed=20260902):
    generator = np.random.default_rng(seed)
    lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    yield np.array(READY)
    for _ in range(count):
        yield generator.uniform(lower, upper)


def _placed(model, values):
    configuration = {'panda1': list(values),
                     'panda2': list(np.array(values) * MIRROR_JOINTS)}
    sample = model._sample(configuration)
    ends_a, ends_b, _ = model._place(sample)
    return {entry[0]: (ends_a[index], ends_b[index])
            for index, entry in enumerate(model._volume_list)}


def _unordered_error(first, second):
    straight = max(float(np.abs(first[0] - second[0]).max()),
                   float(np.abs(first[1] - second[1]).max()))
    swapped = max(float(np.abs(first[0] - second[1]).max()),
                  float(np.abs(first[1] - second[0]).max()))
    return min(straight, swapped)


@pytest.mark.parametrize('name', MIRRORING_VOLUMES)
def test_axial_volumes_mirror_exactly(cell_model, name):
    for values in _configurations():
        placed = _placed(cell_model, values)
        first = tuple(REFLECTION @ point for point in placed['panda1_' + name])
        error = _unordered_error(first, placed['panda2_' + name])
        assert error < 1e-9, '{}: mirror error {}'.format(name, error)


@pytest.mark.parametrize('name', CHIRAL_VOLUMES)
def test_chiral_volumes_provably_do_not_mirror(cell_model, name):
    """The arm's collision geometry is chiral; this pins that, rather than hiding it."""
    worst = 0.0
    for values in _configurations():
        placed = _placed(cell_model, values)
        first = tuple(REFLECTION @ point for point in placed['panda1_' + name])
        worst = max(worst, _unordered_error(first, placed['panda2_' + name]))
    assert worst > 1e-3, (
        '{} now mirrors; either the description changed or the placement is '
        'wrong'.format(name))


def test_base_link_is_invariant_under_the_reflection(cell_model):
    for values in _configurations(count=5):
        placed = _placed(cell_model, values)
        first, second = placed['base_link_v0']
        assert float(np.abs(REFLECTION @ first - first).max()) < 1e-12
        assert float(np.abs(REFLECTION @ second - second).max()) < 1e-12


def test_a_mirror_symmetric_pose_gives_a_symmetric_cross_arm_witness(cell_model):
    """The deepest cross-arm pair at a mirror-symmetric pose is self-mirrored."""
    values = [-1.4, 0.7, 0.0, -1.6, 0.0, 1.5708, 0.7854]
    result = cell_model.check_configuration(
        {'panda1': values, 'panda2': list(np.array(values) * MIRROR_JOINTS)})
    deepest = result.contacts[0]
    assert deepest.kind == 'cross_arm'
    assert deepest.a.split('_', 1)[1] == deepest.b.split('_', 1)[1]
