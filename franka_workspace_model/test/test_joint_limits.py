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

"""T2: q = 0 is rejected, and exactly one joint per arm is the reason."""

from conftest import READY

from franka_workspace_model.model import WorkspaceModelError

import pytest


ZERO = [0.0] * 7
# Joint 4's range is [-3.0718, -0.0698] and joint 6's is [-0.0175, 3.7525].
JOINT_FOUR_EXCESS = 0.0698
JOINT_SIX_SLACK = 0.0175


def test_zero_configuration_violates_exactly_joint_four(cell_model):
    result = cell_model.check_configuration({'panda1': ZERO, 'panda2': ZERO})
    assert not result.ok
    violating = {contact.a for contact in result.contacts
                 if contact.kind == 'joint_limit'}
    assert violating == {'panda1_joint4', 'panda2_joint4'}


def test_joint_six_has_only_a_hair_of_slack_at_zero_and_is_not_flagged(cell_model):
    lower, _ = cell_model._joint_limits
    assert abs(float(lower[5]) + JOINT_SIX_SLACK) < 1e-12
    result = cell_model.check_configuration({'panda1': ZERO, 'panda2': ZERO})
    assert not any(contact.a.endswith('joint6') for contact in result.contacts)


def test_the_reported_excess_is_radians_past_the_limit(cell_model):
    result = cell_model.check_configuration({'panda1': ZERO, 'panda2': ZERO})
    for contact in result.contacts:
        if contact.kind != 'joint_limit':
            continue
        assert contact.required == 0.0
        assert contact.b == ''
        assert abs(contact.distance + JOINT_FOUR_EXCESS) < 1e-12
    assert abs(result.min_clearance + JOINT_FOUR_EXCESS) < 1e-12


def test_a_value_exactly_on_the_bound_is_not_a_violation(cell_model):
    lower, upper = cell_model._joint_limits
    on_bound = [float(upper[index]) if index == 3 else 0.0 for index in range(7)]
    on_bound[5] = float(lower[5])
    result = cell_model.check_configuration({'panda1': on_bound, 'panda2': READY})
    assert not any(contact.kind == 'joint_limit' and contact.arm_id == 'panda1'
                   for contact in result.contacts)


def test_a_hair_outside_the_bound_is_a_violation(cell_model):
    _, upper = cell_model._joint_limits
    outside = [0.0] * 7
    outside[3] = float(upper[3]) + 1e-9
    result = cell_model.check_configuration({'panda1': outside, 'panda2': READY})
    assert any(contact.a == 'panda1_joint4' for contact in result.contacts)


@pytest.mark.parametrize('bad', [
    {'panda1': READY},
    {'panda1': READY, 'panda2': READY, 'panda3': READY},
    {'panda1': READY, 'panda2': list(READY)[:6]},
    {'panda1': READY, 'panda2': [0.0, 0.0, 0.0, float('nan'), 0.0, 1.0, 0.0]},
    {'panda1': READY, 'panda2': [0.0, 0.0, 0.0, float('inf'), 0.0, 1.0, 0.0]},
    {'panda1': READY, 'panda2': 'not a joint vector'},
    {'panda1': READY, 'panda2': [True, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]},
])
def test_a_malformed_request_is_refused_not_answered(cell_model, bad):
    with pytest.raises(WorkspaceModelError):
        cell_model.check_configuration(bad)
