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
T17: the containment exemption is derived, and its constants are the stated ones.

The exemption is the one rule in the checking policy that REMOVES checks.  A
rule that removes checks must be pinned exactly, or it will quietly grow.  The
tilted-axis case at the end is what proves the set is derived from the joint
chain and not a hand-written list wearing a derivation's clothes.
"""

import copy

from conftest import LINK_GEOMETRY_PATH, make_scratch_repository

from franka_workspace_model.model import CellModel

import pytest

import yaml


FULLY_STATIC = {'base_link_v0', 'panda1_link0_v0', 'panda2_link0_v0'}
Z_STATIC = {'panda1_link1_v0', 'panda2_link1_v0'}
# Constants of the robot description, independent of q:
#   the 0.1 m pedestal cube, evaluated as its bounding sphere of radius
#   0.05*sqrt(3), reaches 0.0866025 m below the table top;
#   link0's capsule sits at link-frame z = 0.06 with r = 0.09, and link0's frame
#   IS the mounting plane, so it reaches 0.03 m below;
#   link1's capsule ends exactly on the table top and its radius carries it
#   0.09 m below.
CONSTANTS = {'base_link_v0': 0.0866025, 'panda1_link0_v0': 0.03,
             'panda2_link0_v0': 0.03, 'panda1_link1_v0': 0.09,
             'panda2_link1_v0': 0.09}


def test_the_exemption_sets_are_exactly_the_stated_ones(cell_model):
    fully_static = {name for name, reason in cell_model._exempt_volumes
                    if reason == 'fully static'}
    z_static = {name for name, reason in cell_model._exempt_volumes
                if reason == 'z-static'}
    assert fully_static == FULLY_STATIC
    assert z_static == Z_STATIC


def test_no_exempt_volume_appears_in_a_per_query_step(cell_model):
    moving = {name for name, _ in cell_model._moving_volumes}
    assert not (FULLY_STATIC & moving)
    assert Z_STATIC <= moving


def test_a_z_static_volume_is_exempt_from_the_two_z_faces_only(cell_model):
    faces = {name: allowed for name, _, allowed in cell_model._containment_volumes}
    for name in Z_STATIC:
        assert set(faces[name]) == {'x_min', 'x_max', 'y_min', 'y_max'}
    assert set(faces['panda1_link2_v0']) == {'x_min', 'x_max', 'y_min', 'y_max',
                                             'z_min', 'z_max'}


def test_the_face_evaluation_count_is_the_cell_arithmetic(cell_model):
    """9 volumes x 4 lateral x 2 arms, plus 8 x z_min x 2, plus 8 x z_max x 2."""
    total = sum(len(faces) for _, _, faces in cell_model._containment_volumes)
    assert total == 9 * 4 * 2 + 8 * 2 + 8 * 2
    assert total == 104


def test_the_reported_constants_are_the_derived_ones(cell_model):
    reported = {}
    for line in cell_model.diagnostics():
        for name, expected in CONSTANTS.items():
            if line.startswith('volume {} '.format(name)):
                reported[name] = expected
                assert '{:.7f}'.format(-expected) in line, line
    assert set(reported) == set(CONSTANTS)


def test_the_diagnostic_says_whose_property_this_is(cell_model):
    lines = [line for line in cell_model.diagnostics() if 'link1_v0' in line]
    assert lines
    for line in lines:
        assert 'not of your measurements' in line


def _model_with_geometry(tmp_path, mutate_geometry):
    root = make_scratch_repository(tmp_path)
    geometry = copy.deepcopy(
        yaml.safe_load(LINK_GEOMETRY_PATH.read_text(encoding='utf-8')))
    mutate_geometry(geometry)
    text = yaml.safe_dump(geometry, default_flow_style=False, sort_keys=False)
    path = root / 'cell' / 'link_geometry_tilted.yaml'
    path.write_text(text, encoding='utf-8')

    import hashlib
    from conftest import CELL_MODEL_PATH
    document = copy.deepcopy(
        yaml.safe_load(CELL_MODEL_PATH.read_text(encoding='utf-8')))
    document['sources']['link_geometry'] = 'link_geometry_tilted.yaml'
    document['sources']['link_geometry_sha256'] = hashlib.sha256(
        text.encode('utf-8')).hexdigest()
    target = root / 'cell' / 'cell_model_tilted.yaml'
    target.write_text(yaml.safe_dump(document, default_flow_style=False,
                                     sort_keys=False), encoding='utf-8')
    return CellModel.load(target, profile='dual')


def test_a_tilted_first_joint_removes_link1_from_the_z_static_set(tmp_path):
    """The set is computed from the chain, not from a list of names."""
    def tilt(geometry):
        for entry in geometry['links']:
            if entry['link'].endswith('_link1'):
                entry['parent_joint']['axis'] = [0.0, 0.1961161351, 0.9805806757]
    model = _model_with_geometry(tmp_path, tilt)
    z_static = {name for name, reason in model._exempt_volumes
                if reason == 'z-static'}
    assert z_static == set()
    faces = {name: allowed for name, _, allowed in model._containment_volumes}
    assert set(faces['panda1_link1_v0']) == {'x_min', 'x_max', 'y_min', 'y_max',
                                             'z_min', 'z_max'}


def test_a_revolute_mounting_joint_removes_link0_from_the_static_set(tmp_path):
    def unfix(geometry):
        for entry in geometry['links']:
            if entry['link'].endswith('_link0'):
                entry['parent_joint'].update({
                    'type': 'revolute', 'axis': [0.0, 0.0, 1.0],
                    'limit_lower': -3.0, 'limit_upper': 3.0})
    with pytest.raises(Exception) as raised:
        _model_with_geometry(tmp_path, unfix)
    # The mounting joint is not one of the arm's seven, so the checker has no
    # value to drive it with and refuses rather than inventing one.
    assert 'seven joints' in str(raised.value)


def test_without_the_exemption_every_pose_would_be_refused(cell_model):
    """The canary: the exempt volumes really do sit below the floor."""
    from conftest import READY
    sample = cell_model._sample({'panda1': list(READY), 'panda2': list(READY)})
    ends_a, ends_b, _ = cell_model._place(sample)
    for name, expected in CONSTANTS.items():
        if name == 'base_link_v0':
            continue
        index = cell_model._volume_position[name]
        lowest = min(float(ends_a[index][2]), float(ends_b[index][2]))
        lowest -= float(cell_model._radii[index])
        assert abs(lowest + expected) < 1e-9, name
