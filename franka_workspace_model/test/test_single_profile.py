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
The single profile is arm-agnostic: a panda2-only session is checked correctly.

The shipped cell file describes the dual cell, and its `cell_frame.anchors.single`
is the one consistent with a panda1-only bringup.  Nothing in the loader is
panda1-only, though: rule A.3.9 derives the expected anchor from whichever arm
is declared.  This file proves that by building a panda2-only model from the
single-arm description and checking it in the cell frame.
"""

import copy
import hashlib

from conftest import (CELL_MODEL_PATH, make_scratch_repository, READY,
                      REPOSITORY_ROOT)

from franka_workspace_model.generate_link_geometry import generate
from franka_workspace_model.model import CellModel, WorkspaceModelError

import pytest

import yaml


SINGLE_DESCRIPTION = 'franka_description/robots/real/panda_arm.urdf.xacro'
SINGLE_SRDF = 'franka_moveit_config/srdf/panda_arm.srdf.xacro'


def _single_arm_model(tmp_path, arm_id, mutate=None):
    root = make_scratch_repository(tmp_path)
    arguments = [('arm_id', arm_id), ('hand', 'false'), ('robot_ip', ''),
                 ('use_fake_hardware', 'true'), ('fake_sensor_commands', 'false')]
    geometry_text = generate(REPOSITORY_ROOT, SINGLE_DESCRIPTION, arguments)
    geometry_path = root / 'cell' / 'link_geometry_single.yaml'
    geometry_path.write_text(geometry_text, encoding='utf-8')

    document = copy.deepcopy(
        yaml.safe_load(CELL_MODEL_PATH.read_text(encoding='utf-8')))
    keep = 0 if arm_id == 'panda1' else 1
    arm = copy.deepcopy(document['arms'][keep])
    document['arms'] = [arm]
    document['cell_frame']['anchors']['single']['xyz'] = [
        -value for value in arm['urdf_base_pose']['xyz']]
    document['policy']['cross_arm']['enabled'] = False
    document['policy']['self_collision']['extra_disabled_pairs'] = []
    document['policy']['self_collision']['extra_enabled_pairs'] = [
        entry for entry in document['policy']['self_collision']['extra_enabled_pairs']
        if entry['a'].startswith(arm_id)]
    # The ruled wrist margins are per-arm too: the single-arm description
    # declares one arm's links, so the other arm's entries name links that do
    # not exist and the loader refuses them by name.
    document['policy']['self_collision']['pair_margins'] = [
        entry for entry in document['policy']['self_collision']['pair_margins']
        if entry['a'].startswith(arm_id)]
    document['sources']['srdf_xacro'] = SINGLE_SRDF
    document['sources']['srdf_xacro_sha256'] = hashlib.sha256(
        (REPOSITORY_ROOT / SINGLE_SRDF).read_bytes()).hexdigest()
    document['sources']['urdf_xacro'] = SINGLE_DESCRIPTION
    document['sources']['urdf_xacro_sha256'] = hashlib.sha256(
        (REPOSITORY_ROOT / SINGLE_DESCRIPTION).read_bytes()).hexdigest()
    document['sources']['link_geometry'] = 'link_geometry_single.yaml'
    document['sources']['link_geometry_sha256'] = hashlib.sha256(
        geometry_text.encode('utf-8')).hexdigest()
    if mutate is not None:
        mutate(document)
    target = root / 'cell' / 'cell_model_single.yaml'
    target.write_text(yaml.safe_dump(document, default_flow_style=False,
                                     sort_keys=False), encoding='utf-8')
    return CellModel.load(target, profile='single')


@pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
def test_a_single_arm_model_loads_for_either_arm(tmp_path, arm_id):
    model = _single_arm_model(tmp_path / arm_id, arm_id)
    assert model.arm_ids() == (arm_id,)
    assert model.allowed_volume().id == 'work_area'


@pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
def test_the_arm_is_checked_in_the_cell_frame_not_its_own(tmp_path, arm_id):
    """The whole point of R2: a panda2-only session must not be checked at y = +0.5."""
    model = _single_arm_model(tmp_path / arm_id, arm_id)
    sample = model._sample({arm_id: list(READY)})
    ends_a, _, _ = model._place(sample)
    index = model._volume_position['{}_link2_v0'.format(arm_id)]
    expected = 0.5 if arm_id == 'panda1' else -0.5
    assert abs(float(ends_a[index][1]) - (expected + 0.06)) < 1e-9


@pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
def test_the_ready_pose_is_clear_under_the_single_profile(tmp_path, arm_id):
    model = _single_arm_model(tmp_path / arm_id, arm_id)
    result = model.check_configuration({arm_id: list(READY)})
    assert result.ok, [(c.kind, c.a, c.b, c.distance) for c in result.contacts]


@pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
def test_the_single_profile_has_no_pedestal_step(tmp_path, arm_id):
    """The single description has no base_link at all, so step 2b is empty."""
    model = _single_arm_model(tmp_path / arm_id, arm_id)
    assert model._structure_pairs == ()
    assert not model._cross_pairs
    # Twenty-one: nineteen the single SRDF leaves enabled, plus the two pairs
    # the cell file's extra_enabled_pairs delta pins - link4/link8, which the
    # single SRDF disables and the dual macro does not, and link2/link6, whose
    # SRDF reason="Never" is measured false (test_falsified_srdf_pair.py).
    assert len(model._intra_pairs) == 21


@pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
def test_the_gap_two_delta_is_load_bearing_under_the_single_profile(tmp_path, arm_id):
    """The single SRDF disables link4/link8; the delta pins it enabled anyway."""
    model = _single_arm_model(tmp_path / arm_id, arm_id)
    pairs = {(first, second) for first, second, _ in model._intra_pairs}
    assert ('{}_link4_v0'.format(arm_id), '{}_link8_v0'.format(arm_id)) in pairs

    def drop_the_delta(document):
        document['policy']['self_collision']['extra_enabled_pairs'] = []
    without = _single_arm_model(tmp_path / (arm_id + '-without'), arm_id,
                                drop_the_delta)
    pairs = {(first, second) for first, second, _ in without._intra_pairs}
    assert ('{}_link4_v0'.format(arm_id), '{}_link8_v0'.format(arm_id)) not in pairs
    assert ('{}_link2_v0'.format(arm_id), '{}_link6_v0'.format(arm_id)) not in pairs
    assert len(without._intra_pairs) == 19


def test_a_dual_link_geometry_is_refused_under_the_single_profile(tmp_path):
    """The single anchor is expressed in the arm's own link0 frame."""
    def keep_the_dual_geometry(document):
        document['sources']['link_geometry'] = 'link_geometry_v1.yaml'
        document['sources']['link_geometry_sha256'] = hashlib.sha256(
            (REPOSITORY_ROOT / 'franka_workspace_model' / 'cell'
             / 'link_geometry_v1.yaml').read_bytes()).hexdigest()
        document['sources']['urdf_xacro'] = (
            'franka_description/robots/real/dual_panda_arm.urdf.xacro')
        document['sources']['urdf_xacro_sha256'] = hashlib.sha256(
            (REPOSITORY_ROOT
             / 'franka_description/robots/real/dual_panda_arm.urdf.xacro').read_bytes()
        ).hexdigest()
    with pytest.raises(WorkspaceModelError, match='rooted at'):
        _single_arm_model(tmp_path, 'panda1', keep_the_dual_geometry)


def test_the_dual_file_is_refused_under_the_single_profile():
    with pytest.raises(WorkspaceModelError, match='exactly one arm'):
        CellModel.load(CELL_MODEL_PATH, profile='single')
