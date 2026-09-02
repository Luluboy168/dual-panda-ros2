# [DURABLE] URDF export contract tests move with the asset pipeline.
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

"""Tests for deterministic, fake-only dual-Panda URDF export."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from franka_ghost.asset_prep import (
    MAX_ASSET_BYTES,
    prepare_assets,
    VENDOR_SHA256,
)
from franka_ghost.urdf_export import (
    collect_mesh_references,
    expand_dual_urdf,
)
import pytest


LIMITS = (
    (-2.8973, 2.8973, 2.1750, 87.0),
    (-1.7628, 1.7628, 2.1750, 87.0),
    (-2.8973, 2.8973, 2.1750, 87.0),
    (-3.0718, -0.0698, 2.1750, 87.0),
    (-2.8973, 2.8973, 2.6100, 12.0),
    (-0.0175, 3.7525, 2.6100, 12.0),
    (-2.8973, 2.8973, 2.6100, 12.0),
)


@pytest.fixture(scope='module')
def urdf_text() -> str:
    """Expand the model once for this module."""
    return expand_dual_urdf()


def test_expanded_joint_names_and_limits(urdf_text: str) -> None:
    """The model has exactly the canonical 14 revolute joints and limits."""
    root = ET.fromstring(urdf_text)
    revolute = {
        joint.attrib['name']: joint
        for joint in root.findall('joint')
        if joint.attrib.get('type') == 'revolute'
    }
    expected_names = {
        f'panda{arm}_joint{joint}'
        for arm in (1, 2)
        for joint in range(1, 8)
    }
    assert set(revolute) == expected_names
    assert len(revolute) == 14

    for arm in (1, 2):
        for joint_number, expected in enumerate(LIMITS, start=1):
            limit = revolute[f'panda{arm}_joint{joint_number}'].find('limit')
            assert limit is not None
            actual = tuple(
                float(limit.attrib[name])
                for name in ('lower', 'upper', 'velocity', 'effort')
            )
            assert actual == pytest.approx(expected, abs=1e-9)


def test_tree_root_and_arm_offsets(urdf_text: str) -> None:
    """The dual model has one root and the two specified base offsets."""
    root = ET.fromstring(urdf_text)
    links = {link.attrib['name'] for link in root.findall('link')}
    child_links = {
        child.attrib['link']
        for child in root.findall('joint/child')
    }
    assert links - child_links == {'base_link'}

    expected_y = {'panda1_joint_base_link': 0.5, 'panda2_joint_base_link': -0.5}
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    for name, expected in expected_y.items():
        origin = joints[name].find('origin')
        assert origin is not None
        xyz = tuple(float(value) for value in origin.attrib['xyz'].split())
        assert math.isclose(xyz[1], expected, rel_tol=0.0, abs_tol=1e-12)
        assert xyz[0] == pytest.approx(0.0, abs=1e-12)
        assert xyz[2] == pytest.approx(0.0, abs=1e-12)


def test_export_is_fake_only_and_references_eight_arm_meshes(urdf_text: str) -> None:
    """No real address leaks and only link0 through link7 are visual assets."""
    assert len(urdf_text.encode('utf-8')) == 24459
    assert '/home/' not in urdf_text
    assert '172.16.' not in urdf_text
    expected = tuple(
        f'package://franka_description/meshes/visual/link{index}.dae'
        for index in range(8)
    )
    assert collect_mesh_references(urdf_text) == expected


def test_asset_prep_generates_then_skips_unchanged_inputs(tmp_path) -> None:
    """Prep writes the complete ignored asset tree and then takes its hash fast path."""
    first = prepare_assets(tmp_path)
    second = prepare_assets(tmp_path)
    assert first.regenerated is True
    assert second.regenerated is False
    assert first.mesh_count == second.mesh_count == 8
    assert first.asset_bytes == second.asset_bytes <= MAX_ASSET_BYTES

    assets = tmp_path / 'web/assets'
    assert (assets / 'model.urdf').is_file()
    assert (assets / 'manifest.json').is_file()
    assert len(tuple((assets / 'meshes').glob('*.ghostmesh.json'))) == 8
    assert len(tuple((assets / 'meshes').glob('*.ghostmesh.bin'))) == 8
    vendor = tmp_path / 'web/vendor'
    assert VENDOR_SHA256 in (vendor / 'VENDOR.md').read_text(encoding='utf-8')
    assert (vendor / 'three.min.js').is_file()
