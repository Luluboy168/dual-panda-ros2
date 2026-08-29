# [DURABLE] Mesh conversion contract tests move with the asset pipeline.
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

"""Integrity and determinism tests for the COLLADA mesh converter."""

from __future__ import annotations

import json
import math
from pathlib import Path
import struct
import textwrap

from ament_index_python.packages import get_package_share_directory
from franka_ghost.mesh_convert import convert_mesh
import pytest


TRIANGLE_COUNTS = {
    'link0': 20483,
    'link1': 12516,
    'link2': 12716,
    'link3': 14233,
    'link4': 14621,
    'link5': 18327,
    'link6': 21620,
    'link7': 12082,
}


@pytest.fixture(scope='module')
def converted_meshes(tmp_path_factory: pytest.TempPathFactory) -> list[tuple[Path, Path]]:
    """Convert every arm mesh twice into isolated output directories."""
    share = Path(get_package_share_directory('franka_description'))
    mesh_root = share / 'meshes/visual'
    first = tmp_path_factory.mktemp('ghostmesh-first')
    second = tmp_path_factory.mktemp('ghostmesh-second')
    results = []
    for stem in TRIANGLE_COUNTS:
        source = mesh_root / f'{stem}.dae'
        label = f'franka_description/meshes/visual/{stem}.dae'
        one = convert_mesh(source, first, label)
        two = convert_mesh(source, second, label)
        assert one.metadata_path.read_bytes() == two.metadata_path.read_bytes()
        assert one.binary_path.read_bytes() == two.binary_path.read_bytes()
        results.append((one.metadata_path, one.binary_path))
    return results


def _attribute_values(binary: bytes, layout: dict) -> tuple[float, ...]:
    count = layout['count'] * layout['items']
    return struct.unpack_from(f'<{count}f', binary, layout['byteOffset'])


def test_mesh_counts_layout_and_determinism(
    converted_meshes: list[tuple[Path, Path]],
) -> None:
    """Every mesh has the expected count and a complete binary layout."""
    for metadata_path, binary_path in converted_meshes:
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        stem = metadata_path.name.split('.', 1)[0]
        assert metadata['schema'] == 'franka.ghost.mesh/1'
        assert metadata['counts']['triangles'] == TRIANGLE_COUNTS[stem]
        assert metadata['counts']['vertices'] == TRIANGLE_COUNTS[stem] * 3
        assert metadata['up_axis'] == 'Z_UP'
        assert metadata['unit_m'] == 1.0

        binary = binary_path.read_bytes()
        layout_end = max(
            item['byteOffset']
            + item['count'] * item['items'] * struct.calcsize('<f')
            for item in metadata['layout']
        )
        assert len(binary) == layout_end
        assert metadata['bin'] == binary_path.name


def test_mesh_vertices_normals_bounds_and_groups(
    converted_meshes: list[tuple[Path, Path]],
) -> None:
    """Geometry is finite, normalised, plausibly sized, and fully grouped."""
    for metadata_path, binary_path in converted_meshes:
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        binary = binary_path.read_bytes()
        layout = {item['name']: item for item in metadata['layout']}
        positions = _attribute_values(binary, layout['position'])
        normals = _attribute_values(binary, layout['normal'])
        assert all(math.isfinite(value) for value in positions)
        assert all(math.isfinite(value) for value in normals)

        lengths = (
            math.sqrt(sum(normals[index + axis] ** 2 for axis in range(3)))
            for index in range(0, len(normals), 3)
        )
        assert all(abs(length - 1.0) <= 1e-4 for length in lengths)

        dimensions = []
        for axis in range(3):
            values = positions[axis::3]
            dimensions.append(max(values) - min(values))
        assert all(0.001 < dimension < 1.2 for dimension in dimensions)

        cursor = 0
        for group in metadata['groups']:
            assert group['start'] == cursor
            assert group['count'] > 0
            assert len(group['color']) == 3
            assert all(math.isfinite(value) for value in group['color'])
            assert math.isfinite(group['opacity'])
            cursor += group['count']
        assert cursor == metadata['counts']['vertices']


def test_nested_node_transform_inverse_transpose_normal_and_material(tmp_path) -> None:
    """A synthetic mesh makes the three easy-to-miss C5 rules observable."""
    source = tmp_path / 'probe.dae'
    source.write_text(
        textwrap.dedent(
            """\
            <?xml version="1.0" encoding="utf-8"?>
            <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema"
                     version="1.4.1">
              <asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset>
              <library_effects>
                <effect id="effect"><profile_COMMON><technique sid="common"><lambert>
                  <diffuse><color>0.2 0.4 0.6 0.8</color></diffuse>
                </lambert></technique></profile_COMMON></effect>
              </library_effects>
              <library_materials>
                <material id="material"><instance_effect url="#effect"/></material>
              </library_materials>
              <library_geometries>
                <geometry id="geometry"><mesh>
                  <source id="positions">
                    <float_array id="positions-array" count="9">
                      0 0 0  1 0 0  0 1 0
                    </float_array>
                    <technique_common>
                      <accessor source="#positions-array" count="3" stride="3">
                        <param name="X"/><param name="Y"/><param name="Z"/>
                      </accessor>
                    </technique_common>
                  </source>
                  <source id="normals">
                    <float_array id="normals-array" count="3">1 1 0</float_array>
                    <technique_common>
                      <accessor source="#normals-array" count="1" stride="3">
                        <param name="X"/><param name="Y"/><param name="Z"/>
                      </accessor>
                    </technique_common>
                  </source>
                  <vertices id="vertices">
                    <input semantic="POSITION" source="#positions"/>
                  </vertices>
                  <triangles count="1" material="symbol">
                    <input semantic="VERTEX" source="#vertices" offset="0"/>
                    <input semantic="NORMAL" source="#normals" offset="1"/>
                    <p>0 0 1 0 2 0</p>
                  </triangles>
                </mesh></geometry>
              </library_geometries>
              <library_visual_scenes>
                <visual_scene id="scene">
                  <node id="parent">
                    <matrix>1 0 0 1  0 1 0 2  0 0 1 3  0 0 0 1</matrix>
                    <node id="scaled-child">
                      <matrix>2 0 0 0.5  0 3 0 -0.25  0 0 4 0.75  0 0 0 1</matrix>
                      <instance_geometry url="#geometry">
                        <bind_material><technique_common>
                          <instance_material symbol="symbol" target="#material"/>
                        </technique_common></bind_material>
                      </instance_geometry>
                    </node>
                  </node>
                </visual_scene>
              </library_visual_scenes>
              <scene><instance_visual_scene url="#scene"/></scene>
            </COLLADA>
            """
        ),
        encoding='utf-8',
    )

    converted = convert_mesh(source, tmp_path / 'output', 'synthetic/probe.dae')
    metadata = json.loads(converted.metadata_path.read_text(encoding='utf-8'))
    binary = converted.binary_path.read_bytes()
    layout = {item['name']: item for item in metadata['layout']}
    positions = _attribute_values(binary, layout['position'])
    normals = _attribute_values(binary, layout['normal'])

    assert positions == pytest.approx(
        (1.5, 1.75, 3.75, 3.5, 1.75, 3.75, 1.5, 4.75, 3.75),
        abs=1e-6,
    )
    expected_normal = (3.0 / math.sqrt(13.0), 2.0 / math.sqrt(13.0), 0.0)
    for offset in range(0, len(normals), 3):
        assert normals[offset:offset + 3] == pytest.approx(expected_normal, abs=1e-6)
    assert metadata['groups'] == [{
        'start': 0,
        'count': 3,
        'color': [0.2, 0.4, 0.6],
        'opacity': 0.8,
    }]
