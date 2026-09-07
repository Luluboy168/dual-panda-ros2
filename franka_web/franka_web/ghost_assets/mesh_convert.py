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

"""Convert COLLADA triangle meshes to the ghost mesh format."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


SCHEMA = 'franka.ghost.mesh/1'
COLLADA_NAMESPACE = 'http://www.collada.org/2005/11/COLLADASchema'
NS = {'c': COLLADA_NAMESPACE}
IDENTITY = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)


@dataclass(frozen=True)
class ConvertedMesh:
    """Paths and summary values produced by one conversion."""

    metadata_path: Path
    binary_path: Path
    source_sha256: str
    vertices: int
    triangles: int


def _strip_ref(value: str | None, context: str) -> str:
    if not value or not value.startswith('#'):
        raise ValueError(f'{context} must be a local COLLADA reference')
    return value[1:]


def _floats(element: ET.Element | None, context: str) -> tuple[float, ...]:
    if element is None or not element.text:
        raise ValueError(f'missing numeric data for {context}')
    values = tuple(float(value) for value in element.text.split())
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'non-finite numeric data in {context}')
    return values


def _multiply(left: tuple[float, ...], right: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(
        sum(left[row * 4 + k] * right[k * 4 + column] for k in range(4))
        for row in range(4)
        for column in range(4)
    )


def _point(matrix: tuple[float, ...], value: tuple[float, ...]) -> tuple[float, ...]:
    x, y, z = value
    transformed = (
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    )
    if not all(math.isfinite(component) for component in transformed):
        raise ValueError('node transform produced a non-finite vertex')
    return transformed


def _normal_matrix(matrix: tuple[float, ...]) -> tuple[float, ...]:
    a, b, c = matrix[0:3]
    d, e, f = matrix[4:7]
    g, h, i = matrix[8:11]
    determinant = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(determinant) < 1e-15:
        raise ValueError('node transform has a singular normal matrix')
    inverse = (
        (e * i - f * h) / determinant,
        (c * h - b * i) / determinant,
        (b * f - c * e) / determinant,
        (f * g - d * i) / determinant,
        (a * i - c * g) / determinant,
        (c * d - a * f) / determinant,
        (d * h - e * g) / determinant,
        (b * g - a * h) / determinant,
        (a * e - b * d) / determinant,
    )
    return (
        inverse[0], inverse[3], inverse[6],
        inverse[1], inverse[4], inverse[7],
        inverse[2], inverse[5], inverse[8],
    )


def _normal(matrix: tuple[float, ...], value: tuple[float, ...]) -> tuple[float, ...]:
    x, y, z = value
    transformed = (
        matrix[0] * x + matrix[1] * y + matrix[2] * z,
        matrix[3] * x + matrix[4] * y + matrix[5] * z,
        matrix[6] * x + matrix[7] * y + matrix[8] * z,
    )
    length = math.sqrt(sum(component * component for component in transformed))
    if not math.isfinite(length) or length < 1e-15:
        raise ValueError('normal transform produced a zero or non-finite normal')
    return tuple(component / length for component in transformed)


def _sources(mesh: ET.Element) -> dict[str, tuple[tuple[float, ...], ...]]:
    arrays = {
        array.attrib['id']: _floats(array, array.attrib['id'])
        for array in mesh.findall('c:source/c:float_array', NS)
    }
    result = {}
    for source in mesh.findall('c:source', NS):
        accessor = source.find('c:technique_common/c:accessor', NS)
        if accessor is None:
            raise ValueError(f"source {source.attrib['id']} has no accessor")
        data = arrays[_strip_ref(accessor.get('source'), 'accessor source')]
        count = int(accessor.attrib['count'])
        stride = int(accessor.get('stride', '1'))
        offset = int(accessor.get('offset', '0'))
        if stride < 3 or offset + count * stride > len(data):
            raise ValueError(f"invalid accessor for source {source.attrib['id']}")
        result[source.attrib['id']] = tuple(
            tuple(data[offset + index * stride:offset + index * stride + 3])
            for index in range(count)
        )
    return result


def _effects(root: ET.Element) -> dict[str, tuple[float, float, float, float]]:
    result = {}
    for effect in root.findall('c:library_effects/c:effect', NS):
        diffuse = effect.find('.//c:diffuse/c:color', NS)
        values = _floats(diffuse, f"effect {effect.attrib['id']} diffuse")
        if len(values) not in (3, 4):
            raise ValueError(f"effect {effect.attrib['id']} has invalid diffuse colour")
        rgba = (*values[:3], values[3] if len(values) == 4 else 1.0)
        result[effect.attrib['id']] = rgba
    return result


def _materials(root: ET.Element) -> dict[str, tuple[float, float, float, float]]:
    effects = _effects(root)
    result = {}
    for material in root.findall('c:library_materials/c:material', NS):
        instance = material.find('c:instance_effect', NS)
        effect_id = _strip_ref(
            instance.get('url') if instance is not None else None,
            f"material {material.attrib['id']}",
        )
        result[material.attrib['id']] = effects[effect_id]
    return result


def _geometry_data(root: ET.Element) -> dict[str, tuple[ET.Element, dict]]:
    result = {}
    for geometry in root.findall('c:library_geometries/c:geometry', NS):
        mesh = geometry.find('c:mesh', NS)
        if mesh is None:
            raise ValueError(f"geometry {geometry.attrib['id']} has no mesh")
        vertices = {}
        for vertex_set in mesh.findall('c:vertices', NS):
            semantic_sources = {
                item.attrib['semantic']: _strip_ref(
                    item.get('source'), 'vertices input source'
                )
                for item in vertex_set.findall('c:input', NS)
            }
            vertices[vertex_set.attrib['id']] = semantic_sources
        unsupported = [
            child.tag.rsplit('}', 1)[-1]
            for child in mesh
            if child.tag.rsplit('}', 1)[-1]
            in {'polylist', 'polygons', 'trifans', 'tristrips'}
        ]
        if unsupported:
            raise ValueError(f'unsupported COLLADA primitives: {unsupported}')
        result[geometry.attrib['id']] = (
            mesh,
            {'sources': _sources(mesh), 'vertices': vertices},
        )
    return result


def _node_matrix(node: ET.Element) -> tuple[float, ...]:
    transform = IDENTITY
    for child in node:
        tag = child.tag.rsplit('}', 1)[-1]
        if tag == 'matrix':
            values = _floats(child, f"node {node.get('id', '<unnamed>')} matrix")
            if len(values) != 16:
                raise ValueError('COLLADA node matrix must contain 16 values')
            transform = _multiply(transform, values)
        elif tag in {'translate', 'rotate', 'scale', 'lookat', 'skew'}:
            raise ValueError(f'unsupported node transform element: {tag}')
    return transform


def _scene_instances(
    root: ET.Element,
) -> tuple[tuple[ET.Element, tuple[float, ...]], ...]:
    scene_instance = root.find('c:scene/c:instance_visual_scene', NS)
    scene_id = _strip_ref(
        scene_instance.get('url') if scene_instance is not None else None,
        'visual scene',
    )
    scenes = {
        scene.attrib['id']: scene
        for scene in root.findall('c:library_visual_scenes/c:visual_scene', NS)
    }
    if scene_id not in scenes:
        raise ValueError(f'visual scene not found: {scene_id}')

    instances = []

    def visit(node: ET.Element, parent: tuple[float, ...]) -> None:
        world = _multiply(parent, _node_matrix(node))
        for instance in node.findall('c:instance_geometry', NS):
            instances.append((instance, world))
        for child in node.findall('c:node', NS):
            visit(child, world)

    for node in scenes[scene_id].findall('c:node', NS):
        visit(node, IDENTITY)
    return tuple(instances)


def _input_sources(
    triangle: ET.Element,
    geometry: dict,
) -> tuple[int, int, str, int, str]:
    inputs = triangle.findall('c:input', NS)
    if not inputs:
        raise ValueError('triangles primitive has no inputs')
    stride = max(int(item.get('offset', '0')) for item in inputs) + 1
    position_offset = normal_offset = None
    position_source = normal_source = None
    for item in inputs:
        semantic = item.attrib['semantic']
        source_id = _strip_ref(item.get('source'), 'triangle input source')
        if semantic == 'VERTEX':
            position_offset = int(item.get('offset', '0'))
            try:
                position_source = geometry['vertices'][source_id]['POSITION']
            except KeyError as error:
                raise ValueError('VERTEX input has no POSITION source') from error
        elif semantic == 'NORMAL':
            normal_offset = int(item.get('offset', '0'))
            normal_source = source_id
    if position_offset is None or normal_offset is None:
        raise ValueError('triangles must declare VERTEX and NORMAL inputs')
    return stride, position_offset, position_source, normal_offset, normal_source


def _append_instance(
    instance: ET.Element,
    transform: tuple[float, ...],
    geometries: dict,
    materials: dict,
    positions: list[float],
    normals: list[float],
    groups: list[dict],
) -> None:
    geometry_id = _strip_ref(instance.get('url'), 'instance geometry')
    mesh, geometry = geometries[geometry_id]
    bindings = {
        binding.attrib['symbol']: _strip_ref(
            binding.get('target'), 'instance material target'
        )
        for binding in instance.findall(
            'c:bind_material/c:technique_common/c:instance_material', NS
        )
    }
    normal_matrix = _normal_matrix(transform)
    for triangle in mesh.findall('c:triangles', NS):
        declared_count = int(triangle.attrib['count'])
        stride, position_offset, position_source, normal_offset, normal_source = (
            _input_sources(triangle, geometry)
        )
        indices = []
        for index_list in triangle.findall('c:p', NS):
            indices.extend(int(value) for value in (index_list.text or '').split())
        expected = declared_count * 3 * stride
        if len(indices) != expected:
            raise ValueError(
                f'triangles index count is {len(indices)}, expected {expected}'
            )
        start = len(positions) // 3
        sources = geometry['sources']
        for offset in range(0, len(indices), stride):
            position = sources[position_source][indices[offset + position_offset]]
            normal = sources[normal_source][indices[offset + normal_offset]]
            positions.extend(_point(transform, position))
            normals.extend(_normal(normal_matrix, normal))
        symbol = triangle.get('material')
        material_id = bindings.get(symbol, symbol)
        if not material_id or material_id not in materials:
            raise ValueError(f'material binding not found: {symbol}')
        colour = materials[material_id]
        groups.append(
            {
                'start': start,
                'count': declared_count * 3,
                'color': list(colour[:3]),
                'opacity': colour[3],
            }
        )


def _validate_document(root: ET.Element) -> None:
    namespace = root.tag.split('}', 1)[0].lstrip('{')
    if namespace != COLLADA_NAMESPACE or root.get('version') != '1.4.1':
        raise ValueError('only COLLADA 1.4.1 documents are supported')
    up_axis = root.findtext('c:asset/c:up_axis', namespaces=NS)
    unit = root.find('c:asset/c:unit', NS)
    if up_axis != 'Z_UP':
        raise ValueError(f'expected Z_UP, got {up_axis!r}')
    if unit is None or not math.isclose(float(unit.get('meter', 'nan')), 1.0):
        raise ValueError('expected COLLADA unit meter=1')


def convert_mesh(
    source_path: Path,
    output_directory: Path,
    source_label: str | None = None,
) -> ConvertedMesh:
    """Convert one COLLADA file to deterministic JSON metadata and binary data."""
    source = Path(source_path)
    source_bytes = source.read_bytes()
    root = ET.fromstring(source_bytes)
    _validate_document(root)

    positions: list[float] = []
    normals: list[float] = []
    groups: list[dict] = []
    geometries = _geometry_data(root)
    materials = _materials(root)
    for instance, transform in _scene_instances(root):
        _append_instance(
            instance,
            transform,
            geometries,
            materials,
            positions,
            normals,
            groups,
        )
    if not positions or len(positions) != len(normals):
        raise ValueError('converted mesh has missing position or normal data')

    vertex_count = len(positions) // 3
    triangle_count = vertex_count // 3
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    stem = source.stem

    position_data = struct.pack(f'<{len(positions)}f', *positions)
    normal_data = struct.pack(f'<{len(normals)}f', *normals)
    payload = position_data + normal_data
    # Content-addressed names: the first twelve hex characters of the
    # binary's own digest. Two runs over the same source produce the same
    # bytes and therefore the same name, and a file whose name changes when
    # its content does is one a browser may cache forever.
    digest12 = hashlib.sha256(payload).hexdigest()[:12]
    binary_name = f'{stem}.{digest12}.ghostmesh.bin'
    metadata_name = f'{stem}.{digest12}.ghostmesh.json'
    binary_path = output / binary_name
    metadata_path = output / metadata_name
    binary_path.write_bytes(payload)
    metadata = {
        'schema': SCHEMA,
        'source': source_label or source.as_posix(),
        'sha256': hashlib.sha256(source_bytes).hexdigest(),
        'up_axis': 'Z_UP',
        'unit_m': 1.0,
        'counts': {'vertices': vertex_count, 'triangles': triangle_count},
        'bin': binary_name,
        'layout': [
            {
                'name': 'position',
                'type': 'float32',
                'items': 3,
                'count': vertex_count,
                'byteOffset': 0,
            },
            {
                'name': 'normal',
                'type': 'float32',
                'items': 3,
                'count': vertex_count,
                'byteOffset': len(position_data),
            },
        ],
        'groups': groups,
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2, separators=(',', ': ')) + '\n',
        encoding='utf-8',
    )
    return ConvertedMesh(
        metadata_path=metadata_path,
        binary_path=binary_path,
        source_sha256=metadata['sha256'],
        vertices=vertex_count,
        triangles=triangle_count,
    )
