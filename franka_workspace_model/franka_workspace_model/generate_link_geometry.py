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
Generate link_geometry_v1.yaml from the robot description.

Every number this writes is already present in franka_description; the file's
whole job is to make a change to the description visible as a diff instead of as
a silent change of behaviour in the checker.  Generation is fail-closed: the
generator aborts rather than emit a volume it does not fully understand, because
a half-understood collision model is worse than no model - it is trusted.

The generator imports no ROS module.  It invokes the ``xacro`` executable as a
subprocess against a temporary ament-index overlay that resolves the exact
source tree being generated from, so the result never depends on what happens to
be installed.
"""

import argparse
import hashlib
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ElementTree

import numpy as np

from .geometry import rotation_from_rpy, segment_point_distance


GENERATOR_VERSION = 1
#: Emitted radii are rounded up to a whole multiple of this, so a conservative
#: radius is a round number a human can check by eye.
RADIUS_ROUNDING_M = 0.0005
#: The composition of a cylinder and two spheres is an exact capsule only if the
#: caps land on the sphere centres to this tolerance.
EXACT_CAPSULE_TOLERANCE = 1e-12
#: Rounding used by the canonical endpoint ordering and by float emission.
EMITTED_DECIMALS = 12
#: Recovered radii, minus the recovered safety distance, must be exactly this
#: set.  Any other value means the description grew geometry this generator has
#: never seen.
EXPECTED_RADIUS_BASES = (0.025, 0.03, 0.04, 0.05, 0.06)


class GenerationError(RuntimeError):
    """The description contains geometry this generator refuses to guess about."""


def _format_float(value: float) -> str:
    """Emit a float deterministically: fixed rounding, no exponent, no minus zero."""
    rounded = round(float(value), EMITTED_DECIMALS)
    if rounded == 0.0:
        rounded = 0.0
    text = '{:.{}f}'.format(rounded, EMITTED_DECIMALS).rstrip('0')
    if text.endswith('.'):
        text += '0'
    return text


def _format_vector(values) -> str:
    return '[' + ', '.join(_format_float(value) for value in values) + ']'


def _quote(value: str) -> str:
    if any(character in value for character in '"\\\n'):
        raise GenerationError('cannot emit the string {!r}'.format(value))
    return '"{}"'.format(value)


def _floats(text: str, count: int, context: str) -> np.ndarray:
    parts = text.split()
    if len(parts) != count:
        raise GenerationError('{} must carry {} numbers, found {!r}'.format(
            context, count, text))
    try:
        values = [float(part) for part in parts]
    except ValueError as error:
        raise GenerationError('{} is not numeric: {!r}'.format(context, text)) from error
    if not all(math.isfinite(value) for value in values):
        raise GenerationError('{} must be finite: {!r}'.format(context, text))
    return np.array(values, dtype=float)


def declared_xacro_arguments(xacro_path: Path) -> tuple:
    """Return the argument names the xacro declares, in declaration order."""
    root = ElementTree.parse(str(xacro_path)).getroot()
    names = []
    for element in root:
        if element.tag.endswith('}arg') or element.tag == 'xacro:arg':
            name = element.attrib.get('name')
            if name is not None and name not in names:
                names.append(name)
    if not names:
        raise GenerationError('{} declares no xacro arguments'.format(xacro_path))
    return tuple(names)


def recover_safety_distance(xacro_path: Path) -> float:
    """
    Recover the collision inflation from the arm call sites, never assume it.

    The arm macro's own default is zero, so a caller that omits the argument
    silently gets uninflated capsules.  A generator that assumed 0.03 would then
    record a number the geometry does not contain.
    """
    root = ElementTree.parse(str(xacro_path)).getroot()
    values = []
    for element in root.iter():
        tag = element.tag.rsplit('}', 1)[-1]
        if tag not in ('panda_arm', 'hand'):
            continue
        if 'safety_distance' not in element.attrib:
            raise GenerationError(
                'the arm instantiation in {} passes no safety_distance; the macro '
                'default is 0 and the capsules would be uninflated'.format(xacro_path.name))
        values.append(element.attrib['safety_distance'])
    if not values:
        raise GenerationError('{} instantiates no arm macro'.format(xacro_path.name))
    if len(set(values)) != 1:
        raise GenerationError(
            'the arm instantiations in {} disagree about safety_distance: {}'.format(
                xacro_path.name, sorted(set(values))))
    try:
        recovered = float(values[0])
    except ValueError as error:
        raise GenerationError('safety_distance {!r} is not numeric'.format(
            values[0])) from error
    if not math.isfinite(recovered) or recovered < 0.0:
        raise GenerationError('safety_distance must be finite and non-negative')
    return recovered


def render_urdf(xacro_path: Path, description_share: Path, arguments) -> str:
    """Render the xacro against an ament-index overlay for the exact source tree."""
    executable = shutil.which('xacro')
    if executable is None:
        raise GenerationError('the xacro executable is not on PATH')
    with tempfile.TemporaryDirectory() as scratch:
        prefix = Path(scratch) / 'source_overlay'
        marker = prefix / 'share/ament_index/resource_index/packages' / description_share.name
        marker.parent.mkdir(parents=True)
        marker.write_text('', encoding='utf-8')
        share = prefix / 'share' / description_share.name
        share.parent.mkdir(parents=True, exist_ok=True)
        share.symlink_to(description_share, target_is_directory=True)
        environment = dict(os.environ)
        existing = environment.get('AMENT_PREFIX_PATH', '')
        environment['AMENT_PREFIX_PATH'] = (
            str(prefix) if not existing else '{}{}{}'.format(prefix, os.pathsep, existing))
        command = [executable, str(xacro_path)]
        command.extend('{}:={}'.format(name, value) for name, value in arguments)
        try:
            completed = subprocess.run(
                command, check=True, capture_output=True, encoding='utf-8',
                env=environment, timeout=120)
        except subprocess.CalledProcessError as error:
            raise GenerationError('xacro failed: {}'.format(error.stderr.strip())) from error
    return completed.stdout


def _collision_primitive(element, index: int, link_name: str) -> dict:
    origin = element.find('origin')
    origin_xyz = np.zeros(3)
    origin_rpy = np.zeros(3)
    if origin is not None:
        if 'xyz' in origin.attrib:
            origin_xyz = _floats(origin.attrib['xyz'], 3,
                                 '{} collision {} origin xyz'.format(link_name, index))
        if 'rpy' in origin.attrib:
            origin_rpy = _floats(origin.attrib['rpy'], 3,
                                 '{} collision {} origin rpy'.format(link_name, index))
    geometry = element.find('geometry')
    if geometry is None or len(list(geometry)) != 1:
        raise GenerationError(
            '{} collision {} has a geometry element with {} children; exactly one is '
            'required, guessing which child is the geometry is guessing about a '
            'collision volume'.format(
                link_name, index, 0 if geometry is None else len(list(geometry))))
    shape = list(geometry)[0]
    kind = shape.tag.rsplit('}', 1)[-1]
    if kind not in ('cylinder', 'sphere', 'box'):
        raise GenerationError(
            '{} collision {} is a <{}>; this contract has no vocabulary for it and '
            'emitting nothing would silently shrink the collision model'.format(
                link_name, index, kind))
    primitive = {'kind': kind, 'origin_xyz': origin_xyz, 'origin_rpy': origin_rpy,
                 'index': index}
    if kind == 'cylinder':
        primitive['radius'] = float(shape.attrib['radius'])
        primitive['length'] = float(shape.attrib['length'])
    elif kind == 'sphere':
        primitive['radius'] = float(shape.attrib['radius'])
    else:
        primitive['size'] = _floats(shape.attrib['size'], 3,
                                    '{} collision {} box size'.format(link_name, index))
    return primitive


def _canonical_order(first: np.ndarray, second: np.ndarray):
    """Emit the endpoint pair in descending lexicographic order on (z, y, x)."""
    key_first = tuple(round(float(value), EMITTED_DECIMALS) for value in first[::-1])
    key_second = tuple(round(float(value), EMITTED_DECIMALS) for value in second[::-1])
    if key_first >= key_second:
        return first, second
    return second, first


def _cylinder_extreme_points(primitive: dict, samples: int = 8192) -> np.ndarray:
    """Return the extreme points of a solid cylinder: its two rim circles."""
    rotation = rotation_from_rpy(*primitive['origin_rpy'])
    axis = rotation[:, 2]
    first = rotation[:, 0]
    second = rotation[:, 1]
    centre = primitive['origin_xyz']
    half_length = 0.5 * primitive['length']
    radius = primitive['radius']
    angles = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
    angles = np.concatenate([angles, [0.0, math.pi, 0.5 * math.pi, 1.5 * math.pi]])
    rim = (np.cos(angles)[:, None] * first[None, :]
           + np.sin(angles)[:, None] * second[None, :]) * radius
    points = np.vstack([centre + half_length * axis + rim,
                        centre - half_length * axis + rim])
    return points


def _minimal_enclosing_radius(primitives, point_a, point_b) -> float:
    """
    Return the smallest radius about [a, b] containing the primitives.

    Distance to a segment is convex, so on each convex source primitive the
    maximum is attained at an extreme point: the rim circles of a cylinder, the
    farthest surface point of a sphere.
    """
    worst = 0.0
    for primitive in primitives:
        if primitive['kind'] == 'sphere':
            centre_distance = segment_point_distance(point_a, point_b,
                                                     primitive['origin_xyz'])
            worst = max(worst, centre_distance + primitive['radius'])
        elif primitive['kind'] == 'cylinder':
            for point in _cylinder_extreme_points(primitive):
                worst = max(worst, segment_point_distance(point_a, point_b, point))
        else:
            corners = primitive['origin_xyz'][None, :] + 0.5 * np.array([
                [sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
            ]) * primitive['size'][None, :]
            for point in corners:
                worst = max(worst, segment_point_distance(point_a, point_b, point))
    return worst


def _volume_from_triple(triple, link_name: str, volume_index: int) -> dict:
    kinds = [primitive['kind'] for primitive in triple]
    if kinds != ['cylinder', 'sphere', 'sphere']:
        raise GenerationError(
            '{} collision triple {} is {} rather than [cylinder, sphere, sphere]; the '
            'grouping assumption has failed and the resulting capsule would be '
            'arbitrary'.format(link_name, volume_index, kinds))
    cylinder, sphere_one, sphere_two = triple
    radii = [primitive['radius'] for primitive in triple]
    point_a, point_b = _canonical_order(sphere_one['origin_xyz'], sphere_two['origin_xyz'])
    rotation = rotation_from_rpy(*cylinder['origin_rpy'])
    axis = rotation[:, 2]
    cap_one = cylinder['origin_xyz'] + 0.5 * cylinder['length'] * axis
    cap_two = cylinder['origin_xyz'] - 0.5 * cylinder['length'] * axis
    equal_radii = max(radii) - min(radii) <= EXACT_CAPSULE_TOLERANCE
    caps_match = min(
        max(float(np.abs(cap_one - sphere_one['origin_xyz']).max()),
            float(np.abs(cap_two - sphere_two['origin_xyz']).max())),
        max(float(np.abs(cap_one - sphere_two['origin_xyz']).max()),
            float(np.abs(cap_two - sphere_one['origin_xyz']).max())),
    ) <= EXACT_CAPSULE_TOLERANCE
    volume = {
        'id': '{}_v{}'.format(link_name, volume_index),
        'kind': 'capsule',
        'a': point_a,
        'b': point_b,
        'source_elements': [primitive['index'] for primitive in triple],
    }
    if equal_radii and caps_match:
        volume['radius'] = radii[0]
        volume['containment'] = 'exact'
        volume['containment_margin'] = 0.0
        return volume
    minimal = _minimal_enclosing_radius(triple, point_a, point_b)
    emitted = math.ceil(minimal / RADIUS_ROUNDING_M - 1e-12) * RADIUS_ROUNDING_M
    volume['radius'] = emitted
    volume['containment'] = 'conservative'
    volume['containment_margin'] = emitted - minimal
    return volume


def _link_volumes(link, link_name: str, safety_distance: float) -> tuple:
    collisions = link.findall('collision')
    primitives = [_collision_primitive(element, index, link_name)
                  for index, element in enumerate(collisions)]
    if not primitives:
        return (), ()
    if len(primitives) == 1 and primitives[0]['kind'] == 'box':
        primitive = primitives[0]
        if float(np.abs(primitive['origin_rpy']).max()) > EXACT_CAPSULE_TOLERANCE:
            raise GenerationError(
                '{} carries a rotated collision box; derived geometry is '
                'axis-aligned in its own link frame in v1'.format(link_name))
        return ({
            'id': '{}_v0'.format(link_name),
            'kind': 'box',
            'size': primitive['size'],
            'origin_xyz': primitive['origin_xyz'],
            'containment': 'exact',
            'containment_margin': 0.0,
            'source_elements': [0],
        },), tuple(primitives)
    if len(primitives) % 3 != 0:
        raise GenerationError(
            '{} has {} collision elements, which is not a multiple of three; the '
            'cylinder + sphere + sphere grouping is positional, so a non-multiple '
            'means the assumed grouping is wrong'.format(link_name, len(primitives)))
    volumes = []
    for volume_index in range(len(primitives) // 3):
        triple = primitives[volume_index * 3:volume_index * 3 + 3]
        volumes.append(_volume_from_triple(triple, link_name, volume_index))
    return tuple(volumes), tuple(primitives)


def _joint_entry(joint) -> dict:
    kind = joint.attrib.get('type')
    if kind not in ('revolute', 'fixed'):
        raise GenerationError(
            'joint {!r} has type {!r}; only revolute and fixed joints are '
            'understood'.format(joint.attrib.get('name'), kind))
    origin = joint.find('origin')
    origin_xyz = np.zeros(3)
    origin_rpy = np.zeros(3)
    if origin is not None:
        if 'xyz' in origin.attrib:
            origin_xyz = _floats(origin.attrib['xyz'], 3, 'joint origin xyz')
        if 'rpy' in origin.attrib:
            origin_rpy = _floats(origin.attrib['rpy'], 3, 'joint origin rpy')
    entry = {
        'name': joint.attrib['name'],
        'type': kind,
        'origin_xyz': origin_xyz,
        'origin_rpy': origin_rpy,
        'parent': joint.find('parent').attrib['link'],
        'child': joint.find('child').attrib['link'],
    }
    if kind == 'revolute':
        axis = joint.find('axis')
        entry['axis'] = _floats(
            '0 0 1' if axis is None else axis.attrib['xyz'], 3, 'joint axis')
        norm = float(np.linalg.norm(entry['axis']))
        if abs(norm - 1.0) > 1e-9:
            raise GenerationError(
                'joint {!r} axis is not a unit vector'.format(entry['name']))
        limit = joint.find('limit')
        if limit is None:
            raise GenerationError(
                'revolute joint {!r} declares no limit'.format(entry['name']))
        entry['limit_lower'] = float(limit.attrib['lower'])
        entry['limit_upper'] = float(limit.attrib['upper'])
        if not entry['limit_lower'] < entry['limit_upper']:
            raise GenerationError(
                'revolute joint {!r} has an empty limit range'.format(entry['name']))
    return entry


def _check_radius_decomposition(primitives, safety_distance: float) -> None:
    bases = set()
    for primitive in primitives:
        if 'radius' not in primitive:
            continue
        bases.add(round(primitive['radius'] - safety_distance, 9))
    unexpected = sorted(bases - {round(value, 9) for value in EXPECTED_RADIUS_BASES})
    if unexpected:
        raise GenerationError(
            'collision radii minus the recovered safety_distance ({}) give the '
            'unexpected bases {}; the expected set is {}'.format(
                safety_distance, unexpected, list(EXPECTED_RADIUS_BASES)))


def build_document(urdf_text: str, urdf_xacro_relative: str, xacro_arguments,
                   safety_distance: float) -> str:
    """Build the deterministic YAML text of link_geometry_v1.yaml."""
    root = ElementTree.fromstring(urdf_text)
    joints = [_joint_entry(joint) for joint in root.findall('joint')]
    by_child = {joint['child']: joint for joint in joints}
    lines = []
    all_primitives = []
    link_entries = []
    for link in root.findall('link'):
        name = link.attrib['name']
        volumes, primitives = _link_volumes(link, name, safety_distance)
        all_primitives.extend(primitives)
        if not volumes:
            continue
        link_entries.append((name, volumes))
    if not link_entries:
        raise GenerationError('the description carries no collision geometry')
    _check_radius_decomposition(all_primitives, safety_distance)
    roots = [name for name, _ in link_entries if name not in by_child]
    if len(roots) != 1:
        raise GenerationError(
            'the description has {} root links with collision geometry; exactly one '
            'is required'.format(len(roots)))
    root_link = roots[0]

    lines.append('schema_version: 1')
    lines.append('source:')
    lines.append('  urdf_xacro: {}'.format(urdf_xacro_relative))
    lines.append('  xacro_args:')
    for name, value in xacro_arguments:
        lines.append('    {}: {}'.format(name, _quote(value)))
    digest = hashlib.sha256(urdf_text.encode('utf-8')).hexdigest()
    lines.append('  urdf_sha256: {}'.format(_quote(digest)))
    lines.append('  safety_distance: {}'.format(_format_float(safety_distance)))
    lines.append('  generator_version: {}'.format(GENERATOR_VERSION))
    lines.append('root_link: {}'.format(root_link))
    lines.append('links:')
    for name, volumes in link_entries:
        lines.append('  - link: {}'.format(name))
        joint = by_child.get(name)
        if joint is not None:
            lines.append('    parent_link: {}'.format(joint['parent']))
            lines.append('    parent_joint:')
            lines.append('      name: {}'.format(joint['name']))
            lines.append('      type: {}'.format(joint['type']))
            lines.append('      origin_xyz: {}'.format(_format_vector(joint['origin_xyz'])))
            lines.append('      origin_rpy: {}'.format(_format_vector(joint['origin_rpy'])))
            if joint['type'] == 'revolute':
                lines.append('      axis: {}'.format(_format_vector(joint['axis'])))
                lines.append('      limit_lower: {}'.format(
                    _format_float(joint['limit_lower'])))
                lines.append('      limit_upper: {}'.format(
                    _format_float(joint['limit_upper'])))
        lines.append('    volumes:')
        for volume in volumes:
            lines.append('      - id: {}'.format(volume['id']))
            lines.append('        kind: {}'.format(volume['kind']))
            if volume['kind'] == 'capsule':
                lines.append('        a: {}'.format(_format_vector(volume['a'])))
                lines.append('        b: {}'.format(_format_vector(volume['b'])))
                lines.append('        radius: {}'.format(_format_float(volume['radius'])))
            elif volume['kind'] == 'sphere':
                lines.append('        origin_xyz: {}'.format(
                    _format_vector(volume['origin_xyz'])))
                lines.append('        radius: {}'.format(_format_float(volume['radius'])))
            else:
                lines.append('        size: {}'.format(_format_vector(volume['size'])))
                lines.append('        origin_xyz: {}'.format(
                    _format_vector(volume['origin_xyz'])))
            lines.append('        containment: {}'.format(volume['containment']))
            lines.append('        containment_margin: {}'.format(
                _format_float(volume['containment_margin'])))
            lines.append('        source_elements: [{}]'.format(
                ', '.join(str(index) for index in volume['source_elements'])))
    return '\n'.join(lines) + '\n'


def generate(repository_root: Path, urdf_xacro_relative: str, xacro_arguments) -> str:
    """Render the description and return the link_geometry_v1.yaml text."""
    xacro_path = repository_root / urdf_xacro_relative
    if not xacro_path.is_file():
        raise GenerationError('{} is not a file'.format(xacro_path))
    declared = declared_xacro_arguments(xacro_path)
    supplied = tuple(name for name, _ in xacro_arguments)
    if sorted(declared) != sorted(supplied):
        raise GenerationError(
            'xacro_args must record exactly the declared arguments {}; found {}'.format(
                list(declared), list(supplied)))
    ordered = [(name, dict(xacro_arguments)[name]) for name in declared]
    for name, value in ordered:
        if name.startswith('robot_ip') and value != '':
            raise GenerationError(
                'xacro_args.{} must be the empty string; a committed artefact never '
                'contains a network address'.format(name))
    safety_distance = recover_safety_distance(xacro_path)
    urdf_text = render_urdf(xacro_path, repository_root / 'franka_description', ordered)
    return build_document(urdf_text, urdf_xacro_relative, ordered, safety_distance)


def main(argv=None) -> int:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(
        description='Generate link_geometry_v1.yaml from franka_description')
    parser.add_argument('--repository-root', type=Path, required=True)
    parser.add_argument(
        '--urdf-xacro', default='franka_description/robots/real/dual_panda_arm.urdf.xacro')
    parser.add_argument('--arg', action='append', default=[], metavar='NAME:=VALUE')
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args(argv)
    pairs = []
    for item in arguments.arg:
        if ':=' not in item:
            print('--arg expects NAME:=VALUE, got {!r}'.format(item), file=sys.stderr)
            return 2
        name, value = item.split(':=', 1)
        pairs.append((name, value))
    try:
        text = generate(arguments.repository_root, arguments.urdf_xacro, pairs)
    except GenerationError as error:
        print('generation aborted: {}'.format(error), file=sys.stderr)
        return 2
    arguments.output.write_text(text, encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
