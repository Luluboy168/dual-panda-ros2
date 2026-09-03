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
Generate ``cell/mesh_bodies_v1.yaml``: the convex bodies the mesh fence measures.

WHAT A BODY IS.  For each link the *metal proxy* is the union of the
description's own collision solid and its own visual shell::

    M(L) = collision solid  u  visual-shell solid

and the bodies are built so that they CONTAIN that union rather than
approximate it.  Each visual-shell triangle is assigned, as a whole triangle, to
the collision piece nearest to it, and the body is the convex hull of that
piece's vertices together with the vertices of its assigned triangles.  The hull
of a superset contains the hull of any subset of its generators, so every
assigned triangle - all three of its vertices being generators - lies inside the
body it was assigned to, and therefore the whole shell surface lies inside the
union of the bodies.  That is a proof, not a sample, and it is what lets the
fence report ``gjk(body_a, body_b)`` with **nothing subtracted**: there is no
undercut term to fold back in, because the bodies already contain the shell.

Assignment is by whole TRIANGLE and not by vertex.  A vertex-wise assignment
splits triangles that span two collision pieces and the union then stops
containing the shell - measured at 1.1598 mm on ``link5``, silently, on the one
link that binds.

``link8`` IS DIFFERENT AND THE DIFFERENCE IS RECORDED.  ``franka_description``
ships no ``link8`` mesh at all - no visual, no collision - so its body is not
derived from geometry anybody measured.  It is the convex hull of the URDF's own
three ``link8`` collision primitives evaluated at METAL radius (the recovered
``safety_distance`` subtracted), sampled on a lattice that is inflated by a
pinned scale so that the hull CIRCUMSCRIBES the primitives.  A hull of points
sampled *on* a sphere lies strictly inside that sphere; emitting at the bare
metal radius would produce a body 0.308 mm SMALLER than the geometry it claims
to represent, which is optimism on the one body in the model with nothing behind
it.  The containment is certified face plane by face plane against the
closed-form support function of the primitive union, and the certificate travels
in the artefact.

THIS MODULE IS OFFLINE.  It is the single sanctioned reader of
``franka_description/mujoco`` and of any ``.stl`` / ``.obj`` / ``.dae`` bytes in
this package; no runtime module imports it or names those paths, and
``test_purity_and_performance.py`` asserts both halves.  It may use ``scipy``,
which the runtime may not.

GENERATION IS FAIL-CLOSED.  Nine aborts, each named in ``GenerationError``
messages; a body this generator does not fully understand is never emitted,
because a half-understood collision body is worse than none - it is trusted.
"""

import argparse
import hashlib
import math
import os
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ElementTree

import numpy as np

from .generate_link_geometry import GenerationError


#: Bumped whenever the emitted bytes change for a fixed description.
GENERATOR_VERSION = 1
#: Vertices and volumes are emitted at this many decimals.  Measured: rounding
#: the body vertices to 12 or 9 decimals changes the wrist hard case's distance
#: by exactly zero; 7 decimals moves it by 2.26e-09 m, outside T-GJK-1's
#: tolerance.  Twelve is therefore the shipped precision and the test reads the
#: shipped artefact rather than a re-rounded copy.
EMITTED_DECIMALS = 12

#: The metal radius of the three link8 primitives: the URDF's declared radius
#: with the recovered safety_distance subtracted.
FLANGE_METAL_RADIUS = 0.03
#: The lattice is emitted at FLANGE_METAL_RADIUS * this, so that the HULL of the
#: sampled points contains the primitives instead of being inscribed in them.
#: Not a guess: the smallest four-decimal value above the bisected minimum
#: containing scale below.
FLANGE_LATTICE_SCALE = 1.0104
FLANGE_MINIMUM_CONTAINING_SCALE = 1.010373504
FLANGE_CYLINDER_RIM_SAMPLES = 64
FLANGE_SPHERE_LATTICE = (16, 32)
#: Resolution of the dense re-check in abort 9, finer than the emitting lattice.
FLANGE_CERTIFICATE_LATTICE = (200, 400)
FLANGE_CERTIFICATE_RIM = (2000, 9)

#: Drift detection on the SOURCE assets, not soundness.  GJK reads vertices only
#: and therefore evaluates the convex HULL of whatever it is given; the hull
#: contains the mesh, so a convexity defect can only make GJK under-report the
#: body's extent - the safe direction.  Measured worst on the shipped assets:
#: 1.000151038e-03 m on link3.stl, well inside this gate.
CONVEXITY_TOLERANCE_M = 0.002

#: The mesh-bodies reader's own three caps.  strictyaml's MAXIMUM_MODEL_BYTES
#: (65 536), MAXIMUM_YAML_DEPTH (8) and MAXIMUM_YAML_SCALARS (2 048) stay in
#: force for the cell files; this artefact carries 34 779 vertex scalars, which
#: is seventeen times strictyaml's scalar cap, so the byte bound was never going
#: to be the binding one and all three are named here.
MAXIMUM_MESH_BODIES_BYTES = 786432
MAXIMUM_MESH_BODIES_SCALARS = 40000
MAXIMUM_MESH_BODIES_DEPTH = 8

MUJOCO_MODEL_RELATIVE = 'franka_description/mujoco/franka/mj_dual.xml'
VISUAL_MESH_DIRECTORY = 'franka_description/meshes/visual'

ARM_LINKS = ('link0', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7')

#: Collision geoms this generator deliberately does not turn into bodies.  Every
#: one is named with its reason and the generator ABORTS on a geom that is
#: skipped without being on this list; silence is what lets a collision body go
#: missing without anybody noticing.
IGNORED_COLLISION_GEOMS = {
    'hand_c': 'the Franka Hand is out of scope: this cell runs grippers-off, the '
              'hand is not in the URDF the model is generated from, and an '
              'end-effector volume is a CONTRACT A declaration, not derived geometry',
    'finger_0': 'the two finger collision geoms name finger_0, which is a VISUAL '
                'asset reused as a collision shape; grippers-off, out of scope',
}
#: The five fingertip-pad subclasses.  They are nested INSIDE <default
#: class="collision">, so a generator that resolves the MuJoCo class tree - the
#: natural implementation - sees them as collision geoms, and each of them
#: carries a pos.  They are ignored for the same reason as finger_0, and they
#: are named here so that abort 1 can be scoped to the mesh-typed geoms whose
#: class resolves to `collision` itself without becoming a lie.
IGNORED_COLLISION_CLASSES = tuple(
    'fingertip_pad_collision_{}'.format(index) for index in range(1, 6))

#: How many mesh-typed collision geoms the shipped file carries, and how many
#: fingertip-pad geoms.  Both are asserted, because both have been miscounted.
EXPECTED_MESH_COLLISION_GEOMS = 26
EXPECTED_FINGERTIP_PAD_GEOMS = 20

LINK8_REASON = (
    'franka_description ships no link8 mesh: meshes/visual holds link0.dae..link7.dae, '
    'hand.dae and finger.dae and nothing else, the URDF link8 block carries three '
    '<collision> elements and no <visual> element at all, and mj_dual.xml gives '
    'mj_left_link8 / mj_right_link8 a site and the hand subtree with no geom. The body '
    'is therefore the convex hull of the URDF primitives at metal radius, CIRCUMSCRIBING '
    'them, with a face-plane containment certificate; it is an envelope, not measured '
    'metal, and one caliper reading of the flange boss retires it.'
)


# ---------------------------------------------------------------------------
# Mesh IO.  This package's only reader of asset bytes.
# ---------------------------------------------------------------------------

def sha256_of_file(path: Path) -> str:
    """SHA-256 of one asset, read in bounded blocks."""
    digest = hashlib.sha256()
    try:
        with open(path, 'rb') as handle:
            while True:
                block = handle.read(65536)
                if not block:
                    break
                digest.update(block)
    except OSError as error:
        raise GenerationError('unable to read the asset {}'.format(path)) from error
    return digest.hexdigest()


def _deduplicate(triangles: np.ndarray):
    """(F, 3, 3) triangle soup -> (V, 3) vertices and (F, 3) indices."""
    flat = triangles.reshape(-1, 3)
    unique, inverse = np.unique(np.round(flat, 9), axis=0, return_inverse=True)
    return unique, inverse.reshape(-1, 3)


def read_stl(path: Path):
    """Read a binary STL.  An ASCII STL is refused rather than guessed at."""
    data = Path(path).read_bytes()
    if data[:5].lower().startswith(b'solid'):
        raise GenerationError('{} is an ASCII STL; only binary STL is read'.format(path))
    if len(data) < 84:
        raise GenerationError('{} is too short to be an STL'.format(path))
    count = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84 + 50 * count:
        raise GenerationError(
            '{} declares {} triangles but is {} bytes, not {}'.format(
                path, count, len(data), 84 + 50 * count))
    raw = np.frombuffer(data, dtype=np.uint8, offset=84).reshape(count, 50)
    triangles = raw[:, 12:48].copy().view('<f4').reshape(count, 3, 3).astype(float)
    return _deduplicate(triangles)


def read_obj(path: Path):
    """Read the vertex and face records of an ASCII OBJ, fan-triangulating."""
    vertices = []
    faces = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'v':
            vertices.append([float(value) for value in parts[1:4]])
        elif parts[0] == 'f':
            indices = [int(token.split('/')[0]) - 1 for token in parts[1:]]
            for offset in range(1, len(indices) - 1):
                faces.append([indices[0], indices[offset], indices[offset + 1]])
    if not vertices or not faces:
        raise GenerationError('{} carries no geometry'.format(path))
    return np.array(vertices, dtype=float), np.array(faces, dtype=int)


_COLLADA = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def read_dae(path: Path):
    """
    Read a COLLADA visual shell, applying every visual-scene node transform.

    The node matrices are not decoration: ``link5.dae`` carries a node at
    ``z = -0.259`` and a reader that ignores it places a quarter of the forearm
    in the wrong link frame.
    """
    root = ElementTree.parse(str(path)).getroot()
    geometries = {}
    for geometry in root.iter('{%s}geometry' % _COLLADA['c']):
        mesh = geometry.find('c:mesh', _COLLADA)
        if mesh is None:
            continue
        sources = {}
        for source in mesh.findall('c:source', _COLLADA):
            array = source.find('c:float_array', _COLLADA)
            if array is None:
                continue
            values = np.fromstring(array.text, sep=' ')
            accessor = source.find('c:technique_common/c:accessor', _COLLADA)
            stride = int(accessor.get('stride', 3)) if accessor is not None else 3
            sources[source.get('id')] = values.reshape(-1, stride)
        vertex_map = {}
        vertices_element = mesh.find('c:vertices', _COLLADA)
        if vertices_element is not None:
            for entry in vertices_element.findall('c:input', _COLLADA):
                if entry.get('semantic') == 'POSITION':
                    vertex_map[vertices_element.get('id')] = sources[
                        entry.get('source')[1:]]
        collected = []
        positions = None
        primitives = (list(mesh.findall('c:triangles', _COLLADA))
                      + list(mesh.findall('c:polylist', _COLLADA)))
        for primitive in primitives:
            inputs = primitive.findall('c:input', _COLLADA)
            stride = max(int(item.get('offset', 0)) for item in inputs) + 1
            offset = None
            for item in inputs:
                if item.get('semantic') == 'VERTEX':
                    offset = int(item.get('offset', 0))
                    name = item.get('source')[1:]
                    positions = vertex_map[name] if name in vertex_map else sources[name]
            indices = np.fromstring(
                primitive.find('c:p', _COLLADA).text, sep=' ').astype(int)
            indices = indices.reshape(-1, stride)[:, offset]
            if primitive.tag.endswith('polylist'):
                counts = np.fromstring(
                    primitive.find('c:vcount', _COLLADA).text, sep=' ').astype(int)
                cursor = 0
                for count in counts:
                    face = indices[cursor:cursor + count]
                    cursor += count
                    for corner in range(1, count - 1):
                        collected.append(
                            np.array([[face[0], face[corner], face[corner + 1]]]))
            else:
                collected.append(indices.reshape(-1, 3))
        if positions is None:
            continue
        stacked = (np.vstack(collected) if collected
                   else np.zeros((0, 3), dtype=int))
        geometries[geometry.get('id')] = (positions[:, :3], stacked)

    points = []
    faces = []

    def walk(node, transform):
        local = transform
        for matrix in node.findall('c:matrix', _COLLADA):
            local = local @ np.fromstring(matrix.text, sep=' ').reshape(4, 4)
        for translate in node.findall('c:translate', _COLLADA):
            step = np.eye(4)
            step[:3, 3] = np.fromstring(translate.text, sep=' ')
            local = local @ step
        for instance in node.findall('c:instance_geometry', _COLLADA):
            name = instance.get('url')[1:]
            if name in geometries:
                source_points, source_faces = geometries[name]
                placed = (local[:3, :3] @ source_points.T).T + local[:3, 3]
                faces.append(source_faces + sum(len(item) for item in points))
                points.append(placed)
        for child in node.findall('c:node', _COLLADA):
            walk(child, local)

    scene = root.find('c:library_visual_scenes/c:visual_scene', _COLLADA)
    if scene is None:
        raise GenerationError('{} carries no visual scene'.format(path))
    for node in scene.findall('c:node', _COLLADA):
        walk(node, np.eye(4))
    if not points:
        raise GenerationError('{} instances no geometry'.format(path))
    stacked_points = np.vstack(points)
    stacked_faces = np.vstack(faces) if faces else np.zeros((0, 3), dtype=int)
    unique, inverse = np.unique(np.round(stacked_points, 9), axis=0,
                                return_inverse=True)
    return unique, inverse[stacked_faces]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def hull_planes(vertices: np.ndarray, faces: np.ndarray):
    """Outward unit normals and offsets of a convex mesh's face planes."""
    first, second, third = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    normals = np.cross(second - first, third - first)
    lengths = np.linalg.norm(normals, axis=1)
    keep = lengths > 1e-14
    normals = normals[keep] / lengths[keep][:, None]
    offsets = np.einsum('ij,ij->i', normals, first[keep])
    interior = vertices.mean(axis=0)
    flip = (normals @ interior - offsets) > 0
    normals[flip] *= -1
    offsets[flip] *= -1
    key = np.round(np.column_stack([normals, offsets]), 7)
    _, index = np.unique(key, axis=0, return_index=True)
    return normals[index], offsets[index]


def point_triangle_distance(points: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """Exact point-to-triangle distances, (N, M), Ericson's closest-point regions."""
    first = triangles[:, 0][None, :, :]
    second = triangles[:, 1][None, :, :]
    third = triangles[:, 2][None, :, :]
    query = points[:, None, :]
    edge_one = second - first
    edge_two = third - first
    to_first = query - first
    d1 = np.einsum('nmk,nmk->nm', edge_one, to_first)
    d2 = np.einsum('nmk,nmk->nm', edge_two, to_first)
    to_second = query - second
    d3 = np.einsum('nmk,nmk->nm', edge_one, to_second)
    d4 = np.einsum('nmk,nmk->nm', edge_two, to_second)
    to_third = query - third
    d5 = np.einsum('nmk,nmk->nm', edge_one, to_third)
    d6 = np.einsum('nmk,nmk->nm', edge_two, to_third)
    vc = d1 * d4 - d3 * d2
    vb = d5 * d2 - d1 * d6
    va = d3 * d6 - d5 * d4
    total = va + vb + vc
    with np.errstate(divide='ignore', invalid='ignore'):
        safe = np.where(total != 0, total, 1.0)
        v_face = np.where(total != 0, vb / safe, 0.0)
        w_face = np.where(total != 0, vc / safe, 0.0)
        closest = first + v_face[..., None] * edge_one + w_face[..., None] * edge_two
        denominator_ab = d1 - d3
        t_ab = np.where(denominator_ab != 0,
                        d1 / np.where(denominator_ab != 0, denominator_ab, 1.0), 0.0)
        denominator_ac = d2 - d6
        t_ac = np.where(denominator_ac != 0,
                        d2 / np.where(denominator_ac != 0, denominator_ac, 1.0), 0.0)
        denominator_bc = (d4 - d3) + (d5 - d6)
        t_bc = np.where(denominator_bc != 0,
                        (d4 - d3) / np.where(denominator_bc != 0, denominator_bc, 1.0),
                        0.0)
    region_bc = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
    region_ac = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
    region_ab = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
    closest = np.where(region_bc[..., None], second + t_bc[..., None] * (third - second),
                       closest)
    closest = np.where(region_ac[..., None], first + t_ac[..., None] * edge_two, closest)
    closest = np.where(region_ab[..., None], first + t_ab[..., None] * edge_one, closest)
    closest = np.where(((d6 >= 0) & (d5 <= d6))[..., None], third, closest)
    closest = np.where(((d3 >= 0) & (d4 <= d3))[..., None], second, closest)
    closest = np.where(((d1 <= 0) & (d2 <= 0))[..., None], first, closest)
    return np.linalg.norm(query - closest, axis=2)


def distance_to_convex(points, vertices, faces, planes=None, chunk=2048):
    """Exact distance from each point to a convex solid; zero inside."""
    if planes is None:
        planes = hull_planes(vertices, faces)
    normals, offsets = planes
    result = np.zeros(len(points))
    triangles = vertices[faces]
    for start in range(0, len(points), chunk):
        block = points[start:start + chunk]
        outside = ~np.all(block @ normals.T - offsets[None, :] <= 0.0, axis=1)
        if outside.any():
            result[start:start + chunk][outside] = point_triangle_distance(
                block[outside], triangles).min(axis=1)
    return result


def bounding_capsule(vertices: np.ndarray):
    """
    Return the bounding capsule the broad phase uses.

    An axis, and the EXACT maximum vertex-to-segment distance about it.  The
    axis is the principal direction of the vertex cloud and the radius is a
    maximum over the actual vertices, so ``seg_seg(A, B) - r_a - r_b`` is a
    certified lower bound on the body-to-body distance whatever the axis is.
    A worse axis costs GJK calls; it can never make the bound unsound.
    """
    centre = vertices.mean(axis=0)
    centred = vertices - centre
    _, directions = np.linalg.eigh(centred.T @ centred)
    axis = directions[:, -1]
    # Fix the sign so the axis does not flip on numerically tied eigenvectors.
    dominant = int(np.argmax(np.abs(axis)))
    if axis[dominant] < 0.0:
        axis = -axis
    projection = centred @ axis
    point_a = centre + axis * projection.min()
    point_b = centre + axis * projection.max()
    span = point_b - point_a
    length_squared = float(span @ span)
    if length_squared <= 0.0:
        radius = float(np.linalg.norm(vertices - point_a, axis=1).max())
        return point_a, point_b, radius
    parameter = np.clip(((vertices - point_a) @ span) / length_squared, 0.0, 1.0)
    foot = point_a + parameter[:, None] * span
    radius = float(np.linalg.norm(vertices - foot, axis=1).max())
    return point_a, point_b, radius


# ---------------------------------------------------------------------------
# The MuJoCo collision inventory
# ---------------------------------------------------------------------------

def _default_tree(root):
    """Return {class name: (parent class, attributes of its <geom>)}."""
    classes = {}

    def walk(element, parent):
        name = element.get('class')
        if name is not None:
            geom = element.find('geom')
            classes[name] = (parent, dict(geom.attrib) if geom is not None else {})
            parent = name
        for child in element.findall('default'):
            walk(child, parent)

    for block in root.findall('default'):
        walk(block, None)
    return classes


def _resolves_to_collision(name, classes):
    seen = set()
    while name is not None:
        if name == 'collision':
            return True
        if name in seen:
            return False
        seen.add(name)
        name = classes.get(name, (None, {}))[0]
    return False


def collision_inventory(mujoco_path: Path):
    """
    Enumerate the collision geoms of ``mj_dual.xml``, refusing what it cannot read.

    Returns ``{link name: [asset file names]}`` for the arm links, and asserts
    the two inventory counts that have been miscounted before.
    """
    try:
        root = ElementTree.parse(str(mujoco_path)).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise GenerationError(
            'unable to read the MuJoCo model {}'.format(mujoco_path)) from error
    classes = _default_tree(root)
    for name in IGNORED_COLLISION_CLASSES:
        if name not in classes:
            raise GenerationError(
                'mj_dual.xml no longer declares the class {!r}; the fingertip-pad '
                'exemption of abort 1 names a class that is gone'.format(name))
        if not _resolves_to_collision(name, classes):
            raise GenerationError(
                '{!r} no longer resolves to class "collision"; abort 1 is scoped '
                'against a class tree that has changed'.format(name))
    assets = {}
    for asset in root.iter('mesh'):
        filename = asset.get('file')
        if filename is None:
            continue
        assets[asset.get('name') or Path(filename).stem] = filename

    mesh_geoms = 0
    fingertip_geoms = 0
    per_link = {}
    for body in root.iter('body'):
        name = body.get('name') or ''
        for geom in body.findall('geom'):
            geom_class = geom.get('class')
            if geom_class is None or not _resolves_to_collision(geom_class, classes):
                continue
            if geom_class in IGNORED_COLLISION_CLASSES:
                fingertip_geoms += 1
                continue
            mesh = geom.get('mesh')
            if mesh is None:
                raise GenerationError(
                    'a collision geom in body {!r} of class {!r} names no mesh and is '
                    'not one of the declared fingertip-pad classes'.format(
                        name, geom_class))
            mesh_geoms += 1
            # ABORT 1, scoped exactly.  Of the mesh-typed collision geoms whose
            # class resolves to `collision` itself, none carries a pos, quat or
            # euler - the asset-to-link transform is the identity, and the whole
            # model rests on that.  It is NOT true of the class tree: the five
            # fingertip-pad subclasses are nested inside <default
            # class="collision"> and each carries a pos, which is why they are
            # named in IGNORED_COLLISION_CLASSES rather than silently skipped.
            for attribute in ('pos', 'quat', 'euler'):
                if attribute in geom.attrib:
                    raise GenerationError(
                        'the collision geom {!r} in body {!r} carries {}={!r}; every '
                        'body of this model is expressed in its own link frame and a '
                        'geom transform would silently move it'.format(
                            mesh, name, attribute, geom.attrib[attribute]))
            # ABORT 2.
            if mesh not in assets:
                raise GenerationError(
                    'the collision geom {!r} in body {!r} names a mesh absent from the '
                    '<asset> block'.format(mesh, name))
            if mesh in IGNORED_COLLISION_GEOMS:
                continue
            match = None
            for link in ARM_LINKS:
                if name.endswith('_' + link):
                    match = link
            if match is None:
                raise GenerationError(
                    'the collision geom {!r} sits in body {!r}, which is not one of the '
                    'arm links {} and is not on the ignored list; a geom that is skipped '
                    'without a written reason is a collision body going missing'.format(
                        mesh, name, list(ARM_LINKS)))
            per_link.setdefault(match, [])
            if assets[mesh] not in per_link[match]:
                per_link[match].append(assets[mesh])

    if mesh_geoms != EXPECTED_MESH_COLLISION_GEOMS:
        raise GenerationError(
            'mj_dual.xml carries {} mesh-typed collision geoms, not the {} this '
            'generator has read'.format(mesh_geoms, EXPECTED_MESH_COLLISION_GEOMS))
    if fingertip_geoms != EXPECTED_FINGERTIP_PAD_GEOMS:
        raise GenerationError(
            'mj_dual.xml carries {} fingertip-pad collision geoms, not the {} this '
            'generator has read'.format(fingertip_geoms, EXPECTED_FINGERTIP_PAD_GEOMS))
    missing = [link for link in ARM_LINKS if link not in per_link]
    if missing:
        raise GenerationError(
            'no collision geom was found for {}'.format(missing))
    return per_link


# ---------------------------------------------------------------------------
# The flange
# ---------------------------------------------------------------------------

def _rotation_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def flange_primitives(safety_distance: float):
    """
    Return the three ``link8`` collision primitives, at METAL radius.

    ``robots/common/panda_arm.xacro`` declares them at
    ``${0.03 + safety_distance}``; the recovered ``safety_distance`` is
    subtracted here, exactly as CONTRACT B's ``_recovered_inflation`` does for
    every capsule, so no pair rides a 30 mm policy.
    """
    if abs(safety_distance) > 1.0:
        raise GenerationError(
            'the recovered safety_distance {} is not a plausible inflation'.format(
                safety_distance))
    rotation = _rotation_from_rpy(math.pi, math.pi / 2.0, math.pi / 2.0)
    return {
        'radius': FLANGE_METAL_RADIUS,
        'sphere_a': np.array([0.0424, 0.0424, -0.02]),
        'sphere_b': np.array([0.0424, 0.0424, -0.03]),
        'cylinder_centre': np.array([0.0424, 0.0424, -0.025]),
        'cylinder_axis': rotation @ np.array([0.0, 0.0, 1.0]),
        'cylinder_frame': rotation,
        'cylinder_half_length': 0.005,
    }


def flange_support(primitives, direction: np.ndarray) -> float:
    """
    Return the closed-form support of the two spheres and the cylinder.

    The union is taken at metal radius.  A convex polytope with outward face
    planes ``(n_i, d_i)`` contains a convex set ``P`` if and only if
    ``h_P(n_i) <= d_i`` for every face.  Having this in
    closed form is what makes the flange's containment a CERTIFICATE rather than
    a sample: no lattice resolution can hide a violation from it.
    """
    length = float(np.linalg.norm(direction))
    radius = primitives['radius']
    best = max(float(primitives['sphere_a'] @ direction),
               float(primitives['sphere_b'] @ direction)) + radius * length
    axial = float(primitives['cylinder_axis'] @ direction)
    radial = max(length * length - axial * axial, 0.0)
    cylinder = (float(primitives['cylinder_centre'] @ direction)
                + primitives['cylinder_half_length'] * abs(axial)
                + radius * math.sqrt(radial))
    return max(best, cylinder)


def flange_lattice(primitives, scale: float, rim: int, lattice):
    """Sample the three primitives at ``metal radius * scale``."""
    radius = primitives['radius'] * scale
    points = []
    angles = np.linspace(0.0, 2.0 * math.pi, rim, endpoint=False)
    for offset in (-primitives['cylinder_half_length'],
                   primitives['cylinder_half_length']):
        local = np.column_stack([radius * np.cos(angles), radius * np.sin(angles),
                                 np.full_like(angles, offset)])
        points.append((primitives['cylinder_frame'] @ local.T).T
                      + primitives['cylinder_centre'])
    polar = np.linspace(0.0, math.pi, lattice[0])
    azimuth = np.linspace(0.0, 2.0 * math.pi, lattice[1], endpoint=False)
    grid_polar, grid_azimuth = np.meshgrid(polar, azimuth, indexing='ij')
    sphere = np.column_stack([
        (radius * np.sin(grid_polar) * np.cos(grid_azimuth)).ravel(),
        (radius * np.sin(grid_polar) * np.sin(grid_azimuth)).ravel(),
        (radius * np.cos(grid_polar)).ravel(),
    ])
    points.append(sphere + primitives['sphere_a'])
    points.append(sphere + primitives['sphere_b'])
    return np.vstack(points)


def build_flange(safety_distance: float):
    """
    Build ``link8_flange`` and its containment certificate, or abort.

    ABORT 9: every face plane of the emitted hull must satisfy
    ``h_union(n_i) <= d_i``.  A hull of points sampled ON a sphere is INSCRIBED
    in it, so at scale 1.0 all 920 faces are violated and the spheres protrude
    0.3068 mm - optimism, on the one body in the model with no mesh behind it,
    touching six of the fifteen enabled self pairs.  Rule 6 certifies shell
    triangles and M-8 guards against building the flange too LARGE; neither
    catches building it too small, which is the unsafe direction.
    """
    from scipy.spatial import ConvexHull

    primitives = flange_primitives(safety_distance)
    points = flange_lattice(primitives, FLANGE_LATTICE_SCALE,
                            FLANGE_CYLINDER_RIM_SAMPLES, FLANGE_SPHERE_LATTICE)
    hull = ConvexHull(points)
    vertices = points[hull.vertices]
    hull = ConvexHull(vertices)
    normals = hull.equations[:, :3]
    offsets = -hull.equations[:, 3]
    slack = np.array([offsets[index] - flange_support(primitives, normals[index])
                      for index in range(len(normals))])
    violated = int((slack < 0.0).sum())
    if violated:
        raise GenerationError(
            'the link8_flange containment certificate FAILS: {} of {} face planes are '
            'violated by the primitive union, worst by {:.6f} mm. The hull must '
            'CIRCUMSCRIBE the primitives; an inscribed hull under-reports the flange '
            'and is optimism on the one body with no mesh behind it'.format(
                violated, len(normals), -float(slack.min()) * 1000.0))

    # The dense re-check, at a finer resolution than the emitting lattice.
    dense = flange_lattice(primitives, 1.0, FLANGE_CERTIFICATE_RIM[0],
                           FLANGE_CERTIFICATE_LATTICE)
    axis = primitives['cylinder_axis']
    frame = primitives['cylinder_frame']
    angles = np.linspace(0.0, 2.0 * math.pi, FLANGE_CERTIFICATE_RIM[0], endpoint=False)
    walls = []
    for offset in np.linspace(-primitives['cylinder_half_length'],
                              primitives['cylinder_half_length'],
                              FLANGE_CERTIFICATE_RIM[1]):
        local = np.column_stack([
            primitives['radius'] * np.cos(angles),
            primitives['radius'] * np.sin(angles),
            np.full_like(angles, offset)])
        walls.append((frame @ local.T).T + primitives['cylinder_centre'])
    dense = np.vstack([dense] + walls)
    protrusion = float((dense @ normals.T - offsets[None, :]).max())
    if protrusion > 0.0:
        raise GenerationError(
            'the link8_flange dense containment re-check FAILS by {:.6f} mm'.format(
                protrusion * 1000.0))
    del axis
    return {
        'vertices': vertices,
        'volume': float(hull.volume),
        'face_count': len(hull.simplices),
        'min_face_slack_m': float(slack.min()),
        'max_face_slack_m': float(slack.max()),
        'worst_primitive_protrusion_m': max(0.0, protrusion),
    }


# ---------------------------------------------------------------------------
# The shell-tight bodies
# ---------------------------------------------------------------------------

def build_link_bodies(link: str, pieces, shell):
    """
    Build one link's bodies, and return the per-link shell undercut with them.

    ABORT 6 is discharged by CONSTRUCTION: each shell triangle is assigned as a
    whole and all three of its vertices join the generating point set of the
    body it was assigned to, so the triangle lies in that body's hull.  The
    assertion below is that the assignment really did put every triangle's three
    vertices in one body's generator set - the property the proof rests on.
    """
    from scipy.spatial import ConvexHull

    shell_points, shell_faces = shell
    per_piece = np.column_stack([
        distance_to_convex(shell_points, vertices, faces)
        for _, vertices, faces in pieces])
    undercut = float(per_piece.min(axis=1).max())
    triangle_cost = np.maximum(
        np.maximum(per_piece[shell_faces[:, 0]], per_piece[shell_faces[:, 1]]),
        per_piece[shell_faces[:, 2]])
    owner = np.argmin(triangle_cost, axis=1)
    bodies = []
    for index, (filename, vertices, faces) in enumerate(pieces):
        assigned = np.unique(shell_faces[owner == index].ravel())
        generators = np.vstack([vertices, shell_points[assigned]])
        hull = ConvexHull(generators)
        body_vertices = generators[hull.vertices]
        hull = ConvexHull(body_vertices)
        piece_volume = float(ConvexHull(vertices).volume)
        # ABORT 7, on the SOURCE asset: a collision piece that is no longer
        # convex means the description grew geometry this model has not seen.
        normals, offsets = hull_planes(vertices, faces)
        defect = float((vertices @ normals.T - offsets[None, :]).max())
        if defect > CONVEXITY_TOLERANCE_M:
            raise GenerationError(
                'the source asset {} has a convexity defect of {:.9f} m, outside the '
                '{} m drift gate'.format(filename, defect, CONVEXITY_TOLERANCE_M))
        point_a, point_b, radius = bounding_capsule(body_vertices)
        bodies.append({
            'id': Path(filename).stem + '_st',
            'link': link,
            'body_source': 'collision_mesh_and_shell',
            'source_piece': filename,
            'vertices': body_vertices,
            'face_count': len(hull.simplices),
            'volume': float(hull.volume),
            'volume_ratio_to_piece': float(hull.volume) / piece_volume,
            'shell_undercut_m': undercut,
            'convexity_defect_m': defect,
            'capsule': (point_a, point_b, radius),
            'assigned_triangles': int((owner == index).sum()),
        })
    covered = sum(body['assigned_triangles'] for body in bodies)
    if covered != len(shell_faces):
        raise GenerationError(
            '{}: the triangle assignment covers {} of {} shell triangles; the '
            'containment proof requires every triangle to belong to exactly '
            'one body'.format(link, covered, len(shell_faces)))
    return bodies, undercut


# ---------------------------------------------------------------------------
# Emission
# ---------------------------------------------------------------------------

def _format_float(value: float) -> str:
    rounded = round(float(value), EMITTED_DECIMALS)
    if rounded == 0.0:
        rounded = 0.0
    text = '{:.{}f}'.format(rounded, EMITTED_DECIMALS).rstrip('0')
    if text.endswith('.'):
        text += '0'
    return text


def _quote(value: str) -> str:
    if any(character in value for character in '"\\\n'):
        raise GenerationError('cannot emit the string {!r}'.format(value))
    return '"{}"'.format(value)


def _sorted_vertices(vertices: np.ndarray) -> np.ndarray:
    """
    Descending lexicographic on (z, y, x) at the emitted precision.

    The same ordering CONTRACT B applies to capsule endpoints, for the same
    reason: the file must be byte-comparable between two runs and between two
    machines, and a hull library's vertex order is not a stable quantity.
    """
    rounded = np.round(vertices, EMITTED_DECIMALS) + 0.0
    order = np.lexsort((-rounded[:, 0], -rounded[:, 1], -rounded[:, 2]))
    return rounded[order]


def build_document(repository_root: Path, urdf_sha256: str,
                   safety_distance: float) -> str:
    """Build the deterministic YAML text of mesh_bodies_v1.yaml."""
    repository_root = Path(repository_root)
    mujoco_path = repository_root / MUJOCO_MODEL_RELATIVE
    assets_directory = mujoco_path.parent / 'assets'
    visual_directory = repository_root / VISUAL_MESH_DIRECTORY
    per_link = collision_inventory(mujoco_path)

    collision_digests = {}
    visual_digests = {}
    bodies = []
    for link in ARM_LINKS:
        pieces = []
        for filename in per_link[link]:
            path = assets_directory / filename
            if not path.is_file():
                raise GenerationError(
                    'the collision asset {} named by mj_dual.xml is missing'.format(path))
            collision_digests[filename] = sha256_of_file(path)
            vertices, faces = (read_stl(path) if filename.endswith('.stl')
                               else read_obj(path))
            pieces.append((filename, vertices, faces))
        shell_path = visual_directory / '{}.dae'.format(link)
        if not shell_path.is_file():
            # ABORT 5 for a link that HAS a collision mesh but no shell: the
            # metal proxy is the union of the two and half of it is missing.
            raise GenerationError(
                'the visual shell {} is missing; the metal proxy is the union of the '
                'collision solid and the shell, and a body built without the shell is '
                'optimistic by up to 6.5 mm by construction'.format(shell_path))
        visual_digests['{}.dae'.format(link)] = sha256_of_file(shell_path)
        shell = read_dae(shell_path)
        link_bodies, _ = build_link_bodies(link, pieces, shell)
        bodies.extend(link_bodies)

    # ABORT 5: a link with no collision mesh must be on the written list.
    flange = build_flange(safety_distance)
    point_a, point_b, radius = bounding_capsule(flange['vertices'])
    primitives = flange_primitives(safety_distance)
    bodies.append({
        'id': 'link8_flange',
        'link': 'link8',
        'body_source': 'urdf_primitive_at_metal_radius',
        'source_piece': None,
        'vertices': flange['vertices'],
        'face_count': flange['face_count'],
        'volume': flange['volume'],
        'volume_ratio_to_piece': None,
        'shell_undercut_m': None,
        'convexity_defect_m': 0.0,
        'capsule': (point_a, point_b, radius),
        'certificate': flange,
        'primitives': primitives,
    })

    lines = []
    lines.append('schema_version: 1')
    lines.append('source:')
    lines.append('  generator_version: {}'.format(GENERATOR_VERSION))
    lines.append('  urdf_sha256: {}'.format(_quote(urdf_sha256)))
    lines.append('  mujoco_model: {}'.format(MUJOCO_MODEL_RELATIVE))
    lines.append('  mujoco_model_sha256: {}'.format(_quote(sha256_of_file(mujoco_path))))
    lines.append('  collision_meshes:')
    for name in sorted(collision_digests):
        lines.append('    {}: {}'.format(name, _quote(collision_digests[name])))
    lines.append('  visual_meshes:')
    for name in sorted(visual_digests):
        lines.append('    {}: {}'.format(name, _quote(visual_digests[name])))
    lines.append('  metal_definition: collision_union_visual_shell')
    lines.append('  assignment: whole_triangle_nearest_piece')
    lines.append('  convexity_tolerance_m: {}'.format(
        _format_float(CONVEXITY_TOLERANCE_M)))
    lines.append('  emitted_decimals: {}'.format(EMITTED_DECIMALS))
    lines.append('  links_without_collision_mesh:')
    lines.append('    - link: link8')
    lines.append('      reason: {}'.format(_quote(LINK8_REASON)))
    lines.append('bodies:')
    for body in bodies:
        lines.append('  - id: {}'.format(body['id']))
        lines.append('    link: {}'.format(body['link']))
        lines.append('    body_source: {}'.format(body['body_source']))
        if body['source_piece'] is not None:
            lines.append('    source_piece: {}'.format(body['source_piece']))
            lines.append('    volume_ratio_to_piece: {}'.format(
                '{:.4f}'.format(body['volume_ratio_to_piece'])))
            lines.append('    shell_undercut_m: {}'.format(
                _format_float(body['shell_undercut_m'])))
        else:
            lines.append('    derivation:')
            lines.append('      primitives:')
            lines.append(
                '        - {{kind: cylinder, radius: {}, length: {}, '
                'origin_xyz: [0.0424, 0.0424, -0.025], '
                'origin_rpy_pi_multiples: [1.0, 0.5, 0.5]}}'.format(
                    _format_float(body['primitives']['radius']),
                    _format_float(2.0 * body['primitives']['cylinder_half_length'])))
            lines.append(
                '        - {{kind: sphere, radius: {}, '
                'origin_xyz: [0.0424, 0.0424, -0.02]}}'.format(
                    _format_float(body['primitives']['radius'])))
            lines.append(
                '        - {{kind: sphere, radius: {}, '
                'origin_xyz: [0.0424, 0.0424, -0.03]}}'.format(
                    _format_float(body['primitives']['radius'])))
            lines.append('      recovered_safety_distance: {}'.format(
                _format_float(safety_distance)))
            lines.append('      cylinder_rim_samples: {}'.format(
                FLANGE_CYLINDER_RIM_SAMPLES))
            lines.append('      sphere_lattice: [{}, {}]'.format(*FLANGE_SPHERE_LATTICE))
            lines.append('      lattice_scale: {}'.format(
                _format_float(FLANGE_LATTICE_SCALE)))
            lines.append('      minimum_containing_scale: {}'.format(
                _format_float(FLANGE_MINIMUM_CONTAINING_SCALE)))
            lines.append('      containment_certificate:')
            lines.append('        faces_violated: 0')
            lines.append('        min_face_slack_m: {}'.format(
                _format_float(body['certificate']['min_face_slack_m'])))
            lines.append('        max_face_slack_m: {}'.format(
                _format_float(body['certificate']['max_face_slack_m'])))
            lines.append('        worst_primitive_protrusion_m: {}'.format(
                _format_float(body['certificate']['worst_primitive_protrusion_m'])))
        lines.append('    volume_m3: {}'.format(_format_float(body['volume'])))
        lines.append('    face_count: {}'.format(body['face_count']))
        lines.append('    convexity_defect_m: {}'.format(
            _format_float(body['convexity_defect_m'])))
        capsule_a, capsule_b, capsule_r = body['capsule']
        lines.append(
            '    bounding_capsule: {{a: [{}], b: [{}], r: {}}}'.format(
                ', '.join(_format_float(value) for value in capsule_a),
                ', '.join(_format_float(value) for value in capsule_b),
                _format_float(capsule_r)))
        lines.append('    vertices:')
        for vertex in _sorted_vertices(body['vertices']):
            lines.append('      - [{}, {}, {}]'.format(
                _format_float(vertex[0]), _format_float(vertex[1]),
                _format_float(vertex[2])))
    text = '\n'.join(lines) + '\n'
    _check_artefact_bounds(text)
    return text


def _check_artefact_bounds(text: str) -> None:
    """
    ABORT 8: all three caps, not only bytes.

    The vertex payload is 34 779 numeric scalars, seventeen times strictyaml's
    2 048, so a reader built on the cell files' constants refuses this artefact
    long before any byte bound is reached.  The emission style is part of the
    contract for the same reason: one flow-sequence line per vertex measures
    677 097 bytes at 12 decimals, while a fully block-style emission measures
    978 515 and would fail this abort by 192 KB.
    """
    import yaml

    encoded = text.encode('utf-8')
    if len(encoded) > MAXIMUM_MESH_BODIES_BYTES:
        raise GenerationError(
            'the emitted artefact is {} bytes, outside MAXIMUM_MESH_BODIES_BYTES '
            '({})'.format(len(encoded), MAXIMUM_MESH_BODIES_BYTES))
    depth = 0
    worst = 0
    scalars = 0
    for event in yaml.parse(text):
        if isinstance(event, yaml.events.ScalarEvent):
            scalars += 1
        elif isinstance(event, (yaml.events.MappingStartEvent,
                                yaml.events.SequenceStartEvent)):
            depth += 1
            worst = max(worst, depth)
        elif isinstance(event, (yaml.events.MappingEndEvent,
                                yaml.events.SequenceEndEvent)):
            depth -= 1
    if scalars > MAXIMUM_MESH_BODIES_SCALARS:
        raise GenerationError(
            'the emitted artefact carries {} YAML scalars, outside '
            'MAXIMUM_MESH_BODIES_SCALARS ({})'.format(
                scalars, MAXIMUM_MESH_BODIES_SCALARS))
    if worst > MAXIMUM_MESH_BODIES_DEPTH:
        raise GenerationError(
            'the emitted artefact nests {} deep, outside MAXIMUM_MESH_BODIES_DEPTH '
            '({})'.format(worst, MAXIMUM_MESH_BODIES_DEPTH))


def generate(repository_root: Path, urdf_sha256: str, safety_distance: float) -> str:
    """Return the mesh_bodies_v1.yaml text for this description."""
    return build_document(repository_root, urdf_sha256, safety_distance)


def main(argv=None) -> int:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(
        description='Generate mesh_bodies_v1.yaml from franka_description')
    parser.add_argument('--repository-root', type=Path, required=True)
    parser.add_argument('--link-geometry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args(argv)
    import yaml

    geometry = yaml.safe_load(arguments.link_geometry.read_text(encoding='utf-8'))
    try:
        text = generate(arguments.repository_root,
                        geometry['source']['urdf_sha256'],
                        float(geometry['source']['safety_distance']))
    except GenerationError as error:
        print('generation aborted: {}'.format(error), file=sys.stderr)
        return 2
    arguments.output.write_text(text, encoding='utf-8')
    print('wrote {} ({} bytes)'.format(arguments.output,
                                       len(text.encode('utf-8'))), file=sys.stderr)
    return 0


if __name__ == '__main__':
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    raise SystemExit(main())
