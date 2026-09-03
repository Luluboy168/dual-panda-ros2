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
Read ``cell/mesh_bodies_v1.yaml``: the second pinned artefact, with its own caps.

WHY IT NEEDS ITS OWN CAPS.  ``strictyaml`` pins ``MAXIMUM_MODEL_BYTES = 65_536``,
``MAXIMUM_YAML_DEPTH = 8`` and ``MAXIMUM_YAML_SCALARS = 2_048``, and those stay
in force for ``cell_model_v1.yaml`` and ``link_geometry_v1.yaml``.  This artefact
carries 11 593 vertices, which is **34 779 numeric scalars** - seventeen times
the scalar cap - so a reader built on the cell files' constants refuses it long
before any byte bound is reached.  The byte cap was never going to be the
binding one, and all three are therefore named, measured and asserted here.

A MISMATCH IS A LOAD FAILURE, NEVER A FALLBACK.  If this artefact does not match
what the cell file pins, the model does not load.  Falling back to capsules
would mean an asset problem silently produces the LOOSER fence - which is the
one failure mode this whole line of work exists to prevent.
"""

from dataclasses import dataclass
import os
from pathlib import Path
import stat

import numpy as np

from ..strictyaml import exact_keys, WorkspaceModelError


#: Measured payload 545 223 bytes in the pinned emission style at 12 decimals,
#: 69 % of this cap.  The style is part of the artefact contract, not a
#: preference: one flow-sequence line per vertex against a fully block-style
#: emission is worth about 300 KB, which is more than the whole margin.
MAXIMUM_MESH_BODIES_BYTES = 786432
#: Measured 34 779 vertex scalars plus the source, derivation and per-body
#: metadata.
MAXIMUM_MESH_BODIES_SCALARS = 40000
#: Measured maximum nesting 7 (root -> bodies -> body -> derivation ->
#: primitives -> item -> origin_xyz).  The same value strictyaml already uses,
#: so nothing new is permitted.
MAXIMUM_MESH_BODIES_DEPTH = 8

SCHEMA_VERSION = 1
#: The generator version whose output this reader understands.
MINIMUM_GENERATOR_VERSION = 1

TOP_KEYS = ('schema_version', 'source', 'bodies')
SOURCE_KEYS = ('generator_version', 'urdf_sha256', 'mujoco_model',
               'mujoco_model_sha256', 'collision_meshes', 'visual_meshes',
               'metal_definition', 'assignment', 'convexity_tolerance_m',
               'emitted_decimals', 'links_without_collision_mesh')
MESH_BODY_KEYS = ('id', 'link', 'body_source', 'source_piece',
                  'volume_ratio_to_piece', 'shell_undercut_m', 'volume_m3',
                  'face_count', 'convexity_defect_m', 'bounding_capsule', 'vertices')
FLANGE_BODY_KEYS = ('id', 'link', 'body_source', 'derivation', 'volume_m3',
                    'face_count', 'convexity_defect_m', 'bounding_capsule', 'vertices')
BODY_SOURCES = ('collision_mesh_and_shell', 'urdf_primitive_at_metal_radius')
#: A body that carried either of these would be claiming there is something to
#: add back.  There is not, and the message says why.
FORBIDDEN_BODY_KEYS = ('inflation', 'coverage')


@dataclass(frozen=True)
class MeshBody:
    """One convex body of the metal proxy, in its own link frame."""

    id: str  # noqa: A003 - the field name is part of the published surface
    link: str
    body_source: str
    vertices: np.ndarray
    capsule_a: np.ndarray
    capsule_b: np.ndarray
    capsule_radius: float
    volume_m3: float
    shell_undercut_m: float


class MeshBodySet:
    """The parsed artefact: the bodies of ONE arm, plus what derived them."""

    def __init__(self, bodies, source, digest, byte_count, path, diagnostics=()):
        """Keep the parsed bodies and the provenance that produced them."""
        self.bodies = tuple(bodies)
        self.source = dict(source)
        self.sha256 = digest
        self.bytes = byte_count
        self.path = path
        self.diagnostics = tuple(diagnostics)
        self.by_link = {}
        for body in self.bodies:
            self.by_link.setdefault(body.link, []).append(body)
        self.by_link = {link: tuple(items) for link, items in self.by_link.items()}
        self.by_id = {body.id: body for body in self.bodies}

    def __len__(self):
        """Return the number of bodies, which is one arm's worth."""
        return len(self.bodies)


def read_bounded_mesh_bodies_text(path: Path) -> str:
    """Read the artefact under ITS OWN byte cap, refusing anything irregular."""
    descriptor = -1
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK)
        details = os.fstat(descriptor)
        if not stat.S_ISREG(details.st_mode):
            raise WorkspaceModelError('the mesh bodies artefact must be a regular file')
        if details.st_size > MAXIMUM_MESH_BODIES_BYTES:
            raise WorkspaceModelError(
                'the mesh bodies artefact is {} bytes, outside '
                'MAXIMUM_MESH_BODIES_BYTES ({})'.format(
                    details.st_size, MAXIMUM_MESH_BODIES_BYTES))
        chunks = []
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            chunks.append(block)
        data = b''.join(chunks)
        if len(data) > MAXIMUM_MESH_BODIES_BYTES:
            raise WorkspaceModelError(
                'the mesh bodies artefact exceeds MAXIMUM_MESH_BODIES_BYTES')
        return data.decode('utf-8')
    except WorkspaceModelError:
        raise
    except (OSError, UnicodeError) as error:
        raise WorkspaceModelError(
            'unable to read the mesh bodies artefact {}'.format(path)) from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _scan_bounds(text: str) -> None:
    """Apply the scalar and depth caps, on this artefact's own constants."""
    import yaml

    depth = 0
    worst = 0
    scalars = 0
    documents = 0
    try:
        for event in yaml.parse(text):
            if isinstance(event, yaml.events.DocumentStartEvent):
                documents += 1
                if documents > 1:
                    raise WorkspaceModelError(
                        'the mesh bodies artefact must be one YAML document')
            if isinstance(event, yaml.events.AliasEvent):
                raise WorkspaceModelError('YAML aliases are forbidden')
            if getattr(event, 'anchor', None) is not None:
                raise WorkspaceModelError('YAML anchors are forbidden')
            if getattr(event, 'tag', None) is not None:
                raise WorkspaceModelError('explicit YAML tags are forbidden')
            if isinstance(event, yaml.events.ScalarEvent):
                scalars += 1
                if scalars > MAXIMUM_MESH_BODIES_SCALARS:
                    raise WorkspaceModelError(
                        'the mesh bodies artefact carries more than '
                        'MAXIMUM_MESH_BODIES_SCALARS ({}) scalars'.format(
                            MAXIMUM_MESH_BODIES_SCALARS))
            elif isinstance(event, (yaml.events.MappingStartEvent,
                                    yaml.events.SequenceStartEvent)):
                depth += 1
                worst = max(worst, depth)
                if worst > MAXIMUM_MESH_BODIES_DEPTH:
                    raise WorkspaceModelError(
                        'the mesh bodies artefact nests deeper than '
                        'MAXIMUM_MESH_BODIES_DEPTH ({})'.format(
                            MAXIMUM_MESH_BODIES_DEPTH))
            elif isinstance(event, (yaml.events.MappingEndEvent,
                                    yaml.events.SequenceEndEvent)):
                depth -= 1
    except WorkspaceModelError:
        raise
    except yaml.YAMLError as error:
        raise WorkspaceModelError('the mesh bodies artefact is malformed YAML') from error
    if documents != 1 or depth != 0:
        raise WorkspaceModelError('the mesh bodies artefact must be one YAML document')


def _vertices(entry, context) -> np.ndarray:
    rows = entry['vertices']
    if not isinstance(rows, list) or len(rows) < 4:
        raise WorkspaceModelError(
            '{}: a body needs at least four vertices'.format(context))
    array = np.empty((len(rows), 3), dtype=float)
    for index, row in enumerate(rows):
        if (not isinstance(row, list) or len(row) != 3
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       for value in row)):
            raise WorkspaceModelError(
                '{}: vertex {} is not three numbers'.format(context, index))
        array[index] = row
    if not np.all(np.isfinite(array)):
        raise WorkspaceModelError('{}: vertices must be finite'.format(context))
    return array


def load_mesh_bodies(path, *, urdf_sha256=None, safety_distance=None) -> MeshBodySet:
    """
    Load and validate the mesh bodies, or raise.  There is no partial load.

    ``safety_distance`` is the inflation ``link_geometry_v1.yaml`` recovered from
    the description's own arm call sites.  The flange is the one body derived
    from URDF primitives rather than from a mesh, and it is emitted at the METAL
    radius - the declared radius with that inflation subtracted - so a
    disagreement means the flange carries an inflation the rest of the model
    does not.  That is mutation M-8, it is worth 30 mm of air that is not there
    on six of the sixteen enabled self pairs, and it is a LOAD FAILURE.

    ``urdf_sha256`` is the digest of the description the link geometry was
    generated from, and a mismatch is reported as a DIAGNOSTIC rather than
    refused.  The bodies are per-LINK and carry no arm prefix, and every input
    they are derived from - ``mj_dual.xml``, the ten collision assets, the eight
    shells, the recovered inflation - is the same in the single-arm description
    as in the dual one.  Refusing here would mean a single-arm session could not
    use bodies that are, link for link, the same geometry; saying nothing at all
    would hide a real provenance difference.  So it is said, once, at load.
    """
    import hashlib
    import yaml

    path = Path(path)
    text = read_bounded_mesh_bodies_text(path)
    _scan_bounds(text)
    digest = hashlib.sha256(text.encode('utf-8')).hexdigest()
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise WorkspaceModelError('the mesh bodies artefact is malformed YAML') from error
    if not isinstance(document, dict):
        raise WorkspaceModelError('the mesh bodies artefact must be a mapping')
    exact_keys(document, TOP_KEYS, 'mesh bodies')
    if document['schema_version'] != SCHEMA_VERSION:
        raise WorkspaceModelError(
            'mesh bodies schema_version must be {}, found {!r}'.format(
                SCHEMA_VERSION, document['schema_version']))
    source = exact_keys(document['source'], SOURCE_KEYS, 'mesh bodies source')
    version = source['generator_version']
    if isinstance(version, bool) or not isinstance(version, int) or (
            version < MINIMUM_GENERATOR_VERSION):
        raise WorkspaceModelError(
            'mesh bodies source.generator_version must be an integer of at least '
            '{}, found {!r}; regenerate the artefact'.format(
                MINIMUM_GENERATOR_VERSION, version))
    if source['metal_definition'] != 'collision_union_visual_shell':
        raise WorkspaceModelError(
            'mesh bodies source.metal_definition must be '
            'collision_union_visual_shell; a body built from the collision '
            'solid alone is optimistic by up to 6.5 mm by construction')
    if source['assignment'] != 'whole_triangle_nearest_piece':
        raise WorkspaceModelError(
            "mesh bodies source.assignment must be 'whole_triangle_nearest_piece'; "
            'assigning shell vertices instead of whole triangles stops the union '
            'containing the shell, measured at 1.1598 mm on link5')
    diagnostics = []
    if urdf_sha256 is not None and source['urdf_sha256'] != urdf_sha256:
        diagnostics.append(
            'the mesh bodies were derived from the description whose digest is {}, '
            'while the link geometry records {}. The bodies are per-link, carry no '
            'arm prefix, and depend only on the collision assets, the visual shells '
            'and the recovered inflation - all of which are shared between the '
            'single-arm and dual descriptions - so they are used as they are. This '
            'is reported so that the provenance difference is visible rather than '
            'inferred'.format(source['urdf_sha256'], urdf_sha256))

    entries = document['bodies']
    if not isinstance(entries, list) or not entries:
        raise WorkspaceModelError('mesh bodies must be a non-empty list')
    bodies = []
    seen = set()
    for index, entry in enumerate(entries):
        context = 'mesh body {}'.format(index)
        if not isinstance(entry, dict) or 'body_source' not in entry:
            raise WorkspaceModelError('{} needs a body_source'.format(context))
        for forbidden in FORBIDDEN_BODY_KEYS:
            if forbidden in entry:
                raise WorkspaceModelError(
                    '{}: a mesh body may not carry {!r}. A mesh body contains '
                    'the visual shell by construction; there is nothing to add back '
                    '- see doc/CONTRACT.md, the section on clearance being real '
                    'air'.format(context, forbidden))
        kind = entry['body_source']
        if kind not in BODY_SOURCES:
            raise WorkspaceModelError(
                '{}: unknown body_source {!r}'.format(context, kind))
        expected = (MESH_BODY_KEYS if kind == 'collision_mesh_and_shell'
                    else FLANGE_BODY_KEYS)
        exact_keys(entry, expected, context)
        identifier = entry['id']
        if not isinstance(identifier, str) or not identifier:
            raise WorkspaceModelError('{}: id must be a non-empty string'.format(context))
        if identifier in seen:
            raise WorkspaceModelError('duplicate mesh body id {!r}'.format(identifier))
        seen.add(identifier)
        capsule = exact_keys(entry['bounding_capsule'], ('a', 'b', 'r'),
                             '{} bounding_capsule'.format(context))
        vertices = _vertices(entry, context)
        radius = float(capsule['r'])
        point_a = np.array([float(value) for value in capsule['a']])
        point_b = np.array([float(value) for value in capsule['b']])
        if point_a.shape != (3,) or point_b.shape != (3,):
            raise WorkspaceModelError(
                '{}: bounding_capsule endpoints must be three numbers'.format(context))
        if not (radius > 0.0 and np.isfinite(radius)):
            raise WorkspaceModelError(
                '{}: the bounding-capsule radius must be positive'.format(context))
        bodies.append(MeshBody(
            id=identifier,
            link=entry['link'],
            body_source=kind,
            vertices=vertices,
            capsule_a=point_a,
            capsule_b=point_b,
            capsule_radius=radius,
            volume_m3=float(entry['volume_m3']),
            shell_undercut_m=(float(entry['shell_undercut_m'])
                              if kind == 'collision_mesh_and_shell' else 0.0),
        ))
    flange = [entry for entry in entries
              if entry['body_source'] == 'urdf_primitive_at_metal_radius']
    if len(flange) != 1:
        raise WorkspaceModelError(
            'exactly one body may be derived from URDF primitives; found {}'.format(
                len(flange)))
    recovered = float(flange[0]['derivation']['recovered_safety_distance'])
    if safety_distance is not None and abs(recovered - safety_distance) > 1e-12:
        raise WorkspaceModelError(
            'the link8 body was emitted with recovered_safety_distance {} but the '
            'link geometry recovered {} from the description; a flange built at the '
            'PADDED radius carries 30 mm of air that is not there, on six of the '
            'enabled self pairs, at the part of the arm most likely to be near '
            'something'.format(recovered, safety_distance))
    scale = float(flange[0]['derivation']['lattice_scale'])
    minimum = float(flange[0]['derivation']['minimum_containing_scale'])
    if scale < minimum:
        raise WorkspaceModelError(
            'the link8 lattice_scale {} is below the minimum containing scale {}; a '
            'hull of points sampled ON a sphere is INSCRIBED in it, and an inscribed '
            'flange under-reports its own declared geometry - optimism, on the one '
            'body in this model with no mesh behind it'.format(scale, minimum))
    certificate = flange[0]['derivation']['containment_certificate']
    if certificate['faces_violated'] != 0 or (
            float(certificate['worst_primitive_protrusion_m']) > 0.0):
        raise WorkspaceModelError(
            'the link8 containment certificate in the artefact records a violation; '
            'the body does not contain the primitives it claims to represent')

    listed = source['links_without_collision_mesh']
    if not isinstance(listed, list):
        raise WorkspaceModelError(
            'mesh bodies source.links_without_collision_mesh must be a list')
    for item in listed:
        item = exact_keys(item, ('link', 'reason'),
                          'mesh bodies links_without_collision_mesh entry')
        if not isinstance(item['reason'], str) or len(item['reason'].strip()) < 40:
            raise WorkspaceModelError(
                'links_without_collision_mesh entry {!r} carries no written '
                'reason'.format(item['link']))
    return MeshBodySet(bodies, source, digest, len(text.encode('utf-8')), path,
                       diagnostics)


def certified_lower_bounds(ends_a, ends_b, radii, first, second) -> np.ndarray:
    """
    Return ``seg_seg(A, B) - r_a - r_b`` for a batch of body pairs.

    That quantity is a CERTIFIED LOWER BOUND on ``gjk(body_a, body_b)`` and is
    never a reported clearance.  Each radius is the EXACT maximum
    vertex-to-segment distance of its own body, so every vertex of body ``a``
    lies within ``r_a`` of segment ``A`` and the inequality holds whatever the
    axis is.  ``r_a`` and ``r_b`` appear here and
    nowhere else; subtracting them from a reported clearance would be a double
    subtraction worth -107 mm on link5/link7 at the ready pose.
    """
    from ..geometry import segment_segment_distance_batch

    separation = segment_segment_distance_batch(
        ends_a[first], ends_b[first], ends_a[second], ends_b[second])
    return separation - radii[first] - radii[second]
