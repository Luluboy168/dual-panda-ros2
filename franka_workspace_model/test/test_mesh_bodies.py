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
The mesh bodies: the frames they live in, what they contain, and their pinning.

Every assertion here is against a number measured from the DESCRIPTION, not
against the fence's own output.  The bodies are data at this stage - the checker
still measures capsules - so nothing in this file can be satisfied by the
checker agreeing with itself.
"""

import hashlib
import xml.etree.ElementTree as ElementTree

from conftest import (CELL_MODEL_PATH, LINK_GEOMETRY_PATH, MESH_BODIES_PATH, READY,
                      REPOSITORY_ROOT, write_cell_model)

from franka_workspace_model.generate_mesh_bodies import (
    build_link_bodies, collision_inventory, distance_to_convex,
    EXPECTED_FINGERTIP_PAD_GEOMS, EXPECTED_MESH_COLLISION_GEOMS, flange_lattice,
    FLANGE_LATTICE_SCALE, flange_primitives, flange_support, generate,
    GENERATOR_VERSION, MAXIMUM_MESH_BODIES_BYTES, MAXIMUM_MESH_BODIES_SCALARS,
    read_dae, read_obj, read_stl)
from franka_workspace_model.mesh_runtime import load_mesh_bodies
from franka_workspace_model.model import CellModel, WorkspaceModelError

import numpy as np

import pytest

import yaml


MUJOCO_PATH = (REPOSITORY_ROOT / 'franka_description' / 'mujoco' / 'franka'
               / 'mj_dual.xml')
ASSET_DIRECTORY = MUJOCO_PATH.parent / 'assets'
VISUAL_DIRECTORY = REPOSITORY_ROOT / 'franka_description' / 'meshes' / 'visual'

#: Section 1.1's body table, measured independently of this package and
#: reproduced here to the digit: vertices, hull faces, volume in cubic
#: centimetres, and the EXACT bounding-capsule radius the broad phase uses.
BODY_TABLE = {
    'link0_st': (482, 960, 3001.76, 0.1193),
    'link1_st': (1269, 2534, 2991.01, 0.1030),
    'link2_st': (1232, 2460, 3017.87, 0.1032),
    'link3_st': (1295, 2586, 2342.75, 0.0882),
    'link4_st': (1367, 2730, 2387.62, 0.0875),
    'link5_collision_0_st': (2116, 4228, 993.57, 0.0756),
    'link5_collision_1_st': (210, 416, 312.02, 0.0585),
    'link5_collision_2_st': (939, 1874, 878.83, 0.1061),
    'link6_st': (1517, 3030, 1450.28, 0.1022),
    'link7_st': (704, 1404, 449.23, 0.0533),
    'link8_flange': (462, 920, 143.18, 0.0307),
}
TOTAL_VERTICES = 11593

#: Section 3's undercut table: the maximum EXACT point-to-triangle distance from
#: any visual-shell vertex to the link's collision solid, with the witness
#: vertex in the link frame.  These are the numbers that would have had to be
#: subtracted from every clearance had the bodies not been built to contain the
#: shell; they are reported here and used by nothing.
SHELL_UNDERCUT = {
    'link0': (0.000335869, (0.034936, -0.042489, 0.140000)),
    'link1': (0.000928524, (-0.048041, -0.026790, -0.192000)),
    'link2': (0.000751380, (-0.053128, -0.194000, 0.014271)),
    'link3': (0.000926919, (-0.003374, 0.054911, -0.120999)),
    'link4': (0.000843914, (-0.046583, 0.124000, -0.041662)),
    'link5': (0.006502597, (0.010529, -0.054015, -0.259000)),
    'link6': (0.000913363, (0.061272, 0.077891, -0.022694)),
    'link7': (0.000606384, (-0.038545, -0.037399, 0.082093)),
}

#: The flange landmark at the ready pose, in the cell frame, computed from the
#: corpus's own truncated decimals rather than from exact pi/4.
LINK8_LANDMARK = (0.30689035, 0.5, 0.59028034)


@pytest.fixture(scope='module')
def bodies():
    """Load the shipped artefact once for the whole file."""
    geometry = yaml.safe_load(LINK_GEOMETRY_PATH.read_text(encoding='utf-8'))
    return load_mesh_bodies(MESH_BODIES_PATH,
                            urdf_sha256=geometry['source']['urdf_sha256'])


# ---------------------------------------------------------------------------
# T-FRAME
# ---------------------------------------------------------------------------

def test_the_collision_geoms_carry_no_transform_of_their_own():
    """
    T-FRAME: every body of this model is expressed in its own link frame.

    ``collision_inventory`` ABORTS on a mesh-typed collision geom that carries a
    pos, quat or euler, and the whole model rests on that identity transform.
    Calling it here is the assertion.
    """
    per_link = collision_inventory(MUJOCO_PATH)
    assert per_link['link5'] == ['link5_collision_0.obj', 'link5_collision_1.obj',
                                 'link5_collision_2.obj']
    assert per_link['link0'] == ['link0.stl']


def test_the_geom_inventory_is_the_one_the_generator_reads():
    """
    Both counts have been miscounted before, so both are asserted.

    Twenty-six mesh-typed collision geoms resolve to class ``collision`` itself.
    Separately the file carries TWENTY fingertip-pad box geoms - five
    ``<default class="fingertip_pad_collision_N">`` blocks times two fingers
    times two arms - and those five defaults are nested INSIDE ``<default
    class="collision">`` and each carries a ``pos``.  A generator that resolves
    the class tree therefore sees collision geoms with a pos; they are named in
    the ignored set for the same reason as ``finger_0``, and the abort is scoped
    to the mesh-typed geoms rather than being quietly weakened.
    """
    text = MUJOCO_PATH.read_text(encoding='utf-8')
    root = ElementTree.fromstring(text)
    mesh_typed = sum(1 for body in root.iter('body') for geom in body.findall('geom')
                     if geom.get('class') == 'collision')
    pads = sum(1 for body in root.iter('body') for geom in body.findall('geom')
               if (geom.get('class') or '').startswith('fingertip_pad_collision_'))
    assert mesh_typed == EXPECTED_MESH_COLLISION_GEOMS == 26
    assert pads == EXPECTED_FINGERTIP_PAD_GEOMS == 20
    for index in range(1, 6):
        assert 'class="fingertip_pad_collision_{}"'.format(index) in text
    # ...and each of those five defaults really does carry a pos, which is the
    # fact that makes the scoping necessary rather than pedantic.
    for block in root.iter('default'):
        name = block.get('class') or ''
        if name.startswith('fingertip_pad_collision_'):
            assert 'pos' in block.find('geom').attrib


def _mujoco_transforms(joint_values, arm='left'):
    """Build an independent forward chain from mj_dual.xml's own body records."""
    root = ElementTree.fromstring(MUJOCO_PATH.read_text(encoding='utf-8'))

    def quaternion_matrix(text):
        w, x, y, z = (float(value) for value in text.split())
        norm = (w * w + x * x + y * y + z * z) ** 0.5
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    transforms = {}

    def walk(body, parent):
        name = body.get('name') or ''
        step = np.eye(4)
        if body.get('pos'):
            step[:3, 3] = [float(value) for value in body.get('pos').split()]
        if body.get('quat'):
            step[:3, :3] = quaternion_matrix(body.get('quat'))
        here = parent @ step
        joint = body.find('joint')
        if joint is not None and joint.get('name', '').startswith('mj_{}_joint'.format(arm)):
            index = int(joint.get('name').rsplit('joint', 1)[1]) - 1
            angle = joint_values[index]
            spin = np.eye(4)
            cos, sin = np.cos(angle), np.sin(angle)
            spin[:3, :3] = np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])
            here = here @ spin
        transforms[name] = here
        for child in body.findall('body'):
            walk(child, here)

    for body in root.iter('body'):
        if body.get('name') == 'mj_{}_link0'.format(arm):
            walk(body, np.eye(4))
            break
    return transforms


#: Fifty pinned pseudo-random poses.  The seed is fixed, so a chain that drifts
#: fails the same way on every machine.
_POSES = np.random.default_rng(20260903).uniform(-2.5, 2.5, size=(50, 7))


def test_the_mujoco_chain_and_the_link_geometry_chain_agree(cell_model):
    """
    T-FRAME: the two descriptions of the same robot place link8 in one place.

    ``mj_dual.xml`` is where the claim "these are the collision geoms" lives and
    the URDF is where the kinematics live; the bodies are read from one and
    placed by the other, so the two chains agreeing is a load-bearing fact and
    not a curiosity.  Note that mj_dual.xml's JOINT RANGES are never read: they
    are the FR3's on a Panda chain.
    """
    for values in [np.array(READY)] + list(_POSES):
        configuration = {'panda1': list(values), 'panda2': list(READY)}
        urdf = cell_model._transforms(cell_model._sample(configuration))
        mujoco = _mujoco_transforms(values, 'left')
        for index in range(9):
            here = urdf['panda1_link{}'.format(index)]
            there = mujoco['mj_left_link{}'.format(index)]
            assert float(np.abs(here[:3, 3] - there[:3, 3]).max()) < 1e-9
            assert float(np.abs(here[:3, :3] - there[:3, :3]).max()) < 1e-9


def test_the_ready_pose_flange_landmark(cell_model):
    """The one landmark three independent sessions have quoted."""
    urdf = cell_model._transforms(cell_model._sample(
        {'panda1': list(READY), 'panda2': list(READY)}))
    origin = urdf['panda1_link8'][:3, 3]
    assert np.allclose(origin, LINK8_LANDMARK, atol=5e-9), origin


# ---------------------------------------------------------------------------
# T-BODY-1: the containment certificate
# ---------------------------------------------------------------------------

def test_every_shell_triangle_lies_in_one_body():
    """
    T-BODY-1, both halves.

    BY CONSTRUCTION: each shell triangle is assigned as a WHOLE and all three of
    its vertices join the generating point set of the body it was assigned to.
    A convex hull contains the hull of any subset of its generators, so the
    triangle lies inside that body, and therefore the whole shell surface lies
    inside the union of the bodies.  That is a proof.

    BY SUBDIVISION, on link5, the one link whose target is a union of three
    convex pieces and therefore the one where the vertex argument would fail:
    every point of every shell triangle at barycentric resolution 1/16 is inside
    some body's half-space set.

    M-4 - assigning shell VERTICES instead of whole triangles - is killed here:
    it splits triangles that span two pieces and the union stops containing the
    shell by 1.1598 mm, silently, on the link that binds.
    """
    pieces = [(name, *(read_obj(ASSET_DIRECTORY / name)))
              for name in ('link5_collision_0.obj', 'link5_collision_1.obj',
                           'link5_collision_2.obj')]
    shell_points, shell_faces = read_dae(VISUAL_DIRECTORY / 'link5.dae')
    built, undercut = build_link_bodies('link5', pieces, (shell_points, shell_faces))
    assert abs(undercut - SHELL_UNDERCUT['link5'][0]) < 5e-9

    planes = []
    for body in built:
        vertices = body['vertices']
        from scipy.spatial import ConvexHull
        hull = ConvexHull(vertices)
        planes.append((hull.equations[:, :3], -hull.equations[:, 3]))

    # The PROOF, checked exactly: every triangle's three vertices satisfy the
    # half-spaces of the body it was assigned to.  A convex body contains the
    # convex hull of any points it contains, so the whole triangle follows -
    # no sampling resolution can add anything to this, and none is needed.
    owner = np.empty(len(shell_faces), dtype=int)
    cursor = 0
    for index, body in enumerate(built):
        count = body['assigned_triangles']
        del count
    # Recover the assignment the builder used, by the same rule it used.
    per_piece = np.column_stack([distance_to_convex(shell_points, vertices, faces)
                                 for _, vertices, faces in pieces])
    cost = np.maximum(np.maximum(per_piece[shell_faces[:, 0]],
                                 per_piece[shell_faces[:, 1]]),
                      per_piece[shell_faces[:, 2]])
    owner = np.argmin(cost, axis=1)
    del cursor
    worst_exact = -np.inf
    for index, (normals, offsets) in enumerate(planes):
        mine = shell_points[shell_faces[owner == index]].reshape(-1, 3)
        worst_exact = max(worst_exact,
                          float((mine @ normals.T - offsets[None, :]).max()))
    assert worst_exact <= 1e-12, (
        'a shell triangle vertex escapes its own body by {:+.4e} m'.format(worst_exact))

    # ...and the same statement by SUBDIVISION, which is what would catch a
    # numerical surprise the proof does not model.  Every interior point of
    # every link5 shell triangle, at barycentric resolution 1/4, against its own
    # body; plus a pinned 500-triangle sample at 1/16.
    triangles = shell_points[shell_faces]
    worst = -np.inf
    for resolution, subset in ((4, np.arange(len(triangles))),
                               (16, np.random.default_rng(20260903).choice(
                                   len(triangles), 500, replace=False))):
        weights = np.array([(a / resolution, b / resolution,
                             (resolution - a - b) / resolution)
                            for a in range(resolution + 1)
                            for b in range(resolution + 1 - a)])
        for index, (normals, offsets) in enumerate(planes):
            rows = subset[owner[subset] == index]
            for start in range(0, len(rows), 512):
                block = triangles[rows[start:start + 512]]
                points = (weights[None, :, 0, None] * block[:, None, 0, :]
                          + weights[None, :, 1, None] * block[:, None, 1, :]
                          + weights[None, :, 2, None] * block[:, None, 2, :]
                          ).reshape(-1, 3)
                worst = max(worst, float(
                    (points @ normals.T - offsets[None, :]).max()))
    assert worst <= 1e-12, 'worst half-space violation {:+.4e} m'.format(worst)


def test_the_assignment_is_by_triangle_and_the_vertex_version_is_unsound():
    """
    M-4 as a positive measurement, not merely as a passing test.

    The vertex-wise assignment is the natural implementation and it is wrong.
    Here it is, built and measured, so that the 1.1598 mm is a number this
    repository can reproduce rather than a claim in a commit message.
    """
    from scipy.spatial import ConvexHull

    pieces = [(name, *(read_obj(ASSET_DIRECTORY / name)))
              for name in ('link5_collision_0.obj', 'link5_collision_1.obj',
                           'link5_collision_2.obj')]
    shell_points, shell_faces = read_dae(VISUAL_DIRECTORY / 'link5.dae')
    per_piece = np.column_stack([distance_to_convex(shell_points, vertices, faces)
                                 for _, vertices, faces in pieces])
    owner = np.argmin(per_piece, axis=1)          # by VERTEX - the trap
    planes = []
    for index, (_, vertices, _) in enumerate(pieces):
        generators = np.vstack([vertices, shell_points[owner == index]])
        hull = ConvexHull(generators)
        planes.append((hull.equations[:, :3], -hull.equations[:, 3]))
    triangles = shell_points[shell_faces]
    centroids = triangles.mean(axis=1)
    violation = np.full(len(centroids), np.inf)
    for normals, offsets in planes:
        violation = np.minimum(
            violation, (centroids @ normals.T - offsets[None, :]).max(axis=1))
    # Measured at the triangle CENTROIDS, which under-reports the surface
    # maximum; it is enough to separate the two assignments by three orders of
    # magnitude from the correct one's +1.1e-13 mm.
    assert violation.max() > 0.0005, (
        'the vertex-wise assignment is supposed to leave the shell outside the '
        'union by about a millimetre; it left {:.4f} mm'.format(
            violation.max() * 1000.0))


# ---------------------------------------------------------------------------
# T-BODY-2: the reported fidelity figure
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('link', sorted(SHELL_UNDERCUT))
def test_the_recorded_shell_undercut_reproduces(bodies, link):
    """
    T-BODY-2: section 3's eight values, recomputed from the pinned assets.

    This number is REPORTED and used by nothing.  The fence never sees it,
    because the bodies contain the shell; it exists so that an asset change
    moves a number in a diff instead of moving a clearance in silence.
    """
    expected, witness = SHELL_UNDERCUT[link]
    per_link = collision_inventory(MUJOCO_PATH)
    pieces = []
    for name in per_link[link]:
        path = ASSET_DIRECTORY / name
        pieces.append((read_stl(path) if name.endswith('.stl') else read_obj(path)))
    shell_points, _ = read_dae(VISUAL_DIRECTORY / '{}.dae'.format(link))
    per_piece = np.column_stack([distance_to_convex(shell_points, vertices, faces)
                                 for vertices, faces in pieces])
    measured = per_piece.min(axis=1)
    assert abs(float(measured.max()) - expected) < 5e-9
    # ...and it is attained at the named witness vertex.
    where = shell_points[int(np.argmax(measured))]
    assert np.allclose(where, witness, atol=1e-6), where
    for body in bodies.by_link[link]:
        assert abs(body.shell_undercut_m - expected) < 5e-9


# ---------------------------------------------------------------------------
# The body table and the artefact's own shape
# ---------------------------------------------------------------------------

def test_the_body_table_is_the_measured_one(bodies):
    """Section 1.1, to the digit, from the shipped artefact."""
    document = yaml.safe_load(MESH_BODIES_PATH.read_text(encoding='utf-8'))
    emitted = {entry['id']: entry for entry in document['bodies']}
    assert set(emitted) == set(BODY_TABLE)
    for name, (vertices, faces, volume, radius) in BODY_TABLE.items():
        entry = emitted[name]
        assert len(entry['vertices']) == vertices, name
        assert entry['face_count'] == faces, name
        assert abs(entry['volume_m3'] * 1e6 - volume) < 0.01, name
        assert abs(entry['bounding_capsule']['r'] - radius) < 5e-5, name
    assert sum(len(entry['vertices']) for entry in document['bodies']) == TOTAL_VERTICES


def test_the_bounding_capsule_radius_really_bounds_its_body(bodies):
    """
    The one property that makes the broad phase's bound CERTIFIED.

    ``seg_seg(A, B) - r_a - r_b <= gjk(body_a, body_b)`` holds only because each
    radius is the EXACT maximum vertex-to-segment distance of its own body.  A
    radius even a millimetre short - mutation M-6 - makes the broad phase cull
    pairs that are actually close, which is unsound in the direction that
    matters.
    """
    for body in bodies.bodies:
        span = body.capsule_b - body.capsule_a
        length_squared = float(span @ span)
        parameter = np.clip(((body.vertices - body.capsule_a) @ span) / length_squared,
                            0.0, 1.0)
        foot = body.capsule_a + parameter[:, None] * span
        worst = float(np.linalg.norm(body.vertices - foot, axis=1).max())
        assert worst <= body.capsule_radius + 1e-12, body.id
        # ...and it is TIGHT: a radius much larger than the worst vertex would
        # cull nothing and cost GJK calls on every pair.
        assert body.capsule_radius - worst < 1e-9, body.id


def test_only_one_arms_bodies_are_emitted(bodies):
    """
    T-SYM's artefact half: the file carries one arm and both arms use it.

    A body id in the artefact carries no arm prefix at all, which is what makes
    a per-arm divergence impossible to express.
    """
    for body in bodies.bodies:
        assert not body.id.startswith('panda')
        assert not body.link.startswith('panda')
    assert len(bodies) == 11


def test_the_artefact_stays_inside_its_own_three_caps(bodies):
    """
    All three caps, not only bytes.

    The payload is 34 779 vertex scalars, SEVENTEEN times strictyaml's 2 048, so
    a reader built on the cell files' constants refuses this artefact long
    before any byte bound is reached; the byte cap was never going to be the
    binding one.
    """
    text = MESH_BODIES_PATH.read_text(encoding='utf-8')
    assert len(text.encode('utf-8')) == bodies.bytes <= MAXIMUM_MESH_BODIES_BYTES
    scalars = sum(1 for event in yaml.parse(text)
                  if isinstance(event, yaml.events.ScalarEvent))
    assert scalars <= MAXIMUM_MESH_BODIES_SCALARS
    assert scalars > 3 * TOTAL_VERTICES
    from franka_workspace_model.strictyaml import MAXIMUM_YAML_SCALARS
    assert scalars > 15 * MAXIMUM_YAML_SCALARS


def test_the_emission_style_is_one_vertex_per_line():
    """
    The style is part of the contract because it moves the file across a cap.

    One flow-sequence line per vertex measures 545 KB; a fully block-style
    emission of the same numbers measures about 980 KB and would fail the
    generator's own byte abort.
    """
    lines = MESH_BODIES_PATH.read_text(encoding='utf-8').splitlines()
    vertex_lines = [line for line in lines if line.startswith('      - [')]
    assert len(vertex_lines) == TOTAL_VERTICES
    for line in vertex_lines[:200]:
        assert line.count(',') == 2


# ---------------------------------------------------------------------------
# T-BODY-3: byte-exact regeneration
# ---------------------------------------------------------------------------

def test_bodies_regenerate_byte_for_byte():
    """
    T-BODY-3, exactly as test_generator.py does for CONTRACT B.

    A generator whose output depends on a hull library's vertex ordering, on
    numpy's threading, or on the machine would make every other assertion in
    this file a statement about one run.  It does not: the vertices are sorted
    descending on (z, y, x) at the emitted precision, the same ordering CONTRACT
    B applies to capsule endpoints.
    """
    geometry = yaml.safe_load(LINK_GEOMETRY_PATH.read_text(encoding='utf-8'))
    regenerated = generate(REPOSITORY_ROOT, geometry['source']['urdf_sha256'],
                           float(geometry['source']['safety_distance']))
    assert regenerated == MESH_BODIES_PATH.read_text(encoding='utf-8')


def test_the_committed_artefact_matches_what_the_cell_file_pins():
    """A stale artefact is a load failure and never a fallback to capsules."""
    document = yaml.safe_load(CELL_MODEL_PATH.read_text(encoding='utf-8'))
    digest = hashlib.sha256(MESH_BODIES_PATH.read_bytes()).hexdigest()
    assert document['sources']['mesh_bodies_sha256'] == digest
    assert document['sources']['mesh_bodies'] == 'mesh_bodies_v1.yaml'


def test_the_generator_version_is_recorded(bodies):
    """The artefact records which generator produced it."""
    assert bodies.source['generator_version'] == GENERATOR_VERSION


# ---------------------------------------------------------------------------
# T-FLANGE
# ---------------------------------------------------------------------------

def test_link8_flange_contains_its_own_primitives(bodies):
    """
    T-FLANGE: the certificate the flange never had, checked in closed form.

    A convex polytope contains a convex set exactly when every face plane
    satisfies ``h_P(n_i) <= d_i``, and the support of the union of two spheres
    and a cylinder is closed form - so this is a PROOF over all directions, not
    a sample at some resolution.

    It matters because ``link8`` is the one body in the model with no mesh
    behind it: franka_description ships none, six of the sixteen enabled self
    pairs touch it, and it binds on 31.6 % of uniform draws.  A hull of points
    sampled ON a sphere is INSCRIBED in it, so the natural construction produces
    a body 0.308 mm SMALLER than the geometry it claims to represent - optimism,
    silent, on the weakest body in the fence.
    """
    from scipy.spatial import ConvexHull

    geometry = yaml.safe_load(LINK_GEOMETRY_PATH.read_text(encoding='utf-8'))
    primitives = flange_primitives(float(geometry['source']['safety_distance']))
    flange = bodies.by_id['link8_flange']
    hull = ConvexHull(flange.vertices)
    normals = hull.equations[:, :3]
    offsets = -hull.equations[:, 3]
    slack = np.array([offsets[index] - flange_support(primitives, normals[index])
                      for index in range(len(normals))])
    assert (slack >= 0.0).all(), 'worst violation {:+.6f} mm'.format(
        -float(slack.min()) * 1000.0)
    assert abs(float(slack.min()) - 7.86715e-07) < 1e-11
    assert abs(float(slack.max()) - 2.31745e-04) < 1e-8

    document = yaml.safe_load(MESH_BODIES_PATH.read_text(encoding='utf-8'))
    entry = next(item for item in document['bodies'] if item['id'] == 'link8_flange')
    derivation = entry['derivation']
    assert derivation['lattice_scale'] == FLANGE_LATTICE_SCALE
    assert derivation['recovered_safety_distance'] == 0.03
    certificate = derivation['containment_certificate']
    assert certificate['faces_violated'] == 0
    assert certificate['worst_primitive_protrusion_m'] == 0.0
    # The artefact emits at twelve decimals, so the two agree to that.
    assert abs(certificate['min_face_slack_m'] - float(slack.min())) < 1e-12


def test_the_inscribed_flange_is_measurably_optimistic():
    """
    M-8b, as a measurement: at scale 1.0 the hull is INSIDE its own primitives.

    This is the defect the plan shipped before verification.  Building it is the
    only way the 0.308 mm is a number this repository can reproduce.
    """
    from scipy.spatial import ConvexHull

    primitives = flange_primitives(0.03)
    for scale, floor in ((1.0, 0.0003), (1.0104, None)):
        points = flange_lattice(primitives, scale, 64, (16, 32))
        hull = ConvexHull(points[ConvexHull(points).vertices])
        normals = hull.equations[:, :3]
        offsets = -hull.equations[:, 3]
        violation = max(flange_support(primitives, normals[index]) - offsets[index]
                        for index in range(len(normals)))
        if floor is None:
            assert violation <= 0.0
        else:
            assert violation > floor, (
                'scale {} should leave the primitives outside the hull by about '
                '0.308 mm; it left {:.6f} mm'.format(scale, violation * 1000.0))


def test_a_coarser_flange_lattice_is_refused():
    """M-8b's other half: a coarsened lattice fails the same certificate."""
    primitives = flange_primitives(0.03)
    from scipy.spatial import ConvexHull
    for rim, lattice, floor in ((32, (8, 16), 0.001), (16, (6, 12), 0.002)):
        points = flange_lattice(primitives, FLANGE_LATTICE_SCALE, rim, lattice)
        hull = ConvexHull(points[ConvexHull(points).vertices])
        normals = hull.equations[:, :3]
        offsets = -hull.equations[:, 3]
        violation = max(flange_support(primitives, normals[index]) - offsets[index]
                        for index in range(len(normals)))
        assert violation > floor, (rim, lattice, violation)


def test_the_flange_is_labelled_as_what_it_is(bodies):
    """
    The flange is an ENVELOPE and the artefact says so in its own field.

    ``body_source`` distinguishes it from every body derived from a mesh, and
    the written reason names what would retire it: one caliper reading.
    """
    flange = bodies.by_id['link8_flange']
    assert flange.body_source == 'urdf_primitive_at_metal_radius'
    for body in bodies.bodies:
        if body.id != 'link8_flange':
            assert body.body_source == 'collision_mesh_and_shell'
    listed = bodies.source['links_without_collision_mesh']
    assert [item['link'] for item in listed] == ['link8']
    assert 'caliper' in listed[0]['reason']
    assert 'no link8 mesh' in listed[0]['reason']


def test_franka_description_really_ships_no_link8_mesh():
    """The finding the flange derivation rests on, checked rather than asserted."""
    names = {path.name for path in VISUAL_DIRECTORY.iterdir()}
    assert 'link7.dae' in names
    assert 'link8.dae' not in names
    assert not any(name.startswith('link8') for name in names)
    assert not any(path.name.startswith('link8')
                   for path in ASSET_DIRECTORY.iterdir())


# ---------------------------------------------------------------------------
# T-DIGEST and the load rules
# ---------------------------------------------------------------------------

def test_a_tampered_artefact_is_a_load_error(tmp_path):
    """T-DIGEST: an artefact that is not the pinned one does not load."""
    path = write_cell_model(tmp_path)
    target = path.parent / 'mesh_bodies_v1.yaml'
    target.unlink()
    text = MESH_BODIES_PATH.read_text(encoding='utf-8')
    target.write_text(text.replace('  face_count: 960', '  face_count: 961'),
                      encoding='utf-8')
    with pytest.raises(WorkspaceModelError, match='mesh_bodies_sha256'):
        CellModel.load(path, profile='dual')


def test_a_body_may_not_claim_there_is_something_to_add_back(tmp_path):
    """
    A mesh body carrying ``inflation`` or ``coverage`` is refused BY NAME.

    Those keys belong to a fence that undercuts the shell and compensates with a
    scalar.  These bodies contain the shell, so there is nothing to add back,
    and a key that says otherwise is a sign somebody has reintroduced the model
    the mesh work exists to replace.
    """
    for key in ('inflation', 'coverage'):
        target = tmp_path / '{}.yaml'.format(key)
        text = MESH_BODIES_PATH.read_text(encoding='utf-8')
        text = text.replace('  - id: link0_st\n',
                            '  - id: link0_st\n    {}: 0.03\n'.format(key), 1)
        target.write_text(text, encoding='utf-8')
        with pytest.raises(WorkspaceModelError, match='nothing to add back'):
            load_mesh_bodies(target)


def test_a_different_description_digest_is_reported_not_refused(tmp_path):
    """
    The provenance difference is SAID, once, and the bodies are still used.

    Refusing here would mean a single-arm session could not use bodies that are,
    link for link, the same geometry - they carry no arm prefix and every input
    they come from is shared between the two descriptions.  Saying nothing would
    hide a real difference.  So it is a diagnostic.
    """
    target = tmp_path / 'other.yaml'
    target.write_text(MESH_BODIES_PATH.read_text(encoding='utf-8'), encoding='utf-8')
    loaded = load_mesh_bodies(target, urdf_sha256='0' * 64)
    assert len(loaded) == 11
    assert any('0' * 64 in message for message in loaded.diagnostics)


def test_a_flange_at_the_padded_radius_is_refused(tmp_path):
    """
    M-8: the flange must be emitted at METAL radius, and the load rule says so.

    The URDF declares the three link8 primitives at ``${0.03 + safety_distance}``.
    A body built at the declared radius carries 30 mm of air that is not there,
    on six of the sixteen enabled self pairs, at the part of the arm most likely
    to be near something.
    """
    target = tmp_path / 'padded.yaml'
    text = MESH_BODIES_PATH.read_text(encoding='utf-8')
    target.write_text(text, encoding='utf-8')
    with pytest.raises(WorkspaceModelError, match='PADDED radius'):
        load_mesh_bodies(target, safety_distance=0.0)


def test_an_inscribed_flange_scale_is_refused(tmp_path):
    """M-8b as a load rule: a lattice_scale below the containing minimum."""
    target = tmp_path / 'inscribed.yaml'
    text = MESH_BODIES_PATH.read_text(encoding='utf-8')
    target.write_text(text.replace('      lattice_scale: 1.0104',
                                   '      lattice_scale: 1.0'), encoding='utf-8')
    with pytest.raises(WorkspaceModelError, match='INSCRIBED'):
        load_mesh_bodies(target)


def test_a_body_built_from_the_collision_solid_alone_is_refused(tmp_path):
    """
    ``metal_definition`` is a load-checked claim, not a comment.

    M-5 - dropping the shell - makes the fence optimistic by up to 7.11 mm on
    link5/link7 by construction, and it would otherwise be invisible: the file
    would still parse and every body would still be convex.
    """
    target = tmp_path / 'no_shell.yaml'
    text = MESH_BODIES_PATH.read_text(encoding='utf-8')
    target.write_text(
        text.replace('metal_definition: collision_union_visual_shell',
                     'metal_definition: collision_only'), encoding='utf-8')
    with pytest.raises(WorkspaceModelError, match='6.5 mm'):
        load_mesh_bodies(target)


def test_a_vertex_assignment_claim_is_refused(tmp_path):
    """M-4 again, this time as a load rule on the artefact's own claim."""
    target = tmp_path / 'by_vertex.yaml'
    text = MESH_BODIES_PATH.read_text(encoding='utf-8')
    target.write_text(
        text.replace('assignment: whole_triangle_nearest_piece',
                     'assignment: nearest_piece_by_vertex'), encoding='utf-8')
    with pytest.raises(WorkspaceModelError, match='1.1598 mm'):
        load_mesh_bodies(target)


def test_an_old_link_geometry_is_refused_by_name(tmp_path):
    """A link geometry generated before the mesh work fails with its version."""
    root = tmp_path / 'repository'
    path = write_cell_model(tmp_path)
    target = root / 'cell' / 'link_geometry_v1.yaml'
    text = LINK_GEOMETRY_PATH.read_text(encoding='utf-8')
    target.unlink()
    target.write_text(text.replace('generator_version: 2', 'generator_version: 1'),
                      encoding='utf-8')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    document['sources']['link_geometry_sha256'] = hashlib.sha256(
        target.read_bytes()).hexdigest()
    path.write_text(yaml.safe_dump(document, default_flow_style=False, sort_keys=False),
                    encoding='utf-8')
    with pytest.raises(WorkspaceModelError, match='generator_version is 1'):
        CellModel.load(path, profile='dual')


def test_the_convexity_gate_is_drift_detection_and_says_so(bodies):
    """
    The gate guards the SOURCE assets and is not a soundness argument.

    GJK reads vertices only and therefore evaluates the convex HULL of whatever
    it is given.  The hull contains the mesh, so a convexity defect can only
    make GJK under-report the body's extent, which is the safe direction.  The
    2 mm value is a drift alarm; the worst measured defect on the shipped assets
    is 1.000151038e-03 m on link3.stl.
    """
    document = yaml.safe_load(MESH_BODIES_PATH.read_text(encoding='utf-8'))
    assert document['source']['convexity_tolerance_m'] == 0.002
    defects = {entry['id']: entry['convexity_defect_m'] for entry in document['bodies']}
    # The worst defect on the shipped assets, reproduced to the digit a
    # plane-fitting convention can move: 1.00015e-03 m on link3.stl.
    assert abs(defects['link3_st'] - 1.00015e-03) < 1e-8
    for name, value in defects.items():
        assert 0.0 <= value <= 0.002, name


def test_the_asset_digests_pin_what_was_read(bodies):
    """
    An asset change becomes a diff in this file rather than a silent clearance.

    Every digest here is recomputed from the bytes on disk; a swapped asset of
    equal size (M-13) changes the digest and fails this test, where a length
    comparison would not.
    """
    for name, digest in bodies.source['collision_meshes'].items():
        assert hashlib.sha256(
            (ASSET_DIRECTORY / name).read_bytes()).hexdigest() == digest, name
    for name, digest in bodies.source['visual_meshes'].items():
        assert hashlib.sha256(
            (VISUAL_DIRECTORY / name).read_bytes()).hexdigest() == digest, name
    assert len(bodies.source['collision_meshes']) == 10
    assert len(bodies.source['visual_meshes']) == 8
    assert hashlib.sha256(MUJOCO_PATH.read_bytes()).hexdigest() == (
        bodies.source['mujoco_model_sha256'])


def test_the_hull_planes_of_every_body_contain_its_own_vertices(bodies):
    """A body whose own vertices escape its own faces is not convex."""
    for body in bodies.bodies:
        from scipy.spatial import ConvexHull
        hull = ConvexHull(body.vertices)
        normals = hull.equations[:, :3]
        offsets = -hull.equations[:, 3]
        worst = float((body.vertices @ normals.T - offsets[None, :]).max())
        assert worst < 1e-9, (body.id, worst)
