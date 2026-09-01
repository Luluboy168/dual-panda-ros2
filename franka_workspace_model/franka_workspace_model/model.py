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
The workspace model: one loaded object, one source of truth, three consumers.

This module is the whole public surface of the package.  It imports numpy and
pyyaml and nothing else; it must never import ROS, in either direction of the
call graph, because the model sits *above* the reviewed controller fence and may
only ever reject - it never widens, relaxes, overrides or substitutes for any
check the controller performs.

Concurrency: ``CellModel`` is immutable after ``load``.  Nothing in
``check_configuration``, ``check_path`` or ``check_jog`` mutates the instance or
any module-level state, so the three are safe to call concurrently from several
threads on one loaded model without a lock.
"""

from dataclasses import dataclass
import datetime
import hashlib
import math
from pathlib import Path
import re
from typing import Any, Mapping, Optional, Sequence
import xml.etree.ElementTree as ElementTree

import numpy as np

from .geometry import (GeometryError, homogeneous, rotation_from_rpy,
                       segment_box_distance, segment_halfspace_distance,
                       segment_point_distance, segment_segment_distance,
                       segment_segment_distance_batch)
from .strictyaml import (exact_keys, load_strict_yaml, read_bounded_regular_text,
                         WorkspaceModelError)


__all__ = [
    'AllowedVolume',
    'CellModel',
    'CheckResult',
    'Contact',
    'JogResult',
    'WorkspaceModelError',
    'default_cell_model_path',
    'result_to_json',
]

SCHEMA_VERSION = 1
JOINT_COUNT = 7
CELL_MODEL_FILENAME = 'cell_model_v1.yaml'

#: The exact key paths at which a boolean value is accepted.  Everywhere else a
#: boolean is a load failure, as in the reviewed controller-config precedent.
BOOLEAN_KEY_PATHS = (
    'arms[].end_effector.present',
    'keep_out[].enabled',
    'policy.fail_closed',
    'policy.cross_arm.enabled',
    'policy.environment.enabled',
    'policy.containment.enabled',
)

TOP_LEVEL_KEYS = (
    'schema_version', 'model_id', 'revision', 'measured_on', 'measured_by', 'units',
    'sources', 'cell_frame', 'arms', 'allowed_volume', 'environment', 'keep_out',
    'margins', 'policy',
)
SOURCES_KEYS = (
    'urdf_xacro', 'urdf_xacro_sha256', 'srdf_xacro', 'srdf_xacro_sha256',
    'joint_limit_policy', 'joint_limit_policy_sha256', 'link_geometry',
    'link_geometry_sha256',
)
ARM_KEYS = ('arm_id', 'base_link', 'urdf_base_pose', 'measured_base_pose', 'end_effector')
MEASURED_POSE_KEYS = ('xyz', 'rpy', 'tolerance_m', 'tolerance_rad', 'measurement_status')
END_EFFECTOR_KEYS = ('present', 'profile', 'volumes')
END_EFFECTOR_VOLUME_KEYS = (
    'id', 'kind', 'a', 'b', 'radius', 'containment', 'containment_margin',
    'derivation_status', 'provenance',
)
ALLOWED_VOLUME_KEYS = (
    'id', 'frame', 'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max',
    'measurement_status', 'source_question', 'note',
)
ENVIRONMENT_COMMON_KEYS = ('id', 'kind', 'frame', 'measurement_status',
                           'source_question', 'note')
KEEP_OUT_COMMON_KEYS = ('id', 'kind', 'frame', 'applies_to', 'enabled', 'reason',
                        'source_question')
GEOMETRY_KEYS = {
    'box': ('pose', 'size'),
    'cylinder': ('pose', 'length', 'radius'),
    'sphere': ('pose', 'radius'),
    'capsule': ('a', 'b', 'radius'),
    'plane_halfspace': ('normal', 'offset'),
}
MARGIN_KEYS = ('urdf_builtin_inflation', 'self_collision', 'cross_arm', 'environment',
               'keep_out', 'swept_path_extra', 'rationale')
POLICY_KEYS = ('schema', 'default_mode', 'max_joint_step_rad', 'fail_closed',
               'self_collision', 'cross_arm', 'environment', 'containment',
               'joint_limits')
MEASUREMENT_STATUS = ('measured', 'assumed', 'inherited_from_urdf')
END_EFFECTOR_PROFILES = ('none', 'robotiq_2f85')
CONTACT_KINDS = ('joint_limit', 'self', 'cross_arm', 'containment', 'environment',
                 'keep_out')
BOX_FACES = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max')

IDENTIFIER_PATTERN = re.compile(r'^[a-z][a-z0-9_]{2,63}$')
ARM_ID_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9_]{0,63}$')
HASH_PATTERN = re.compile(r'^[0-9a-f]{64}$')
DATE_PATTERN = re.compile(r'^\d{4}-\d{2}-\d{2}$')
SOURCE_QUESTION_PATTERN = re.compile(r'^Q(1[0-7]|[1-9])(,Q(1[0-7]|[1-9]))*$')
PATH_PATTERN = re.compile(r'^[A-Za-z0-9._/-]{1,256}$')
PRINTABLE_PATTERN = re.compile(r'^[\x20-\x7E]*$')

ANTISYMMETRY_TOLERANCE = 1e-9
INFLATION_TOLERANCE = 1e-12
UNIT_NORMAL_TOLERANCE = 1e-9
BASE_PLANE_TOLERANCE = 1e-9
VERTICAL_AXIS_TOLERANCE = 1e-9
#: Half the space diagonal of the 0.1 m pedestal cube: the conservative bounding
#: sphere the load-time static diagnostic uses, so the diagnostic needs no
#: box-box primitive.
PEDESTAL_BOUNDING_RADIUS = 0.05 * math.sqrt(3.0)
#: An arm's flange cannot rise above this many metres over its own base; used to
#: tell the operator when a declared ceiling could actually bind.
REACHABILITY_HEIGHT_BOUND = 1.401


@dataclass(frozen=True)
class Contact:
    """One reported reason a configuration is not allowed."""

    kind: str
    a: str
    b: str
    distance: float
    required: float
    arm_id: str


@dataclass(frozen=True)
class CheckResult:
    """The verdict on one configuration or one resampled path."""

    ok: bool
    min_clearance: float
    contacts: tuple
    sample_index: Optional[int]
    samples_evaluated: int
    model_id: str
    model_revision: int
    model_sha256: str


@dataclass(frozen=True)
class JogResult:
    """The verdict on one single-joint relative move, clamped rather than refused."""

    allowed: bool
    q_target: tuple
    clamped: bool
    limiting: Optional[Contact]
    result: CheckResult


@dataclass(frozen=True)
class AllowedVolume:
    """The measured cell as an axis-aligned box the arms must stay inside."""

    id: str  # noqa: A003 - the field name is part of the published surface
    frame: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


def default_cell_model_path() -> Optional[Path]:
    """
    Return the installed example cell file, or None when there is none.

    A consumer that must find a cell model without being configured calls this
    rather than hard-coding a filename from this package.
    """
    candidate = Path(__file__).resolve().parent.parent / 'cell' / CELL_MODEL_FILENAME
    if candidate.is_file():
        return candidate
    for prefix in (Path(__file__).resolve().parents[3:]):
        installed = prefix / 'share' / 'franka_workspace_model' / 'cell' / CELL_MODEL_FILENAME
        if installed.is_file():
            return installed
    return None


def _round(value: Any) -> Any:
    if isinstance(value, float):
        if math.isinf(value):
            return 'inf' if value > 0 else '-inf'
        return round(value, 6)
    return value


def result_to_json(result) -> dict:
    """Serialise a result with exactly the dataclass field names, no renaming."""
    if isinstance(result, JogResult):
        return {
            'allowed': result.allowed,
            'q_target': [_round(value) for value in result.q_target],
            'clamped': result.clamped,
            'limiting': None if result.limiting is None else _contact_to_json(result.limiting),
            'result': result_to_json(result.result),
        }
    if isinstance(result, CheckResult):
        return {
            'ok': result.ok,
            'min_clearance': _round(result.min_clearance),
            'contacts': [_contact_to_json(contact) for contact in result.contacts],
            'sample_index': result.sample_index,
            'samples_evaluated': result.samples_evaluated,
            'model_id': result.model_id,
            'model_revision': result.model_revision,
            'model_sha256': result.model_sha256,
        }
    raise WorkspaceModelError('result_to_json accepts a CheckResult or a JogResult')


def _contact_to_json(contact: Contact) -> dict:
    return {
        'kind': contact.kind,
        'a': contact.a,
        'b': contact.b,
        'distance': _round(contact.distance),
        'required': _round(contact.required),
        'arm_id': contact.arm_id,
    }


def _sha256_of_file(path: Path, context: str) -> str:
    try:
        with open(path, 'rb') as handle:
            digest = hashlib.sha256()
            while True:
                block = handle.read(65536)
                if not block:
                    break
                digest.update(block)
    except OSError as error:
        raise WorkspaceModelError('unable to read {}'.format(context)) from error
    return digest.hexdigest()


def _string(mapping, key, context, pattern=None, minimum=0, maximum=1024):
    value = mapping[key]
    if not isinstance(value, str):
        raise WorkspaceModelError('{}.{} must be a string'.format(context, key))
    if not minimum <= len(value) <= maximum:
        raise WorkspaceModelError(
            '{}.{} must be {}..{} characters long'.format(context, key, minimum, maximum))
    if not PRINTABLE_PATTERN.match(value):
        raise WorkspaceModelError(
            '{}.{} must be printable ASCII'.format(context, key))
    if pattern is not None and not pattern.match(value):
        raise WorkspaceModelError(
            '{}.{} does not match the required form: {!r}'.format(context, key, value))
    return value


def _enum(mapping, key, context, allowed):
    value = _string(mapping, key, context)
    if value not in allowed:
        raise WorkspaceModelError(
            '{}.{} must be one of {}, found {!r}'.format(context, key, list(allowed), value))
    return value


def _integer(mapping, key, context):
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, int):
        raise WorkspaceModelError('{}.{} must be an integer'.format(context, key))
    return value


def _number(mapping, key, context):
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WorkspaceModelError('{}.{} must be numeric'.format(context, key))
    result = float(value)
    if not math.isfinite(result):
        raise WorkspaceModelError('{}.{} must be finite'.format(context, key))
    return result


def _boolean(mapping, key, context):
    value = mapping[key]
    if not isinstance(value, bool):
        raise WorkspaceModelError('{}.{} must be true or false'.format(context, key))
    return value


def _number_list(mapping, key, context, length):
    values = mapping[key]
    if not isinstance(values, list) or len(values) != length:
        raise WorkspaceModelError(
            '{}.{} must be a list of {} numbers'.format(context, key, length))
    result = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise WorkspaceModelError(
                '{}.{} must contain only numbers'.format(context, key))
        converted = float(value)
        if not math.isfinite(converted):
            raise WorkspaceModelError(
                '{}.{} must contain only finite numbers'.format(context, key))
        result.append(converted)
    return np.array(result, dtype=float)


def _angles(mapping, key, context):
    values = _number_list(mapping, key, context, 3)
    if any(abs(value) > math.pi + 1e-12 for value in values):
        raise WorkspaceModelError(
            '{}.{} components must lie in [-pi, pi]'.format(context, key))
    return values


# ---------------------------------------------------------------------------
# CONTRACT B - the generated link geometry
# ---------------------------------------------------------------------------

LINK_GEOMETRY_TOP_KEYS = ('schema_version', 'source', 'root_link', 'links')
LINK_GEOMETRY_SOURCE_KEYS = ('urdf_xacro', 'xacro_args', 'urdf_sha256',
                             'safety_distance', 'generator_version')
LINK_GEOMETRY_VOLUME_KEYS = {
    'capsule': ('id', 'kind', 'a', 'b', 'radius', 'containment', 'containment_margin',
                'source_elements'),
    'sphere': ('id', 'kind', 'origin_xyz', 'radius', 'containment',
               'containment_margin', 'source_elements'),
    'box': ('id', 'kind', 'size', 'origin_xyz', 'containment', 'containment_margin',
            'source_elements'),
}


class _Volume:
    """One derived collision volume, in its own link frame."""

    def __init__(self, entry, link_name, context):
        self.id = _string(entry, 'id', context)
        expected = '^{}_v[0-9]+$'.format(re.escape(link_name))
        if not re.match(expected, self.id):
            raise WorkspaceModelError(
                '{}: volume id {!r} must be <link>_v<n>'.format(context, self.id))
        self.link = link_name
        self.kind = entry['kind']
        self.containment = _enum(entry, 'containment', context, ('exact', 'conservative'))
        self.containment_margin = _number(entry, 'containment_margin', context)
        if self.containment_margin < 0.0:
            raise WorkspaceModelError(
                '{}: containment_margin must be non-negative'.format(context))
        if (self.containment == 'exact') != (self.containment_margin == 0.0):
            raise WorkspaceModelError(
                '{}: containment_margin is zero exactly when containment is '
                'exact'.format(context))
        elements = entry['source_elements']
        if (not isinstance(elements, list) or not elements
                or any(isinstance(item, bool) or not isinstance(item, int)
                       for item in elements)
                or sorted(elements) != elements
                or len(set(elements)) != len(elements)):
            raise WorkspaceModelError(
                '{}: source_elements must be ascending unique integers'.format(context))
        self.source_elements = tuple(elements)
        if self.kind == 'capsule':
            self.a = _number_list(entry, 'a', context, 3)
            self.b = _number_list(entry, 'b', context, 3)
            if float(np.linalg.norm(self.b - self.a)) < 1e-9:
                raise WorkspaceModelError(
                    '{}: a zero-length capsule segment is rejected'.format(context))
            self.radius = _number(entry, 'radius', context)
        elif self.kind == 'sphere':
            self.a = _number_list(entry, 'origin_xyz', context, 3)
            self.b = self.a
            self.radius = _number(entry, 'radius', context)
        else:
            self.size = _number_list(entry, 'size', context, 3)
            if any(value <= 0.0 for value in self.size):
                raise WorkspaceModelError('{}: box size must be positive'.format(context))
            self.a = _number_list(entry, 'origin_xyz', context, 3)
            self.b = self.a
            self.radius = 0.0
        if self.kind in ('capsule', 'sphere') and self.radius <= 0.0:
            raise WorkspaceModelError('{}: radius must be positive'.format(context))


class _LinkGeometry:
    """The parsed CONTRACT B artefact."""

    def __init__(self, document, path):
        context = 'link geometry'
        exact_keys(document, LINK_GEOMETRY_TOP_KEYS, context)
        if _integer(document, 'schema_version', context) != SCHEMA_VERSION:
            raise WorkspaceModelError(
                'link geometry schema_version must be {}'.format(SCHEMA_VERSION))
        source = exact_keys(document['source'], LINK_GEOMETRY_SOURCE_KEYS,
                            'link geometry source')
        self.urdf_xacro = _string(source, 'urdf_xacro', 'link geometry source',
                                  PATH_PATTERN)
        self.urdf_sha256 = _string(source, 'urdf_sha256', 'link geometry source',
                                   HASH_PATTERN)
        self.safety_distance = _number(source, 'safety_distance', 'link geometry source')
        if self.safety_distance < 0.0:
            raise WorkspaceModelError(
                'link geometry source.safety_distance must be non-negative')
        if _integer(source, 'generator_version', 'link geometry source') < 1:
            raise WorkspaceModelError(
                'link geometry source.generator_version must be at least 1')
        arguments = source['xacro_args']
        if not isinstance(arguments, dict) or not arguments:
            raise WorkspaceModelError('link geometry source.xacro_args must be a mapping')
        for name, value in arguments.items():
            if not isinstance(value, str):
                raise WorkspaceModelError(
                    'link geometry source.xacro_args.{} must be a string'.format(name))
            if name.startswith('robot_ip') and value != '':
                raise WorkspaceModelError(
                    'link geometry source.xacro_args.{} must be the empty string; a '
                    'committed artefact never contains a network address'.format(name))
        self.xacro_args = dict(arguments)
        self.path = path
        self.root_link = document['root_link']
        if not isinstance(self.root_link, str):
            raise WorkspaceModelError('link geometry root_link must be a string')
        entries = document['links']
        if not isinstance(entries, list) or not entries:
            raise WorkspaceModelError('link geometry links must be a non-empty list')
        self.link_order = []
        self.parent = {}
        self.joints = {}
        self.volumes = {}
        volume_ids = set()
        for entry in entries:
            if not isinstance(entry, dict) or 'link' not in entry:
                raise WorkspaceModelError('every link geometry entry needs a link name')
            name = entry['link']
            if not isinstance(name, str):
                raise WorkspaceModelError('link geometry link names must be strings')
            if name in self.volumes:
                raise WorkspaceModelError('duplicate link geometry entry {!r}'.format(name))
            is_root = name == self.root_link
            expected = ('link', 'volumes') if is_root else (
                'link', 'parent_link', 'parent_joint', 'volumes')
            exact_keys(entry, expected, 'link geometry entry {!r}'.format(name))
            if not is_root:
                self.parent[name] = entry['parent_link']
                self.joints[name] = self._joint(entry['parent_joint'], name)
            volumes = entry['volumes']
            if not isinstance(volumes, list) or not volumes:
                raise WorkspaceModelError(
                    'link geometry entry {!r} carries no volumes'.format(name))
            parsed = []
            for index, volume_entry in enumerate(volumes):
                context = 'link geometry {} volume {}'.format(name, index)
                if not isinstance(volume_entry, dict) or 'kind' not in volume_entry:
                    raise WorkspaceModelError('{} needs a kind'.format(context))
                kind = volume_entry['kind']
                if kind not in LINK_GEOMETRY_VOLUME_KEYS:
                    raise WorkspaceModelError(
                        '{}: unknown volume kind {!r}'.format(context, kind))
                exact_keys(volume_entry, LINK_GEOMETRY_VOLUME_KEYS[kind], context)
                volume = _Volume(volume_entry, name, context)
                if volume.id != '{}_v{}'.format(name, index):
                    raise WorkspaceModelError(
                        '{}: volume ids must be contiguous from zero'.format(context))
                if volume.id in volume_ids:
                    raise WorkspaceModelError(
                        'duplicate volume id {!r}'.format(volume.id))
                volume_ids.add(volume.id)
                parsed.append(volume)
            self.link_order.append(name)
            self.volumes[name] = tuple(parsed)
        if self.root_link not in self.volumes:
            raise WorkspaceModelError(
                'link geometry root_link {!r} is not among the links'.format(
                    self.root_link))
        if self.root_link in self.parent:
            raise WorkspaceModelError('link geometry root_link carries a parent joint')
        for name in self.link_order:
            seen = set()
            walker = name
            while walker in self.parent:
                if walker in seen:
                    raise WorkspaceModelError('link geometry parents form a cycle')
                seen.add(walker)
                walker = self.parent[walker]
                if walker not in self.volumes:
                    raise WorkspaceModelError(
                        'link geometry parent_link {!r} is not a declared link'.format(
                            walker))
            if walker != self.root_link:
                raise WorkspaceModelError(
                    'link geometry link {!r} is not rooted at {!r}'.format(
                        name, self.root_link))

    @staticmethod
    def _joint(entry, link_name):
        context = 'link geometry {} parent_joint'.format(link_name)
        if not isinstance(entry, dict) or 'type' not in entry:
            raise WorkspaceModelError('{} must declare a type'.format(context))
        kind = entry['type']
        if kind not in ('revolute', 'fixed'):
            raise WorkspaceModelError(
                '{}: type must be revolute or fixed, found {!r}'.format(context, kind))
        expected = ('name', 'type', 'origin_xyz', 'origin_rpy')
        if kind == 'revolute':
            expected = expected + ('axis', 'limit_lower', 'limit_upper')
        exact_keys(entry, expected, context)
        joint = {
            'name': _string(entry, 'name', context),
            'type': kind,
            'origin_xyz': _number_list(entry, 'origin_xyz', context, 3),
            'origin_rpy': _number_list(entry, 'origin_rpy', context, 3),
        }
        if kind == 'revolute':
            axis = _number_list(entry, 'axis', context, 3)
            if abs(float(np.linalg.norm(axis)) - 1.0) > 1e-9:
                raise WorkspaceModelError('{}: axis must be a unit vector'.format(context))
            joint['axis'] = axis
            joint['limit_lower'] = _number(entry, 'limit_lower', context)
            joint['limit_upper'] = _number(entry, 'limit_upper', context)
            if not joint['limit_lower'] < joint['limit_upper']:
                raise WorkspaceModelError('{}: limits must be ordered'.format(context))
        return joint


def _load_link_geometry(path: Path) -> _LinkGeometry:
    text = read_bounded_regular_text(path, 'the generated link geometry')
    document = load_strict_yaml(text)
    return _LinkGeometry(document, path)


# ---------------------------------------------------------------------------
# The SRDF allowed-collision matrix, read rather than restated
# ---------------------------------------------------------------------------

def _read_disabled_pairs(srdf_path: Path, arm_ids) -> set:
    """
    Read the disable_collisions entries out of the SRDF xacro and its includes.

    The allowed-collision matrix comes from the SRDF, never from a hand-written
    list.  Includes are resolved by basename inside the SRDF's own directory, so
    no package index and no ROS environment is needed.
    """
    seen_files = []
    pending = [srdf_path]
    pairs = set()
    while pending:
        current = pending.pop(0)
        if current in seen_files:
            continue
        seen_files.append(current)
        text = read_bounded_regular_text(current, 'the SRDF source {}'.format(current.name))
        try:
            root = ElementTree.fromstring(text)
        except ElementTree.ParseError as error:
            raise WorkspaceModelError(
                'the SRDF source {} is not well-formed XML'.format(current.name)) from error
        for element in root.iter():
            tag = element.tag.rsplit('}', 1)[-1]
            if tag == 'include':
                filename = element.attrib.get('filename', '')
                candidate = srdf_path.parent / Path(filename).name
                if candidate.is_file():
                    pending.append(candidate)
            elif tag == 'disable_collisions':
                first = element.attrib.get('link1')
                second = element.attrib.get('link2')
                if first is None or second is None:
                    raise WorkspaceModelError(
                        'a disable_collisions entry in {} names no link pair'.format(
                            current.name))
                for arm_id in arm_ids:
                    left = first.replace('${arm_id}', arm_id)
                    right = second.replace('${arm_id}', arm_id)
                    if '${' in left or '${' in right:
                        continue
                    pairs.add(frozenset((left, right)))
    if not pairs:
        raise WorkspaceModelError(
            'the SRDF source carries no disable_collisions entries; the '
            'allowed-collision matrix would be empty')
    return pairs


# ---------------------------------------------------------------------------
# The joint-limit policy, referenced rather than restated
# ---------------------------------------------------------------------------

def _read_joint_limits(policy_path: Path):
    text = read_bounded_regular_text(policy_path, 'the joint limit policy')
    document = load_strict_yaml(text)
    if not isinstance(document, dict):
        raise WorkspaceModelError('the joint limit policy must be a mapping')
    for key in ('schema_version', 'position_lower', 'position_upper'):
        if key not in document:
            raise WorkspaceModelError(
                'the joint limit policy carries no {}'.format(key))
    if _integer(document, 'schema_version', 'joint limit policy') != SCHEMA_VERSION:
        raise WorkspaceModelError('unsupported joint limit policy version')
    lower = _number_list(document, 'position_lower', 'joint limit policy', JOINT_COUNT)
    upper = _number_list(document, 'position_upper', 'joint limit policy', JOINT_COUNT)
    if any(low >= high for low, high in zip(lower, upper)):
        raise WorkspaceModelError('the joint limit policy bounds are not ordered')
    return lower, upper


# ---------------------------------------------------------------------------
# CONTRACT A / C / D - the loaded model and the checking policy
# ---------------------------------------------------------------------------

class CellModel:
    """The cell, loaded once, validated completely, then never mutated."""

    def __init__(self, state):
        self.__dict__.update(state)

    # -- loading ----------------------------------------------------------

    @classmethod
    def load(cls, cell_model_path, *, profile: str) -> 'CellModel':
        """
        Load and validate a cell model.

        ``profile`` is ``"dual"`` or ``"single"``.  Raises
        :class:`WorkspaceModelError` on ANY validation failure; there is no
        partial load and no degraded mode.
        """
        if profile not in ('dual', 'single'):
            raise WorkspaceModelError(
                "profile must be 'dual' or 'single', found {!r}".format(profile))
        path = Path(cell_model_path)
        text = read_bounded_regular_text(path, 'the cell model')
        document = load_strict_yaml(text, BOOLEAN_KEY_PATHS)
        digest = hashlib.sha256(text.encode('utf-8')).hexdigest()
        builder = _Loader(document, path.resolve(), profile, digest)
        return cls(builder.build())

    # -- identity ---------------------------------------------------------

    def arm_ids(self) -> tuple:
        """Return the declared arm ids, in declaration order."""
        return tuple(arm['arm_id'] for arm in self._arms)

    def urdf_sha256(self) -> str:
        """Return the generated-URDF digest, for the caller's own interlock."""
        return self._geometry.urdf_sha256

    def xacro_args(self) -> dict:
        """Return the xacro arguments the recorded URDF digest was produced with."""
        return dict(self._geometry.xacro_args)

    def allowed_volume(self) -> AllowedVolume:
        """Return the measured cell box the arms must stay inside."""
        return self._allowed_volume

    def diagnostics(self) -> tuple:
        """Return the load-time facts a reader needs; none is a defect."""
        return self._diagnostics

    def model_identity(self) -> tuple:
        """Return the (model_id, revision, sha256) triple every result carries."""
        return (self._model_id, self._revision, self._sha256)

    # -- checking ---------------------------------------------------------

    def check_configuration(self, q, *, first_violation: bool = False) -> CheckResult:
        """Check one configuration; q maps arm_id to seven joint positions."""
        sample = self._sample(q)
        contacts, minimum = self._evaluate(sample, 0.0, first_violation)
        return self._result(contacts, minimum, 0 if contacts else None, 1)

    def check_path(self, waypoints, *, first_violation: bool = False) -> CheckResult:
        """Check a resampled path, both endpoints included, at swept margins."""
        if not isinstance(waypoints, Sequence) or isinstance(waypoints, (str, bytes)):
            raise WorkspaceModelError('waypoints must be a sequence of configurations')
        if len(waypoints) < 1:
            raise WorkspaceModelError('a path needs at least one waypoint')
        samples = self._resample([self._sample(point) for point in waypoints])
        return self._check_samples(samples, first_violation)

    def check_jog(self, arm_id: str, q_now, joint_index: int, delta: float) -> JogResult:
        """
        Single-joint relative move, clamped toward zero rather than refused.

        The requested ``delta`` is reduced to the largest safe whole multiple of
        ``policy.max_joint_step_rad``; if even the first step is unsafe the jog
        is refused with ``limiting`` set.
        """
        if arm_id not in self.arm_ids():
            raise WorkspaceModelError(
                'unknown arm_id {!r}; the model declares {}'.format(
                    arm_id, list(self.arm_ids())))
        if isinstance(joint_index, bool) or not isinstance(joint_index, int):
            raise WorkspaceModelError('joint_index must be an integer')
        if not 0 <= joint_index < JOINT_COUNT:
            raise WorkspaceModelError(
                'joint_index must be 0..{}, found {}'.format(JOINT_COUNT - 1, joint_index))
        if isinstance(delta, bool) or not isinstance(delta, (int, float)):
            raise WorkspaceModelError('delta must be numeric')
        delta = float(delta)
        if not math.isfinite(delta):
            raise WorkspaceModelError('delta must be finite')
        start = self._sample(q_now)
        step = self._policy['max_joint_step_rad']
        magnitude = abs(delta)
        direction = 0.0 if magnitude == 0.0 else math.copysign(1.0, delta)
        whole_steps = int(math.floor(magnitude / step + 1e-12))
        offsets = [0.0]
        offsets.extend(direction * step * (index + 1) for index in range(whole_steps))
        if magnitude > 0.0 and abs(offsets[-1] - delta) > 1e-12:
            offsets.append(delta)
        arm_position = self.arm_ids().index(arm_id)
        samples = []
        for offset in offsets:
            values = np.array(start, dtype=float)
            values[arm_position][joint_index] += offset
            samples.append(values)
        accepted = -1
        failure = None
        minimum = math.inf
        margin_extra = self._margins['swept_path_extra']
        for index, sample in enumerate(samples):
            contacts, sample_minimum = self._evaluate(sample, margin_extra, False)
            if contacts:
                failure = (index, contacts, sample_minimum)
                break
            minimum = min(minimum, sample_minimum)
            accepted = index
        if accepted < 0:
            index, contacts, sample_minimum = failure
            return JogResult(
                allowed=False,
                q_target=tuple(float(value) for value in start[arm_position]),
                clamped=False,
                limiting=contacts[0],
                result=self._result(contacts, sample_minimum, index, index + 1),
            )
        evaluated = accepted + 1 if failure is None else failure[0] + 1
        return JogResult(
            allowed=True,
            q_target=tuple(float(value) for value in samples[accepted][arm_position]),
            clamped=accepted != len(samples) - 1,
            limiting=None if failure is None else failure[1][0],
            result=self._result((), minimum, None, evaluated),
        )

    # -- internals --------------------------------------------------------

    def _result(self, contacts, minimum, sample_index, samples_evaluated):
        return CheckResult(
            ok=not contacts,
            min_clearance=minimum,
            contacts=tuple(contacts),
            sample_index=sample_index if contacts else None,
            samples_evaluated=samples_evaluated,
            model_id=self._model_id,
            model_revision=self._revision,
            model_sha256=self._sha256,
        )

    def _check_samples(self, samples, first_violation):
        collected = []
        minimum = math.inf
        first_index = None
        for index, sample in enumerate(samples):
            contacts, sample_minimum = self._evaluate(
                sample, self._margins['swept_path_extra'], first_violation)
            minimum = min(minimum, sample_minimum)
            if contacts:
                if first_index is None:
                    first_index = index
                collected.extend(contacts)
                if first_violation:
                    break
        collected = self._sorted(collected)
        return self._result(collected, minimum, first_index, index + 1)

    @staticmethod
    def _sorted(contacts):
        return tuple(sorted(
            contacts, key=lambda item: (item.distance - item.required, item.kind,
                                        item.a, item.b)))

    def _sample(self, q):
        """Validate one multi-arm configuration into an array of joint vectors."""
        if not isinstance(q, Mapping):
            raise WorkspaceModelError(
                'a configuration maps arm_id to seven joint positions')
        declared = self.arm_ids()
        if set(q) != set(declared):
            raise WorkspaceModelError(
                'the configuration names arms {} but the model declares {}; a request '
                'that does not match the loaded profile is refused'.format(
                    sorted(q), list(declared)))
        rows = []
        for arm_id in declared:
            values = q[arm_id]
            if (isinstance(values, (str, bytes)) or not isinstance(values, Sequence)
                    or len(values) != JOINT_COUNT):
                raise WorkspaceModelError(
                    'arm {!r}: expected {} joint positions ordered joint1..joint{}'.format(
                        arm_id, JOINT_COUNT, JOINT_COUNT))
            row = []
            for value in values:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise WorkspaceModelError(
                        'arm {!r}: joint positions must be numbers'.format(arm_id))
                converted = float(value)
                if not math.isfinite(converted):
                    raise WorkspaceModelError(
                        'arm {!r}: joint positions must be finite'.format(arm_id))
                row.append(converted)
            rows.append(row)
        return np.array(rows, dtype=float)

    def _resample(self, waypoints):
        step = self._policy['max_joint_step_rad']
        samples = [waypoints[0]]
        for previous, current in zip(waypoints, waypoints[1:]):
            span = float(np.abs(current - previous).max())
            count = max(1, int(math.ceil(span / step - 1e-12)))
            for index in range(1, count + 1):
                samples.append(previous + (current - previous) * (index / count))
        return samples

    def _transforms(self, sample):
        """Forward kinematics: every link's pose in the cell frame."""
        geometry = self._geometry
        transforms = {geometry.root_link: self._cell_from_root}
        for name in geometry.link_order:
            if name in transforms:
                continue
            chain = []
            walker = name
            while walker not in transforms:
                chain.append(walker)
                walker = geometry.parent[walker]
            for link_name in reversed(chain):
                joint = geometry.joints[link_name]
                step = homogeneous(rotation_from_rpy(*joint['origin_rpy']),
                                   joint['origin_xyz'])
                if joint['type'] == 'revolute':
                    arm_position, joint_index = self._actuated[joint['name']]
                    angle = sample[arm_position][joint_index]
                    step = step @ homogeneous(
                        rotation_from_rpy(0.0, 0.0, angle), np.zeros(3))
                transforms[link_name] = transforms[geometry.parent[link_name]] @ step
        return transforms

    def _place(self, sample):
        """Every checked volume's cell-frame endpoints, as arrays in a fixed order."""
        transforms = self._transforms(sample)
        count = len(self._volume_list)
        ends_a = np.empty((count, 3))
        ends_b = np.empty((count, 3))
        boxes = {}
        for index, (volume_id, link_name, volume) in enumerate(self._volume_list):
            transform = transforms[link_name]
            rotation = transform[:3, :3]
            translation = transform[:3, 3]
            ends_a[index] = rotation @ volume.a + translation
            ends_b[index] = rotation @ volume.b + translation
            if volume.kind == 'box':
                boxes[volume_id] = (ends_a[index].copy(), rotation,
                                    volume.size / 2.0)
        return ends_a, ends_b, boxes

    def _evaluate(self, sample, margin_extra, first_violation):
        try:
            return self._evaluate_inner(sample, margin_extra, first_violation)
        except GeometryError as error:
            raise WorkspaceModelError(
                'the checker produced a non-finite distance ({}); the only safe verdict '
                'is refusal'.format(error)) from error

    def _evaluate_inner(self, sample, margin_extra, first_violation):
        contacts = []
        minimum = math.inf

        # Step 1 - joint limits.  First because a configuration outside the joint
        # box is not merely unsafe but meaningless: the forward kinematics would
        # place links where the robot cannot put them.
        lower, upper = self._joint_limits
        for arm_position, arm in enumerate(self._arms):
            for joint_index in range(JOINT_COUNT):
                value = sample[arm_position][joint_index]
                if value < lower[joint_index] or value > upper[joint_index]:
                    past = min(value - lower[joint_index], upper[joint_index] - value)
                    minimum = min(minimum, past)
                    contacts.append(Contact(
                        kind='joint_limit',
                        a='{}_joint{}'.format(arm['arm_id'], joint_index + 1),
                        b='',
                        distance=past,
                        required=0.0,
                        arm_id=arm['arm_id']))
                    if first_violation:
                        return self._sorted(contacts), minimum
        ends_a, ends_b, boxes = self._place(sample)
        radii = self._radii

        def _capsule_step(index_pair, meta, margin, kind):
            nonlocal minimum
            first, second = index_pair
            if not len(first):
                return False
            distances = segment_segment_distance_batch(
                ends_a[first], ends_b[first], ends_a[second], ends_b[second])
            clearances = distances - radii[first] - radii[second]
            minimum = min(minimum, float(clearances.min()) - margin)
            for position in np.nonzero(clearances < margin)[0]:
                volume_a, volume_b, arm_id = meta[int(position)]
                contacts.append(Contact(kind=kind, a=volume_a, b=volume_b,
                                        distance=float(clearances[position]),
                                        required=margin, arm_id=arm_id))
                if first_violation:
                    return True
            return False

        # Step 2a - intra-arm pairs: the SRDF matrix plus the recorded deltas.
        margin = self._margins['self_collision'] + margin_extra
        if _capsule_step(self._intra_index, self._intra_meta, margin, 'self'):
            return self._sorted(contacts), minimum

        # Step 2b - the pedestal against each arm, same margin and same matrix route.
        for structure_id, volume_id, arm_id in self._structure_pairs:
            centre, rotation, half = boxes[structure_id]
            index = self._volume_position[volume_id]
            distance = segment_box_distance(ends_a[index], ends_b[index], centre,
                                            rotation, half)
            clearance = distance - radii[index]
            minimum = min(minimum, clearance - margin)
            if clearance < margin:
                contacts.append(Contact(kind='self', a=structure_id, b=volume_id,
                                        distance=clearance, required=margin,
                                        arm_id=arm_id))
                if first_violation:
                    return self._sorted(contacts), minimum

        # Step 3 - cross-arm, the only cross-arm protection in this design.
        if self._policy['cross_arm_enabled']:
            margin = self._margins['cross_arm'] + margin_extra
            if _capsule_step(self._cross_index, self._cross_meta, margin, 'cross_arm'):
                return self._sorted(contacts), minimum

        margin = self._margins['environment'] + margin_extra
        # Step 4a - containment: the arm must stay INSIDE the allowed volume.  The
        # clearance is measured from the inside and is negative on protrusion, so
        # the violation test is the same "< margin" comparison as everywhere else.
        if self._policy['containment_enabled'] and len(self._containment_index):
            rows = self._containment_index
            low = np.minimum(ends_a[rows], ends_b[rows])
            high = np.maximum(ends_a[rows], ends_b[rows])
            radius = radii[rows][:, None]
            values = np.empty((len(rows), 6))
            values[:, 0::2] = low - radius - self._box_lower[None, :]
            values[:, 1::2] = self._box_upper[None, :] - high - radius
            masked = np.where(self._containment_mask, values, math.inf)
            best = masked.min(axis=1)
            faces = masked.argmin(axis=1)
            if not np.all(np.isfinite(best)):
                raise GeometryError('containment')
            minimum = min(minimum, float(best.min()) - margin)
            for position in np.nonzero(best < margin)[0]:
                volume_id, arm_id = self._containment_meta[int(position)]
                contacts.append(Contact(
                    kind='containment', a=volume_id,
                    b='{}.{}'.format(self._allowed_volume.id,
                                     BOX_FACES[int(faces[position])]),
                    distance=float(best[position]), required=margin, arm_id=arm_id))
                if first_violation:
                    return self._sorted(contacts), minimum

        # Step 4b - environment: the arm must stay OUTSIDE each declared solid.
        if self._policy['environment_enabled'] and self._environment:
            for volume_id, arm_id in self._moving_volumes:
                index = self._volume_position[volume_id]
                placed = (ends_a[index], ends_b[index], radii[index])
                for solid in self._environment:
                    clearance = self._solid_clearance(placed, solid)
                    minimum = min(minimum, clearance - margin)
                    if clearance < margin:
                        contacts.append(Contact(kind='environment', a=volume_id,
                                                b=solid['id'], distance=clearance,
                                                required=margin, arm_id=arm_id))
                        if first_violation:
                            return self._sorted(contacts), minimum

        # Step 5 - keep-out zones, per arm, enabled only.
        margin = self._margins['keep_out'] + margin_extra
        if self._active_zones:
            for volume_id, arm_id in self._moving_volumes:
                index = self._volume_position[volume_id]
                placed = (ends_a[index], ends_b[index], radii[index])
                for zone in self._active_zones:
                    if arm_id not in zone['arms']:
                        continue
                    clearance = self._solid_clearance(placed, zone)
                    minimum = min(minimum, clearance - margin)
                    if clearance < margin:
                        contacts.append(Contact(kind='keep_out', a=volume_id,
                                                b=zone['id'], distance=clearance,
                                                required=margin, arm_id=arm_id))
                        if first_violation:
                            return self._sorted(contacts), minimum
        return self._sorted(contacts), minimum

    @staticmethod
    def _solid_clearance(placed_volume, solid):
        point_a, point_b, radius = placed_volume[0], placed_volume[1], placed_volume[2]
        if solid['kind'] == 'box':
            return segment_box_distance(point_a, point_b, solid['centre'],
                                        solid['rotation'], solid['half_extents']) - radius
        if solid['kind'] == 'sphere':
            return (segment_point_distance(point_a, point_b, solid['centre'])
                    - radius - solid['radius'])
        if solid['kind'] == 'capsule':
            return (segment_segment_distance(point_a, point_b, solid['a'], solid['b'])
                    - radius - solid['radius'])
        return (segment_halfspace_distance(point_a, point_b, solid['normal'],
                                           solid['offset']) - radius)


class _Loader:
    """Validates one cell model completely, or raises.  Nothing loads partially."""

    def __init__(self, document, path, profile, digest):
        self.document = document
        self.path = path
        self.profile = profile
        self.digest = digest
        self.diagnostics = []

    # -- entry point ------------------------------------------------------

    def build(self):
        document = exact_keys(self.document, TOP_LEVEL_KEYS, 'cell model')
        if _integer(document, 'schema_version', 'cell model') != SCHEMA_VERSION:
            raise WorkspaceModelError(
                'cell model schema_version must be {}, found {!r}'.format(
                    SCHEMA_VERSION, document['schema_version']))
        model_id = _string(document, 'model_id', 'cell model', IDENTIFIER_PATTERN)
        revision = _integer(document, 'revision', 'cell model')
        if revision < 1:
            raise WorkspaceModelError('cell model revision must be at least 1')
        measured_on = _string(document, 'measured_on', 'cell model', DATE_PATTERN)
        try:
            datetime.date.fromisoformat(measured_on)
        except ValueError as error:
            raise WorkspaceModelError(
                'cell model measured_on {!r} is not a real calendar date'.format(
                    measured_on)) from error
        _string(document, 'measured_by', 'cell model', minimum=1, maximum=64)
        units = exact_keys(document['units'], ('length', 'angle'), 'units')
        if _string(units, 'length', 'units') != 'm':
            raise WorkspaceModelError("units.length must be 'm'")
        if _string(units, 'angle', 'units') != 'rad':
            raise WorkspaceModelError("units.angle must be 'rad'")

        geometry, srdf_path, policy_path = self._sources(document['sources'])
        arms = self._arms(document['arms'], geometry)
        margins = self._margins(document['margins'])
        allowed_volume = self._allowed_volume(document['allowed_volume'])
        cell_frame = self._cell_frame(document['cell_frame'], arms)
        identifiers = {allowed_volume.id}
        environment = self._solids(document['environment'], 'environment', identifiers)
        keep_out = self._keep_out(document['keep_out'], arms, identifiers)
        policy = self._policy(document['policy'], arms, geometry)

        # A.3.7 - the built-in inflation is documentation, not an applied margin.
        if abs(margins['urdf_builtin_inflation'] - geometry.safety_distance) > (
                INFLATION_TOLERANCE):
            raise WorkspaceModelError(
                'margins.urdf_builtin_inflation ({}) must equal link geometry '
                'source.safety_distance ({}); it is documentation, not an applied '
                'margin'.format(margins['urdf_builtin_inflation'],
                                geometry.safety_distance))
        self._base_pose_rules(arms, allowed_volume, margins)
        self._ceiling_diagnostics(allowed_volume)

        joint_limits = _read_joint_limits(policy_path)
        disabled = _read_disabled_pairs(srdf_path, [arm['arm_id'] for arm in arms])
        state = self._geometry_sets(arms, geometry, disabled, policy, allowed_volume,
                                    cell_frame)
        self._static_diagnostics(state, geometry, allowed_volume, environment, arms)
        state.update({
            '_box_lower': np.array([allowed_volume.x_min, allowed_volume.y_min,
                                    allowed_volume.z_min], dtype=float),
            '_box_upper': np.array([allowed_volume.x_max, allowed_volume.y_max,
                                    allowed_volume.z_max], dtype=float),
            '_active_zones': tuple(zone for zone in keep_out if zone['enabled']),
            '_model_id': model_id,
            '_revision': revision,
            '_sha256': self.digest,
            '_profile': self.profile,
            '_arms': arms,
            '_geometry': geometry,
            '_margins': margins,
            '_allowed_volume': allowed_volume,
            '_environment': environment,
            '_keep_out': keep_out,
            '_policy': policy,
            '_joint_limits': joint_limits,
            '_diagnostics': tuple(self.diagnostics),
        })
        return state

    # -- sources ----------------------------------------------------------

    def _sources(self, sources):
        sources = exact_keys(sources, SOURCES_KEYS, 'sources')
        for key in ('urdf_xacro', 'srdf_xacro', 'joint_limit_policy', 'link_geometry'):
            value = _string(sources, key, 'sources', PATH_PATTERN, minimum=1, maximum=256)
            if value.startswith('/') or '..' in Path(value).parts:
                raise WorkspaceModelError(
                    'sources.{} must be a relative path with no parent component'.format(
                        key))
            _string(sources, key + '_sha256', 'sources', HASH_PATTERN)
        directory = self.path.parent
        link_geometry_path = directory / sources['link_geometry']
        repository = self._repository_root(directory, sources)
        resolved = {
            'link_geometry': link_geometry_path,
            'urdf_xacro': repository / sources['urdf_xacro'],
            'srdf_xacro': repository / sources['srdf_xacro'],
            'joint_limit_policy': repository / sources['joint_limit_policy'],
        }
        for key, path in resolved.items():
            if not path.is_file():
                raise WorkspaceModelError(
                    'sources.{} resolves to {}, which is not a file'.format(key, path))
            actual = _sha256_of_file(path, 'sources.{}'.format(key))
            recorded = sources[key + '_sha256']
            if actual != recorded:
                raise WorkspaceModelError(
                    '{}: recorded hash {} does not match {} ({}); regenerate the model '
                    'against the current description'.format(
                        key + '_sha256', recorded, path, actual))
        geometry = _load_link_geometry(resolved['link_geometry'])
        if geometry.urdf_xacro != sources['urdf_xacro']:
            raise WorkspaceModelError(
                'sources.urdf_xacro ({}) and the link geometry source ({}) name '
                'different descriptions'.format(sources['urdf_xacro'],
                                                geometry.urdf_xacro))
        return geometry, resolved['srdf_xacro'], resolved['joint_limit_policy']

    def _repository_root(self, directory, sources):
        wanted = [sources['urdf_xacro'], sources['srdf_xacro'],
                  sources['joint_limit_policy']]
        for candidate in [directory] + list(directory.parents):
            if all((candidate / relative).is_file() for relative in wanted):
                return candidate
        raise WorkspaceModelError(
            'no directory at or above {} contains all of {}; the cell model can only '
            'be loaded from a tree that carries the description it was derived '
            'from'.format(directory, wanted))

    # -- schema sections --------------------------------------------------

    def _cell_frame(self, cell_frame, arms):
        cell_frame = exact_keys(cell_frame, ('name', 'anchors'), 'cell_frame')
        if _string(cell_frame, 'name', 'cell_frame') != 'cell':
            raise WorkspaceModelError("cell_frame.name must be 'cell' in v1")
        anchors = exact_keys(cell_frame['anchors'], ('dual', 'single'),
                             'cell_frame.anchors')
        parsed = {}
        for name in ('dual', 'single'):
            context = 'cell_frame.anchors.{}'.format(name)
            anchor = exact_keys(anchors[name], ('link', 'xyz', 'rpy'), context)
            parsed[name] = {
                'link': _string(anchor, 'link', context),
                'xyz': _number_list(anchor, 'xyz', context, 3),
                'rpy': _angles(anchor, 'rpy', context),
            }
        # A.3.8 - the dual anchor is the identity, which is what makes the cell
        # frame coincide with base_link.
        if (float(np.abs(parsed['dual']['xyz']).max()) > 0.0
                or float(np.abs(parsed['dual']['rpy']).max()) > 0.0):
            raise WorkspaceModelError('cell_frame.anchors.dual must be the identity in v1')
        if parsed['dual']['link'] != 'base_link':
            raise WorkspaceModelError(
                "cell_frame.anchors.dual.link must be 'base_link'")
        if parsed['single']['link'] != '{arm_id}_link0':
            raise WorkspaceModelError(
                "cell_frame.anchors.single.link must be the literal '{arm_id}_link0'")
        if self.profile == 'single':
            arm = arms[0]
            expected = -arm['urdf_xyz']
            if float(np.abs(parsed['single']['xyz'] - expected).max()) > (
                    ANTISYMMETRY_TOLERANCE):
                raise WorkspaceModelError(
                    "cell_frame.anchors.single is inconsistent with arm '{}': expected "
                    'xyz {}, found {}'.format(arm['arm_id'], list(expected),
                                              list(parsed['single']['xyz'])))
            if float(np.abs(parsed['single']['rpy'] + arm['urdf_rpy']).max()) > (
                    ANTISYMMETRY_TOLERANCE):
                raise WorkspaceModelError(
                    "cell_frame.anchors.single is inconsistent with arm '{}': rpy must "
                    'be the inverse of the arm mounting'.format(arm['arm_id']))
        return parsed

    def _arms(self, arms, geometry):
        if not isinstance(arms, list) or not 1 <= len(arms) <= 2:
            raise WorkspaceModelError('arms must be a list of one or two entries')
        if self.profile == 'dual' and len(arms) != 2:
            raise WorkspaceModelError(
                "profile 'dual' requires exactly two arms, found {}".format(len(arms)))
        if self.profile == 'single' and len(arms) != 1:
            raise WorkspaceModelError(
                "profile 'single' requires exactly one arm, found {}".format(len(arms)))
        parsed = []
        seen = set()
        for index, entry in enumerate(arms):
            context = 'arms[{}]'.format(index)
            entry = exact_keys(entry, ARM_KEYS, context)
            arm_id = _string(entry, 'arm_id', context, ARM_ID_PATTERN, minimum=1,
                             maximum=64)
            if arm_id in seen:
                raise WorkspaceModelError("duplicate arm_id '{}'".format(arm_id))
            seen.add(arm_id)
            base_link = _string(entry, 'base_link', context)
            if base_link != '{}_link0'.format(arm_id):
                raise WorkspaceModelError(
                    "{}: base_link must be '{}_link0'".format(context, arm_id))
            if base_link not in geometry.volumes:
                raise WorkspaceModelError(
                    "{}: base_link '{}' does not exist in the link geometry".format(
                        context, base_link))
            urdf = exact_keys(entry['urdf_base_pose'], ('xyz', 'rpy'),
                              context + '.urdf_base_pose')
            measured = exact_keys(entry['measured_base_pose'], MEASURED_POSE_KEYS,
                                  context + '.measured_base_pose')
            arm = {
                'arm_id': arm_id,
                'base_link': base_link,
                'urdf_xyz': _number_list(urdf, 'xyz', context + '.urdf_base_pose', 3),
                'urdf_rpy': _angles(urdf, 'rpy', context + '.urdf_base_pose'),
                'measured_xyz': _number_list(measured, 'xyz',
                                             context + '.measured_base_pose', 3),
                'measured_rpy': _angles(measured, 'rpy',
                                        context + '.measured_base_pose'),
                'tolerance_m': _number(measured, 'tolerance_m',
                                       context + '.measured_base_pose'),
                'tolerance_rad': _number(measured, 'tolerance_rad',
                                         context + '.measured_base_pose'),
                'measurement_status': _enum(measured, 'measurement_status',
                                            context + '.measured_base_pose',
                                            MEASUREMENT_STATUS),
            }
            if arm['tolerance_m'] <= 0.0 or arm['tolerance_rad'] <= 0.0:
                raise WorkspaceModelError(
                    '{}.measured_base_pose tolerances must be positive'.format(context))
            arm['end_effector'] = self._end_effector(entry['end_effector'], arm_id,
                                                     context)
            parsed.append(arm)
        # A.3.5 - the URDF-disagreement rule.
        for arm in parsed:
            for axis, index in (('x', 0), ('y', 1), ('z', 2)):
                delta = abs(arm['measured_xyz'][index] - arm['urdf_xyz'][index])
                if delta > arm['tolerance_m']:
                    raise WorkspaceModelError(
                        "arm '{}': measured base pose disagrees with the URDF by {} m "
                        'in {} (tolerance {} m); see doc/CONTRACT.md section '
                        "'The URDF-disagreement rule' -- stop and report, do not edit "
                        'the scene spec'.format(arm['arm_id'], delta, axis,
                                                arm['tolerance_m']))
                difference = (arm['measured_rpy'][index] - arm['urdf_rpy'][index]
                              + math.pi) % (2.0 * math.pi) - math.pi
                if abs(difference) > arm['tolerance_rad']:
                    raise WorkspaceModelError(
                        "arm '{}': measured base orientation disagrees with the URDF by "
                        '{} rad about {} (tolerance {} rad); see doc/CONTRACT.md section '
                        "'The URDF-disagreement rule' -- stop and report, do not edit "
                        'the scene spec'.format(arm['arm_id'], abs(difference), axis,
                                                arm['tolerance_rad']))
        # A.3.8 - antisymmetry of the two mountings about the cell origin.
        if len(parsed) == 2:
            total = parsed[0]['urdf_xyz'] + parsed[1]['urdf_xyz']
            if float(np.abs(total).max()) > ANTISYMMETRY_TOLERANCE:
                raise WorkspaceModelError(
                    "arms' urdf_base_pose are not symmetric about the cell frame "
                    'origin: {} vs {}'.format(list(parsed[0]['urdf_xyz']),
                                              list(parsed[1]['urdf_xyz'])))
        return tuple(parsed)

    def _end_effector(self, entry, arm_id, context):
        context = context + '.end_effector'
        entry = exact_keys(entry, END_EFFECTOR_KEYS, context)
        present = _boolean(entry, 'present', context)
        profile = _enum(entry, 'profile', context, END_EFFECTOR_PROFILES)
        volumes = entry['volumes']
        if not isinstance(volumes, list):
            raise WorkspaceModelError('{}.volumes must be a list'.format(context))
        # A.3.11 - the profile carries the geometry; the switch does not.
        if (profile != 'none') != bool(volumes):
            raise WorkspaceModelError(
                "arm '{}': end_effector.profile is '{}' but volumes has {} entries; a "
                "named profile carries its geometry and 'none' carries none".format(
                    arm_id, profile, len(volumes)))
        # A.3.18 - an enabled end effector must be fully derived.
        if present and profile == 'none':
            raise WorkspaceModelError(
                "arm '{}': end_effector.present is true but profile is 'none'; nothing "
                'is declared to check'.format(arm_id))
        parsed = []
        for index, volume in enumerate(volumes):
            volume_context = '{}.volumes[{}]'.format(context, index)
            volume = exact_keys(volume, END_EFFECTOR_VOLUME_KEYS, volume_context)
            identifier = _string(volume, 'id', volume_context, IDENTIFIER_PATTERN)
            if _string(volume, 'kind', volume_context) != 'capsule':
                raise WorkspaceModelError(
                    '{}: v1 end-effector volumes are capsules'.format(volume_context))
            status = _enum(volume, 'derivation_status', volume_context,
                           ('derived', 'to_be_derived'))
            _string(volume, 'provenance', volume_context, minimum=1, maximum=512)
            containment = _enum(volume, 'containment', volume_context,
                                ('exact', 'conservative'))
            margin = _number(volume, 'containment_margin', volume_context)
            point_a = _number_list(volume, 'a', volume_context, 3)
            point_b = _number_list(volume, 'b', volume_context, 3)
            radius = _number(volume, 'radius', volume_context)
            if present:
                if status != 'derived':
                    raise WorkspaceModelError(
                        "arm '{}': end_effector is ENABLED but volume '{}' of profile "
                        "'{}' still has derivation_status 'to_be_derived'. The checker "
                        'would model the mounted tool as a zero-size capsule and report '
                        'the most collision-prone part of the arm as clear. Derive the '
                        'enclosing capsule from the {} datasheet, record its provenance, '
                        "set derivation_status to 'derived' -- see doc/CONTRACT.md "
                        "section 'The switchable end effector'. Until then set "
                        'end_effector.present to false and do not mount the '
                        'tool.'.format(arm_id, identifier, profile, profile))
                if radius <= 0.0:
                    raise WorkspaceModelError(
                        '{}: an enabled end-effector volume needs a positive '
                        'radius'.format(volume_context))
                if float(np.linalg.norm(point_b - point_a)) < 1e-9:
                    raise WorkspaceModelError(
                        '{}: an enabled end-effector capsule needs a non-degenerate '
                        'segment'.format(volume_context))
                if margin < 0.0 or (containment == 'exact') != (margin == 0.0):
                    raise WorkspaceModelError(
                        '{}: containment_margin is zero exactly when containment is '
                        'exact'.format(volume_context))
            parsed.append({'id': identifier, 'a': point_a, 'b': point_b,
                           'radius': radius, 'derivation_status': status})
        return {'present': present, 'profile': profile, 'volumes': tuple(parsed)}

    def _allowed_volume(self, entry):
        entry = exact_keys(entry, ALLOWED_VOLUME_KEYS, 'allowed_volume')
        identifier = _string(entry, 'id', 'allowed_volume', IDENTIFIER_PATTERN)
        if _string(entry, 'frame', 'allowed_volume') != 'cell':
            raise WorkspaceModelError(
                "allowed_volume '{}': frame must be the declared cell frame "
                "'cell'".format(identifier))
        bounds = {}
        for key in ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'):
            bounds[key] = _number(entry, key, 'allowed_volume')
        for axis in 'xyz':
            low_key, high_key = axis + '_min', axis + '_max'
            if not bounds[low_key] < bounds[high_key]:
                raise WorkspaceModelError(
                    "allowed_volume '{}': {} ({}) must be strictly less than {} "
                    '({})'.format(identifier, low_key, bounds[low_key], high_key,
                                  bounds[high_key]))
        _enum(entry, 'measurement_status', 'allowed_volume', MEASUREMENT_STATUS)
        _string(entry, 'source_question', 'allowed_volume', SOURCE_QUESTION_PATTERN)
        self._source_question(entry['source_question'], 'allowed_volume')
        _string(entry, 'note', 'allowed_volume', maximum=256)
        return AllowedVolume(id=identifier, frame='cell', **bounds)

    @staticmethod
    def _source_question(value, context):
        parts = value.split(',')
        if len(set(parts)) != len(parts):
            raise WorkspaceModelError(
                "solid '{}': source_question '{}' repeats a question".format(
                    context, value))

    def _geometry_entry(self, entry, kind, context):
        if kind == 'box':
            pose = self._pose(entry['pose'], context)
            size = _number_list(entry, 'size', context, 3)
            if any(value <= 0.0 for value in size):
                raise WorkspaceModelError('{}: box size must be positive'.format(context))
            return {'kind': 'box', 'centre': pose[1], 'rotation': pose[0],
                    'half_extents': size / 2.0}
        if kind == 'cylinder':
            pose = self._pose(entry['pose'], context)
            length = _number(entry, 'length', context)
            radius = _number(entry, 'radius', context)
            if length <= 0.0 or radius <= 0.0:
                raise WorkspaceModelError(
                    '{}: cylinder length and radius must be positive'.format(context))
            axis = pose[0][:, 2]
            self.diagnostics.append(
                'solid {}: the cylinder is promoted to its minimal enclosing capsule, '
                'which adds a hemispherical cap of radius {} at each end'.format(
                    context, radius))
            return {'kind': 'capsule', 'a': pose[1] + 0.5 * length * axis,
                    'b': pose[1] - 0.5 * length * axis, 'radius': radius}
        if kind == 'sphere':
            pose = self._pose(entry['pose'], context)
            radius = _number(entry, 'radius', context)
            if radius <= 0.0:
                raise WorkspaceModelError(
                    '{}: sphere radius must be positive'.format(context))
            return {'kind': 'sphere', 'centre': pose[1], 'radius': radius}
        if kind == 'capsule':
            point_a = _number_list(entry, 'a', context, 3)
            point_b = _number_list(entry, 'b', context, 3)
            radius = _number(entry, 'radius', context)
            if radius <= 0.0:
                raise WorkspaceModelError(
                    '{}: capsule radius must be positive'.format(context))
            if float(np.linalg.norm(point_b - point_a)) < 1e-9:
                raise WorkspaceModelError(
                    '{}: a zero-length capsule segment is rejected rather than '
                    'degenerating into a sphere'.format(context))
            return {'kind': 'capsule', 'a': point_a, 'b': point_b, 'radius': radius}
        normal = _number_list(entry, 'normal', context, 3)
        norm = float(np.linalg.norm(normal))
        if abs(norm - 1.0) > UNIT_NORMAL_TOLERANCE:
            raise WorkspaceModelError(
                '{}: plane_halfspace normal must be a unit vector; it is rejected '
                'rather than normalised because normalising would silently move the '
                'plane'.format(context))
        return {'kind': 'plane_halfspace', 'normal': normal,
                'offset': _number(entry, 'offset', context)}

    @staticmethod
    def _pose(pose, context):
        pose = exact_keys(pose, ('xyz', 'rpy'), context + '.pose')
        translation = _number_list(pose, 'xyz', context + '.pose', 3)
        angles = _angles(pose, 'rpy', context + '.pose')
        return rotation_from_rpy(*angles), translation

    def _solids(self, entries, name, identifiers):
        if not isinstance(entries, list):
            raise WorkspaceModelError('{} must be a list'.format(name))
        parsed = []
        for index, entry in enumerate(entries):
            context = '{}[{}]'.format(name, index)
            if not isinstance(entry, dict) or 'kind' not in entry:
                raise WorkspaceModelError('{} must declare a kind'.format(context))
            kind = entry['kind']
            if kind not in GEOMETRY_KEYS:
                raise WorkspaceModelError(
                    '{}: unknown solid kind {!r}'.format(context, kind))
            exact_keys(entry, ENVIRONMENT_COMMON_KEYS + GEOMETRY_KEYS[kind], context)
            identifier = _string(entry, 'id', context, IDENTIFIER_PATTERN)
            if identifier in identifiers:
                raise WorkspaceModelError(
                    "duplicate solid id '{}': ids must be unique across "
                    'allowed_volume, environment and keep_out'.format(identifier))
            identifiers.add(identifier)
            if _string(entry, 'frame', context) != 'cell':
                raise WorkspaceModelError(
                    "solid '{}': frame '{}' is not the declared cell frame "
                    "'cell'".format(identifier, entry['frame']))
            _enum(entry, 'measurement_status', context, MEASUREMENT_STATUS)
            _string(entry, 'source_question', context, SOURCE_QUESTION_PATTERN)
            self._source_question(entry['source_question'], identifier)
            _string(entry, 'note', context, maximum=256)
            solid = self._geometry_entry(entry, kind, context)
            solid['id'] = identifier
            parsed.append(solid)
        return tuple(parsed)

    def _keep_out(self, entries, arms, identifiers):
        if not isinstance(entries, list):
            raise WorkspaceModelError('keep_out must be a list')
        declared = [arm['arm_id'] for arm in arms]
        parsed = []
        for index, entry in enumerate(entries):
            context = 'keep_out[{}]'.format(index)
            if not isinstance(entry, dict) or 'kind' not in entry:
                raise WorkspaceModelError('{} must declare a kind'.format(context))
            kind = entry['kind']
            if kind not in GEOMETRY_KEYS:
                raise WorkspaceModelError(
                    '{}: unknown zone kind {!r}'.format(context, kind))
            exact_keys(entry, KEEP_OUT_COMMON_KEYS + GEOMETRY_KEYS[kind], context)
            identifier = _string(entry, 'id', context, IDENTIFIER_PATTERN)
            if identifier in identifiers:
                raise WorkspaceModelError(
                    "duplicate solid id '{}': ids must be unique across "
                    'allowed_volume, environment and keep_out'.format(identifier))
            identifiers.add(identifier)
            if _string(entry, 'frame', context) != 'cell':
                raise WorkspaceModelError(
                    "solid '{}': frame '{}' is not the declared cell frame "
                    "'cell'".format(identifier, entry['frame']))
            applies = entry['applies_to']
            if isinstance(applies, str):
                if applies != 'all':
                    raise WorkspaceModelError(
                        "keep_out '{}' applies_to must be a list of arm ids or the "
                        "string 'all'".format(identifier))
                arm_set = tuple(declared)
            elif isinstance(applies, list) and applies:
                if len(set(applies)) != len(applies):
                    raise WorkspaceModelError(
                        "keep_out '{}' applies_to repeats an arm".format(identifier))
                for value in applies:
                    if not isinstance(value, str) or value not in declared:
                        raise WorkspaceModelError(
                            "keep_out '{}' applies_to names unknown arm '{}'; declared "
                            'arms are {}'.format(identifier, value, declared))
                arm_set = tuple(applies)
            else:
                raise WorkspaceModelError(
                    "keep_out '{}' applies_to must be a non-empty list or "
                    "'all'".format(identifier))
            zone = self._geometry_entry(entry, kind, context)
            zone['id'] = identifier
            zone['enabled'] = _boolean(entry, 'enabled', context)
            zone['arms'] = arm_set
            _string(entry, 'reason', context, minimum=1, maximum=256)
            _string(entry, 'source_question', context, SOURCE_QUESTION_PATTERN)
            self._source_question(entry['source_question'], identifier)
            parsed.append(zone)
        return tuple(parsed)

    def _margins(self, margins):
        margins = exact_keys(margins, MARGIN_KEYS, 'margins')
        parsed = {}
        for key in MARGIN_KEYS[:-1]:
            value = _number(margins, key, 'margins')
            if value < 0.0:
                raise WorkspaceModelError('margins.{} must be non-negative'.format(key))
            parsed[key] = value
        _string(margins, 'rationale', 'margins', minimum=1, maximum=512)
        return parsed

    def _policy(self, policy, arms, geometry):
        policy = exact_keys(policy, POLICY_KEYS, 'policy')
        if _string(policy, 'schema', 'policy') != 'checking_policy_v1':
            raise WorkspaceModelError("policy.schema must be 'checking_policy_v1'")
        mode = _string(policy, 'default_mode', 'policy')
        if mode != 'swept':
            raise WorkspaceModelError(
                "policy.default_mode must be 'swept' in v1, found '{}'".format(mode))
        step = _number(policy, 'max_joint_step_rad', 'policy')
        if step <= 0.0:
            raise WorkspaceModelError('policy.max_joint_step_rad must be positive')
        if not _boolean(policy, 'fail_closed', 'policy'):
            raise WorkspaceModelError('policy.fail_closed must be true in v1')
        self_collision = exact_keys(
            policy['self_collision'],
            ('acm_source', 'extra_disabled_pairs', 'extra_enabled_pairs'),
            'policy.self_collision')
        if _string(self_collision, 'acm_source', 'policy.self_collision') != 'srdf':
            raise WorkspaceModelError("policy.self_collision.acm_source must be 'srdf'")
        cross_arm = exact_keys(policy['cross_arm'], ('enabled',), 'policy.cross_arm')
        cross_enabled = _boolean(cross_arm, 'enabled', 'policy.cross_arm')
        if len(arms) == 2 and not cross_enabled:
            raise WorkspaceModelError(
                'policy.cross_arm.enabled must be true for a two-arm model')
        if len(arms) == 1 and cross_enabled:
            raise WorkspaceModelError(
                'policy.cross_arm.enabled must be false for a one-arm model')
        environment = exact_keys(policy['environment'], ('enabled',),
                                 'policy.environment')
        if not _boolean(environment, 'enabled', 'policy.environment'):
            raise WorkspaceModelError('policy.environment.enabled must be true in v1')
        containment = exact_keys(policy['containment'], ('enabled',),
                                 'policy.containment')
        if not _boolean(containment, 'enabled', 'policy.containment'):
            raise WorkspaceModelError('policy.containment.enabled must be true in v1')
        joint_limits = exact_keys(policy['joint_limits'], ('source',),
                                  'policy.joint_limits')
        if _string(joint_limits, 'source', 'policy.joint_limits') != 'joint_limit_policy':
            raise WorkspaceModelError(
                "policy.joint_limits.source must be 'joint_limit_policy'")
        return {
            'max_joint_step_rad': step,
            'cross_arm_enabled': cross_enabled,
            'environment_enabled': True,
            'containment_enabled': True,
            'extra_disabled_pairs': self._pair_deltas(
                self_collision['extra_disabled_pairs'], 'extra_disabled_pairs', geometry),
            'extra_enabled_pairs': self._pair_deltas(
                self_collision['extra_enabled_pairs'], 'extra_enabled_pairs', geometry),
        }

    @staticmethod
    def _pair_deltas(entries, name, geometry):
        if not isinstance(entries, list):
            raise WorkspaceModelError(
                'policy.self_collision.{} must be a list'.format(name))
        pairs = []
        for index, entry in enumerate(entries):
            context = 'policy.self_collision.{}[{}]'.format(name, index)
            entry = exact_keys(entry, ('a', 'b', 'reason'), context)
            first = _string(entry, 'a', context)
            second = _string(entry, 'b', context)
            for link in (first, second):
                if link not in geometry.volumes:
                    raise WorkspaceModelError(
                        "{} entry names unknown link '{}'".format(name, link))
            if first == second:
                raise WorkspaceModelError(
                    "{} entry names the same link twice: '{}'".format(name, first))
            _string(entry, 'reason', context, minimum=1, maximum=256)
            pairs.append(frozenset((first, second)))
        return tuple(pairs)

    # -- cross-field rules and derived sets --------------------------------

    def _base_pose_rules(self, arms, allowed_volume, margins):
        """A.3.17 - the arms are mounted inside their own permitted region."""
        slack = margins['environment']
        faces = (('x_min', 0, allowed_volume.x_min, 1.0),
                 ('x_max', 0, allowed_volume.x_max, -1.0),
                 ('y_min', 1, allowed_volume.y_min, 1.0),
                 ('y_max', 1, allowed_volume.y_max, -1.0))
        for arm in arms:
            position = arm['measured_xyz']
            nearest = None
            for name, axis, bound, direction in faces:
                distance = direction * (position[axis] - bound)
                if nearest is None or distance < nearest[1]:
                    nearest = (name, distance)
            if nearest[1] < slack:
                raise WorkspaceModelError(
                    "arm '{}': base position {} is not inside allowed_volume '{}' with "
                    'margins.environment ({}) of lateral slack; nearest face is {} at '
                    '{} m'.format(arm['arm_id'], list(position), allowed_volume.id,
                                  slack, nearest[0], nearest[1]))
            if abs(position[2] - allowed_volume.z_min) > BASE_PLANE_TOLERANCE:
                raise WorkspaceModelError(
                    "arm '{}': base z ({}) is not the declared table top "
                    'allowed_volume.z_min ({}); the table-top convention requires '
                    'co-planar bases on the mounting plane'.format(
                        arm['arm_id'], position[2], allowed_volume.z_min))

    def _ceiling_diagnostics(self, allowed_volume):
        """A.3.19 (a) and (b) - facts a reader needs; neither is a defect."""
        if allowed_volume.z_min != 0.0:
            self.diagnostics.append(
                'allowed_volume.z_min is {} rather than 0.0, so the mounting plane is '
                'not the table top; the table-top convention this schema is built '
                'around assumes they are the same plane'.format(allowed_volume.z_min))
        if allowed_volume.z_max < REACHABILITY_HEIGHT_BOUND:
            self.diagnostics.append(
                'allowed_volume.z_max is {} m, below the {} m reachability bound, so '
                'the ceiling face can bind on a reachable configuration'.format(
                    allowed_volume.z_max, REACHABILITY_HEIGHT_BOUND))

    def _geometry_sets(self, arms, geometry, disabled, policy, allowed_volume,
                       cell_frame):
        arm_ids = [arm['arm_id'] for arm in arms]
        owner = {}
        for link in geometry.link_order:
            for arm_id in arm_ids:
                if link.startswith(arm_id + '_'):
                    owner[link] = arm_id
        participating = [link for link in geometry.link_order
                         if link in owner or link == geometry.root_link]
        if self.profile == 'dual' and geometry.root_link in owner:
            raise WorkspaceModelError(
                'the dual profile expects a structural root link that belongs to no '
                'arm, but {} belongs to {}'.format(geometry.root_link,
                                                   owner[geometry.root_link]))
        if self.profile == 'single' and geometry.root_link != arms[0]['base_link']:
            # The single anchor is expressed in the arm's own link0 frame, so a
            # single-arm model needs the link geometry generated from the
            # single-arm description, whose root IS that link.
            raise WorkspaceModelError(
                "profile 'single' requires a link geometry rooted at arm '{}'s own "
                '{}, but it is rooted at {}; generate the geometry from the '
                'single-arm description'.format(arms[0]['arm_id'],
                                                arms[0]['base_link'],
                                                geometry.root_link))
        for arm in arms:
            if not any(owner.get(link) == arm['arm_id'] for link in participating):
                raise WorkspaceModelError(
                    "arm '{}' has no links in the link geometry".format(arm['arm_id']))

        volume_index = {}
        link_volumes = {}
        for link in participating:
            volumes = list(geometry.volumes[link])
            arm_id = owner.get(link)
            if arm_id is not None and link == '{}_link8'.format(arm_id):
                end_effector = next(arm['end_effector'] for arm in arms
                                    if arm['arm_id'] == arm_id)
                if end_effector['present']:
                    for entry in end_effector['volumes']:
                        volumes.append(_EndEffectorVolume(entry, link))
            for volume in volumes:
                if volume.id in volume_index:
                    raise WorkspaceModelError(
                        "duplicate volume id '{}'; end-effector volume ids must not "
                        'collide with derived ones'.format(volume.id))
                volume_index[volume.id] = (link, volume)
            link_volumes[link] = tuple(volumes)

        # A.3.4 - a pair may not travel in both directions at once.
        both = set(policy['extra_disabled_pairs']) & set(policy['extra_enabled_pairs'])
        if both:
            pair = sorted(sorted(item) for item in both)[0]
            raise WorkspaceModelError(
                'pair ({}, {}) appears in both extra_disabled_pairs and '
                'extra_enabled_pairs'.format(pair[0], pair[1]))
        effective = (set(disabled) | set(policy['extra_disabled_pairs'])) - set(
            policy['extra_enabled_pairs'])

        intra_pairs = []
        for arm_id in arm_ids:
            links = [link for link in participating if owner.get(link) == arm_id]
            for first_index, first in enumerate(links):
                for second in links[first_index + 1:]:
                    if frozenset((first, second)) in effective:
                        continue
                    for volume_a in link_volumes[first]:
                        for volume_b in link_volumes[second]:
                            intra_pairs.append((volume_a.id, volume_b.id, arm_id))

        structure_pairs = []
        root = geometry.root_link
        if self.profile == 'dual' and root not in owner:
            for structure_volume in link_volumes[root]:
                for link in participating:
                    arm_id = owner.get(link)
                    if arm_id is None:
                        continue
                    if frozenset((root, link)) in effective:
                        continue
                    for volume in link_volumes[link]:
                        structure_pairs.append(
                            (structure_volume.id, volume.id, arm_id))

        cross_pairs = []
        if len(arm_ids) == 2:
            first_links = [link for link in participating if owner.get(link) == arm_ids[0]]
            second_links = [link for link in participating
                            if owner.get(link) == arm_ids[1]]
            for first in first_links:
                for second in second_links:
                    for volume_a in link_volumes[first]:
                        for volume_b in link_volumes[second]:
                            cross_pairs.append((volume_a.id, volume_b.id, arm_ids[0]))

        cell_from_root = np.eye(4)
        if self.profile == 'single':
            anchor = cell_frame['single']
            cell_from_root = np.linalg.inv(homogeneous(
                rotation_from_rpy(*anchor['rpy']), anchor['xyz']))

        moving_volumes = []
        containment_volumes = []
        exempt = []
        for link in participating:
            arm_id = owner.get(link)
            fully_static, z_static = self._chain_flags(geometry, link, cell_from_root)
            for volume in link_volumes[link]:
                if fully_static:
                    exempt.append((volume.id, 'fully static'))
                    continue
                if arm_id is None:
                    continue
                moving_volumes.append((volume.id, arm_id))
                faces = BOX_FACES if not z_static else tuple(
                    face for face in BOX_FACES if not face.startswith('z'))
                if z_static:
                    exempt.append((volume.id, 'z-static'))
                containment_volumes.append((volume.id, arm_id, faces))

        actuated = {}
        for link, joint in geometry.joints.items():
            if joint['type'] != 'revolute':
                continue
            if link not in participating:
                continue
            arm_id = owner.get(link)
            match = re.match(r'^(.+)_joint([1-7])$', joint['name'])
            if arm_id is None or match is None or match.group(1) != arm_id:
                raise WorkspaceModelError(
                    "revolute joint '{}' is not one of arm '{}'s seven joints; the "
                    'checker has no value to drive it with'.format(joint['name'], arm_id))
            actuated[joint['name']] = (arm_ids.index(arm_id), int(match.group(2)) - 1)
        for arm_position, arm_id in enumerate(arm_ids):
            expected = {'{}_joint{}'.format(arm_id, index + 1)
                        for index in range(JOINT_COUNT)}
            found = {name for name, value in actuated.items()
                     if value[0] == arm_position}
            if found != expected:
                raise WorkspaceModelError(
                    "arm '{}': the link geometry declares the revolute joints {} but "
                    'the checker drives exactly {}'.format(
                        arm_id, sorted(found), sorted(expected)))

        volume_list = []
        for link in participating:
            for volume in link_volumes[link]:
                volume_list.append((volume.id, link, volume))
        position = {entry[0]: index for index, entry in enumerate(volume_list)}
        radii = np.array([entry[2].radius for entry in volume_list], dtype=float)

        def _index_pair(pairs):
            first = np.array([position[item[0]] for item in pairs], dtype=int)
            second = np.array([position[item[1]] for item in pairs], dtype=int)
            return first, second

        mask = np.zeros((len(containment_volumes), 6), dtype=bool)
        for row, (_, _, faces) in enumerate(containment_volumes):
            for column, face in enumerate(BOX_FACES):
                mask[row, column] = face in faces
        return {
            '_volume_index': volume_index,
            '_volume_list': tuple(volume_list),
            '_volume_position': position,
            '_radii': radii,
            '_intra_pairs': tuple(intra_pairs),
            '_intra_index': _index_pair(intra_pairs),
            '_intra_meta': tuple(intra_pairs),
            '_structure_pairs': tuple(structure_pairs),
            '_cross_pairs': tuple(cross_pairs),
            '_cross_index': _index_pair(cross_pairs),
            '_cross_meta': tuple(cross_pairs),
            '_containment_volumes': tuple(containment_volumes),
            '_containment_index': np.array(
                [position[item[0]] for item in containment_volumes], dtype=int),
            '_containment_meta': tuple(
                (item[0], item[1]) for item in containment_volumes),
            '_containment_mask': mask,
            '_moving_volumes': tuple(moving_volumes),
            '_actuated': actuated,
            '_cell_from_root': cell_from_root,
            '_exempt_volumes': tuple(exempt),
        }

    @staticmethod
    def _chain_flags(geometry, link, cell_from_root):
        """
        Decide whether this link is fixed, and whether its height is constant.

        A volume whose clearance to a face is a constant cannot be a fence,
        because no motion can change it.  Both classes are derived here from the
        joint chain, so a description change cannot silently invalidate them.
        """
        chain = []
        walker = link
        while walker in geometry.parent:
            chain.append(walker)
            walker = geometry.parent[walker]
        chain.reverse()
        rotation = cell_from_root[:3, :3].copy()
        fully_static = True
        z_static = True
        for name in chain:
            joint = geometry.joints[name]
            rotation = rotation @ rotation_from_rpy(*joint['origin_rpy'])
            if joint['type'] == 'revolute':
                fully_static = False
                axis = rotation @ joint['axis']
                if abs(abs(float(axis[2])) - 1.0) > VERTICAL_AXIS_TOLERANCE:
                    z_static = False
                    break
        return fully_static, z_static

    def _static_diagnostics(self, state, geometry, allowed_volume, environment, arms):
        """Report static-against-static geometry once at load, never per query."""
        bounds = ((allowed_volume.x_min, allowed_volume.x_max),
                  (allowed_volume.y_min, allowed_volume.y_max),
                  (allowed_volume.z_min, allowed_volume.z_max))
        transforms = {geometry.root_link: state['_cell_from_root']}
        for link in geometry.link_order:
            if link in transforms or link not in geometry.parent:
                continue
            chain = []
            walker = link
            while walker not in transforms:
                chain.append(walker)
                walker = geometry.parent[walker]
            for name in reversed(chain):
                joint = geometry.joints[name]
                transforms[name] = transforms[geometry.parent[name]] @ homogeneous(
                    rotation_from_rpy(*joint['origin_rpy']), joint['origin_xyz'])
        statics = []
        for volume_id, reason in state['_exempt_volumes']:
            if reason != 'fully static':
                continue
            link, volume = state['_volume_index'][volume_id]
            transform = transforms[link]
            centre = transform[:3, :3] @ volume.a + transform[:3, 3]
            # The pedestal cube is evaluated as its conservative bounding sphere:
            # that makes the check a zero-length segment against each solid, so it
            # needs no box-box primitive, and it can only ever over-report.
            radius = (PEDESTAL_BOUNDING_RADIUS if volume.kind == 'box'
                      else volume.radius)
            statics.append((volume_id, centre, radius))
        for volume_id, centre, radius in statics:
            clearance = min(
                min(centre[axis] - radius - bounds[axis][0],
                    bounds[axis][1] - centre[axis] - radius)
                for axis in range(3))
            self.diagnostics.append(
                'volume {} is static in the cell frame: its clearance to the '
                'allowed_volume boundary is a constant {:+.7f} m, so it is exempt from '
                'the per-query containment check'.format(volume_id, clearance))
            for solid in environment:
                distance = CellModel._solid_clearance((centre, centre, radius), solid)
                if distance < 0.0:
                    self.diagnostics.append(
                        'volume {} overlaps the declared solid {} by {:.7f} m (measured '
                        'with the conservative bounding sphere); this is reported, not '
                        'a load failure'.format(volume_id, solid['id'], -distance))
        for volume_id, reason in state['_exempt_volumes']:
            if reason != 'z-static':
                continue
            link, volume = state['_volume_index'][volume_id]
            transform = transforms[link]
            rotation = transform[:3, :3]
            translation = transform[:3, 3]
            lowest = min(float((rotation @ point + translation)[2])
                         for point in (volume.a, volume.b)) - volume.radius
            self.diagnostics.append(
                'volume {} rotates only about the vertical, so its height above the '
                'table top cannot change: its lowest point is a constant {:+.7f} m and '
                'it is exempt from the z_min and z_max faces only. This is a property '
                'of the robot description, not of your measurements'.format(
                    volume_id, lowest))


class _EndEffectorVolume:
    """A declared, derived end-effector capsule, in the flange link's frame."""

    def __init__(self, entry, link_name):
        self.id = entry['id']
        self.link = link_name
        self.kind = 'capsule'
        self.a = entry['a']
        self.b = entry['b']
        self.radius = entry['radius']
        self.containment = 'conservative'
        self.containment_margin = 0.0
        self.source_elements = ()
