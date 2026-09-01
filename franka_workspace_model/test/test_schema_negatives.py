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
T11: malformed models are refused, each with its own message.

Fail-closed is enforced here rather than aspired to.  Every case below must
raise, and no two cases may produce the same message: two different defects that
read the same are two defects nobody can tell apart at three in the morning.

The last test in this file is the positive one: no message this package can
emit, and no string in any file it ships, names a document the operator cannot
open.
"""

from pathlib import Path

from conftest import CELL_MODEL_PATH, load_mutated, SOURCE_DIR

from franka_workspace_model.model import CellModel, WorkspaceModelError

import pytest


TEXT = CELL_MODEL_PATH.read_text(encoding='utf-8')
# Assembled rather than written out, so that this file does not trip its own
# purity scan.
FORBIDDEN_SUBSTRINGS = (
    'WORKSPACE_' + 'MODEL_',
    'CELL_MODEL_' + 'DRAFT',
    'plans/' + 'post_mvp',
    'notes' + '/',
)


def _pop_top(key):
    def mutate(document):
        document.pop(key)
    return mutate


def _set(path, value):
    def mutate(document):
        target = document
        for step in path[:-1]:
            target = target[step]
        target[path[-1]] = value
    return mutate


DOCUMENT_CASES = {
    'missing top-level key': _pop_top('margins'),
    'unknown top-level key': _set(['unexpected'], 'value'),
    'wrong schema_version': _set(['schema_version'], 2),
    'schema_version as a string': _set(['schema_version'], '1'),
    'revision below one': _set(['revision'], 0),
    'model_id pattern': _set(['model_id'], 'Not A Valid Id'),
    'measured_on is not a real date': _set(['measured_on'], '2026-02-30'),
    'units.length': _set(['units', 'length'], 'mm'),
    'units.angle': _set(['units', 'angle'], 'deg'),
    'sources hash is not hex': _set(['sources', 'urdf_xacro_sha256'], 'TO-BE-GENERATED'),
    'sources hash mismatch': _set(['sources', 'urdf_xacro_sha256'], '0' * 64),
    'sources path escapes the tree': _set(['sources', 'urdf_xacro'], '../etc/passwd'),
    'link geometry names another description': _set(
        ['sources', 'urdf_xacro'],
        'franka_description/robots/real/panda_arm.urdf.xacro'),
    'measured base pose outside tolerance': _set(
        ['arms', 0, 'measured_base_pose', 'xyz'], [0.0, 0.6, 0.0]),
    'measured base orientation outside tolerance': _set(
        ['arms', 0, 'measured_base_pose', 'rpy'], [0.0, 0.0, 0.2]),
    'non-positive tolerance': _set(['arms', 0, 'measured_base_pose', 'tolerance_m'], 0.0),
    'unknown measurement_status': _set(
        ['arms', 0, 'measured_base_pose', 'measurement_status'], 'guessed'),
    'duplicate arm_id': _set(['arms', 1, 'arm_id'], 'panda1'),
    'arm base_link does not match arm_id': _set(['arms', 0, 'base_link'], 'panda2_link0'),
    'asymmetric mounting': _set(['arms', 1, 'urdf_base_pose', 'xyz'], [0.0, -0.4, 0.0]),
    'inverted allowed_volume bounds': _set(['allowed_volume', 'x_max'], -0.9),
    'arm base outside the allowed volume': _set(['allowed_volume', 'y_max'], 0.51),
    'arm base off the declared table top': _set(['allowed_volume', 'z_min'], 0.02),
    'allowed_volume frame': _set(['allowed_volume', 'frame'], 'world'),
    'source_question outside the range': _set(['allowed_volume', 'source_question'], 'Q18'),
    'source_question repeats': _set(['allowed_volume', 'source_question'], 'Q1,Q1'),
    'end effector enabled with no profile': _set(['arms', 0, 'end_effector'], {
        'present': True, 'profile': 'none', 'volumes': []}),
    'end effector enabled but underived': _set(
        ['arms', 0, 'end_effector', 'present'], True),
    'end effector profile without volumes': _set(
        ['arms', 0, 'end_effector', 'volumes'], []),
    'unknown end effector profile': _set(
        ['arms', 0, 'end_effector', 'profile'], 'imaginary_gripper'),
    'inflation is not the recovered one': _set(
        ['margins', 'urdf_builtin_inflation'], 0.06),
    'negative margin': _set(['margins', 'cross_arm'], -0.01),
    'empty margin rationale': _set(['margins', 'rationale'], ''),
    'default_mode target_only': _set(['policy', 'default_mode'], 'target_only'),
    'fail_closed false': _set(['policy', 'fail_closed'], False),
    'non-positive step': _set(['policy', 'max_joint_step_rad'], 0.0),
    'cross_arm disabled on a two-arm model': _set(
        ['policy', 'cross_arm', 'enabled'], False),
    'environment disabled': _set(['policy', 'environment', 'enabled'], False),
    'containment disabled': _set(['policy', 'containment', 'enabled'], False),
    'acm_source other than srdf': _set(
        ['policy', 'self_collision', 'acm_source'], 'hand_written'),
    'joint limits from somewhere else': _set(
        ['policy', 'joint_limits', 'source'], 'guesswork'),
    'delta names an unknown link': _set(
        ['policy', 'self_collision', 'extra_disabled_pairs', 0, 'a'], 'panda9_link0'),
    'delta names the same link twice': _set(
        ['policy', 'self_collision', 'extra_disabled_pairs', 0, 'b'], 'base_link'),
    'delta with an empty reason': _set(
        ['policy', 'self_collision', 'extra_disabled_pairs', 0, 'reason'], ''),
    'keep_out applies to an unknown arm': _set(['keep_out', 0, 'applies_to'], ['panda9']),
    'keep_out with an empty reason': _set(['keep_out', 0, 'reason'], ''),
    'keep_out frame': _set(['keep_out', 0, 'frame'], 'world'),
    'keep_out with a non-positive size': _set(['keep_out', 0, 'size'], [1.25, 0.0, 1.4]),
    'duplicate solid id': _set(['keep_out', 0, 'id'], 'work_area'),
    'unknown solid kind': _set(['keep_out', 0, 'kind'], 'torus'),
    'boolean where a number is declared': _set(['margins', 'cross_arm'], True),
    'a list where a mapping is declared': _set(['units'], ['m', 'rad']),
    'dual anchor is not the identity': _set(
        ['cell_frame', 'anchors', 'dual', 'xyz'], [0.0, 0.1, 0.0]),
    'cell frame name': _set(['cell_frame', 'name'], 'world'),
    'single anchor link literal': _set(
        ['cell_frame', 'anchors', 'single', 'link'], 'panda1_link0'),
}


def _environment_solid(kind, **fields):
    entry = {'id': 'probe_solid', 'kind': kind, 'frame': 'cell',
             'measurement_status': 'assumed', 'source_question': 'Q6', 'note': ''}
    entry.update(fields)

    def mutate(document):
        document['environment'] = [entry]
    return mutate


DOCUMENT_CASES.update({
    'plane normal is not a unit vector': _environment_solid(
        'plane_halfspace', normal=[0.0, 0.0, 2.0], offset=0.0),
    'sphere with a non-positive radius': _environment_solid(
        'sphere', pose={'xyz': [1.0, 0.0, 0.5], 'rpy': [0.0, 0.0, 0.0]}, radius=0.0),
    'capsule with a zero-length segment': _environment_solid(
        'capsule', a=[1.0, 0.0, 0.5], b=[1.0, 0.0, 0.5], radius=0.1),
    'solid geometry keys do not match the kind': _environment_solid(
        'box', pose={'xyz': [1.0, 0.0, 0.5], 'rpy': [0.0, 0.0, 0.0]}, radius=0.1),
    'environment solid frame': _environment_solid(
        'sphere', frame='world', pose={'xyz': [1.0, 0.0, 0.5], 'rpy': [0.0, 0.0, 0.0]},
        radius=0.1),
})


def _pair_in_both(document):
    document['policy']['self_collision']['extra_enabled_pairs'].append(
        {'a': 'base_link', 'b': 'panda1_link0', 'reason': 'contradictory on purpose'})


DOCUMENT_CASES['pair travels in both directions'] = _pair_in_both

TEXT_CASES = {
    'empty file': '',
    'NUL byte': TEXT.replace('schema_version: 1', 'schema_version: 1\x00'),
    'duplicate key': TEXT + '\nrevision: 2\n',
    'multiple documents': TEXT + '\n---\nschema_version: 1\n',
    # An alias cannot occur without an anchor, so the two share one rejection:
    # the defect a reader can act on is "this file uses anchors".
    'YAML anchor, and therefore any alias': TEXT.replace(
        'units:\n  length: m', 'units: &shared\n  length: m').replace(
        'environment: []', 'environment: *shared'),
    # Anchor-free, so that the merge key is what the scanner meets first.
    'YAML merge key': TEXT.replace(
        'policy:\n  schema:', 'policy:\n  <<: {schema: checking_policy_v1}\n  schema:'),
    'explicit tag': TEXT.replace('revision: 1', 'revision: !!int 1'),
    'tab indentation': TEXT.replace('units:\n  length: m', 'units:\n\tlength: m'),
    'not a mapping at the root': '- one\n- two\n',
    'oversize file': TEXT + '\n'.join('# padding {}'.format(index)
                                      for index in range(6000)),
    'a null value': TEXT.replace('revision: 1', 'revision:'),
    'a non-finite number': TEXT.replace('x_min: -0.35', 'x_min: .nan'),
}

# Variants of a case above: also refused, but by the same message, because they
# are the same defect wearing a different hat.
VARIANT_TEXT_CASES = {
    'whitespace only': '   \n\n  \n',
    'positive infinity': TEXT.replace('x_max: 0.9', 'x_max: .inf'),
    'negative infinity': TEXT.replace('x_min: -0.35', 'x_min: -.inf'),
}


def _collect_message(case, tmp_path):
    with pytest.raises(WorkspaceModelError) as raised:
        load_mutated(tmp_path, **case)
    return str(raised.value)


@pytest.mark.parametrize('name', sorted(DOCUMENT_CASES))
def test_a_malformed_document_is_refused(tmp_path, name):
    _collect_message({'mutate': DOCUMENT_CASES[name]}, tmp_path)


@pytest.mark.parametrize('name', sorted(TEXT_CASES))
def test_malformed_text_is_refused(tmp_path, name):
    _collect_message({'text': TEXT_CASES[name]}, tmp_path)


@pytest.mark.parametrize('name', sorted(VARIANT_TEXT_CASES))
def test_a_variant_of_a_malformed_text_is_refused_too(tmp_path, name):
    _collect_message({'text': VARIANT_TEXT_CASES[name]}, tmp_path)


def test_the_negative_set_is_large_and_every_message_is_distinct(tmp_path):
    messages = {}
    for index, (name, mutate) in enumerate(sorted(DOCUMENT_CASES.items())):
        messages[name] = _collect_message({'mutate': mutate},
                                          tmp_path / 'd{}'.format(index))
    for index, (name, text) in enumerate(sorted(TEXT_CASES.items())):
        messages[name] = _collect_message({'text': text},
                                          tmp_path / 't{}'.format(index))
    assert len(messages) >= 20
    duplicates = {}
    for name, message in messages.items():
        duplicates.setdefault(message, []).append(name)
    collisions = {message: names for message, names in duplicates.items()
                  if len(names) > 1}
    assert not collisions, 'these defects are indistinguishable: {}'.format(collisions)


def test_a_path_that_is_not_a_regular_file_is_refused(tmp_path):
    with pytest.raises(WorkspaceModelError):
        CellModel.load(tmp_path, profile='dual')
    with pytest.raises(WorkspaceModelError):
        CellModel.load(tmp_path / 'absent.yaml', profile='dual')


def test_an_unknown_profile_is_refused():
    with pytest.raises(WorkspaceModelError, match='profile must be'):
        CellModel.load(CELL_MODEL_PATH, profile='triple')


def test_a_model_cannot_load_without_the_description_it_was_derived_from(tmp_path):
    lonely = tmp_path / 'cell'
    lonely.mkdir(parents=True)
    (lonely / 'cell_model_v1.yaml').write_text(TEXT, encoding='utf-8')
    with pytest.raises(WorkspaceModelError, match='no directory at or above'):
        CellModel.load(lonely / 'cell_model_v1.yaml', profile='dual')


def test_no_error_message_names_a_document_the_operator_cannot_open(tmp_path):
    """The purity half of T11, asserted mechanically rather than remembered."""
    messages = []
    for index, mutate in enumerate(DOCUMENT_CASES.values()):
        messages.append(_collect_message({'mutate': mutate},
                                         tmp_path / 'p{}'.format(index)))
    for index, text in enumerate(TEXT_CASES.values()):
        messages.append(_collect_message({'text': text},
                                         tmp_path / 'q{}'.format(index)))
    for message in messages:
        for needle in FORBIDDEN_SUBSTRINGS:
            assert needle not in message, message


SHIPPED_FILES = sorted(
    path for pattern in ('franka_workspace_model/**/*.py', 'cell/*.yaml',
                         'doc/*.md', 'doc/*.svg', 'README.md', 'package.xml',
                         'CMakeLists.txt')
    for path in SOURCE_DIR.glob(pattern))


def test_the_shipped_files_exist():
    names = {Path(path).name for path in SHIPPED_FILES}
    assert {'model.py', 'geometry.py', 'cell_model_v1.yaml', 'link_geometry_v1.yaml',
            'CONTRACT.md', 'cell_frame_top_view.svg', 'README.md'} <= names


@pytest.mark.parametrize('path', SHIPPED_FILES, ids=[p.name for p in SHIPPED_FILES])
def test_no_shipped_file_names_a_document_outside_the_installed_package(path):
    text = path.read_text(encoding='utf-8')
    for needle in FORBIDDEN_SUBSTRINGS:
        assert needle not in text, '{} names {}'.format(path, needle)


def test_the_shipped_cell_file_carries_no_network_address():
    text = (SOURCE_DIR / 'cell' / 'link_geometry_v1.yaml').read_text(encoding='utf-8')
    assert 'robot_ip_1: ""' in text
    assert 'robot_ip_2: ""' in text


def test_a_model_with_an_enabled_but_underived_end_effector_teaches(tmp_path):
    """A.3.18's message is read by an operator standing at the robot."""
    def mutate(document):
        document['arms'][0]['end_effector']['present'] = True
    with pytest.raises(WorkspaceModelError) as raised:
        load_mutated(tmp_path, mutate)
    message = str(raised.value)
    assert 'zero-size capsule' in message
    assert 'doc/CONTRACT.md' in message
    assert 'do not mount the tool' in message


def test_the_urdf_disagreement_message_says_stop_and_report(tmp_path):
    def mutate(document):
        document['arms'][0]['measured_base_pose']['xyz'] = [0.0, 0.74, 0.0]
    with pytest.raises(WorkspaceModelError) as raised:
        load_mutated(tmp_path, mutate)
    message = str(raised.value)
    assert 'stop and report' in message
    assert 'do not edit the scene spec' in message
    assert 'doc/CONTRACT.md' in message
