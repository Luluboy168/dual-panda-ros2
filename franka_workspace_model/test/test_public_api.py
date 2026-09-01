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

"""The public surface, the jog semantics, and the two constraint classes at work."""

import dataclasses
import threading

from conftest import CELL_MODEL_PATH, load_mutated, READY

from franka_workspace_model import model as model_module
from franka_workspace_model.model import (AllowedVolume, CellModel, CheckResult,
                                          Contact, JogResult, result_to_json,
                                          WorkspaceModelError)

import pytest


BOTH_READY = {'panda1': list(READY), 'panda2': list(READY)}
SHOULDER_BACK = {'panda1': [0.0, -1.75, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
                 'panda2': list(READY)}
STEP = 0.0175


def test_the_public_surface_is_exactly_the_contracted_one():
    assert set(model_module.__all__) == {
        'AllowedVolume', 'CellModel', 'CheckResult', 'Contact', 'JogResult',
        'WorkspaceModelError', 'default_cell_model_path', 'result_to_json'}
    for kind in (Contact, CheckResult, JogResult, AllowedVolume):
        assert dataclasses.is_dataclass(kind)
        assert kind.__dataclass_params__.frozen
    assert [field.name for field in dataclasses.fields(Contact)] == [
        'kind', 'a', 'b', 'distance', 'required', 'arm_id']
    assert [field.name for field in dataclasses.fields(CheckResult)] == [
        'ok', 'min_clearance', 'contacts', 'sample_index', 'samples_evaluated',
        'model_id', 'model_revision', 'model_sha256']
    assert [field.name for field in dataclasses.fields(JogResult)] == [
        'allowed', 'q_target', 'clamped', 'limiting', 'result']


def test_the_allowed_volume_accessor_returns_the_measured_box(cell_model):
    """R1: the console draws the measured cell, so it must be able to read it."""
    box = cell_model.allowed_volume()
    assert isinstance(box, AllowedVolume)
    assert (box.id, box.frame) == ('work_area', 'cell')
    assert (box.x_min, box.x_max) == (-0.35, 0.9)
    assert (box.y_min, box.y_max) == (-1.0, 1.0)
    assert (box.z_min, box.z_max) == (0.0, 2.0)
    assert dataclasses.replace(box, id='other').id == 'other'


def test_the_identity_triple_travels_with_every_result(cell_model):
    identity = cell_model.model_identity()
    assert identity[0] == 'hcislab_dual_panda_cell'
    assert identity[1] == 1
    assert len(identity[2]) == 64
    result = cell_model.check_configuration(BOTH_READY)
    assert (result.model_id, result.model_revision, result.model_sha256) == identity
    jog = cell_model.check_jog('panda1', BOTH_READY, 0, STEP)
    assert jog.result.model_sha256 == identity[2]


def test_the_urdf_digest_is_the_generated_text_not_the_xacro(cell_model):
    import hashlib
    xacro = (CELL_MODEL_PATH.parent.parent.parent
             / 'franka_description/robots/real/dual_panda_arm.urdf.xacro')
    assert cell_model.urdf_sha256() != hashlib.sha256(xacro.read_bytes()).hexdigest()
    assert len(cell_model.urdf_sha256()) == 64


def test_the_recorded_xacro_arguments_are_readable(cell_model):
    """R3: a consumer must be able to reproduce the digest."""
    arguments = cell_model.xacro_args()
    assert arguments['arm_id_1'] == 'panda1'
    assert arguments['arm_id_2'] == 'panda2'
    assert arguments['robot_ip_1'] == ''
    arguments['arm_id_1'] = 'mutated'
    assert cell_model.xacro_args()['arm_id_1'] == 'panda1'


def test_a_small_jog_is_allowed_unclamped(cell_model):
    result = cell_model.check_jog('panda1', BOTH_READY, 1, 2.0 * STEP)
    assert result.allowed
    assert not result.clamped
    assert result.limiting is None
    assert abs(result.q_target[1] - (READY[1] + 2.0 * STEP)) < 1e-12
    assert result.result.samples_evaluated == 3


def test_a_jog_toward_a_boundary_is_clamped_and_reported(cell_model):
    """Clamp-then-report: the UI jogs the reduced amount and says it was clamped."""
    result = cell_model.check_jog('panda1', BOTH_READY, 1, -3.0)
    assert result.allowed
    assert result.clamped
    assert result.limiting is not None
    travelled = READY[1] - result.q_target[1]
    assert travelled > 0.0
    assert abs(travelled / STEP - round(travelled / STEP)) < 1e-9
    # The clamped target is itself safe.
    target = dict(BOTH_READY)
    target['panda1'] = list(result.q_target)
    assert cell_model.check_configuration(target).ok


def test_a_jog_whose_first_step_is_unsafe_is_refused(cell_model):
    result = cell_model.check_jog('panda1', SHOULDER_BACK, 1, -0.02)
    assert not result.allowed
    assert not result.clamped
    assert result.limiting.kind == 'containment'
    assert tuple(result.q_target) == tuple(SHOULDER_BACK['panda1'])
    assert result.result.sample_index == 0


def test_a_zero_jog_is_a_query_about_where_the_arm_already_is(cell_model):
    result = cell_model.check_jog('panda1', BOTH_READY, 3, 0.0)
    assert result.allowed
    assert not result.clamped
    assert tuple(result.q_target) == tuple(READY)


def test_the_jog_holds_the_other_arm_where_it_actually_is(cell_model):
    """A jog is only safe relative to where the other arm is."""
    extended = [0.0, 0.7, 0.0, -1.6, 0.0, 1.5708, -0.7854]
    towards = cell_model.check_jog(
        'panda2', {'panda1': [-1.4, 0.7, 0.0, -1.6, 0.0, 1.5708, 0.7854],
                   'panda2': extended}, 0, 1.4)
    assert towards.allowed and towards.clamped
    assert towards.q_target[0] < 1.4
    away = cell_model.check_jog(
        'panda2', {'panda1': list(READY), 'panda2': extended}, 0, 1.4)
    assert away.allowed and not away.clamped
    assert abs(away.q_target[0] - 1.4) < 1e-9


@pytest.mark.parametrize('bad', [
    ('panda9', 0, 0.1), ('panda1', 7, 0.1), ('panda1', -1, 0.1),
    ('panda1', 0, float('nan')), ('panda1', 0, float('inf')), ('panda1', True, 0.1),
])
def test_a_malformed_jog_request_is_refused(cell_model, bad):
    with pytest.raises(WorkspaceModelError):
        cell_model.check_jog(bad[0], BOTH_READY, bad[1], bad[2])


def test_a_path_is_resampled_at_the_declared_step(cell_model):
    start = dict(BOTH_READY)
    end = {'panda1': [0.0, -0.7854 + 10 * STEP, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
           'panda2': list(READY)}
    result = cell_model.check_path([start, end])
    assert result.ok
    assert result.samples_evaluated == 11


def test_a_path_that_passes_through_contact_is_refused_even_when_both_ends_are_clear(
        cell_model):
    """The case target-only checking would miss, which is why v1 is always swept."""
    start = dict(BOTH_READY)
    through = {'panda1': [0.0, -1.75, 0.0, -2.3562, 0.0, 1.5708, 0.7854],
               'panda2': list(READY)}
    assert cell_model.check_configuration(start).ok
    assert not cell_model.check_configuration(through).ok
    back = cell_model.check_path([start, through, start])
    assert not back.ok
    assert back.sample_index is not None
    assert 0 < back.sample_index < back.samples_evaluated


def test_a_malformed_path_is_refused(cell_model):
    with pytest.raises(WorkspaceModelError):
        cell_model.check_path([])
    with pytest.raises(WorkspaceModelError):
        cell_model.check_path('not a path')
    with pytest.raises(WorkspaceModelError):
        cell_model.check_path([BOTH_READY, {'panda1': list(READY)}])


def test_check_configuration_is_safe_to_call_concurrently(cell_model):
    """R6: the console calls this from several HTTP handler threads without a lock."""
    results = []
    errors = []

    def worker():
        try:
            for _ in range(50):
                results.append(cell_model.check_configuration(BOTH_READY).min_clearance)
                results.append(
                    cell_model.check_configuration(SHOULDER_BACK).min_clearance)
        except Exception as error:  # noqa: BLE001 - the test is what reports it
            errors.append(error)

    threads = [threading.Thread(target=worker) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert not errors
    assert len(set(results)) == 2


def test_result_to_json_refuses_anything_else(cell_model):
    with pytest.raises(WorkspaceModelError):
        result_to_json({'ok': True})


def test_a_declared_environment_solid_produces_an_environment_contact(tmp_path):
    """The positive case the shipped cell cannot express: environment is empty."""
    def mutate(document):
        document['environment'] = [{
            'id': 'overhead_frame', 'kind': 'box', 'frame': 'cell',
            'pose': {'xyz': [0.3, 0.5, 0.75], 'rpy': [0.0, 0.0, 0.0]},
            'size': [0.2, 0.2, 0.2], 'measurement_status': 'assumed',
            'source_question': 'Q6', 'note': 'synthetic, for the test suite'}]
    model = load_mutated(tmp_path, mutate)
    result = model.check_configuration(BOTH_READY)
    assert not result.ok
    kinds = {contact.kind for contact in result.contacts}
    assert kinds == {'environment'}
    contact = result.contacts[0]
    assert contact.b == 'overhead_frame'
    assert contact.arm_id == 'panda1'
    assert contact.required == 0.03


def test_an_enabled_keep_out_zone_produces_a_keep_out_contact(tmp_path):
    """The positive case the shipped cell cannot express: the zone is disabled."""
    reaching = {'panda1': [-1.4, 0.7, 0.0, -1.6, 0.0, 1.5708, 0.7854],
                'panda2': [1.4, 0.7, 0.0, -1.6, 0.0, 1.5708, -0.7854]}

    def mutate(document):
        document['keep_out'][0]['enabled'] = True
    model = load_mutated(tmp_path, mutate)
    result = model.check_configuration(reaching)
    contacts = [contact for contact in result.contacts if contact.kind == 'keep_out']
    assert {contact.arm_id for contact in contacts} == {'panda1', 'panda2'}
    assert {contact.b for contact in contacts} == {'midplane'}
    # The disabled zone in the shipped file produces none at the same pose.
    shipped = CellModel.load(CELL_MODEL_PATH, profile='dual')
    assert not [contact for contact in shipped.check_configuration(reaching).contacts
                if contact.kind == 'keep_out']


def test_a_zone_that_applies_to_one_arm_only_constrains_that_arm(tmp_path):
    reaching = {'panda1': [-1.4, 0.7, 0.0, -1.6, 0.0, 1.5708, 0.7854],
                'panda2': [1.4, 0.7, 0.0, -1.6, 0.0, 1.5708, -0.7854]}

    def mutate(document):
        document['keep_out'][0]['enabled'] = True
        document['keep_out'][0]['applies_to'] = ['panda2']
    model = load_mutated(tmp_path, mutate)
    contacts = [contact for contact in model.check_configuration(reaching).contacts
                if contact.kind == 'keep_out']
    assert {contact.arm_id for contact in contacts} == {'panda2'}


def test_a_cylinder_solid_is_promoted_to_its_enclosing_capsule(tmp_path):
    def mutate(document):
        document['environment'] = [{
            'id': 'post_between_arms', 'kind': 'cylinder', 'frame': 'cell',
            'pose': {'xyz': [0.3, 0.0, 0.5], 'rpy': [0.0, 0.0, 0.0]},
            'length': 0.4, 'radius': 0.05, 'measurement_status': 'assumed',
            'source_question': 'Q7', 'note': 'synthetic, for the test suite'}]
    model = load_mutated(tmp_path, mutate)
    assert model._environment[0]['kind'] == 'capsule'
    assert any('minimal enclosing capsule' in line for line in model.diagnostics())


def test_a_half_space_solid_is_evaluated_at_the_endpoints(tmp_path):
    """
    The forbidden region is x < -0.45: clear at ready, breached shoulder-back.

    At ready the furthest-back point of any volume is link3_v0 at -0.26394859;
    with the shoulder driven to -1.75 it is link5_v0 at -0.44128352.
    """
    def mutate(document):
        document['environment'] = [{
            'id': 'back_wall', 'kind': 'plane_halfspace', 'frame': 'cell',
            'normal': [1.0, 0.0, 0.0], 'offset': -0.45,
            'measurement_status': 'assumed', 'source_question': 'Q10',
            'note': 'synthetic, for the test suite'}]
    model = load_mutated(tmp_path, mutate)
    assert model.check_configuration(BOTH_READY).ok
    result = model.check_configuration(SHOULDER_BACK)
    kinds = {contact.kind for contact in result.contacts}
    assert 'environment' in kinds


def test_an_end_effector_that_is_derived_and_present_is_checked(tmp_path):
    """The whole point of the switch: mounting the tool makes it visible."""
    def mutate(document):
        end_effector = document['arms'][0]['end_effector']
        end_effector['present'] = True
        end_effector['volumes'][0].update({
            'a': [0.0, 0.0, 0.0], 'b': [0.0, 0.0, 0.16], 'radius': 0.05,
            'containment': 'conservative', 'containment_margin': 0.004,
            'derivation_status': 'derived',
            'provenance': 'synthetic geometry, for the test suite only'})
    model = load_mutated(tmp_path, mutate)
    assert 'robotiq_2f85_v0' in model._volume_position
    pairs = {(first, second) for first, second, _ in model._cross_pairs}
    assert any('robotiq_2f85_v0' in pair for pair in pairs)
    assert ('robotiq_2f85_v0', 'panda1') in model._moving_volumes
    assert len(model._cross_pairs) == 110


def test_an_absent_end_effector_is_not_checked(cell_model):
    assert 'robotiq_2f85_v0' not in cell_model._volume_position
    assert len(cell_model._cross_pairs) == 100
