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

"""T1, T3, T4, T5: every corpus verdict, against its hand-derived expectation."""

from conftest import CORPUS_DIR
from corpus_loader import load_corpus

import pytest


CORPUS = load_corpus(CORPUS_DIR)


def test_every_category_file_is_present_and_non_empty():
    categories = {entry.category for entry in CORPUS}
    assert categories == {'known_clear', 'self', 'cross_arm', 'containment',
                          'environment', 'keep_out', 'joint_limit'}


@pytest.mark.parametrize('entry', CORPUS, ids=[entry.id for entry in CORPUS])
def test_corpus_entry(cell_model, entry):
    result = cell_model.check_configuration(entry.q)
    kinds = {contact.kind for contact in result.contacts}
    assert result.ok == entry.ok, (
        '{}: expected ok={} and got {} with contacts {}'.format(
            entry.id, entry.ok, result.ok,
            [(c.kind, c.a, c.b, c.distance) for c in result.contacts]))
    assert set(entry.kinds) <= kinds, (
        '{}: expected kinds {} among {}'.format(entry.id, entry.kinds, sorted(kinds)))
    assert not (set(entry.forbidden_kinds) & kinds), (
        '{}: forbidden kinds {} appeared'.format(
            entry.id, sorted(set(entry.forbidden_kinds) & kinds)))
    if entry.witness is not None:
        pairs = {(contact.a, contact.b) for contact in result.contacts}
        assert entry.witness in pairs, (
            '{}: witness {} is not among {}'.format(entry.id, entry.witness,
                                                    sorted(pairs)))
    if entry.min_clearance_max is not None:
        assert result.min_clearance <= entry.min_clearance_max, (
            '{}: min_clearance {} exceeds the bound {}'.format(
                entry.id, result.min_clearance, entry.min_clearance_max))
    assert result.model_id == 'hcislab_dual_panda_cell'
    assert result.samples_evaluated == 1
    assert (result.sample_index is None) == result.ok


@pytest.mark.parametrize('entry', CORPUS, ids=[entry.id for entry in CORPUS])
def test_contacts_are_sorted_most_violating_first(cell_model, entry):
    result = cell_model.check_configuration(entry.q)
    keys = [(contact.distance - contact.required, contact.kind, contact.a, contact.b)
            for contact in result.contacts]
    assert keys == sorted(keys)


@pytest.mark.parametrize('entry', CORPUS, ids=[entry.id for entry in CORPUS])
def test_contacts_name_volume_ids_never_link_names(cell_model, entry):
    result = cell_model.check_configuration(entry.q)
    for contact in result.contacts:
        if contact.kind == 'joint_limit':
            assert contact.b == ''
            assert '_joint' in contact.a
            continue
        assert contact.a in cell_model._volume_position, contact.a
        if contact.kind == 'containment':
            box, _, face = contact.b.rpartition('.')
            assert box == cell_model.allowed_volume().id
            assert face in ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max')
        elif contact.kind in ('self', 'cross_arm'):
            assert contact.b in cell_model._volume_position, contact.b
        assert contact.arm_id in cell_model.arm_ids()


def test_first_violation_agrees_with_the_full_evaluation(cell_model):
    for entry in CORPUS:
        full = cell_model.check_configuration(entry.q)
        fast = cell_model.check_configuration(entry.q, first_violation=True)
        assert fast.ok == full.ok
        if not full.ok:
            assert len(fast.contacts) == 1
            assert fast.contacts[0].kind in {c.kind for c in full.contacts}


def test_a_clear_pose_stays_clear_as_a_one_point_path(cell_model):
    entry = next(item for item in CORPUS if item.id == 'clear_ready_both_arms')
    result = cell_model.check_path([entry.q])
    assert result.ok
    assert result.samples_evaluated == 1


def test_swept_margins_are_tighter_than_single_configuration_margins(cell_model):
    entry = next(item for item in CORPUS if item.id == 'clear_ready_both_arms')
    single = cell_model.check_configuration(entry.q)
    swept = cell_model.check_path([entry.q, entry.q])
    assert swept.min_clearance < single.min_clearance
    assert abs((single.min_clearance - swept.min_clearance) - 0.01) < 1e-12
