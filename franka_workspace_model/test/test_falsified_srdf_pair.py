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
link2 against link6: the inherited SRDF says "Never" and it is not never.

``franka_moveit_config/srdf/panda_arm.xacro`` and ``dual_panda_arm.xacro`` both
carry ``<disable_collisions link1="${arm_id}_link2" link2="${arm_id}_link6"
reason="Never"/>``.  There is a configuration, every joint strictly inside its
own URDF limit, at which the two links' collision meshes interpenetrate.  The
SRDF is inherited and is never edited (ruling 4), so the correction lands in the
cell file as an ``extra_enabled_pairs`` delta with its measurement written into
the mandatory ``reason``.

The witness ``q`` below is a MEASUREMENT, not a checker output: it was found by
maximising the link2/link6 penetration over the joint box with a linear program
over the two collision hulls' half-spaces, which shares no code with this
package.  At it the meshes overlap by 0.954 mm and the model's own padded
capsules by 1.036 mm.  Nothing in this file asks the checker what the answer is;
it asks whether the checker now has an opinion at all.

This delta only ever TIGHTENS.  A pair that was not being evaluated cannot
become looser by being evaluated, which is why the change needs no mesh work
behind it and lands before any of it.
"""

from conftest import CELL_MODEL_PATH, load_mutated, READY

import pytest


#: Every joint inside its URDF limit; the smallest slack is 0.004213 rad on j4.
WITNESS = [2.313281, -0.882717, 0.476635, -3.067587, -2.642360, 3.401809, 0.880777]

FALSIFIED_PAIR = ('link2', 'link6')


def _drop_the_delta(document):
    document['policy']['self_collision']['extra_enabled_pairs'] = [
        entry for entry in document['policy']['self_collision']['extra_enabled_pairs']
        if not (entry['a'].endswith('_link2') and entry['b'].endswith('_link6'))]


def _pair_set(model):
    """Return the link pairs the fence evaluates, as it sees them."""
    return {(first, second) for first, second, _ in model._intra_pairs}


def _contact(result, arm_id):
    """Find the link2/link6 contact, whatever its bodies are called."""
    return [contact for contact in result.contacts
            if contact.a.startswith('{}_link2'.format(arm_id))
            and contact.b.startswith('{}_link6'.format(arm_id))]


def test_the_shipped_model_evaluates_link2_against_link6(cell_model):
    """Without the delta the pair is structurally absent from the fence."""
    pairs = _pair_set(cell_model)
    for arm_id in cell_model.arm_ids():
        assert ('{}_link2_v0'.format(arm_id), '{}_link6_v0'.format(arm_id)) in pairs


def test_without_the_delta_the_pair_is_not_checked_at_all(tmp_path):
    """The state this delta corrects: no opinion, on a pair that can touch."""
    without = load_mutated(tmp_path, _drop_the_delta)
    pairs = _pair_set(without)
    for arm_id in without.arm_ids():
        assert ('{}_link2_v0'.format(arm_id), '{}_link6_v0'.format(arm_id)) not in pairs


@pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
def test_the_witness_pose_produces_a_link2_link6_contact(cell_model, arm_id):
    """The pair is not merely present: at the measured witness it fires."""
    configuration = {name: list(READY) for name in cell_model.arm_ids()}
    configuration[arm_id] = list(WITNESS)
    result = cell_model.check_configuration(configuration)
    assert not result.ok
    named = _contact(result, arm_id)
    assert named, [(c.a, c.b) for c in result.contacts]
    assert named[0].kind == 'self'
    assert named[0].arm_id == arm_id
    assert named[0].distance < named[0].required


def test_the_pair_reads_the_same_on_either_arm(cell_model):
    """
    An intra-arm distance does not depend on where the arm is bolted.

    The reason string says "on either arm"; this is that sentence as a test, and
    it is what lets one measured witness justify both delta entries.
    """
    values = []
    for arm_id in cell_model.arm_ids():
        configuration = {name: list(READY) for name in cell_model.arm_ids()}
        configuration[arm_id] = list(WITNESS)
        contact = _contact(cell_model.check_configuration(configuration), arm_id)
        assert contact
        values.append(contact[0].distance)
    assert abs(values[0] - values[1]) < 1e-12


def test_the_delta_only_ever_tightens(tmp_path, cell_model):
    """Enabling a pair adds pairs to the fence; it can never remove one."""
    without = load_mutated(tmp_path, _drop_the_delta)
    assert _pair_set(without) < _pair_set(cell_model)
    added = _pair_set(cell_model) - _pair_set(without)
    assert added == {('{}_link2_v0'.format(arm_id), '{}_link6_v0'.format(arm_id))
                     for arm_id in cell_model.arm_ids()}


def test_the_home_pose_is_unaffected(cell_model):
    """The delta costs nothing at the pose the lab actually sits in."""
    result = cell_model.check_configuration(
        {arm_id: list(READY) for arm_id in cell_model.arm_ids()})
    assert result.ok, [(c.kind, c.a, c.b) for c in result.contacts]


def test_the_srdf_itself_is_not_edited():
    """
    Ruling 4: the inherited description is read, never corrected in place.

    A future author who "fixes" the SRDF instead would make this package's
    correction invisible to every other consumer of the same file, and would put
    a local edit in an inherited tree.  The delta is the supported route.
    """
    from conftest import REPOSITORY_ROOT
    for name in ('panda_arm.xacro', 'dual_panda_arm.xacro'):
        text = (REPOSITORY_ROOT / 'franka_moveit_config' / 'srdf' / name).read_text(
            encoding='utf-8')
        assert 'link1="${arm_id}_link2" link2="${arm_id}_link6"' in text
        assert 'reason="Never"' in text


def test_the_delta_carries_its_measurement_in_the_reason():
    """CONTRACT A's mandatory reason is where the falsification is recorded."""
    import yaml
    document = yaml.safe_load(CELL_MODEL_PATH.read_text(encoding='utf-8'))
    entries = [entry for entry in
               document['policy']['self_collision']['extra_enabled_pairs']
               if entry['a'].endswith('_link2') and entry['b'].endswith('_link6')]
    assert len(entries) == 2
    for entry in entries:
        assert 'Never' in entry['reason']
        assert '0.954 mm' in entry['reason']
        assert '2.313281' in entry['reason']
