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

"""T12 and T13: the same answer every time, and never a looser one than L1's."""

import json

from conftest import READY

from franka_workspace_model.model import result_to_json

import numpy as np


# The controller's per-joint box, read from the same policy the checker reads.
# T13 is a subset relation, not a spot check: no `allowed: true` result anywhere
# may produce a q_target outside this box.
POSITION_LOWER = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
POSITION_UPPER = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)

BOTH_READY = {'panda1': list(READY), 'panda2': list(READY)}
# The ready pose is the one that matters for T12: two cross-arm volume pairs tie
# there at exactly +0.700000, so the sort rule is the only thing that can order
# them, and any set or dict iteration order that leaked would show up here.
TIED_PAIRS = (('panda1_link2_v0', 'panda2_link2_v0'),
              ('panda1_link4_v0', 'panda2_link4_v0'))


def test_the_structural_tie_at_ready_is_exact(cell_model):
    sample = cell_model._sample(BOTH_READY)
    ends_a, ends_b, _ = cell_model._place(sample)
    from franka_workspace_model.geometry import segment_segment_distance
    values = []
    for first, second in TIED_PAIRS:
        one = cell_model._volume_position[first]
        two = cell_model._volume_position[second]
        values.append(segment_segment_distance(ends_a[one], ends_b[one],
                                               ends_a[two], ends_b[two])
                      - cell_model._radii[one] - cell_model._radii[two])
    # Exact in real arithmetic - 2*(0.50 - 0.06) - 0.18 - and equal to within one
    # ulp in doubles, because the two forward-kinematic chains that reach link2
    # and link4 round differently.  Both serialise to the same six decimals, so
    # the contact order at this pose is decided by the sort rule and by nothing
    # else; that is what makes it the pose T12 has to include.
    assert abs(values[0] - values[1]) <= 2.3e-16
    assert round(values[0], 12) == round(values[1], 12) == 0.7


def test_repeated_runs_serialise_byte_identically(cell_model):
    """T12: no set or dict iteration order leaks into the wire form."""
    configurations = [
        BOTH_READY,
        {'panda1': [-1.4, 0.7, 0.0, -1.6, 0.0, 1.5708, 0.7854],
         'panda2': [1.4, 0.7, 0.0, -1.6, 0.0, 1.5708, -0.7854]},
        {'panda1': [0.0, 1.1, 0.0, -1.75, 0.0, 1.5708, 0.7854], 'panda2': list(READY)},
    ]
    for configuration in configurations:
        first = json.dumps(result_to_json(cell_model.check_configuration(configuration)),
                           sort_keys=True)
        for _ in range(100):
            again = json.dumps(
                result_to_json(cell_model.check_configuration(configuration)),
                sort_keys=True)
            assert again == first


def test_the_wire_form_carries_exactly_the_dataclass_fields(cell_model):
    payload = result_to_json(cell_model.check_configuration(
        {'panda1': [0.0, 1.1, 0.0, -1.75, 0.0, 1.5708, 0.7854], 'panda2': list(READY)}))
    assert set(payload) == {'ok', 'min_clearance', 'contacts', 'sample_index',
                            'samples_evaluated', 'model_id', 'model_revision',
                            'model_sha256'}
    assert set(payload['contacts'][0]) == {'kind', 'a', 'b', 'distance', 'required',
                                           'arm_id'}
    assert json.dumps(payload)


def test_an_unevaluated_result_serialises_infinity_as_a_string(cell_model):
    from franka_workspace_model.model import CheckResult
    result = CheckResult(ok=True, min_clearance=float('inf'), contacts=(),
                         sample_index=None, samples_evaluated=0, model_id='x',
                         model_revision=1, model_sha256='y')
    payload = result_to_json(result)
    assert payload['min_clearance'] == 'inf'
    assert payload['sample_index'] is None
    assert json.dumps(payload)


def test_every_approved_jog_target_lies_inside_the_controller_box(cell_model):
    """T13: A_L2 is a subset of A_L1.  One counterexample fails the suite."""
    generator = np.random.default_rng(20260904)
    lower = np.array(POSITION_LOWER)
    upper = np.array(POSITION_UPPER)
    checked = 0
    for _ in range(10000):
        values = generator.uniform(lower - 0.2, upper + 0.2, size=(2, 7))
        configuration = {'panda1': list(values[0]), 'panda2': list(values[1])}
        arm_id = 'panda1' if generator.random() < 0.5 else 'panda2'
        joint_index = int(generator.integers(0, 7))
        delta = float(generator.uniform(-0.5, 0.5))
        result = cell_model.check_jog(arm_id, configuration, joint_index, delta)
        checked += 1
        if not result.allowed:
            continue
        target = np.array(result.q_target)
        assert np.all(target >= lower - 1e-12), (arm_id, joint_index, delta, target)
        assert np.all(target <= upper + 1e-12), (arm_id, joint_index, delta, target)
    assert checked == 10000


def test_the_corpus_never_approves_a_jog_out_of_the_box(cell_model):
    from corpus_loader import load_corpus
    from conftest import CORPUS_DIR
    lower = np.array(POSITION_LOWER)
    upper = np.array(POSITION_UPPER)
    for entry in load_corpus(CORPUS_DIR):
        for arm_id in entry.q:
            for joint_index in range(7):
                for delta in (-0.2, 0.2):
                    result = cell_model.check_jog(arm_id, entry.q, joint_index, delta)
                    if not result.allowed:
                        continue
                    target = np.array(result.q_target)
                    assert np.all(target >= lower - 1e-12)
                    assert np.all(target <= upper + 1e-12)
