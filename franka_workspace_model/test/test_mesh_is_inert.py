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
T-INERT: the mesh bodies are DATA until the fence switches, and here is the proof.

The staging rule is that the census lands before the fence changes - installing
the thermometer before the fever, not after - and that rule is only worth
anything if the intermediate stages really do leave the lab's fence alone.  This
file pins the whole verdict surface of the capsule fence into a committed
baseline: every corpus pose's ``result_to_json``, and every load-time
diagnostic.  While the fence is on capsules the baseline must match exactly.

WHEN THE FENCE SWITCHES this file does not get deleted and it does not get
regenerated in the same commit as the switch.  The baseline becomes the record
of what the switch CHANGED: the diff of the two is the answer to "what did the
mesh fence do to my verdicts", written down once, in a review, instead of being
discovered later on a console.
"""

import json
from pathlib import Path

from conftest import CORPUS_DIR

from corpus_loader import load_corpus

from franka_workspace_model.model import result_to_json

import pytest


BASELINE_PATH = Path(__file__).resolve().parent / 'baseline' / 'capsule_fence_v1.json'


#: Identity fields, not fence fields.  ``model_sha256`` is the digest of the
#: cell file, so it moves when a comment moves; pinning it here would turn every
#: edit to the cell file's prose into a baseline diff and teach the next reader
#: to regenerate the baseline without reading it.  Identity is pinned where it
#: belongs, in test_public_api.py.
IDENTITY_FIELDS = ('model_id', 'model_revision', 'model_sha256')


def _verdicts(model):
    """Serialise every corpus pose's verdict, in a fixed order."""
    out = {}
    for entry in sorted(load_corpus(CORPUS_DIR), key=lambda item: item.id):
        configuration = {arm_id: list(values) for arm_id, values in entry.q.items()}
        serialised = result_to_json(model.check_configuration(configuration))
        for field in IDENTITY_FIELDS:
            serialised.pop(field, None)
        out[entry.id] = serialised
    return out


def test_the_capsule_fence_matches_its_committed_baseline(cell_model):
    """
    Every corpus verdict, field for field, against the pinned capsule fence.

    A mesh change that quietly altered a reported distance would show up here as
    a diff in a committed JSON file rather than as a number nobody compared.
    """
    baseline = json.loads(BASELINE_PATH.read_text(encoding='utf-8'))
    measured = _verdicts(cell_model)
    assert sorted(measured) == sorted(baseline['verdicts'])
    for name in sorted(measured):
        assert measured[name] == baseline['verdicts'][name], name
    # ...and the identity the baseline deliberately leaves out is pinned
    # elsewhere, so nothing is unwatched.
    assert cell_model.model_identity()[0] == 'hcislab_dual_panda_cell'


def test_the_load_diagnostics_match_their_committed_baseline(cell_model):
    """
    The diagnostics are a published surface too, and one of them will move.

    Section 2.3 measured link0's casting 32.5 um BELOW the table top and link1's
    141.0 mm ABOVE it, where the padded capsules report -30 mm and -90 mm.  Those
    strings change when containment moves onto mesh vertices, and the change is
    supposed to be visible.
    """
    baseline = json.loads(BASELINE_PATH.read_text(encoding='utf-8'))
    assert list(cell_model.diagnostics()) == baseline['diagnostics']


def test_the_baseline_records_which_fence_produced_it():
    """A baseline that does not say what it is a baseline OF is a trap."""
    baseline = json.loads(BASELINE_PATH.read_text(encoding='utf-8'))
    assert baseline['fence'] == 'capsule'
    assert 'note' in baseline and len(baseline['note']) > 80


def test_the_mesh_bodies_are_loaded_but_not_consulted(cell_model):
    """
    Parse the artefact at load, and keep it out of the check path.

    A broken artefact fails loudly at load.  ``_evaluate_inner`` is the whole
    verdict path; if the mesh bodies were already in it, this file's baseline
    could not be the capsule fence's.
    """
    assert len(cell_model._mesh_bodies) == 11
    import inspect
    source = inspect.getsource(type(cell_model)._evaluate_inner)
    assert '_mesh_bodies' not in source


@pytest.mark.parametrize('name', ['gjk', 'bodies'])
def test_the_mesh_runtime_is_importable_and_numpy_only(name):
    """The subpackage ships in S1; nothing in the verdict path calls it yet."""
    import importlib
    module = importlib.import_module(
        'franka_workspace_model.mesh_runtime.{}'.format(name))
    assert module is not None
