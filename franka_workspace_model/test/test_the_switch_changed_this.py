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
What the switch changed, as two committed files and a diff between them.

Before the fence moved onto mesh bodies this file was ``test_mesh_is_inert.py``
and it asserted one baseline: the padded-capsule fence's whole verdict surface,
pinned so that S1, S2 and S3 could be shown to leave the lab's fence alone.

That baseline is NOT deleted and it was NOT regenerated in the switch's commit.
It stays exactly as it was measured, and a second one is committed beside it.
The diff of the two is the answer to "what did the mesh fence do to my
verdicts", written down once, in a review, instead of being discovered later on
a console - and both are asserted, so neither can move without somebody reading
the other.

Everything below is a *consequence* of geometry, not a policy: the mesh fence
measures the castings and the visual shells, the capsule fence measured radii
carrying 30 mm of inflation, and every difference in the table is one or the
other of those two facts.
"""

import json
from pathlib import Path

from conftest import CORPUS_DIR

from corpus_loader import load_corpus

from franka_workspace_model.model import result_to_json


BASELINE_DIRECTORY = Path(__file__).resolve().parent / 'baseline'
CAPSULE_BASELINE = BASELINE_DIRECTORY / 'capsule_fence_v1.json'
MESH_BASELINE = BASELINE_DIRECTORY / 'mesh_fence_v1.json'

#: Identity fields, not fence fields.  ``model_sha256`` is the digest of the
#: cell file, so it moves when a comment moves; pinning it here would turn every
#: edit to the cell file's prose into a baseline diff and teach the next reader
#: to regenerate the baseline without reading it.  Identity is pinned where it
#: belongs, in test_public_api.py.
IDENTITY_FIELDS = ('model_id', 'model_revision', 'model_sha256')

#: The entry the mesh work ADDS: panda1 at ready with j6 = 0, the pose the
#: operator taped and the shipped fence refused.
ADDED_ENTRY = 'clear_wrist_unfolded_j6_zero'

#: Every corpus entry whose reported verdict differs between the two fences,
#: with the reason in one line.  A change to this dictionary is a change to
#: what the switch did, and it has to be argued for.
WHAT_CHANGED = {
    'clear_ready_both_arms':
        'min_clearance +0.003238 -> +0.011779: the capsule fence measured '
        'padded radii and the mesh fence measures 21.778618 mm of real metal '
        'against the ruled 10 mm wrist margin',
    'clear_spread_outward_mirrored':
        'min_clearance moves for the same reason; verdict unchanged',
    'containment_below_table_static_volumes':
        'min_clearance moves; the exemptions and the verdict do not',
    'containment_below_table_wrist':
        'the witness names link7_st and the penetration is 125.354 mm rather '
        'than the padded 140+; verdict unchanged',
    'containment_outside_x_min_shoulder_back':
        'the witness names link3_st; link4_st is now the deepest volume, which '
        'the entry records; verdict unchanged',
    'cross_arm_extended_toward_midplane':
        'the two forearms are 12.405 mm apart, not 100 mm overlapped; the '
        'witness names the link5 collision piece that binds; verdict unchanged',
    'environment_empty_list_reports_containment_not_environment':
        'containment rather than environment, still, and 89.852 mm rather than '
        '120; verdict and kind unchanged',
    'joint_limit_all_zero':
        'the self contact DISAPPEARS: at q = 0 link5 and link7 are 21.7869 mm '
        'apart and the capsule contact was entirely inflation; the joint-limit '
        'contact and the verdict are unchanged',
    'keep_out_disabled_midplane_is_not_evaluated':
        'the cross-arm number moves with the entry above; the disabled zone is '
        'still not evaluated, which is what the entry is about',
    'self_wrist_folded_link7_over_link2':
        'the witness names link2_st/link7_st and the penetration is 11.583 mm '
        'rather than the padded 50+; verdict unchanged',
}


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


def test_the_mesh_fence_matches_its_committed_baseline(cell_model):
    """
    Every corpus verdict, field for field, against the pinned mesh fence.

    This is the same assertion the capsule baseline used to carry, moved onto
    the fence that now runs.  A change that quietly altered a reported distance
    shows up as a diff in a committed JSON file rather than as a number nobody
    compared.
    """
    baseline = json.loads(MESH_BASELINE.read_text(encoding='utf-8'))
    measured = _verdicts(cell_model)
    assert sorted(measured) == sorted(baseline['verdicts'])
    for name in sorted(measured):
        assert measured[name] == baseline['verdicts'][name], name
    assert cell_model.model_identity()[0] == 'hcislab_dual_panda_cell'


def test_the_load_diagnostics_match_their_committed_baseline(cell_model):
    """
    The diagnostics are a published surface too, and four of them are new.

    The ruled wrist margins print themselves at load - one line per pair per
    arm - so an operator reading the console sees the ruling rather than
    inferring it from a number that is quietly different.
    """
    baseline = json.loads(MESH_BASELINE.read_text(encoding='utf-8'))
    assert list(cell_model.diagnostics()) == baseline['diagnostics']
    ruled = [line for line in cell_model.diagnostics() if 'RULED' in line]
    assert len(ruled) == 4
    for line in ruled:
        assert 'still refused inside the ruled distance' in line


def test_the_capsule_baseline_is_kept_exactly_as_it_was_measured():
    """
    The "before" file is history and is not regenerated.

    A baseline that gets refreshed alongside the change it is supposed to
    measure is not a baseline; it is a rubber stamp.
    """
    capsule = json.loads(CAPSULE_BASELINE.read_text(encoding='utf-8'))
    assert capsule['fence'] == 'capsule'
    assert len(capsule['verdicts']) == 10
    assert ADDED_ENTRY not in capsule['verdicts']
    mesh = json.loads(MESH_BASELINE.read_text(encoding='utf-8'))
    assert mesh['fence'] == 'mesh'
    assert ADDED_ENTRY in mesh['verdicts']


def test_the_switch_changed_exactly_what_is_written_down():
    """
    The diff of the two baselines, enumerated, with a reason for each line.

    If a future change moves a verdict this file does not name, this test says
    so by name.  If it stops moving one that is named, it says that too.  The
    point is that "what did the fence start doing differently" is a question
    with a written answer rather than an archaeology exercise.
    """
    capsule = json.loads(CAPSULE_BASELINE.read_text(encoding='utf-8'))['verdicts']
    mesh = json.loads(MESH_BASELINE.read_text(encoding='utf-8'))['verdicts']
    assert set(mesh) - set(capsule) == {ADDED_ENTRY}
    assert not set(capsule) - set(mesh)
    moved = {name for name in capsule if capsule[name] != mesh[name]}
    assert moved == set(WHAT_CHANGED), {
        'unexplained': sorted(moved - set(WHAT_CHANGED)),
        'explained but unchanged': sorted(set(WHAT_CHANGED) - moved)}


def test_no_verdict_flipped_from_refused_to_allowed():
    """
    THE ONE PROPERTY THAT WOULD HAVE CAUGHT ``fix/true-clearance``.

    That branch made the fence looser and passed every test in this package.
    The mesh fence measures real geometry rather than padding, so it reports
    less penetration on poses that were already refused - but not one corpus
    pose the capsule fence refused is now allowed.  Where a pose IS newly
    allowed it is a new entry with its own derivation, not a silent flip.

    (The acceptance census is the version of this question asked over twenty
    thousand draws instead of ten poses.  This is the version a reader can
    check by eye.)
    """
    capsule = json.loads(CAPSULE_BASELINE.read_text(encoding='utf-8'))['verdicts']
    mesh = json.loads(MESH_BASELINE.read_text(encoding='utf-8'))['verdicts']
    for name in sorted(capsule):
        if not capsule[name]['ok']:
            assert not mesh[name]['ok'], (
                '{} was refused by the capsule fence and is allowed by the '
                'mesh fence'.format(name))


def test_the_fence_really_is_the_mesh_one_now(cell_model):
    """The switch, asserted where somebody would look for it."""
    import inspect
    source = inspect.getsource(type(cell_model)._evaluate_inner)
    assert '_mesh_evaluate' in source
    assert len(cell_model._mesh_bodies) == 11
    fence = cell_model._mesh_fence()
    assert len(fence.entries) == 22
    assert len(fence.intra) == 52 and len(fence.cross) == 121
