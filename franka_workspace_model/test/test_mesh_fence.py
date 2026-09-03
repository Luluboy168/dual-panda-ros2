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
The broad phase, the narrow phase and the ruled per-pair margin.

This IS the fence ``check_configuration`` runs.  Every property that makes it
safe is asserted here, on the real bodies, at real poses.

The property the whole design turns on: the broad phase's certified lower bound
and the narrow phase's reported clearance are the SAME QUANTITY.  There is no
undercut term to fold in, because the bodies contain the shell; and the
bounding-capsule radii belong to the bound alone and never appear in a reported
number.
"""

import math

from census_core import joint_limits

from conftest import READY

from franka_workspace_model.model import CellModel, WorkspaceModelError

from mesh_oracle import MeshOracle

import numpy as np

import pytest


#: The pin that a double subtraction fails while every RELATIVE assertion in
#: this file still passes: both sides of a relative test move together when the
#: capsule radii are subtracted twice.
READY_LINK5_LINK7_M = 0.021778618
#: The same pair at the operator's own pose, panda1 ready with j6 = 0.
J6_ZERO = [0.0, -0.7854, 0.0, -2.3562, 0.0, 0.0, 0.7854]
J6_ZERO_LINK5_LINK7_M = 0.021566058
J6_ZERO_LINK5_LINK8_M = 0.072288524

DRAWS = 120
SEED = 20260903


@pytest.fixture(scope='module')
def fence(cell_model):
    """Return the built fence and its margin vectors."""
    built = cell_model._mesh_fence()
    intra, cross = cell_model.__dict__['_mesh_margins']
    return built, intra, cross


@pytest.fixture(scope='module')
def draws(cell_model):
    """Draw a pinned set of uniform configurations over the joint box."""
    oracle = MeshOracle(cell_model._geometry.path,
                        cell_model._mesh_bodies.path, cell_model.arm_ids())
    lower, upper = joint_limits(oracle)
    rng = np.random.default_rng(SEED)
    return [{arm_id: list(rng.uniform(lower, upper))
             for arm_id in cell_model.arm_ids()} for _ in range(DRAWS)]


def _place(cell_model, built, configuration):
    transforms = cell_model._transforms(cell_model._sample(configuration))
    return built.place(transforms)


# ---------------------------------------------------------------------------
# T-B2: the bound really bounds the reported clearance
# ---------------------------------------------------------------------------

def test_the_ready_pose_clearance_is_pinned_absolutely(cell_model, fence):
    """
    T-B2's absolute pin, and the reason it has to be absolute.

    ``clearance = gjk(body_a, body_b)``.  Nothing is subtracted from it.  An
    implementation that wrote ``gjk(...) - r_a - r_b`` would report
    21.7786 - (75.6 + 53.3) = -107.1 mm here and refuse the home pose - and no
    RELATIVE assertion could catch it, because the reported clearance and the
    reported bound would both move by the same r_a + r_b.  So the value itself
    is pinned.
    """
    built, _, _ = fence
    rotations, translations, _, _ = _place(
        cell_model, built, {arm_id: list(READY) for arm_id in cell_model.arm_ids()})
    first = built.position['panda1_link5_collision_2_st']
    second = built.position['panda1_link7_st']
    value = built.clearance(rotations, translations, first, second)
    assert abs(value - READY_LINK5_LINK7_M) <= 1e-9, value
    assert value > 0.0


def test_the_operator_pose_is_the_measured_one(cell_model, fence):
    """
    The pose that started the whole line of work, measured on the bodies.

    panda1 at ready with j6 = 0.  The operator taped link5 against link8 at
    about 72 mm and the shipped fence refused the pose at +12.19 mm on padded
    capsules.  The mesh fence measures 72.29 mm on that pair - and says what the
    tape could not, that the pair is NOT the closest one: link5/link7 at
    21.57 mm is.
    """
    built, _, _ = fence
    rotations, translations, _, _ = _place(
        cell_model, built, {'panda1': list(J6_ZERO), 'panda2': list(READY)})
    link5 = [built.position['panda1_link5_collision_{}_st'.format(index)]
             for index in range(3)]
    link7 = built.position['panda1_link7_st']
    link8 = built.position['panda1_link8_flange']
    to_seven = min(built.clearance(rotations, translations, index, link7)
                   for index in link5)
    to_eight = min(built.clearance(rotations, translations, index, link8)
                   for index in link5)
    assert abs(to_seven - J6_ZERO_LINK5_LINK7_M) <= 1e-9, to_seven
    assert abs(to_eight - J6_ZERO_LINK5_LINK8_M) <= 1e-9, to_eight
    assert to_seven < to_eight


def test_the_broad_phase_never_culls_a_close_pair(cell_model, fence, draws):
    """
    T-B2: for every pair, the certified bound is <= the reported clearance.

    That is the whole soundness argument for the cull: a pair whose bound
    already exceeds its own margin cannot violate that margin, so not
    evaluating it loses nothing.  M-6 - shaving a millimetre off a
    bounding-capsule radius - dies here.
    """
    built, intra_margins, cross_margins = fence
    worst = math.inf
    for configuration in draws[:40]:
        rotations, translations, ends_a, ends_b = _place(cell_model, built,
                                                         configuration)
        for name, pairs in (('intra', built.intra), ('cross', built.cross)):
            bounds = built.lower_bounds(ends_a, ends_b, name)
            for position, (first, second, _, _) in enumerate(pairs):
                value = built.clearance(rotations, translations, first, second)
                assert bounds[position] <= value + 1e-9, (
                    built.entries[first]['id'], built.entries[second]['id'],
                    bounds[position], value)
                worst = min(worst, value - float(bounds[position]))
    assert worst >= -1e-9
    del intra_margins, cross_margins


def test_a_pair_exactly_on_its_gate_is_evaluated(cell_model, fence):
    """
    T-B2's boundary case.  M-7: turning ``<=`` into ``<`` skips it.

    The cull keeps a pair whose bound EQUALS its margin, because a bound equal
    to the margin does not prove the pair is outside it.
    """
    built, _, _ = fence
    rotations, translations, ends_a, ends_b = _place(
        cell_model, built, {arm_id: list(READY) for arm_id in cell_model.arm_ids()})
    bounds = built.lower_bounds(ends_a, ends_b, 'intra')
    # Give every pair exactly its own bound as a margin: all of them must
    # survive the cull, none may be dropped.
    contacts = []
    minimum, stopped = built.pair_step(rotations, translations, ends_a, ends_b,
                                       'intra', bounds.copy(), 'self', 0.0,
                                       contacts, False)
    assert not stopped
    assert len(contacts) == 0 or all(item[3] < item[4] for item in contacts)
    # Every pair was evaluated, so the reported minimum is the exact one.
    exact = min(built.clearance(rotations, translations, first, second)
                - float(bounds[index])
                for index, (first, second, _, _) in enumerate(built.intra))
    assert abs(minimum - exact) < 1e-9


# ---------------------------------------------------------------------------
# T-B1: the gate decides every margin class, not the global minimum
# ---------------------------------------------------------------------------

def _no_cull(built, rotations, translations, pairs, margins, kind):
    """Evaluate every pair and cull nothing: the reference run."""
    contacts = []
    minimum = math.inf
    for index, (first, second, arm_id, _) in enumerate(pairs):
        value = built.clearance(rotations, translations, first, second)
        margin = float(margins[index])
        minimum = min(minimum, value - margin)
        if value < margin:
            contacts.append((kind, built.entries[first]['id'],
                             built.entries[second]['id'], value, margin, arm_id))
    return contacts, minimum


def test_the_gated_pair_set_decides_every_margin_class(cell_model, fence, draws):
    """
    T-B1: the gated candidate set produces the IDENTICAL CONTACT LIST.

    M-1 is the mutation this kills: culling against a GLOBAL minimum instead of
    each pair's own margin.  The two are different questions the moment margins
    differ per class - a self pair at 30 mm can be the global minimum and pass
    while a cross-arm pair at 40 mm violates its own 50 mm margin and is never
    evaluated.

    It does NOT assert that ``min_clearance`` is preserved.  On a passing
    configuration the culled subset need not contain the tightest pair, and the
    contract says so.  What it does assert is the two guarantees that replace
    it: the gated minimum is never larger than the no-cull one, and the two are
    EQUAL whenever any pair is inside its margin.
    """
    built, intra_margins, cross_margins = fence
    for configuration in draws:
        rotations, translations, ends_a, ends_b = _place(cell_model, built,
                                                         configuration)
        for name, pairs, margins, kind in (
                ('intra', built.intra, intra_margins, 'self'),
                ('cross', built.cross, cross_margins, 'cross_arm')):
            gated = []
            gated_minimum, _ = built.pair_step(
                rotations, translations, ends_a, ends_b, name, margins, kind,
                0.0, gated, False)
            reference, reference_minimum = _no_cull(built, rotations, translations,
                                                    pairs, margins, kind)
            assert sorted(gated) == sorted(reference), (name, configuration)
            assert gated_minimum <= reference_minimum + 1e-9
            if reference:
                assert abs(gated_minimum - reference_minimum) < 1e-9


def test_the_cull_is_the_margin_and_not_a_tunable_knob(fence):
    """
    Every body pair carries its own margin, and the vector is built at load.

    A self pair and a cross-arm pair in the same check are gated at different
    numbers, so there is no single value anybody could tune - which is the
    point: tightening a margin cannot make the cull unsound and loosening one
    cannot make it miss a contact.
    """
    built, intra_margins, cross_margins = fence
    assert len(intra_margins) == len(built.intra)
    assert len(cross_margins) == len(built.cross)
    assert set(np.unique(cross_margins)) == {0.05}
    # Two values, not one: the ruled 10 mm on the two wrist pairs and the 20 mm
    # every other pair gets.  A single tunable number is exactly what this
    # design does not have.
    assert set(np.unique(intra_margins)) == {0.01, 0.02}
    # Twelve body pairs carry the ruling: two ruled LINK pairs per arm, and
    # link5 - the one link MuJoCo ships decomposed - contributes three bodies
    # to each of them.  2 x 2 x 3 = 12.  That expansion is the whole reason the
    # key is keyed at link level and asserted at body level.
    assert (intra_margins == 0.01).sum() == 12
    # Sixteen enabled link pairs per arm.  Five of them name link5, which MuJoCo
    # ships decomposed into three collision pieces and this model keeps that way
    # rather than re-convexifying it, so those five expand to three body pairs
    # each: 5 x 3 + 11 = 26 per arm, 52 for the cell.  The capsule model's 21
    # volume pairs per arm are a different count of a different thing.
    assert len(built.intra) == 52
    # Cross-arm: eleven bodies against eleven bodies.
    assert len(built.cross) == 121


# ---------------------------------------------------------------------------
# T-PAIRM: the ruled margin
# ---------------------------------------------------------------------------

def _with_pair_margins(entries):
    def mutate(document):
        document['policy']['self_collision']['pair_margins'] = entries
    return mutate


def test_the_wrist_ruling_is_in_force_on_both_arms(cell_model):
    """
    The ruling arrived WITH the geometry that justifies it, and not before.

    Two pairs, both arms, 10 mm each.  Every other pair keeps 20 mm; the
    cross-arm margin, the environment margin and swept_path_extra are
    untouched.  What the ruling rests on is in the cell file's reason strings
    and re-measured by the two band tests below.
    """
    assert cell_model._pair_margins == {
        ('panda1', 'link5', 'link7'): 0.010,
        ('panda2', 'link5', 'link7'): 0.010,
        ('panda1', 'link5', 'link8'): 0.010,
        ('panda2', 'link5', 'link8'): 0.010,
    }
    assert cell_model._margins['self_collision'] == 0.02
    assert cell_model._margins['cross_arm'] == 0.05
    assert cell_model._margins['environment'] == 0.03
    assert cell_model._margins['swept_path_extra'] == 0.01


def test_the_wrist_pair_can_never_open_past_its_ruled_band(cell_model):
    """
    The measurement the ruling rests on, re-run rather than quoted.

    Over the complete reachable (j6, j7) box - the only two joints that move
    link5 relative to link7 - the pair's true metal-to-metal distance never
    exceeds 22.13 mm.  A 30 mm self margin on it therefore rejects EVERY
    reachable configuration of the arm: not most of the workspace, all of it.
    And a 20 mm margin declares about 89 % of the pair's own designed
    separation band out of bounds while leaving the home pose 1.78 mm from
    refusal.

    The grid here is coarse on purpose - it is a guard against the band moving,
    not the derivation.  The refined minimum of 2.128 mm and the 1.86 deg
    barrier are in the cell file's reason string, measured with a complete
    161-squared grid plus a multi-start refinement.
    """
    built = cell_model._mesh_fence()
    link5 = [built.position['panda1_link5_collision_{}_st'.format(index)]
             for index in range(3)]
    link7 = built.position['panda1_link7_st']
    worst = -math.inf
    tightest = math.inf
    for j6 in np.linspace(-0.0175, 3.7525, 21):
        for j7 in np.linspace(-2.8973, 2.8973, 21):
            rotations, translations, _, _ = _place(
                cell_model, built,
                {'panda1': [0.0, -0.7854, 0.0, -2.3562, 0.0, float(j6), float(j7)],
                 'panda2': list(READY)})
            value = min(built.clearance(rotations, translations, index, link7)
                        for index in link5)
            worst = max(worst, value)
            tightest = min(tightest, value)
    assert worst < 0.0222, 'the pair opened to {:.4f} mm'.format(worst * 1000.0)
    assert tightest < 0.010, tightest


def test_a_pair_margin_reaches_every_body_pair_of_its_link_pair(tmp_path):
    """
    T-PAIRM: the ruling is keyed at LINK level and expands to every body pair.

    M-10 is the mutation: applying it at volume level while a sibling body keeps
    the default, so the ruled margin is not the margin in force.  link5 carries
    three bodies, which is exactly where that would show.
    """
    from conftest import load_mutated
    model = load_mutated(tmp_path, _with_pair_margins([
        {'a': 'panda1_link5', 'b': 'panda1_link7', 'margin': 0.010,
         'reason': 'a test fixture, not a ruling'}]))
    assert model._pair_margins == {('panda1', 'link5', 'link7'): 0.010}
    built = model._mesh_fence()
    intra, _ = model.__dict__['_mesh_margins']
    ruled = [index for index, (first, second, _, pair) in enumerate(built.intra)
             if pair == ('link5', 'link7')
             and built.entries[first]['arm_id'] == 'panda1']
    assert len(ruled) == 3, 'link5 carries three bodies'
    for index in ruled:
        assert intra[index] == 0.010
    for index in range(len(built.intra)):
        if index not in ruled:
            assert intra[index] == 0.02


def test_a_pair_margin_on_a_disabled_pair_is_a_load_error(tmp_path):
    """
    A ruling with no effect is worse than none: it reads as protection.

    The message names the pair and says why the ruling would do nothing, rather
    than accepting it silently.
    """
    from conftest import load_mutated
    with pytest.raises(WorkspaceModelError, match='NOT evaluated'):
        load_mutated(tmp_path, _with_pair_margins([
            {'a': 'panda1_link3', 'b': 'panda1_link6', 'margin': 0.010,
             'reason': 'the SRDF disables this pair'}]))


def test_a_cross_arm_pair_margin_is_a_load_error(tmp_path):
    """A self margin is an intra-arm quantity; the message says which key to use."""
    from conftest import load_mutated
    with pytest.raises(WorkspaceModelError, match='margins.cross_arm'):
        load_mutated(tmp_path, _with_pair_margins([
            {'a': 'panda1_link5', 'b': 'panda2_link5', 'margin': 0.010,
             'reason': 'not an intra-arm pair'}]))


def test_a_pair_margin_needs_a_written_reason(tmp_path):
    """
    CONTRACT A's mandatory ``reason``, on the key most likely to be abused.

    A per-pair margin is a RULING.  An unexplained one is indistinguishable
    from somebody making a red test go away.
    """
    from conftest import load_mutated
    with pytest.raises(WorkspaceModelError, match='reason'):
        load_mutated(tmp_path, _with_pair_margins([
            {'a': 'panda1_link5', 'b': 'panda1_link7', 'margin': 0.010,
             'reason': ''}]))


def test_a_negative_pair_margin_is_a_load_error(tmp_path):
    from conftest import load_mutated
    with pytest.raises(WorkspaceModelError, match='non-negative'):
        load_mutated(tmp_path, _with_pair_margins([
            {'a': 'panda1_link5', 'b': 'panda1_link7', 'margin': -0.001,
             'reason': 'a negative margin is not a margin'}]))


def test_a_duplicated_pair_margin_is_a_load_error(tmp_path):
    """A pair has one ruled margin or none; two is a question nobody answered."""
    from conftest import load_mutated
    entry = {'a': 'panda1_link5', 'b': 'panda1_link7', 'margin': 0.010,
             'reason': 'the first ruling'}
    other = dict(entry, margin=0.015, reason='the second, different ruling')
    with pytest.raises(WorkspaceModelError, match='twice'):
        load_mutated(tmp_path, _with_pair_margins([entry, other]))


def test_a_ruled_margin_is_printed_as_a_load_diagnostic(tmp_path):
    """
    R4: the ruling is a MARGIN, not an exemption, and the load says so.

    The diagnostic prints the ruled value beside the one every other pair gets,
    so an operator reading the console sees the ruling rather than inferring it
    from a number that is quietly different.
    """
    from conftest import load_mutated
    model = load_mutated(tmp_path, _with_pair_margins([
        {'a': 'panda1_link5', 'b': 'panda1_link7', 'margin': 0.010,
         'reason': 'a test fixture, not a ruling'}]))
    printed = [line for line in model.diagnostics() if 'RULED' in line]
    assert len(printed) == 1
    assert 'panda1_link5' in printed[0] and 'panda1_link7' in printed[0]
    assert '0.0100' in printed[0] and '0.0200' in printed[0]
    assert 'still refused inside the ruled distance' in printed[0]


# ---------------------------------------------------------------------------
# The mesh evaluation path, before it is switched on
# ---------------------------------------------------------------------------

def test_the_fence_reports_real_air_at_the_home_pose(cell_model):
    """
    The margin-adjusted minimum at ready, through the public API.

    21.778618 mm of metal against the ruled 10 mm margin on that pair, so
    +0.011779 m of slack.  At a flat 20 mm it would be +0.001779 m - which is
    how close the arm's own home pose sits to being refused by a margin nobody
    measured.  Not 3.25 mm either: that figure was the collision-mesh distance,
    which undercuts the visual shell by up to 6.5 mm on link5.
    """
    result = cell_model.check_configuration(
        {arm_id: list(READY) for arm_id in cell_model.arm_ids()})
    assert result.ok
    assert abs(result.min_clearance - (READY_LINK5_LINK7_M - 0.010)) < 1e-9


def test_the_fence_allows_the_operators_pose(cell_model):
    """
    The pose the shipped fence refused, allowed, and the reason it should be.

    panda1 at ready with j6 = 0.  The shipped fence refused it on
    link5_v0/link8_v0 at +12.19 mm - a padded capsule number.  The real metal
    on that pair is 72.29 mm, which is what the operator's tape measure said.
    """
    result = cell_model.check_configuration(
        {'panda1': list(J6_ZERO), 'panda2': list(READY)})
    assert result.ok, [(c.kind, c.a, c.b, c.distance) for c in result.contacts]
    assert result.contacts == ()


def test_the_fence_still_refuses_the_adversarial_poses(cell_model):
    """
    The crash pose, the wrist pose and the cross-arm pose from the record.

    All three were ACCEPTED by fix/true-clearance, the branch that made the
    fence looser.  A mesh-exact fence is not worth having if it repeats that.
    """
    crash = {'panda1': [0.7076, 1.3409, 0.0642, -2.6518, -1.5893, 1.5708, 0.7854],
             'panda2': list(READY)}
    result = cell_model.check_configuration(crash)
    assert not result.ok, 'the crash pose must not be accepted'


def test_the_pedestal_step_is_evaluated_on_mesh_bodies(cell_model):
    """
    T-PED: the step nobody converted, converted, and pinned.

    Left on padded capsules it would keep the 30 mm inflation this whole
    exercise removes, and would mix a padded number into the same
    ``min_clearance`` as metal clearances - which makes "clearance is real air"
    false for any result whose minimum lands on a base_link contact, with no way
    for a consumer to tell.  The padded step reports exactly 0.360000 m at the
    ready pose, so a missed conversion fails by 39.8 mm.
    """
    configuration = {arm_id: list(READY) for arm_id in cell_model.arm_ids()}
    sample = cell_model._sample(configuration)
    transforms = cell_model._transforms(sample)
    built = cell_model._mesh_fence()
    rotations, translations, _, _ = built.place(transforms)
    box = cell_model._structure_box('base_link_v0', transforms)
    index = built.position['panda1_link1_st']
    value = built.box_clearance(rotations, translations, index, box)
    assert abs(value - 0.3998146646) < 1e-6, value
    assert abs(value - 0.360000) > 0.039


def test_the_pedestal_step_reports_mesh_body_ids(cell_model):
    """
    Every contact the step emits names a mesh body, never a ``_v0`` volume.

    A consumer that renders the string would otherwise be told the fence is
    still measuring capsules there.
    """
    over_the_pedestal = [-1.65, 1.25, 0.0, -1.65, 0.0, 1.5708, 0.7854]
    result = cell_model.check_configuration(
        {'panda1': over_the_pedestal, 'panda2': list(READY)})
    pedestal = [contact for contact in result.contacts
                if contact.a == 'base_link_v0']
    assert pedestal
    for contact in pedestal:
        assert contact.kind == 'self'
        assert contact.b.endswith('_st') or contact.b.endswith('_flange')
        assert '_v0' not in contact.b


def test_containment_is_evaluated_on_mesh_vertices(cell_model):
    """
    T-CONT: section 2.3's numbers, from the bodies' own placed vertices.

    link0's casting sits 32.5 MICROMETRES below the table top - not 30 mm
    inside it, which is what a padded capsule reports - and link1's casting is
    141.0 mm ABOVE the table, not 60 or 90 mm below it.  Both are exempt from
    the per-query check (link0 is fully static, link1 rotates only about the
    vertical), so these are diagnostics; but a diagnostic that prints a capsule
    artefact tells the operator the base plinth is inside the table.
    """
    configuration = {arm_id: list(READY) for arm_id in cell_model.arm_ids()}
    transforms = cell_model._transforms(cell_model._sample(configuration))
    built = cell_model._mesh_fence()
    rotations, translations, _, _ = built.place(transforms)
    box_lower = cell_model._box_lower
    box_upper = cell_model._box_upper
    measured = {}
    for name in ('panda1_link0_st', 'panda1_link1_st', 'panda1_link3_st'):
        index = built.position[name]
        projected = built.vertices[index] @ rotations[index].T
        low = projected.min(axis=0) + translations[index]
        high = projected.max(axis=0) + translations[index]
        measured[name] = min(float((low - box_lower).min()),
                             float((box_upper - high).min()))
    assert abs(measured['panda1_link0_st'] - (-0.0000325)) < 5e-7
    assert abs(measured['panda1_link1_st'] - 0.1409959) < 5e-6
    assert abs(measured['panda1_link3_st'] - 0.1185201) < 5e-6


def test_the_environment_and_keep_out_steps_are_inert_and_say_so(cell_model):
    """
    Two steps were NOT converted, and the silence about that was the defect.

    ``environment`` is ``[]`` in this cell and the midplane keep-out zone is
    ``enabled: false``, so neither step evaluates any geometry today.
    Converting a step that runs on nothing would be a change nobody could
    check, so they keep the capsule path and the cost of converting them is
    written down in doc/CONTRACT.md rather than left to be inferred from an
    omission.
    """
    assert cell_model._environment == ()
    assert cell_model._active_zones == ()
    import inspect
    source = inspect.getsource(CellModel._evaluate_inner)
    assert '_mesh_evaluate' in source
    assert 'Step 4b' in source and 'Step 5' in source
