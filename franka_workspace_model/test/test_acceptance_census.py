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
The acceptance census: the test that would have caught ``fix/true-clearance``.

That branch made the fence LOOSER and passed every test in this package,
because every test asked "does the checker refuse the poses I wrote down" and
none asked "what does the checker accept, and is the metal really there".  This
file asks the second question, over twenty thousand uniform draws, against an
oracle that shares no kinematics and no distance code with the checker.

IT CANNOT PASS BY ACCIDENT.  Four hard criteria say nothing unsafe is accepted.
Three anti-vacuity criteria say the fence is still a fence: a model that refuses
everything fails (e) and (e2), a model that only ever accepts wide-open poses
fails (h), and a census whose oracle never ran fails (g).

IT CANNOT BE SKIPPED SILENTLY.  No marker, no ``skipif``, no environment
opt-out; it runs in the default ``colcon test`` set.  Its seven constants are
asserted exactly by a second test, so weakening the census means editing two
places and explaining both.

IT IS NOW A HARD GATE.  It was installed BEFORE the fence changed - the way a
thermometer is installed before the fever - and it ran as a strict xfail
against the padded-capsule fence's measured baseline while the mesh work was
built.  That baseline is kept below, because the DIFFERENCE between the two
runs is the answer to what the switch did:

    on main's padded capsules   646 accepted (3.23 %), and 155 of them with
                                true metal inside the margin, worst 4.3900 mm
    on the mesh bodies          3 455 accepted (17.27 %), and NOT ONE of them
                                with metal inside its margin

The fence became five times more permissive and stopped accepting a single
configuration whose metal was inside the margin it claims to enforce.  Both
halves of that sentence are the point: a fence that only got stricter would
have cost the operator workspace for nothing, and a fence that only got looser
would be fix/true-clearance again.
"""

import math

from census_core import (ADJACENT_BY_CONSTRUCTION, ENVELOPE_EXEMPT, format_report,
                         in_force_self_margins, joint_limits, link_pairs,
                         non_adjacent_pairs, run_census)

from conftest import CELL_MODEL_PATH, LINK_GEOMETRY_PATH, MESH_BODIES_PATH

from mesh_oracle import MeshOracle

import pytest


#: The seed the pre-switch measurement used, so that census and this one
#: draw the same twenty thousand configurations and are comparable.
CENSUS_SEED = 20260903
#: Twenty thousand.  The draw count lives here, in the file, so that reducing it
#: is a reviewable diff rather than a quiet edit to a runtime flag.
CENSUS_DRAWS = 20000
#: Measured on this host: 549.9 s for the full census against the mesh fence
#: (476.0 s against the capsule one).  The budget is set at 900 s, which fails
#: a 1.6x slowdown loudly instead of quietly making CI slow, and sits inside
#: the 1800 s pytest timeout with room for the rest of the suite.
CENSUS_BUDGET_S = 900.0
#: WHOLE-FENCE accepted fraction: the model's own verdict, with self, the
#: pedestal, cross-arm, containment, environment and keep-out all passing.  This
#: is the ONLY population the census computes and it is the population this
#: constant is measured on.  It is NOT the self-and-cross figure below.
CENSUS_MIN_ACCEPT_FRACTION = 0.15
#: SELF-AND-CROSS accepted fraction, recomputed from the oracle's own per-pair
#: table with containment, the pedestal, environment and keep-out EXCLUDED.  A
#: second, separately named statistic so that one word does not do two jobs:
#: (e) guards "the fence still accepts real poses" and (e2) guards "the MARGINS
#: have not quietly closed", and a containment regression cannot mask a margin
#: regression or the reverse.
CENSUS_MIN_SELF_CROSS_FRACTION = 0.60
#: The oracle is asked for 153 link pairs per draw - all 36 intra-arm pairs on
#: each arm plus 81 cross-arm - before any verdict is consulted, so this floor
#: is independent of the margin rule and fails a short-circuited oracle by
#: orders of magnitude.
CENSUS_MIN_ORACLE_CALLS = 2_000_000
#: The tightest metal on any ACCEPTED configuration must get at least this
#: close, or the fence is only ever being asked about wide-open poses.
#:
#: MEANINGFUL ONLY WHILE A MARGIN BELOW IT IS IN FORCE.  With a flat self margin
#: no accepted configuration can be below that margin BY DEFINITION, so this
#: ceiling has to be derived from the smallest margin actually in force, and the
#: test below refuses a census whose smallest in-force self margin is above it -
#: naming both numbers, so the criterion cannot silently become unsatisfiable.
CENSUS_MAX_ACCEPTED_FLOOR = 0.012

#: The padded-capsule fence over the same draws, measured BEFORE any mesh
#: geometry reached the check path.  Kept, not deleted: it is the "before" half
#: of what the switch did, and a number nobody can compare against is a number
#: nobody checks.
CAPSULE_BASELINE = {
    'accepted': 646,
    'accepted_fraction': 0.0323,
    'accepted_below_margin': 155,
    'worst_accepted_hazard_m': 0.0043900,
    'self_cross_fraction': 0.6116,
}
#: The mesh fence over the same draws, in the shipped configuration: the ruled
#: 10 mm on the two wrist pairs, 20 mm everywhere else, 50 mm cross-arm, and
#: the link2/link6 delta.
MESH_BASELINE = {
    'accepted': 3455,
    'accepted_fraction': 0.172750,
    'self_cross_fraction': 0.712500,
    'tightest_accepted_enabled_m': 0.0100954,
}


@pytest.fixture(scope='module')
def oracle(cell_model):
    """Build the independent oracle once for the whole census."""
    return MeshOracle(LINK_GEOMETRY_PATH, MESH_BODIES_PATH, cell_model.arm_ids())


@pytest.fixture(scope='module')
def census(cell_model, oracle):
    """Run the census once; every criterion below reads this one report."""
    report = run_census(cell_model, oracle, CENSUS_DRAWS, CENSUS_SEED,
                        progress=2000)
    print('\n' + format_report(report))
    return report


def test_census_constants_are_the_pinned_ones():
    """
    All seven, exactly.

    Weakening the census means editing this list and the constant it names, and
    explaining both in one diff.  M-11 - dropping the acceptance floor to zero
    so that a fence which refuses everything passes (a) to (d) - dies here.
    """
    assert CENSUS_SEED == 20260903
    assert CENSUS_DRAWS == 20000
    assert CENSUS_BUDGET_S == 900.0
    assert CENSUS_MIN_ACCEPT_FRACTION == 0.15
    assert CENSUS_MIN_SELF_CROSS_FRACTION == 0.60
    assert CENSUS_MIN_ORACLE_CALLS == 2_000_000
    assert CENSUS_MAX_ACCEPTED_FLOOR == 0.012


def test_the_accepted_floor_is_meaningful_for_the_margins_in_force(cell_model):
    """
    Criterion (h) is only a guard while a margin below its ceiling is in force.

    Under a flat 20 mm self margin no accepted configuration can be inside
    20 mm, so a 12 mm ceiling on the tightest accepted metal would be logically
    unsatisfiable - not merely unmet.  This test says so with both numbers, so
    that emptying ``pair_margins`` produces a sentence rather than a mystery.
    """
    margins = in_force_self_margins(cell_model)
    smallest = min(margins.values())
    if smallest > CENSUS_MAX_ACCEPTED_FLOOR:
        pytest.xfail(
            'criterion (h) is not meaningful here: the smallest self margin in '
            'force is {:.4f} m and CENSUS_MAX_ACCEPTED_FLOOR is {:.4f} m, so no '
            'accepted configuration can be below the ceiling by definition. '
            'Derive the ceiling from the smallest margin actually in force, or '
            'rule a per-pair margin below it.'.format(
                smallest, CENSUS_MAX_ACCEPTED_FLOOR))
    assert smallest < CENSUS_MAX_ACCEPTED_FLOOR


def test_the_census_draws_over_the_artefacts_own_joint_box(oracle):
    """
    Never a restated constant, and never ``mj_dual.xml``.

    That file declares ``j6`` in [0.5445, 4.5169] - the FR3's range on a Panda
    chain.  The URDF, the joint-limit policy and the loaded model all agree on
    [-0.0175, 3.7525].  A census drawn over the wrong box would measure a robot
    that is not in this lab, and would do it silently.
    """
    lower, upper = joint_limits(oracle)
    assert abs(lower[5] - (-0.0175)) < 1e-12
    assert abs(upper[5] - 3.7525) < 1e-12
    assert not (0.5445 <= lower[5] <= 4.5169)


def test_the_census_covers_every_link_pair_not_only_the_enabled_ones(cell_model):
    """
    M-12: a census restricted to enabled pairs cannot see an ACM hole.

    One exists in this description - ``link2``/``link6`` - and it was found by
    asking about a pair the matrix said would never touch.
    """
    assert len(link_pairs()) == 36
    assert len(non_adjacent_pairs()) == 28
    assert len(ADJACENT_BY_CONSTRUCTION) == 8
    enabled = {pair for _, pair in in_force_self_margins(cell_model)}
    assert len(enabled) == 16
    assert ('link2', 'link6') in enabled
    # ...and the exemptions are written with what exempts them.
    assert set(ADJACENT_BY_CONSTRUCTION) & set(ENVELOPE_EXEMPT) == set()
    assert ('link6', 'link8') in ENVELOPE_EXEMPT
    assert 'caliper' in ENVELOPE_EXEMPT[('link6', 'link8')]


def test_the_census_stays_inside_its_own_wall_clock_budget(census):
    """A 1.8x slowdown fails loudly instead of quietly making CI slow."""
    assert census['seconds'] < CENSUS_BUDGET_S, (
        '{:.1f} s against a {:.1f} s budget'.format(census['seconds'],
                                                    CENSUS_BUDGET_S))


def test_the_oracle_was_actually_asked(census):
    """Criterion (g): a census whose oracle never ran proves nothing."""
    assert census['oracle_calls'] >= CENSUS_MIN_ORACLE_CALLS
    assert census['oracle_calls'] == 153 * CENSUS_DRAWS


def test_the_mesh_fence_baseline_is_the_measured_one(census):
    """
    The acceptance table, pinned, so that a change to it is a change to review.

    These four numbers are what the fence does to the joint box.  A future
    change that moves any of them - a margin, a body, a broad-phase bound -
    fails here with both values, and somebody has to say which is right.
    """
    assert census['accepted'] == MESH_BASELINE['accepted']
    assert abs(census['accepted_fraction']
               - MESH_BASELINE['accepted_fraction']) < 5e-5
    assert abs(census['self_cross_fraction']
               - MESH_BASELINE['self_cross_fraction']) < 5e-5
    assert abs(census['tightest_accepted_enabled']
               - MESH_BASELINE['tightest_accepted_enabled_m']) < 5e-6


def test_the_switch_made_the_fence_permissive_AND_honest(census):
    """
    Both halves of what the switch did, in one place, as numbers.

    The capsule fence accepted 646 of 20 000 draws and 155 of those had true
    metal inside the margin it claims to enforce - worst 4.3900 mm - because
    the clearance it measured was 30 mm of inflation on each radius rather than
    air.  The mesh fence accepts 3 455 and NOT ONE of them is inside its
    margin.

    A fence that had only become stricter would have cost the operator
    workspace for nothing.  A fence that had only become looser would be
    fix/true-clearance again - the branch that made this fence weaker and
    passed every test in the package.  This test is the sentence that both
    happened.
    """
    assert census['accepted'] > 5 * CAPSULE_BASELINE['accepted']
    assert CAPSULE_BASELINE['accepted_below_margin'] == 155
    assert census['accepted_below_margin'] == 0


def test_the_disabled_pairs_are_reported(census):
    """
    A disabled, non-adjacent pair that touches is a FINDING, reported by name.

    It becomes a failure only under criterion (d), which covers disabled pairs
    too - the report is what makes a hole visible before it becomes a verdict.
    """
    report = census['disabled_report']
    assert report, 'the census reported nothing about the disabled pairs'
    for pair, record in report.items():
        assert pair not in ADJACENT_BY_CONSTRUCTION
        assert record['minimum'] < math.inf
    # link2/link6 is no longer here: the cell file's delta enabled it.
    assert ('link2', 'link6') not in report


def test_acceptance_census(census):
    """
    Criteria (a)-(h) and (e2), the whole gate, in one place.

    (a) no accepted configuration has an ENABLED intra-arm pair below that
        pair's own margin;
    (b) none has a cross-arm pair below the cross-arm margin;
    (c) none has an enabled pair at metal <= 0;
    (d) none has ANY non-adjacent pair at metal <= 0, enabled or not;
    (e) the whole fence still accepts a real fraction of the joint box;
    (e2) and so do the margins alone, measured separately;
    (g) the oracle was asked;
    (h) and the fence is being exercised near its margins, not only in the open.
    """
    failures = []
    if census['accepted_below_margin']:
        failures.append(
            '(a) {} accepted pairs below their own margin, worst {:.4f} mm'.format(
                census['accepted_below_margin'],
                census['worst_accepted_below_margin'] * 1000.0))
    if census['accepted_cross_below_margin']:
        failures.append('(b) {} accepted cross-arm pairs below the margin'.format(
            census['accepted_cross_below_margin']))
    if census['accepted_metal_at_or_below_zero']:
        failures.append('(c) {} accepted enabled pairs at metal <= 0'.format(
            census['accepted_metal_at_or_below_zero']))
    if census['accepted_metal_unresolved_at_zero']:
        failures.append(
            '(c) {} accepted enabled pairs within 0.1 mm, too close to call '
            'apart from contact'.format(
                census['accepted_metal_unresolved_at_zero']))
    if census['accepted_non_adjacent_contact']:
        failures.append('(d) {} accepted non-adjacent pairs at metal <= 0'.format(
            census['accepted_non_adjacent_contact']))
    if census['accepted_non_adjacent_unresolved']:
        failures.append('(d) {} accepted non-adjacent pairs within 0.1 mm'.format(
            census['accepted_non_adjacent_unresolved']))
    if census['accepted_fraction'] < CENSUS_MIN_ACCEPT_FRACTION:
        failures.append(
            '(e) whole-fence acceptance {:.2%} below the floor {:.2%}'.format(
                census['accepted_fraction'], CENSUS_MIN_ACCEPT_FRACTION))
    if census['self_cross_fraction'] < CENSUS_MIN_SELF_CROSS_FRACTION:
        failures.append(
            '(e2) self-and-cross acceptance {:.2%} below the floor {:.2%}'.format(
                census['self_cross_fraction'], CENSUS_MIN_SELF_CROSS_FRACTION))
    if census['oracle_calls'] < CENSUS_MIN_ORACLE_CALLS:
        failures.append('(g) the oracle was asked only {} times'.format(
            census['oracle_calls']))
    if census['tightest_accepted_enabled'] > CENSUS_MAX_ACCEPTED_FLOOR:
        failures.append(
            '(h) the tightest accepted metal is {:.4f} mm, above the {:.4f} mm '
            'ceiling: the fence is only being asked about open poses'.format(
                census['tightest_accepted_enabled'] * 1000.0,
                CENSUS_MAX_ACCEPTED_FLOOR * 1000.0))
    assert not failures, '\n'.join(failures)


def test_the_census_file_carries_no_skip_switch():
    """
    No skip marker and no environment opt-out.  R5, as a test.

    A census that can be turned off is a census that will be, on the day it is
    inconvenient - and that day is exactly the day it is telling the truth.

    The check reads the file's SYNTAX and not its prose: a substring scan would
    fail on this docstring for naming the very thing it forbids, and a test that
    cannot describe its own rule is a test somebody deletes.
    """
    import ast

    with open(__file__, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    forbidden_decorators = set()
    forbidden_calls = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for decorator in node.decorator_list:
                name = ast.unparse(decorator)
                if 'skip' in name or 'slow' in name:
                    forbidden_decorators.add(name)
        if isinstance(node, ast.Call):
            name = ast.unparse(node.func)
            if name.endswith(('getenv', 'environ.get', 'skip')):
                forbidden_calls.add(name)
        if isinstance(node, ast.Attribute) and ast.unparse(node) == 'os.environ':
            forbidden_calls.add('os.environ')
    assert not forbidden_decorators, forbidden_decorators
    assert not forbidden_calls, forbidden_calls
    # The one marker that IS here is a strict xfail, which runs the census on
    # every build and FAILS if it unexpectedly passes.  It is the opposite of a
    # skip, and it is removed at the switch.
    assert CELL_MODEL_PATH.is_file()
