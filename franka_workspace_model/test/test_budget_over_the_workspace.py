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
The budgets, measured over the WORKSPACE rather than at one pose.

``test_purity_and_performance.py`` measures both budgets at ``BOTH_READY``, a
module constant, repeated a thousand times.  That is a real measurement of a
real pose and it stays - but it is the CHEAPEST pose the arm has.  At ready the
mesh fence makes two GJK calls; over the joint box it makes six or seven, and
the tail is worse than that.

Left alone, the shipped budget tests would therefore have gone green while the
fence was two-fold outside its jog budget across the workspace.  That is not a
guard against "the budgets are met by loosening them" - which the constants
being pinned already covers - but against "the budgets are met because the test
only looks at the easy case", and it cuts in the unsafe direction: a console
that is responsive at home and stutters when the arm is folded is a console
that has to be trusted least exactly where it is used most.

So this file is the entry condition for the switch.  A pinned set of uniform
configurations, a pinned seed, drawn over the ARTEFACT's own joint limits, with
the median for ``check_configuration`` and the p99 for ``check_jog`` - the two
statistics the design already defines - against the two budgets it already
names.  Nothing here is a new budget and nothing here is a widened one.

MEASURED ON THIS HOST, at the numbers below:

    check_configuration median        1.043 ms   (budget 2.0)
    check_configuration p99           2.627 ms
    check_jog at 2 degrees, p99      ~7.9 ms     (budget 10.0)

The first working version of the mesh fence measured 3.26 ms at the ready pose
alone.  Three changes closed the gap, and the one that mattered was the third:
the simplex routine rewritten in scalar floats.  A GJK call on this host is
63.8 us, of which about half is the support scans over the pinned vertex
arrays, and those are irreducible.  REDUCING VERTEX COUNTS IS NOT THE
OPTIMISATION - it is about a tenth of the cost and every sound reduction
available costs volume.
"""

import time

from census_core import joint_limits

from conftest import READY

from mesh_oracle import MeshOracle

import numpy as np

import pytest


#: The two budgets the design defines.  Restated here rather than imported so
#: that widening one means editing two files.
CONFIGURATION_BUDGET_MS = 2.0
JOG_BUDGET_P99_MS = 10.0
TWO_DEGREES = 0.0349

BUDGET_DRAWS = 400
BUDGET_SEED = 20260903
#: Untimed draws first, exactly as the ready-pose tests do.
WARM_UP = 20


@pytest.fixture(scope='module')
def workspace_draws(cell_model):
    """
    Draw a pinned uniform sample of the joint box: the workload, not home.

    Drawn over the ARTEFACT's own ``parent_joint.limit_lower/upper``, never
    ``mj_dual.xml``: that file declares ``j6`` in [0.5445, 4.5169], which are
    the FR3's ranges on a Panda chain, and a budget measured over the wrong box
    would be a budget for a robot that is not in this lab.
    """
    oracle = MeshOracle(cell_model._geometry.path, cell_model._mesh_bodies.path,
                        cell_model.arm_ids())
    lower, upper = joint_limits(oracle)
    generator = np.random.default_rng(BUDGET_SEED)
    return [{arm_id: list(generator.uniform(lower, upper))
             for arm_id in cell_model.arm_ids()} for _ in range(BUDGET_DRAWS)]


def _percentile(samples, fraction):
    ordered = sorted(samples)
    return ordered[min(int(fraction * len(ordered)), len(ordered) - 1)]


def test_check_configuration_holds_its_budget_over_the_workspace(cell_model,
                                                                 workspace_draws):
    """The median of ``check_configuration``, over the joint box."""
    for configuration in workspace_draws[:WARM_UP]:
        cell_model.check_configuration(configuration)
    samples = []
    for configuration in workspace_draws:
        started = time.perf_counter()
        cell_model.check_configuration(configuration)
        samples.append((time.perf_counter() - started) * 1000.0)
    median = _percentile(samples, 0.5)
    assert median < CONFIGURATION_BUDGET_MS, (
        'median {:.3f} ms over {} uniform configurations, against a {:.1f} ms '
        'budget; p99 {:.3f} ms, max {:.3f} ms'.format(
            median, BUDGET_DRAWS, CONFIGURATION_BUDGET_MS,
            _percentile(samples, 0.99), max(samples)))


def test_a_two_degree_jog_holds_its_budget_over_the_workspace(cell_model,
                                                              workspace_draws):
    """
    The p99 of ``check_jog``, over the joint box, on the joint the console jogs.

    A jog is three samples at ``policy.max_joint_step_rad``, so it costs three
    checks; measuring it at the ready pose costs three of the cheapest checks
    the arm has.  Some draws are already refused and stop after one sample,
    which is realistic - a console jogging out of a refused pose is the common
    case - so the assertion is on the p99 of the whole distribution rather than
    on a filtered one.
    """
    for configuration in workspace_draws[:WARM_UP]:
        cell_model.check_jog('panda1', configuration, 1, TWO_DEGREES)
    samples = []
    for configuration in workspace_draws:
        started = time.perf_counter()
        cell_model.check_jog('panda1', configuration, 1, TWO_DEGREES)
        samples.append((time.perf_counter() - started) * 1000.0)
    percentile_99 = _percentile(samples, 0.99)
    assert percentile_99 < JOG_BUDGET_P99_MS, (
        'p99 {:.3f} ms over {} uniform configurations, against a {:.1f} ms '
        'budget; median {:.3f} ms, max {:.3f} ms'.format(
            percentile_99, BUDGET_DRAWS, JOG_BUDGET_P99_MS,
            _percentile(samples, 0.5), max(samples)))


def test_the_ready_pose_is_the_cheap_case_and_this_file_says_so(cell_model,
                                                                workspace_draws):
    """
    The measurement that makes this file necessary rather than decorative.

    If the ready pose were representative, the two shipped budget tests would
    be enough and this one would be noise.  It is not: at ready the fence makes
    two GJK calls and over the workspace it makes several times that, so the
    p99 is multiples of the home-pose cost.  A future change that made the home
    pose fast and the workspace slow would show up here and nowhere else.
    """
    both_ready = {arm_id: list(READY) for arm_id in cell_model.arm_ids()}
    for _ in range(WARM_UP):
        cell_model.check_configuration(both_ready)
    ready_samples = []
    for _ in range(200):
        started = time.perf_counter()
        cell_model.check_configuration(both_ready)
        ready_samples.append((time.perf_counter() - started) * 1000.0)
    ready = _percentile(ready_samples, 0.5)

    workspace = []
    for configuration in workspace_draws:
        started = time.perf_counter()
        cell_model.check_configuration(configuration)
        workspace.append((time.perf_counter() - started) * 1000.0)
    assert _percentile(workspace, 0.99) > 1.5 * ready, (
        'the ready pose costs {:.3f} ms and the workspace p99 {:.3f} ms; if '
        'those are the same number this test is measuring nothing'.format(
            ready, _percentile(workspace, 0.99)))


def test_the_budgets_are_the_ones_the_design_names():
    """
    Widening a budget means editing this file AND the shipped one.

    R6: the budgets are met by making the fence faster, never by moving the
    line.  The prototype missed the jog budget by 2.1x and the answer was the
    scalar-float simplex, not a larger number here.
    """
    from test_purity_and_performance import (CONFIGURATION_BUDGET_MS as shipped,
                                             JOG_BUDGET_P99_MS as shipped_jog)
    assert CONFIGURATION_BUDGET_MS == shipped == 2.0
    assert JOG_BUDGET_P99_MS == shipped_jog == 10.0
    assert (BUDGET_DRAWS, BUDGET_SEED) == (400, 20260903)
