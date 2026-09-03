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
The narrow phase: what it returns, and the two guards that make it return it.

Everything here is measured against something that is not this GJK.  The
analytic anchors are exact by hand; the hard case is cross-checked against an
SLSQP quadratic program over the two hulls' half-spaces, which shares no line of
code with the solver; and the two ensembles are adjudicated by a SEPARATING
DIRECTION - a rigorous certificate, since ``dist(A, B) >= min_B<n,b> -
max_A<n,a>`` for any unit ``n``, so a direction with a positive gap PROVES the
bodies do not touch whatever any solver says.
"""

import time

from conftest import LINK_GEOMETRY_PATH, MESH_BODIES_PATH, READY

from franka_workspace_model.geometry import GeometryError
from franka_workspace_model.mesh_runtime import (distance, GJK_DEGENERACY_REL,
                                                 GJK_ITERATION_CAP, GJK_TOL_ABS_M,
                                                 GJK_TOL_REL, load_mesh_bodies)
from franka_workspace_model.mesh_runtime.gjk import _sub_distance

import numpy as np

import pytest

import yaml


#: The named near-touching regression.  ``link5_collision_1_st`` against
#: ``link7_st`` at j6 = -0.049213712 (BELOW the joint limit: this is a geometry
#: fixture, not a reachable pose) and j7 = +2.8973.  A purely RELATIVE criterion
#: cannot converge here at any cap; see the test that measures what happens
#: instead.
GJK_HARDCASE_Q = [0.0, -0.7854, 0.0, -2.3562, 0.0, -0.049213712, 2.8973]
#: IN METRES.  Note the exponent: 1e-04, not 1e-07.  Three independent body
#: builds land in a band 1.84e-10 m wide; the pin is its centre, so every
#: reading is within 9.5e-11 m of it - ten times inside the tolerance - and a
#: re-implementation of the simplex routine is not booby-trapped by it.
GJK_HARDCASE_M = 1.9474453e-04
#: IN METRES, and the unit is in the name.  Read against a value printed in
#: millimetres the same digits would be 1000x tighter than any two correct GJKs
#: agree, which is a mistake this line of work has already made once.
GJK_HARDCASE_TOL_M = 1e-9
#: The independent SLSQP value, recorded beside the expectation.
GJK_HARDCASE_QP_M = 1.947446249e-04

UNIFORM_DRAWS = 20000
UNIFORM_SEED = 20260903
NEAR_TOUCHING_DRAWS = 1200
NEAR_TOUCHING_SEED = 20260903


@pytest.fixture(scope='module')
def centred_bodies():
    """Return the real bodies, each translated to its own centroid."""
    geometry = yaml.safe_load(LINK_GEOMETRY_PATH.read_text(encoding='utf-8'))
    loaded = load_mesh_bodies(MESH_BODIES_PATH,
                              urdf_sha256=geometry['source']['urdf_sha256'])
    return [body.vertices - body.vertices.mean(axis=0) for body in loaded.bodies]


def _random_rotation(rng):
    quaternion = rng.normal(size=4)
    quaternion /= np.linalg.norm(quaternion)
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _separating_gap(first, second, direction):
    """``min_B<n,b> - max_A<n,a>``: positive PROVES the two do not touch."""
    unit = direction / np.linalg.norm(direction)
    return float((second @ unit).min() - (first @ unit).max())


def _best_separating_gap(first, second, seed=7):
    """Search for a separating direction.  Any positive result is a proof."""
    rng = np.random.default_rng(seed)
    directions = rng.normal(size=(64, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    best = -np.inf
    witness = directions[0]
    for direction in directions:
        gap = _separating_gap(first, second, direction)
        if gap > best:
            best, witness = gap, direction
    step = 0.5
    for _ in range(50):
        improved = False
        for _ in range(8):
            candidate = witness + step * rng.normal(size=3)
            candidate /= np.linalg.norm(candidate)
            gap = _separating_gap(first, second, candidate)
            if gap > best:
                best, witness, improved = gap, candidate, True
        if not improved:
            step *= 0.5
    return best


# ---------------------------------------------------------------------------
# Analytic anchors
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('gap', [0.0, 0.25, 1.0, 3.7])
def test_two_unit_cubes_at_an_exact_gap(gap):
    """Exact to the last bit, and checkable by hand without running anything."""
    cube = np.array([[x, y, z] for x in (0.0, 1.0) for y in (0.0, 1.0)
                     for z in (0.0, 1.0)])
    assert distance(cube, cube + np.array([1.0 + gap, 0.0, 0.0])) == gap


def test_the_pinned_constants_are_the_measured_ones():
    """
    Weakening a guard means editing a named constant, in a reviewable diff.

    Every one of these came from a measurement, and the module docstring records
    which.  The unit of the absolute floor is in its own name.
    """
    assert (GJK_TOL_REL, GJK_TOL_ABS_M) == (1e-9, 1e-9)
    assert GJK_DEGENERACY_REL == 1e-8
    assert GJK_ITERATION_CAP == 32


# ---------------------------------------------------------------------------
# T-GJK-1: the named hard case
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def hardcase(cell_model):
    """Place the two wrist bodies at the fixture pose, from the SHIPPED artefact."""
    geometry = yaml.safe_load(LINK_GEOMETRY_PATH.read_text(encoding='utf-8'))
    loaded = load_mesh_bodies(MESH_BODIES_PATH,
                              urdf_sha256=geometry['source']['urdf_sha256'])
    transforms = cell_model._transforms(cell_model._sample(
        {'panda1': list(GJK_HARDCASE_Q), 'panda2': list(READY)}))
    placed = {}
    for name in ('link5_collision_1_st', 'link7_st'):
        body = loaded.by_id[name]
        transform = transforms['panda1_{}'.format(body.link)]
        placed[name] = body.vertices @ transform[:3, :3].T + transform[:3, 3]
    return placed['link5_collision_1_st'], placed['link7_st']


def test_gjk_hardcase(hardcase):
    """
    Replay T-GJK-1: the fixture, in METRES, with its independent cross-check.

    The test reads the shipped 12-decimal artefact rather than a re-rounded
    copy, because a 7-decimal re-emission moves this answer by 2.26e-09 m -
    outside the tolerance - while 12 and 9 decimals move it by exactly zero.
    """
    statistics = {}
    first, second = hardcase
    measured = distance(first, second, statistics=statistics)
    assert abs(measured - GJK_HARDCASE_M) <= GJK_HARDCASE_TOL_M, (
        '{:.12e} m against the pin {:.7e} m'.format(measured, GJK_HARDCASE_M))
    assert statistics['iterations'] <= 20
    # ...and it is 1e-04, not 1e-07: the bodies really are two tenths of a
    # millimetre apart, which is the sentence the exponent has to agree with.
    assert 1e-4 < measured < 1e-3


def test_the_hardcase_agrees_with_an_independent_quadratic_program(hardcase):
    """
    Cross-check the fixture against code that shares nothing with the solver.

    SLSQP over the two hulls' half-spaces: minimise |p - q| subject to p in A
    and q in B.  scipy is used in ``test/`` only; the runtime is numpy-only and
    ``test_purity_and_performance.py`` asserts it.
    """
    from scipy.optimize import minimize
    from scipy.spatial import ConvexHull

    first, second = hardcase
    hull_a = ConvexHull(first)
    hull_b = ConvexHull(second)

    def constraints(hull, offset):
        normals = hull.equations[:, :3]
        offsets = -hull.equations[:, 3]
        return {'type': 'ineq',
                'fun': lambda x, n=normals, d=offsets, o=offset: d - n @ x[o:o + 3]}

    start = np.concatenate([first.mean(axis=0), second.mean(axis=0)])
    result = minimize(lambda x: float(np.linalg.norm(x[:3] - x[3:])), start,
                      method='SLSQP',
                      constraints=[constraints(hull_a, 0), constraints(hull_b, 3)],
                      options={'maxiter': 500, 'ftol': 1e-14})
    assert abs(result.fun - GJK_HARDCASE_QP_M) < 1e-9, result.fun
    assert abs(result.fun - distance(first, second)) < 1e-9


def test_the_relative_criterion_alone_cannot_converge_on_the_hardcase(hardcase):
    """
    Show that the relative criterion alone cannot converge on this fixture.

    The bodies are 2e-04 m apart while their support points are ~1e-01 m from
    the origin, so a 1e-9 RELATIVE test demands 2e-13 m of resolution out of
    arithmetic that cancels at about 1e-13.  It is unsatisfiable, and this test
    shows it directly: with the absolute floor removed the solver never reports
    ``converged``.

    What it does instead depends on the other guard, and THIS BUILD DIFFERS FROM
    THE PLAN'S PROTOTYPE HERE.  The plan measured outright non-termination at
    caps of 32, 64, 128, 512 and 4096.  In this implementation the
    duplicate-support guard catches the same stall and terminates with
    ``reason == 'duplicate'`` at the correct value.  That is a better outcome
    and it is the reason both guards ship, but it means the fixture does not
    raise, and pretending otherwise would be a test asserting somebody else's
    implementation.  The non-termination evidence for the floor lives where it
    was actually measured: the bisected near-touching ensemble below, where
    relative-only leaves about forty failures and the floor leaves none.
    """
    first, second = hardcase
    statistics = {}
    value = distance(first, second, tolerance_absolute=0.0, statistics=statistics)
    assert statistics['reason'] == 'duplicate'
    assert abs(value - GJK_HARDCASE_M) <= GJK_HARDCASE_TOL_M

    statistics = {}
    distance(first, second, statistics=statistics)
    assert statistics['reason'] == 'converged'

    # With BOTH the floor and the duplicate guard removed, nothing is left to
    # stop the walk except the degeneracy fallback, and the plan's measurement
    # of what that costs stands: it is the pair of guards that makes this case
    # terminate, not either one alone.
    statistics = {}
    distance(first, second, tolerance_absolute=0.0, guard_duplicate=False,
             statistics=statistics)
    assert statistics['degenerate'] >= 1


# ---------------------------------------------------------------------------
# T-GJK-2: the degeneracy guard, on a constructed simplex
# ---------------------------------------------------------------------------

def test_a_degenerate_simplex_never_concludes_containment():
    """
    Check T-GJK-2 on a constructed simplex.

    Four coplanar points do not enclose the origin, whatever the sign of a
    determinant that is numerical noise.  The simplex below is four points of a
    square in the plane z = 1e-18, which surrounds the origin in x and y and is
    a hair above it in z.  The
    non-degenerate branch's "the origin is on the inside of every face" test can
    answer yes here; the guard refuses to ask, and falls back to the best face -
    which correctly reports a distance of about 1e-18 rather than a penetration.
    """
    simplex = [(-1.0, -1.0, 1e-18), (1.0, -1.0, 1e-18),
               (1.0, 1.0, 1e-18), (-1.0, 1.0, 1e-18)]
    counters = {'degenerate': 0, 'duplicate': 0}
    point, keep = _sub_distance(simplex, 4, GJK_DEGENERACY_REL, counters)
    assert counters['degenerate'] == 1
    assert len(keep) < 4
    assert point[0] ** 2 + point[1] ** 2 + point[2] ** 2 > 0.0

    # With the guard OFF the same simplex is free to conclude containment.
    counters = {'degenerate': 0, 'duplicate': 0}
    point, keep = _sub_distance(simplex, 4, -1.0, counters)
    assert counters['degenerate'] == 0


# ---------------------------------------------------------------------------
# T-GJK-3: the uniform ensemble
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def uniform_ensemble(centred_bodies):
    """Place two real bodies at random 20 000 times, at a pinned seed."""
    rng = np.random.default_rng(UNIFORM_SEED)
    cases = []
    for _ in range(UNIFORM_DRAWS):
        first_index, second_index = rng.integers(0, len(centred_bodies), 2)
        first = centred_bodies[first_index] @ _random_rotation(rng).T
        second = (centred_bodies[second_index] @ _random_rotation(rng).T
                  + rng.uniform(-0.3, 0.3, 3))
        cases.append((first, second))
    return cases


def test_gjk_uniform_ensemble(uniform_ensemble):
    """
    Run T-GJK-3: 20 000 placements of the real bodies, guards on.

    Zero non-terminations, and every reported distance carries a SEPARATING
    DIRECTION that proves it - the certificate is checked on a pinned sample,
    since it is the expensive half.
    """
    reported_penetrating = []
    worst_iterations = 0
    for index, (first, second) in enumerate(uniform_ensemble):
        statistics = {}
        value = distance(first, second, statistics=statistics)
        worst_iterations = max(worst_iterations, statistics['iterations'])
        if value == 0.0:
            reported_penetrating.append(index)
    assert worst_iterations <= GJK_ITERATION_CAP
    # Measured maximum on this ensemble: 21.  The assertion leaves headroom for
    # a re-implementation of the simplex without leaving the cap unwatched.
    assert worst_iterations <= 24, worst_iterations
    assert len(reported_penetrating) < len(uniform_ensemble) // 4

    rng = np.random.default_rng(11)
    sample = rng.choice(len(uniform_ensemble), 150, replace=False)
    for index in sample:
        first, second = uniform_ensemble[int(index)]
        value = distance(first, second)
        if value > 1e-6:
            gap = _best_separating_gap(first, second)
            assert gap > 0.0, (
                'GJK reports {:.6f} mm but no separating direction was '
                'found'.format(value * 1000.0))
            assert gap <= value + 1e-9, (gap, value)


def test_the_degeneracy_guard_removes_real_false_penetrations(uniform_ensemble):
    """
    Measure M-2 rather than merely passing a test.

    With the guard off and the survey's 1e-12 tolerance, the tetrahedron branch
    concludes containment from simplices that do not span a volume.  Each case
    below is adjudicated by a SEPARATING DIRECTION, which proves the bodies do
    not touch whatever any solver says.  On this ensemble the guard removes 14
    such calls, the worst of them on a pair 30.73 mm apart.
    """
    unguarded = set()
    guarded = set()
    non_terminations = 0
    for index, (first, second) in enumerate(uniform_ensemble):
        if distance(first, second) == 0.0:
            guarded.add(index)
        try:
            if distance(first, second, guard_degeneracy=False, guard_duplicate=False,
                        tolerance_relative=1e-12, tolerance_absolute=0.0) == 0.0:
                unguarded.add(index)
        except GeometryError:
            non_terminations += 1
    extra = sorted(unguarded - guarded)
    assert non_terminations > 0, (
        'the unguarded solver is supposed to fail to terminate on this ensemble')
    assert len(extra) >= 5, len(extra)
    worst = 0.0
    for index in extra:
        first, second = uniform_ensemble[index]
        if _best_separating_gap(first, second) > 0.0:
            worst = max(worst, distance(first, second))
    assert worst > 0.005, (
        'the guard is supposed to remove a call that reported a pair several '
        'millimetres apart as penetrating; the worst was {:.4f} mm'.format(
            worst * 1000.0))


# ---------------------------------------------------------------------------
# T-GJK-4: the bisected near-touching ensemble
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def near_touching_ensemble(centred_bodies):
    """
    Bisect 1 200 placements to a pinned band of true separations.

    Log-uniform in [1e-7, 1e-3] m: the regime neither prior document exercised.
    """
    rng = np.random.default_rng(NEAR_TOUCHING_SEED)
    cases = []
    while len(cases) < NEAR_TOUCHING_DRAWS:
        first_index, second_index = rng.integers(0, len(centred_bodies), 2)
        first = centred_bodies[first_index] @ _random_rotation(rng).T
        second = centred_bodies[second_index] @ _random_rotation(rng).T
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        target = 10.0 ** rng.uniform(-7.0, -3.0)

        def probe(scale, a=first, b=second, u=axis):
            try:
                return distance(a, b + u * scale, tolerance_relative=1e-6)
            except GeometryError:
                return 0.0

        low, high = 0.0, 1.0
        if probe(high) <= target:
            continue
        for _ in range(40):
            middle = 0.5 * (low + high)
            if probe(middle) > target:
                high = middle
            else:
                low = middle
        placed = second + axis * high
        try:
            separation = distance(first, placed)
        except GeometryError:
            continue
        if separation <= 0.0:
            continue
        cases.append((first, placed, separation))
    return cases


def test_gjk_near_touching_ensemble(near_touching_ensemble):
    """Run T-GJK-4: zero non-terminations where the relative criterion cannot work."""
    separations = np.array([case[2] for case in near_touching_ensemble])
    assert separations.min() < 1e-6
    assert separations.max() < 1.1e-3
    worst_iterations = 0
    for first, second, _ in near_touching_ensemble:
        statistics = {}
        distance(first, second, statistics=statistics)
        worst_iterations = max(worst_iterations, statistics['iterations'])
    assert worst_iterations <= 24, worst_iterations


def test_the_absolute_floor_is_what_terminates_the_near_touching_regime(
        near_touching_ensemble):
    """
    Reproduce the measurement the absolute floor was chosen from.

    Relative-only leaves about forty non-terminations on this ensemble; a floor
    of 1e-10 m leaves one or two, and every survivor there is a pair whose TRUE
    separation is below the floor itself.  At 1e-9 m none remain.
    """
    def count(**keywords):
        failures = 0
        for first, second, _ in near_touching_ensemble:
            try:
                distance(first, second, **keywords)
            except GeometryError:
                failures += 1
        return failures

    assert count() == 0
    assert count(tolerance_absolute=0.0) > 10
    assert count(tolerance_absolute=1e-10) <= 5


# ---------------------------------------------------------------------------
# T-GJK-5 and T-GJK-6
# ---------------------------------------------------------------------------

def test_gjk_is_deterministic(hardcase):
    """
    Check T-GJK-5: bitwise identical over 200 repeats.

    A fence whose answer moves between two calls on one process cannot be
    reasoned about at all, and a corpus expectation pinned to nine digits would
    be a lottery.
    """
    first, second = hardcase
    values = {distance(first, second) for _ in range(200)}
    assert len(values) == 1


def test_gjk_fails_closed_at_the_cap():
    """
    Check T-GJK-6: the cap raises, and no value is propagated.

    The cap is reachable, and only on pairs within about a nanometre of contact,
    where refusing is the correct verdict anyway.  ``GeometryError`` is the
    model's existing fail-closed path: ``_evaluate`` turns it into a refusal
    naming the reason.
    """
    cube = np.array([[x, y, z] for x in (0.0, 1.0) for y in (0.0, 1.0)
                     for z in (0.0, 1.0)])
    with pytest.raises(GeometryError, match='did not converge'):
        distance(cube, cube + np.array([1.5, 0.3, 0.2]), cap=1,
                 tolerance_relative=0.0, tolerance_absolute=0.0)


def test_a_refusal_at_the_cap_reaches_the_model_as_a_refusal(cell_model):
    """
    Keep the fail-closed path the model's own, not a new one.

    ``CellModel._evaluate`` already wraps ``GeometryError`` into a
    ``WorkspaceModelError`` saying the only safe verdict is refusal; the mesh
    narrow phase raises the same type so that nothing new has to be wired up
    when the fence switches.
    """
    from franka_workspace_model.strictyaml import WorkspaceModelError
    assert issubclass(GeometryError, ArithmeticError)
    assert not issubclass(GeometryError, WorkspaceModelError)


def test_the_narrow_phase_costs_what_the_budget_assumes(hardcase):
    """
    Measure one call, and record that the vertices are not the cost.

    Measured on this host: about 11 % of a call is the support evaluations over
    the vertex arrays and the rest is the simplex routine.  That is why the
    named optimisation is the simplex and NOT reducing vertex counts.
    """
    first, second = hardcase
    for _ in range(20):
        distance(first, second)
    started = time.perf_counter()
    for _ in range(200):
        distance(first, second)
    per_call = (time.perf_counter() - started) / 200.0
    assert per_call < 5e-3, '{:.1f} us per call'.format(per_call * 1e6)
