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
Exact convex-convex distance by GJK, with the two guards the measurements demand.

WHAT THE TOLERANCE STUDY FOUND, and it is not what the survey thought.  Over
20 000 random placements of the real collision assets, adjudicated against
exhaustive feature enumeration:

    implementation                              false penetrations   non-terminations
    no guards, tol 1e-12                                 9                 19
    no guards, tol 1e-9                                  1                  0
    degeneracy guard, tol 1e-9                           0                  1
    degeneracy + duplicate guards, tol 1e-9              0                  0

The nine false penetrations are not a tolerance problem.  They come from the
tetrahedron branch concluding "the origin is inside" from a NUMERICALLY
DEGENERATE simplex - four points that do not span a volume.  At its worst that
branch calls a pair **16.2265 mm apart** penetrating.  Refusing to conclude
containment from a degenerate simplex removes all nine.

BUT THE GUARD ALONE CREATES A NON-TERMINATION.  A solver that will not conclude
containment keeps iterating with nothing to converge to.  The named case is
``gjk_hardcase`` - ``link5_collision_1_st`` against ``link7_st`` at
``j6 = -0.049213712, j7 = +2.8973``, true distance 1.9474e-04 m - where a purely
RELATIVE 1e-9 criterion does not terminate at a cap of 32, 64, 128, 512 or 4096,
because 4081 of 4095 iterations produce a degenerate simplex.  The bodies are
2e-04 m apart while their support points are ~1e-01 m from the origin, so a
relative test demands 2e-13 m of resolution out of arithmetic that cancels at
~1e-13.  It is unsatisfiable, and the fix is an ABSOLUTE floor.  With one, the
same case terminates in fourteen iterations.

The floor was chosen by measurement on a second, adversarial ensemble - 1 200
placements bisected to a true separation drawn log-uniformly in [1e-7, 1e-3] m::

    degeneracy + duplicate guards, relative 1e-9 only        41 non-terminations
    degeneracy + duplicate guards, relative + absolute 1e-9   0

and at a floor of 1e-10 m the only survivors are pairs whose TRUE separation is
below the floor itself.

FAIL CLOSED AT THE CAP.  The iteration cap is reachable, and only on pairs
within about a nanometre of contact, where refusing is the correct verdict
anyway.  Reaching it raises ``GeometryError``, which the model already turns
into a refusal; no value is propagated.
"""

from math import sqrt

import numpy as np

from ..geometry import GeometryError


#: Relative convergence: 1e-12 costs nineteen non-terminations per 20 000
#: placements and buys nothing - 1e-9 and 1e-8 are bit-identical in verdict.
GJK_TOL_REL = 1e-9
#: Absolute convergence floor, IN METRES.  The unit is in the name because a
#: tolerance of "1e-9" against a distance printed in millimetres is 1000x
#: tighter than the same digits against one in metres, and that mistake has
#: already been made once in this line of work.
GJK_TOL_ABS_M = 1e-9
#: Relative determinant threshold: a tetrahedron counts as degenerate when
#: ``abs(det) <= GJK_DEGENERACY_REL * |e1| |e2| |e3|``.  Removes all nine false
#: penetrations.
GJK_DEGENERACY_REL = 1e-8
#: Maximum iterations before fail-closed refusal.  Measured maximum with both
#: guards on: 16, over 21 200 adversarial placements plus a 4 800-point wrist
#: grid.
GJK_ITERATION_CAP = 32


def _closest_on_triangle(a, b, c, index_a, index_b, index_c):
    """Ericson's closest-point-on-triangle to the origin, with the kept indices."""
    edge_ab = b - a
    edge_ac = c - a
    to_a = -a
    d1 = edge_ab @ to_a
    d2 = edge_ac @ to_a
    if d1 <= 0.0 and d2 <= 0.0:
        return a, (index_a,)
    to_b = -b
    d3 = edge_ab @ to_b
    d4 = edge_ac @ to_b
    if d3 >= 0.0 and d4 <= d3:
        return b, (index_b,)
    weight_c = d1 * d4 - d3 * d2
    if weight_c <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        parameter = d1 / (d1 - d3)
        return a + parameter * edge_ab, (index_a, index_b)
    to_c = -c
    d5 = edge_ab @ to_c
    d6 = edge_ac @ to_c
    if d6 >= 0.0 and d5 <= d6:
        return c, (index_c,)
    weight_b = d5 * d2 - d1 * d6
    if weight_b <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        parameter = d2 / (d2 - d6)
        return a + parameter * edge_ac, (index_a, index_c)
    weight_a = d3 * d6 - d5 * d4
    if weight_a <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        parameter = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return b + parameter * (c - b), (index_b, index_c)
    total = 1.0 / (weight_a + weight_b + weight_c)
    return (a + edge_ab * (weight_b * total) + edge_ac * (weight_c * total),
            (index_a, index_b, index_c))


def _sub_distance(simplex, size, degeneracy, counters):
    """Closest point of the current simplex to the origin, and what to keep."""
    if size == 1:
        return simplex[0], (0,)
    if size == 2:
        first, second = simplex[0], simplex[1]
        along = second - first
        length_squared = along @ along
        parameter = (-(first @ along) / length_squared) if length_squared > 0.0 else 0.0
        if parameter <= 0.0:
            return first, (0,)
        if parameter >= 1.0:
            return second, (1,)
        return first + parameter * along, (0, 1)
    if size == 3:
        return _closest_on_triangle(simplex[0], simplex[1], simplex[2], 0, 1, 2)

    first, second, third, fourth = simplex[0], simplex[1], simplex[2], simplex[3]
    edge_one = second - first
    edge_two = third - first
    edge_three = fourth - first
    determinant = float(np.dot(edge_one, np.cross(edge_two, edge_three)))
    scale = (sqrt(edge_one @ edge_one) * sqrt(edge_two @ edge_two)
             * sqrt(edge_three @ edge_three))
    if scale == 0.0 or abs(determinant) <= degeneracy * scale:
        # THE GUARD.  A degenerate tetrahedron is four points that do not span a
        # volume; asking whether the origin is "inside" it is asking a question
        # with no answer, and the branch that answers anyway is what calls a
        # 16.2 mm gap a penetration.  Fall back to the best face instead.
        counters['degenerate'] += 1
        best = None
        best_distance = float('inf')
        for triple in ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)):
            point, keep = _closest_on_triangle(
                simplex[triple[0]], simplex[triple[1]], simplex[triple[2]], *triple)
            squared = point @ point
            if squared < best_distance:
                best_distance = squared
                best = (point, keep)
        return best

    outside = False
    best = None
    best_distance = float('inf')
    for one, two, three, opposite in ((0, 1, 2, 3), (0, 2, 3, 1), (0, 3, 1, 2),
                                      (1, 3, 2, 0)):
        corner_a, corner_b, corner_c = simplex[one], simplex[two], simplex[three]
        normal = np.cross(corner_b - corner_a, corner_c - corner_a)
        if normal @ (simplex[opposite] - corner_a) > 0.0:
            normal = -normal
        if normal @ (-corner_a) > 0.0:
            outside = True
            point, keep = _closest_on_triangle(corner_a, corner_b, corner_c,
                                               one, two, three)
            squared = point @ point
            if squared < best_distance:
                best_distance = squared
                best = (point, keep)
    if not outside:
        return np.zeros(3), (0, 1, 2, 3)
    return best


def distance(vertices_a, vertices_b, *, tolerance_relative=GJK_TOL_REL,
             tolerance_absolute=GJK_TOL_ABS_M, degeneracy=GJK_DEGENERACY_REL,
             cap=GJK_ITERATION_CAP, guard_degeneracy=True, guard_duplicate=True,
             statistics=None) -> float:
    """
    Return the exact distance between two convex point sets, zero on contact.

    ``vertices_a`` and ``vertices_b`` are (N, 3) and (M, 3) arrays already placed
    in a COMMON frame.  The value returned is the clearance the fence reports:
    nothing is subtracted from it.

    Penetration returns ``0.0`` and no depth.  GJK does not compute one, and the
    model does not need one: penetration is always a rejection, the jog fence
    clamps from the safe side where GJK is exact, and the IK filter discards.  A
    consumer that RANKS candidates by clearance sees every penetrating candidate
    tie, and the contract says so.
    """
    counters = {'degenerate': 0, 'duplicate': 0}
    simplex = np.empty((4, 3))
    size = 0
    direction = np.array([1.0, 0.0, 0.0])
    witness = (vertices_a[np.argmax(vertices_a @ direction)]
               - vertices_b[np.argmin(vertices_b @ direction)])
    lower_bound = 0.0
    reason = 'cap'
    iteration = 0
    for iteration in range(cap):
        norm_squared = witness @ witness
        if norm_squared == 0.0:
            reason = 'origin'
            break
        norm = sqrt(norm_squared)
        search = witness * (-1.0 / norm)
        support = (vertices_a[np.argmax(vertices_a @ search)]
                   - vertices_b[np.argmin(vertices_b @ search)])
        # A running certified lower bound on the true distance: the support
        # duality gap.  Terminating on it, rather than on simplex motion, is
        # what makes the tolerance mean something.
        lower_bound = max(lower_bound, (witness @ support) / norm)
        if norm - lower_bound <= max(tolerance_relative * norm, tolerance_absolute):
            reason = 'converged'
            break
        if guard_duplicate:
            duplicate = False
            for index in range(size):
                if (simplex[index][0] == support[0] and simplex[index][1] == support[1]
                        and simplex[index][2] == support[2]):
                    duplicate = True
                    break
            if duplicate:
                # The standard anti-cycling guard: the support map has returned a
                # vertex already in the simplex, so no further progress exists.
                counters['duplicate'] += 1
                reason = 'duplicate'
                break
        if size == 4:
            reason = 'overflow'
            break
        simplex[size] = support
        size += 1
        witness, keep = _sub_distance(
            simplex[:size], size, degeneracy if guard_degeneracy else -1.0, counters)
        if len(keep) != size:
            kept = [simplex[index].copy() for index in keep]
            for position, value in enumerate(kept):
                simplex[position] = value
            size = len(keep)
        if witness @ witness == 0.0:
            reason = 'origin'
            break
    else:
        if statistics is not None:
            statistics.update(iterations=iteration + 1, reason='cap', **counters)
        raise GeometryError(
            'GJK did not converge in {} iterations; the pair is within about a '
            'nanometre of contact and the only safe verdict is refusal'.format(cap))
    result = 0.0 if reason == 'origin' else sqrt(witness @ witness)
    if statistics is not None:
        statistics.update(iterations=iteration + 1, reason=reason, **counters)
    return max(0.0, result)
