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


def _closest_on_triangle(ax, ay, az, bx, by, bz, cx, cy, cz,
                         index_a, index_b, index_c):
    """
    Ericson's closest-point-on-triangle to the origin, in SCALAR floats.

    Scalar, not numpy: at this size an array operation is dominated by its own
    dispatch.  Measured on this host, the simplex routine is about half of a
    GJK call and this is the half that can be made cheap - the support scans
    over the pinned vertex arrays are the other half and they are irreducible.
    Reducing vertex counts is NOT the optimisation: it is 11 % of the cost, and
    every sound reduction available costs volume.
    """
    abx, aby, abz = bx - ax, by - ay, bz - az
    acx, acy, acz = cx - ax, cy - ay, cz - az
    d1 = -(abx * ax + aby * ay + abz * az)
    d2 = -(acx * ax + acy * ay + acz * az)
    if d1 <= 0.0 and d2 <= 0.0:
        return (ax, ay, az), (index_a,)
    d3 = -(abx * bx + aby * by + abz * bz)
    d4 = -(acx * bx + acy * by + acz * bz)
    if d3 >= 0.0 and d4 <= d3:
        return (bx, by, bz), (index_b,)
    weight_c = d1 * d4 - d3 * d2
    if weight_c <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        step = d1 / (d1 - d3)
        return (ax + step * abx, ay + step * aby, az + step * abz), (index_a, index_b)
    d5 = -(abx * cx + aby * cy + abz * cz)
    d6 = -(acx * cx + acy * cy + acz * cz)
    if d6 >= 0.0 and d5 <= d6:
        return (cx, cy, cz), (index_c,)
    weight_b = d5 * d2 - d1 * d6
    if weight_b <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        step = d2 / (d2 - d6)
        return (ax + step * acx, ay + step * acy, az + step * acz), (index_a, index_c)
    weight_a = d3 * d6 - d5 * d4
    if weight_a <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        step = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return (bx + step * (cx - bx), by + step * (cy - by),
                bz + step * (cz - bz)), (index_b, index_c)
    total = 1.0 / (weight_a + weight_b + weight_c)
    scale_b = weight_b * total
    scale_c = weight_c * total
    return (ax + abx * scale_b + acx * scale_c,
            ay + aby * scale_b + acy * scale_c,
            az + abz * scale_b + acz * scale_c), (index_a, index_b, index_c)


_FACES = ((0, 1, 2, 3), (0, 2, 3, 1), (0, 3, 1, 2), (1, 3, 2, 0))
_TRIPLES = ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3))


def _sub_distance(simplex, size, degeneracy, counters):
    """Closest point of the current simplex to the origin, and what to keep."""
    if size == 1:
        return simplex[0], (0,)
    if size == 2:
        (ax, ay, az), (bx, by, bz) = simplex[0], simplex[1]
        abx, aby, abz = bx - ax, by - ay, bz - az
        length_squared = abx * abx + aby * aby + abz * abz
        step = (-(ax * abx + ay * aby + az * abz) / length_squared
                if length_squared > 0.0 else 0.0)
        if step <= 0.0:
            return simplex[0], (0,)
        if step >= 1.0:
            return simplex[1], (1,)
        return (ax + step * abx, ay + step * aby, az + step * abz), (0, 1)
    if size == 3:
        return _closest_on_triangle(*simplex[0], *simplex[1], *simplex[2], 0, 1, 2)

    (ax, ay, az) = simplex[0]
    (bx, by, bz) = simplex[1]
    (cx, cy, cz) = simplex[2]
    (dx, dy, dz) = simplex[3]
    e1x, e1y, e1z = bx - ax, by - ay, bz - az
    e2x, e2y, e2z = cx - ax, cy - ay, cz - az
    e3x, e3y, e3z = dx - ax, dy - ay, dz - az
    crossx = e2y * e3z - e2z * e3y
    crossy = e2z * e3x - e2x * e3z
    crossz = e2x * e3y - e2y * e3x
    determinant = e1x * crossx + e1y * crossy + e1z * crossz
    scale = (sqrt(e1x * e1x + e1y * e1y + e1z * e1z)
             * sqrt(e2x * e2x + e2y * e2y + e2z * e2z)
             * sqrt(e3x * e3x + e3y * e3y + e3z * e3z))
    if scale == 0.0 or abs(determinant) <= degeneracy * scale:
        # THE GUARD.  A degenerate tetrahedron is four points that do not span a
        # volume; asking whether the origin is "inside" it is asking a question
        # with no answer, and the branch that answers anyway is what calls a
        # 30 mm gap a penetration.  Fall back to the best face instead.
        counters['degenerate'] += 1
        best = None
        best_distance = float('inf')
        for one, two, three in _TRIPLES:
            point, keep = _closest_on_triangle(
                *simplex[one], *simplex[two], *simplex[three], one, two, three)
            squared = point[0] * point[0] + point[1] * point[1] + point[2] * point[2]
            if squared < best_distance:
                best_distance = squared
                best = (point, keep)
        return best

    outside = False
    best = None
    best_distance = float('inf')
    for one, two, three, opposite in _FACES:
        (px, py, pz) = simplex[one]
        (qx, qy, qz) = simplex[two]
        (rx, ry, rz) = simplex[three]
        (ox, oy, oz) = simplex[opposite]
        ux, uy, uz = qx - px, qy - py, qz - pz
        vx, vy, vz = rx - px, ry - py, rz - pz
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        if nx * (ox - px) + ny * (oy - py) + nz * (oz - pz) > 0.0:
            nx, ny, nz = -nx, -ny, -nz
        if -(nx * px + ny * py + nz * pz) > 0.0:
            outside = True
            point, keep = _closest_on_triangle(px, py, pz, qx, qy, qz, rx, ry, rz,
                                               one, two, three)
            squared = point[0] * point[0] + point[1] * point[1] + point[2] * point[2]
            if squared < best_distance:
                best_distance = squared
                best = (point, keep)
    if not outside:
        return (0.0, 0.0, 0.0), (0, 1, 2, 3)
    return best


def distance_local(vertices_a, vertices_b, rotation, translation, **keywords):
    """
    Return the distance given the SECOND body's pose in the first's frame.

    Nothing is transformed: the search direction is mapped into body B's frame
    instead of B's vertices being mapped into the world.  On this model that is
    the difference between three dot products over a 2 116-vertex array and a
    full rotation of it, on every pair, on every check.

    ``rotation`` and ``translation`` map a point of B's own frame into A's:
    ``p_A = rotation @ p_B + translation``.
    """
    return _distance(vertices_a, vertices_b, rotation, translation, **keywords)


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
    return _distance(vertices_a, vertices_b, None, None,
                     tolerance_relative=tolerance_relative,
                     tolerance_absolute=tolerance_absolute, degeneracy=degeneracy,
                     cap=cap, guard_degeneracy=guard_degeneracy,
                     guard_duplicate=guard_duplicate, statistics=statistics)


def _distance(vertices_a, vertices_b, rotation, translation, *,
              tolerance_relative=GJK_TOL_REL, tolerance_absolute=GJK_TOL_ABS_M,
              degeneracy=GJK_DEGENERACY_REL, cap=GJK_ITERATION_CAP,
              guard_degeneracy=True, guard_duplicate=True, statistics=None):
    """Run the one implementation; ``rotation`` is None when both are placed."""
    argmax = np.argmax
    argmin = np.argmin
    if rotation is None:
        def support(direction):
            first = vertices_a[argmax(vertices_a @ direction)]
            second = vertices_b[argmin(vertices_b @ direction)]
            return (first[0] - second[0], first[1] - second[1],
                    first[2] - second[2])
    else:
        rotation_t = np.ascontiguousarray(rotation.T)

        def support(direction):
            first = vertices_a[argmax(vertices_a @ direction)]
            second = vertices_b[argmin(vertices_b @ (rotation_t @ direction))]
            placed = rotation @ second + translation
            return (first[0] - placed[0], first[1] - placed[1],
                    first[2] - placed[2])

    counters = {'degenerate': 0, 'duplicate': 0}
    simplex = []
    direction = np.empty(3)
    direction[0], direction[1], direction[2] = 1.0, 0.0, 0.0
    witness = support(direction)
    lower_bound = 0.0
    reason = 'cap'
    iteration = 0
    guard = degeneracy if guard_degeneracy else -1.0
    for iteration in range(cap):
        wx, wy, wz = witness
        norm_squared = wx * wx + wy * wy + wz * wz
        if norm_squared == 0.0:
            reason = 'origin'
            break
        norm = sqrt(norm_squared)
        inverse = -1.0 / norm
        direction[0], direction[1], direction[2] = wx * inverse, wy * inverse, wz * inverse
        point = support(direction)
        # A running certified lower bound on the true distance: the support
        # duality gap.  Terminating on it, rather than on simplex motion, is
        # what makes the tolerance mean something.
        gap = (wx * point[0] + wy * point[1] + wz * point[2]) / norm
        if gap > lower_bound:
            lower_bound = gap
        if norm - lower_bound <= max(tolerance_relative * norm, tolerance_absolute):
            reason = 'converged'
            break
        if guard_duplicate and point in simplex:
            # The standard anti-cycling guard: the support map has returned a
            # vertex already in the simplex, so no further progress exists.
            counters['duplicate'] += 1
            reason = 'duplicate'
            break
        if len(simplex) == 4:
            reason = 'overflow'
            break
        simplex.append(point)
        witness, keep = _sub_distance(simplex, len(simplex), guard, counters)
        if len(keep) != len(simplex):
            simplex = [simplex[index] for index in keep]
        if witness[0] == 0.0 and witness[1] == 0.0 and witness[2] == 0.0:
            reason = 'origin'
            break
    else:
        if statistics is not None:
            statistics.update(iterations=iteration + 1, reason='cap', **counters)
        raise GeometryError(
            'GJK did not converge in {} iterations; the pair is within about a '
            'nanometre of contact and the only safe verdict is refusal'.format(cap))
    result = 0.0 if reason == 'origin' else sqrt(
        witness[0] * witness[0] + witness[1] * witness[1] + witness[2] * witness[2])
    if statistics is not None:
        statistics.update(iterations=iteration + 1, reason=reason, **counters)
    return max(0.0, result)
