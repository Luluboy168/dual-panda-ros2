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
The census oracle: an independent answer to "how far apart are these two links".

WHERE IT LIVES AND WHY.  Under ``test/``.  Never under
``franka_workspace_model/``, never imported by a runtime module, never a package
dependency; ``test_purity_and_performance.py`` asserts all three, and it also
asserts that this file EXISTS and exposes ``distance`` and ``link_bodies``, so
that deleting the oracle fails a different file than the one deleted.

WHAT IS INDEPENDENT ABOUT IT, stated precisely rather than claimed loosely.

*The kinematics are independent.*  A second forward chain, built from
``link_geometry_v1.yaml``'s own ``parent_joint`` origins with the rotation
written longhand as ``Rz(yaw) Ry(pitch) Rx(roll)`` and Rodrigues for the
revolute axes.  It shares no line of code with ``CellModel._transforms``.

*The distance computation is independent.*  Not a second GJK.  GJK's answer
comes out of a simplex sub-distance routine, and a second implementation of the
same routine would be a second chance to make the same mistake.  This one is
**MDM** (Mitchell-Dem'yanov-Malozemov): a pairwise-step Frank-Wolfe method over
the Minkowski difference, with no simplex at all.  Its convergence mechanism is
different, its failure modes are different, and - the property that matters - it
produces a **certified interval** on every call::

    lower = max over steps of  <n, support gap>       (a SEPARATING DIRECTION:
                                                       rigorous, whatever any
                                                       solver says)
    upper = |x| for the current convex combination x  (a POINT of the Minkowski
                                                       difference: also rigorous)

so the oracle never has to be trusted.  It says how far apart the two bodies
are AND how much it does not know, and the census reads both.

*The bodies are not independent, and pretending otherwise would be worse.*  The
oracle reads the same pinned vertices the fence reads.  They are checked
elsewhere and by different means - byte-exact regeneration from the assets, the
shell-containment certificate, the eight undercut witnesses, the flange's
face-plane certificate - and building a second set here would measure a
different robot, which is not a cross-check but a confusion.

*Feature enumeration is NOT the arbiter.*  Its point-to-triangle term
over-reports on sliver triangles, and decimated STLs are full of slivers.  Where
this oracle and a solver disagree, the tie-breaker is the SLSQP quadratic
program over the two hulls' half-spaces (``quadratic_program_distance`` below),
which shares no code with either.
"""

import math
from pathlib import Path

from franka_workspace_model.mesh_runtime import load_mesh_bodies

import numpy as np

import yaml


#: Above this the census does not ask for an exact distance: every margin in the
#: model is at most 50 mm and ``swept_path_extra`` is 10 mm, so a certified lower
#: bound above 60 mm settles every criterion the census evaluates.
ORACLE_EXACT_GATE_M = 0.060
#: MDM stops when the certified interval is this tight, in metres.
ORACLE_TOLERANCE_M = 1e-9
#: ...and a relative companion, because the certificate's last digits converge
#: slowly on a wide separation and no census decision turns on them: at 0.2 m
#: this is 2e-08 m, four orders of magnitude finer than the tightest margin in
#: the model.  The bracket is returned either way, so a caller that needs more
#: can ask for more.
ORACLE_TOLERANCE_REL = 1e-7
#: The census asks for a DECISION far more often than for a digit, and the two
#: cost very different amounts.  Away-step Frank-Wolfe brackets a 30 mm pair to
#: a few micrometres in twenty-five iterations and needs hundreds more to reach
#: the last nanometre - so the census runs the cheap pass first and escalates
#: ONLY the pairs whose bracket actually straddles a threshold it has to decide.
#: Nothing is guessed at either level: the bracket is rigorous at every cap.
ORACLE_FAST_CAP = 25
ORACLE_DEEP_CAP = 2000
#: How tight a REPORTED number has to be before the quadratic program is asked.
#: One micrometre: four orders of magnitude below the tightest margin, and it is
#: the acceptance table's own precision.
ORACLE_REPORTED_TOLERANCE_M = 1e-6
ORACLE_ITERATION_CAP = 400


class OracleError(RuntimeError):
    """The oracle could not certify an answer; the census must not guess."""


# ---------------------------------------------------------------------------
# The second forward-kinematics chain
# ---------------------------------------------------------------------------

def rotation_from_rpy(roll, pitch, yaw):
    """Build R = Rz(yaw) Ry(pitch) Rx(roll) longhand, from three angles."""
    about_x = np.array([[1.0, 0.0, 0.0],
                        [0.0, math.cos(roll), -math.sin(roll)],
                        [0.0, math.sin(roll), math.cos(roll)]])
    about_y = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                        [0.0, 1.0, 0.0],
                        [-math.sin(pitch), 0.0, math.cos(pitch)]])
    about_z = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                        [math.sin(yaw), math.cos(yaw), 0.0],
                        [0.0, 0.0, 1.0]])
    return about_z @ about_y @ about_x


def rodrigues(axis, angle):
    """Rotation about a unit axis, from the Rodrigues formula written out."""
    unit = np.asarray(axis, dtype=float)
    unit = unit / np.linalg.norm(unit)
    cross = np.array([[0.0, -unit[2], unit[1]],
                      [unit[2], 0.0, -unit[0]],
                      [-unit[1], unit[0], 0.0]])
    return (np.eye(3) + math.sin(angle) * cross
            + (1.0 - math.cos(angle)) * (cross @ cross))


# ---------------------------------------------------------------------------
# MDM: the independent distance
# ---------------------------------------------------------------------------

def distance(body_a, body_b, tolerance=ORACLE_TOLERANCE_M, cap=ORACLE_ITERATION_CAP):
    """
    Return the distance between two convex point sets, with no simplex anywhere.

    Both arguments are (N, 3) arrays already placed in a common frame.  The
    method is MDM on the Minkowski difference ``A - B``: keep a convex
    combination ``x`` of difference vertices, and at each step move weight from
    the vertex that is worst-aligned with ``-x`` to the support vertex that is
    best-aligned, with an exact line search.  ``|x|`` is an upper bound because
    ``x`` is a point of the difference; the support gap along ``-x`` is a lower
    bound because it is a separating direction.  The two close, and the gap is
    returned by :func:`certified_interval` for a caller that wants to see it.
    """
    return certified_interval(body_a, body_b, tolerance, cap)[0]


def certified_interval(body_a, body_b, tolerance=ORACLE_TOLERANCE_M,
                       cap=ORACLE_ITERATION_CAP):
    """
    Return ``(value, lower, upper)``: the answer and what brackets it.

    ``lower`` is a rigorous lower bound from a separating direction and may be
    negative or zero when the bodies overlap; ``upper`` is a rigorous upper
    bound from an actual point of the Minkowski difference.  A caller that needs
    a decision rather than a number should read the bracket, not the value.
    """
    first = np.asarray(body_a, dtype=float)
    second = np.asarray(body_b, dtype=float)
    # Start from the support pair along the centroid offset: a good iterate
    # costs one pass and saves many.
    direction = second.mean(axis=0) - first.mean(axis=0)
    if not np.any(direction):
        direction = np.array([1.0, 0.0, 0.0])
    start = (int(np.argmax(first @ direction)), int(np.argmin(second @ direction)))
    active = [start]
    weights = np.array([1.0])
    points = np.array([first[start[0]] - second[start[1]]])
    iterate = points[0].copy()
    lower = -np.inf
    argmax = np.argmax
    argmin = np.argmin
    for _ in range(cap):
        norm = math.sqrt(float(iterate @ iterate))
        if norm <= tolerance:
            return 0.0, min(lower, 0.0), norm
        search = -iterate / norm
        index_a = int(argmax(first @ search))
        index_b = int(argmin(second @ search))
        support = first[index_a] - second[index_b]
        # The separating-direction certificate along the current -x.  Rigorous
        # whatever the iteration does next, and monotone because it is a max.
        lower = max(lower, float(iterate @ support) / norm)
        if norm - lower <= max(tolerance, ORACLE_TOLERANCE_REL * norm):
            return max(0.0, norm), lower, norm
        # Away-step Frank-Wolfe on f(x) = |x|^2 / 2 over the Minkowski
        # difference.  The gradient is x itself, so the two candidate
        # directions are s - x (toward the support vertex) and x - p_away
        # (away from the active vertex that is worst for the objective).  The
        # away step is what stops the plain method stalling along a face, and
        # it is the reason this converges without a simplex routine.
        toward_direction = support - iterate
        away = int(argmax(points @ iterate))
        away_direction = iterate - points[away]
        if float(-iterate @ toward_direction) >= float(-iterate @ away_direction):
            step_direction = toward_direction
            maximum = 1.0
            kind = 'toward'
        else:
            step_direction = away_direction
            weight = float(weights[away])
            if weight >= 1.0:
                return max(0.0, norm), lower, norm
            maximum = weight / (1.0 - weight)
            kind = 'away'
        denominator = float(step_direction @ step_direction)
        if denominator <= 0.0:
            return max(0.0, norm), lower, norm
        step = -float(iterate @ step_direction) / denominator
        step = max(0.0, min(step, maximum))
        if step <= 0.0:
            return max(0.0, norm), lower, norm
        key = (index_a, index_b)
        if kind == 'toward':
            weights = weights * (1.0 - step)
            if key in active:
                weights[active.index(key)] += step
            else:
                active.append(key)
                weights = np.append(weights, step)
                points = np.vstack([points, support])
        else:
            # w_i <- (1+g) w_i for every i, then w_away <- (1+g) w_away - g.
            # The sum stays exactly one: (1+g) - g = 1.
            weights = weights * (1.0 + step)
            weights[away] -= step
            weights[away] = max(0.0, float(weights[away]))
        iterate = points.T @ weights
        keep = weights > 1e-16
        if not keep.all() and keep.any():
            weights = weights[keep]
            points = points[keep]
            active = [pair for pair, flag in zip(active, keep) if flag]
            weights = weights / float(weights.sum())
            iterate = points.T @ weights
    # The cap is not a failure: the iterate is a real point of the Minkowski
    # difference and the best direction found is a real separating direction, so
    # the BRACKET is valid whatever the iteration did.  Returning it - rather
    # than raising - is the whole point of an oracle that reports what it does
    # not know.  A caller whose decision the bracket does not settle escalates
    # to the quadratic program.
    residual = math.sqrt(float(iterate @ iterate))
    return max(0.0, residual), lower, residual


def separating_gap(body_a, body_b, direction):
    """Return ``min_B<n,b> - max_A<n,a>``; positive PROVES no contact."""
    unit = np.asarray(direction, dtype=float)
    unit = unit / np.linalg.norm(unit)
    return float((np.asarray(body_b) @ unit).min() - (np.asarray(body_a) @ unit).max())


def quadratic_program_distance(body_a, body_b):
    """
    Settle a disagreement with SLSQP over the two hulls' half-spaces.

    Used to settle a disagreement and to validate the oracle at load; never in
    the census's inner loop, where it would cost hours.
    """
    from scipy.optimize import minimize
    from scipy.spatial import ConvexHull

    hull_a = ConvexHull(np.asarray(body_a))
    hull_b = ConvexHull(np.asarray(body_b))

    def halfspaces(hull, offset):
        normals = hull.equations[:, :3]
        offsets = -hull.equations[:, 3]
        return {'type': 'ineq',
                'fun': lambda x, n=normals, d=offsets, o=offset: d - n @ x[o:o + 3]}

    start = np.concatenate([np.asarray(body_a).mean(axis=0),
                            np.asarray(body_b).mean(axis=0)])
    result = minimize(lambda x: float(np.linalg.norm(x[:3] - x[3:])), start,
                      method='SLSQP',
                      constraints=[halfspaces(hull_a, 0), halfspaces(hull_b, 3)],
                      options={'maxiter': 500, 'ftol': 1e-14})
    return float(result.fun)


# ---------------------------------------------------------------------------
# The oracle proper
# ---------------------------------------------------------------------------

class MeshOracle:
    """The bodies, a second chain to place them, and a certified distance."""

    def __init__(self, link_geometry_path, mesh_bodies_path, arm_ids,
                 base_poses=None):
        self.geometry = yaml.safe_load(
            Path(link_geometry_path).read_text(encoding='utf-8'))
        self.bodies = load_mesh_bodies(
            mesh_bodies_path, urdf_sha256=self.geometry['source']['urdf_sha256'])
        self.arm_ids = tuple(arm_ids)
        self.joints = {}
        self.parent = {}
        self.root = self.geometry['root_link']
        for entry in self.geometry['links']:
            if 'parent_joint' in entry:
                self.joints[entry['link']] = entry['parent_joint']
                self.parent[entry['link']] = entry['parent_link']
        self.base_poses = dict(base_poses or {})
        self.link_names = tuple(sorted(self.bodies.by_link))
        self.calls = 0
        self.deep = 0
        self.escalations = 0

    # -- geometry ---------------------------------------------------------

    def link_bodies(self, link):
        """Return the bodies of one link, in that link's own frame."""
        return tuple(body.vertices for body in self.bodies.by_link[link])

    def transforms(self, configuration):
        """Place every arm link in the cell frame, from the artefact's origins."""
        placed = {self.root: np.eye(4)}
        for name in list(self.parent):
            chain = []
            walker = name
            while walker not in placed:
                chain.append(walker)
                walker = self.parent[walker]
            for link in reversed(chain):
                joint = self.joints[link]
                step = np.eye(4)
                step[:3, :3] = rotation_from_rpy(*joint['origin_rpy'])
                step[:3, 3] = joint['origin_xyz']
                if joint['type'] == 'revolute':
                    arm_id, index = self._joint_position(joint['name'])
                    spin = np.eye(4)
                    spin[:3, :3] = rodrigues(joint['axis'],
                                             configuration[arm_id][index])
                    step = step @ spin
                placed[link] = placed[self.parent[link]] @ step
        return placed

    def _joint_position(self, name):
        for arm_id in self.arm_ids:
            if name.startswith(arm_id + '_joint'):
                return arm_id, int(name.rsplit('joint', 1)[1]) - 1
        raise OracleError('joint {!r} belongs to no declared arm'.format(name))

    def place(self, configuration):
        """Return ``{(arm_id, body_id): vertices}`` in the cell frame."""
        transforms = self.transforms(configuration)
        placed = {}
        for arm_id in self.arm_ids:
            for link, bodies in self.bodies.by_link.items():
                transform = transforms['{}_{}'.format(arm_id, link)]
                rotation = transform[:3, :3]
                translation = transform[:3, 3]
                for body in bodies:
                    placed[(arm_id, body.id)] = body.vertices @ rotation.T + translation
        return placed

    # -- distances --------------------------------------------------------

    def capsule_bounds(self, configuration):
        """Certified lower bounds for every body, from the pinned capsules."""
        transforms = self.transforms(configuration)
        bounds = {}
        for arm_id in self.arm_ids:
            for link, bodies in self.bodies.by_link.items():
                transform = transforms['{}_{}'.format(arm_id, link)]
                rotation = transform[:3, :3]
                translation = transform[:3, 3]
                for body in bodies:
                    bounds[(arm_id, body.id)] = (
                        rotation @ body.capsule_a + translation,
                        rotation @ body.capsule_b + translation,
                        body.capsule_radius)
        return bounds

    def resolve(self, body_a, body_b, thresholds):
        """
        Return a distance whose comparison with every threshold is CERTIFIED.

        The cheap pass brackets the answer; if the bracket straddles a threshold
        the census has to decide, the pair is escalated - first to a deep
        Frank-Wolfe run, then to the SLSQP quadratic program, which shares no
        code with this file or with the fence.  A decision is never taken on an
        undecided bracket, and the escalation counter is reported so that an
        oracle that has quietly become useless is visible rather than silent.
        """
        for cap in (ORACLE_FAST_CAP, ORACLE_DEEP_CAP):
            value, lower, upper = certified_interval(body_a, body_b, cap=cap)
            if all(not (lower <= threshold <= upper) for threshold in thresholds):
                return value
            self.deep += 1
        self.escalations += 1
        return quadratic_program_distance(body_a, body_b)

    def refine(self, placed, first, second):
        """
        Recompute one link pair at full precision, for a number that is REPORTED.

        The census decides most comparisons from a cheap bracket, which is sound
        but leaves a value a few micrometres high.  A number that lands in the
        acceptance table - the tightest accepted metal, the worst hazard - is
        read by a person, so it is re-solved deeply and, where even that leaves
        the last digits open, by the quadratic program.
        """
        arm_a, link_a = first
        arm_b, link_b = second
        best = math.inf
        for body_a in self.bodies.by_link[link_a]:
            for body_b in self.bodies.by_link[link_b]:
                pair = (placed[(arm_a, body_a.id)], placed[(arm_b, body_b.id)])
                value, lower, upper = certified_interval(*pair, cap=ORACLE_DEEP_CAP)
                if upper - lower > ORACLE_REPORTED_TOLERANCE_M:
                    self.escalations += 1
                    value = quadratic_program_distance(*pair)
                best = min(best, value)
        return best

    def link_pair_distance(self, placed, first, second, gate=ORACLE_EXACT_GATE_M,
                           capsules=None, thresholds=()):
        """
        Return the distance between two placed LINKS, min over their bodies.

        Above ``gate`` the value returned is a certified LOWER BOUND rather than
        an exact distance, and that is enough: every margin in this model is at
        most 50 mm and ``swept_path_extra`` is 10 mm, so nothing the census
        decides can turn on the difference between 61 mm and 400 mm.  The value
        is conservative in the direction that matters - it never claims more
        clearance than there is.
        """
        best = math.inf
        arm_a, link_a = first
        arm_b, link_b = second
        for body_a in self.bodies.by_link[link_a]:
            for body_b in self.bodies.by_link[link_b]:
                key_a = (arm_a, body_a.id)
                key_b = (arm_b, body_b.id)
                if capsules is not None:
                    point_a, point_b, radius_a = capsules[key_a]
                    point_c, point_d, radius_b = capsules[key_b]
                    bound = (_segment_segment(point_a, point_b, point_c, point_d)
                             - radius_a - radius_b)
                    if bound >= gate:
                        best = min(best, bound)
                        continue
                self.calls += 1
                best = min(best, self.resolve(placed[key_a], placed[key_b],
                                              thresholds))
        return best


def _segment_segment(a_1, b_1, a_2, b_2):
    """Closest distance between two segments, written out for this file alone."""
    d_1 = b_1 - a_1
    d_2 = b_2 - a_2
    r = a_1 - a_2
    a = float(d_1 @ d_1)
    e = float(d_2 @ d_2)
    f = float(d_2 @ r)
    if a <= 1e-18 and e <= 1e-18:
        return float(np.linalg.norm(r))
    if a <= 1e-18:
        s = 0.0
        t = min(max(f / e, 0.0), 1.0)
    else:
        c = float(d_1 @ r)
        if e <= 1e-18:
            t = 0.0
            s = min(max(-c / a, 0.0), 1.0)
        else:
            b = float(d_1 @ d_2)
            denominator = a * e - b * b
            if denominator > 1e-18 * a * e:
                s = min(max((b * f - c * e) / denominator, 0.0), 1.0)
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t = 1.0
                s = min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm(r + s * d_1 - t * d_2))
