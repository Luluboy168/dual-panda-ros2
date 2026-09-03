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
The mesh fence: a certified broad phase that culls PER PAIR, and GJK behind it.

THE CULL IS THE MARGIN, NOT A KNOB.  The obvious design sorts candidate pairs
by a lower bound and stops as soon as the bound exceeds the best exact distance
found so far.  That computes the MINIMUM.  It does not compute the contact
LIST, and the two are different questions the moment margins differ per class -
self 20 mm, cross-arm 50 mm, plus ``swept_path_extra`` on a path.  A self pair
at 30 mm can be the global minimum and pass while a cross-arm pair at 40 mm
violates its own 50 mm margin and is never evaluated.

So every body pair carries its OWN margin, in a vector built once at load, and
a pair is culled only when its certified lower bound exceeds THAT margin::

    lower_bound(a, b) = seg_seg(A, B) - r_a - r_b   <=   gjk(body_a, body_b)
    evaluate  iff  lower_bound(a, b) <= M[pair]

A culled pair provably cannot violate its own margin.  There is no global
quantity anywhere in the cull, so tightening a margin cannot make it unsound
and loosening one cannot make it miss a contact.

``r_a`` and ``r_b`` live here and nowhere else.  They are the bounding-capsule
radii - each the EXACT maximum vertex-to-segment distance of its own body - and
they never appear in a reported number.  The reported clearance is
``gjk(body_a, body_b)``, full stop; subtracting the radii from it would be a
double subtraction worth -107 mm on link5/link7 at the ready pose, which would
refuse the home pose.

NO VERTEX ARRAY IS EVER TRANSFORMED.  GJK's support is evaluated in each body's
own frame by mapping the search direction instead, so a check costs a handful of
dot products over the pinned vertices and not a rotation of eleven thousand
points.
"""

import math

import numpy as np

from .gjk import distance_local
from ..geometry import GeometryError, segment_segment_distance_batch


class MeshFence:
    """The body set, the pair sets and the per-pair margins, built once."""

    def __init__(self, bodies, arm_ids, enabled_link_pairs, cross_arm_enabled,
                 pedestal_link=None):
        self.arm_ids = tuple(arm_ids)
        self.entries = []
        for arm_id in self.arm_ids:
            for body in bodies.bodies:
                self.entries.append({
                    'id': '{}_{}'.format(arm_id, body.id),
                    'arm_id': arm_id,
                    'link': '{}_{}'.format(arm_id, body.link),
                    'bare_link': body.link,
                    'body': body,
                })
        self.position = {entry['id']: index
                         for index, entry in enumerate(self.entries)}
        count = len(self.entries)
        self.capsule_a = np.array([entry['body'].capsule_a
                                   for entry in self.entries])
        self.capsule_b = np.array([entry['body'].capsule_b
                                   for entry in self.entries])
        self.radii = np.array([entry['body'].capsule_radius
                               for entry in self.entries])
        self.vertices = [entry['body'].vertices for entry in self.entries]

        intra = []
        for arm_id in self.arm_ids:
            allowed = enabled_link_pairs.get(arm_id, set())
            rows = [index for index, entry in enumerate(self.entries)
                    if entry['arm_id'] == arm_id]
            for position, first in enumerate(rows):
                for second in rows[position + 1:]:
                    pair = tuple(sorted((self.entries[first]['bare_link'],
                                         self.entries[second]['bare_link'])))
                    if pair not in allowed:
                        continue
                    intra.append((first, second, arm_id, pair))
        self.intra = tuple(intra)
        cross = []
        if cross_arm_enabled and len(self.arm_ids) == 2:
            first_rows = [index for index, entry in enumerate(self.entries)
                          if entry['arm_id'] == self.arm_ids[0]]
            second_rows = [index for index, entry in enumerate(self.entries)
                           if entry['arm_id'] == self.arm_ids[1]]
            for first in first_rows:
                for second in second_rows:
                    cross.append((first, second, self.arm_ids[0], None))
        self.cross = tuple(cross)
        # The index arrays and the radius sums are built ONCE.  Rebuilding them
        # per query costs more than the distance computation they feed.
        self.index = {}
        for name, pairs in (('intra', self.intra), ('cross', self.cross)):
            first = np.array([item[0] for item in pairs], dtype=int)
            second = np.array([item[1] for item in pairs], dtype=int)
            self.index[name] = (first, second,
                                self.radii[first] + self.radii[second])
        self.pedestal_link = pedestal_link
        del count

    # -- the per-pair margin vector, built once at load --------------------

    def margin_vectors(self, self_margin, cross_margin, pair_margins):
        """
        Return the per-pair margin arrays.  Built ONCE, never per query.

        ``pair_margins`` maps ``(arm_id, link_a, link_b)`` to the ruled margin
        for that link pair; every body pair of that link pair inherits it, which
        is the same expansion rule ``extra_enabled_pairs`` already uses.
        """
        intra = np.empty(len(self.intra))
        for index, (_, _, arm_id, pair) in enumerate(self.intra):
            intra[index] = pair_margins.get((arm_id,) + pair, self_margin)
        cross = np.full(len(self.cross), cross_margin)
        return intra, cross

    # -- placement --------------------------------------------------------

    def place(self, transforms):
        """
        Return the per-body rotations, translations and capsule endpoints.

        Only the two capsule endpoints are transformed per body.  The vertex
        arrays stay in their own frames and the search direction is mapped into
        them instead.
        """
        rotations = np.empty((len(self.entries), 3, 3))
        translations = np.empty((len(self.entries), 3))
        for index, entry in enumerate(self.entries):
            transform = transforms[entry['link']]
            rotations[index] = transform[:3, :3]
            translations[index] = transform[:3, 3]
        ends_a = np.einsum('kij,kj->ki', rotations, self.capsule_a) + translations
        ends_b = np.einsum('kij,kj->ki', rotations, self.capsule_b) + translations
        return rotations, translations, ends_a, ends_b

    def lower_bounds(self, ends_a, ends_b, name):
        """Return the certified bound for one whole pair class, vectorised."""
        first, second, radius_sum = self.index[name]
        if not len(first):
            return np.zeros(0)
        separation = segment_segment_distance_batch(
            ends_a[first], ends_b[first], ends_a[second], ends_b[second])
        return separation - radius_sum

    def clearance(self, rotations, translations, first, second):
        """
        Return the REPORTED clearance between two placed bodies.

        ``gjk(body_a, body_b)``.  Nothing is subtracted from it: not an undercut
        (the bodies contain the shell), not a coverage term, and not a capsule
        radius.
        """
        rotation_a = rotations[first]
        rotation_b = rotations[second]
        relative_rotation = rotation_a.T @ rotation_b
        relative_translation = rotation_a.T @ (translations[second]
                                               - translations[first])
        return distance_local(self.vertices[first], self.vertices[second],
                              relative_rotation, relative_translation)

    # -- the two steps a caller drives ------------------------------------

    def pair_step(self, rotations, translations, ends_a, ends_b, name, margins,
                  kind, extra, contacts, first_violation):
        """
        Evaluate one class of body pairs, culling each against its OWN margin.

        Returns the running minimum contribution and whether the caller should
        stop.  Culled pairs contribute their certified LOWER BOUND to the
        minimum, which is valid, already computed, costs nothing, and keeps
        ``min_clearance`` a true lower bound on the tightest slack instead of an
        unbounded over-estimate.
        """
        minimum = math.inf
        pairs = self.intra if name == 'intra' else self.cross
        if not pairs:
            return minimum, False
        bounds = self.lower_bounds(ends_a, ends_b, name)
        effective = margins if extra == 0.0 else margins + extra
        candidates = np.nonzero(bounds <= effective)[0]
        culled = np.nonzero(bounds > effective)[0]
        if len(culled):
            minimum = min(minimum, float((bounds[culled]
                                          - effective[culled]).min()))
        for position in candidates:
            first, second, arm_id, _ = pairs[int(position)]
            value = self.clearance(rotations, translations, first, second)
            margin = float(effective[position])
            minimum = min(minimum, value - margin)
            if value < margin:
                contacts.append((kind, self.entries[first]['id'],
                                 self.entries[second]['id'], value, margin,
                                 arm_id))
                if first_violation:
                    return minimum, True
        return minimum, False

    def containment(self, rotations, translations, ends_a, ends_b, rows, masks,
                    box_lower, box_upper, margin):
        """
        Every body's placed extent against the six faces of the allowed volume.

        Exact where it matters, and cheaper than a GJK call: the constraint is
        per half-space, so the extreme really is at a vertex and a min/max over
        the placed vertices settles it.  No union argument is needed and none is
        made - that argument is what section 2.3 retired.

        The bodies that are nowhere near a face are settled by their own bounding
        capsule first.  Every point of a body lies within ``r`` of its segment,
        so the capsule's axis-aligned extent is an OUTER bound on the body's, and
        a body whose capsule clears every face by more than the margin cannot
        violate one.  Those bodies contribute the conservative value - a valid
        lower bound on their true clearance - and skip the vertex pass entirely.
        """
        rows = np.asarray(rows, dtype=int)
        low = np.minimum(ends_a[rows], ends_b[rows]) - self.radii[rows][:, None]
        high = np.maximum(ends_a[rows], ends_b[rows]) + self.radii[rows][:, None]
        values = np.empty((len(rows), 6))
        values[:, 0::2] = low - box_lower
        values[:, 1::2] = box_upper - high
        masked = np.where(masks, values, math.inf)
        close = np.nonzero(masked.min(axis=1) < margin)[0]
        for position in close:
            index = int(rows[position])
            projected = self.vertices[index] @ rotations[index].T
            exact_low = projected.min(axis=0) + translations[index]
            exact_high = projected.max(axis=0) + translations[index]
            values[position, 0::2] = exact_low - box_lower
            values[position, 1::2] = box_upper - exact_high
        masked = np.where(masks, values, math.inf)
        if not np.all(np.isfinite(masked.min(axis=1))):
            raise GeometryError('containment')
        return masked, close

    def box_lower_bounds(self, ends_a, ends_b, rows, centre, radius):
        """Certified pedestal bounds for a whole batch of bodies at once."""
        rows = np.asarray(rows, dtype=int)
        start = ends_a[rows]
        span = ends_b[rows] - start
        length = np.einsum('ij,ij->i', span, span)
        offset = centre[None, :] - start
        with np.errstate(divide='ignore', invalid='ignore'):
            parameter = np.where(length > 0.0,
                                 np.einsum('ij,ij->i', offset, span)
                                 / np.where(length > 0.0, length, 1.0), 0.0)
        parameter = np.clip(parameter, 0.0, 1.0)
        foot = start + parameter[:, None] * span
        return (np.linalg.norm(centre[None, :] - foot, axis=1)
                - self.radii[rows] - radius)

    def box_lower_bound(self, ends_a, ends_b, index, centre, radius):
        """
        Bound the pedestal step from the same two capsules, and certify it.

        The declared box is bounded by the sphere the load-time diagnostic
        already uses; the body is bounded by its own exact capsule.  A pedestal
        pair whose bound clears its margin is culled exactly like any other, so
        the step costs a segment-point distance rather than a GJK call on the
        eighteen pairs that are nowhere near the plinth.
        """
        from ..geometry import segment_point_distance
        return (segment_point_distance(ends_a[index], ends_b[index], centre)
                - self.radii[index] - radius)

    def box_clearance(self, rotations, translations, index, box_vertices):
        """
        Measure a body against a declared box with an ordinary GJK call.

        A min over placed vertices of point-to-box distance is NOT exact here -
        it over-estimates, which is the unsafe direction.  Containment is
        different: there the constraint is per half-space and the extreme really
        is at a vertex.
        """
        rotation = rotations[index]
        relative_rotation = rotation.T
        relative_translation = -rotation.T @ translations[index]
        return distance_local(self.vertices[index], box_vertices,
                              relative_rotation, relative_translation)
