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
Closed-form distance primitives for the workspace model.

Four primitives cover every check the checking policy performs: segment-segment,
segment-point, segment-OBB and point-plane.  All are closed form, all use numpy
only, and none needs a collision library.  Every primitive validates that its
result is finite before returning it; a non-finite result is a fail-closed
condition for the caller, never a value to propagate.

No module here imports ROS, and none may.
"""

import math

import numpy as np


# Relative singularity threshold for the parallel-segment case.  Relative, not
# absolute, so the test is scale free.
PARALLEL_EPSILON = 1e-18
# Breakpoint de-duplication tolerance for the segment-OBB enumeration.
BREAKPOINT_EPSILON = 1e-12
# Below this squared length a segment is treated as a point by the runtime.
DEGENERATE_LENGTH_SQUARED = 1e-18


class GeometryError(ArithmeticError):
    """A primitive produced a non-finite result: an assumption is wrong."""


def _finite(value: float, context: str) -> float:
    if not math.isfinite(value):
        raise GeometryError('{} produced a non-finite distance'.format(context))
    return float(value)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the fixed-axis roll-pitch-yaw matrix R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cos_y * cos_p,
         cos_y * sin_p * sin_r - sin_y * cos_r,
         cos_y * sin_p * cos_r + sin_y * sin_r],
        [sin_y * cos_p,
         sin_y * sin_p * sin_r + cos_y * cos_r,
         sin_y * sin_p * cos_r - cos_y * sin_r],
        [-sin_p, cos_p * sin_r, cos_p * cos_r],
    ], dtype=float)


def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Assemble a 4x4 homogeneous transform."""
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def segment_point_distance(point_a, point_b, point_p) -> float:
    """
    Distance from the segment [a, b] to the point p.

    A zero-length segment is answered at its first endpoint, which is the
    correct answer rather than a division by zero.
    """
    direction = point_b - point_a
    length_squared = float(direction @ direction)
    if length_squared <= DEGENERATE_LENGTH_SQUARED:
        parameter = 0.0
    else:
        parameter = float((point_p - point_a) @ direction) / length_squared
        parameter = min(max(parameter, 0.0), 1.0)
    closest = point_a + parameter * direction
    return _finite(float(np.linalg.norm(point_p - closest)), 'segment-point')


def segment_segment_distance(a_1, b_1, a_2, b_2) -> float:
    """
    Distance between the segments [a1, b1] and [a2, b2].

    The clamped two-parameter solve.  Parallel and collinear segments attain the
    minimum on an interval; the tie-break (s = 0, then clamp t, then re-clamp s)
    is deterministic and returns the same number everywhere on that interval.
    """
    d_1 = b_1 - a_1
    d_2 = b_2 - a_2
    offset = a_1 - a_2
    length_1 = float(d_1 @ d_1)
    length_2 = float(d_2 @ d_2)
    if length_1 <= DEGENERATE_LENGTH_SQUARED and length_2 <= DEGENERATE_LENGTH_SQUARED:
        return _finite(float(np.linalg.norm(offset)), 'point-point')
    if length_1 <= DEGENERATE_LENGTH_SQUARED:
        return segment_point_distance(a_2, b_2, a_1)
    if length_2 <= DEGENERATE_LENGTH_SQUARED:
        return segment_point_distance(a_1, b_1, a_2)
    projection = float(d_1 @ d_2)
    start_1 = float(d_1 @ offset)
    start_2 = float(d_2 @ offset)
    denominator = length_1 * length_2 - projection * projection
    if denominator <= PARALLEL_EPSILON * length_1 * length_2:
        parameter_1 = 0.0
    else:
        parameter_1 = (projection * start_2 - start_1 * length_2) / denominator
        parameter_1 = min(max(parameter_1, 0.0), 1.0)
    parameter_2 = (projection * parameter_1 + start_2) / length_2
    if parameter_2 < 0.0:
        parameter_2 = 0.0
        parameter_1 = min(max(-start_1 / length_1, 0.0), 1.0)
    elif parameter_2 > 1.0:
        parameter_2 = 1.0
        parameter_1 = min(max((projection - start_1) / length_1, 0.0), 1.0)
    difference = (a_1 + parameter_1 * d_1) - (a_2 + parameter_2 * d_2)
    return _finite(float(np.linalg.norm(difference)), 'segment-segment')


def segment_segment_distance_batch(a_1, b_1, a_2, b_2) -> np.ndarray:
    """
    Run the clamped solve of :func:`segment_segment_distance` over many pairs.

    Every pair is independent, so this evaluates exactly the same arithmetic on
    exactly the same values as the scalar routine and returns the same doubles;
    it exists only so that one hundred and forty pairs cost one vector op rather
    than one hundred and forty interpreter round trips.
    """
    d_1 = b_1 - a_1
    d_2 = b_2 - a_2
    offset = a_1 - a_2
    length_1 = np.einsum('ij,ij->i', d_1, d_1)
    length_2 = np.einsum('ij,ij->i', d_2, d_2)
    projection = np.einsum('ij,ij->i', d_1, d_2)
    start_1 = np.einsum('ij,ij->i', d_1, offset)
    start_2 = np.einsum('ij,ij->i', d_2, offset)
    denominator = length_1 * length_2 - projection * projection
    singular = denominator <= PARALLEL_EPSILON * length_1 * length_2
    safe = np.where(singular, 1.0, denominator)
    parameter_1 = np.where(
        singular, 0.0,
        np.clip((projection * start_2 - start_1 * length_2) / safe, 0.0, 1.0))
    parameter_2 = (projection * parameter_1 + start_2) / length_2
    below = parameter_2 < 0.0
    above = parameter_2 > 1.0
    parameter_1 = np.where(below, np.clip(-start_1 / length_1, 0.0, 1.0), parameter_1)
    parameter_1 = np.where(
        above, np.clip((projection - start_1) / length_1, 0.0, 1.0), parameter_1)
    parameter_2 = np.clip(parameter_2, 0.0, 1.0)
    difference = ((a_1 + parameter_1[:, None] * d_1)
                  - (a_2 + parameter_2[:, None] * d_2))
    distances = np.sqrt(np.einsum('ij,ij->i', difference, difference))
    if not np.all(np.isfinite(distances)):
        raise GeometryError('batched segment-segment')
    return distances


def _box_signed_distance(point, half_extents) -> float:
    """Return the exact signed distance from a point to an axis-aligned box."""
    offset_x = abs(point[0]) - half_extents[0]
    offset_y = abs(point[1]) - half_extents[1]
    offset_z = abs(point[2]) - half_extents[2]
    outside_x = offset_x if offset_x > 0.0 else 0.0
    outside_y = offset_y if offset_y > 0.0 else 0.0
    outside_z = offset_z if offset_z > 0.0 else 0.0
    exterior = math.sqrt(outside_x * outside_x + outside_y * outside_y
                         + outside_z * outside_z)
    interior = max(offset_x, offset_y, offset_z)
    return exterior + (interior if interior < 0.0 else 0.0)


def _breakpoints(start, direction, half_extents) -> list:
    """Every parameter where the active region of the box distance can change."""
    values = [0.0, 1.0]

    def _crossing(numerator, denominator):
        if abs(denominator) <= 0.0:
            return
        parameter = numerator / denominator
        if 0.0 < parameter < 1.0:
            values.append(parameter)

    for axis in range(3):
        # Slab boundaries p_i(t) = +-h_i, and the sign change p_i(t) = 0.
        _crossing(half_extents[axis] - start[axis], direction[axis])
        _crossing(-half_extents[axis] - start[axis], direction[axis])
        _crossing(-start[axis], direction[axis])
    for first in range(3):
        for second in range(first + 1, 3):
            # h_i - |p_i| = h_j - |p_j| for each combination of the two signs.
            for sign_first in (1.0, -1.0):
                for sign_second in (1.0, -1.0):
                    numerator = (half_extents[first] - sign_first * start[first]
                                 - half_extents[second] + sign_second * start[second])
                    denominator = (sign_first * direction[first]
                                   - sign_second * direction[second])
                    _crossing(numerator, denominator)
    values.sort()
    unique = [values[0]]
    for value in values[1:]:
        if value - unique[-1] > BREAKPOINT_EPSILON:
            unique.append(value)
    return unique


def _at(start, direction, parameter):
    return (start[0] + parameter * direction[0],
            start[1] + parameter * direction[1],
            start[2] + parameter * direction[2])


def _minimise_on_interval(start, direction, half_extents, low, high) -> float:
    """Exact minimum of the box distance on one breakpoint-free subinterval."""
    point = _at(start, direction, 0.5 * (low + high))
    offset = [abs(point[axis]) - half_extents[axis] for axis in range(3)]
    signs = [1.0 if point[axis] >= 0.0 else -1.0 for axis in range(3)]
    if max(offset) > 0.0:
        # Outside: the active component set is constant on this subinterval, so
        # the squared exterior distance is a single quadratic in the parameter.
        coefficient_2 = 0.0
        coefficient_1 = 0.0
        for axis in range(3):
            if offset[axis] <= 0.0:
                continue
            slope = signs[axis] * direction[axis]
            constant = signs[axis] * start[axis] - half_extents[axis]
            coefficient_2 += slope * slope
            coefficient_1 += 2.0 * slope * constant
        candidates = [low, high]
        if coefficient_2 > 0.0:
            stationary = -coefficient_1 / (2.0 * coefficient_2)
            if low < stationary < high:
                candidates.append(stationary)
        return min(_box_signed_distance(_at(start, direction, value), half_extents)
                   for value in candidates)
    # Inside: the nearest-face index is constant, so the value is linear.
    return min(_box_signed_distance(_at(start, direction, value), half_extents)
               for value in (low, high))


def segment_box_distance(point_a, point_b, centre, rotation, half_extents) -> float:
    """
    Distance between the segment [a, b] and an oriented box.

    The box distance restricted to a line is convex but piecewise smooth; the
    minimisation is solved exactly by enumerating the parameters at which the
    active region changes and minimising one closed-form expression on each
    resulting subinterval.
    """
    transposed = rotation.T
    start = tuple(float(value) for value in transposed @ (point_a - centre))
    direction = tuple(float(value) for value in transposed @ (point_b - point_a))
    extents = tuple(float(value) for value in half_extents)
    values = _breakpoints(start, direction, extents)
    best = _box_signed_distance(start, extents)
    for index in range(len(values) - 1):
        best = min(best, _minimise_on_interval(
            start, direction, extents, values[index], values[index + 1]))
    best = min(best, _box_signed_distance(_at(start, direction, 1.0), extents))
    return _finite(best, 'segment-box')


def segment_halfspace_distance(point_a, point_b, normal, offset) -> float:
    """
    Distance from the segment [a, b] to the half-space {p : n.p < offset}.

    Negative when the segment reaches into the forbidden region.  The minimum of
    an affine function over a segment is attained at an endpoint, so no
    minimisation is needed.
    """
    value = min(float(normal @ point_a), float(normal @ point_b)) - offset
    return _finite(value, 'segment-halfspace')


def point_halfspace_distance(point, normal, offset) -> float:
    """Distance from a point to the half-space {p : n.p < offset}."""
    return _finite(float(normal @ point) - offset, 'point-halfspace')


def box_halfspace_distance(centre, rotation, half_extents, normal, offset) -> float:
    """Distance from an oriented box to the half-space {p : n.p < offset}."""
    support = sum(half_extents[axis] * abs(float(normal @ rotation[:, axis]))
                  for axis in range(3))
    return _finite(float(normal @ centre) - support - offset, 'box-halfspace')
