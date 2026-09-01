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

"""The four primitives, against brute force and against their degenerate cases."""

import math

from franka_workspace_model.geometry import (
    box_halfspace_distance, GeometryError, point_halfspace_distance,
    rotation_from_rpy, segment_box_distance, segment_halfspace_distance,
    segment_point_distance, segment_segment_distance,
    segment_segment_distance_batch)

import numpy as np

import pytest


GENERATOR = np.random.default_rng(20260905)
SAMPLES = 400


def _brute_force_segment_segment(a_1, b_1, a_2, b_2, count=600):
    parameters = np.linspace(0.0, 1.0, count)
    first = a_1[None, :] + parameters[:, None] * (b_1 - a_1)[None, :]
    second = a_2[None, :] + parameters[:, None] * (b_2 - a_2)[None, :]
    difference = first[:, None, :] - second[None, :, :]
    return float(np.sqrt((difference * difference).sum(axis=2)).min())


def test_segment_segment_matches_brute_force():
    for _ in range(SAMPLES):
        points = GENERATOR.uniform(-1.0, 1.0, size=(4, 3))
        exact = segment_segment_distance(*points)
        sampled = _brute_force_segment_segment(*points)
        assert exact <= sampled + 1e-12
        assert sampled - exact < 5e-3


def test_the_batched_solve_is_the_scalar_one():
    """
    The batch exists for speed, not for a second opinion.

    It evaluates the same expressions on the same values; numpy's reduction
    order for a three-element dot product can differ from the scalar path's by
    one unit in the last place, which is why this asserts agreement to 1e-15
    rather than bit equality.  The checker uses the batch path for every pair it
    evaluates, so its own answers stay bit-reproducible run to run.
    """
    points = GENERATOR.uniform(-1.0, 1.0, size=(SAMPLES, 4, 3))
    batched = segment_segment_distance_batch(
        points[:, 0], points[:, 1], points[:, 2], points[:, 3])
    for index in range(SAMPLES):
        scalar = segment_segment_distance(*points[index])
        assert abs(batched[index] - scalar) <= 1e-15


def test_parallel_segments_are_answered_deterministically():
    a_1 = np.array([0.0, 0.0, 0.0])
    b_1 = np.array([1.0, 0.0, 0.0])
    a_2 = np.array([0.0, 0.5, 0.0])
    b_2 = np.array([1.0, 0.5, 0.0])
    assert abs(segment_segment_distance(a_1, b_1, a_2, b_2) - 0.5) < 1e-12
    # Reversing either segment is the same solid and must give the same number.
    assert (segment_segment_distance(a_1, b_1, a_2, b_2)
            == segment_segment_distance(b_1, a_1, b_2, a_2))


def test_collinear_and_overlapping_segments_give_zero():
    a_1 = np.array([0.0, 0.0, 0.0])
    b_1 = np.array([1.0, 0.0, 0.0])
    a_2 = np.array([0.5, 0.0, 0.0])
    b_2 = np.array([1.5, 0.0, 0.0])
    assert segment_segment_distance(a_1, b_1, a_2, b_2) == 0.0


def test_crossing_segments_give_zero():
    assert segment_segment_distance(
        np.array([-1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]),
        np.array([0.0, -1.0, 0.0]), np.array([0.0, 1.0, 0.0])) == 0.0


def test_a_degenerate_segment_becomes_a_point():
    point = np.array([0.3, 0.4, 0.0])
    assert abs(segment_segment_distance(point, point, np.array([0.0, 0.0, 0.0]),
                                        np.array([0.0, 0.0, 1.0])) - 0.5) < 1e-12
    assert segment_point_distance(point, point, point) == 0.0


def test_a_point_on_the_axis_needs_no_tie_break():
    assert segment_point_distance(np.array([0.0, 0.0, -1.0]),
                                  np.array([0.0, 0.0, 1.0]),
                                  np.array([0.0, 0.0, 0.25])) == 0.0


def _brute_force_segment_box(point_a, point_b, centre, rotation, half, count=4000):
    parameters = np.linspace(0.0, 1.0, count)
    points = point_a[None, :] + parameters[:, None] * (point_b - point_a)[None, :]
    local = (points - centre[None, :]) @ rotation
    offset = np.abs(local) - half[None, :]
    outside = np.linalg.norm(np.maximum(offset, 0.0), axis=1)
    inside = np.minimum(offset.max(axis=1), 0.0)
    return float((outside + inside).min())


def test_segment_box_matches_brute_force_including_the_interior():
    for _ in range(200):
        centre = GENERATOR.uniform(-0.5, 0.5, size=3)
        half = GENERATOR.uniform(0.05, 0.4, size=3)
        rotation = rotation_from_rpy(*GENERATOR.uniform(-math.pi, math.pi, size=3))
        point_a = GENERATOR.uniform(-1.2, 1.2, size=3)
        point_b = GENERATOR.uniform(-1.2, 1.2, size=3)
        exact = segment_box_distance(point_a, point_b, centre, rotation, half)
        sampled = _brute_force_segment_box(point_a, point_b, centre, rotation, half)
        assert exact <= sampled + 1e-9
        assert sampled - exact < 2e-3


def test_a_segment_parallel_to_a_face_is_handled():
    centre = np.zeros(3)
    half = np.array([0.1, 0.1, 0.1])
    rotation = np.eye(3)
    value = segment_box_distance(np.array([-1.0, 0.2, 0.0]),
                                 np.array([1.0, 0.2, 0.0]), centre, rotation, half)
    assert abs(value - 0.1) < 1e-12


def test_a_segment_inside_the_box_reports_the_deepest_negative_face_distance():
    """Inside, the primitive reports the worst point of the segment, not the best."""
    centre = np.zeros(3)
    half = np.array([0.5, 0.5, 0.5])
    value = segment_box_distance(np.array([0.0, 0.0, 0.0]),
                                 np.array([0.1, 0.0, 0.0]), centre, np.eye(3), half)
    assert abs(value + 0.5) < 1e-12


def test_an_endpoint_exactly_on_a_face_plane_is_zero():
    value = segment_box_distance(np.array([0.5, 0.0, 0.0]),
                                 np.array([1.5, 0.0, 0.0]), np.zeros(3), np.eye(3),
                                 np.array([0.5, 0.5, 0.5]))
    assert abs(value) < 1e-12


def test_the_half_space_primitives_agree_with_their_definitions():
    normal = np.array([0.0, 0.0, 1.0])
    assert abs(point_halfspace_distance(np.array([0.0, 0.0, 0.25]), normal, 0.0)
               - 0.25) < 1e-12
    assert abs(segment_halfspace_distance(np.array([0.0, 0.0, 0.25]),
                                          np.array([0.0, 0.0, -0.1]), normal, 0.0)
               + 0.1) < 1e-12
    rotation = rotation_from_rpy(0.0, 0.0, math.pi / 4)
    value = box_halfspace_distance(np.array([0.0, 0.0, 1.0]), rotation,
                                   np.array([0.5, 0.5, 0.5]), normal, 0.0)
    assert abs(value - 0.5) < 1e-12


def test_a_box_axis_perpendicular_to_the_normal_contributes_nothing():
    value = box_halfspace_distance(np.array([0.0, 0.0, 1.0]), np.eye(3),
                                   np.array([5.0, 5.0, 0.25]),
                                   np.array([0.0, 0.0, 1.0]), 0.0)
    assert abs(value - 0.75) < 1e-12


def test_a_non_finite_input_raises_rather_than_propagating():
    bad = np.array([float('nan'), 0.0, 0.0])
    with pytest.raises(GeometryError):
        segment_segment_distance(bad, np.array([1.0, 0.0, 0.0]),
                                 np.array([0.0, 1.0, 0.0]), np.array([1.0, 1.0, 0.0]))
    with pytest.raises(GeometryError):
        segment_point_distance(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), bad)
    with pytest.raises(GeometryError):
        segment_halfspace_distance(bad, bad, np.array([0.0, 0.0, 1.0]), 0.0)


def test_the_rotation_convention_is_fixed_axis_roll_pitch_yaw():
    """The link8 cylinder's axis resolves to the link frame's exact -y."""
    rotation = rotation_from_rpy(math.pi, math.pi / 2, math.pi / 2)
    axis = rotation @ np.array([0.0, 0.0, 1.0])
    assert float(np.abs(axis - np.array([0.0, -1.0, 0.0])).max()) < 1e-15
