#!/usr/bin/env python3
# Copyright (c) 2026 The multipanda_ros2 contributors
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

"""Generate the deterministic, solver-independent Stage-0 Panda pose corpus."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import sys


SCHEMA_VERSION = 1
RANDOM_SEED = 0x50414E44415F494B
FK_TOLERANCE = 1.0e-12

POSITION_LOWER = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
POSITION_UPPER = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
VELOCITY_LIMIT = (2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61)

CATEGORY_COUNTS = {
    'reachable_random': 2000,
    'reachable_near_limit': 500,
    'singular': 200,
    'unreachable_far': 200,
    'unreachable_limits': 200,
    'drag_traces': 4000,
}
TRACE_COUNT = 20
TRACE_STEPS = 200

# URDF joint origins, parent link -> child link, in joint order. Each origin is
# (xyz, rpy), followed by a rotation about the origin frame's local z axis.
JOINT_ORIGINS = (
    ((0.0, 0.0, 0.333), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, -0.316, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((0.0825, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((-0.0825, 0.384, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((0.088, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
)
FLANGE_OFFSET = (0.0, 0.0, 0.107)

# The Panda Jacobian is rank-deficient for q5=0 at this joint-4 value. It is
# the stationary point of the shoulder-to-wrist distance
# D^2 = C + A*cos(q4) + B*sin(q4), using the literal URDF dimensions below.
ELBOW_COSINE_COEFFICIENT = 2.0 * (0.316 * 0.384 - 0.0825 ** 2)
ELBOW_SINE_COEFFICIENT = -2.0 * 0.0825 * (0.316 + 0.384)
SINGULAR_Q4 = math.atan2(ELBOW_SINE_COEFFICIENT, ELBOW_COSINE_COEFFICIENT)
SINGULAR_VALUE_TOLERANCE = 1.0e-12


def _identity():
    """Return a 4-by-4 identity transform."""
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _multiply(left, right):
    """Multiply two 4-by-4 matrices without a numerical-library dependency."""
    return [
        [sum(left[row][inner] * right[inner][column] for inner in range(4))
         for column in range(4)]
        for row in range(4)
    ]


def _translation(xyz):
    """Return a homogeneous translation transform."""
    result = _identity()
    result[0][3], result[1][3], result[2][3] = xyz
    return result


def _rotation_x(angle):
    """Return a homogeneous rotation about x."""
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, cosine, -sine, 0.0],
        [0.0, sine, cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_y(angle):
    """Return a homogeneous rotation about y."""
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [cosine, 0.0, sine, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-sine, 0.0, cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_z(angle):
    """Return a homogeneous rotation about z."""
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [cosine, -sine, 0.0, 0.0],
        [sine, cosine, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rpy(roll, pitch, yaw):
    """Return the URDF fixed-axis RPY transform Rz(yaw) Ry(pitch) Rx(roll)."""
    return _multiply(
        _rotation_z(yaw),
        _multiply(_rotation_y(pitch), _rotation_x(roll)),
    )


def _joint_origin(xyz, rpy):
    """Return one fixed URDF joint-origin transform."""
    return _multiply(_translation(xyz), _rpy(*rpy))


FIXED_JOINT_TRANSFORMS = tuple(_joint_origin(*origin) for origin in JOINT_ORIGINS)


def _fk_frames(joint_positions):
    """Compute link1..link8 transforms with independent pure-Python FK."""
    if len(joint_positions) != 7:
        raise ValueError('FK requires exactly seven joint positions')
    if not all(math.isfinite(value) for value in joint_positions):
        raise ValueError('FK joint positions must be finite')

    transform = _identity()
    frames = []
    for fixed, angle in zip(FIXED_JOINT_TRANSFORMS, joint_positions):
        transform = _multiply(transform, _multiply(fixed, _rotation_z(angle)))
        frames.append(transform)
    transform = _multiply(transform, _translation(FLANGE_OFFSET))
    frames.append(transform)
    return frames


def _quaternion_from_matrix(transform):
    """Convert a rotation matrix to a canonical xyzw unit quaternion."""
    m00, m01, m02 = transform[0][0], transform[0][1], transform[0][2]
    m10, m11, m12 = transform[1][0], transform[1][1], transform[1][2]
    m20, m21, m22 = transform[2][0], transform[2][1], transform[2][2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
        w = 0.25 * scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
        w = (m21 - m12) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
        w = (m02 - m20) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
        w = (m10 - m01) / scale

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    quaternion = [x / norm, y / norm, z / norm, w / norm]
    if quaternion[3] < 0.0:
        quaternion = [-value for value in quaternion]
    return quaternion


def _pose_from_transform(transform):
    """Return the corpus pose representation for a homogeneous transform."""
    return {
        'position': [transform[row][3] for row in range(3)],
        'orientation_xyzw': _quaternion_from_matrix(transform),
    }


def _transform_from_pose(pose):
    """Convert a corpus pose back to a homogeneous transform."""
    x, y, z, w = pose['orientation_xyzw']
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    result = [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), 0.0],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), 0.0],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    result[0][3], result[1][3], result[2][3] = pose['position']
    return result


def _position_distance(left, right):
    """Return translation distance between two transforms."""
    return math.sqrt(sum((left[index][3] - right[index][3]) ** 2 for index in range(3)))


def _orientation_distance(left, right):
    """Return the shortest orientation angle between two transforms."""
    relative_trace = sum(
        left[row][column] * right[row][column]
        for row in range(3)
        for column in range(3)
    )
    cosine = max(-1.0, min(1.0, (relative_trace - 1.0) / 2.0))
    return math.acos(cosine)


def _matrix_max_error(left, right):
    """Return the maximum elementwise error between two transforms."""
    return max(
        abs(left[row][column] - right[row][column])
        for row in range(4)
        for column in range(4)
    )


class PykdlCrossChecker:
    """Cross-check the independent FK against an Orocos KDL chain."""

    def __init__(self, required):
        """Build the KDL reference chain when PyKDL is installed."""
        self.available = False
        self.count = 0
        self.max_error = 0.0
        self._PyKDL = None
        self._chain = None
        self._solver = None
        self._jacobian_solver = None
        self._numpy = None
        try:
            import PyKDL  # pylint: disable=import-outside-toplevel
        except ImportError as error:
            if required:
                raise RuntimeError('PyKDL is required but unavailable') from error
            return

        chain = PyKDL.Chain()
        for index, (xyz, rpy) in enumerate(JOINT_ORIGINS):
            fixed = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))
            axis = fixed.M * PyKDL.Vector(0.0, 0.0, 1.0)
            joint = PyKDL.Joint(
                f'panda_joint{index + 1}',
                fixed.p,
                axis,
                PyKDL.Joint.RotAxis,
            )
            chain.addSegment(PyKDL.Segment(f'panda_link{index + 1}', joint, fixed))
        chain.addSegment(
            PyKDL.Segment(
                'panda_link8',
                PyKDL.Joint('panda_joint8', PyKDL.Joint.Fixed),
                PyKDL.Frame(PyKDL.Vector(*FLANGE_OFFSET)),
            )
        )
        self._PyKDL = PyKDL
        # ChainFkSolverPos_recursive stores a reference to its Chain. Keep the
        # Python wrapper alive for exactly as long as the solver.
        self._chain = chain
        self._solver = PyKDL.ChainFkSolverPos_recursive(chain)
        self._jacobian_solver = PyKDL.ChainJntToJacSolver(chain)
        try:
            import numpy  # pylint: disable=import-outside-toplevel
        except ImportError as error:
            if required:
                raise RuntimeError('NumPy is required for the singular-value check') from error
        else:
            self._numpy = numpy
        self.available = True

    def check(self, joint_positions, independent_transform):
        """Check one FK result and update the aggregate error statistics."""
        if not self.available:
            return
        joint_array = self._PyKDL.JntArray(7)
        for index, value in enumerate(joint_positions):
            joint_array[index] = value
        kdl_frame = self._PyKDL.Frame()
        result = self._solver.JntToCart(joint_array, kdl_frame)
        if result < 0:
            raise RuntimeError(f'PyKDL FK failed with code {result}')
        reference = _identity()
        for row in range(3):
            for column in range(3):
                reference[row][column] = kdl_frame.M[row, column]
            reference[row][3] = kdl_frame.p[row]
        error = _matrix_max_error(independent_transform, reference)
        self.count += 1
        self.max_error = max(self.max_error, error)
        if error > FK_TOLERANCE:
            raise RuntimeError(
                f'independent FK disagrees with PyKDL at check {self.count}: '
                f'{error:.17g} > {FK_TOLERANCE:.17g}'
            )

    def minimum_singular_value(self, joint_positions):
        """Return the smallest PyKDL Jacobian singular value, when available."""
        if not self.available or self._numpy is None:
            return None
        joint_array = self._PyKDL.JntArray(7)
        for index, value in enumerate(joint_positions):
            joint_array[index] = value
        jacobian = self._PyKDL.Jacobian(7)
        result = self._jacobian_solver.JntToJac(joint_array, jacobian)
        if result < 0:
            raise RuntimeError(f'PyKDL Jacobian failed with code {result}')
        matrix = self._numpy.array(
            [[jacobian[row, column] for column in range(7)] for row in range(6)]
        )
        return float(self._numpy.linalg.svd(matrix, compute_uv=False)[-1])


def _checked_pose(joint_positions, checker):
    """Compute a pose and cross-check that FK invocation."""
    transform = _fk_frames(joint_positions)[-1]
    checker.check(joint_positions, transform)
    return _pose_from_transform(transform)


def _uniform_joint_positions(rng, inset):
    """Sample all joints uniformly within an inset of their limits."""
    return [
        rng.uniform(lower + inset, upper - inset)
        for lower, upper in zip(POSITION_LOWER, POSITION_UPPER)
    ]


def _random_unit_vector(rng):
    """Sample a deterministic isotropic unit vector."""
    while True:
        vector = [rng.gauss(0.0, 1.0) for _ in range(3)]
        norm = math.sqrt(sum(value * value for value in vector))
        if norm > 1.0e-15:
            return [value / norm for value in vector]


def _entry(identifier, category, expected_result, seed_positions, target_pose, **extra):
    """Build one corpus entry."""
    result = {
        'id': identifier,
        'category': category,
        'expected_result': expected_result,
        'seed_positions': seed_positions,
        'target_pose': target_pose,
    }
    result.update(extra)
    return result


def _wrist_distance(joint_positions):
    """Return shoulder (joint 2) to wrist-centre (joint 6) distance."""
    frames = _fk_frames(joint_positions)
    shoulder = frames[1]
    wrist = frames[5]
    return _position_distance(shoulder, wrist)


def _folded_q4_geometry():
    """Derive the unconstrained q4 angle with minimum shoulder-wrist distance."""
    values = []
    for q4 in (0.0, math.pi / 2.0, math.pi):
        joints = [0.0] * 7
        joints[3] = q4
        values.append(_wrist_distance(joints) ** 2)
    constant = (values[0] + values[2]) / 2.0
    cosine_coefficient = (values[0] - values[2]) / 2.0
    sine_coefficient = values[1] - constant
    minimum_angle = math.atan2(sine_coefficient, cosine_coefficient) + math.pi
    minimum_angle %= 2.0 * math.pi

    endpoint_distances = []
    for q4 in (POSITION_LOWER[3], POSITION_UPPER[3]):
        joints = [0.0] * 7
        joints[3] = q4
        endpoint_distances.append(_wrist_distance(joints))
    minimum_valid_distance = min(endpoint_distances)
    return minimum_angle, minimum_valid_distance


def _generate_reachable_random(rng, checker):
    """Generate uniformly sampled reachable poses."""
    entries = []
    for index in range(CATEGORY_COUNTS['reachable_random']):
        joints = _uniform_joint_positions(rng, 0.02)
        entries.append(
            _entry(
                f'reachable_random_{index:04d}',
                'reachable_random',
                'RESULT_SUCCESS',
                joints,
                _checked_pose(joints, checker),
            )
        )
    return entries


def _generate_near_limit(rng, checker):
    """Generate reachable poses with one to three joints near a stop."""
    entries = []
    for index in range(CATEGORY_COUNTS['reachable_near_limit']):
        joints = _uniform_joint_positions(rng, 0.02)
        selected = sorted(rng.sample(range(7), rng.randint(1, 3)))
        sides = []
        for joint_index in selected:
            side = 'lower' if rng.randrange(2) == 0 else 'upper'
            distance = rng.uniform(1.0e-6, 0.01)
            if side == 'lower':
                joints[joint_index] = POSITION_LOWER[joint_index] + distance
            else:
                joints[joint_index] = POSITION_UPPER[joint_index] - distance
            sides.append(side)
        entries.append(
            _entry(
                f'reachable_near_limit_{index:04d}',
                'reachable_near_limit',
                'RESULT_SUCCESS',
                joints,
                _checked_pose(joints, checker),
                near_limit_joints=[joint_index + 1 for joint_index in selected],
                near_limit_sides=sides,
            )
        )
    return entries


def _generate_singular(rng, checker):
    """Generate reachable poses on a rank-deficient elbow family."""
    entries = []
    for index in range(CATEGORY_COUNTS['singular']):
        joints = _uniform_joint_positions(rng, 0.02)
        joints[3] = SINGULAR_Q4
        joints[4] = 0.0
        minimum_singular_value = checker.minimum_singular_value(joints)
        if (
            minimum_singular_value is not None
            and minimum_singular_value > SINGULAR_VALUE_TOLERANCE
        ):
            raise RuntimeError(
                'singular-family Jacobian is full rank: '
                f'{minimum_singular_value:.17g} > {SINGULAR_VALUE_TOLERANCE:.17g}'
            )
        entries.append(
            _entry(
                f'singular_{index:04d}',
                'singular',
                'RESULT_SUCCESS',
                joints,
                _checked_pose(joints, checker),
                singularity='rank_deficient_q4_q5_family',
                minimum_jacobian_singular_value=minimum_singular_value,
            )
        )
    return entries


def _generate_unreachable_far(rng, checker):
    """Generate poses translated 1.5 m beyond the arm's full path length."""
    entries = []
    shoulder = (0.0, 0.0, 0.333)
    maximum_flange_distance = 0.316 + 0.0825 + math.hypot(0.0825, 0.384) + 0.088 + 0.107
    for index in range(CATEGORY_COUNTS['unreachable_far']):
        joints = _uniform_joint_positions(rng, 0.02)
        source_pose = _checked_pose(joints, checker)
        while True:
            axis = _random_unit_vector(rng)
            target_pose = {
                'position': [
                    source_pose['position'][dimension] + 1.5 * axis[dimension]
                    for dimension in range(3)
                ],
                'orientation_xyzw': source_pose['orientation_xyzw'],
            }
            shoulder_distance = math.sqrt(
                sum(
                    (target_pose['position'][dimension] - shoulder[dimension]) ** 2
                    for dimension in range(3)
                )
            )
            if shoulder_distance > maximum_flange_distance + 0.05:
                break
        entries.append(
            _entry(
                f'unreachable_far_{index:04d}',
                'unreachable_far',
                'RESULT_UNREACHABLE',
                joints,
                target_pose,
                translation_axis=axis,
                translation_distance=1.5,
            )
        )
    return entries


def _generate_unreachable_limits(rng, checker):
    """Generate inside-sphere poses excluded by Panda joint-4 limits."""
    entries = []
    folded_q4, minimum_valid_distance = _folded_q4_geometry()
    for index in range(CATEGORY_COUNTS['unreachable_limits']):
        construction_joints = _uniform_joint_positions(rng, 0.02)
        construction_joints[3] = folded_q4 + rng.uniform(-0.01, 0.01)
        target_pose = _checked_pose(construction_joints, checker)
        wrist_distance = _wrist_distance(construction_joints)
        if not wrist_distance < minimum_valid_distance - 0.10:
            raise RuntimeError('limits-only construction lacks its required q4 distance margin')

        seed_positions = list(construction_joints)
        seed_positions[3] = POSITION_LOWER[3] + 0.02
        entries.append(
            _entry(
                f'unreachable_limits_{index:04d}',
                'unreachable_limits',
                'RESULT_LIMITS_VIOLATED',
                seed_positions,
                target_pose,
                fixed_redundancy_value=seed_positions[6],
                construction_joint_positions=construction_joints,
                reflected_joint4=2.0 * SINGULAR_Q4 - construction_joints[3],
                shoulder_to_wrist_distance=wrist_distance,
                minimum_distance_with_valid_q4=minimum_valid_distance,
            )
        )
    return entries


def _trace_candidate(centre, amplitudes, frequencies, phases):
    """Build one smooth joint-space trace and its pure-Python FK poses."""
    joints_sequence = []
    transforms = []
    denominator = TRACE_STEPS - 1
    for step in range(TRACE_STEPS):
        progress = 2.0 * math.pi * step / denominator
        joints = [
            centre[index]
            + amplitudes[index]
            * math.sin(frequencies[index] * progress + phases[index])
            for index in range(7)
        ]
        joints_sequence.append(joints)
        transforms.append(_fk_frames(joints)[-1])
    return joints_sequence, transforms


def _trace_step_maxima(transforms):
    """Return maximum Cartesian translation and rotation between adjacent poses."""
    maximum_position = 0.0
    maximum_orientation = 0.0
    for previous, current in zip(transforms, transforms[1:]):
        maximum_position = max(maximum_position, _position_distance(previous, current))
        maximum_orientation = max(
            maximum_orientation,
            _orientation_distance(previous, current),
        )
    return maximum_position, maximum_orientation


def _generate_drag_traces(rng, checker):
    """Generate twenty smooth, limit-respecting Cartesian drag traces."""
    traces = []
    overall_position = 0.0
    overall_orientation = 0.0
    for trace_index in range(TRACE_COUNT):
        centre = _uniform_joint_positions(rng, 0.25)
        amplitudes = [rng.uniform(0.015, 0.035) for _ in range(7)]
        # REDUNDANCY_FROM_SEED fixes q7. Holding it constant gives every
        # adjacent target a known solution at the previous seed's q7.
        amplitudes[6] = 0.0
        frequencies = [rng.choice((1, 1, 1, 2)) for _ in range(7)]
        phases = [rng.uniform(-math.pi, math.pi) for _ in range(7)]

        for _ in range(8):
            joints_sequence, transforms = _trace_candidate(
                centre,
                amplitudes,
                frequencies,
                phases,
            )
            maximum_position, maximum_orientation = _trace_step_maxima(transforms)
            position_scale = 0.0018 / maximum_position if maximum_position > 0.0018 else 1.0
            orientation_limit = math.radians(0.9)
            orientation_scale = (
                orientation_limit / maximum_orientation
                if maximum_orientation > orientation_limit
                else 1.0
            )
            scale = min(1.0, position_scale, orientation_scale)
            if scale == 1.0:
                break
            amplitudes = [amplitude * scale * 0.98 for amplitude in amplitudes]
        else:
            raise RuntimeError('failed to bound a drag trace')

        steps = []
        for step_index, joints in enumerate(joints_sequence):
            steps.append(
                _entry(
                    f'drag_trace_{trace_index:02d}_step_{step_index:03d}',
                    'drag_traces',
                    'RESULT_SUCCESS',
                    joints,
                    _checked_pose(joints, checker),
                    step=step_index,
                )
            )
        traces.append(
            {
                'id': f'drag_trace_{trace_index:02d}',
                'maximum_step_position': maximum_position,
                'maximum_step_orientation': maximum_orientation,
                'steps': steps,
            }
        )
        overall_position = max(overall_position, maximum_position)
        overall_orientation = max(overall_orientation, maximum_orientation)
    return traces, overall_position, overall_orientation


def _validate_seed(seed_positions):
    """Assert that a stored seed is finite and inside all Panda position limits."""
    if len(seed_positions) != 7 or not all(math.isfinite(value) for value in seed_positions):
        raise RuntimeError('stored seed must contain seven finite values')
    for index, (value, lower, upper) in enumerate(
        zip(seed_positions, POSITION_LOWER, POSITION_UPPER)
    ):
        if not lower <= value <= upper:
            raise RuntimeError(
                f'stored joint {index + 1} value {value} '
                f'is outside [{lower}, {upper}]'
            )


def _validate_pose(pose):
    """Assert that a stored pose is finite and has a unit quaternion."""
    values = pose['position'] + pose['orientation_xyzw']
    if len(pose['position']) != 3 or len(pose['orientation_xyzw']) != 4:
        raise RuntimeError('stored pose has the wrong dimensions')
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError('stored pose contains a non-finite value')
    quaternion_norm = math.sqrt(sum(value * value for value in pose['orientation_xyzw']))
    if abs(quaternion_norm - 1.0) > 2.0e-15:
        raise RuntimeError(f'stored quaternion norm is {quaternion_norm}')


def _validate_corpus(corpus):
    """Validate counts, joint fences, pose shape, and trace Cartesian bounds."""
    singles = corpus['single_poses']
    traces = corpus['drag_traces']
    if len(singles) != 3100 or len(traces) != TRACE_COUNT:
        raise RuntimeError('corpus top-level counts are wrong')

    observed_counts = {key: 0 for key in CATEGORY_COUNTS}
    all_entries = list(singles)
    for trace in traces:
        if len(trace['steps']) != TRACE_STEPS:
            raise RuntimeError(f"{trace['id']} does not contain {TRACE_STEPS} steps")
        transforms = [_transform_from_pose(step['target_pose']) for step in trace['steps']]
        maximum_position, maximum_orientation = _trace_step_maxima(transforms)
        if maximum_position > 0.002 + 1.0e-12:
            raise RuntimeError(f"{trace['id']} exceeds the 2 mm step limit")
        if maximum_orientation > math.radians(1.0) + 1.0e-12:
            raise RuntimeError(f"{trace['id']} exceeds the 1 degree step limit")
        for previous, current in zip(trace['steps'], trace['steps'][1:]):
            maximum_joint_step = max(
                abs(after - before)
                for before, after in zip(
                    previous['seed_positions'], current['seed_positions']
                )
            )
            if maximum_joint_step > 0.15:
                raise RuntimeError(f"{trace['id']} exceeds the 0.15 rad joint-step limit")
            if current['seed_positions'][6] != previous['seed_positions'][6]:
                raise RuntimeError(f"{trace['id']} changes its fixed q7 redundancy value")
        all_entries.extend(trace['steps'])

    if len(all_entries) != 7100:
        raise RuntimeError(f'expected 7100 entries, found {len(all_entries)}')
    for entry in all_entries:
        observed_counts[entry['category']] += 1
        _validate_seed(entry['seed_positions'])
        _validate_pose(entry['target_pose'])
    if observed_counts != CATEGORY_COUNTS:
        raise RuntimeError(f'category counts differ: {observed_counts}')
    if corpus['counts']['total_entries'] != len(all_entries):
        raise RuntimeError('metadata total does not match corpus entries')


def _generate(require_pykdl):
    """Generate and validate the complete corpus."""
    rng = random.Random(RANDOM_SEED)
    checker = PykdlCrossChecker(require_pykdl)
    singles = []
    singles.extend(_generate_reachable_random(rng, checker))
    singles.extend(_generate_near_limit(rng, checker))
    singles.extend(_generate_singular(rng, checker))
    singles.extend(_generate_unreachable_far(rng, checker))
    singles.extend(_generate_unreachable_limits(rng, checker))
    traces, maximum_step_position, maximum_step_orientation = _generate_drag_traces(
        rng,
        checker,
    )
    corpus = {
        'schema_version': SCHEMA_VERSION,
        'generator': {
            'random_seed': RANDOM_SEED,
            'random_seed_hex': f'0x{RANDOM_SEED:016x}',
            'fk_oracle': 'pure_python_urdf_origin_matrix_chain',
            'tip_frame': 'panda_link8',
            'base_frame': 'panda_link0',
        },
        'joint_limits': {
            'position_lower': POSITION_LOWER,
            'position_upper': POSITION_UPPER,
            'velocity': VELOCITY_LIMIT,
        },
        'counts': {
            **CATEGORY_COUNTS,
            'single_poses': 3100,
            'trace_count': TRACE_COUNT,
            'trace_steps': TRACE_STEPS,
            'total_entries': 7100,
        },
        'trace_bounds': {
            'maximum_allowed_step_position': 0.002,
            'maximum_allowed_step_orientation': math.radians(1.0),
            'observed_maximum_step_position': maximum_step_position,
            'observed_maximum_step_orientation': maximum_step_orientation,
        },
        'single_poses': singles,
        'drag_traces': traces,
    }
    _validate_corpus(corpus)
    if checker.available and checker.count != 7100:
        raise RuntimeError(f'expected 7100 PyKDL checks, performed {checker.count}')
    return corpus, checker


def _corpus_bytes(corpus):
    """Serialize the corpus in one deterministic JSON representation."""
    return (json.dumps(corpus, indent=2, sort_keys=True, allow_nan=False) + '\n').encode('utf-8')


def _checksum_bytes(corpus_bytes):
    """Return the sha256sum-compatible checksum-file bytes."""
    digest = hashlib.sha256(corpus_bytes).hexdigest()
    return f'{digest}  corpus_v1.json\n'.encode('ascii')


def _parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path(__file__).resolve().parent,
        help='directory for corpus_v1.json and corpus_v1.sha256',
    )
    parser.add_argument(
        '--check',
        action='store_true',
        help='verify checked-in files instead of replacing them',
    )
    parser.add_argument(
        '--require-pykdl',
        action='store_true',
        help='fail instead of skipping the independent PyKDL cross-check when unavailable',
    )
    return parser.parse_args()


def main():
    """Generate or verify the checked-in corpus and checksum."""
    arguments = _parse_arguments()
    corpus, checker = _generate(arguments.require_pykdl)
    corpus_bytes = _corpus_bytes(corpus)
    checksum_bytes = _checksum_bytes(corpus_bytes)
    corpus_path = arguments.output_dir / 'corpus_v1.json'
    checksum_path = arguments.output_dir / 'corpus_v1.sha256'

    if arguments.check:
        if not corpus_path.is_file() or not checksum_path.is_file():
            raise RuntimeError('checked-in corpus or checksum file is missing')
        if corpus_path.read_bytes() != corpus_bytes:
            raise RuntimeError('corpus_v1.json is not reproducible from the fixed seed')
        if checksum_path.read_bytes() != checksum_bytes:
            raise RuntimeError('corpus_v1.sha256 does not match the generated corpus')
        action = 'verified'
    else:
        arguments.output_dir.mkdir(parents=True, exist_ok=True)
        corpus_path.write_bytes(corpus_bytes)
        checksum_path.write_bytes(checksum_bytes)
        action = 'wrote'

    digest = hashlib.sha256(corpus_bytes).hexdigest()
    if checker.available:
        cross_check = (
            f'PyKDL checks={checker.count}, max element error={checker.max_error:.3e}'
        )
    else:
        cross_check = 'PyKDL unavailable; cross-check skipped'
    print(f'{action} {corpus_path} ({len(corpus_bytes)} bytes)')
    print(f'sha256={digest}')
    print('entries=7100 (single=3100, drag_steps=4000)')
    print(cross_check)


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        raise SystemExit(1) from error
