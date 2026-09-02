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
A forward-kinematics oracle over a parsed URDF, for the live IK gate.

The gate needs a target pose it did not get from the thing under test, so
this computes one from the generated description instead. Nothing here is
hard-coded: every origin, every rotation and every joint AXIS is read out of
the file. Reading the axis rather than assuming +Z is what makes this an
independent implementation of the chain and not a half-copy of one -- the
renderer's own kinematics reads it, and an oracle that assumed it would
agree with the renderer for the wrong reason.
"""

import math
import xml.etree.ElementTree as ElementTree

IDENTITY = (1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0)


def multiply(left, right):
    """Return the product of two row-major 4x4 matrices."""
    return tuple(
        sum(left[row * 4 + k] * right[k * 4 + column] for k in range(4))
        for row in range(4)
        for column in range(4))


def translation(matrix):
    """Return the translation column of a 4x4 matrix."""
    return (matrix[3], matrix[7], matrix[11])


def quaternion(matrix):
    """Return (x, y, z, w) for the rotation part of a 4x4 matrix."""
    m00, m01, m02 = matrix[0], matrix[1], matrix[2]
    m10, m11, m12 = matrix[4], matrix[5], matrix[6]
    m20, m21, m22 = matrix[8], matrix[9], matrix[10]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / scale, (m02 - m20) / scale,
                (m10 - m01) / scale, 0.25 * scale)
    if m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale,
                (m21 - m12) / scale)
    if m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale,
                (m02 - m20) / scale)
    scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale,
            (m10 - m01) / scale)


def angle_between(first, second):
    """Return the rotation angle in radians between two unit quaternions."""
    dot = abs(sum(a * b for a, b in zip(first, second)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _rpy(roll, pitch, yaw):
    """Return the URDF's fixed-axis roll-pitch-yaw rotation as a 4x4."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0,
            -sp, cp * sr, cp * cr, 0.0,
            0.0, 0.0, 0.0, 1.0)


def _origin(element):
    """Return one <origin> as a 4x4, defaulting to the identity."""
    if element is None:
        return IDENTITY
    xyz = [float(value) for value in element.attrib.get('xyz', '0 0 0').split()]
    rpy = [float(value) for value in element.attrib.get('rpy', '0 0 0').split()]
    matrix = list(_rpy(*rpy))
    matrix[3], matrix[7], matrix[11] = xyz
    return tuple(matrix)


def _axis_rotation(axis, angle):
    """Return the rotation of ``angle`` about ``axis`` as a 4x4."""
    length = math.sqrt(sum(value * value for value in axis))
    x, y, z = (value / length for value in axis)
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return (t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0,
            t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0,
            t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0,
            0.0, 0.0, 0.0, 1.0)


class Chain:
    """The joints of a parsed URDF, indexed by their child link."""

    def __init__(self, path):
        """Parse one URDF file."""
        root = ElementTree.parse(str(path)).getroot()
        self.joints = {}
        # Direct children only: ros2_control blocks carry their own <joint>
        # elements, which name a joint but describe no link at all.
        for joint in root.findall('joint'):
            child = joint.find('child').attrib['link']
            axis = joint.find('axis')
            self.joints[child] = {
                'name': joint.attrib['name'],
                'type': joint.attrib.get('type', 'fixed'),
                'parent': joint.find('parent').attrib['link'],
                'origin': _origin(joint.find('origin')),
                'axis': ([float(value)
                          for value in axis.attrib.get('xyz', '0 0 1').split()]
                         if axis is not None else [0.0, 0.0, 1.0]),
            }

    def path_to(self, base_link, tip_link):
        """Return the joints from ``base_link`` to ``tip_link``, base first."""
        chain = []
        link = tip_link
        while link != base_link:
            joint = self.joints.get(link)
            if joint is None:
                raise ValueError(
                    '{} is not reachable from {}'.format(tip_link, base_link))
            chain.append(joint)
            link = joint['parent']
        chain.reverse()
        return chain

    def forward(self, base_link, tip_link, values):
        """
        Return the 4x4 pose of ``tip_link`` in ``base_link``.

        ``values`` maps joint NAME to angle; a joint the caller did not name
        is held at zero, and a fixed joint ignores any value given for it.
        """
        matrix = IDENTITY
        for joint in self.path_to(base_link, tip_link):
            matrix = multiply(matrix, joint['origin'])
            if joint['type'] in ('revolute', 'continuous'):
                angle = float(values.get(joint['name'], 0.0))
                matrix = multiply(matrix, _axis_rotation(joint['axis'], angle))
        return matrix

    def arm_pose(self, arm_id, positions, tip='link8'):
        """Return the pose of one arm's tip in that arm's own base frame."""
        values = {'{}_joint{}'.format(arm_id, index + 1): value
                  for index, value in enumerate(positions)}
        return self.forward('{}_link0'.format(arm_id),
                            '{}_{}'.format(arm_id, tip), values)
