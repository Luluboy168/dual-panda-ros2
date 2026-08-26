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
"""Validation helpers for robot-identifying launch arguments."""

import re


_ARM_ID_PATTERN = re.compile(r'[A-Za-z][A-Za-z0-9_]*')


def validate_arm_ids(arm_ids):
    """Reject malformed or duplicate arm IDs before constructing launch actions."""
    seen_ids = {}
    for argument_name, arm_id in arm_ids.items():
        if not isinstance(arm_id, str) or _ARM_ID_PATTERN.fullmatch(arm_id) is None:
            raise ValueError(
                "Launch argument '{}' must be a nonempty identifier matching "
                "'[A-Za-z][A-Za-z0-9_]*'; got {!r}".format(argument_name, arm_id))

        if arm_id in seen_ids:
            raise ValueError(
                "Launch arguments '{}' and '{}' must use unique arm IDs; both are {!r}".format(
                    seen_ids[arm_id], argument_name, arm_id))
        seen_ids[arm_id] = argument_name


def validate_single_arm_id(arm_id):
    """Require the fixed arm ID used by the single-arm controller configuration."""
    validate_arm_ids({'arm_id': arm_id})
    if arm_id != 'panda':
        raise ValueError(
            'Launch argument {!r} must be {!r} because single_controllers.yaml uses the fixed '
            'panda joint prefix; got {!r}'.format('arm_id', 'panda', arm_id))


# The two production arm IDs the safe dual-arm controllers and the shared Panda limit policy
# (panda_joint_limits_v1.yaml) recognize, in one-arm mode as in two-arm mode.
ONE_ARM_MODE_IDS = ('panda1', 'panda2')


def validate_one_arm_mode_arm_id(arm_id):
    """Require one of the two production arm IDs recognized by one-arm mode."""
    validate_arm_ids({'arm_id': arm_id})
    if arm_id not in ONE_ARM_MODE_IDS:
        raise ValueError(
            "Launch argument 'arm_id' must be one of {!r} for one-arm mode; got {!r}".format(
                ONE_ARM_MODE_IDS, arm_id))
