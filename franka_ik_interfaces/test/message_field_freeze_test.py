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

"""Freeze the complete public IK rosidl schema at its source boundary."""

import os
from pathlib import Path


SOURCE_DIR = Path(os.environ['FRANKA_IK_INTERFACES_SOURCE_DIR'])

EXPECTED_SCHEMA = {
    'msg/IkRequest.msg': (
        'string<=128 frame_id',
        'string<=64 arm_id',
        'uint8 TIP_FLANGE=0',
        'uint8 TIP_HAND_TCP=1',
        'uint8 tip_frame 0',
        'geometry_msgs/Pose target_pose',
        'float64[7] seed_positions',
        'uint8 REDUNDANCY_FROM_SEED=0',
        'uint8 REDUNDANCY_FIXED=1',
        'uint8 redundancy_mode 0',
        'float64 redundancy_value 0.0',
        'uint8 max_solutions 1',
        'uint8 SOLVER_DEFAULT=0',
        'uint8 SOLVER_ANALYTIC=1',
        'uint8 SOLVER_NUMERIC=2',
        'uint8 solver 0',
        'float64 position_tolerance 0.0',
        'float64 orientation_tolerance 0.0',
        'float64 joint_limit_margin 0.0',
    ),
    'msg/IkSolution.msg': (
        'float64[7] positions',
        'float64 redundancy_value',
        'float64 position_error',
        'float64 orientation_error',
        'float64 seed_distance',
        'uint8 BRANCH_NUMERIC=255',
        'uint8 branch',
    ),
    'msg/IkResult.msg': (
        'uint8 RESULT_SUCCESS=0',
        'uint8 RESULT_BAD_REQUEST=1',
        'uint8 RESULT_UNKNOWN_ARM=2',
        'uint8 RESULT_UNKNOWN_FRAME=3',
        'uint8 RESULT_UNSUPPORTED_TIP=4',
        'uint8 RESULT_SEED_OUT_OF_LIMITS=5',
        'uint8 RESULT_UNREACHABLE=6',
        'uint8 RESULT_LIMITS_VIOLATED=7',
        'uint8 RESULT_TOLERANCE_NOT_MET=8',
        'uint8 RESULT_ITERATION_BUDGET_EXHAUSTED=9',
        'uint8 RESULT_NO_ACCEPTABLE_SOLUTION=10',
        'uint8 RESULT_INTERNAL_ERROR=11',
        'uint8 result',
        'string<=256 message',
        'franka_ik_interfaces/IkSolution[<=4] solutions',
        'uint8 solver_used',
        'uint16 iterations',
        'builtin_interfaces/Duration solve_time',
    ),
    'msg/ChainInfo.msg': (
        'string<=64 arm_id',
        'string base_frame',
        'string flange_frame',
        'string hand_tcp_frame',
        'string[7] joint_names',
        'float64[7] position_lower',
        'float64[7] position_upper',
        'float64[7] velocity_limit',
        'geometry_msgs/Transform root_to_base',
    ),
    'srv/SolveIk.srv': (
        'franka_ik_interfaces/IkRequest request',
        '---',
        'franka_ik_interfaces/IkResult result',
    ),
    'srv/GetChainInfo.srv': (
        '---',
        'string urdf_root_frame',
        'string<=64 urdf_sha256',
        'franka_ik_interfaces/ChainInfo[<=4] chains',
        'uint8 default_solver',
        'bool analytic_backend_available',
        'string<=64 package_version',
    ),
}


def _semantic_lines(relative_path):
    """Return schema lines with whitespace normalized and comments removed."""
    lines = []
    for raw_line in (SOURCE_DIR / relative_path).read_text(encoding='utf-8').splitlines():
        semantic = raw_line.partition('#')[0].strip()
        if semantic:
            lines.append(' '.join(semantic.split()))
    return tuple(lines)


def test_exact_message_and_service_schema():
    """Reject any public field, type, bound, default, constant, or service edit."""
    actual_paths = {
        str(path.relative_to(SOURCE_DIR))
        for directory in ('msg', 'srv')
        for path in (SOURCE_DIR / directory).glob('*')
        if path.is_file()
    }
    assert actual_paths == set(EXPECTED_SCHEMA)

    for relative_path, expected in EXPECTED_SCHEMA.items():
        assert _semantic_lines(relative_path) == expected
