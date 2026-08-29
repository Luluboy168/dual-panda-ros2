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

# [THROWAWAY] Prototype-server mirror; browser C1 checks move into franka_web.

"""Golden and controller-rule checks for frozen GhostApplyEvent Contract C1."""

from copy import deepcopy
import json
from pathlib import Path

from franka_ghost.dev_server import validate_apply_event
from franka_ghost.joint_source import URDF_LOWER, URDF_UPPER
import pytest


GOLDEN_PATH = Path(__file__).with_name('golden_ghost_apply_event.json')


@pytest.fixture
def golden_event() -> dict:
    """Load the deliberately reviewable, checked-in C1 golden event."""
    return json.loads(GOLDEN_PATH.read_text(encoding='utf-8'))


def test_golden_event_is_an_exact_valid_c1_snapshot(golden_event):
    assert list(golden_event) == [
        'schema',
        'arm_index',
        'arm_id',
        'joint_names',
        'positions',
        'fence',
        'measured_at_apply',
        'ghost_epoch',
    ]
    assert validate_apply_event(golden_event) == []
    assert golden_event['joint_names'] == [
        f"{golden_event['arm_id']}_joint{number}" for number in range(1, 8)
    ]


@pytest.mark.parametrize('arm_index', [1, 2])
def test_joint_names_match_controller_canonical_order(golden_event, arm_index):
    event = deepcopy(golden_event)
    event['arm_index'] = arm_index
    event['arm_id'] = f'panda{arm_index}'
    event['joint_names'] = [f'panda{arm_index}_joint{number}' for number in range(1, 8)]
    assert validate_apply_event(event) == []


@pytest.mark.parametrize(
    'field',
    [
        'velocities',
        'accelerations',
        'effort',
        'efforts',
        'time_from_start',
        'frame_id',
        'duration',
        'timestamp',
    ],
)
def test_c1_forbids_controller_command_and_timing_fields(golden_event, field):
    event = deepcopy(golden_event)
    event['nested'] = {field: []}
    errors = validate_apply_event(event)
    assert any(error.startswith('I7:') for error in errors), errors


def test_positions_are_seven_finite_and_inside_the_echoed_fence(golden_event):
    too_short = deepcopy(golden_event)
    too_short['positions'].pop()
    assert any(error.startswith('I2:') for error in validate_apply_event(too_short))

    nonfinite = deepcopy(golden_event)
    nonfinite['positions'][2] = float('nan')
    assert any(error.startswith('I2:') for error in validate_apply_event(nonfinite))

    outside = deepcopy(golden_event)
    outside['positions'][3] = outside['fence']['upper'][3] + 0.01
    assert any(error.startswith('I3:') for error in validate_apply_event(outside))


def test_session_fence_can_only_narrow_urdf_policy(golden_event):
    for index, (lower, upper) in enumerate(zip(URDF_LOWER, URDF_UPPER)):
        assert golden_event['fence']['lower'][index] >= lower
        assert golden_event['fence']['upper'][index] <= upper

    wider = deepcopy(golden_event)
    wider['fence']['upper'][6] = URDF_UPPER[6] + 0.001
    errors = validate_apply_event(wider)
    assert any(error.startswith('I4:') for error in errors), errors


def test_epoch_is_positive_and_strictly_mount_scoped(golden_event):
    assert validate_apply_event(golden_event, previous_epoch=0) == []
    errors = validate_apply_event(golden_event, previous_epoch=1)
    assert any(error.startswith('I6:') for error in errors), errors
