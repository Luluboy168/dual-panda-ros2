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
"""Unit coverage for single- and dual-arm launch input validation."""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from franka_bringup.launch_validation import validate_arm_ids, validate_single_arm_id
import pytest


class _UnitLaunchContext:
    """Minimal substitution context that avoids creating ROS log files in a unit test."""

    def __init__(self, launch_configurations):
        self.launch_configurations = launch_configurations

    def perform_substitution(self, substitution):
        return substitution.perform(self)


def _load_launch_module(file_name):
    launch_path = os.path.join(
        get_package_share_directory('franka_bringup'),
        'launch',
        'real',
        file_name,
    )
    module_name = '_test_{}'.format(file_name.replace('.', '_'))
    specification = importlib.util.spec_from_file_location(module_name, launch_path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


@pytest.mark.parametrize('arm_id', ['panda', 'panda1', 'Panda_2'])
def test_valid_arm_ids_are_accepted(arm_id):
    validate_arm_ids({'arm_id': arm_id})


@pytest.mark.parametrize(
    'arm_id',
    ['', ' ', '1panda', '_panda', 'panda-1', 'panda/1', 'panda.1', 'pandá'],
)
def test_malformed_arm_ids_are_rejected(arm_id):
    with pytest.raises(ValueError, match='must be a nonempty identifier'):
        validate_arm_ids({'arm_id': arm_id})


def test_duplicate_arm_ids_are_rejected():
    with pytest.raises(ValueError, match='must use unique arm IDs'):
        validate_arm_ids({'arm_id_1': 'panda1', 'arm_id_2': 'panda1'})


def test_single_arm_id_accepts_fixed_controller_prefix():
    validate_single_arm_id('panda')


@pytest.mark.parametrize('arm_id', ['panda1', 'Panda_2'])
def test_single_arm_id_rejects_unsupported_controller_prefix(arm_id):
    with pytest.raises(ValueError, match="must be 'panda'"):
        validate_single_arm_id(arm_id)


@pytest.mark.parametrize(
    'arm_id, error',
    [
        ('', 'must be a nonempty identifier'),
        ('panda-1', 'must be a nonempty identifier'),
        ('1panda', 'must be a nonempty identifier'),
        ('panda1', "must be 'panda'"),
    ],
)
def test_single_launch_rejects_bad_arm_id_during_opaque_setup(arm_id, error, monkeypatch):
    launch_module = _load_launch_module('franka.launch.py')
    context = _UnitLaunchContext({'arm_id': arm_id})
    monkeypatch.setattr(
        launch_module,
        'get_package_share_directory',
        lambda unused_name: pytest.fail('launch actions were constructed before validation'),
    )

    with pytest.raises(ValueError, match=error):
        launch_module._launch_setup(context)


@pytest.mark.parametrize(
    'arm_ids, error',
    [
        ({'arm_id_1': '', 'arm_id_2': 'panda2'}, 'must be a nonempty identifier'),
        ({'arm_id_1': 'panda1', 'arm_id_2': 'panda-2'}, 'must be a nonempty identifier'),
        ({'arm_id_1': 'panda1', 'arm_id_2': 'panda1'}, 'must use unique arm IDs'),
    ],
)
def test_dual_launch_rejects_bad_arm_ids_during_opaque_setup(arm_ids, error, monkeypatch):
    launch_module = _load_launch_module('dual_franka.launch.py')
    context = _UnitLaunchContext(arm_ids)
    monkeypatch.setattr(
        launch_module,
        'get_package_share_directory',
        lambda unused_name: pytest.fail('launch actions were constructed before validation'),
    )

    with pytest.raises(ValueError, match=error):
        launch_module._launch_setup(context)
