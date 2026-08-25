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

import os
from pathlib import Path

from franka_bringup import controller_config_validator as validator
import pytest
import yaml


_SOURCE_ROOT = Path(__file__).resolve().parents[2]
_POLICY = _SOURCE_ROOT / 'franka_example_controllers' / 'config' / (
    'panda_joint_limits_v1.yaml')
_EFFORT = [10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 3.0]
_K_GAINS = [20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 5.0]
_D_GAINS = [1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.25]


@pytest.fixture(autouse=True)
def _use_source_limit_policy(monkeypatch):
    monkeypatch.setattr(validator, 'default_limit_policy_path', lambda: _POLICY)


def _joints(arm_id):
    return ['{}_joint{}'.format(arm_id, index) for index in range(1, 8)]


def _configuration(controller_name):
    arms = {}
    for slot, arm_id in enumerate(('panda1', 'panda2'), start=1):
        common = {
            'arm_id': arm_id,
            'k_gains': list(_K_GAINS),
            'd_gains': list(_D_GAINS),
            'max_effort': list(_EFFORT),
        }
        if controller_name == 'dual_arm_joint_velocity_controller':
            common = {
                'arm_id': arm_id,
                'joint_names': _joints(arm_id),
                'max_velocity': [0.1] * 7,
                'max_acceleration': [1.0] * 7,
            }
        elif controller_name == 'dual_arm_joint_impedance_controller':
            common.update({
                'joint_names': _joints(arm_id),
                'position_lower':
                    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
                'position_upper':
                    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
                'max_target_velocity': [1.0] * 7,
            })
        arms['arm_{}'.format(slot)] = common
    if controller_name.endswith(('velocity_controller', 'impedance_controller')):
        timings = {
            'watchdog_timeout': 0.02 if controller_name.endswith('velocity_controller') else 0.1,
            'max_header_age': 1.0,
            'future_tolerance': 0.1,
        }
        arms = dict(timings, **arms)
    return {'/' + controller_name: {'ros__parameters': arms}}


def _text(controller_name):
    return yaml.safe_dump(_configuration(controller_name), sort_keys=False)


@pytest.mark.parametrize('controller_name', sorted(validator.REVIEWED_CONTROLLERS))
def test_accepts_only_complete_reviewed_configurations(controller_name):
    result = validator.validate_controller_config_text(_text(controller_name), controller_name)
    assert result.controller_name == controller_name
    assert result.controller_type == validator.REVIEWED_CONTROLLERS[controller_name]
    assert len(result.command_interfaces) == 14
    assert len(result.state_interfaces) == (0 if controller_name.endswith(
        'velocity_controller') else 32)
    assert result.as_dict() == result.as_dict()


def test_output_json_is_stable_and_sorted(tmp_path, capsys):
    path = tmp_path / 'hold.yaml'
    path.write_text(_text('dual_arm_joint_hold_controller'), encoding='utf-8')
    assert validator.main([
        str(path), '--controller-name', 'dual_arm_joint_hold_controller']) == 0
    first = capsys.readouterr().out
    assert validator.main([
        str(path), '--controller-name', 'dual_arm_joint_hold_controller']) == 0
    assert capsys.readouterr().out == first
    assert first.startswith('{"command_interfaces":')


def test_unknown_controller_cli_error_is_stable_json(tmp_path, capsys):
    path = tmp_path / 'unknown.yaml'
    path.write_text('/legacy:\n  ros__parameters: {}\n', encoding='utf-8')
    assert validator.main([str(path), '--controller-name', 'legacy']) == 2
    error = capsys.readouterr().err
    assert error == '{"error":"controller name is not in the reviewed whitelist","ok":false}\n'


@pytest.mark.parametrize('fragment', [
    '&timing 0.02',
    '*timing',
    '<<: {}',
    '!!str value',
    '!custom {key: value}',
    '!custom [value]',
])
def test_rejects_anchors_aliases_merges_and_tags(fragment):
    with pytest.raises(validator.ControllerConfigError):
        validator.load_strict_yaml('value: {}\n'.format(fragment))


def test_rejects_duplicate_keys_and_multiple_documents():
    with pytest.raises(validator.ControllerConfigError, match='duplicate'):
        validator.load_strict_yaml('root:\n  value: 1\n  value: 2\n')
    with pytest.raises(validator.ControllerConfigError, match='multiple'):
        validator.load_strict_yaml('root: 1\n---\nroot: 2\n')


@pytest.mark.parametrize('value', ['null', 'true', '.nan', '.inf', '-.inf'])
def test_rejects_null_boolean_and_nonfinite_scalars(value):
    with pytest.raises(validator.ControllerConfigError):
        validator.load_strict_yaml('root: {}\n'.format(value))


def test_rejects_depth_scalar_count_and_byte_size_bounds():
    nested = 'leaf'
    for _ in range(validator.MAXIMUM_YAML_DEPTH + 1):
        nested = '[{}]'.format(nested)
    with pytest.raises(validator.ControllerConfigError, match='nesting'):
        validator.load_strict_yaml(nested)
    many = 'items:\n' + ''.join('  - {}\n'.format(index) for index in range(
        validator.MAXIMUM_YAML_SCALARS + 1))
    with pytest.raises(validator.ControllerConfigError, match='scalar count'):
        validator.load_strict_yaml(many)
    oversized = 'root: value\n' + '#' * validator.MAXIMUM_CONFIG_BYTES
    with pytest.raises(validator.ControllerConfigError, match='size'):
        validator.load_strict_yaml(oversized)


def test_file_reader_rejects_oversized_and_nonregular_inputs_without_blocking(tmp_path):
    oversized = tmp_path / 'oversized.yaml'
    oversized.write_bytes(b'x' * (validator.MAXIMUM_CONFIG_BYTES + 1))
    with pytest.raises(validator.ControllerConfigError, match='bounded'):
        validator.validate_controller_config_file(
            oversized, 'dual_arm_joint_hold_controller')
    fifo = tmp_path / 'config-fifo'
    os.mkfifo(fifo)
    with pytest.raises(validator.ControllerConfigError, match='bounded regular'):
        validator.validate_controller_config_file(
            fifo, 'dual_arm_joint_hold_controller')


def test_rejects_unknown_incomplete_or_non_slash_qualified_shapes():
    name = 'dual_arm_joint_hold_controller'
    configuration = _configuration(name)
    configuration['/' + name]['ros__parameters']['unknown'] = 1
    with pytest.raises(validator.ControllerConfigError, match='unknown'):
        validator.validate_controller_config_text(
            yaml.safe_dump(configuration), name)
    with pytest.raises(validator.ControllerConfigError):
        validator.validate_controller_config_text(
            yaml.safe_dump({'/' + name: {'ros__parameters': {}}}), name)
    with pytest.raises(validator.ControllerConfigError):
        validator.validate_controller_config_text(
            yaml.safe_dump({name: _configuration(name)['/' + name]}), name)


def test_rejects_unknown_controller_and_type_only_registration_file():
    with pytest.raises(validator.ControllerConfigError, match='whitelist'):
        validator.validate_controller_config_text('/legacy:\n  ros__parameters: {}\n',
                                                  'legacy')
    type_only = {
        '/dual_arm_joint_hold_controller': {
            'ros__parameters': {
                'type': validator.REVIEWED_CONTROLLERS['dual_arm_joint_hold_controller']}}}
    with pytest.raises(validator.ControllerConfigError):
        validator.validate_controller_config_text(
            yaml.safe_dump(type_only), 'dual_arm_joint_hold_controller')


def test_rejects_swapped_arms_reordered_or_wildcard_joint_names():
    name = 'dual_arm_joint_velocity_controller'
    configuration = _configuration(name)
    parameters = configuration['/' + name]['ros__parameters']
    parameters['arm_1']['arm_id'], parameters['arm_2']['arm_id'] = 'panda2', 'panda1'
    with pytest.raises(validator.ControllerConfigError, match='panda1'):
        validator.validate_controller_config_text(yaml.safe_dump(configuration), name)

    configuration = _configuration(name)
    joints = configuration['/' + name]['ros__parameters']['arm_1']['joint_names']
    joints[0], joints[1] = joints[1], joints[0]
    with pytest.raises(validator.ControllerConfigError, match='canonical'):
        validator.validate_controller_config_text(yaml.safe_dump(configuration), name)
    joints[0] = 'panda1_joint*'
    with pytest.raises(validator.ControllerConfigError, match='canonical'):
        validator.validate_controller_config_text(yaml.safe_dump(configuration), name)


@pytest.mark.parametrize('key,bad_value', [
    ('k_gains', -1.0),
    ('d_gains', True),
    ('max_effort', 88.0),
    ('max_effort', 10 ** 400),
])
def test_rejects_invalid_hold_gain_and_effort_values(key, bad_value):
    name = 'dual_arm_joint_hold_controller'
    configuration = _configuration(name)
    configuration['/' + name]['ros__parameters']['arm_1'][key][0] = bad_value
    with pytest.raises(validator.ControllerConfigError):
        validator.validate_controller_config_text(yaml.safe_dump(configuration), name)


def test_rejects_loose_velocity_acceleration_position_rate_and_timing():
    name = 'dual_arm_joint_velocity_controller'
    for key, bad_value in (('max_velocity', 3.0), ('max_acceleration', 21.0)):
        configuration = _configuration(name)
        configuration['/' + name]['ros__parameters']['arm_1'][key][0] = bad_value
        with pytest.raises(validator.ControllerConfigError):
            validator.validate_controller_config_text(
                yaml.safe_dump(configuration), name)
    configuration = _configuration(name)
    configuration['/' + name]['ros__parameters']['watchdog_timeout'] = 0.03
    with pytest.raises(validator.ControllerConfigError, match='reviewed'):
        validator.validate_controller_config_text(yaml.safe_dump(configuration), name)

    name = 'dual_arm_joint_impedance_controller'
    for key, bad_value in (
            ('position_lower', -3.0), ('position_upper', 3.0),
            ('max_target_velocity', 3.0)):
        configuration = _configuration(name)
        configuration['/' + name]['ros__parameters']['arm_1'][key][0] = bad_value
        with pytest.raises(validator.ControllerConfigError):
            validator.validate_controller_config_text(
                yaml.safe_dump(configuration), name)


def test_policy_itself_is_strict_and_exact(tmp_path):
    policy = validator._load_policy(_POLICY)
    assert policy['arm_ids'] == ['panda1', 'panda2']
    changed = yaml.safe_load(_POLICY.read_text(encoding='utf-8'))
    changed['arm_ids'].reverse()
    temporary = tmp_path / 'changed-policy.yaml'
    temporary.write_text(yaml.safe_dump(changed, sort_keys=False), encoding='utf-8')
    with pytest.raises(validator.ControllerConfigError, match='ordered'):
        validator._load_policy(temporary)
    changed = yaml.safe_load(_POLICY.read_text(encoding='utf-8'))
    changed['schema_version'] = 1.0
    temporary.write_text(yaml.safe_dump(changed, sort_keys=False), encoding='utf-8')
    with pytest.raises(validator.ControllerConfigError, match='version'):
        validator._load_policy(temporary)
