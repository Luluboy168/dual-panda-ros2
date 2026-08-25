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

"""Strict validation for the three reviewed dual-Panda motion controllers."""

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import sys
from typing import Any, Mapping, Sequence

from ament_index_python.packages import get_package_share_directory
import yaml
from yaml.events import AliasEvent
from yaml.events import DocumentStartEvent
from yaml.events import MappingEndEvent
from yaml.events import MappingStartEvent
from yaml.events import ScalarEvent
from yaml.events import SequenceEndEvent
from yaml.events import SequenceStartEvent


MAXIMUM_CONFIG_BYTES = 65536
MAXIMUM_YAML_DEPTH = 8
MAXIMUM_YAML_SCALARS = 2048
JOINT_COUNT = 7
ARM_IDS = ('panda1', 'panda2')

REVIEWED_CONTROLLERS = {
    'dual_arm_joint_hold_controller':
        'franka_example_controllers/DualArmJointHoldController',
    'dual_arm_joint_impedance_controller':
        'franka_example_controllers/DualArmJointImpedanceController',
    'dual_arm_joint_velocity_controller':
        'franka_example_controllers/DualArmJointVelocityController',
}


class ControllerConfigError(ValueError):
    """A deterministic, user-correctable controller configuration failure."""


class _UniqueKeySafeLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ControllerConfigError('every YAML mapping key must be a string')
        if key == '<<':
            raise ControllerConfigError('YAML merge keys are forbidden')
        if key in mapping:
            raise ControllerConfigError('duplicate YAML key: {}'.format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _scan_yaml_events(text: str) -> None:
    document_count = 0
    depth = 0
    scalar_count = 0
    try:
        events = yaml.parse(text)
        for event in events:
            if isinstance(event, DocumentStartEvent):
                document_count += 1
                if document_count > 1:
                    raise ControllerConfigError('multiple YAML documents are forbidden')
            if isinstance(event, AliasEvent):
                raise ControllerConfigError('YAML aliases are forbidden')
            if getattr(event, 'anchor', None) is not None:
                raise ControllerConfigError('YAML anchors are forbidden')
            if getattr(event, 'tag', None) is not None:
                raise ControllerConfigError('explicit YAML tags are forbidden')
            if isinstance(event, ScalarEvent):
                scalar_count += 1
                if scalar_count > MAXIMUM_YAML_SCALARS:
                    raise ControllerConfigError('YAML scalar count exceeds the fixed limit')
                if event.value == '<<':
                    raise ControllerConfigError('YAML merge keys are forbidden')
            if isinstance(event, (MappingStartEvent, SequenceStartEvent)):
                depth += 1
                if depth > MAXIMUM_YAML_DEPTH:
                    raise ControllerConfigError('YAML nesting exceeds the fixed limit')
            elif isinstance(event, (MappingEndEvent, SequenceEndEvent)):
                depth -= 1
    except ControllerConfigError:
        raise
    except yaml.YAMLError as error:
        raise ControllerConfigError('malformed YAML') from error
    if document_count != 1 or depth != 0:
        raise ControllerConfigError('configuration must contain exactly one YAML document')


def _reject_unsafe_scalars(value: Any) -> None:
    if value is None:
        raise ControllerConfigError('null YAML values are forbidden')
    if isinstance(value, bool):
        raise ControllerConfigError('boolean YAML values are forbidden')
    if isinstance(value, float) and not math.isfinite(value):
        raise ControllerConfigError('non-finite YAML numbers are forbidden')
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise ControllerConfigError('every YAML mapping key must be a string')
            _reject_unsafe_scalars(child)
    elif isinstance(value, list):
        for child in value:
            _reject_unsafe_scalars(child)
    elif not isinstance(value, (str, int, float)):
        raise ControllerConfigError('unsupported YAML value')


def load_strict_yaml(text: str) -> Any:
    encoded = text.encode('utf-8')
    if not encoded or len(encoded) > MAXIMUM_CONFIG_BYTES:
        raise ControllerConfigError('YAML size is outside the fixed limit')
    if '\x00' in text:
        raise ControllerConfigError('NUL bytes are forbidden')
    _scan_yaml_events(text)
    try:
        value = yaml.load(text, Loader=_UniqueKeySafeLoader)
    except ControllerConfigError:
        raise
    except yaml.YAMLError as error:
        raise ControllerConfigError('malformed YAML') from error
    _reject_unsafe_scalars(value)
    return value


def default_limit_policy_path() -> Path:
    return Path(get_package_share_directory('franka_example_controllers')) / 'config' / (
        'panda_joint_limits_v1.yaml')


def _read_bounded_regular_text(path: Path, context: str) -> str:
    descriptor = -1
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK)
        details = os.fstat(descriptor)
        if not stat.S_ISREG(details.st_mode) or details.st_size > MAXIMUM_CONFIG_BYTES:
            raise ControllerConfigError('{} must be a bounded regular file'.format(context))
        data = os.read(descriptor, MAXIMUM_CONFIG_BYTES + 1)
        if len(data) > MAXIMUM_CONFIG_BYTES:
            raise ControllerConfigError('{} exceeds the fixed size limit'.format(context))
        return data.decode('utf-8')
    except ControllerConfigError:
        raise
    except (OSError, UnicodeError) as error:
        raise ControllerConfigError('unable to read {}'.format(context)) from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _exact_keys(mapping: Any, expected: Sequence[str], context: str) -> Mapping[str, Any]:
    if not isinstance(mapping, dict):
        raise ControllerConfigError('{} must be a mapping'.format(context))
    expected_set = set(expected)
    actual_set = set(mapping)
    if actual_set != expected_set:
        missing = sorted(expected_set - actual_set)
        unknown = sorted(actual_set - expected_set)
        raise ControllerConfigError(
            '{} keys differ; missing={} unknown={}'.format(context, missing, unknown))
    return mapping


def _string_list(mapping: Mapping[str, Any], key: str, expected_length: int) -> list[str]:
    values = mapping[key]
    if (not isinstance(values, list) or len(values) != expected_length or
            any(not isinstance(value, str) for value in values)):
        raise ControllerConfigError('{} must be a {}-element string list'.format(
            key, expected_length))
    return values


def _number_list(mapping: Mapping[str, Any], key: str) -> list[float]:
    values = mapping[key]
    if not isinstance(values, list) or len(values) != JOINT_COUNT:
        raise ControllerConfigError('{} must contain exactly seven numbers'.format(key))
    result = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ControllerConfigError('{} must contain only numbers'.format(key))
        try:
            converted = float(value)
        except OverflowError as error:
            raise ControllerConfigError('{} must contain only finite numbers'.format(key)) \
                from error
        if not math.isfinite(converted):
            raise ControllerConfigError('{} must contain only finite numbers'.format(key))
        result.append(converted)
    return result


def _number(mapping: Mapping[str, Any], key: str) -> float:
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ControllerConfigError('{} must be numeric'.format(key))
    try:
        result = float(value)
    except OverflowError as error:
        raise ControllerConfigError('{} must be finite'.format(key)) from error
    if not math.isfinite(result):
        raise ControllerConfigError('{} must be finite'.format(key))
    return result


def _load_policy(path: Path) -> Mapping[str, Any]:
    text = _read_bounded_regular_text(path, 'the versioned Panda limit policy')
    policy = load_strict_yaml(text)
    expected_keys = (
        'schema_version', 'libfranka_version', 'arm_ids', 'joint_suffixes',
        'hardware_command_interfaces', 'hardware_state_interfaces', 'effort_ceiling',
        'position_lower', 'position_upper', 'urdf_velocity_ceiling',
        'libfranka_velocity_ceiling', 'libfranka_acceleration_ceiling',
        'reviewed_timing_seconds',
    )
    _exact_keys(policy, expected_keys, 'limit policy')
    if (type(policy['schema_version']) is not int or policy['schema_version'] != 1 or
            policy['libfranka_version'] != '0.9.2'):
        raise ControllerConfigError('unsupported Panda limit policy version')
    if tuple(_string_list(policy, 'arm_ids', 2)) != ARM_IDS:
        raise ControllerConfigError('limit policy arm IDs must be ordered panda1,panda2')
    expected_suffixes = tuple('joint{}'.format(index) for index in range(1, 8))
    if tuple(_string_list(policy, 'joint_suffixes', 7)) != expected_suffixes:
        raise ControllerConfigError('limit policy joint suffixes are not canonical')
    if _string_list(policy, 'hardware_command_interfaces', 3) != [
            'effort', 'position', 'velocity']:
        raise ControllerConfigError('limit policy command interfaces are not canonical')
    if _string_list(policy, 'hardware_state_interfaces', 3) != [
            'effort', 'position', 'velocity']:
        raise ControllerConfigError('limit policy state interfaces are not canonical')
    for key in (
            'effort_ceiling', 'position_lower', 'position_upper', 'urdf_velocity_ceiling',
            'libfranka_velocity_ceiling', 'libfranka_acceleration_ceiling'):
        _number_list(policy, key)
    timings = _exact_keys(
        policy['reviewed_timing_seconds'],
        ('dual_arm_joint_velocity_controller', 'dual_arm_joint_impedance_controller'),
        'reviewed_timing_seconds',
    )
    for name in timings:
        values = _exact_keys(
            timings[name],
            ('watchdog_timeout', 'max_header_age', 'future_tolerance'),
            'reviewed_timing_seconds.{}'.format(name),
        )
        for key in values:
            if _number(values, key) <= 0.0:
                raise ControllerConfigError('reviewed timing values must be positive')
    return policy


def canonical_joint_names(arm_id: str) -> list[str]:
    return ['{}_joint{}'.format(arm_id, joint) for joint in range(1, 8)]


def expected_controller_interfaces(controller_name: str) -> tuple[list[str], list[str]]:
    command_kind = 'velocity' if controller_name.endswith('velocity_controller') else 'effort'
    commands = [
        '{}/{}'.format(joint, command_kind)
        for arm_id in ARM_IDS
        for joint in canonical_joint_names(arm_id)
    ]
    if controller_name.endswith('velocity_controller'):
        return commands, []
    states = []
    for arm_id in ARM_IDS:
        for joint in canonical_joint_names(arm_id):
            states.extend(('{}/position'.format(joint), '{}/velocity'.format(joint)))
        states.extend(('{}/robot_state'.format(arm_id), '{}/robot_model'.format(arm_id)))
    return commands, states


def _validate_arm_common(
        arm: Any, arm_slot: int, keys: Sequence[str], require_joint_names: bool,
) -> Mapping[str, Any]:
    arm = _exact_keys(arm, keys, 'arm_{}'.format(arm_slot))
    arm_id = ARM_IDS[arm_slot - 1]
    if arm['arm_id'] != arm_id:
        raise ControllerConfigError('arm_{} arm_id must be {}'.format(arm_slot, arm_id))
    if require_joint_names:
        expected = canonical_joint_names(arm_id)
        if _string_list(arm, 'joint_names', JOINT_COUNT) != expected:
            raise ControllerConfigError('arm_{} joint_names are not canonical'.format(arm_slot))
    return arm


def _validate_nonnegative(values: Sequence[float], name: str) -> None:
    if any(value < 0.0 for value in values):
        raise ControllerConfigError('{} must contain finite nonnegative values'.format(name))


def _validate_positive_ceiling(
        values: Sequence[float], ceilings: Sequence[float], name: str,
) -> None:
    if any(value <= 0.0 or value > ceiling for value, ceiling in zip(values, ceilings)):
        raise ControllerConfigError('{} must be positive and no greater than policy'.format(name))


def _validate_timings(
        parameters: Mapping[str, Any], policy: Mapping[str, Any], controller_name: str,
) -> None:
    reviewed = policy['reviewed_timing_seconds'][controller_name]
    for key in ('watchdog_timeout', 'max_header_age', 'future_tolerance'):
        if _number(parameters, key) != _number(reviewed, key):
            raise ControllerConfigError('{} must equal the reviewed policy value'.format(key))


@dataclass(frozen=True)
class ValidatedControllerConfig:
    controller_name: str
    controller_type: str
    config_sha256: str
    command_interfaces: tuple[str, ...]
    state_interfaces: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            'command_interfaces': list(self.command_interfaces),
            'config_sha256': self.config_sha256,
            'controller_name': self.controller_name,
            'controller_type': self.controller_type,
            'ok': True,
            'state_interfaces': list(self.state_interfaces),
        }


def validate_controller_config_text(
        text: str, controller_name: str,
) -> ValidatedControllerConfig:
    if controller_name not in REVIEWED_CONTROLLERS:
        raise ControllerConfigError('controller name is not in the reviewed whitelist')
    root = load_strict_yaml(text)
    root_key = '/' + controller_name
    root = _exact_keys(root, (root_key,), 'controller configuration')
    node = _exact_keys(root[root_key], ('ros__parameters',), root_key)
    parameters = node['ros__parameters']
    policy = _load_policy(default_limit_policy_path())

    if controller_name == 'dual_arm_joint_hold_controller':
        parameters = _exact_keys(parameters, ('arm_1', 'arm_2'), 'ros__parameters')
        arm_keys = ('arm_id', 'k_gains', 'd_gains', 'max_effort')
        for slot in (1, 2):
            arm = _validate_arm_common(parameters['arm_{}'.format(slot)], slot, arm_keys, False)
            _validate_nonnegative(_number_list(arm, 'k_gains'), 'k_gains')
            _validate_nonnegative(_number_list(arm, 'd_gains'), 'd_gains')
            _validate_positive_ceiling(
                _number_list(arm, 'max_effort'), _number_list(policy, 'effort_ceiling'),
                'max_effort')
    elif controller_name == 'dual_arm_joint_velocity_controller':
        parameters = _exact_keys(
            parameters,
            ('watchdog_timeout', 'max_header_age', 'future_tolerance', 'arm_1', 'arm_2'),
            'ros__parameters',
        )
        _validate_timings(parameters, policy, controller_name)
        arm_keys = ('arm_id', 'joint_names', 'max_velocity', 'max_acceleration')
        for slot in (1, 2):
            arm = _validate_arm_common(parameters['arm_{}'.format(slot)], slot, arm_keys, True)
            _validate_positive_ceiling(
                _number_list(arm, 'max_velocity'),
                _number_list(policy, 'libfranka_velocity_ceiling'), 'max_velocity')
            _validate_positive_ceiling(
                _number_list(arm, 'max_acceleration'),
                _number_list(policy, 'libfranka_acceleration_ceiling'), 'max_acceleration')
    else:
        parameters = _exact_keys(
            parameters,
            ('watchdog_timeout', 'max_header_age', 'future_tolerance', 'arm_1', 'arm_2'),
            'ros__parameters',
        )
        _validate_timings(parameters, policy, controller_name)
        arm_keys = (
            'arm_id', 'joint_names', 'k_gains', 'd_gains', 'max_effort', 'position_lower',
            'position_upper', 'max_target_velocity',
        )
        policy_lower = _number_list(policy, 'position_lower')
        policy_upper = _number_list(policy, 'position_upper')
        for slot in (1, 2):
            arm = _validate_arm_common(parameters['arm_{}'.format(slot)], slot, arm_keys, True)
            _validate_nonnegative(_number_list(arm, 'k_gains'), 'k_gains')
            _validate_nonnegative(_number_list(arm, 'd_gains'), 'd_gains')
            _validate_positive_ceiling(
                _number_list(arm, 'max_effort'), _number_list(policy, 'effort_ceiling'),
                'max_effort')
            lower = _number_list(arm, 'position_lower')
            upper = _number_list(arm, 'position_upper')
            if any(
                    configured_lower < canonical_lower or
                    configured_upper > canonical_upper or
                    configured_lower >= configured_upper
                    for configured_lower, configured_upper, canonical_lower, canonical_upper in
                    zip(lower, upper, policy_lower, policy_upper)):
                raise ControllerConfigError('position bounds must be inside the Panda policy')
            _validate_positive_ceiling(
                _number_list(arm, 'max_target_velocity'),
                _number_list(policy, 'urdf_velocity_ceiling'), 'max_target_velocity')

    commands, states = expected_controller_interfaces(controller_name)
    return ValidatedControllerConfig(
        controller_name=controller_name,
        controller_type=REVIEWED_CONTROLLERS[controller_name],
        config_sha256=hashlib.sha256(text.encode('utf-8')).hexdigest(),
        command_interfaces=tuple(commands),
        state_interfaces=tuple(states),
    )


def validate_controller_config_file(
        path: Path, controller_name: str,
) -> ValidatedControllerConfig:
    text = _read_bounded_regular_text(path, 'controller configuration')
    return validate_controller_config_text(text, controller_name)


def _json_line(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Validate one reviewed dual-Panda controller YAML')
    parser.add_argument('config', type=Path)
    parser.add_argument('--controller-name', required=True)
    arguments = parser.parse_args(argv)
    try:
        result = validate_controller_config_file(arguments.config, arguments.controller_name)
    except ControllerConfigError as error:
        print(_json_line({'error': str(error), 'ok': False}), file=sys.stderr)
        return 2
    print(_json_line(result.as_dict()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
