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

"""Read-only, bounded dual-Panda status collection."""

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareComponents
from controller_manager_msgs.srv import ListHardwareInterfaces
from diagnostic_msgs.msg import DiagnosticArray
from franka_bringup.controller_config_validator import ARM_IDS
from franka_bringup.controller_config_validator import expected_controller_interfaces
from franka_bringup.controller_config_validator import REVIEWED_CONTROLLERS
import rclpy
from rclpy.node import Node


CONTROLLER_MANAGER_NAME = '/controller_manager'
FRANKA_COMPONENT_NAME = 'FrankaMultiHardwareInterface'
FRANKA_COMPONENT_PLUGIN = 'franka_hardware/FrankaMultiHardwareInterface'
DIAGNOSTIC_TOPIC = '/diagnostics'
DIAGNOSTIC_KEYS = frozenset({
    'accepted_state_samples',
    'active_mode',
    'arm_id',
    'command_queue_saturated',
    'dropped_state_samples',
    'failure_reason',
    'fault_category',
    'global_fault_cause',
    'global_fault_origin_arm_id',
    'global_fault_origin_arm_slot',
    'hardware_lifecycle_id',
    'hardware_lifecycle_label',
    'last_recovery_result',
    'libfranka_version',
    'rclcpp_version',
    'recovering',
    'recovery_attempts',
    'recovery_failures',
    'recovery_successes',
    'rejected_backend_commands',
    'requested_mode',
    'ros2_control_version',
    'ros_distro',
    'service_operation',
    'source_commit',
    'source_dirty',
    'state_age_ms',
    'state_queue_saturated',
    'stopped',
    'unsafe_none_request_mask',
    'unsafe_safe_publish_mask',
    'worker_state',
})
_INTERFACE_CONTRACT_KEYS = frozenset({
    'arm_ids',
    'cartesian_matrix_interfaces',
    'cartesian_velocity_command_interfaces',
    'joint_command_interfaces',
    'joint_state_interfaces',
    'pointer_state_interfaces',
    'schema_version',
})


class StatusError(RuntimeError):
    """Status is unavailable, incomplete, or inconsistent."""


def _load_hardware_interface_contract():
    path = Path(get_package_share_directory('franka_hardware')) / 'config' / (
        'franka_multi_hardware_interface_contract.json')
    try:
        contract = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError('Franka hardware interface contract is unavailable') from error
    if not isinstance(contract, dict) or set(contract) != _INTERFACE_CONTRACT_KEYS:
        raise RuntimeError('Franka hardware interface contract keys differ')
    if isinstance(contract['schema_version'], bool) or contract['schema_version'] != 1:
        raise RuntimeError('Franka hardware interface contract version differs')
    if contract['arm_ids'] != list(ARM_IDS):
        raise RuntimeError('Franka hardware interface contract arm IDs differ')
    expected_lengths = {
        'arm_ids': 2,
        'cartesian_matrix_interfaces': 16,
        'cartesian_velocity_command_interfaces': 6,
        'joint_command_interfaces': 3,
        'joint_state_interfaces': 3,
        'pointer_state_interfaces': 2,
    }
    result = {}
    for key, expected_length in expected_lengths.items():
        values = contract[key]
        if (not isinstance(values, list) or len(values) != expected_length or
                any(not isinstance(value, str) or not value or '/' in value for value in values) or
                len(set(values)) != expected_length):
            raise RuntimeError('Franka hardware interface contract {} is invalid'.format(key))
        result[key] = tuple(values)
    return result


_HARDWARE_INTERFACE_CONTRACT = _load_hardware_interface_contract()
MATRIX_NAMES = _HARDWARE_INTERFACE_CONTRACT['cartesian_matrix_interfaces']
CARTESIAN_VELOCITY_COMMAND_NAMES = _HARDWARE_INTERFACE_CONTRACT[
    'cartesian_velocity_command_interfaces']
JOINT_COMMAND_INTERFACE_NAMES = _HARDWARE_INTERFACE_CONTRACT['joint_command_interfaces']
JOINT_STATE_INTERFACE_NAMES = _HARDWARE_INTERFACE_CONTRACT['joint_state_interfaces']
POINTER_STATE_INTERFACE_NAMES = _HARDWARE_INTERFACE_CONTRACT['pointer_state_interfaces']


def _joint_names(arm_id):
    return ['{}_joint{}'.format(arm_id, joint) for joint in range(1, 8)]


def expected_hardware_interfaces():
    command_names = set()
    state_names = set()
    for arm_id in ARM_IDS:
        for joint in _joint_names(arm_id):
            for interface in JOINT_COMMAND_INTERFACE_NAMES:
                command_names.add('{}/{}'.format(joint, interface))
            for interface in JOINT_STATE_INTERFACE_NAMES:
                state_names.add('{}/{}'.format(joint, interface))
        command_names.update(
            '{}_ee_cartesian_position/{}'.format(arm_id, name) for name in MATRIX_NAMES)
        command_names.update(
            '{}_ee_cartesian_velocity/{}'.format(arm_id, name)
            for name in CARTESIAN_VELOCITY_COMMAND_NAMES)
        state_names.update(
            '{}_ee_cartesian_position/{}'.format(arm_id, name) for name in MATRIX_NAMES)
        state_names.update(
            '{}_ee_cartesian_velocity/{}'.format(arm_id, name) for name in MATRIX_NAMES)
        state_names.update(
            '{}/{}'.format(arm_id, name) for name in POINTER_STATE_INTERFACE_NAMES)
    return command_names, state_names


EXPECTED_COMMAND_INTERFACES, EXPECTED_STATE_INTERFACES = expected_hardware_interfaces()


def _is_configured_arm_interface(name):
    return any(
        name.startswith(arm_id + '_') or name.startswith(arm_id + '/')
        for arm_id in ARM_IDS)


def _interface_map(interfaces, context):
    result = {}
    for interface in interfaces:
        name = interface.name
        if name in result:
            raise StatusError('{} contains duplicate interface {}'.format(context, name))
        result[name] = interface
    return result


def _configured_interface_map(interfaces, context):
    return {
        name: interface for name, interface in _interface_map(interfaces, context).items()
        if _is_configured_arm_interface(name)
    }


def _require_exact_interface_names(actual, expected, context):
    actual_names = set(actual)
    if actual_names != expected:
        raise StatusError('{} differs; missing={} unknown={}'.format(
            context, sorted(expected - actual_names), sorted(actual_names - expected)))


def _diagnostic_values(status):
    result = {}
    for value in status.values:
        if value.key in result:
            raise StatusError('diagnostic contains duplicate key {}'.format(value.key))
        result[value.key] = value.value
    if set(result) != DIAGNOSTIC_KEYS:
        raise StatusError('diagnostic keys differ; missing={} unknown={}'.format(
            sorted(DIAGNOSTIC_KEYS - set(result)), sorted(set(result) - DIAGNOSTIC_KEYS)))
    return result


def _diagnostic_level(value):
    if isinstance(value, bytes):
        if len(value) != 1:
            raise StatusError('diagnostic level is not a single byte')
        return value[0]
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise StatusError('diagnostic level is not an integer byte') from error
    if result < 0 or result > 255:
        raise StatusError('diagnostic level is outside the byte range')
    return result


def _select_diagnostics(message):
    required = {
        arm_id: 'franka_hardware_diagnostics: franka_hardware/{}'.format(arm_id)
        for arm_id in ARM_IDS
    }
    selected = {}
    required_names = set(required.values())
    for status in message.status:
        if status.name not in required_names:
            continue
        if status.name in selected:
            raise StatusError('diagnostic array contains duplicate canonical status')
        selected[status.name] = status
    if set(selected) != required_names:
        raise StatusError('fresh diagnostic array is missing a canonical arm status')
    result = {}
    for arm_id in ARM_IDS:
        status = selected[required[arm_id]]
        if status.hardware_id != arm_id:
            raise StatusError('{} diagnostic hardware_id mismatch'.format(arm_id))
        values = _diagnostic_values(status)
        if values['arm_id'] != arm_id:
            raise StatusError('{} diagnostic arm_id key mismatch'.format(arm_id))
        result[arm_id] = {
            'hardware_id': status.hardware_id,
            'level': _diagnostic_level(status.level),
            'message': status.message,
            'name': status.name,
            'values': values,
        }
    return result


def _select_component(response):
    components = [
        component for component in response.component
        if component.name == FRANKA_COMPONENT_NAME
        or component.plugin_name == FRANKA_COMPONENT_PLUGIN
    ]
    if len(components) != 1:
        raise StatusError('expected exactly one Franka hardware component')
    component = components[0]
    if component.name != FRANKA_COMPONENT_NAME or component.plugin_name != FRANKA_COMPONENT_PLUGIN:
        raise StatusError('Franka hardware component identity mismatch')
    return component


def _controller_payload(response, command_interfaces):
    controllers = {}
    claimed_by = {}
    reviewed_types = set(REVIEWED_CONTROLLERS.values())
    for controller in response.controller:
        if controller.name in controllers:
            raise StatusError('duplicate controller name: {}'.format(controller.name))
        claims = list(controller.claimed_interfaces)
        if len(claims) != len(set(claims)):
            raise StatusError('{} reports duplicate claims'.format(controller.name))
        configured_claims = [claim for claim in claims if _is_configured_arm_interface(claim)]
        if controller.name in REVIEWED_CONTROLLERS:
            if controller.type != REVIEWED_CONTROLLERS[controller.name]:
                raise StatusError('{} has an unreviewed type'.format(controller.name))
            expected_commands, expected_states = expected_controller_interfaces(controller.name)
            required_commands = list(controller.required_command_interfaces)
            required_states = list(controller.required_state_interfaces)
            if required_commands != list(expected_commands):
                raise StatusError('{} required command interfaces differ'.format(controller.name))
            if required_states != list(expected_states):
                raise StatusError('{} required state interfaces differ'.format(controller.name))
            expected_claims = set(expected_commands) if controller.state == 'active' else set()
            if set(configured_claims) != expected_claims:
                raise StatusError('{} claims do not match its lifecycle state'.format(
                    controller.name))
        elif controller.type in reviewed_types:
            raise StatusError('reviewed controller type is loaded under an unreviewed name')
        elif configured_claims:
            raise StatusError('unreviewed controller claims a configured Panda interface')
        elif controller.state == 'active' and any(
                _is_configured_arm_interface(name)
                for name in controller.required_command_interfaces):
            raise StatusError(
                'active unreviewed controller requires a configured Panda command interface')
        for claim in configured_claims:
            if claim not in command_interfaces:
                raise StatusError('{} claims an unknown command interface'.format(controller.name))
            if claim in claimed_by:
                raise StatusError('{} is claimed by multiple controllers'.format(claim))
            claimed_by[claim] = controller.name
        controllers[controller.name] = {
            'claimed_interfaces': sorted(claims),
            'state': controller.state,
            'type': controller.type,
        }
    return [dict({'name': name}, **controllers[name]) for name in sorted(controllers)], claimed_by


def validate_status_snapshot(
        diagnostic_array, controllers_response, interfaces_response, components_response):
    diagnostics = _select_diagnostics(diagnostic_array)
    component = _select_component(components_response)

    command_interfaces = _configured_interface_map(
        interfaces_response.command_interfaces, 'list_hardware_interfaces command response')
    state_interfaces = _configured_interface_map(
        interfaces_response.state_interfaces, 'list_hardware_interfaces state response')
    component_commands = _configured_interface_map(
        component.command_interfaces, 'Franka component command interfaces')
    component_states = _configured_interface_map(
        component.state_interfaces, 'Franka component state interfaces')
    _require_exact_interface_names(
        command_interfaces, EXPECTED_COMMAND_INTERFACES, 'global Franka command interfaces')
    _require_exact_interface_names(
        state_interfaces, EXPECTED_STATE_INTERFACES, 'global Franka state interfaces')
    _require_exact_interface_names(
        component_commands, EXPECTED_COMMAND_INTERFACES, 'component Franka command interfaces')
    _require_exact_interface_names(
        component_states, EXPECTED_STATE_INTERFACES, 'component Franka state interfaces')

    for name in EXPECTED_COMMAND_INTERFACES:
        if (bool(command_interfaces[name].is_available) !=
                bool(component_commands[name].is_available) or
                bool(command_interfaces[name].is_claimed) !=
                bool(component_commands[name].is_claimed)):
            raise StatusError('{} command interface views disagree'.format(name))
    for name in EXPECTED_STATE_INTERFACES:
        if bool(state_interfaces[name].is_available) != bool(component_states[name].is_available):
            raise StatusError('{} state interface views disagree'.format(name))

    controllers, claimed_by = _controller_payload(controllers_response, command_interfaces)
    for name, interface in command_interfaces.items():
        if bool(interface.is_claimed) != (name in claimed_by):
            raise StatusError('{} claim flag disagrees with list_controllers'.format(name))

    lifecycle_id = int(component.state.id)
    lifecycle_label = component.state.label
    for arm_id in ARM_IDS:
        values = diagnostics[arm_id]['values']
        try:
            diagnostic_lifecycle_id = int(values['hardware_lifecycle_id'])
        except ValueError as error:
            raise StatusError('diagnostic lifecycle ID is not an integer') from error
        if (diagnostic_lifecycle_id != lifecycle_id or
                values['hardware_lifecycle_label'] != lifecycle_label):
            raise StatusError('{} lifecycle disagrees with hardware component'.format(arm_id))

    diagnostic_error = any(status['level'] >= 2 for status in diagnostics.values())
    return {
        'controller_manager': CONTROLLER_MANAGER_NAME,
        'controllers': controllers,
        'diagnostic_error': diagnostic_error,
        'diagnostics': [dict({'arm_id': arm_id}, **diagnostics[arm_id]) for arm_id in ARM_IDS],
        'hardware': {
            'command_interface_count': len(EXPECTED_COMMAND_INTERFACES),
            'lifecycle_id': lifecycle_id,
            'lifecycle_label': lifecycle_label,
            'name': component.name,
            'plugin_name': component.plugin_name,
            'state_interface_count': len(EXPECTED_STATE_INTERFACES),
        },
        'ok': not diagnostic_error,
    }


class FrankaStatusNode(Node):
    def __init__(self):
        super().__init__('franka_status')
        self._diagnostic_array = None
        self._subscription_started_ns = time.monotonic_ns()
        self._subscription = self.create_subscription(
            DiagnosticArray, DIAGNOSTIC_TOPIC, self._diagnostic_callback, 10)
        self._read_only_clients = {
            'controllers': self.create_client(
                ListControllers, CONTROLLER_MANAGER_NAME + '/list_controllers'),
            'interfaces': self.create_client(
                ListHardwareInterfaces, CONTROLLER_MANAGER_NAME + '/list_hardware_interfaces'),
            'components': self.create_client(
                ListHardwareComponents, CONTROLLER_MANAGER_NAME + '/list_hardware_components'),
        }

    def _diagnostic_callback(self, message):
        required_names = {
            'franka_hardware_diagnostics: franka_hardware/{}'.format(arm_id)
            for arm_id in ARM_IDS
        }
        if required_names.issubset({status.name for status in message.status}):
            self._diagnostic_array = (time.monotonic_ns(), message)

    def collect(self, timeout_seconds):
        deadline = time.monotonic() + timeout_seconds
        for client in self._read_only_clients.values():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_service(timeout_sec=remaining):
                raise StatusError('timed out waiting for a read-only controller-manager service')
        requests = {
            'controllers': ListControllers.Request(),
            'interfaces': ListHardwareInterfaces.Request(),
            'components': ListHardwareComponents.Request(),
        }
        futures = {
            name: self._read_only_clients[name].call_async(request)
            for name, request in requests.items()
        }
        responses = {}
        for name in ('controllers', 'interfaces', 'components'):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise StatusError('timed out waiting for read-only status responses')
            rclpy.spin_until_future_complete(self, futures[name], timeout_sec=remaining)
            if not futures[name].done() or futures[name].exception() is not None:
                raise StatusError('read-only {} request failed'.format(name))
            responses[name] = futures[name].result()
        while self._diagnostic_array is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise StatusError(
                    'timed out waiting for a fresh post-subscription diagnostic array')
            rclpy.spin_once(self, timeout_sec=remaining)
        received_ns, diagnostic_array = self._diagnostic_array
        if received_ns < self._subscription_started_ns:
            raise StatusError('diagnostic array predates the subscription')
        return validate_status_snapshot(
            diagnostic_array, responses['controllers'], responses['interfaces'],
            responses['components'])


def _json_line(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Read-only dual-Panda status')
    parser.add_argument('--timeout', type=float, default=5.0)
    arguments = parser.parse_args(argv)
    if (not math.isfinite(arguments.timeout) or arguments.timeout < 0.1 or
            arguments.timeout > 30.0):
        print(_json_line({'error': 'timeout must be finite and between 0.1 and 30 seconds',
                         'ok': False}), file=sys.stderr)
        return 2

    node = None
    result = None
    failure = None
    cleanup_failed = False
    try:
        rclpy.init(args=[])
        node = FrankaStatusNode()
        result = node.collect(arguments.timeout)
    except StatusError:
        failure = 'status unavailable or inconsistent'
    except (Exception, KeyboardInterrupt):
        failure = 'status runtime failed'
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (Exception, KeyboardInterrupt):
                cleanup_failed = True
        try:
            rclpy.try_shutdown()
        except (Exception, KeyboardInterrupt):
            cleanup_failed = True
    if failure is None and cleanup_failed:
        failure = 'status cleanup failed'
    if failure is not None:
        print(_json_line({'error': failure, 'ok': False}), file=sys.stderr)
        return 3
    print(_json_line(result))
    return 0 if result['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
