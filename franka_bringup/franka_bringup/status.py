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
Read-only, bounded Franka status collection for the dual and one-arm bringups.

Two arm modes are supported and selected with ``--arm-mode``:

``dual`` (the default)
    The two-arm layout: both ``panda1`` and ``panda2`` must be present, and every constant,
    check and output field is exactly what this tool produced before one-arm mode existed.
    Default output is byte-identical to that prior contract -- no key was added, removed or
    renamed on this path.

``single``
    The one-arm bringup (``launch/real/one_arm_franka.launch.py``), whose arm ID is a
    launch-time argument and therefore is *not* known to this tool. The arm is auto-detected
    from the canonical Franka diagnostics and the detection refuses rather than guesses: see
    ``resolve_arm_ids``. Only the interfaces, controller interface arrays and diagnostics that
    exist for the detected arm are validated, and they are validated exactly -- an interface
    belonging to the other production arm is reported as unknown, not silently ignored.
"""

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
from franka_bringup.controller_config_validator import expected_single_controller_interfaces
from franka_bringup.controller_config_validator import ONE_ARM_MODE_IDS
from franka_bringup.controller_config_validator import REVIEWED_CONTROLLERS
import rclpy
from rclpy.node import Node


CONTROLLER_MANAGER_NAME = '/controller_manager'
DEFAULT_ARM_MODE = 'dual'
ARM_MODES = ('dual', 'single')
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


def expected_hardware_interfaces_for(arm_ids):
    """
    Return the exact (command, state) interface names the given arms must expose.

    ``expected_hardware_interfaces()`` is this function applied to the two production arms and
    is the sole producer of the module-level dual constants, so the dual sets cannot drift from
    the one-arm sets: both come from this one body.
    """
    command_names = set()
    state_names = set()
    for arm_id in arm_ids:
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


def expected_hardware_interfaces():
    return expected_hardware_interfaces_for(ARM_IDS)


EXPECTED_COMMAND_INTERFACES, EXPECTED_STATE_INTERFACES = expected_hardware_interfaces()


def _is_configured_arm_interface(name):
    # Deliberately spans BOTH production arm IDs in every arm mode. In one-arm mode the
    # *expected* set is narrowed to the detected arm, but this predicate is not: an interface
    # belonging to the arm that should not be there stays "configured", so it surfaces as
    # unknown=... in _require_exact_interface_names instead of being filtered away unseen.
    return any(
        name.startswith(arm_id + '_') or name.startswith(arm_id + '/')
        for arm_id in ARM_IDS)


def validate_arm_mode(arm_mode):
    if arm_mode not in ARM_MODES:
        raise StatusError('arm mode must be one of: {}'.format(', '.join(sorted(ARM_MODES))))
    return arm_mode


def canonical_diagnostic_name(arm_id):
    return 'franka_hardware_diagnostics: franka_hardware/{}'.format(arm_id)


def observed_arm_ids(message):
    """Return the production arm IDs whose canonical Franka diagnostic is in this array."""
    names = {status.name for status in message.status}
    return tuple(
        arm_id for arm_id in ONE_ARM_MODE_IDS if canonical_diagnostic_name(arm_id) in names)


def resolve_arm_ids(diagnostic_array, arm_mode=DEFAULT_ARM_MODE):
    """
    Return the arm IDs this snapshot must be validated against, refusing when ambiguous.

    dual
        Always the two production arms, exactly as before one-arm mode existed. Nothing is
        auto-detected, so a one-arm bringup fails the ordinary "missing a canonical arm status"
        check rather than being silently accepted as half a dual stack.
    single
        Auto-detected from the canonical Franka diagnostics, with two refusal rules -- the tool
        never guesses which arm an operator meant:
          * no canonical arm diagnostic  -> refuse (nothing to validate against);
          * more than one                -> refuse as ambiguous (this is a dual stack, or a
            stale second publisher; validating "the first one" could report a healthy arm while
            the other one is faulted).
    """
    validate_arm_mode(arm_mode)
    if arm_mode == 'dual':
        return tuple(ARM_IDS)
    observed = observed_arm_ids(diagnostic_array)
    if not observed:
        raise StatusError(
            'single arm mode found no canonical Franka arm diagnostic; expected exactly one of '
            '{}'.format(', '.join(ONE_ARM_MODE_IDS)))
    if len(observed) != 1:
        raise StatusError(
            'single arm mode is ambiguous: canonical Franka diagnostics for {} are all '
            'present'.format(', '.join(observed)))
    return observed


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


def _select_diagnostics(message, arm_ids=ARM_IDS):
    required = {arm_id: canonical_diagnostic_name(arm_id) for arm_id in arm_ids}
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
    for arm_id in arm_ids:
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


def _expected_controller_interfaces_for(arm_ids):
    """
    Return the reviewed-controller interface resolver for the arms under validation.

    The three reviewed controllers accept arm_count 1 or 2 and expose a different (exact,
    ordered) interface array in each case, so one-arm mode must compare against the one-arm
    arrays -- see controller_config_validator.expected_single_controller_interfaces.
    """
    if tuple(arm_ids) == tuple(ARM_IDS):
        return expected_controller_interfaces
    arm_id, = arm_ids
    return lambda controller_name: expected_single_controller_interfaces(controller_name, arm_id)


def _controller_payload(
        response, command_interfaces, expected_interfaces=expected_controller_interfaces):
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
            expected_commands, expected_states = expected_interfaces(controller.name)
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
        diagnostic_array, controllers_response, interfaces_response, components_response,
        arm_mode=DEFAULT_ARM_MODE):
    arm_ids = resolve_arm_ids(diagnostic_array, arm_mode)
    expected_command_interfaces, expected_state_interfaces = (
        expected_hardware_interfaces_for(arm_ids))
    diagnostics = _select_diagnostics(diagnostic_array, arm_ids)
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
        command_interfaces, expected_command_interfaces, 'global Franka command interfaces')
    _require_exact_interface_names(
        state_interfaces, expected_state_interfaces, 'global Franka state interfaces')
    _require_exact_interface_names(
        component_commands, expected_command_interfaces, 'component Franka command interfaces')
    _require_exact_interface_names(
        component_states, expected_state_interfaces, 'component Franka state interfaces')

    for name in expected_command_interfaces:
        if (bool(command_interfaces[name].is_available) !=
                bool(component_commands[name].is_available) or
                bool(command_interfaces[name].is_claimed) !=
                bool(component_commands[name].is_claimed)):
            raise StatusError('{} command interface views disagree'.format(name))
    for name in expected_state_interfaces:
        if bool(state_interfaces[name].is_available) != bool(component_states[name].is_available):
            raise StatusError('{} state interface views disagree'.format(name))

    controllers, claimed_by = _controller_payload(
        controllers_response, command_interfaces,
        _expected_controller_interfaces_for(arm_ids))
    for name, interface in command_interfaces.items():
        if bool(interface.is_claimed) != (name in claimed_by):
            raise StatusError('{} claim flag disagrees with list_controllers'.format(name))

    lifecycle_id = int(component.state.id)
    lifecycle_label = component.state.label
    for arm_id in arm_ids:
        values = diagnostics[arm_id]['values']
        try:
            diagnostic_lifecycle_id = int(values['hardware_lifecycle_id'])
        except ValueError as error:
            raise StatusError('diagnostic lifecycle ID is not an integer') from error
        if (diagnostic_lifecycle_id != lifecycle_id or
                values['hardware_lifecycle_label'] != lifecycle_label):
            raise StatusError('{} lifecycle disagrees with hardware component'.format(arm_id))

    diagnostic_error = any(status['level'] >= 2 for status in diagnostics.values())
    result = {
        'controller_manager': CONTROLLER_MANAGER_NAME,
        'controllers': controllers,
        'diagnostic_error': diagnostic_error,
        'diagnostics': [dict({'arm_id': arm_id}, **diagnostics[arm_id]) for arm_id in arm_ids],
        'hardware': {
            'command_interface_count': len(expected_command_interfaces),
            'lifecycle_id': lifecycle_id,
            'lifecycle_label': lifecycle_label,
            'name': component.name,
            'plugin_name': component.plugin_name,
            'state_interface_count': len(expected_state_interfaces),
        },
        'ok': not diagnostic_error,
    }
    # arm_mode and the auto-detected arm_id appear only for non-default modes, so default dual
    # output stays byte-identical to the pre-one-arm contract (the F-9d review finding that the
    # recorder's --arm-mode already follows).
    if arm_mode != DEFAULT_ARM_MODE:
        result['arm_id'] = arm_ids[0]
        result['arm_mode'] = arm_mode
    return result


class FrankaStatusNode(Node):
    def __init__(self, arm_mode=DEFAULT_ARM_MODE):
        # Validated before super().__init__ so an unusable mode never creates a ROS node.
        arm_mode = validate_arm_mode(arm_mode)
        super().__init__('franka_status')
        self._arm_mode = arm_mode
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
        if self._arm_mode == 'dual':
            required_names = {canonical_diagnostic_name(arm_id) for arm_id in ARM_IDS}
            if required_names.issubset({status.name for status in message.status}):
                self._diagnostic_array = (time.monotonic_ns(), message)
            return
        # One-arm mode accepts any array carrying at least one canonical arm status, including
        # an array carrying both: resolve_arm_ids then REFUSES that array as ambiguous. Filtering
        # it out here instead would turn "you pointed single-arm status at a dual stack" into an
        # indistinguishable collection timeout.
        if observed_arm_ids(message):
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
            responses['components'], self._arm_mode)


def _json_line(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Read-only Franka status for the given arm mode')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument(
        '--arm-mode', choices=sorted(ARM_MODES), default=DEFAULT_ARM_MODE,
        help='Layout to validate: the dual-Panda stack (default) or a one-arm bringup, whose '
             'arm ID is auto-detected and refused if ambiguous')
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
        node = FrankaStatusNode(arguments.arm_mode)
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
