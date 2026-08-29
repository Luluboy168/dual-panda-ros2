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

import ast
import builtins
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from controller_manager_msgs.msg import ControllerState
from franka_bringup import status
import pytest


def _interface(name, claimed=False):
    return SimpleNamespace(name=name, data_type='double', is_available=True, is_claimed=claimed)


def _diagnostic(arm_id, level=0):
    values = {key: '0' for key in status.DIAGNOSTIC_KEYS}
    values.update({
        'arm_id': arm_id,
        'hardware_lifecycle_id': '3',
        'hardware_lifecycle_label': 'active',
        'source_commit': '0' * 40,
    })
    return SimpleNamespace(
        name='franka_hardware_diagnostics: franka_hardware/{}'.format(arm_id),
        hardware_id=arm_id,
        level=level,
        message='healthy',
        values=[SimpleNamespace(key=key, value=values[key]) for key in sorted(values)],
    )


def test_diagnostic_level_accepts_ros_byte_and_rejects_invalid_values():
    assert status._diagnostic_level(b'\x02') == 2
    assert status._diagnostic_level(1) == 1
    for value in (b'', b'\x00\x01', -1, 256, 'not-an-integer'):
        with pytest.raises(status.StatusError):
            status._diagnostic_level(value)


def test_cli_reports_contract_initialization_failure_without_traceback(monkeypatch, capsys):
    script_path = Path(__file__).resolve().parents[1] / 'scripts' / 'franka_status.py'
    spec = importlib.util.spec_from_file_location('franka_status_cli_test', script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    real_import = builtins.__import__

    def reject_status_import(name, *args, **kwargs):
        if name == 'franka_bringup.status':
            raise RuntimeError('Franka hardware interface contract is unavailable')
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', reject_status_import)
    assert module.main([]) == 2
    captured = capsys.readouterr()
    assert captured.out == ''
    assert json.loads(captured.err) == {
        'error': 'franka_status initialization failed',
        'ok': False,
    }
    assert 'Traceback' not in captured.err


def _responses(controller=None, diagnostic_level=0):
    command_names = sorted(status.EXPECTED_COMMAND_INTERFACES)
    state_names = sorted(status.EXPECTED_STATE_INTERFACES)
    claims = set(controller.claimed_interfaces) if controller is not None else set()
    command_interfaces = [_interface(name, name in claims) for name in command_names]
    state_interfaces = [_interface(name) for name in state_names]
    component = SimpleNamespace(
        name=status.FRANKA_COMPONENT_NAME,
        plugin_name=status.FRANKA_COMPONENT_PLUGIN,
        state=SimpleNamespace(id=3, label='active'),
        command_interfaces=[_interface(name, name in claims) for name in command_names],
        state_interfaces=[_interface(name) for name in state_names],
    )
    controllers = [] if controller is None else [controller]
    return (
        SimpleNamespace(status=[
            SimpleNamespace(name='unrelated: task', hardware_id='other', level=0,
                            message='ignored', values=[]),
            _diagnostic('panda1', diagnostic_level),
            _diagnostic('panda2', diagnostic_level),
        ]),
        SimpleNamespace(controller=controllers),
        SimpleNamespace(command_interfaces=command_interfaces, state_interfaces=state_interfaces),
        SimpleNamespace(component=[
            SimpleNamespace(name='OtherSystem', plugin_name='other/Plugin'), component]),
    )


def _reviewed_controller(name, lifecycle_state='active'):
    commands, states = status.expected_controller_interfaces(name)
    claims = commands if lifecycle_state == 'active' else []
    return SimpleNamespace(
        name=name,
        type=status.REVIEWED_CONTROLLERS[name],
        state=lifecycle_state,
        claimed_interfaces=claims,
        required_command_interfaces=commands,
        required_state_interfaces=states,
    )


def test_valid_snapshot_is_stable_complete_and_tolerates_unrelated_entries():
    responses = _responses()
    first = status.validate_status_snapshot(*responses)
    second = status.validate_status_snapshot(*responses)
    assert first == second
    assert first['ok']
    assert [entry['arm_id'] for entry in first['diagnostics']] == ['panda1', 'panda2']
    assert first['hardware']['command_interface_count'] == 86
    assert first['hardware']['state_interface_count'] == 110


def test_expected_contract_uses_production_matrix_and_angular_labels():
    assert status.MATRIX_NAMES == tuple('{:02d}'.format(index) for index in range(16))
    assert status.CARTESIAN_VELOCITY_COMMAND_NAMES == (
        'tx', 'ty', 'tz', 'omega_x', 'omega_y', 'omega_z')
    for arm_id in status.ARM_IDS:
        assert '{}_ee_cartesian_position/04'.format(arm_id) in (
            status.EXPECTED_COMMAND_INTERFACES)
        assert '{}_ee_cartesian_position/15'.format(arm_id) in (
            status.EXPECTED_STATE_INTERFACES)
        assert '{}_ee_cartesian_velocity/omega_x'.format(arm_id) in (
            status.EXPECTED_COMMAND_INTERFACES)
        assert '{}_ee_cartesian_position/20'.format(arm_id) not in (
            status.EXPECTED_COMMAND_INTERFACES)
        assert '{}_ee_cartesian_velocity/wx'.format(arm_id) not in (
            status.EXPECTED_COMMAND_INTERFACES)


@pytest.mark.parametrize(('family', 'actual_name', 'legacy_near_miss'), (
    ('command', 'panda1_ee_cartesian_position/04', 'panda1_ee_cartesian_position/20'),
    ('command', 'panda2_ee_cartesian_velocity/omega_x',
     'panda2_ee_cartesian_velocity/wx'),
    ('state', 'panda1_ee_cartesian_velocity/04', 'panda1_ee_cartesian_velocity/20'),
))
def test_equal_count_legacy_interface_near_misses_are_rejected(
        family, actual_name, legacy_near_miss):
    diagnostics, controllers, interfaces, components = _responses()
    global_interfaces = getattr(interfaces, family + '_interfaces')
    component_interfaces = getattr(components.component[1], family + '_interfaces')
    next(interface for interface in global_interfaces
         if interface.name == actual_name).name = legacy_near_miss
    next(interface for interface in component_interfaces
         if interface.name == actual_name).name = legacy_near_miss
    assert len(global_interfaces) == (
        len(status.EXPECTED_COMMAND_INTERFACES) if family == 'command'
        else len(status.EXPECTED_STATE_INTERFACES))
    with pytest.raises(status.StatusError, match='missing=.*{}.*unknown=.*{}'.format(
            actual_name, legacy_near_miss)):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)


@pytest.mark.parametrize('name', sorted(status.REVIEWED_CONTROLLERS))
def test_reviewed_active_controller_claims_exact_interfaces(name):
    controller = _reviewed_controller(name)
    result = status.validate_status_snapshot(*_responses(controller))
    assert result['ok']
    assert result['controllers'][0]['name'] == name


def test_real_controller_state_message_preserves_exact_required_interface_contract():
    name = 'dual_arm_joint_velocity_controller'
    commands, states = status.expected_controller_interfaces(name)
    controller = ControllerState()
    controller.name = name
    controller.type = status.REVIEWED_CONTROLLERS[name]
    controller.state = 'active'
    controller.claimed_interfaces = list(commands)
    controller.required_command_interfaces = list(commands)
    controller.required_state_interfaces = list(states)
    result = status.validate_status_snapshot(*_responses(controller))
    assert result['ok']
    controller.required_command_interfaces = list(reversed(commands))
    with pytest.raises(status.StatusError, match='required command interfaces differ'):
        status.validate_status_snapshot(*_responses(controller))


@pytest.mark.parametrize('name', sorted(status.REVIEWED_CONTROLLERS))
@pytest.mark.parametrize('field', (
    'required_command_interfaces',
    'required_state_interfaces',
))
def test_reviewed_controller_requires_exact_interface_arrays(name, field):
    controller = _reviewed_controller(name)
    values = list(getattr(controller, field))
    if values:
        mutations = [[]]
        reordered = list(values)
        reordered[0], reordered[1] = reordered[1], reordered[0]
        mutations.append(reordered)
    else:
        mutations = [[sorted(status.EXPECTED_STATE_INTERFACES)[0]]]
    for mutation in mutations:
        controller = _reviewed_controller(name)
        setattr(controller, field, mutation)
        with pytest.raises(status.StatusError, match=field.replace('_', ' ') + ' differ'):
            status.validate_status_snapshot(*_responses(controller))


def test_diagnostic_error_is_observable_without_corrupting_snapshot():
    result = status.validate_status_snapshot(*_responses(diagnostic_level=2))
    assert not result['ok']
    assert result['diagnostic_error']


def test_rejects_missing_duplicate_or_wrong_diagnostic_identity():
    diagnostics, controllers, interfaces, components = _responses()
    diagnostics.status = diagnostics.status[:-1]
    with pytest.raises(status.StatusError, match='missing'):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)

    diagnostics, controllers, interfaces, components = _responses()
    diagnostics.status.append(_diagnostic('panda1'))
    with pytest.raises(status.StatusError, match='duplicate'):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)

    diagnostics, controllers, interfaces, components = _responses()
    diagnostics.status[1].hardware_id = 'panda2'
    with pytest.raises(status.StatusError, match='hardware_id'):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)


def test_rejects_changed_32_key_contract_and_duplicate_values():
    diagnostics, controllers, interfaces, components = _responses()
    diagnostics.status[1].values.pop()
    with pytest.raises(status.StatusError, match='keys differ'):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)

    diagnostics, controllers, interfaces, components = _responses()
    diagnostics.status[1].values.append(diagnostics.status[1].values[0])
    with pytest.raises(status.StatusError, match='duplicate key'):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)


def test_rejects_component_interface_and_claim_disagreement():
    diagnostics, controllers, interfaces, components = _responses()
    components.component[1].plugin_name = 'mock_components/GenericSystem'
    with pytest.raises(status.StatusError, match='identity'):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)

    diagnostics, controllers, interfaces, components = _responses()
    interfaces.command_interfaces.pop()
    with pytest.raises(status.StatusError, match='interfaces'):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)

    controller = _reviewed_controller('dual_arm_joint_hold_controller')
    diagnostics, controllers, interfaces, components = _responses(controller)
    claimed_name = controller.claimed_interfaces[0]
    next(interface for interface in interfaces.command_interfaces
         if interface.name == claimed_name).is_claimed = False
    with pytest.raises(status.StatusError, match='views disagree|claim flag'):
        status.validate_status_snapshot(diagnostics, controllers, interfaces, components)


def test_rejects_unreviewed_claims_name_type_aliasing_and_wrong_lifecycle_claims():
    command = sorted(status.EXPECTED_COMMAND_INTERFACES)[0]
    unreviewed = SimpleNamespace(
        name='legacy', type='legacy/MovingController', state='active',
        claimed_interfaces=[command], required_command_interfaces=[command],
        required_state_interfaces=[])
    with pytest.raises(status.StatusError, match='unreviewed'):
        status.validate_status_snapshot(*_responses(unreviewed))

    aliased = _reviewed_controller('dual_arm_joint_hold_controller')
    aliased.name = 'renamed_hold'
    with pytest.raises(status.StatusError, match='unreviewed name'):
        status.validate_status_snapshot(*_responses(aliased))

    inactive = _reviewed_controller('dual_arm_joint_velocity_controller', 'inactive')
    inactive.claimed_interfaces = status.expected_controller_interfaces(inactive.name)[0]
    with pytest.raises(status.StatusError, match='lifecycle'):
        status.validate_status_snapshot(*_responses(inactive))


def test_rejects_active_unreviewed_required_panda_command_even_without_current_claims():
    command = sorted(status.EXPECTED_COMMAND_INTERFACES)[0]
    controller = SimpleNamespace(
        name='preactivation_gap', type='legacy/MovingController', state='active',
        claimed_interfaces=[], required_command_interfaces=[command],
        required_state_interfaces=[])
    with pytest.raises(status.StatusError, match='active unreviewed controller requires'):
        status.validate_status_snapshot(*_responses(controller))


def _callback_node(arm_mode='dual'):
    node = status.FrankaStatusNode.__new__(status.FrankaStatusNode)
    node._arm_mode = arm_mode
    node._diagnostic_array = None
    node._subscription_started_ns = 100
    return node


def test_subscription_callback_accepts_only_complete_post_subscription_array(monkeypatch):
    node = _callback_node()
    monkeypatch.setattr(status.time, 'monotonic_ns', lambda: 101)
    node._diagnostic_callback(SimpleNamespace(status=[_diagnostic('panda1')]))
    assert node._diagnostic_array is None
    complete = SimpleNamespace(status=[_diagnostic('panda1'), _diagnostic('panda2')])
    node._diagnostic_callback(complete)
    assert node._diagnostic_array == (101, complete)


def test_source_constructs_only_three_read_only_clients_and_one_diagnostic_subscription():
    source_path = Path(status.__file__)
    tree = ast.parse(source_path.read_text(encoding='utf-8'))
    clients = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and
        node.func.attr == 'create_client'
    ]
    subscriptions = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and
        node.func.attr == 'create_subscription'
    ]
    assert len(clients) == 3
    assert len(subscriptions) == 1
    text = source_path.read_text(encoding='utf-8')
    for forbidden in (
            '/switch_controller', '/load_controller', '/configure_controller',
            '/unload_controller', '/set_hardware_component_state', '/recover',
            '/set_parameters'):
        assert forbidden not in text


@pytest.mark.parametrize(('boundary', 'expected_message'), (
    ('init', 'status runtime failed'),
    ('construct', 'status runtime failed'),
    ('collect_runtime', 'status runtime failed'),
    ('collect_status', 'status unavailable or inconsistent'),
    ('destroy', 'status cleanup failed'),
    ('shutdown', 'status cleanup failed'),
))
def test_main_sanitizes_every_runtime_and_cleanup_boundary(
        monkeypatch, capsys, boundary, expected_message):
    secret = RuntimeError('private /tmp/status-secret')

    class FakeNode:
        def collect(self, _timeout):
            if boundary == 'collect_runtime':
                raise secret
            if boundary == 'collect_status':
                raise status.StatusError('private /tmp/status-detail')
            return {'ok': True}

        def destroy_node(self):
            if boundary == 'destroy':
                raise secret

    def initialize(**_kwargs):
        if boundary == 'init':
            raise secret

    def construct(_arm_mode='dual'):
        if boundary == 'construct':
            raise secret
        return FakeNode()

    def shutdown():
        if boundary == 'shutdown':
            raise secret

    monkeypatch.setattr(status.rclpy, 'init', initialize)
    monkeypatch.setattr(status.rclpy, 'try_shutdown', shutdown)
    monkeypatch.setattr(status, 'FrankaStatusNode', construct)
    assert status.main(['--timeout', '1']) == 3
    captured = capsys.readouterr()
    assert captured.out == ''
    assert json.loads(captured.err) == {'error': expected_message, 'ok': False}
    assert 'Traceback' not in captured.err
    assert '/tmp/' not in captured.err


def test_main_preserves_primary_failure_category_when_both_teardowns_fail(
        monkeypatch, capsys):
    class FakeNode:
        def __init__(self, _arm_mode='dual'):
            pass

        def collect(self, _timeout):
            raise status.StatusError('primary private detail')

        def destroy_node(self):
            raise RuntimeError('destroy private detail')

    monkeypatch.setattr(status.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(
        status.rclpy, 'try_shutdown', lambda: (_ for _ in ()).throw(
            RuntimeError('shutdown private detail')))
    monkeypatch.setattr(status, 'FrankaStatusNode', FakeNode)
    assert status.main(['--timeout', '1']) == 3
    captured = capsys.readouterr()
    assert json.loads(captured.err) == {
        'error': 'status unavailable or inconsistent', 'ok': False}
    assert 'private' not in captured.err


# --- One-arm mode (F-10e) ------------------------------------------------------------------
#
# The one-arm bringup (launch/real/one_arm_franka.launch.py) chooses its arm ID at launch time,
# so franka_status cannot know it and must auto-detect it -- and must refuse rather than guess
# when the observation is ambiguous. Everything below exercises that detection and the
# mode-scoped validation; the dual-mode tests above are unchanged and still assert the exact
# prior contract.


def _snapshot(arm_ids, controller=None, diagnostic_level=0, diagnostic_arm_ids=None,
              extra_command_interfaces=(), extra_state_interfaces=()):
    """Build one (diagnostics, controllers, interfaces, components) tuple for the given arms."""
    expected_commands, expected_states = status.expected_hardware_interfaces_for(arm_ids)
    command_names = sorted(set(expected_commands) | set(extra_command_interfaces))
    state_names = sorted(set(expected_states) | set(extra_state_interfaces))
    claims = set(controller.claimed_interfaces) if controller is not None else set()
    if diagnostic_arm_ids is None:
        diagnostic_arm_ids = arm_ids
    component = SimpleNamespace(
        name=status.FRANKA_COMPONENT_NAME,
        plugin_name=status.FRANKA_COMPONENT_PLUGIN,
        state=SimpleNamespace(id=3, label='active'),
        command_interfaces=[_interface(name, name in claims) for name in command_names],
        state_interfaces=[_interface(name) for name in state_names],
    )
    return (
        SimpleNamespace(status=[
            SimpleNamespace(name='unrelated: task', hardware_id='other', level=0,
                            message='ignored', values=[]),
        ] + [_diagnostic(arm_id, diagnostic_level) for arm_id in diagnostic_arm_ids]),
        SimpleNamespace(controller=[] if controller is None else [controller]),
        SimpleNamespace(
            command_interfaces=[_interface(name, name in claims) for name in command_names],
            state_interfaces=[_interface(name) for name in state_names]),
        SimpleNamespace(component=[
            SimpleNamespace(name='OtherSystem', plugin_name='other/Plugin'), component]),
    )


def _single_reviewed_controller(name, arm_id, lifecycle_state='active'):
    commands, states = status.expected_single_controller_interfaces(name, arm_id)
    return SimpleNamespace(
        name=name,
        type=status.REVIEWED_CONTROLLERS[name],
        state=lifecycle_state,
        claimed_interfaces=commands if lifecycle_state == 'active' else [],
        required_command_interfaces=commands,
        required_state_interfaces=states,
    )


def test_dual_expected_interfaces_come_from_the_same_body_as_the_one_arm_sets():
    assert status.expected_hardware_interfaces_for(status.ARM_IDS) == (
        status.EXPECTED_COMMAND_INTERFACES, status.EXPECTED_STATE_INTERFACES)
    assert status.expected_hardware_interfaces() == (
        status.EXPECTED_COMMAND_INTERFACES, status.EXPECTED_STATE_INTERFACES)


@pytest.mark.parametrize('arm_id', status.ONE_ARM_MODE_IDS)
def test_one_arm_interface_sets_are_exactly_that_arms_half_of_the_dual_sets(arm_id):
    commands, states = status.expected_hardware_interfaces_for((arm_id,))
    # 7 joints x 3 command interfaces + 16 Cartesian pose + 6 Cartesian velocity = 43
    # 7 joints x 3 state interfaces + 16 pose + 16 velocity + 2 pointers = 55
    assert len(commands) == 43
    assert len(states) == 55
    assert commands < status.EXPECTED_COMMAND_INTERFACES
    assert states < status.EXPECTED_STATE_INTERFACES
    other = [candidate for candidate in status.ARM_IDS if candidate != arm_id]
    assert not any(name.startswith(tuple(other)) for name in commands | states)


def test_arm_mode_validation_accepts_exactly_dual_and_single():
    assert status.ARM_MODES == ('dual', 'single')
    assert status.DEFAULT_ARM_MODE == 'dual'
    for arm_mode in status.ARM_MODES:
        assert status.validate_arm_mode(arm_mode) == arm_mode
    for rejected in ('both', 'DUAL', '', None, 1):
        with pytest.raises(status.StatusError, match='arm mode'):
            status.validate_arm_mode(rejected)


@pytest.mark.parametrize('arm_id', status.ONE_ARM_MODE_IDS)
def test_one_arm_snapshot_is_valid_stable_and_names_the_detected_arm(arm_id):
    responses = _snapshot((arm_id,))
    first = status.validate_status_snapshot(*responses, arm_mode='single')
    second = status.validate_status_snapshot(*responses, arm_mode='single')
    assert first == second
    assert first['ok']
    assert first['arm_id'] == arm_id
    assert first['arm_mode'] == 'single'
    assert [entry['arm_id'] for entry in first['diagnostics']] == [arm_id]
    assert first['hardware']['command_interface_count'] == 43
    assert first['hardware']['state_interface_count'] == 55


def test_default_dual_output_adds_no_arm_mode_or_arm_id_key():
    result = status.validate_status_snapshot(*_responses())
    assert 'arm_mode' not in result
    assert 'arm_id' not in result
    assert set(result) == {
        'controller_manager', 'controllers', 'diagnostic_error', 'diagnostics', 'hardware', 'ok'}
    assert status.validate_status_snapshot(*_responses(), arm_mode='dual') == result


def test_one_arm_mode_refuses_an_array_with_no_canonical_arm_diagnostic():
    responses = _snapshot(('panda1',), diagnostic_arm_ids=())
    with pytest.raises(status.StatusError, match='no canonical Franka arm diagnostic'):
        status.validate_status_snapshot(*responses, arm_mode='single')


def test_one_arm_mode_refuses_an_ambiguous_two_arm_array_instead_of_picking_one():
    responses = _responses()
    with pytest.raises(status.StatusError, match='ambiguous'):
        status.validate_status_snapshot(*responses, arm_mode='single')
    assert status.observed_arm_ids(responses[0]) == ('panda1', 'panda2')
    assert status.resolve_arm_ids(responses[0], 'dual') == status.ARM_IDS


def test_dual_mode_still_refuses_a_one_arm_bringup():
    responses = _snapshot(('panda1',))
    with pytest.raises(status.StatusError, match='missing a canonical arm status'):
        status.validate_status_snapshot(*responses)


@pytest.mark.parametrize('family', ('command', 'state'))
def test_one_arm_mode_reports_a_stray_other_arm_interface_as_unknown(family):
    stray = 'panda2_joint1/position' if family == 'state' else 'panda2_joint1/effort'
    responses = _snapshot(
        ('panda1',),
        extra_command_interfaces=() if family == 'state' else (stray,),
        extra_state_interfaces=(stray,) if family == 'state' else ())
    with pytest.raises(status.StatusError, match='unknown.*{}'.format(stray)):
        status.validate_status_snapshot(*responses, arm_mode='single')


def test_one_arm_mode_reports_a_missing_interface_as_missing():
    diagnostics, controllers, interfaces, components = _snapshot(('panda2',))
    dropped = interfaces.command_interfaces.pop().name
    with pytest.raises(status.StatusError, match='missing.*{}'.format(dropped)):
        status.validate_status_snapshot(
            diagnostics, controllers, interfaces, components, arm_mode='single')


@pytest.mark.parametrize('arm_id', status.ONE_ARM_MODE_IDS)
@pytest.mark.parametrize('name', sorted(status.REVIEWED_CONTROLLERS))
def test_one_arm_reviewed_active_controller_claims_exact_one_arm_interfaces(name, arm_id):
    controller = _single_reviewed_controller(name, arm_id)
    result = status.validate_status_snapshot(
        *_snapshot((arm_id,), controller), arm_mode='single')
    assert result['ok']
    assert result['controllers'][0]['name'] == name
    assert len(controller.required_command_interfaces) == 7


@pytest.mark.parametrize('name', sorted(status.REVIEWED_CONTROLLERS))
def test_one_arm_mode_rejects_a_dual_shaped_reviewed_controller(name):
    controller = _reviewed_controller(name)
    responses = _snapshot(('panda1',), controller)
    with pytest.raises(status.StatusError, match='required command interfaces differ'):
        status.validate_status_snapshot(*responses, arm_mode='single')


def test_one_arm_mode_rejects_the_other_arms_reviewed_controller():
    controller = _single_reviewed_controller('dual_arm_joint_hold_controller', 'panda2')
    responses = _snapshot(('panda1',), controller)
    with pytest.raises(status.StatusError, match='required command interfaces differ'):
        status.validate_status_snapshot(*responses, arm_mode='single')


def test_dual_mode_rejects_a_one_arm_shaped_reviewed_controller():
    controller = _single_reviewed_controller('dual_arm_joint_hold_controller', 'panda1')
    diagnostics, _controllers, interfaces, components = _responses()
    with pytest.raises(status.StatusError, match='required command interfaces differ'):
        status.validate_status_snapshot(
            diagnostics, SimpleNamespace(controller=[controller]), interfaces, components)


def test_one_arm_mode_still_reports_lifecycle_and_diagnostic_error_levels():
    error_level = status.validate_status_snapshot(
        *_snapshot(('panda2',), diagnostic_level=2), arm_mode='single')
    assert not error_level['ok']
    assert error_level['diagnostic_error']

    diagnostics, controllers, interfaces, components = _snapshot(('panda2',))
    components.component[1].state = SimpleNamespace(id=2, label='inactive')
    with pytest.raises(status.StatusError, match='lifecycle disagrees'):
        status.validate_status_snapshot(
            diagnostics, controllers, interfaces, components, arm_mode='single')


def test_one_arm_subscription_callback_accepts_one_arm_and_forwards_ambiguity(monkeypatch):
    monkeypatch.setattr(status.time, 'monotonic_ns', lambda: 101)
    node = _callback_node('single')
    node._diagnostic_callback(SimpleNamespace(status=[
        SimpleNamespace(name='unrelated: task')]))
    assert node._diagnostic_array is None
    one_arm = SimpleNamespace(status=[_diagnostic('panda2')])
    node._diagnostic_callback(one_arm)
    assert node._diagnostic_array == (101, one_arm)
    # An ambiguous array is delivered, not filtered, so resolve_arm_ids can refuse it loudly
    # instead of the collection timing out with an indistinguishable message.
    ambiguous = SimpleNamespace(status=[_diagnostic('panda1'), _diagnostic('panda2')])
    node._diagnostic_callback(ambiguous)
    assert node._diagnostic_array == (101, ambiguous)
    with pytest.raises(status.StatusError, match='ambiguous'):
        status.resolve_arm_ids(ambiguous, 'single')


def test_node_constructor_rejects_an_unknown_arm_mode_before_touching_ros(monkeypatch):
    constructed = []
    monkeypatch.setattr(
        status.Node, '__init__', lambda self, name: constructed.append(name))
    with pytest.raises(status.StatusError, match='arm mode'):
        status.FrankaStatusNode('both')
    # No ROS node is created for an unusable mode.
    assert constructed == []


@pytest.mark.parametrize(('argv', 'expected_arm_mode'), (
    ([], 'dual'),
    (['--arm-mode', 'dual'], 'dual'),
    (['--arm-mode', 'single'], 'single'),
))
def test_cli_passes_the_selected_arm_mode_to_the_node(monkeypatch, capsys, argv,
                                                      expected_arm_mode):
    captured = {}

    class FakeNode:
        def __init__(self, arm_mode='dual'):
            captured['arm_mode'] = arm_mode

        def collect(self, _timeout):
            return {'arm_mode': captured['arm_mode'], 'ok': True}

        def destroy_node(self):
            pass

    monkeypatch.setattr(status.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(status.rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(status, 'FrankaStatusNode', FakeNode)
    assert status.main(['--timeout', '1'] + argv) == 0
    assert captured['arm_mode'] == expected_arm_mode
    assert json.loads(capsys.readouterr().out) == {
        'arm_mode': expected_arm_mode, 'ok': True}


def test_cli_rejects_an_unknown_arm_mode_via_argparse():
    with pytest.raises(SystemExit) as caught:
        status.main(['--arm-mode', 'both'])
    assert caught.value.code == 2
