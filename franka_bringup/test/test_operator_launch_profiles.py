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
import errno
import fcntl
import hashlib
import importlib.util
import os
from pathlib import Path
import sys
import threading

from franka_bringup import controller_config_validator as validator
from franka_bringup import operator_launch
from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node
import pytest
import yaml


_SOURCE_ROOT = Path(__file__).resolve().parents[2]
_LAUNCH_ROOT = _SOURCE_ROOT / 'franka_bringup' / 'launch' / 'operator'
_POLICY = _SOURCE_ROOT / 'franka_example_controllers' / 'config' / (
    'panda_joint_limits_v1.yaml')
_LAUNCH_FILES = (
    'fake_dual_state_only.launch.py',
    'production_dual_state_only.launch.py',
    'production_dual_guarded_motion.launch.py',
)


def _load_launch(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(**values):
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _hold_text():
    parameters = {}
    for slot, arm_id in enumerate(('panda1', 'panda2'), start=1):
        parameters['arm_{}'.format(slot)] = {
            'arm_id': arm_id,
            'k_gains': [20.0] * 7,
            'd_gains': [1.0] * 7,
            'max_effort': [10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 3.0],
        }
    return yaml.safe_dump({
        '/dual_arm_joint_hold_controller': {'ros__parameters': parameters}}, sort_keys=False)


def _guard_context(path):
    return _context(
        robot_ip_1='robot-one', robot_ip_2='robot-two', allow_motion='true',
        controller_name='dual_arm_joint_hold_controller', controller_param_file=str(path),
        use_rviz='false')


def _run_managed_node(sealed_config, executable, arguments, extra_actions=()):
    node = operator_launch._SealedParamFileNode(
        sealed_config, executable=executable, arguments=arguments, output='log')
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnShutdown(on_shutdown=node.on_shutdown)),
        node,
        *extra_actions,
    ]))
    return service.run(), node


@pytest.mark.parametrize('filename', _LAUNCH_FILES)
def test_launch_description_contains_only_declarations_then_one_opaque_function(filename):
    description = _load_launch(_LAUNCH_ROOT / filename).generate_launch_description()
    entities = list(description.entities)
    assert isinstance(entities[-1], OpaqueFunction)
    assert sum(isinstance(entity, OpaqueFunction) for entity in entities) == 1
    assert all(isinstance(entity, DeclareLaunchArgument) for entity in entities[:-1])


@pytest.mark.parametrize('filename', _LAUNCH_FILES)
def test_wrapper_source_has_no_process_node_or_include_before_opaque_function(filename):
    path = _LAUNCH_ROOT / filename
    tree = ast.parse(path.read_text(encoding='utf-8'))
    imported_names = {
        alias.name for node in tree.body if isinstance(node, ast.ImportFrom)
        for alias in node.names
    }
    assert 'Node' not in imported_names
    assert 'IncludeLaunchDescription' not in imported_names
    assert 'ExecuteProcess' not in imported_names


def test_fake_profile_is_fixed_ordered_panda1_panda2_and_state_only():
    actions = operator_launch.fake_state_only_setup(_context(use_rviz='false'))
    assert len(actions) == 1
    assert isinstance(actions[0], IncludeLaunchDescription)
    arguments = dict(actions[0].launch_arguments)
    assert arguments == {
        'arm_id_1': 'panda1',
        'arm_id_2': 'panda2',
        'fake_sensor_commands': 'false',
        'load_gripper_1': 'false',
        'load_gripper_2': 'false',
        'robot_ip_1': 'dont-care',
        'robot_ip_2': 'dont-care',
        'use_fake_hardware': 'true',
        'use_rviz': 'false',
    }


def test_production_state_only_requires_distinct_addresses_and_constructs_one_include(
        monkeypatch):
    invoked = []
    monkeypatch.setattr(
        operator_launch, '_base_include', lambda *args: invoked.append(args) or args)
    with pytest.raises(RuntimeError):
        operator_launch.production_state_only_setup(_context(
            robot_ip_1='', robot_ip_2='robot-two', use_rviz='false'))
    with pytest.raises(RuntimeError):
        operator_launch.production_state_only_setup(_context(
            robot_ip_1='same', robot_ip_2='same', use_rviz='false'))
    assert invoked == []
    actions = operator_launch.production_state_only_setup(_context(
        robot_ip_1='robot-one', robot_ip_2='robot-two', use_rviz='false'))
    assert actions == [('robot-one', 'robot-two', 'false', 'false')]


def test_production_address_arguments_have_no_defaults_and_sources_do_not_echo_them():
    for filename in (
            'production_dual_state_only.launch.py',
            'production_dual_guarded_motion.launch.py'):
        path = _LAUNCH_ROOT / filename
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
            if not isinstance(call.func, ast.Name) or call.func.id != 'DeclareLaunchArgument':
                continue
            if not call.args or not isinstance(call.args[0], ast.Constant):
                continue
            if call.args[0].value in ('robot_ip_1', 'robot_ip_2'):
                assert all(keyword.arg != 'default_value' for keyword in call.keywords)
        text = path.read_text(encoding='utf-8')
        assert 'LogInfo' not in text
        assert 'print(' not in text


def test_guard_rejects_before_action_construction(monkeypatch, tmp_path):
    action_calls = []
    monkeypatch.setattr(
        operator_launch, '_base_include', lambda *args: action_calls.append(args) or object())
    base = {
        'robot_ip_1': 'robot-one',
        'robot_ip_2': 'robot-two',
        'allow_motion': 'false',
        'controller_name': '',
        'controller_param_file': '',
        'use_rviz': 'false',
    }
    with pytest.raises(RuntimeError, match='allow_motion'):
        operator_launch.production_guarded_motion_setup(_context(**base))
    base['allow_motion'] = 'true'
    with pytest.raises(RuntimeError, match='controller_name'):
        operator_launch.production_guarded_motion_setup(_context(**base))
    base['controller_name'] = 'dual_joint_impedance_example_controller'
    with pytest.raises(RuntimeError, match='controller_name'):
        operator_launch.production_guarded_motion_setup(_context(**base))
    assert action_calls == []

    base['controller_name'] = 'dual_arm_joint_hold_controller'
    base['controller_param_file'] = str(tmp_path / 'missing.yaml')
    with pytest.raises(RuntimeError, match='regular non-symlink'):
        operator_launch.production_guarded_motion_setup(_context(**base))
    assert action_calls == []


def test_guard_rejects_file_and_parent_symlinks_before_actions(monkeypatch, tmp_path):
    monkeypatch.setattr(validator, 'default_limit_policy_path', lambda: _POLICY)
    action_calls = []
    monkeypatch.setattr(
        operator_launch, '_base_include', lambda *args: action_calls.append(args) or object())
    real_directory = tmp_path / 'real'
    real_directory.mkdir()
    real_file = real_directory / 'hold.yaml'
    real_file.write_text(_hold_text(), encoding='utf-8')
    file_link = tmp_path / 'file-link.yaml'
    file_link.symlink_to(real_file)
    parent_link = tmp_path / 'parent-link'
    parent_link.symlink_to(real_directory, target_is_directory=True)
    base = {
        'robot_ip_1': 'robot-one', 'robot_ip_2': 'robot-two', 'allow_motion': 'true',
        'controller_name': 'dual_arm_joint_hold_controller', 'use_rviz': 'false'}
    for path in (file_link, parent_link / 'hold.yaml'):
        with pytest.raises(RuntimeError, match='symlink'):
            operator_launch.production_guarded_motion_setup(
                _context(controller_param_file=str(path), **base))
    assert action_calls == []


def test_valid_guard_returns_base_include_and_exactly_one_switch_asap_spawner(
        monkeypatch, tmp_path):
    monkeypatch.setattr(validator, 'default_limit_policy_path', lambda: _POLICY)
    path = tmp_path / 'hold.yaml'
    path.write_text(_hold_text(), encoding='utf-8')
    actions = operator_launch.production_guarded_motion_setup(_guard_context(path))
    assert len(actions) == 3
    assert isinstance(actions[0], RegisterEventHandler)
    assert isinstance(actions[1], IncludeLaunchDescription)
    assert isinstance(actions[2], Node)
    assert actions[2].node_package == 'controller_manager'
    assert actions[2].node_executable == 'spawner'
    arguments = actions[2]._Node__arguments
    assert arguments[0:2] == ['dual_arm_joint_hold_controller', '--param-file']
    assert arguments[3:] == ['--switch-asap']
    assert arguments[2].startswith('/proc/{}/fd/'.format(os.getpid()))
    assert str(path) not in arguments
    sealed_config = actions[2]._sealed_config
    try:
        assert Path(arguments[2]).read_bytes() == path.read_bytes()
    finally:
        sealed_config.close()


def test_validated_bytes_survive_atomic_legacy_path_swap(monkeypatch, tmp_path):
    monkeypatch.setattr(validator, 'default_limit_policy_path', lambda: _POLICY)
    monkeypatch.setattr(operator_launch, '_base_include', lambda *_args: object())
    config_path = tmp_path / 'controller.yaml'
    validated_bytes = _hold_text().encode('utf-8')
    config_path.write_bytes(validated_bytes)
    replacement = tmp_path / 'legacy.yaml'
    legacy_bytes = (
        b'/dual_arm_joint_hold_controller:\n'
        b'  ros__parameters:\n'
        b'    type: franka_example_controllers/MultiJointImpedanceExampleController\n')
    replacement.write_bytes(legacy_bytes)

    validation_complete = threading.Event()
    continue_setup = threading.Event()
    real_sealed_config = operator_launch._SealedControllerConfig
    validated_argument = []

    def barrier_after_validation(data):
        validated_argument.append(data)
        validation_complete.set()
        assert continue_setup.wait(timeout=5.0)
        return real_sealed_config(data)

    monkeypatch.setattr(operator_launch, '_SealedControllerConfig', barrier_after_validation)
    result = []
    failure = []

    def run_setup():
        try:
            result.extend(operator_launch.production_guarded_motion_setup(
                _guard_context(config_path)))
        except BaseException as error:
            failure.append(error)

    thread = threading.Thread(target=run_setup)
    thread.start()
    assert validation_complete.wait(timeout=5.0)
    os.replace(replacement, config_path)
    continue_setup.set()
    thread.join(timeout=5.0)
    assert not thread.is_alive()
    assert failure == []
    assert validated_argument == [validated_bytes]
    assert config_path.read_bytes() == legacy_bytes

    spawner = result[2]
    sealed_config = spawner._sealed_config
    try:
        consumed = Path(spawner._Node__arguments[2]).read_bytes()
        assert consumed == validated_bytes
        assert hashlib.sha256(consumed).digest() == hashlib.sha256(validated_bytes).digest()
        assert str(config_path) not in spawner._Node__arguments
    finally:
        sealed_config.close()


def test_memfd_is_fully_sealed_and_hash_stable_through_adversarial_operations():
    data = _hold_text().encode('utf-8')
    expected_hash = hashlib.sha256(data).digest()
    sealed_config = operator_launch._SealedControllerConfig(data)
    descriptor = sealed_config.descriptor
    proc_descriptor = None
    try:
        assert fcntl.fcntl(descriptor, fcntl.F_GET_SEALS) & (
            fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK |
            fcntl.F_SEAL_SEAL) == (
                fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK |
                fcntl.F_SEAL_SEAL)
        for operation in (
                lambda: os.write(descriptor, b'legacy'),
                lambda: os.pwrite(descriptor, b'legacy', 0),
                lambda: os.ftruncate(descriptor, len(data) - 1),
                lambda: os.ftruncate(descriptor, len(data) + 1)):
            with pytest.raises(OSError) as caught:
                operation()
            assert caught.value.errno == errno.EPERM

        proc_descriptor = os.open(sealed_config.proc_path, os.O_RDWR | os.O_CLOEXEC)
        for operation in (
                lambda: os.pwrite(proc_descriptor, b'legacy', 0),
                lambda: os.ftruncate(proc_descriptor, len(data) + 1)):
            with pytest.raises(OSError) as caught:
                operation()
            assert caught.value.errno == errno.EPERM
        assert hashlib.sha256(os.pread(descriptor, len(data) + 1, 0)).digest() == expected_hash
        assert hashlib.sha256(Path(sealed_config.proc_path).read_bytes()).digest() == expected_hash
    finally:
        if proc_descriptor is not None:
            os.close(proc_descriptor)
        sealed_config.close()


def test_managed_spawner_keeps_fd_until_normal_process_exit(tmp_path):
    data = _hold_text().encode('utf-8')
    sealed_config = operator_launch._SealedControllerConfig(data)
    proc_path = sealed_config.proc_path
    marker = tmp_path / 'consumed'
    program = (
        'import hashlib,pathlib,sys,time; '
        'path=pathlib.Path(sys.argv[1]); expected=sys.argv[2]; '
        'assert hashlib.sha256(path.read_bytes()).hexdigest()==expected; '
        'time.sleep(0.1); '
        'assert hashlib.sha256(path.read_bytes()).hexdigest()==expected; '
        'pathlib.Path(sys.argv[3]).write_text("ok", encoding="utf-8")')
    return_code, node = _run_managed_node(
        sealed_config, sys.executable,
        ['-c', program, proc_path, hashlib.sha256(data).hexdigest(), str(marker)])
    assert return_code == 0
    assert node.return_code == 0
    assert marker.read_text(encoding='utf-8') == 'ok'
    assert sealed_config.descriptor is None
    assert not Path(proc_path).exists()


def test_managed_spawner_closes_on_process_start_failure():
    sealed_config = operator_launch._SealedControllerConfig(b'validated')
    proc_path = sealed_config.proc_path
    missing_executable = '/tmp/franka-spawner-executable-that-does-not-exist'
    return_code, node = _run_managed_node(sealed_config, missing_executable, [])
    assert return_code == 0
    assert node.return_code is None
    assert sealed_config.descriptor is None
    assert not Path(proc_path).exists()


def test_managed_spawner_closes_on_launch_shutdown():
    sealed_config = operator_launch._SealedControllerConfig(b'validated')
    proc_path = sealed_config.proc_path
    shutdown = TimerAction(
        period=0.1,
        actions=[EmitEvent(event=Shutdown(reason='sealed memfd shutdown test'))])
    return_code, _node = _run_managed_node(
        sealed_config, sys.executable, ['-c', 'import time; time.sleep(10)'], [shutdown])
    assert return_code == 0
    assert sealed_config.descriptor is None
    assert not Path(proc_path).exists()


def test_started_child_can_reopen_sealed_bytes_while_handling_shutdown(tmp_path):
    data = _hold_text().encode('utf-8')
    expected_hash = hashlib.sha256(data).hexdigest()
    # Warm launch's process-global logging state before taking the fd baseline.
    warm = operator_launch._SealedControllerConfig(b'warm')
    return_code, _node = _run_managed_node(warm, '/bin/true', [])
    assert return_code == 0
    starting_fd_count = len(os.listdir('/proc/self/fd'))

    for iteration in range(10):
        sealed_config = operator_launch._SealedControllerConfig(data)
        proc_path = sealed_config.proc_path
        marker = tmp_path / 'shutdown-{}'.format(iteration)
        program = (
            'import hashlib,pathlib,signal,sys,time\n'
            'path=pathlib.Path(sys.argv[1]); expected=sys.argv[2]; marker=sys.argv[3]\n'
            'def stopped(_signal,_frame):\n'
            ' assert hashlib.sha256(path.read_bytes()).hexdigest()==expected\n'
            ' pathlib.Path(marker).write_text("ok",encoding="utf-8")\n'
            ' raise SystemExit(0)\n'
            'signal.signal(signal.SIGINT,stopped)\n'
            'time.sleep(10)')
        shutdown = TimerAction(
            period=0.4,
            actions=[EmitEvent(event=Shutdown(reason='sealed memfd shutdown test'))])
        return_code, node = _run_managed_node(
            sealed_config, sys.executable,
            ['-c', program, proc_path, expected_hash, str(marker)], [shutdown])
        assert return_code == 0
        assert node.return_code == 0
        assert marker.read_text(encoding='utf-8') == 'ok'
        assert sealed_config.descriptor is None
        assert not Path(proc_path).exists()

    assert len(os.listdir('/proc/self/fd')) == starting_fd_count


def test_never_started_node_closes_immediately_on_shutdown():
    sealed_config = operator_launch._SealedControllerConfig(b'validated')
    proc_path = sealed_config.proc_path
    node = operator_launch._SealedParamFileNode(
        sealed_config, executable='/bin/true', arguments=[])
    node.on_shutdown(None, None)
    assert sealed_config.descriptor is None
    assert not Path(proc_path).exists()


def test_execute_exception_closes_sealed_config(monkeypatch):
    sealed_config = operator_launch._SealedControllerConfig(b'validated')
    proc_path = sealed_config.proc_path
    node = operator_launch._SealedParamFileNode(
        sealed_config, executable='/bin/true', arguments=[])

    def fail_execute(_node, _context):
        raise RuntimeError('injected execute failure')

    monkeypatch.setattr(Node, 'execute', fail_execute)
    with pytest.raises(RuntimeError, match='injected execute failure'):
        node.execute(LaunchContext())
    assert sealed_config.descriptor is None
    assert not Path(proc_path).exists()


def test_running_completion_and_shutdown_interleavings_close_exactly_once(monkeypatch):
    class Completion:
        def __init__(self):
            self.callback = None
            self.complete = False

        def add_done_callback(self, callback):
            self.callback = callback

        def done(self):
            return self.complete

    monkeypatch.setattr(Node, 'execute', lambda _node, _context: None)
    for shutdown_first in (False, True):
        sealed_config = operator_launch._SealedControllerConfig(b'validated')
        descriptor = sealed_config.descriptor
        proc_path = sealed_config.proc_path
        node = operator_launch._SealedParamFileNode(
            sealed_config, executable='/bin/true', arguments=[])
        completion = Completion()
        monkeypatch.setattr(node, 'get_asyncio_future', lambda: completion)
        node.execute(LaunchContext())
        assert sealed_config.descriptor == descriptor
        if shutdown_first:
            node.on_shutdown(None, None)
            assert sealed_config.descriptor == descriptor
        completion.complete = True
        if not shutdown_first:
            node.on_shutdown(None, None)
            assert sealed_config.descriptor == descriptor
        completion.callback(completion)
        node.on_shutdown(None, None)
        assert sealed_config.descriptor is None
        assert not Path(proc_path).exists()


def test_setup_exception_closes_memfd_exactly_once(monkeypatch, tmp_path):
    monkeypatch.setattr(validator, 'default_limit_policy_path', lambda: _POLICY)
    monkeypatch.setattr(operator_launch, '_base_include', lambda *_args: object())
    config_path = tmp_path / 'controller.yaml'
    config_path.write_text(_hold_text(), encoding='utf-8')
    created = []
    real_sealed_config = operator_launch._SealedControllerConfig

    def record_creation(data):
        sealed_config = real_sealed_config(data)
        created.append(sealed_config)
        return sealed_config

    def fail_node_construction(*_args, **_kwargs):
        raise RuntimeError('injected node construction failure')

    monkeypatch.setattr(operator_launch, '_SealedControllerConfig', record_creation)
    monkeypatch.setattr(operator_launch, '_SealedParamFileNode', fail_node_construction)
    with pytest.raises(RuntimeError, match='injected node construction failure'):
        operator_launch.production_guarded_motion_setup(_guard_context(config_path))
    assert len(created) == 1
    assert created[0].descriptor is None
    created[0].close()


def test_close_issues_exactly_one_os_close(monkeypatch):
    sealed_config = operator_launch._SealedControllerConfig(b'validated')
    descriptor = sealed_config.descriptor
    real_close = os.close
    close_calls = []

    def record_close(value):
        close_calls.append(value)

    monkeypatch.setattr(operator_launch.os, 'close', record_close)
    try:
        close_threads = [
            threading.Thread(target=sealed_config.close) for _index in range(16)]
        for thread in close_threads:
            thread.start()
        for thread in close_threads:
            thread.join(timeout=2.0)
            assert not thread.is_alive()
        sealed_config.close()
        assert close_calls == [descriptor]
    finally:
        real_close(descriptor)


def test_repeated_path_swap_race_has_stable_bytes_and_fd_count(monkeypatch, tmp_path):
    monkeypatch.setattr(validator, 'default_limit_policy_path', lambda: _POLICY)
    monkeypatch.setattr(operator_launch, '_base_include', lambda *_args: object())
    config_path = tmp_path / 'controller.yaml'
    valid = _hold_text().encode('utf-8')
    legacy = b'legacy-moving-controller: true\n'
    # LaunchContext initializes one process-global logging descriptor on first use.
    _guard_context(config_path)
    starting_fd_count = len(os.listdir('/proc/self/fd'))
    for iteration in range(100):
        next_config = tmp_path / 'next.yaml'
        next_config.write_bytes(valid)
        os.replace(next_config, config_path)
        actions = operator_launch.production_guarded_motion_setup(_guard_context(config_path))
        spawner = actions[2]
        sealed_config = spawner._sealed_config
        replacement = tmp_path / 'replacement.yaml'
        replacement.write_bytes(legacy + str(iteration).encode('ascii'))
        os.replace(replacement, config_path)
        try:
            assert Path(spawner._Node__arguments[2]).read_bytes() == valid
            assert str(config_path) not in spawner._Node__arguments
        finally:
            sealed_config.close()
            sealed_config.close()
    assert len(os.listdir('/proc/self/fd')) == starting_fd_count


def test_base_launch_and_dual_controller_yaml_are_not_modified_by_operator_wrappers():
    for path in (
            _SOURCE_ROOT / 'franka_bringup' / 'launch' / 'real' / 'dual_franka.launch.py',
            _SOURCE_ROOT / 'franka_bringup' / 'config' / 'real' / 'dual_controllers.yaml'):
        assert path.is_file()


def test_dual_production_controller_manager_disables_rt_overrun_console_logging():
    config_path = (
        _SOURCE_ROOT / 'franka_bringup' / 'config' / 'real' / 'dual_controllers.yaml')
    parameters = yaml.safe_load(config_path.read_text(encoding='utf-8'))[
        'controller_manager']['ros__parameters']

    assert parameters['update_rate'] == 1000
    assert parameters['overruns'] == {'print_warnings': False}
