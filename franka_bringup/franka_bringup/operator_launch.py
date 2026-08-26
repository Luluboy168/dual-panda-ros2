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

"""Validation and action factories shared by the operator launch wrappers."""

import fcntl
import os
from pathlib import Path
import re
import stat
import threading

from franka_bringup.controller_config_validator import ControllerConfigError
from franka_bringup.controller_config_validator import REVIEWED_CONTROLLERS
from franka_bringup.controller_config_validator import validate_controller_config_text
from franka_bringup.controller_config_validator import validate_single_controller_config_text
from franka_bringup.launch_validation import validate_one_arm_mode_arm_id
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_ADDRESS_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9.:-]{0,252}$')
_MAXIMUM_CONTROLLER_CONFIG_BYTES = 65536
_REQUIRED_MEMFD_SEALS = (
    fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL)


class _SealedControllerConfig:
    """Own an immutable in-memory copy of one already-validated controller config."""

    def __init__(self, data):
        self._close_lock = threading.Lock()
        flags = os.MFD_ALLOW_SEALING | os.MFD_CLOEXEC
        descriptor = os.memfd_create('franka-controller-config', flags)
        self._descriptor = descriptor
        self._proc_path = '/proc/{}/fd/{}'.format(os.getpid(), descriptor)
        try:
            remaining = memoryview(data)
            while remaining:
                written = os.write(descriptor, remaining)
                if written <= 0:
                    raise RuntimeError('memfd write made no progress')
                remaining = remaining[written:]
            if os.fstat(descriptor).st_size != len(data):
                raise RuntimeError('memfd size differs from validated controller config')
            os.lseek(descriptor, 0, os.SEEK_SET)
            fcntl.fcntl(descriptor, fcntl.F_ADD_SEALS, _REQUIRED_MEMFD_SEALS)
            actual_seals = fcntl.fcntl(descriptor, fcntl.F_GET_SEALS)
            if actual_seals & _REQUIRED_MEMFD_SEALS != _REQUIRED_MEMFD_SEALS:
                raise RuntimeError('memfd controller config is not completely sealed')
            if os.pread(descriptor, len(data) + 1, 0) != data:
                raise RuntimeError('memfd bytes differ from validated controller config')
        except BaseException:
            self.close()
            raise

    @property
    def descriptor(self):
        return self._descriptor

    @property
    def proc_path(self):
        return self._proc_path

    def close(self):
        with self._close_lock:
            descriptor = self._descriptor
            if descriptor is None:
                return
            self._descriptor = None
        os.close(descriptor)


class _SealedParamFileNode(Node):
    """Keep a sealed config live exactly until its subprocess can no longer use it."""

    def __init__(self, sealed_config, **kwargs):
        self._sealed_config = sealed_config
        self._execution_started = False
        self._execute_finished = False
        self._completion_future = None
        self._completion_callback_attached = False
        super().__init__(**kwargs)

    def execute(self, context):
        self._execution_started = True
        try:
            result = super().execute(context)
            completion = self.get_asyncio_future()
            if completion is None:
                self._sealed_config.close()
            else:
                self._completion_future = completion
                completion.add_done_callback(self._on_process_complete)
                self._completion_callback_attached = True
            return result
        except BaseException:
            self._sealed_config.close()
            raise
        finally:
            self._execute_finished = True

    def _on_process_complete(self, _future):
        self._sealed_config.close()

    def on_shutdown(self, _event, _context):
        # A started process may still reopen the parameter path while handling
        # its shutdown signal.  Its completion future is the only safe point at
        # which the parent-owned descriptor can be released.
        if not self._execution_started:
            self._sealed_config.close()
        elif not self._execute_finished:
            pass
        elif not self._completion_callback_attached:
            self._sealed_config.close()
        return None


def _literal_boolean(context, name):
    value = LaunchConfiguration(name).perform(context)
    if value not in ('true', 'false'):
        raise RuntimeError('{} must be the literal true or false'.format(name))
    return value


def _production_addresses(context):
    addresses = (
        LaunchConfiguration('robot_ip_1').perform(context),
        LaunchConfiguration('robot_ip_2').perform(context),
    )
    for index, address in enumerate(addresses, start=1):
        if not _ADDRESS_PATTERN.fullmatch(address):
            raise RuntimeError('robot_ip_{} is missing or malformed'.format(index))
    if addresses[0] == addresses[1]:
        raise RuntimeError('production robot addresses must be distinct')
    return addresses


def _base_include(robot_ip_1, robot_ip_2, use_fake_hardware, use_rviz):
    base_launch = PathJoinSubstitution([
        FindPackageShare('franka_bringup'), 'launch', 'real', 'dual_franka.launch.py'])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'arm_id_1': 'panda1',
            'arm_id_2': 'panda2',
            'fake_sensor_commands': 'false',
            'load_gripper_1': 'false',
            'load_gripper_2': 'false',
            'robot_ip_1': robot_ip_1,
            'robot_ip_2': robot_ip_2,
            'use_fake_hardware': use_fake_hardware,
            'use_rviz': use_rviz,
        }.items(),
    )


def fake_state_only_setup(context):
    use_rviz = _literal_boolean(context, 'use_rviz')
    return [_base_include('dont-care', 'dont-care', 'true', use_rviz)]


def production_state_only_setup(context):
    addresses = _production_addresses(context)
    use_rviz = _literal_boolean(context, 'use_rviz')
    return [_base_include(addresses[0], addresses[1], 'false', use_rviz)]


def _read_regular_validated_config(raw_path, controller_name, validate=None):
    if validate is None:
        validate = validate_controller_config_text
    path = Path(raw_path)
    if not path.is_absolute() or Path(os.path.normpath(str(path))) != path:
        raise RuntimeError('controller_param_file must be a normalized absolute path')
    try:
        if path.resolve(strict=True) != path:
            raise RuntimeError('controller_param_file must not traverse a symlink')
        descriptor = os.open(
            path, os.O_RDONLY | os.O_CLOEXEC | getattr(os, 'O_NOFOLLOW', 0))
    except (OSError, RuntimeError) as error:
        raise RuntimeError('controller_param_file must be a regular non-symlink file') from error
    try:
        before = os.fstat(descriptor)
        if (not stat.S_ISREG(before.st_mode) or
                before.st_size > _MAXIMUM_CONTROLLER_CONFIG_BYTES):
            raise RuntimeError('controller_param_file must be a bounded regular file')
        chunks = []
        remaining = _MAXIMUM_CONTROLLER_CONFIG_BYTES + 1
        while remaining:
            chunk = os.read(descriptor, remaining)
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        data = b''.join(chunks)
        if len(data) > _MAXIMUM_CONTROLLER_CONFIG_BYTES:
            raise RuntimeError('controller_param_file exceeds the fixed size limit')
        after_path = os.stat(path, follow_symlinks=False)
        if (before.st_dev, before.st_ino) != (after_path.st_dev, after_path.st_ino):
            raise RuntimeError('controller_param_file changed during validation')
    finally:
        os.close(descriptor)
    try:
        text = data.decode('utf-8')
        validate(text, controller_name)
        return data
    except (UnicodeError, ControllerConfigError) as error:
        raise RuntimeError('controller_param_file failed strict validation: {}'.format(error)) \
            from error


def production_guarded_motion_setup(context):
    addresses = _production_addresses(context)
    use_rviz = _literal_boolean(context, 'use_rviz')
    if LaunchConfiguration('allow_motion').perform(context) != 'true':
        raise RuntimeError('allow_motion must be the literal true')
    controller_name = LaunchConfiguration('controller_name').perform(context)
    if not controller_name or controller_name not in REVIEWED_CONTROLLERS:
        raise RuntimeError('controller_name must name exactly one reviewed controller')
    config_path = LaunchConfiguration('controller_param_file').perform(context)
    if not config_path:
        raise RuntimeError('controller_param_file is required')
    validated_config = _read_regular_validated_config(config_path, controller_name)

    base = _base_include(addresses[0], addresses[1], 'false', use_rviz)
    sealed_config = None
    try:
        sealed_config = _SealedControllerConfig(validated_config)
        spawner = _SealedParamFileNode(
            sealed_config,
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller_name,
                '--param-file', sealed_config.proc_path,
                '--switch-asap',
            ],
            output='screen',
        )
        cleanup = RegisterEventHandler(
            OnShutdown(on_shutdown=spawner.on_shutdown))
        return [cleanup, base, spawner]
    except BaseException:
        if sealed_config is not None:
            sealed_config.close()
        raise


# --- One-arm mode: a single physical (or fake) Panda, arm ID chosen at launch time. ---
#
# These mirror the two-arm factories above rather than generalizing them, so the two-arm code
# paths above -- and every existing dual config, topic contract, validation rule and test that
# exercises them -- keep executing unchanged.


def _one_arm_address(context):
    address = LaunchConfiguration('robot_ip').perform(context)
    if not _ADDRESS_PATTERN.fullmatch(address):
        raise RuntimeError('robot_ip is missing or malformed')
    return address


def _one_arm_id(context):
    arm_id = LaunchConfiguration('arm_id').perform(context)
    try:
        validate_one_arm_mode_arm_id(arm_id)
    except ValueError as error:
        raise RuntimeError(str(error)) from error
    return arm_id


def _single_base_include(robot_ip, arm_id, use_fake_hardware, use_rviz):
    base_launch = PathJoinSubstitution([
        FindPackageShare('franka_bringup'), 'launch', 'real', 'one_arm_franka.launch.py'])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'arm_id': arm_id,
            'fake_sensor_commands': 'false',
            'load_gripper': 'false',
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'use_rviz': use_rviz,
        }.items(),
    )


def fake_single_state_only_setup(context):
    arm_id = _one_arm_id(context)
    use_rviz = _literal_boolean(context, 'use_rviz')
    return [_single_base_include('dont-care', arm_id, 'true', use_rviz)]


def production_single_state_only_setup(context):
    arm_id = _one_arm_id(context)
    address = _one_arm_address(context)
    use_rviz = _literal_boolean(context, 'use_rviz')
    return [_single_base_include(address, arm_id, 'false', use_rviz)]


def production_single_guarded_motion_setup(context):
    arm_id = _one_arm_id(context)
    address = _one_arm_address(context)
    use_rviz = _literal_boolean(context, 'use_rviz')
    if LaunchConfiguration('allow_motion').perform(context) != 'true':
        raise RuntimeError('allow_motion must be the literal true')
    controller_name = LaunchConfiguration('controller_name').perform(context)
    if not controller_name or controller_name not in REVIEWED_CONTROLLERS:
        raise RuntimeError('controller_name must name exactly one reviewed controller')
    config_path = LaunchConfiguration('controller_param_file').perform(context)
    if not config_path:
        raise RuntimeError('controller_param_file is required')

    def _validate_single(text, name):
        return validate_single_controller_config_text(text, name, arm_id)

    validated_config = _read_regular_validated_config(
        config_path, controller_name, validate=_validate_single)

    base = _single_base_include(address, arm_id, 'false', use_rviz)
    sealed_config = None
    try:
        sealed_config = _SealedControllerConfig(validated_config)
        spawner = _SealedParamFileNode(
            sealed_config,
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller_name,
                '--param-file', sealed_config.proc_path,
                '--switch-asap',
            ],
            output='screen',
        )
        cleanup = RegisterEventHandler(
            OnShutdown(on_shutdown=spawner.on_shutdown))
        return [cleanup, base, spawner]
    except BaseException:
        if sealed_config is not None:
            sealed_config.close()
        raise
