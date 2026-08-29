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
Structural gate on every INSTALLED production controller configuration (P0-B).

``REVIEWED_CONTROLLERS``, ``allow_motion`` and the sealed ``controller_param_file`` are all
launch-time constructs. Nothing in them constrains what the shipped ``config/real/*.yaml`` files
register with ``controller_manager``, and controller_manager will happily load, configure and
activate any controller that file names -- ``ros2 control load_controller <name> --set-state
active`` is one command, needs no ``allow_motion``, no ``controller_name``, no
``controller_param_file``, and never reaches the validator.

Until 2026-08-29 ``config/real/dual_controllers.yaml`` registered
``dual_joint_impedance_example_controller`` and ``dual_joint_velocity_example_controller`` -- both
absent from ``REVIEWED_CONTROLLERS``, both with no safety envelope -- *and* supplied their complete
inline parameter sets, so that one command put an unreviewed motion controller in command of both
real arms. This module makes that hole structural rather than a matter of review discipline:

  (a) every controller registered in an installed production config is a broadcaster or one of the
      three ``REVIEWED_CONTROLLERS``, registered under its reviewed name and type; and
  (b) no parameter block in those files carries gains, limits or any other motion-shaping value
      for anything other than a broadcaster -- motion parameters may only ever arrive through the
      sealed ``controller_param_file`` at guarded-motion spawn time.

The files under test are resolved from the INSTALLED share directory, because that -- not the
source tree -- is what ``ros2 launch`` reads.
"""

import ast
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from franka_bringup.controller_config_validator import REVIEWED_CONTROLLERS
import pytest
import yaml


_SOURCE_ROOT = Path(__file__).resolve().parents[2]

# Broadcasters publish state and claim no command interface; they are the only controller types a
# production config may register without a reviewed-controller entry.
_BROADCASTER_TYPES = frozenset({
    'joint_state_broadcaster/JointStateBroadcaster',
    'franka_robot_state_broadcaster/FrankaRobotStateBroadcaster',
    'franka_robot_state_broadcaster/FrankaRobotModelBroadcaster',
})

# The only parameters a broadcaster block may carry. Anything else in a broadcaster block is a
# motion-shaping value hiding behind a state-only name.
_BROADCASTER_PARAMETER_KEYS = frozenset({'arm_id', 'frequency', 'use_sim_time'})

# controller_manager's own settings. Everything else under controller_manager.ros__parameters must
# be a controller registration (a mapping with a "type").
_CONTROLLER_MANAGER_KEYS = frozenset({
    'update_rate',
    'thread_priority',
    'overruns',
    'robot_description',
    'use_sim_time',
    'diagnostics',
})

# Substrings that mark a key as motion-shaping. Matched case-insensitively against every parameter
# key in every non-broadcaster block.
_MOTION_PARAMETER_TOKENS = (
    'gain',
    'limit',
    'stiffness',
    'damping',
    'effort',
    'velocit',
    'accel',
    'torque',
    'threshold',
    'max_',
    'min_',
    'arm_id',
    'arm_count',
    'joints',
)

# Named explicitly so this file also stands as a regression pin for the exact P0-B finding.
_PURGED_CONTROLLERS = (
    'dual_joint_impedance_example_controller',
    'dual_joint_velocity_example_controller',
    'franka_example_controllers/MultiJointImpedanceExampleController',
    'franka_example_controllers/DualJointVelocityExampleController',
)

_YAML_FILENAME = re.compile(r'^[A-Za-z0-9_]+\.yaml$')


def _installed_share() -> Path:
    share = Path(get_package_share_directory('franka_bringup'))
    assert share.is_dir(), 'franka_bringup is not installed; run colcon build first'
    return share


def _production_config_dirs(share: Path):
    """Config directories that a production (real-hardware) launch can reach."""
    # config/mixed is deliberately excluded from install (see CMakeLists.txt EXCLUDE reason 1);
    # it is still listed here so that re-including it later cannot silently skip these checks.
    return [directory for directory in (share / 'config' / 'real', share / 'config' / 'mixed')
            if directory.is_dir()]


def _installed_production_configs(share: Path):
    paths = []
    for directory in _production_config_dirs(share):
        paths.extend(sorted(directory.glob('*.yaml')))
    return paths


def _production_launch_dirs(share: Path):
    return [directory for directory in (share / 'launch' / 'real',
                                        share / 'launch' / 'operator',
                                        share / 'launch' / 'mixed')
            if directory.is_dir()]


def _config_filenames_referenced_by_installed_launches(share: Path):
    """Every ``*.yaml`` named as a whole string literal by an installed production launch file."""
    referenced = set()
    for directory in _production_launch_dirs(share):
        for launch_file in sorted(directory.glob('*.launch.py')):
            tree = ast.parse(launch_file.read_text(encoding='utf-8'))
            for node in ast.walk(tree):
                # Whole-literal match only: a docstring that merely mentions a filename is prose,
                # not a path element.
                if isinstance(node, ast.Constant) and isinstance(node.value, str):
                    if _YAML_FILENAME.match(node.value):
                        referenced.add(node.value)
    return referenced


def _load(path: Path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _registrations(document):
    """Map controller instance name -> declared plugin type, from controller_manager."""
    parameters = document.get('controller_manager', {}).get('ros__parameters', {})
    return {name: value['type']
            for name, value in parameters.items()
            if isinstance(value, dict) and 'type' in value}


def _walk_keys(value, prefix=''):
    if isinstance(value, dict):
        for key, child in value.items():
            name = '{}.{}'.format(prefix, key) if prefix else str(key)
            yield name, str(key)
            yield from _walk_keys(child, name)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _walk_keys(child, '{}[{}]'.format(prefix, index))


_CONFIGS = _installed_production_configs(_installed_share())


def test_at_least_one_installed_production_config_is_under_test():
    # Guards against the whole suite silently passing because the glob found nothing.
    assert _CONFIGS, 'no installed production config found under share/franka_bringup/config'


@pytest.mark.parametrize('config_path', _CONFIGS, ids=lambda path: path.name)
def test_installed_config_registers_only_broadcasters_and_reviewed_controllers(config_path):
    document = _load(config_path)
    offenders = {}
    for name, controller_type in _registrations(document).items():
        if controller_type in _BROADCASTER_TYPES:
            continue
        if REVIEWED_CONTROLLERS.get(name) == controller_type:
            continue
        offenders[name] = controller_type
    assert not offenders, (
        '{} registers controllers that are neither broadcasters nor REVIEWED_CONTROLLERS: {}. '
        'A registered controller can be loaded, configured and activated against real arms with '
        'one `ros2 control load_controller --set-state active`, bypassing allow_motion and the '
        'sealed controller_param_file entirely.'.format(config_path.name, offenders))


@pytest.mark.parametrize('config_path', _CONFIGS, ids=lambda path: path.name)
def test_installed_config_carries_no_motion_parameters_outside_broadcasters(config_path):
    document = _load(config_path)
    registrations = _registrations(document)
    broadcaster_names = {name for name, controller_type in registrations.items()
                         if controller_type in _BROADCASTER_TYPES}

    for section, body in document.items():
        if section == 'controller_manager':
            continue
        assert isinstance(body, dict) and 'ros__parameters' in body, (
            '{}: unexpected top-level section {!r}'.format(config_path.name, section))
        assert section in broadcaster_names, (
            '{} parameterizes {!r}, which is not a broadcaster registered in the same file. '
            'Motion parameters may only reach a controller through the sealed '
            'controller_param_file.'.format(config_path.name, section))
        keys = set(body['ros__parameters'])
        assert keys <= _BROADCASTER_PARAMETER_KEYS, (
            '{}: broadcaster {!r} carries unexpected parameters {}'.format(
                config_path.name, section, sorted(keys - _BROADCASTER_PARAMETER_KEYS)))

    # controller_manager.ros__parameters itself: registrations plus its own settings, nothing else.
    manager = document.get('controller_manager', {}).get('ros__parameters', {})
    for key, value in manager.items():
        if isinstance(value, dict) and 'type' in value:
            assert set(value) == {'type'}, (
                '{}: registration {!r} carries inline parameters {} -- registration must be '
                'type-only'.format(config_path.name, key, sorted(set(value) - {'type'})))
            continue
        assert key in _CONTROLLER_MANAGER_KEYS, (
            '{}: unexpected controller_manager parameter {!r}'.format(config_path.name, key))
        for path_name, leaf_key in _walk_keys(value, key):
            lowered = leaf_key.lower()
            offending = [token for token in _MOTION_PARAMETER_TOKENS if token in lowered]
            assert not offending, (
                '{}: controller_manager parameter {!r} looks motion-shaping ({})'.format(
                    config_path.name, path_name, offending))


@pytest.mark.parametrize('config_path', _CONFIGS, ids=lambda path: path.name)
def test_installed_config_never_mentions_the_purged_unreviewed_controllers(config_path):
    text = config_path.read_text(encoding='utf-8')
    # The explanatory comment in dual_controllers.yaml names them on purpose; only non-comment
    # lines may not.
    body = '\n'.join(line for line in text.splitlines() if not line.lstrip().startswith('#'))
    for name in _PURGED_CONTROLLERS:
        assert name not in body, (
            '{} references {!r} outside a comment (P0-B, 2026-08-29)'.format(
                config_path.name, name))


@pytest.mark.parametrize('config_path', _CONFIGS, ids=lambda path: path.name)
def test_installed_config_is_byte_identical_to_its_source(config_path):
    source = (_SOURCE_ROOT / 'franka_bringup' /
              config_path.relative_to(config_path.parents[2]))
    assert source.is_file(), 'no source file for installed config {}'.format(config_path)
    assert source.read_bytes() == config_path.read_bytes(), (
        'installed {} differs from its source; the checks above only bind what is installed'
        .format(config_path.name))


def test_every_installed_production_launch_config_reference_is_under_test():
    share = _installed_share()
    referenced = _config_filenames_referenced_by_installed_launches(share)
    covered = {path.name for path in _CONFIGS}
    assert referenced, 'no installed production launch names a config file'
    assert referenced <= covered, (
        'installed production launches reference config files that this test does not check: '
        '{}'.format(sorted(referenced - covered)))
    assert covered <= referenced, (
        'installed production configs that no installed production launch reaches: {}. Either '
        'wire them up or stop installing them.'.format(sorted(covered - referenced)))
