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
Test double for ``franka_web.config`` (see ``part1_stub/__init__.py``).

It implements the contract's public interface -- ``ConfigError``,
``MotionProfile``, ``SettlingConfig``, ``Settings``, ``default_config_path``,
``load``, ``degrees_to_radians`` and ``broadcast7`` -- with the same names,
signatures, units and defaults, so the backend can be exercised against it.

It is DELIBERATELY not a substitute for the real loader's operator-facing
error prose: refusals here name the key and say what is allowed, but the
byte-exact sentences the specification pins are the real module's, and every
test whose subject is one of those sentences skips while this double is
installed.
"""

from dataclasses import dataclass
import math
import os

from support.part1_stub import defaults

import yaml

_ARMS = ('panda1', 'panda2')
_JOINTS = defaults.JOINT_COUNT


class ConfigError(ValueError):
    """One operator-fixable problem with config.yaml."""


def _number(value):
    """Return ``value`` as a finite float, or None when it is not a number."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def degrees_to_radians(value):
    """Convert a scalar or a 7-sequence of degrees into radians."""
    if isinstance(value, (list, tuple)):
        return tuple(math.radians(float(item)) for item in value)
    return math.radians(float(value))


def broadcast7(name, value, *, unit, minimum=None, maximum=None,
               exclusive_minimum=True):
    """Accept a scalar or a 7-list, range-check each element, return 7 floats."""
    if isinstance(value, (list, tuple)):
        items = list(value)
        if len(items) != _JOINTS:
            raise ConfigError(
                '{}: expected a number or a list of {} numbers{}'.format(
                    name, _JOINTS, '' if unit is None else ' in ' + unit))
    else:
        items = [value] * _JOINTS
    numbers = []
    for item in items:
        number = _number(item)
        if number is None:
            raise ConfigError(
                '{}: expected a number or a list of {} numbers{}, found '
                '{!r}'.format(name, _JOINTS,
                              '' if unit is None else ' in ' + unit, item))
        if minimum is not None:
            if exclusive_minimum and not number > minimum:
                raise ConfigError(
                    '{}: expected a value greater than {}, found {}'.format(
                        name, minimum, number))
            if not exclusive_minimum and number < minimum:
                raise ConfigError(
                    '{}: expected a value of at least {}, found {}'.format(
                        name, minimum, number))
        if maximum is not None and number > maximum:
            raise ConfigError(
                '{}: expected a value of at most {}, found {}'.format(
                    name, maximum, number))
        numbers.append(number)
    return tuple(numbers)


@dataclass(frozen=True)
class MotionProfile:
    """One arm's impedance parameter set, in SI."""

    arm_id: str
    k_gains: tuple
    d_gains: tuple
    max_effort_nm: tuple
    max_target_velocity_rad_s: tuple
    watchdog_timeout_s: float
    max_header_age_s: float
    future_tolerance_s: float
    position_lower_rad: tuple
    position_upper_rad: tuple
    fence_enabled: bool
    from_file: bool

    def public_view(self):
        """Return the GET /api/config ``profiles.<arm>`` object."""
        return {
            'k_gains': list(self.k_gains),
            'd_gains': list(self.d_gains),
            'max_effort_nm': list(self.max_effort_nm),
            'max_target_velocity_rad_s': list(self.max_target_velocity_rad_s),
            'watchdog_timeout_s': self.watchdog_timeout_s,
            'max_header_age_s': self.max_header_age_s,
            'future_tolerance_s': self.future_tolerance_s,
            'fence_enabled': self.fence_enabled,
            'position_lower_rad': list(self.position_lower_rad),
            'position_upper_rad': list(self.position_upper_rad),
            'source': 'config' if self.from_file else 'default',
        }


@dataclass(frozen=True)
class SettlingConfig:
    """The activation-settling numbers, in SI."""

    drift_limit_rad: tuple
    span_limit_rad: tuple
    velocity_limit_rad_s: tuple
    fence_margin_rad: tuple
    stable_window_s: float
    min_samples: int
    timeout_s: float

    def policy(self):
        """Return the ActivationSettlingPolicy these values describe."""
        from franka_web.settling import ActivationSettlingPolicy
        return ActivationSettlingPolicy(
            max_watch_delta_rad=tuple(self.drift_limit_rad),
            max_position_span_rad=tuple(self.span_limit_rad),
            max_abs_velocity_rad_s=tuple(self.velocity_limit_rad_s),
            min_fence_margin_rad=tuple(self.fence_margin_rad),
            stable_window_s=self.stable_window_s,
            min_sample_count=self.min_samples,
            timeout_s=self.timeout_s)

    def public_view(self):
        """Return the GET /api/config ``settling`` object."""
        return {
            'drift_limit_rad': list(self.drift_limit_rad),
            'span_limit_rad': list(self.span_limit_rad),
            'velocity_limit_rad_s': list(self.velocity_limit_rad_s),
            'fence_margin_rad': list(self.fence_margin_rad),
            'stable_window_s': self.stable_window_s,
            'min_samples': self.min_samples,
            'timeout_s': self.timeout_s,
            'policy_sha256': self.policy().sha256,
        }


@dataclass(frozen=True)
class Settings:
    """The whole effective configuration, in SI."""

    bind: str
    port: int
    ros_domain_id: int
    state_dir: str
    recording_root: str
    franka_dir: str
    recording_enabled: bool
    jog_step_rad: float
    robot_ips: dict
    settling: SettlingConfig
    profiles: dict
    config_path: str
    config_present: bool

    def robot_ip(self, arm_id):
        """Return the configured address of one arm."""
        return self.robot_ips[arm_id]

    def profile(self, arm_id):
        """Return one arm's motion profile."""
        return self.profiles[arm_id]

    def public_view(self):
        """Return the GET /api/config body minus ``ok``."""
        return {
            'config_path': self.config_path,
            'config_present': self.config_present,
            'port': self.port,
            'bind': self.bind,
            'ros_domain_id': self.ros_domain_id,
            'state_dir': self.state_dir,
            'recording_root': self.recording_root,
            'recording_enabled': self.recording_enabled,
            'jog_step_rad': self.jog_step_rad,
            'robots': dict(self.robot_ips),
            'settling': self.settling.public_view(),
            'profiles': {arm_id: profile.public_view()
                         for arm_id, profile in self.profiles.items()},
        }


def default_config_path(environ=None):
    """Return the path the server reads its configuration from."""
    environ = os.environ if environ is None else environ
    base = environ.get('XDG_CONFIG_HOME')
    if not base:
        base = os.path.join(environ.get('HOME', os.path.expanduser('~')), '.config')
    return os.path.join(base, 'franka_web', 'config.yaml')


def _expand(path, environ):
    """Expand ``~`` and ``$VAR`` in a directory path against ``environ``."""
    home = environ.get('HOME')
    text = os.path.expandvars(str(path))
    if text.startswith('~') and home:
        text = home + text[1:]
    return os.path.normpath(os.path.expanduser(text))


def _mapping(document, key):
    """Return a nested mapping of ``document``, or an empty dict."""
    value = document.get(key)
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ConfigError('{}: expected a mapping'.format(key))
    return value


def _known_keys(name, mapping, allowed):
    """Refuse any key of ``mapping`` that is not in ``allowed``."""
    for key in mapping:
        if key not in allowed:
            raise ConfigError(
                '{}{}: unknown key. Allowed keys under {}: {}'.format(
                    name + '.' if name else '', key, name or 'the file',
                    ', '.join(sorted(allowed))))


def _settling(document):
    """Build the SettlingConfig from the file's ``settling`` mapping."""
    section = _mapping(document, 'settling')
    _known_keys('settling', section, (
        'drift_limit_deg', 'span_limit_deg', 'velocity_limit_deg_s',
        'fence_margin_deg', 'stable_window_s', 'min_samples', 'timeout_s'))
    values = dict(defaults.DEFAULT_SETTLING)
    pairs = (
        ('drift_limit_deg', 'drift_limit_rad', 'deg', True, 0.0),
        ('span_limit_deg', 'span_limit_rad', 'deg', True, 0.0),
        ('velocity_limit_deg_s', 'velocity_limit_rad_s', 'deg/s', True, 0.0),
        ('fence_margin_deg', 'fence_margin_rad', 'deg', False, 0.0),
    )
    for key, target, unit, exclusive, minimum in pairs:
        if key in section:
            degrees = broadcast7('settling.' + key, section[key], unit=unit,
                                 minimum=minimum, exclusive_minimum=exclusive)
            values[target] = tuple(math.radians(value) for value in degrees)
    for key in ('stable_window_s', 'timeout_s'):
        if key in section:
            number = _number(section[key])
            if number is None or number <= 0.0:
                raise ConfigError(
                    'settling.{}: expected a number greater than 0'.format(key))
            values[key.replace('_s', '_s')] = number
    if 'stable_window_s' in section:
        values['stable_window_s'] = _number(section['stable_window_s'])
    if 'timeout_s' in section:
        values['timeout_s'] = _number(section['timeout_s'])
    if 'min_samples' in section:
        samples = section['min_samples']
        if isinstance(samples, bool) or not isinstance(samples, int) or samples < 2:
            raise ConfigError('settling.min_samples: expected an integer of at least 2')
        values['min_samples'] = samples
    if values['timeout_s'] <= values['stable_window_s']:
        raise ConfigError(
            'settling.timeout_s: expected a value greater than '
            'settling.stable_window_s')
    return SettlingConfig(**values)


def _profile(arm_id, document):
    """Build one arm's MotionProfile from the file's ``profiles``/``fence``."""
    profiles = _mapping(document, 'profiles')
    _known_keys('profiles', profiles, _ARMS)
    section = _mapping(profiles, arm_id)
    _known_keys('profiles.' + arm_id, section, (
        'stiffness', 'damping', 'torque_limit_nm', 'speed_limit_deg_s'))
    baked = defaults.DEFAULT_PROFILES[arm_id]
    k_gains = tuple(baked['k_gains'])
    d_gains = tuple(baked['d_gains'])
    efforts = tuple(baked['max_effort_nm'])
    velocity = tuple(baked['max_target_velocity_rad_s'])
    if 'stiffness' in section:
        k_gains = broadcast7('profiles.{}.stiffness'.format(arm_id),
                             section['stiffness'], unit=None, minimum=0.0,
                             exclusive_minimum=False)
    if 'damping' in section:
        d_gains = broadcast7('profiles.{}.damping'.format(arm_id),
                             section['damping'], unit=None, minimum=0.0,
                             exclusive_minimum=False)
    if 'torque_limit_nm' in section:
        efforts = broadcast7('profiles.{}.torque_limit_nm'.format(arm_id),
                             section['torque_limit_nm'], unit='N.m', minimum=0.0)
        for index, value in enumerate(efforts):
            ceiling = defaults.POLICY_EFFORT_CEILING_NM[index]
            if value > ceiling:
                raise ConfigError(
                    'profiles.{}.torque_limit_nm[{}]: expected a value in '
                    '0 < x <= {} N.m, found {}'.format(
                        arm_id, index, ceiling, value))
    if 'speed_limit_deg_s' in section:
        degrees = broadcast7('profiles.{}.speed_limit_deg_s'.format(arm_id),
                             section['speed_limit_deg_s'], unit='deg/s',
                             minimum=0.0)
        velocity = tuple(math.radians(value) for value in degrees)
        for index, value in enumerate(velocity):
            if value > defaults.POLICY_VELOCITY_CEILING_RAD_S[index]:
                raise ConfigError(
                    'profiles.{}.speed_limit_deg_s[{}]: exceeds the factory '
                    'velocity ceiling'.format(arm_id, index))

    fences = _mapping(document, 'fence')
    _known_keys('fence', fences, _ARMS)
    fence = _mapping(fences, arm_id)
    _known_keys('fence.' + arm_id, fence, ('enabled', 'lower_deg', 'upper_deg'))
    lower = tuple(defaults.POLICY_POSITION_LOWER_RAD)
    upper = tuple(defaults.POLICY_POSITION_UPPER_RAD)
    enabled = bool(fence.get('enabled', False))
    if enabled:
        if 'lower_deg' in fence:
            lower = tuple(math.radians(float(v)) for v in fence['lower_deg'])
        if 'upper_deg' in fence:
            upper = tuple(math.radians(float(v)) for v in fence['upper_deg'])
        for index in range(_JOINTS):
            if lower[index] < defaults.POLICY_POSITION_LOWER_RAD[index]:
                lower = lower[:index] + (
                    defaults.POLICY_POSITION_LOWER_RAD[index],) + lower[index + 1:]
            if upper[index] > defaults.POLICY_POSITION_UPPER_RAD[index]:
                upper = upper[:index] + (
                    defaults.POLICY_POSITION_UPPER_RAD[index],) + upper[index + 1:]
            if not lower[index] < upper[index]:
                raise ConfigError(
                    'fence.{}.lower_deg[{}]: must be below the matching '
                    'upper bound'.format(arm_id, index))
    return MotionProfile(
        arm_id=arm_id,
        k_gains=tuple(k_gains),
        d_gains=tuple(d_gains),
        max_effort_nm=tuple(efforts),
        max_target_velocity_rad_s=tuple(velocity),
        watchdog_timeout_s=defaults.REVIEWED_TIMING_S['watchdog_timeout'],
        max_header_age_s=defaults.REVIEWED_TIMING_S['max_header_age'],
        future_tolerance_s=defaults.REVIEWED_TIMING_S['future_tolerance'],
        position_lower_rad=tuple(lower),
        position_upper_rad=tuple(upper),
        fence_enabled=enabled,
        from_file=bool(section) or bool(fence),
    )


def _domain_id(document, environ):
    """Resolve the DDS domain from the file, then the environment, then 0."""
    value = document.get('ros_domain_id')
    if value is not None:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError('ros_domain_id: expected an integer or null')
        if not 0 <= value <= defaults.ROS_DOMAIN_ID_MAXIMUM:
            raise ConfigError('ros_domain_id: expected a value in 0..{}'.format(
                defaults.ROS_DOMAIN_ID_MAXIMUM))
        return value
    text = (environ.get('ROS_DOMAIN_ID') or '').strip()
    if text.isdigit():
        number = int(text, 10)
        if 0 <= number <= defaults.ROS_DOMAIN_ID_MAXIMUM:
            return number
    return 0


def load(path=None, environ=None, *, make_dirs=True):
    """Read, validate and return Settings; raise ConfigError on a bad file."""
    environ = os.environ if environ is None else environ
    path = default_config_path(environ) if path is None else str(path)
    present = os.path.isfile(path)
    document = {}
    if present:
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                document = yaml.safe_load(handle.read())
        except (OSError, yaml.YAMLError) as error:
            raise ConfigError('{}: could not be parsed as YAML: {}'.format(
                path, error)) from None
        if document is None:
            document = {}
        if not isinstance(document, dict):
            raise ConfigError('{}: the file must be a mapping'.format(path))
    _known_keys('', document, (
        'port', 'bind', 'ros_domain_id', 'robots', 'directories', 'recording',
        'jog', 'settling', 'profiles', 'fence'))

    port = document.get('port', defaults.DEFAULT_PORT)
    if isinstance(port, bool) or not isinstance(port, int) or not 1024 <= port <= 65535:
        raise ConfigError('port: expected an integer in 1024..65535')
    bind = document.get('bind', defaults.DEFAULT_BIND)
    if not isinstance(bind, str) or not bind:
        raise ConfigError('bind: expected an address literal')

    robots = _mapping(document, 'robots')
    _known_keys('robots', robots, _ARMS)
    robot_ips = {}
    for arm_id in _ARMS:
        entry = _mapping(robots, arm_id)
        _known_keys('robots.' + arm_id, entry, ('ip',))
        address = entry.get('ip', defaults.DEFAULT_ROBOT_IPS[arm_id])
        if not isinstance(address, str) or not address:
            raise ConfigError('robots.{}.ip: expected a hostname or IPv4 '
                              'address'.format(arm_id))
        robot_ips[arm_id] = address

    directories = _mapping(document, 'directories')
    _known_keys('directories', directories, ('state', 'recordings', 'franka_dir'))
    state_dir = _expand(directories.get('state', defaults.DEFAULT_STATE_DIR), environ)
    recording_root = _expand(
        directories.get('recordings', defaults.DEFAULT_RECORDING_ROOT), environ)
    franka_dir = directories.get('franka_dir')
    if franka_dir is not None:
        franka_dir = _expand(franka_dir, environ)
        if not os.path.isabs(franka_dir):
            raise ConfigError('directories.franka_dir: expected an absolute path')

    recording = _mapping(document, 'recording')
    _known_keys('recording', recording, ('enabled',))
    recording_enabled = recording.get('enabled', True)
    if not isinstance(recording_enabled, bool):
        raise ConfigError('recording.enabled: expected true or false')

    jog = _mapping(document, 'jog')
    _known_keys('jog', jog, ('step_deg',))
    step_deg = jog.get('step_deg')
    if step_deg is None:
        jog_step_rad = defaults.JOG_STEP_RAD
    else:
        number = _number(step_deg)
        if number is None or not 0.0 < number <= 15.0:
            raise ConfigError('jog.step_deg: expected a value in 0 < x <= 15')
        jog_step_rad = math.radians(number)

    settings = Settings(
        bind=bind,
        port=port,
        ros_domain_id=_domain_id(document, environ),
        state_dir=state_dir,
        recording_root=recording_root,
        franka_dir=franka_dir,
        recording_enabled=recording_enabled,
        jog_step_rad=jog_step_rad,
        robot_ips=robot_ips,
        settling=_settling(document),
        profiles={arm_id: _profile(arm_id, document) for arm_id in _ARMS},
        config_path=path,
        config_present=present,
    )
    if make_dirs:
        os.makedirs(settings.state_dir, mode=0o700, exist_ok=True)
        os.makedirs(settings.recording_root, mode=0o700, exist_ok=True)
    return settings
