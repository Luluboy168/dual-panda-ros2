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
Read, validate and project the one optional ``config.yaml`` the server reads.

There is exactly one configuration surface: ``~/.config/franka_web/config.yaml``
(or ``$XDG_CONFIG_HOME/franka_web/config.yaml``). No environment variable
configures anything. A missing file means pure defaults, silently. A present
file is validated at startup and every refusal is one line that names the file,
the dotted key, what was found and what would be allowed.

Angles in the FILE are degrees; everything this module returns is SI. The
conversion happens at exactly one boundary, inside :func:`load`. A key absent
from the file takes its SI default from :mod:`franka_web.defaults`
bit-exactly -- no degree round-trip is ever performed on a default.
"""

from dataclasses import dataclass, field
import ipaddress
import json
import math
import os
import re
from types import MappingProxyType

from franka_web import defaults
from franka_web.settling import ActivationSettlingPolicy
import yaml


class ConfigError(ValueError):
    """One operator-fixable problem with config.yaml; str() is the message."""

    def __init__(self, key, problem=None, path=None):
        """Build a message as ``<path>: <key>: <problem>``, omitting empty parts."""
        self.key = key
        self.problem = problem
        self.path = path
        super().__init__(self._render())

    def _render(self):
        """Join the non-empty message parts with ``: ``."""
        parts = [self.path, self.key, self.problem]
        return ': '.join(part for part in parts if part)

    def with_path(self, path):
        """Return the same problem, prefixed with the config file path."""
        return ConfigError(self.key, self.problem, path)


# --- message rendering (the whole error-message style, in one place) ---------

def _num(value):
    """Format one number the way the operator wrote it: 6, 1.0, 0.05, 12.0."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    text = '{:.10g}'.format(float(value))
    if not any(marker in text for marker in ('.', 'e', 'n', 'i')):
        text += '.0'
    return text


def _bound(value):
    """Format a range MINIMUM; an exact zero is the bare ``0`` messages use."""
    if isinstance(value, float) and value == 0.0:
        return '0'
    return _num(value)


def _type_name(value):
    """Return the article-plus-noun name of a YAML value's type."""
    if value is None:
        return 'null'
    if isinstance(value, bool):
        return 'a boolean'
    if isinstance(value, (int, float)):
        return 'a number'
    if isinstance(value, str):
        return 'a string'
    if isinstance(value, dict):
        return 'a mapping'
    if isinstance(value, (list, tuple)):
        return 'a list'
    return 'a value'


def _scalar_text(value):
    """Render the offending scalar, and nothing more of the file."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None:
        return 'nothing'
    if isinstance(value, str):
        return json.dumps(value[:40] + '...' if len(value) > 40 else value)
    if isinstance(value, (int, float)):
        return _num(value)
    return _type_name(value)


def _found_wrong_type(value):
    """Describe a value whose TYPE is wrong, e.g. ``"two" (a string)``."""
    if isinstance(value, dict):
        return 'a mapping'
    if isinstance(value, (list, tuple)):
        return 'a list of {} items'.format(len(value))
    if value is None:
        return 'nothing (null)'
    noun = _type_name(value).split(' ', 1)[-1]
    return '{} (a {})'.format(_scalar_text(value), noun)


def _found_value(value):
    """Describe a value whose type was right and only the value is out of range."""
    return _num(value)


def _format_list(values):
    """Render a seven-element example list the way the messages show it."""
    return '[{}]'.format(', '.join(_num(value) for value in values))


_UNIT_PHRASE = {
    None: '',
    'deg': ' in degrees',
    'deg/s': ' in degrees per second',
    'N.m': ' in newton-metres',
    's': ' in seconds',
    # The gripper block has no angles: millimetres, newtons and hertz only.
    'mm': ' in millimetres',
    'mm/s': ' in millimetres per second',
    'N': ' in newtons',
    'Hz': ' in hertz',
}

_UNIT_SUFFIX = {
    None: '',
    'deg': ' deg',
    'deg/s': ' deg/s',
    'N.m': ' N·m',
    's': ' s',
    'mm': ' mm',
    'mm/s': ' mm/s',
    'N': ' N',
    'Hz': ' Hz',
}


def _range_phrase(minimum, maximum, exclusive_minimum):
    """Return the ``Allowed:`` wording of a TYPE error; an exclusive minimum only."""
    if minimum is None:
        if maximum is None:
            return 'any finite number'
        return 'any value of at most {}'.format(_num(maximum))
    if exclusive_minimum:
        # The ceiling belongs in the RANGE error, where it is what went wrong.
        return 'any value greater than {}'.format(_bound(minimum))
    if maximum is None:
        return 'any value of {} or more'.format(_bound(minimum))
    return 'any value of {} or more and at most {}'.format(_bound(minimum), _num(maximum))


def _range_expression(minimum, maximum, exclusive_minimum):
    """Return the compact algebraic wording a RANGE error uses."""
    if minimum is None:
        if maximum is None:
            return 'that is a finite number'
        return 'of at most {}'.format(_num(maximum))
    if exclusive_minimum:
        if maximum is None:
            return 'greater than {}'.format(_bound(minimum))
        return 'in {} < x <= {}'.format(_bound(minimum), _num(maximum))
    if maximum is None:
        return 'of {} or more'.format(_bound(minimum))
    return 'in {} <= x <= {}'.format(_bound(minimum), _num(maximum))


# --- inward-rounded degree bounds and the boundary snap ----------------------
#
# A message that quotes a degree bound must quote a number the very next load()
# accepts. Round-to-nearest does not: joint 6's lower limit rounds to -1.003
# deg, which is 5.65e-6 rad OUTSIDE the policy. Bounds are therefore rounded
# INWARD, and a converted value that lands just outside a policy value is
# snapped onto it -- inward only, never outward.

_SNAP_RAD = math.radians(0.0005) * 1.000001   # 8.726654986617907e-06 rad:
# half a milli-degree, i.e. the resolution of 3-decimal degree entry, with one
# ULP of slack. Do not shrink this; 1e-9 is four orders of magnitude too small.


def _deg_lower(bound_rad):
    """Return a LOWER bound in degrees, rounded inward (up) to 3 decimals."""
    return math.ceil(math.degrees(bound_rad) * 1000.0) / 1000.0


def _deg_upper(bound_rad):
    """Return an UPPER bound or ceiling in degrees, rounded inward (down)."""
    return math.floor(math.degrees(bound_rad) * 1000.0) / 1000.0


def _snap_low(value_rad, policy_rad):
    """Snap a lower bound onto the policy when it sits just outside it."""
    if value_rad < policy_rad and policy_rad - value_rad <= _SNAP_RAD:
        return policy_rad
    return value_rad


def _snap_high(value_rad, policy_rad):
    """Snap an upper bound or ceiling onto the policy when it sits just above."""
    if value_rad > policy_rad and value_rad - policy_rad <= _SNAP_RAD:
        return policy_rad
    return value_rad


# --- unknown keys ------------------------------------------------------------

def _levenshtein(left, right):
    """Return the edit distance between two names (iterative two-row DP)."""
    if left == right:
        return 0
    previous = list(range(len(right) + 1))
    for index, left_character in enumerate(left, start=1):
        current = [index]
        for column, right_character in enumerate(right, start=1):
            cost = 0 if left_character == right_character else 1
            current.append(min(previous[column] + 1,
                               current[column - 1] + 1,
                               previous[column - 1] + cost))
        previous = current
    return previous[-1]


def _suggest(name, allowed):
    """Return the nearest legal sibling within distance 2, or ``None``."""
    best = None
    best_distance = 3
    for candidate in allowed:
        distance = _levenshtein(name, candidate)
        if distance < best_distance:
            best = candidate
            best_distance = distance
    return best


# The reviewed timing values were config keys in an older draft and are not
# keys now: they are fixed by the controller's reviewed timing policy. Writing
# one is an ordinary unknown-key error, with a sentence that teaches where the
# value actually lives instead of a "Did you mean" suggestion.
_DROPPED_TIMING_KEYS = {
    'watchdog_timeout_s': ('watchdog timing', 'watchdog_timeout'),
    'max_header_age_s': ('header-age limit', 'max_header_age'),
    'future_tolerance_s': ('future-tolerance limit', 'future_tolerance'),
}


# `recording` (the on/off switch) and `recordings` (the size cap) differ by one
# letter, so writing one key under the other section is the mistake this file
# should expect. Each entry maps (section, key) to the sentence that says where
# the key really lives, ahead of the ordinary allowed-keys clause.
_MISPLACED_KEYS = {
    ('recording', 'max_total_gb'): (
        'the cap on the total size of stored recordings lives at '
        'recordings.max_total_gb (with the s), not under recording.'),
    ('', 'max_total_gb'): (
        'the cap on the total size of stored recordings lives at '
        'recordings.max_total_gb, under a `recordings:` section.'),
    ('recordings', 'enabled'): (
        'the switch that turns session recording off lives at '
        'recording.enabled (no s), not under recordings.'),
}


def _unknown_key(dotted_key, parent_dotted, name, allowed):
    """Raise the unknown-key ConfigError for ``name`` under ``parent_dotted``."""
    if parent_dotted:
        allowed_clause = 'Allowed keys under {}: {}.'.format(
            parent_dotted, ', '.join(allowed))
    else:
        allowed_clause = 'Allowed top-level keys: {}.'.format(', '.join(allowed))
    dropped = _DROPPED_TIMING_KEYS.get(name)
    if dropped is not None and parent_dotted in _PROFILE_PARENTS:
        noun, timing_key = dropped
        raise ConfigError(dotted_key, (
            "unknown key. The controller's {} ({} s) is fixed by its reviewed "
            'timing policy and is not settable from this file; it is reported '
            'read-only in GET /api/config. {}').format(
                noun, _num(defaults.REVIEWED_TIMING_S[timing_key]), allowed_clause))
    misplaced = _MISPLACED_KEYS.get((parent_dotted, name))
    if misplaced is not None:
        raise ConfigError(dotted_key, 'unknown key: {} {}'.format(
            misplaced, allowed_clause))
    suggestion = _suggest(name, allowed)
    if suggestion is not None:
        raise ConfigError(dotted_key, 'unknown key. Did you mean "{}"? {}'.format(
            suggestion, allowed_clause))
    raise ConfigError(dotted_key, 'unknown key. {}'.format(allowed_clause))


# --- the schema tables -------------------------------------------------------

# Four keys only. The three reviewed timing values are NOT config keys; they
# come from defaults.REVIEWED_TIMING_S and are reported read-only in
# GET /api/config. Writing one is an unknown-key error that says exactly that.
_PROFILE_KEYS = ('stiffness', 'damping', 'torque_limit_nm', 'speed_limit_deg_s')
_SETTLING_KEYS = ('drift_limit_deg', 'span_limit_deg', 'velocity_limit_deg_s',
                  'fence_margin_deg', 'stable_window_s', 'min_samples', 'timeout_s')
_FENCE_KEYS = ('enabled', 'lower_deg', 'upper_deg')

# THIRTEEN keys. joint_names is the one whose default depends on the arm id,
# so it cannot live in defaults.DEFAULT_GRIPPER and is filled from
# defaults.GRIPPER_JOINT_NAME_TEMPLATE in GripperConfig.__post_init__.
_GRIPPER_KEYS = ('enabled', 'serial_id', 'usb_path', 'speed_mm_s', 'force_n',
                 'open_width_mm', 'close_width_mm', 'poll_rate_hz',
                 'auto_activate', 'motion_timeout_s', 'activation_timeout_s',
                 'reconnect_interval_s', 'joint_names')

_ALLOWED_KEYS = {
    # Alphabetical: this tuple is rendered into every unknown-top-level-key
    # message, so its order is operator-visible.
    '': ('bind', 'directories', 'fence', 'grippers', 'jog', 'port', 'profiles',
         'recording', 'recordings', 'robots', 'ros_domain_id', 'settling'),
    'robots': defaults.ARM_IDS,
    'robots.panda1': ('ip',),
    'robots.panda2': ('ip',),
    'directories': ('state', 'recordings', 'franka_dir'),
    'recording': ('enabled',),
    'recordings': ('max_total_gb',),
    'jog': ('step_deg',),
    'settling': _SETTLING_KEYS,
    'profiles': defaults.ARM_IDS,
    'profiles.panda1': _PROFILE_KEYS,
    'profiles.panda2': _PROFILE_KEYS,
    'fence': defaults.ARM_IDS,
    'fence.panda1': _FENCE_KEYS,
    'fence.panda2': _FENCE_KEYS,
    'grippers': defaults.ARM_IDS,
    'grippers.panda1': _GRIPPER_KEYS,
    'grippers.panda2': _GRIPPER_KEYS,
}

_PROFILE_PARENTS = ('profiles.panda1', 'profiles.panda2')


def _join(parent_dotted, name):
    """Return the dotted path of ``name`` under ``parent_dotted``."""
    return '{}.{}'.format(parent_dotted, name) if parent_dotted else name


def _not_a_mapping(dotted, value):
    """Return the ConfigError for a section that is not a mapping."""
    return ConfigError(dotted, 'expected a mapping of settings, found {}. '
                       'Allowed keys under {}: {}.'.format(
                           _found_wrong_type(value), dotted,
                           ', '.join(_ALLOWED_KEYS[dotted])))


def _validate_structure(mapping, dotted):
    """Check every key of ``mapping`` and of its known sub-sections, depth-first."""
    allowed = _ALLOWED_KEYS[dotted]
    for key in mapping:
        if not isinstance(key, str):
            raise ConfigError(dotted, 'keys must be names, found {}.'.format(
                _found_wrong_type(key)))
        if key not in allowed:
            _unknown_key(_join(dotted, key), dotted, key, allowed)
    for key in allowed:
        child = _join(dotted, key)
        if child not in _ALLOWED_KEYS or key not in mapping:
            continue
        value = mapping[key]
        if value is None:
            continue
        if not isinstance(value, dict):
            raise _not_a_mapping(child, value)
        _validate_structure(value, child)


def _section(mapping, name, parent_dotted):
    """Return the mapping stored under ``name``; ``{}`` when it is absent or null."""
    value = mapping.get(name)
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise _not_a_mapping(_join(parent_dotted, name), value)
    return value


# --- leaf readers ------------------------------------------------------------

def _read_port(mapping, key, dotted, default):
    """Return the listen port, or the default when the key is absent."""
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = 'expected an integer in {}..{}'.format(
        _num(defaults.PORT_MINIMUM), _num(defaults.PORT_MAXIMUM))
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(dotted, '{}, found {}.'.format(sentence, _found_wrong_type(value)))
    if not defaults.PORT_MINIMUM <= value <= defaults.PORT_MAXIMUM:
        raise ConfigError(dotted, '{}, found {}.'.format(sentence, _found_value(value)))
    return value


def _read_bind(mapping, key, dotted, default):
    """Return the listen address, or the default when the key is absent."""
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = ('expected an IPv4 or IPv6 address to listen on, found {}. '
                'Allowed: 0.0.0.0 (every interface), :: , 127.0.0.1, or any '
                'address this machine owns.')
    if not isinstance(value, str):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    try:
        ipaddress.ip_address(value)
    except ValueError:
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value))) from None
    return value


def _read_domain_id(mapping, key, dotted):
    """Return an explicit ROS domain id, or ``None`` when the key is absent or null."""
    if key not in mapping or mapping[key] is None:
        return None
    value = mapping[key]
    sentence = 'expected an integer in {}..{} or null'.format(
        _num(0), _num(defaults.ROS_DOMAIN_ID_MAXIMUM))
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(dotted, '{}, found {}.'.format(sentence, _found_wrong_type(value)))
    if not 0 <= value <= defaults.ROS_DOMAIN_ID_MAXIMUM:
        raise ConfigError(dotted, '{}, found {}.'.format(sentence, _found_value(value)))
    return value


_ADDRESS_RE = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]{0,253}')


def _read_ip(mapping, key, dotted, default):
    """Return one robot address, or the default when the key is absent."""
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = ('expected a hostname or IPv4 address, found {}. Allowed: '
                'letters, digits, dots, dashes and underscores, up to 254 '
                'characters, e.g. 172.16.0.2.')
    if not isinstance(value, str) or not _ADDRESS_RE.fullmatch(value):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    return value


def _read_bool(mapping, key, dotted, default):
    """Return a boolean setting, or the default when the key is absent."""
    if key not in mapping:
        return default
    value = mapping[key]
    if not isinstance(value, bool):
        raise ConfigError(dotted, 'expected true or false, found {}.'.format(
            _found_wrong_type(value)))
    return value


def _read_positive_number(mapping, key, dotted, default):
    """Return a number strictly greater than zero, or the default."""
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = 'expected a number greater than 0, found {}.'
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    number = float(value)
    if not math.isfinite(number):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    if number <= 0.0:
        raise ConfigError(dotted, sentence.format(_found_value(number)))
    return number


def _read_size_cap_gb(mapping, key, dotted, default):
    """
    Return the recordings size cap in GB, ``None`` for ``unlimited``.

    Zero is refused rather than read as "keep nothing": a 0 cap would remove
    every sealed recording on the next pass, and nobody who typed 0 meant
    that. The message says the one spelling that really does switch the pass
    off, so the operator who DID mean it has the line to write.

    A positive value smaller than one byte is refused by the same sentence,
    because it IS zero: the pass floors the cap to whole bytes, so a
    fat-fingered ``0.0000000001`` reaches it as a cap of 0 bytes and empties
    the root exactly as a written 0 would.
    """
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = (
        'expected a number of GB greater than 0, or "{}" to keep every '
        'recording, found {{}}. One GB is 1 000 000 000 bytes; the recorder '
        'writes about 14 GB per hour of recording.').format(
            defaults.RECORDING_RETENTION_UNLIMITED)
    zero_sentence = (
        'expected a number of GB greater than 0, or "{}" to keep every '
        'recording, found {{}}. A cap of zero bytes would remove every sealed '
        'recording the next time the retention pass ran; write '
        '{}: {} if you meant to switch the cap off.').format(
            defaults.RECORDING_RETENTION_UNLIMITED, dotted,
            defaults.RECORDING_RETENTION_UNLIMITED)
    if isinstance(value, str):
        if value.strip().lower() == defaults.RECORDING_RETENTION_UNLIMITED:
            return None
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    number = float(value)
    if not math.isfinite(number):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    # The operator's own spelling: a written `0` reads back as 0, not 0.0.
    if number == 0.0:
        raise ConfigError(dotted, zero_sentence.format(_found_value(value)))
    if number < 0.0:
        raise ConfigError(dotted, sentence.format(_found_value(value)))
    # The cap the retention pass really applies is int(gb * BYTES_PER_GB), so
    # anything under a byte arrives there as zero and removes everything.
    if int(number * defaults.BYTES_PER_GB) < 1:
        raise ConfigError(dotted, zero_sentence.format(_found_value(value)))
    return number


def _read_min_samples(mapping, key, dotted, default):
    """Return the minimum settling sample count, or the default."""
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = 'expected an integer of 2 or more, found {}.'
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    if value < 2:
        raise ConfigError(dotted, sentence.format(_found_value(value)))
    return value


def _read_bounded_number(mapping, key, dotted, default, *, maximum, unit):
    """Return a number in ``0 < x <= maximum``, or the default."""
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = 'expected a value {}{}, found {{}}.'.format(
        _range_expression(0.0, maximum, True), _UNIT_SUFFIX[unit])
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    number = float(value)
    if not math.isfinite(number):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    if not 0.0 < number <= maximum:
        raise ConfigError(dotted, sentence.format(_found_value(number)))
    return number


_DEVICE_NAME_ROOTS = {'serial_id': '/dev/serial/by-id/',
                      'usb_path': '/dev/serial/by-path/'}

_DEVICE_NAME_EXAMPLE = 'usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0'

#: A basename, and nothing that could turn into a path or a pattern.
_DEVICE_NAME_FORBIDDEN = ('/', '*', '?', '\\', '\x00')


def _read_device_name(mapping, key, dotted, default):
    """
    Return one adapter's device NAME -- a basename, never a path or a pattern.

    This is a syntax check and touches no filesystem: it is the same rule
    whether the adapter is plugged in, absent, or has not been bought yet, so
    a placeholder basename is legal here and only fails when something
    actually tries to open it.
    """
    if key not in mapping:
        return default
    value = mapping[key]
    root = _DEVICE_NAME_ROOTS.get(key, _DEVICE_NAME_ROOTS['serial_id'])
    sentence = ('expected the name of an entry under {}, found {{}}. Allowed: '
                'the basename only, e.g. {}. Run `ls -l {}` to see the names; '
                'see franka_robotiq/doc/SERIAL_BINDING.md.').format(
                    root, _DEVICE_NAME_EXAMPLE, root)
    if not isinstance(value, str):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    if '*' in value or '?' in value:
        raise ConfigError(dotted, (
            'expected the name of an entry under {}, found {} (a pattern). No '
            'wildcard: this binding never scans and never picks the only '
            'adapter present, because that is exactly how two identical '
            'adapters swap arms. Allowed: the basename only, e.g. {}. '
            'See franka_robotiq/doc/SERIAL_BINDING.md.').format(
                root, _scalar_text(value), _DEVICE_NAME_EXAMPLE))
    if (value in ('.', '..')
            or any(bad in value for bad in _DEVICE_NAME_FORBIDDEN)):
        raise ConfigError(dotted, sentence.format(
            '{} (a path)'.format(_scalar_text(value))))
    return value


def _read_closed_range_number(mapping, key, dotted, default, *, minimum,
                              maximum, unit):
    """Return a number in ``minimum <= x <= maximum``, or the default."""
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = 'expected a value {}{}, found {{}}.'.format(
        _range_expression(minimum, maximum, False), _UNIT_SUFFIX[unit])
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    number = float(value)
    if not math.isfinite(number):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value)))
    if not minimum <= number <= maximum:
        raise ConfigError(dotted, sentence.format(_found_value(number)))
    return number


def _read_joint_names(mapping, key, dotted, default):
    """
    Return the two finger-joint names, or the arm's default pair.

    A one-element list and a duplicated name are DIFFERENT mistakes and get
    different sentences: the first is a shape error, the second silently
    loses the second finger, because a consumer resolving a JointState by
    name takes the first entry of a repeated one.
    """
    if key not in mapping:
        return default
    value = mapping[key]
    sentence = ('expected a list of exactly two joint names, one per finger, '
                'found {}. Allowed: two distinct non-empty names, e.g. {}.')
    example = _format_names(default)
    if not isinstance(value, (list, tuple)):
        raise ConfigError(dotted, sentence.format(_found_wrong_type(value), example))
    if len(value) != 2:
        raise ConfigError(dotted, sentence.format(
            'a list of {} items'.format(len(value)), example))
    for index, name in enumerate(value):
        if not isinstance(name, str) or not name.strip():
            raise ConfigError('{}[{}]'.format(dotted, index), (
                'expected a non-empty joint name, found {}.').format(
                    _found_wrong_type(name)))
    if value[0] == value[1]:
        raise ConfigError(dotted, (
            'expected two DISTINCT joint names, found {} twice. A JointState '
            'with a repeated name resolves to its first entry, so the second '
            'finger would disappear from every consumer.').format(
                _scalar_text(value[0])))
    return tuple(str(name) for name in value)


def _format_names(names):
    """Render a joint-name pair the way the messages show it."""
    return '[{}]'.format(', '.join(json.dumps(name) for name in names))


_DIRECTORY_SENTENCE = ('expected an absolute directory path, found {}. Allowed: '
                       'a path starting with /, ~ or $VAR, e.g. '
                       '~/franka_web_recordings.')


def _read_directory(mapping, key, dotted, default, environ):
    """Return an expanded absolute directory path, or the expanded default."""
    value = mapping[key] if key in mapping else default
    if not isinstance(value, str):
        raise ConfigError(dotted, _DIRECTORY_SENTENCE.format(_found_wrong_type(value)))
    expanded = _expand(environ, value)
    if '\x00' in expanded or not os.path.isabs(expanded):
        raise ConfigError(dotted, _DIRECTORY_SENTENCE.format(_found_wrong_type(value)))
    return os.path.normpath(expanded)


def _read_optional_directory(mapping, key, dotted, environ):
    """Return an expanded absolute path, or ``None`` for an absent or null key."""
    if key not in mapping or mapping[key] is None:
        return None
    return _read_directory(mapping, key, dotted, None, environ)


# --- vectors -----------------------------------------------------------------

_EXAMPLES = {
    'settling.drift_limit_deg': (2.0, (2.0, 5.0, 2.0, 2.0, 2.0, 2.0, 2.0)),
    'settling.span_limit_deg': (0.05, (0.05,) * 7),
    'settling.velocity_limit_deg_s': (1.0, (1.0,) * 7),
    'settling.fence_margin_deg': (5.0, (5.0,) * 7),
    'profiles.panda1.speed_limit_deg_s': (5.73, (5.73,) * 7),
    'profiles.panda2.speed_limit_deg_s': (5.73, (5.73,) * 7),
}


def _example_clause(name):
    """Return the ``, e.g. …`` clause for a key, or a bare full stop."""
    example = _EXAMPLES.get(name)
    if example is None:
        return '.'
    scalar, vector = example
    return ', e.g. {} or {}.'.format(_num(scalar), _format_list(vector))


def _as_seven(value):
    """Return seven finite floats from a scalar or a 7-list, else ``None``."""
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return (number,) * 7 if math.isfinite(number) else None
    if isinstance(value, (list, tuple)) and len(value) == defaults.JOINT_COUNT:
        result = []
        for element in value:
            if isinstance(element, bool) or not isinstance(element, (int, float)):
                return None
            number = float(element)
            if not math.isfinite(number):
                return None
            result.append(number)
        return tuple(result)
    return None


def _check_range(name, value, unit, minimum, maximum, exclusive_minimum):
    """Raise the range ConfigError when ``value`` falls outside the bounds."""
    if minimum is not None:
        if exclusive_minimum and value <= minimum:
            _raise_range(name, value, unit, minimum, maximum, exclusive_minimum)
        if not exclusive_minimum and value < minimum:
            _raise_range(name, value, unit, minimum, maximum, exclusive_minimum)
    if maximum is not None and value > maximum:
        _raise_range(name, value, unit, minimum, maximum, exclusive_minimum)


def _raise_range(name, value, unit, minimum, maximum, exclusive_minimum):
    """Raise one range ConfigError in the documented wording."""
    raise ConfigError(name, 'expected a value {}{}, found {}.'.format(
        _range_expression(minimum, maximum, exclusive_minimum),
        _UNIT_SUFFIX[unit], _found_value(value)))


def broadcast7(name, value, *, unit, minimum=None, maximum=None,
               exclusive_minimum=True):
    """
    Accept a scalar or a 7-list, range-check each element, return 7 floats.

    ``name`` is the dotted config key and appears verbatim in any ConfigError.
    ``unit`` is 'deg', 'deg/s', 'N.m' or None and selects the message wording.
    """
    values = _as_seven(value)
    if values is None:
        raise ConfigError(name, (
            'expected a number or a list of 7 numbers{}, found {}. '
            'Allowed: {}{}').format(
                _UNIT_PHRASE[unit], _found_wrong_type(value),
                _range_phrase(minimum, maximum, exclusive_minimum),
                _example_clause(name)))
    indexed = isinstance(value, (list, tuple))
    for index, element in enumerate(values):
        element_name = '{}[{}]'.format(name, index) if indexed else name
        _check_range(element_name, element, unit, minimum, maximum, exclusive_minimum)
    return values


def _list7(name, value, *, unit, allowed, example):
    """Return seven finite floats from a list-only key, or raise the type error."""
    if isinstance(value, (list, tuple)) and len(value) == defaults.JOINT_COUNT:
        values = _as_seven(list(value))
        if values is not None:
            return values
    raise ConfigError(name, (
        'expected a list of 7 numbers{}, found {}. Allowed: {}, e.g. {}.').format(
            _UNIT_PHRASE[unit], _found_wrong_type(value), allowed, _format_list(example)))


def degrees_to_radians(value):
    """Convert a scalar or a 7-sequence in degrees to radians."""
    if isinstance(value, (list, tuple)):
        if len(value) != defaults.JOINT_COUNT:
            raise ValueError('expected {} values, got {}'.format(
                defaults.JOINT_COUNT, len(value)))
        return tuple(math.radians(float(element)) for element in value)
    return math.radians(float(value))


def _seven_floats(name, values):
    """Return a normalized 7-tuple of floats, or raise ``ValueError``."""
    if isinstance(values, (str, bytes)) or values is None:
        raise ValueError('{} must contain exactly 7 numbers'.format(name))
    vector = tuple(values)
    if len(vector) != defaults.JOINT_COUNT:
        raise ValueError('{} must contain exactly 7 numbers'.format(name))
    if any(isinstance(element, bool) for element in vector):
        raise ValueError('{} values must be numeric, not boolean'.format(name))
    return tuple(float(element) for element in vector)


# --- the three record types --------------------------------------------------

@dataclass(frozen=True)
class MotionProfile:
    """One arm's impedance profile and position bounds, in SI units."""

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

    def __post_init__(self):
        """Normalize every joint vector to a 7-tuple of floats."""
        for name in ('k_gains', 'd_gains', 'max_effort_nm',
                     'max_target_velocity_rad_s', 'position_lower_rad',
                     'position_upper_rad'):
            object.__setattr__(self, name, _seven_floats(name, getattr(self, name)))

    def public_view(self):
        """Return the read-only profile object GET /api/config publishes."""
        return {
            'k_gains': list(self.k_gains),
            'd_gains': list(self.d_gains),
            'max_effort_nm': list(self.max_effort_nm),
            'max_target_velocity_rad_s': list(self.max_target_velocity_rad_s),
            'watchdog_timeout_s': float(self.watchdog_timeout_s),
            'max_header_age_s': float(self.max_header_age_s),
            'future_tolerance_s': float(self.future_tolerance_s),
            'fence_enabled': bool(self.fence_enabled),
            'position_lower_rad': list(self.position_lower_rad),
            'position_upper_rad': list(self.position_upper_rad),
            'source': 'config' if self.from_file else 'default',
        }


@dataclass(frozen=True)
class SettlingConfig:
    """The activation-settling numbers, in SI units."""

    drift_limit_rad: tuple
    span_limit_rad: tuple
    velocity_limit_rad_s: tuple
    fence_margin_rad: tuple
    stable_window_s: float
    min_samples: int
    timeout_s: float

    def __post_init__(self):
        """Normalize the four vectors and memoize the reviewed policy object."""
        for name in ('drift_limit_rad', 'span_limit_rad',
                     'velocity_limit_rad_s', 'fence_margin_rad'):
            object.__setattr__(self, name, _seven_floats(name, getattr(self, name)))
        object.__setattr__(self, 'stable_window_s', float(self.stable_window_s))
        object.__setattr__(self, 'timeout_s', float(self.timeout_s))
        object.__setattr__(self, 'min_samples', int(self.min_samples))
        object.__setattr__(self, '_policy', ActivationSettlingPolicy(
            max_watch_delta_rad=self.drift_limit_rad,
            max_position_span_rad=self.span_limit_rad,
            max_abs_velocity_rad_s=self.velocity_limit_rad_s,
            min_fence_margin_rad=self.fence_margin_rad,
            stable_window_s=self.stable_window_s,
            min_sample_count=self.min_samples,
            timeout_s=self.timeout_s))

    def policy(self):
        """
        Return the ActivationSettlingPolicy these values describe.

        The rename table this class exists to hold, spelled out where the
        conversion happens: ``drift_limit_rad`` -> ``max_watch_delta_rad``,
        ``span_limit_rad`` -> ``max_position_span_rad``,
        ``velocity_limit_rad_s`` -> ``max_abs_velocity_rad_s``,
        ``fence_margin_rad`` -> ``min_fence_margin_rad``. The operator-facing
        names are the config keys; the reviewed gate's names are the ones on
        the right, and nothing else may bridge them.
        """
        return self._policy

    def public_view(self):
        """Return the settling block GET /api/config publishes."""
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
class GripperConfig:
    """One arm's gripper block, in millimetres, newtons and seconds."""

    arm_id: str
    enabled: bool = False
    serial_id: str = ''
    usb_path: str = ''
    speed_mm_s: float = defaults.DEFAULT_GRIPPER['speed_mm_s']
    force_n: float = defaults.DEFAULT_GRIPPER['force_n']
    open_width_mm: float = defaults.DEFAULT_GRIPPER['open_width_mm']
    close_width_mm: float = defaults.DEFAULT_GRIPPER['close_width_mm']
    poll_rate_hz: float = defaults.DEFAULT_GRIPPER['poll_rate_hz']
    auto_activate: bool = defaults.DEFAULT_GRIPPER['auto_activate']
    motion_timeout_s: float = defaults.DEFAULT_GRIPPER['motion_timeout_s']
    activation_timeout_s: float = defaults.DEFAULT_GRIPPER['activation_timeout_s']
    reconnect_interval_s: float = defaults.DEFAULT_GRIPPER['reconnect_interval_s']
    joint_names: tuple = None
    from_file: bool = False

    def __post_init__(self):
        """Fill the one default that depends on the arm id, and freeze the pair."""
        if self.joint_names is None:
            object.__setattr__(self, 'joint_names', default_joint_names(self.arm_id))
        else:
            object.__setattr__(self, 'joint_names',
                               tuple(str(name) for name in self.joint_names))

    def public_view(self):
        """
        Return the GET /api/config grippers.<arm> object.

        These are the values THIS FILE carries.  The gripper nodes are
        standing nodes started outside this server, so the values actually in
        force are the node's own parameters -- the page reads those from the
        state frame's speed_mm_s / force_n, which come from the node's
        ~/status.  The `i` popover labels this block accordingly, and the two
        are never conflated.
        """
        return {
            'enabled': bool(self.enabled),
            'serial_id': self.serial_id,
            'usb_path': self.usb_path,
            'speed_mm_s': float(self.speed_mm_s),
            'force_n': float(self.force_n),
            'open_width_mm': float(self.open_width_mm),
            'close_width_mm': float(self.close_width_mm),
            'poll_rate_hz': float(self.poll_rate_hz),
            'auto_activate': bool(self.auto_activate),
            'motion_timeout_s': float(self.motion_timeout_s),
            'activation_timeout_s': float(self.activation_timeout_s),
            'reconnect_interval_s': float(self.reconnect_interval_s),
            'joint_names': list(self.joint_names),
            'source': 'config' if self.from_file else 'default',
        }


def default_joint_names(arm_id):
    """Return one arm's default finger-joint names."""
    return tuple(name.format(arm_id=arm_id)
                 for name in defaults.GRIPPER_JOINT_NAME_TEMPLATE)


@dataclass(frozen=True)
class Settings:
    """The whole effective configuration, in SI units."""

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
    # Last, and defaulted, so a hand-built stub keeps working.
    grippers: dict = field(default_factory=dict)
    # The cap on the TOTAL size of stored recordings, in GB, or None when the
    # operator wrote `recordings.max_total_gb: unlimited`. Defaulted for the
    # same reason `grippers` is.
    recording_max_total_gb: float = defaults.DEFAULT_RECORDING_MAX_TOTAL_GB

    def __post_init__(self):
        """Freeze the interior mappings so a consumer cannot rewrite them."""
        object.__setattr__(self, 'robot_ips', MappingProxyType(dict(self.robot_ips)))
        object.__setattr__(self, 'profiles', MappingProxyType(dict(self.profiles)))
        grippers = dict(self.grippers)
        for arm_id in defaults.ARM_IDS:
            grippers.setdefault(arm_id, GripperConfig(arm_id=arm_id))
        object.__setattr__(self, 'grippers', MappingProxyType(grippers))

    def gripper(self, arm_id):
        """Return the GripperConfig of one arm."""
        if arm_id not in self.grippers:
            raise ValueError('unknown arm_id: {!r}'.format(arm_id))
        return self.grippers[arm_id]

    def robot_ip(self, arm_id):
        """Return the configured address of one arm."""
        if arm_id not in self.robot_ips:
            raise ValueError('unknown arm_id: {!r}'.format(arm_id))
        return self.robot_ips[arm_id]

    def profile(self, arm_id):
        """Return the MotionProfile of one arm."""
        if arm_id not in self.profiles:
            raise ValueError('unknown arm_id: {!r}'.format(arm_id))
        return self.profiles[arm_id]

    def public_view(self):
        """Return the body GET /api/config publishes, minus its ``ok`` key."""
        return {
            'config_path': self.config_path,
            'config_present': self.config_present,
            'port': self.port,
            'bind': self.bind,
            'ros_domain_id': self.ros_domain_id,
            'state_dir': self.state_dir,
            'recording_root': self.recording_root,
            'recording_enabled': self.recording_enabled,
            # null is the wire spelling of `unlimited`: no cap in force.
            'recording_max_total_gb': self.recording_max_total_gb,
            'jog_step_rad': self.jog_step_rad,
            'robots': dict(self.robot_ips),
            'settling': self.settling.public_view(),
            'profiles': {arm_id: profile.public_view()
                         for arm_id, profile in self.profiles.items()},
            # Only the arms that HAVE a gripper; {} is the same "no gripper
            # anywhere" signal capabilities.gripper_arms gives, from the same
            # source.
            'grippers': {arm_id: gripper.public_view()
                         for arm_id, gripper in self.grippers.items()
                         if gripper.enabled},
        }


# --- path expansion ----------------------------------------------------------

_VARIABLE_RE = re.compile(r'\$(\w+|\{[^}]*\})')


def _expand(environ, text):
    """Expand ``$VAR`` references and a leading ``~`` using ``environ`` only."""
    def replace(match):
        name = match.group(1)
        if name.startswith('{'):
            name = name[1:-1]
        value = environ.get(name)
        return match.group(0) if value is None else value

    expanded = _VARIABLE_RE.sub(replace, text)
    if expanded == '~' or expanded.startswith('~/'):
        home = environ.get('HOME')
        if not home or not os.path.isabs(home):
            home = os.path.expanduser('~')
        expanded = home.rstrip('/') + expanded[1:]
    return expanded


def default_config_path(environ=None):
    """Return XDG_CONFIG_HOME/franka_web/config.yaml, else ~/.config/…."""
    if environ is None:
        environ = os.environ
    xdg = environ.get('XDG_CONFIG_HOME')
    if xdg and os.path.isabs(xdg):
        return os.path.join(xdg, 'franka_web', 'config.yaml')
    home = environ.get('HOME')
    if not home or not os.path.isabs(home):
        home = os.path.expanduser('~')
    return os.path.join(home, '.config', 'franka_web', 'config.yaml')


def _ensure_directory(dotted, path, make_dirs):
    """Create ``path`` at mode 0700 when it is missing; never touch an existing one."""
    if os.path.isdir(path):
        return path
    if os.path.lexists(path):
        raise ConfigError(dotted, 'expected a directory, found a file at {}.'.format(path))
    if not make_dirs:
        return path
    try:
        os.makedirs(path, mode=0o700, exist_ok=True)
        os.chmod(path, 0o700)
    except OSError as error:
        raise ConfigError(dotted, 'could not be created at {}: {}. Allowed: any '
                          'absolute path this user can create.'.format(
                              path, error.strerror)) from None
    return path


# --- the file ----------------------------------------------------------------

def _read_file(path):
    """Return the parsed mapping of the config file and whether it existed."""
    if not os.path.exists(path):
        return {}, False
    try:
        with open(path, 'rb') as handle:
            size = os.fstat(handle.fileno()).st_size
            if size > defaults.CONFIG_FILE_MAXIMUM_BYTES:
                raise ConfigError('', 'is larger than 1 MiB; a config file is a '
                                  'few dozen lines.')
            raw_bytes = handle.read()
    except ConfigError:
        raise
    except OSError as error:
        raise ConfigError('', 'could not be read: {}.'.format(error.strerror)) from None
    try:
        text = raw_bytes.decode('utf-8')
    except UnicodeDecodeError:
        raise ConfigError('', 'is not valid UTF-8 text.') from None
    try:
        parsed = yaml.safe_load(text)
    except yaml.YAMLError as error:
        problem = str(getattr(error, 'problem', None) or 'the file is malformed')
        mark = getattr(error, 'problem_mark', None)
        if mark is not None:
            raise ConfigError('', 'could not be parsed as YAML at line {}, column '
                              '{}: {}.'.format(mark.line + 1, mark.column + 1,
                                               problem.rstrip('.'))) from None
        raise ConfigError('', 'could not be parsed as YAML: {}.'.format(
            problem.rstrip('.'))) from None
    if parsed is None:
        return {}, True
    if not isinstance(parsed, dict):
        raise ConfigError('', 'must contain a mapping of settings, found {}.'.format(
            _found_wrong_type(parsed)))
    return parsed, True


_DECIMAL_RE = re.compile('[0-9]+')


def _environment_domain_id(environ):
    """Return a usable ROS_DOMAIN_ID from the environment, else 0."""
    text = environ.get('ROS_DOMAIN_ID')
    if text is None or not _DECIMAL_RE.fullmatch(text):
        return 0
    value = int(text, 10)
    return value if 0 <= value <= defaults.ROS_DOMAIN_ID_MAXIMUM else 0


def _read_settling(raw):
    """Return the SettlingConfig described by the file's ``settling`` section."""
    section = _section(raw, 'settling', '')
    stored = dict(defaults.DEFAULT_SETTLING)
    angular = (
        ('drift_limit_deg', 'drift_limit_rad', 'deg', 0.0, True,
         defaults.SETTLING_DRIFT_MAXIMUM_DEG),
        ('span_limit_deg', 'span_limit_rad', 'deg', 0.0, True, None),
        ('velocity_limit_deg_s', 'velocity_limit_rad_s', 'deg/s', 0.0, True, None),
        ('fence_margin_deg', 'fence_margin_rad', 'deg', 0.0, False, None),
    )
    for key, field_name, unit, minimum, exclusive, maximum in angular:
        if key not in section:
            continue
        degrees = broadcast7('settling.{}'.format(key), section[key], unit=unit,
                             minimum=minimum, maximum=maximum,
                             exclusive_minimum=exclusive)
        stored[field_name] = degrees_to_radians(degrees)
    stored['stable_window_s'] = _read_positive_number(
        section, 'stable_window_s', 'settling.stable_window_s',
        defaults.DEFAULT_SETTLING['stable_window_s'])
    stored['min_samples'] = _read_min_samples(
        section, 'min_samples', 'settling.min_samples',
        defaults.DEFAULT_SETTLING['min_samples'])
    stored['timeout_s'] = _read_bounded_number(
        section, 'timeout_s', 'settling.timeout_s',
        defaults.DEFAULT_SETTLING['timeout_s'],
        maximum=defaults.ACTIVATION_SETTLING_MAX_TIMEOUT_S, unit='s')

    window = stored['stable_window_s']
    samples = stored['min_samples']
    timeout = stored['timeout_s']
    if timeout <= window:
        raise ConfigError('settling.timeout_s', (
            'expected a value greater than settling.stable_window_s ({} s), '
            'found {}.').format(_num(window), _num(timeout)))
    tick_ns = int(defaults.SUPERVISOR_TICK_S * 1e9)
    window_ns = math.ceil(window * 1e9)
    stable_span_ns = ((window_ns + tick_ns - 1) // tick_ns) * tick_ns
    sample_span_ns = (samples - 1) * tick_ns
    minimum_total_ns = (2 * tick_ns) + max(stable_span_ns, sample_span_ns)
    if minimum_total_ns >= math.ceil(timeout * 1e9):
        raise ConfigError('settling.timeout_s', (
            '{} samples and a {} s stable window need at least {} s at the '
            "supervisor's {} s cadence, but timeout_s is {}. Raise "
            'settling.timeout_s, or lower settling.min_samples / '
            'settling.stable_window_s.').format(
                _num(samples), _num(window), _num(round(minimum_total_ns / 1e9, 6)),
                _num(defaults.SUPERVISOR_TICK_S), _num(timeout)))
    try:
        return SettlingConfig(**stored)
    except ValueError as error:
        raise ConfigError('settling', 'is internally inconsistent: {}'.format(
            error)) from None


def _read_speed_limit(dotted, value):
    """Return seven target-velocity limits in rad/s from a degree-per-second key."""
    degrees = broadcast7(dotted, value, unit='deg/s', minimum=0.0)
    indexed = isinstance(value, (list, tuple))
    result = []
    for index, element in enumerate(degrees_to_radians(degrees)):
        ceiling = defaults.POLICY_VELOCITY_CEILING_RAD_S[index]
        element = _snap_high(element, ceiling)
        if element > ceiling:
            name = '{}[{}]'.format(dotted, index) if indexed else dotted
            raise ConfigError(name, (
                'expected a value in 0 < x <= {} deg/s (the factory URDF '
                'velocity ceiling for joint {}), found {}.').format(
                    _num(_deg_upper(ceiling)), index + 1, _num(degrees[index])))
        result.append(element)
    return tuple(result)


def _read_fence_bounds(arm_id, fence_map, enabled):
    """Return the arm's position bounds in radians, refusing an inert or bad box."""
    dotted = 'fence.{}'.format(arm_id)
    lower_policy = defaults.POLICY_POSITION_LOWER_RAD
    upper_policy = defaults.POLICY_POSITION_UPPER_RAD
    if not enabled:
        for key in ('lower_deg', 'upper_deg'):
            if key in fence_map:
                raise ConfigError('{}.{}'.format(dotted, key), (
                    'set {}.enabled: true to use these bounds, or remove '
                    'lower_deg and upper_deg. With the fence off the arm uses '
                    'the Panda factory limits.').format(dotted))
        return lower_policy, upper_policy

    lower_degrees = None
    upper_degrees = None
    inside = 'any value inside the Panda factory limits'
    if 'lower_deg' in fence_map:
        lower_degrees = _list7(
            '{}.lower_deg'.format(dotted), fence_map['lower_deg'], unit='deg',
            allowed=inside,
            example=tuple(_deg_lower(bound) for bound in lower_policy))
    if 'upper_deg' in fence_map:
        upper_degrees = _list7(
            '{}.upper_deg'.format(dotted), fence_map['upper_deg'], unit='deg',
            allowed=inside,
            example=tuple(_deg_upper(bound) for bound in upper_policy))

    lower = list(lower_policy if lower_degrees is None
                 else degrees_to_radians(lower_degrees))
    upper = list(upper_policy if upper_degrees is None
                 else degrees_to_radians(upper_degrees))
    shown_lower = [(_deg_lower(bound) if lower_degrees is None
                    else lower_degrees[index])
                   for index, bound in enumerate(lower_policy)]
    shown_upper = [(_deg_upper(bound) if upper_degrees is None
                    else upper_degrees[index])
                   for index, bound in enumerate(upper_policy)]

    for index in range(defaults.JOINT_COUNT):
        lower[index] = _snap_low(lower[index], lower_policy[index])
        if lower[index] < lower_policy[index]:
            raise ConfigError('{}.lower_deg[{}]'.format(dotted, index), (
                'expected a value of at least {} deg (the Panda joint-{} '
                'factory lower limit), found {}.').format(
                    _num(_deg_lower(lower_policy[index])), index + 1,
                    _num(shown_lower[index])))
        upper[index] = _snap_high(upper[index], upper_policy[index])
        if upper[index] > upper_policy[index]:
            raise ConfigError('{}.upper_deg[{}]'.format(dotted, index), (
                'expected a value of at most {} deg (the Panda joint-{} '
                'factory upper limit), found {}.').format(
                    _num(_deg_upper(upper_policy[index])), index + 1,
                    _num(shown_upper[index])))
        if lower[index] >= upper[index]:
            raise ConfigError(dotted, (
                'joint {} lower_deg ({}) must be less than upper_deg '
                '({}).').format(index + 1, _num(shown_lower[index]),
                                _num(shown_upper[index])))
    return tuple(lower), tuple(upper)


def _read_profile(arm_id, profiles_raw, fence_raw):
    """Return one arm's MotionProfile from the profiles and fence sections."""
    profile_map = _section(profiles_raw, arm_id, 'profiles')
    fence_map = _section(fence_raw, arm_id, 'fence')
    baked = defaults.DEFAULT_PROFILES[arm_id]
    dotted = 'profiles.{}'.format(arm_id)

    k_gains = baked['k_gains']
    if 'stiffness' in profile_map:
        name = '{}.stiffness'.format(dotted)
        k_gains = _list7(name, profile_map['stiffness'], unit=None,
                         allowed='any value of 0 or more', example=baked['k_gains'])
        for index, element in enumerate(k_gains):
            _check_range('{}[{}]'.format(name, index), element, None, 0.0, None, False)

    d_gains = baked['d_gains']
    if 'damping' in profile_map:
        name = '{}.damping'.format(dotted)
        d_gains = _list7(name, profile_map['damping'], unit=None,
                         allowed='any value of 0 or more', example=baked['d_gains'])
        for index, element in enumerate(d_gains):
            _check_range('{}[{}]'.format(name, index), element, None, 0.0, None, False)

    max_effort_nm = baked['max_effort_nm']
    if 'torque_limit_nm' in profile_map:
        name = '{}.torque_limit_nm'.format(dotted)
        max_effort_nm = _list7(name, profile_map['torque_limit_nm'], unit='N.m',
                               allowed='any value greater than 0',
                               example=baked['max_effort_nm'])
        for index, element in enumerate(max_effort_nm):
            ceiling = defaults.POLICY_EFFORT_CEILING_NM[index]
            if not 0.0 < element <= ceiling:
                raise ConfigError('{}[{}]'.format(name, index), (
                    'expected a value in 0 < x <= {} N·m (the Panda '
                    'joint-{} hardware ceiling), found {}.').format(
                        _num(ceiling), index + 1, _num(element)))

    max_target_velocity_rad_s = baked['max_target_velocity_rad_s']
    if 'speed_limit_deg_s' in profile_map:
        max_target_velocity_rad_s = _read_speed_limit(
            '{}.speed_limit_deg_s'.format(dotted), profile_map['speed_limit_deg_s'])

    fence_enabled = _read_bool(fence_map, 'enabled',
                               'fence.{}.enabled'.format(arm_id), False)
    lower, upper = _read_fence_bounds(arm_id, fence_map, fence_enabled)

    return MotionProfile(
        arm_id=arm_id,
        k_gains=k_gains,
        d_gains=d_gains,
        max_effort_nm=max_effort_nm,
        max_target_velocity_rad_s=max_target_velocity_rad_s,
        watchdog_timeout_s=defaults.REVIEWED_TIMING_S['watchdog_timeout'],
        max_header_age_s=defaults.REVIEWED_TIMING_S['max_header_age'],
        future_tolerance_s=defaults.REVIEWED_TIMING_S['future_tolerance'],
        position_lower_rad=lower,
        position_upper_rad=upper,
        fence_enabled=fence_enabled,
        from_file=bool(profile_map) or bool(fence_map),
    )


def _read_gripper(arm_id, grippers_raw):
    """Return one arm's GripperConfig from the ``grippers`` section."""
    gripper_map = _section(grippers_raw, arm_id, 'grippers')
    dotted = 'grippers.{}'.format(arm_id)
    baked = defaults.DEFAULT_GRIPPER

    def number(key, bounds, unit):
        """Read one inclusive-range number under this arm's dotted key."""
        return _read_closed_range_number(
            gripper_map, key, '{}.{}'.format(dotted, key), baked[key],
            minimum=bounds[0], maximum=bounds[1], unit=unit)

    width_bounds = (0.0, defaults.GRIPPER_STROKE_MM)
    open_width_mm = number('open_width_mm', width_bounds, 'mm')
    close_width_mm = number('close_width_mm', width_bounds, 'mm')
    if close_width_mm >= open_width_mm:
        raise ConfigError('{}.close_width_mm'.format(dotted), (
            'expected a value below {}.open_width_mm ({}), found {}. A close '
            'width at or above the open width would make /close and /open the '
            'same command.').format(dotted, _num(open_width_mm),
                                    _num(close_width_mm)))
    return GripperConfig(
        arm_id=arm_id,
        enabled=_read_bool(gripper_map, 'enabled',
                           '{}.enabled'.format(dotted), baked['enabled']),
        serial_id=_read_device_name(gripper_map, 'serial_id',
                                    '{}.serial_id'.format(dotted),
                                    baked['serial_id']),
        usb_path=_read_device_name(gripper_map, 'usb_path',
                                   '{}.usb_path'.format(dotted),
                                   baked['usb_path']),
        speed_mm_s=number('speed_mm_s', defaults.GRIPPER_SPEED_RANGE_MM_S, 'mm/s'),
        force_n=number('force_n', defaults.GRIPPER_FORCE_RANGE_N, 'N'),
        open_width_mm=open_width_mm,
        close_width_mm=close_width_mm,
        poll_rate_hz=number('poll_rate_hz',
                            defaults.GRIPPER_POLL_RATE_RANGE_HZ, 'Hz'),
        auto_activate=_read_bool(gripper_map, 'auto_activate',
                                 '{}.auto_activate'.format(dotted),
                                 baked['auto_activate']),
        motion_timeout_s=number('motion_timeout_s',
                                defaults.GRIPPER_MOTION_TIMEOUT_RANGE_S, 's'),
        activation_timeout_s=number(
            'activation_timeout_s',
            defaults.GRIPPER_ACTIVATION_TIMEOUT_RANGE_S, 's'),
        reconnect_interval_s=number(
            'reconnect_interval_s',
            defaults.GRIPPER_RECONNECT_INTERVAL_RANGE_S, 's'),
        joint_names=_read_joint_names(gripper_map, 'joint_names',
                                      '{}.joint_names'.format(dotted),
                                      default_joint_names(arm_id)),
        from_file=bool(gripper_map),
    )


def _validate_bindings(bindings):
    """
    Apply the three cross-field binding rules through franka_robotiq's own text.

    The import is LAZY and GUARDED on purpose: franka_web must build, start,
    serve and pass its suite on a workspace where franka_robotiq was never
    built, so a cell with no gripper enabled never reaches this import at all,
    and a cell that DOES enable one gets a teaching refusal rather than a
    crash. discovery.py owns the three sentences; this loader does not restate
    them.
    """
    if not any(binding.enabled for binding in bindings.values()):
        return
    try:
        from franka_robotiq import discovery
    except ImportError:
        raise ConfigError(
            'grippers',
            'a gripper is enabled but the franka_robotiq package is not '
            'installed. Build the workspace (colcon build) and source it, or '
            'set grippers.panda1.enabled and grippers.panda2.enabled to '
            'false.') from None
    for arm_id, binding in sorted(bindings.items()):
        if not binding.enabled:
            continue
        try:
            discovery.check_binding(arm_id, binding.serial_id, binding.usb_path)
        except discovery.BindingError as error:
            raise ConfigError('grippers.{}'.format(arm_id), str(error)) from None
    try:
        # check_cross_arm takes a MAPPING {arm_id: (serial_id, usb_path)} with
        # '' for the unset half of the pair.
        discovery.check_cross_arm({arm_id: (binding.serial_id, binding.usb_path)
                                   for arm_id, binding in bindings.items()
                                   if binding.enabled})
    except discovery.BindingError as error:
        raise ConfigError('grippers', str(error)) from None


def _read_grippers(raw):
    """Return both arms' GripperConfig, cross-checked against each other."""
    grippers_raw = _section(raw, 'grippers', '')
    grippers = {arm_id: _read_gripper(arm_id, grippers_raw)
                for arm_id in defaults.ARM_IDS}
    _validate_bindings(grippers)
    return grippers


def load(path=None, environ=None, *, make_dirs=True):
    """
    Read, validate and return Settings; raise ConfigError with one message.

    ``path`` defaults to default_config_path(environ). ``environ`` defaults to
    os.environ and is read for XDG_CONFIG_HOME, HOME and ROS_DOMAIN_ID only.
    ``make_dirs=False`` skips directory creation (used by --check-config and
    by tests).
    """
    if environ is None:
        environ = os.environ
    if path is None:
        path = default_config_path(environ)
    try:
        return _load_validated(path, environ, make_dirs)
    except ConfigError as error:
        raise error.with_path(path) from None


def _load_validated(path, environ, make_dirs):
    """Do the whole validation; every ConfigError is path-prefixed by ``load``."""
    raw, present = _read_file(path)
    _validate_structure(raw, '')

    port = _read_port(raw, 'port', 'port', defaults.DEFAULT_PORT)
    bind = _read_bind(raw, 'bind', 'bind', defaults.DEFAULT_BIND)
    domain_id = _read_domain_id(raw, 'ros_domain_id', 'ros_domain_id')
    if domain_id is None:
        domain_id = _environment_domain_id(environ)

    robots_raw = _section(raw, 'robots', '')
    robot_ips = {}
    for arm_id in defaults.ARM_IDS:
        arm_map = _section(robots_raw, arm_id, 'robots')
        robot_ips[arm_id] = _read_ip(arm_map, 'ip', 'robots.{}.ip'.format(arm_id),
                                     defaults.DEFAULT_ROBOT_IPS[arm_id])

    directories = _section(raw, 'directories', '')
    state_dir = _read_directory(directories, 'state', 'directories.state',
                                defaults.DEFAULT_STATE_DIR, environ)
    recording_root = _read_directory(directories, 'recordings',
                                     'directories.recordings',
                                     defaults.DEFAULT_RECORDING_ROOT, environ)
    franka_dir = _read_optional_directory(directories, 'franka_dir',
                                          'directories.franka_dir', environ)

    recording = _section(raw, 'recording', '')
    recording_enabled = _read_bool(recording, 'enabled', 'recording.enabled',
                                   defaults.DEFAULT_RECORDING_ENABLED)

    recordings = _section(raw, 'recordings', '')
    recording_max_total_gb = _read_size_cap_gb(
        recordings, 'max_total_gb', 'recordings.max_total_gb',
        defaults.DEFAULT_RECORDING_MAX_TOTAL_GB)

    jog = _section(raw, 'jog', '')
    if 'step_deg' in jog:
        step_degrees = _read_bounded_number(
            jog, 'step_deg', 'jog.step_deg', None,
            maximum=defaults.JOG_STEP_MAXIMUM_DEG, unit='deg')
        jog_step_rad = degrees_to_radians(step_degrees)
    else:
        jog_step_rad = defaults.JOG_STEP_RAD

    settling = _read_settling(raw)

    profiles_raw = _section(raw, 'profiles', '')
    fence_raw = _section(raw, 'fence', '')
    profiles = {arm_id: _read_profile(arm_id, profiles_raw, fence_raw)
                for arm_id in defaults.ARM_IDS}
    grippers = _read_grippers(raw)

    state_dir = _ensure_directory('directories.state', state_dir, make_dirs)
    recording_root = _ensure_directory('directories.recordings', recording_root,
                                       make_dirs)

    return Settings(
        bind=bind,
        port=port,
        ros_domain_id=domain_id,
        state_dir=state_dir,
        recording_root=recording_root,
        franka_dir=franka_dir,
        recording_enabled=recording_enabled,
        jog_step_rad=jog_step_rad,
        robot_ips=robot_ips,
        settling=settling,
        profiles=profiles,
        config_path=path,
        config_present=present,
        grippers=grippers,
        recording_max_total_gb=recording_max_total_gb,
    )
