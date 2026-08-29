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
The frozen launch-profile table, and pure argv construction from it.

Nine ``(arms, mode)`` combinations are reachable from the Session card, and
each one maps to exactly one installed launch file under
``franka_bringup/launch/operator/``. That mapping is frozen here: the web
server never assembles a launch invocation any other way, so the complete set
of command lines this package can ever produce is the nine rows below.

:func:`argv_for` is a PURE function of ``(arms, mode, settings)`` plus the two
motion-only controller arguments. Nothing here spawns, forks, reads the
environment, or touches the filesystem -- which is what lets the six
production rows (``watch`` and ``motion``) be tested exhaustively without ever
being executed. Keep it that way: an impure helper here would silently take
the never-execute guarantee of plan section 4 with it.

Argument set per launch file (plan section 0.1, re-verified against the
installed launch files at ``27ea21f``):

* ``fake_single_state_only``        -- ``arm_id``, ``use_rviz``
* ``fake_dual_state_only``          -- ``use_rviz``
* ``production_single_state_only``  -- ``arm_id``, ``robot_ip``, ``use_rviz``
* ``production_dual_state_only``    -- ``robot_ip_1``, ``robot_ip_2``, ``use_rviz``
* ``production_single_guarded_motion`` -- ``arm_id``, ``robot_ip``, ``allow_motion``,
  ``controller_name``, ``controller_param_file``, ``use_rviz``
* ``production_dual_guarded_motion``   -- ``robot_ip_1``, ``robot_ip_2``, ``allow_motion``,
  ``controller_name``, ``controller_param_file``, ``use_rviz``

The single-arm launches take a required, defaultless ``arm_id`` (``panda1``
or ``panda2``); there is no ``arm_id_1``/``arm_id_2`` on the single-arm path.
``arm_id_1``/``arm_id_2``, ``load_gripper*``, ``fake_sensor_commands`` and
``use_fake_hardware`` are hard-wired inside ``operator_launch.py`` and are not
operator-settable, so this module never emits them. Emitting an argument a
launch file does not declare makes ``ros2 launch`` fail outright, so the
per-profile argument set is as much a correctness rule as a policy one.

Robot addresses come from the SERVER environment only (see
``config.Settings.from_env``), never from the browser, and never appear in an
error raised here: a missing address is reported by naming the environment
variable that is unset, and nothing else.
"""

from collections import namedtuple

Profile = namedtuple('Profile', 'launch_file arm_ids arm_mode requires_addresses allows_motion')

PROFILES = {
    ('panda1', 'simulate'): Profile(
        'fake_single_state_only.launch.py', ('panda1',), 'single', False, False),
    ('panda2', 'simulate'): Profile(
        'fake_single_state_only.launch.py', ('panda2',), 'single', False, False),
    ('both', 'simulate'): Profile(
        'fake_dual_state_only.launch.py', ('panda1', 'panda2'), 'dual', False, False),
    ('panda1', 'watch'): Profile(
        'production_single_state_only.launch.py', ('panda1',), 'single', True, False),
    ('panda2', 'watch'): Profile(
        'production_single_state_only.launch.py', ('panda2',), 'single', True, False),
    ('both', 'watch'): Profile(
        'production_dual_state_only.launch.py', ('panda1', 'panda2'), 'dual', True, False),
    ('panda1', 'motion'): Profile(
        'production_single_guarded_motion.launch.py', ('panda1',), 'single', True, True),
    ('panda2', 'motion'): Profile(
        'production_single_guarded_motion.launch.py', ('panda2',), 'single', True, True),
    ('both', 'motion'): Profile(
        'production_dual_guarded_motion.launch.py', ('panda1', 'panda2'), 'dual', True, True),
}

# (launch argument, Settings field, environment variable) per arm mode. The
# environment-variable names are the ones config.Settings.from_env reads; they
# appear here only so a refusal can say which one is unset.
_ADDRESS_SOURCES = {
    'single': (
        ('robot_ip', 'robot_ip_single', 'FRANKA_WEB_ROBOT_IP'),
    ),
    'dual': (
        ('robot_ip_1', 'robot_ip_1', 'FRANKA_WEB_ROBOT_IP_1'),
        ('robot_ip_2', 'robot_ip_2', 'FRANKA_WEB_ROBOT_IP_2'),
    ),
}

_LAUNCH_PREFIX = ('ros2', 'launch', 'franka_bringup')


class ProfileError(ValueError):
    """The requested profile does not exist, or cannot be launched as asked."""


def _describe_combinations():
    """Return the nine valid ``arms/mode`` combinations as a stable string."""
    return ', '.join('{}/{}'.format(arms, mode) for arms, mode in sorted(PROFILES))


def _lookup(arms, mode):
    """
    Return the :class:`Profile` for ``(arms, mode)`` or raise.

    ``arms`` and ``mode`` arrive from a request body, so they may be of any
    type; an unhashable value is an unknown combination, not a ``TypeError``.
    The refusal never echoes the requested values -- they are attacker-shaped
    strings, and this message reaches a log line.
    """
    try:
        return PROFILES[(arms, mode)]
    except (KeyError, TypeError):
        raise ProfileError(
            'no launch profile for the requested arms/mode combination; '
            'valid combinations are: {}'.format(_describe_combinations())) from None


def _present(value):
    """Return ``value`` stripped, or ``None`` when it is absent or blank."""
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _controller_pairs(profile, controller_name, controller_param_file):
    """
    Validate the motion-only controller arguments and return their pairs.

    A blank string counts as absent: an unset picker and an empty picker are
    the same operator intent, and the guarded launches would reject either
    (their ``controller_name`` default is ``''`` and the guard refuses it).
    """
    controller_name = _present(controller_name)
    controller_param_file = _present(controller_param_file)
    if not profile.allows_motion:
        if controller_name is not None or controller_param_file is not None:
            raise ProfileError(
                'controller_name and controller_param_file are accepted only by a '
                'motion profile')
        return ()
    if controller_name is None:
        raise ProfileError('controller_name is required by a motion profile')
    if controller_param_file is None:
        raise ProfileError('controller_param_file is required by a motion profile')
    return (
        ('allow_motion', 'true'),
        ('controller_name', controller_name),
        ('controller_param_file', controller_param_file),
    )


def _address_pairs(profile, settings):
    """
    Return the address arguments this profile needs, or raise naming the gaps.

    The refusal names the unset environment variables and NOTHING else: no
    address is echoed, not even one that is set (a dual profile missing only
    the second address must not leak the first).
    """
    if not profile.requires_addresses:
        return ()
    pairs = []
    missing = []
    for argument_name, field_name, environment_name in _ADDRESS_SOURCES[profile.arm_mode]:
        value = _present(getattr(settings, field_name, None))
        if value is None:
            missing.append(environment_name)
        else:
            pairs.append((argument_name, value))
    if missing:
        raise ProfileError(
            '{} must be set in the server environment for this profile '
            '(the address itself is never echoed)'.format(', '.join(missing)))
    return tuple(pairs)


def argv_for(arms, mode, settings, controller_name=None, controller_param_file=None):
    """
    Build the full ``ros2 launch`` argv for one profile, or raise.

    Returns ``('ros2', 'launch', 'franka_bringup', <launch file>, '<arg>:=<value>', ...)``
    carrying exactly and only the arguments that launch file declares:
    ``use_rviz:=false`` always (a headless server has nobody to show RViz to),
    ``allow_motion:=true`` and the two controller arguments only on the three
    motion rows, and addresses only on the six production rows.

    Raises :class:`ProfileError` when ``(arms, mode)`` is not one of the nine
    rows, when a required address is absent from ``settings``, when a motion
    profile is missing either controller argument, or when a controller
    argument is supplied for a profile that cannot accept one.
    """
    profile = _lookup(arms, mode)
    controller = _controller_pairs(profile, controller_name, controller_param_file)
    addresses = _address_pairs(profile, settings)

    pairs = []
    if profile.arm_mode == 'single':
        pairs.append(('arm_id', profile.arm_ids[0]))
    pairs.extend(addresses)
    pairs.extend(controller)
    pairs.append(('use_rviz', 'false'))

    return _LAUNCH_PREFIX + (profile.launch_file,) + tuple(
        '{}:={}'.format(name, value) for name, value in pairs)
