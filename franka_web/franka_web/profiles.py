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

:func:`argv_for` is a PURE function of ``(arms, mode, settings)`` plus the one
motion-only controller-parameter argument. Nothing here spawns, forks, reads the
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

Robot addresses come from the server's configuration file (or its baked-in
defaults, ``172.16.0.2`` / ``172.16.0.3``), never from the browser. Every arm
is bound to its OWN address key: there is no shared single-address key, which
is what caused a live cross-robot mislabel, so a single-arm session takes its
address from ``robots.<selected arm>.ip`` and nothing else.
"""

from collections import namedtuple

from franka_web import defaults


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

# (launch argument, arm id) per arm mode, where ``None`` means "this profile's
# single selected arm". There is deliberately no shared address key: a single
# `panda2` session emits `robot_ip:=<panda2's own address>`.
_ADDRESS_SOURCES = {
    'single': (
        ('robot_ip', None),
    ),
    'dual': (
        ('robot_ip_1', 'panda1'),
        ('robot_ip_2', 'panda2'),
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


def _controller_pairs(profile, controller_param_file):
    """
    Validate the motion-only controller argument and return its pairs.

    The controller itself is no longer a choice: a motion session always runs
    ``defaults.MOTION_CONTROLLER``. What still varies is the materialized
    parameter file, and a blank string counts as absent.
    """
    controller_param_file = _present(controller_param_file)
    if not profile.allows_motion:
        if controller_param_file is not None:
            raise ProfileError(
                'controller_param_file is accepted only by a motion profile')
        return ()
    if controller_param_file is None:
        raise ProfileError('controller_param_file is required by a motion profile')
    return (
        ('allow_motion', 'true'),
        ('controller_name', defaults.MOTION_CONTROLLER),
        ('controller_param_file', controller_param_file),
    )


def _address_pairs(profile, settings):
    """
    Return the address arguments this profile needs, or raise naming the gaps.

    Every configured address has a default, so the missing-address refusal is
    a defensive path only. It names the arm and the config key to set; the
    address itself is not part of the sentence because the subject of the
    sentence is its absence.
    """
    if not profile.requires_addresses:
        return ()
    pairs = []
    missing = []
    for argument_name, arm_id in _ADDRESS_SOURCES[profile.arm_mode]:
        arm_id = profile.arm_ids[0] if arm_id is None else arm_id
        try:
            value = _present(settings.robot_ip(arm_id))
        except (AttributeError, KeyError, TypeError):
            value = None
        if value is None:
            missing.append(arm_id)
        else:
            pairs.append((argument_name, value))
    if missing:
        raise ProfileError(
            'no address is configured for {}; set {} in config.yaml'.format(
                ', '.join(missing),
                ', '.join('robots.{}.ip'.format(arm) for arm in missing)))
    return tuple(pairs)


def argv_for(arms, mode, settings, *, controller_param_file=None):
    """
    Build the full ``ros2 launch`` argv for one profile, or raise.

    Returns ``('ros2', 'launch', 'franka_bringup', <launch file>, '<arg>:=<value>', ...)``
    carrying exactly and only the arguments that launch file declares:
    ``use_rviz:=false`` always (a headless server has nobody to show RViz to),
    ``allow_motion:=true`` and the two controller arguments only on the three
    motion rows, and addresses only on the six production rows.

    Raises :class:`ProfileError` when ``(arms, mode)`` is not one of the nine
    rows, when a required address is absent from ``settings``, when a motion
    profile is missing its parameter file, or when a parameter file is
    supplied for a profile that cannot accept one.
    """
    profile = _lookup(arms, mode)
    controller = _controller_pairs(profile, controller_param_file)
    addresses = _address_pairs(profile, settings)

    pairs = []
    if profile.arm_mode == 'single':
        pairs.append(('arm_id', profile.arm_ids[0]))
    pairs.extend(addresses)
    pairs.extend(controller)
    pairs.append(('use_rviz', 'false'))

    return _LAUNCH_PREFIX + (profile.launch_file,) + tuple(
        '{}:={}'.format(name, value) for name, value in pairs)
