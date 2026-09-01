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
Tests for franka_web.profiles: the frozen table and argv_for.

All nine rows are covered here, including the six ``watch``/``motion`` rows
that this session may never execute -- pinning their argv byte for byte in a
pure test is the only review the production command lines get before the one
supervised real session.

Robot addresses in these fixtures are RFC 5737 documentation addresses
(203.0.113.0/24). Every arm is bound to its OWN configured address: there is
no shared single-address key, so a single-arm `panda2` session emits panda2's
address and never panda1's -- the live cross-robot mislabel that key caused.
"""

from franka_web import defaults, profiles
from franka_web.profiles import argv_for, Profile, ProfileError, PROFILES
import pytest
from support.config_factory import make_settings

DOC_IP_1 = '203.0.113.7'
DOC_IP_2 = '203.0.113.8'
DOC_ADDRESSES = (DOC_IP_1, DOC_IP_2)

CONTROLLER = defaults.MOTION_CONTROLLER
PARAM_FILE = '/var/lib/franka_web/profiles/a1b2c3.yaml'

PREFIX = ('ros2', 'launch', 'franka_bringup')

SIMULATE_ROWS = (('panda1', 'simulate'), ('panda2', 'simulate'), ('both', 'simulate'))
WATCH_ROWS = (('panda1', 'watch'), ('panda2', 'watch'), ('both', 'watch'))
MOTION_ROWS = (('panda1', 'motion'), ('panda2', 'motion'), ('both', 'motion'))
ALL_ROWS = SIMULATE_ROWS + WATCH_ROWS + MOTION_ROWS

# Plan section 4, transcribed independently of the module under test.
EXPECTED_PROFILES = {
    ('panda1', 'simulate'): (
        'fake_single_state_only.launch.py', ('panda1',), 'single', False, False),
    ('panda2', 'simulate'): (
        'fake_single_state_only.launch.py', ('panda2',), 'single', False, False),
    ('both', 'simulate'): (
        'fake_dual_state_only.launch.py', ('panda1', 'panda2'), 'dual', False, False),
    ('panda1', 'watch'): (
        'production_single_state_only.launch.py', ('panda1',), 'single', True, False),
    ('panda2', 'watch'): (
        'production_single_state_only.launch.py', ('panda2',), 'single', True, False),
    ('both', 'watch'): (
        'production_dual_state_only.launch.py', ('panda1', 'panda2'), 'dual', True, False),
    ('panda1', 'motion'): (
        'production_single_guarded_motion.launch.py', ('panda1',), 'single', True, True),
    ('panda2', 'motion'): (
        'production_single_guarded_motion.launch.py', ('panda2',), 'single', True, True),
    ('both', 'motion'): (
        'production_dual_guarded_motion.launch.py', ('panda1', 'panda2'), 'dual', True, True),
}

# The complete set of command lines this package can ever produce.
EXPECTED_ARGV = {
    ('panda1', 'simulate'): PREFIX + (
        'fake_single_state_only.launch.py',
        'arm_id:=panda1',
        'use_rviz:=false'),
    ('panda2', 'simulate'): PREFIX + (
        'fake_single_state_only.launch.py',
        'arm_id:=panda2',
        'use_rviz:=false'),
    ('both', 'simulate'): PREFIX + (
        'fake_dual_state_only.launch.py',
        'use_rviz:=false'),
    ('panda1', 'watch'): PREFIX + (
        'production_single_state_only.launch.py',
        'arm_id:=panda1',
        'robot_ip:=' + DOC_IP_1,
        'use_rviz:=false'),
    ('panda2', 'watch'): PREFIX + (
        'production_single_state_only.launch.py',
        'arm_id:=panda2',
        'robot_ip:=' + DOC_IP_2,
        'use_rviz:=false'),
    ('both', 'watch'): PREFIX + (
        'production_dual_state_only.launch.py',
        'robot_ip_1:=' + DOC_IP_1,
        'robot_ip_2:=' + DOC_IP_2,
        'use_rviz:=false'),
    ('panda1', 'motion'): PREFIX + (
        'production_single_guarded_motion.launch.py',
        'arm_id:=panda1',
        'robot_ip:=' + DOC_IP_1,
        'allow_motion:=true',
        'controller_name:=' + CONTROLLER,
        'controller_param_file:=' + PARAM_FILE,
        'use_rviz:=false'),
    ('panda2', 'motion'): PREFIX + (
        'production_single_guarded_motion.launch.py',
        'arm_id:=panda2',
        'robot_ip:=' + DOC_IP_2,
        'allow_motion:=true',
        'controller_name:=' + CONTROLLER,
        'controller_param_file:=' + PARAM_FILE,
        'use_rviz:=false'),
    ('both', 'motion'): PREFIX + (
        'production_dual_guarded_motion.launch.py',
        'robot_ip_1:=' + DOC_IP_1,
        'robot_ip_2:=' + DOC_IP_2,
        'allow_motion:=true',
        'controller_name:=' + CONTROLLER,
        'controller_param_file:=' + PARAM_FILE,
        'use_rviz:=false'),
}


@pytest.fixture()
def settings(tmp_path):
    """Return Settings carrying the two documentation addresses."""
    return make_settings(
        tmp_path, robot_ips={'panda1': DOC_IP_1, 'panda2': DOC_IP_2})


def _argv(arms, mode, settings):
    """Call argv_for, supplying the parameter file only on motion rows."""
    if PROFILES[(arms, mode)].allows_motion:
        return argv_for(arms, mode, settings, controller_param_file=PARAM_FILE)
    return argv_for(arms, mode, settings)


class TestFrozenTable:
    """PROFILES is the frozen plan section 4 table, row for row."""

    def test_exactly_nine_rows_with_the_expected_keys(self):
        """No row may be added or dropped without changing the plan."""
        assert set(PROFILES) == set(EXPECTED_PROFILES)
        assert len(PROFILES) == 9

    def test_profile_field_names(self):
        """The namedtuple field order is part of the frozen contract."""
        assert Profile._fields == (
            'launch_file', 'arm_ids', 'arm_mode', 'requires_addresses', 'allows_motion')

    @pytest.mark.parametrize('row', ALL_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_row_matches_the_plan_field_for_field(self, row):
        """Each row equals its plan section 4 tuple exactly."""
        assert tuple(PROFILES[row]) == EXPECTED_PROFILES[row]

    def test_only_the_simulate_rows_are_fake(self):
        """Simulate is the only path that needs no address; it never moves."""
        for row in SIMULATE_ROWS:
            assert PROFILES[row].launch_file.startswith('fake_')
            assert PROFILES[row].requires_addresses is False
            assert PROFILES[row].allows_motion is False

    def test_only_the_motion_rows_allow_motion(self):
        """allows_motion is true for the three guarded-motion rows only."""
        allowed = {row for row, profile in PROFILES.items() if profile.allows_motion}
        assert allowed == set(MOTION_ROWS)

    def test_every_production_row_requires_addresses(self):
        """Watch and motion both reach a real robot, so both need addresses."""
        required = {row for row, profile in PROFILES.items() if profile.requires_addresses}
        assert required == set(WATCH_ROWS + MOTION_ROWS)

    def test_arm_mode_agrees_with_arm_ids(self):
        """arm_mode is derivable from arm_ids; the two never disagree."""
        for profile in PROFILES.values():
            expected = 'single' if len(profile.arm_ids) == 1 else 'dual'
            assert profile.arm_mode == expected
            assert set(profile.arm_ids) <= {'panda1', 'panda2'}


class TestExactArgv:
    """argv_for emits the pinned command line for every one of the nine rows."""

    @pytest.mark.parametrize('row', ALL_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_argv_is_exact(self, row, settings):
        """The whole argv, in order, matches the pinned expectation."""
        assert _argv(row[0], row[1], settings) == EXPECTED_ARGV[row]

    @pytest.mark.parametrize('row', ALL_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_argv_shape(self, row, settings):
        """Result is a tuple of str beginning with the ros2 launch prefix."""
        argv = _argv(row[0], row[1], settings)
        assert isinstance(argv, tuple)
        assert all(isinstance(token, str) for token in argv)
        assert argv[:3] == PREFIX
        assert argv[3] == PROFILES[row].launch_file

    @pytest.mark.parametrize('row', ALL_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_use_rviz_is_always_false_and_last(self, row, settings):
        """A headless server never opens a window nobody watches."""
        argv = _argv(row[0], row[1], settings)
        assert argv[-1] == 'use_rviz:=false'
        assert sum(token.startswith('use_rviz:=') for token in argv) == 1

    @pytest.mark.parametrize('row', ALL_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_no_undeclared_argument_is_emitted(self, row, settings):
        """The hard-wired operator_launch arguments are never overridden."""
        argv = _argv(row[0], row[1], settings)
        names = {token.split(':=', 1)[0] for token in argv[4:]}
        assert names <= {
            'arm_id', 'robot_ip', 'robot_ip_1', 'robot_ip_2',
            'allow_motion', 'controller_name', 'controller_param_file', 'use_rviz'}
        for forbidden in ('arm_id_1', 'arm_id_2', 'use_fake_hardware',
                          'load_gripper', 'load_gripper_1', 'load_gripper_2',
                          'fake_sensor_commands'):
            assert forbidden not in names

    @pytest.mark.parametrize('row', SIMULATE_ROWS + WATCH_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_allow_motion_absent_outside_motion_rows(self, row, settings):
        """allow_motion appears only where the launch file declares it."""
        argv = _argv(row[0], row[1], settings)
        assert not any(token.startswith('allow_motion') for token in argv)
        assert not any(token.startswith('controller_') for token in argv)

    @pytest.mark.parametrize('row', MOTION_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_allow_motion_true_on_motion_rows(self, row, settings):
        """The three motion rows carry the literal true the guard accepts."""
        assert 'allow_motion:=true' in _argv(row[0], row[1], settings)

    @pytest.mark.parametrize('row', SIMULATE_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_no_address_token_in_any_fake_row(self, row, settings):
        """A Simulate session never receives an address, not even a set one."""
        argv = _argv(row[0], row[1], settings)
        joined = ' '.join(argv)
        for address in DOC_ADDRESSES:
            assert address not in joined
        assert not any(token.startswith('robot_ip') for token in argv)

    @pytest.mark.parametrize('row', ALL_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_argv_is_pure_and_repeatable(self, row, settings):
        """Two identical calls produce identical argv; nothing is consumed."""
        assert _argv(row[0], row[1], settings) == _argv(row[0], row[1], settings)

    def test_a_single_arm_session_takes_its_address_from_that_arms_config_key(
            self, settings):
        """
        A panda2 session emits panda2's address, never panda1's.

        This is the live cross-robot mislabel the shared single-address key
        caused: every arm is bound to its own `robots.<arm>.ip`.
        """
        for arms, mode, expected in (('panda1', 'watch', DOC_IP_1),
                                     ('panda2', 'watch', DOC_IP_2),
                                     ('panda1', 'motion', DOC_IP_1),
                                     ('panda2', 'motion', DOC_IP_2)):
            argv = _argv(arms, mode, settings)
            assert 'robot_ip:=' + expected in argv
            assert not any(token.startswith('robot_ip_') for token in argv)

    def test_no_single_address_key_exists_any_more(self):
        """The address table binds each launch argument to an arm, not a key."""
        assert profiles._ADDRESS_SOURCES == {
            'single': (('robot_ip', None),),
            'dual': (('robot_ip_1', 'panda1'), ('robot_ip_2', 'panda2')),
        }

    def test_dual_rows_use_both_addresses_in_order(self, settings):
        """Dual rows emit robot_ip_1 then robot_ip_2, never robot_ip."""
        for mode in ('watch', 'motion'):
            argv = _argv('both', mode, settings)
            assert argv.index('robot_ip_1:=' + DOC_IP_1) < argv.index('robot_ip_2:=' + DOC_IP_2)
            assert not any(token.startswith('robot_ip:=') for token in argv)

    def test_motion_always_names_the_impedance_controller(self, settings):
        """The controller is no longer a choice; it is the reviewed one."""
        for arms in ('panda1', 'panda2', 'both'):
            argv = _argv(arms, 'motion', settings)
            assert 'controller_name:=' + defaults.MOTION_CONTROLLER in argv


class TestUnknownCombinations:
    """Anything outside the nine rows is refused, never guessed at."""

    @pytest.mark.parametrize('arms, mode', [
        ('panda3', 'simulate'),
        ('panda1', 'teleop'),
        ('both', 'MOTION'),
        ('Both', 'watch'),
        ('', ''),
        (None, 'simulate'),
        ('panda1', None),
        ('panda1', 'simulate '),
    ])
    def test_unknown_combination_raises(self, arms, mode, settings):
        """An unknown arms or mode value has no profile."""
        with pytest.raises(ProfileError):
            argv_for(arms, mode, settings)

    @pytest.mark.parametrize('arms', [['panda1'], {'arms': 'both'}, {'panda1'}])
    def test_unhashable_arms_is_a_profile_error_not_a_type_error(self, arms, settings):
        """A JSON list or object in the arms field is refused cleanly."""
        with pytest.raises(ProfileError):
            argv_for(arms, 'simulate', settings)

    def test_profile_error_is_a_value_error(self):
        """Callers may catch ValueError; ProfileError stays a subclass."""
        assert issubclass(ProfileError, ValueError)

    def test_refusal_lists_the_valid_combinations(self, settings):
        """The message helps without echoing the rejected request."""
        with pytest.raises(ProfileError) as excinfo:
            argv_for('panda9', 'simulate', settings)
        message = str(excinfo.value)
        assert 'panda9' not in message
        assert 'both/simulate' in message


class TestMissingAddresses:
    """
    A missing address is a defensive path only.

    Every arm's address now DEFAULTS, so `robot_addresses_missing` cannot be
    reached from a valid configuration. The refusal survives as a defensive
    code and names the arm and the key to set.
    """

    class _Addressless:
        """Settings whose robot_ip() answers nothing, to reach the refusal."""

        def robot_ip(self, arm_id):
            """Answer as an unconfigured address would."""
            return None

    @pytest.mark.parametrize('row', WATCH_ROWS + MOTION_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_production_row_without_addresses_raises(self, row):
        """Watch and motion cannot be launched without an address."""
        with pytest.raises(ProfileError):
            _argv(row[0], row[1], self._Addressless())

    @pytest.mark.parametrize('row', WATCH_ROWS + MOTION_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_the_refusal_names_the_arm_and_the_config_key(self, row):
        """The operator is told exactly which key to set, for which arm."""
        with pytest.raises(ProfileError) as excinfo:
            _argv(row[0], row[1], self._Addressless())
        message = str(excinfo.value)
        for arm_id in PROFILES[row].arm_ids:
            assert arm_id in message
            assert 'robots.{}.ip'.format(arm_id) in message
        assert 'FRANKA_WEB' not in message

    @pytest.mark.parametrize('row', SIMULATE_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_simulate_rows_need_no_addresses(self, row):
        """Simulate must work on a machine that has no robot configured."""
        argv = _argv(row[0], row[1], self._Addressless())
        assert argv == EXPECTED_ARGV[row]


class TestControllerArguments:
    """The parameter file belongs to motion rows, and to nothing else."""

    @pytest.mark.parametrize('row', MOTION_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_a_motion_profile_without_a_param_file_is_refused(self, row, settings):
        """A motion launch with no parameter file would fail the guard."""
        with pytest.raises(ProfileError) as excinfo:
            argv_for(row[0], row[1], settings)
        assert 'controller_param_file' in str(excinfo.value)

    @pytest.mark.parametrize('blank', ['', '   ', '\t'])
    @pytest.mark.parametrize('row', MOTION_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_blank_controller_argument_counts_as_missing(self, row, blank, settings):
        """A blank value is an unset value, not an empty launch argument."""
        with pytest.raises(ProfileError):
            argv_for(row[0], row[1], settings, controller_param_file=blank)

    @pytest.mark.parametrize('row', SIMULATE_ROWS + WATCH_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_a_param_file_on_a_non_motion_profile_is_refused(self, row, settings):
        """A state-only launch declares no controller_param_file to receive."""
        with pytest.raises(ProfileError) as excinfo:
            argv_for(row[0], row[1], settings, controller_param_file=PARAM_FILE)
        assert 'motion profile' in str(excinfo.value)

    def test_controller_arguments_are_stripped_before_emission(self, settings):
        """Surrounding whitespace never becomes part of a launch argument."""
        argv = argv_for('both', 'motion', settings,
                        controller_param_file=PARAM_FILE + '\n')
        assert 'controller_param_file:=' + PARAM_FILE in argv


class TestModuleIsPure:
    """profiles.py must stay free of process, environment and filesystem work."""

    def test_no_process_or_environment_module_is_imported(self):
        """An impure helper here would undo the never-execute guarantee."""
        for forbidden in ('os', 'subprocess', 'shutil', 'socket', 'rclpy'):
            assert not hasattr(profiles, forbidden)
