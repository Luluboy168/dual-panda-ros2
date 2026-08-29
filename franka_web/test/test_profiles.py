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
(203.0.113.0/24) -- never a real robot address, per the session rules.
"""

import os

from franka_web import profiles
from franka_web.config import Settings
from franka_web.profiles import argv_for, Profile, ProfileError, PROFILES
import pytest

DOC_IP_SINGLE = '203.0.113.9'
DOC_IP_1 = '203.0.113.7'
DOC_IP_2 = '203.0.113.8'
DOC_ADDRESSES = (DOC_IP_SINGLE, DOC_IP_1, DOC_IP_2)

CONTROLLER = 'dual_arm_joint_impedance_controller'
PARAM_FILE = '/var/lib/franka_web/gains/a1b2c3.yaml'

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
        'robot_ip:=' + DOC_IP_SINGLE,
        'use_rviz:=false'),
    ('panda2', 'watch'): PREFIX + (
        'production_single_state_only.launch.py',
        'arm_id:=panda2',
        'robot_ip:=' + DOC_IP_SINGLE,
        'use_rviz:=false'),
    ('both', 'watch'): PREFIX + (
        'production_dual_state_only.launch.py',
        'robot_ip_1:=' + DOC_IP_1,
        'robot_ip_2:=' + DOC_IP_2,
        'use_rviz:=false'),
    ('panda1', 'motion'): PREFIX + (
        'production_single_guarded_motion.launch.py',
        'arm_id:=panda1',
        'robot_ip:=' + DOC_IP_SINGLE,
        'allow_motion:=true',
        'controller_name:=' + CONTROLLER,
        'controller_param_file:=' + PARAM_FILE,
        'use_rviz:=false'),
    ('panda2', 'motion'): PREFIX + (
        'production_single_guarded_motion.launch.py',
        'arm_id:=panda2',
        'robot_ip:=' + DOC_IP_SINGLE,
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


def _settings(tmp_path, **addresses):
    """Build a validated Settings over tmp dirs, with the given addresses."""
    state_dir = tmp_path / 'state'
    state_dir.mkdir(mode=0o700, exist_ok=True)
    recording_root = tmp_path / 'recordings'
    recording_root.mkdir(mode=0o700, exist_ok=True)
    os.chmod(tmp_path, 0o700)
    environment = {
        'FRANKA_WEB_STATE_DIR': str(state_dir),
        'FRANKA_WEB_RECORDING_ROOT': str(recording_root),
        'ROS_DOMAIN_ID': '80',
    }
    environment.update(addresses)
    return Settings.from_env(environment)


@pytest.fixture()
def settings(tmp_path):
    """Return Settings carrying all three documentation addresses."""
    return _settings(
        tmp_path,
        FRANKA_WEB_ROBOT_IP=DOC_IP_SINGLE,
        FRANKA_WEB_ROBOT_IP_1=DOC_IP_1,
        FRANKA_WEB_ROBOT_IP_2=DOC_IP_2,
    )


@pytest.fixture()
def addressless(tmp_path):
    """Return Settings with no robot address set at all."""
    return _settings(tmp_path)


def _argv(arms, mode, settings):
    """Call argv_for, supplying controller arguments only on motion rows."""
    if PROFILES[(arms, mode)].allows_motion:
        return argv_for(arms, mode, settings, CONTROLLER, PARAM_FILE)
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

    def test_single_rows_use_the_single_address(self, settings):
        """Single rows read robot_ip_single, never the dual pair."""
        for arms, mode in (('panda1', 'watch'), ('panda2', 'motion')):
            argv = _argv(arms, mode, settings)
            assert 'robot_ip:=' + DOC_IP_SINGLE in argv
            assert not any(token.startswith('robot_ip_') for token in argv)

    def test_dual_rows_use_both_addresses_in_order(self, settings):
        """Dual rows emit robot_ip_1 then robot_ip_2, never robot_ip."""
        for mode in ('watch', 'motion'):
            argv = _argv('both', mode, settings)
            assert argv.index('robot_ip_1:=' + DOC_IP_1) < argv.index('robot_ip_2:=' + DOC_IP_2)
            assert 'robot_ip:=' + DOC_IP_SINGLE not in argv


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
    """A production row without its addresses refuses, and says which are unset."""

    @pytest.mark.parametrize('row', WATCH_ROWS + MOTION_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_production_row_without_addresses_raises(self, row, addressless):
        """Watch and motion cannot be launched from an addressless server."""
        with pytest.raises(ProfileError):
            _argv(row[0], row[1], addressless)

    @pytest.mark.parametrize('row', WATCH_ROWS + MOTION_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_refusal_names_the_environment_variables(self, row, addressless):
        """The operator is told exactly which variable to set."""
        with pytest.raises(ProfileError) as excinfo:
            _argv(row[0], row[1], addressless)
        message = str(excinfo.value)
        if PROFILES[row].arm_mode == 'single':
            assert 'FRANKA_WEB_ROBOT_IP' in message
        else:
            assert 'FRANKA_WEB_ROBOT_IP_1' in message
            assert 'FRANKA_WEB_ROBOT_IP_2' in message

    @pytest.mark.parametrize('row', WATCH_ROWS + MOTION_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_refusal_contains_no_address(self, row, addressless):
        """Nothing address-shaped may reach a log line."""
        with pytest.raises(ProfileError) as excinfo:
            _argv(row[0], row[1], addressless)
        message = str(excinfo.value)
        assert '203.0.113' not in message
        for address in DOC_ADDRESSES:
            assert address not in message

    def test_partial_dual_addresses_never_leak_the_set_one(self, tmp_path):
        """With only robot_ip_1 set, the refusal names 2 and echoes neither."""
        partial = _settings(tmp_path, FRANKA_WEB_ROBOT_IP_1=DOC_IP_1)
        with pytest.raises(ProfileError) as excinfo:
            argv_for('both', 'watch', partial)
        message = str(excinfo.value)
        assert 'FRANKA_WEB_ROBOT_IP_2' in message
        assert DOC_IP_1 not in message

    def test_single_row_ignores_the_dual_addresses(self, tmp_path):
        """The dual pair does not satisfy a single row's address need."""
        dual_only = _settings(
            tmp_path, FRANKA_WEB_ROBOT_IP_1=DOC_IP_1, FRANKA_WEB_ROBOT_IP_2=DOC_IP_2)
        with pytest.raises(ProfileError) as excinfo:
            argv_for('panda1', 'watch', dual_only)
        assert 'FRANKA_WEB_ROBOT_IP' in str(excinfo.value)

    def test_dual_row_ignores_the_single_address(self, tmp_path):
        """The single address does not satisfy a dual row's address need."""
        single_only = _settings(tmp_path, FRANKA_WEB_ROBOT_IP=DOC_IP_SINGLE)
        with pytest.raises(ProfileError) as excinfo:
            argv_for('both', 'watch', single_only)
        message = str(excinfo.value)
        assert 'FRANKA_WEB_ROBOT_IP_1' in message
        assert 'FRANKA_WEB_ROBOT_IP_2' in message

    @pytest.mark.parametrize('row', SIMULATE_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_simulate_rows_need_no_addresses(self, row, addressless):
        """Simulate must work on a machine that has no robot configured."""
        assert _argv(row[0], row[1], addressless) == EXPECTED_ARGV[row]


class TestControllerArguments:
    """The controller pair belongs to motion rows, and to nothing else."""

    @pytest.mark.parametrize('row', MOTION_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_motion_without_controller_name_raises(self, row, settings):
        """A motion launch with no controller name would fail the guard."""
        with pytest.raises(ProfileError) as excinfo:
            argv_for(row[0], row[1], settings, None, PARAM_FILE)
        assert 'controller_name' in str(excinfo.value)

    @pytest.mark.parametrize('row', MOTION_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_motion_without_param_file_raises(self, row, settings):
        """A motion launch with no parameter file would fail the guard."""
        with pytest.raises(ProfileError) as excinfo:
            argv_for(row[0], row[1], settings, CONTROLLER, None)
        assert 'controller_param_file' in str(excinfo.value)

    @pytest.mark.parametrize('row', MOTION_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_motion_with_no_controller_arguments_at_all_raises(self, row, settings):
        """The default arguments are not a usable motion request."""
        with pytest.raises(ProfileError):
            argv_for(row[0], row[1], settings)

    @pytest.mark.parametrize('blank', ['', '   ', '\t'])
    @pytest.mark.parametrize('row', MOTION_ROWS, ids=lambda row: '{}-{}'.format(*row))
    def test_blank_controller_argument_counts_as_missing(self, row, blank, settings):
        """A blank picker is an unset picker, not an empty launch argument."""
        with pytest.raises(ProfileError):
            argv_for(row[0], row[1], settings, blank, PARAM_FILE)
        with pytest.raises(ProfileError):
            argv_for(row[0], row[1], settings, CONTROLLER, blank)

    @pytest.mark.parametrize('row', SIMULATE_ROWS + WATCH_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_controller_name_rejected_on_non_motion_rows(self, row, settings):
        """A state-only launch declares no controller_name to receive."""
        with pytest.raises(ProfileError) as excinfo:
            argv_for(row[0], row[1], settings, CONTROLLER)
        assert 'motion profile' in str(excinfo.value)

    @pytest.mark.parametrize('row', SIMULATE_ROWS + WATCH_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_controller_param_file_rejected_on_non_motion_rows(self, row, settings):
        """A state-only launch declares no controller_param_file either."""
        with pytest.raises(ProfileError):
            argv_for(row[0], row[1], settings, None, PARAM_FILE)

    @pytest.mark.parametrize('row', SIMULATE_ROWS + WATCH_ROWS,
                             ids=lambda row: '{}-{}'.format(*row))
    def test_both_controller_arguments_rejected_on_non_motion_rows(self, row, settings):
        """Passing the full motion pair to a state-only row is still refused."""
        with pytest.raises(ProfileError):
            argv_for(row[0], row[1], settings, CONTROLLER, PARAM_FILE)

    def test_controller_arguments_are_stripped_before_emission(self, settings):
        """Surrounding whitespace never becomes part of a launch argument."""
        argv = argv_for('both', 'motion', settings, '  ' + CONTROLLER + ' ', PARAM_FILE + '\n')
        assert 'controller_name:=' + CONTROLLER in argv
        assert 'controller_param_file:=' + PARAM_FILE in argv


class TestModuleIsPure:
    """profiles.py must stay free of process, environment and filesystem work."""

    def test_no_process_or_environment_module_is_imported(self):
        """An impure helper here would undo the never-execute guarantee."""
        for forbidden in ('os', 'subprocess', 'shutil', 'socket', 'rclpy'):
            assert not hasattr(profiles, forbidden)
