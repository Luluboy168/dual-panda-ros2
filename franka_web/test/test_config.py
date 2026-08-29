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
Tests for franka_web.config: constants and Settings.from_env validation.

Robot addresses in these fixtures are RFC 5737 documentation addresses
(203.0.113.0/24) -- never a real robot address, per the session rules.
"""

import os
from pathlib import Path
import re

from franka_web import config
from franka_web.config import ConfigError, Settings
import pytest

DOC_IP_1 = '203.0.113.7'
DOC_IP_2 = '203.0.113.8'


@pytest.fixture()
def env(tmp_path):
    """Return a fully valid environment dict backed by private tmp dirs."""
    state_dir = tmp_path / 'state'
    state_dir.mkdir(mode=0o700)
    recording_root = tmp_path / 'recordings'
    recording_root.mkdir(mode=0o700)
    os.chmod(tmp_path, 0o700)
    return {
        'FRANKA_WEB_STATE_DIR': str(state_dir),
        'FRANKA_WEB_RECORDING_ROOT': str(recording_root),
        'ROS_DOMAIN_ID': '80',
    }


def _expect_refusal(env, fragment):
    """Assert from_env raises ConfigError mentioning ``fragment``."""
    with pytest.raises(ConfigError) as excinfo:
        Settings.from_env(env)
    assert fragment in str(excinfo.value)
    return excinfo.value


class TestFrozenConstants:
    """The safety-relevant constants are pinned to their reviewed sources."""

    def test_jog_contract_numbers(self):
        """Jog numbers match the impedance controller's reviewed contract."""
        assert config.JOG_STEP_RAD == pytest.approx(0.034906585)
        assert config.JOG_STREAM_HZ == 20.0
        assert config.WATCHDOG_TIMEOUT_S == 0.1
        assert config.MAX_HEADER_AGE_S == 1.0
        assert 1.0 / config.JOG_STREAM_HZ < config.WATCHDOG_TIMEOUT_S

    def test_size_and_rate_caps(self):
        """Caps match the validator, recorder, and Fast-DDS hard limits."""
        assert config.MAX_GAINS_BYTES == 65536
        assert config.RECORDING_SEGMENT_DURATION_S == 3600
        assert config.ROS_DOMAIN_ID_MAXIMUM == 232
        assert config.SSE_QUEUE_DEPTH == 4
        assert config.STATE_FRAME_HZ == 5.0

    def test_ccsr_gate_is_the_signed_gate(self):
        """F4 uses the user-signed Phase 10/11 gate: 0.95 sustained 5 s."""
        assert config.CCSR_FAULT_THRESHOLD == 0.95
        assert config.CCSR_FAULT_SUSTAIN_S == 5.0

    def test_bind_allowlist_is_loopback_only(self):
        """No routable address can ever be an allowed bind."""
        assert config.ALLOWED_BIND == ('127.0.0.1', '::1')
        assert config.DEFAULT_BIND in config.ALLOWED_BIND


class TestBind:
    """FRANKA_WEB_BIND is restricted to the loopback allowlist."""

    def test_default_bind(self, env):
        """Unset bind falls back to 127.0.0.1."""
        assert Settings.from_env(env).bind == '127.0.0.1'

    def test_blank_bind_uses_default(self, env):
        """Blank bind is treated as unset."""
        env['FRANKA_WEB_BIND'] = '   '
        assert Settings.from_env(env).bind == '127.0.0.1'

    def test_ipv6_loopback_allowed(self, env):
        """::1 is the one non-default allowed bind."""
        env['FRANKA_WEB_BIND'] = '::1'
        assert Settings.from_env(env).bind == '::1'

    @pytest.mark.parametrize('bind', ['0.0.0.0', '192.168.1.5', 'localhost', '::', '127.0.0.2'])
    def test_routable_or_alias_binds_refused(self, env, bind):
        """Anything but the two literal loopback addresses is refused."""
        env['FRANKA_WEB_BIND'] = bind
        _expect_refusal(env, 'FRANKA_WEB_BIND')


class TestPort:
    """FRANKA_WEB_PORT is an unprivileged TCP port."""

    def test_default_port(self, env):
        """Unset port falls back to 8781."""
        assert Settings.from_env(env).port == 8781

    @pytest.mark.parametrize('port', ['1024', '8781', '65535'])
    def test_valid_ports(self, env, port):
        """In-range ports parse."""
        env['FRANKA_WEB_PORT'] = port
        assert Settings.from_env(env).port == int(port)

    @pytest.mark.parametrize('port', ['0', '80', '1023', '65536', '-1', 'http', '8781.0', ''])
    def test_invalid_ports_refused(self, env, port):
        """Privileged, out-of-range, and non-integer ports are refused."""
        env['FRANKA_WEB_PORT'] = port
        if port.strip() == '':
            assert Settings.from_env(env).port == 8781
        else:
            _expect_refusal(env, 'FRANKA_WEB_PORT')

    @pytest.mark.parametrize('port', ['8_781', '+8781', '٨٧٨١'])
    def test_non_plain_decimal_ports_refused(self, env, port):
        """Underscores, signs, and non-ASCII digits are refused even where int() accepts them."""
        env['FRANKA_WEB_PORT'] = port
        _expect_refusal(env, 'FRANKA_WEB_PORT')


class TestDomainId:
    """ROS_DOMAIN_ID is required and bounded by the Fast-DDS ceiling."""

    def test_missing_refused(self, env):
        """The server never invents a domain id."""
        del env['ROS_DOMAIN_ID']
        _expect_refusal(env, 'ROS_DOMAIN_ID')

    def test_blank_refused(self, env):
        """Blank is the same as missing."""
        env['ROS_DOMAIN_ID'] = ''
        _expect_refusal(env, 'ROS_DOMAIN_ID')

    @pytest.mark.parametrize('value', ['0', '80', '232'])
    def test_valid_range(self, env, value):
        """0..232 parses."""
        env['ROS_DOMAIN_ID'] = value
        assert Settings.from_env(env).ros_domain_id == int(value)

    @pytest.mark.parametrize('value', ['233', '-1', 'eighty', '80.0'])
    def test_out_of_range_refused(self, env, value):
        """Above the Fast-DDS ceiling, negative, or non-integer is refused."""
        env['ROS_DOMAIN_ID'] = value
        _expect_refusal(env, 'ROS_DOMAIN_ID')

    @pytest.mark.parametrize('value', ['8_0', '+80', '٨٠'])
    def test_non_plain_decimal_refused(self, env, value):
        """
        Values int() accepts but rcl's strtoul parses differently are refused.

        '8_0' would land every child on domain 8 while the server reports 80;
        Arabic-Indic digits pass int() but rcl rejects them after boot.
        """
        env['ROS_DOMAIN_ID'] = value
        _expect_refusal(env, 'ROS_DOMAIN_ID')


class TestRecordingRoot:
    """FRANKA_WEB_RECORDING_ROOT mirrors the recorder's own checks."""

    def test_missing_refused(self, env):
        """The recording root is required."""
        del env['FRANKA_WEB_RECORDING_ROOT']
        _expect_refusal(env, 'FRANKA_WEB_RECORDING_ROOT')

    def test_nonexistent_refused(self, env, tmp_path):
        """A missing directory is refused at boot, not at first session."""
        env['FRANKA_WEB_RECORDING_ROOT'] = str(tmp_path / 'nope')
        _expect_refusal(env, 'does not exist')

    def test_relative_path_refused(self, env):
        """The recorder requires a normalized absolute path; so do we."""
        env['FRANKA_WEB_RECORDING_ROOT'] = 'recordings'
        _expect_refusal(env, 'normalized absolute')

    def test_unnormalized_path_refused(self, env):
        """Dot segments are refused even when they resolve to a valid dir."""
        root = env['FRANKA_WEB_RECORDING_ROOT']
        env['FRANKA_WEB_RECORDING_ROOT'] = root + '/../' + os.path.basename(root)
        _expect_refusal(env, 'normalized absolute')

    def test_file_refused(self, env, tmp_path):
        """A regular file is not a recording root."""
        target = tmp_path / 'afile'
        target.write_text('x')
        env['FRANKA_WEB_RECORDING_ROOT'] = str(target)
        _expect_refusal(env, 'FRANKA_WEB_RECORDING_ROOT')

    @pytest.mark.parametrize('mode', [0o750, 0o770, 0o707, 0o701])
    def test_group_or_other_bits_refused(self, env, tmp_path, mode):
        """Any group/other permission bit is refused (recorder rule)."""
        loose = tmp_path / 'loose'
        loose.mkdir(mode=0o700)
        os.chmod(loose, mode)
        env['FRANKA_WEB_RECORDING_ROOT'] = str(loose)
        _expect_refusal(env, 'permission bits')

    def test_symlink_component_refused(self, env, tmp_path):
        """A symlink anywhere in the path is refused (O_NOFOLLOW walk)."""
        real = tmp_path / 'real'
        real.mkdir(mode=0o700)
        link = tmp_path / 'link'
        link.symlink_to(real)
        env['FRANKA_WEB_RECORDING_ROOT'] = str(link)
        _expect_refusal(env, 'symlink')

    def test_foreign_owner_refused(self, env):
        """
        The recording root's own ownership check refuses a foreign uid.

        Exercised directly: through from_env the state-dir check (validated
        first, same injected geteuid) would shadow this one.
        """
        with pytest.raises(ConfigError) as excinfo:
            config._validate_owned_directory(
                'FRANKA_WEB_RECORDING_ROOT', env['FRANKA_WEB_RECORDING_ROOT'],
                lambda: os.geteuid() + 1, require_private=True)
        message = str(excinfo.value)
        assert 'FRANKA_WEB_RECORDING_ROOT' in message
        assert 'owned by the user' in message

    @pytest.mark.parametrize('mode', [0o500, 0o600])
    def test_owner_bits_missing_refused(self, env, tmp_path, mode):
        """
        A root the recorder could not mkdir into is refused at boot.

        0500/0600 pass the recorder's no-group/other check but fail its
        session mkdir; the boot gate must catch them early (review F2).
        """
        cramped = tmp_path / 'cramped'
        cramped.mkdir(mode=0o700)
        os.chmod(cramped, mode)
        env['FRANKA_WEB_RECORDING_ROOT'] = str(cramped)
        _expect_refusal(env, 'expected mode 0700')

    def test_trailing_slash_accepted_and_normalized(self, env):
        """A trailing slash (shell tab-completion) is stripped, not refused."""
        root = env['FRANKA_WEB_RECORDING_ROOT']
        env['FRANKA_WEB_RECORDING_ROOT'] = root + '/'
        assert Settings.from_env(env).recording_root == root

    def test_embedded_null_refused(self, env, tmp_path):
        """An embedded NUL is a clean ConfigError, not a ValueError traceback."""
        env['FRANKA_WEB_RECORDING_ROOT'] = str(tmp_path / 'ba\x00d')
        _expect_refusal(env, 'invalid character')

    def test_unreadable_ancestor_reports_permission(self, env, tmp_path):
        """EACCES on a path component names permission, not 'not a directory'."""
        outer = tmp_path / 'outer'
        outer.mkdir(mode=0o700)
        inner = outer / 'inner'
        inner.mkdir(mode=0o700)
        os.chmod(outer, 0o000)
        try:
            env['FRANKA_WEB_RECORDING_ROOT'] = str(inner)
            _expect_refusal(env, 'permission denied')
        finally:
            os.chmod(outer, 0o700)


class TestStateDir:
    """FRANKA_WEB_STATE_DIR may not exist yet, but must be safely placeable."""

    def test_missing_refused(self, env):
        """The state dir is required."""
        del env['FRANKA_WEB_STATE_DIR']
        _expect_refusal(env, 'FRANKA_WEB_STATE_DIR')

    def test_relative_refused(self, env):
        """Relative state dirs are refused."""
        env['FRANKA_WEB_STATE_DIR'] = 'state'
        _expect_refusal(env, 'normalized absolute')

    def test_existing_private_dir_accepted(self, env):
        """An existing 0700 directory owned by us passes."""
        assert Settings.from_env(env).state_dir == env['FRANKA_WEB_STATE_DIR']

    def test_existing_loose_dir_refused(self, env):
        """An existing state dir with group/other bits is refused."""
        os.chmod(env['FRANKA_WEB_STATE_DIR'], 0o750)
        _expect_refusal(env, 'permission bits')

    def test_nonexistent_with_valid_parent_accepted(self, env, tmp_path):
        """A not-yet-created dir under a parent we own passes (boot creates it)."""
        env['FRANKA_WEB_STATE_DIR'] = str(tmp_path / 'newstate')
        settings = Settings.from_env(env)
        assert settings.state_dir == str(tmp_path / 'newstate')
        assert not os.path.exists(settings.state_dir)

    def test_nonexistent_with_missing_parent_refused(self, env, tmp_path):
        """A missing parent is refused."""
        env['FRANKA_WEB_STATE_DIR'] = str(tmp_path / 'no' / 'state')
        _expect_refusal(env, 'parent')

    def test_symlink_component_refused(self, env, tmp_path):
        """A symlinked state dir is refused."""
        real = tmp_path / 'realstate'
        real.mkdir(mode=0o700)
        link = tmp_path / 'statelink'
        link.symlink_to(real)
        env['FRANKA_WEB_STATE_DIR'] = str(link)
        _expect_refusal(env, 'symlink')

    def test_foreign_owner_refused(self, env):
        """A state dir owned by another uid is refused (first check to fire)."""
        with pytest.raises(ConfigError) as excinfo:
            Settings.from_env(env, geteuid=lambda: os.geteuid() + 1)
        message = str(excinfo.value)
        assert 'FRANKA_WEB_STATE_DIR' in message
        assert 'owned by the user' in message

    def test_normal_home_style_parent_accepted(self, env, tmp_path):
        """A 0755 parent (a normal home dir) is fine for a to-be-created state dir."""
        parent = tmp_path / 'homeish'
        parent.mkdir(mode=0o700)
        os.chmod(parent, 0o755)
        env['FRANKA_WEB_STATE_DIR'] = str(parent / 'state')
        assert Settings.from_env(env).state_dir == str(parent / 'state')

    @pytest.mark.parametrize('mode', [0o777, 0o775, 0o720])
    def test_shared_writable_parent_refused(self, env, tmp_path, mode):
        """A group/other-writable parent could pre-create the state dir; refused."""
        parent = tmp_path / 'shared'
        parent.mkdir(mode=0o700)
        os.chmod(parent, mode)
        env['FRANKA_WEB_STATE_DIR'] = str(parent / 'state')
        _expect_refusal(env, 'writable by group or others')

    def test_embedded_null_refused(self, env, tmp_path):
        """A NUL in the (not-yet-existing) leaf is refused, never stored."""
        env['FRANKA_WEB_STATE_DIR'] = str(tmp_path / 'sta\x00te')
        _expect_refusal(env, 'invalid character')


class TestFrankaDir:
    """FRANKA_WEB_FRANKA_DIR is optional but validated when present."""

    def test_unset_is_none(self, env):
        """Absent means None; the preflight path enforces it later."""
        assert Settings.from_env(env).franka_dir is None

    def test_existing_dir_accepted(self, env, tmp_path):
        """An existing directory passes."""
        env['FRANKA_WEB_FRANKA_DIR'] = str(tmp_path)
        assert Settings.from_env(env).franka_dir == str(tmp_path)

    def test_nonexistent_refused(self, env, tmp_path):
        """A missing directory is refused."""
        env['FRANKA_WEB_FRANKA_DIR'] = str(tmp_path / 'nope')
        _expect_refusal(env, 'FRANKA_WEB_FRANKA_DIR')

    def test_relative_refused(self, env):
        """A relative directory is refused."""
        env['FRANKA_WEB_FRANKA_DIR'] = 'build'
        _expect_refusal(env, 'FRANKA_WEB_FRANKA_DIR')

    def test_symlink_refused(self, env, tmp_path):
        """The preflight's build tree may not be reached through a symlink."""
        real = tmp_path / 'realbuild'
        real.mkdir(mode=0o700)
        link = tmp_path / 'buildlink'
        link.symlink_to(real)
        env['FRANKA_WEB_FRANKA_DIR'] = str(link)
        _expect_refusal(env, 'symlink')

    def test_build_tree_mode_0755_accepted(self, env, tmp_path):
        """An ordinary 0755 build tree passes (no privacy requirement)."""
        build = tmp_path / 'build755'
        build.mkdir(mode=0o700)
        os.chmod(build, 0o755)
        env['FRANKA_WEB_FRANKA_DIR'] = str(build)
        assert Settings.from_env(env).franka_dir == str(build)

    def test_shared_writable_refused(self, env, tmp_path):
        """A group/other-writable build tree is refused."""
        build = tmp_path / 'build777'
        build.mkdir(mode=0o700)
        os.chmod(build, 0o777)
        env['FRANKA_WEB_FRANKA_DIR'] = str(build)
        _expect_refusal(env, 'writable by group or others')


class TestVersionBinding:
    """config.SERVER_VERSION must track package.xml."""

    def test_server_version_matches_package_xml(self):
        """The version frozen into the capabilities contract equals package.xml's."""
        package_xml = (Path(__file__).resolve().parents[1] / 'package.xml').read_text()
        declared = re.search(r'<version>([^<]+)</version>', package_xml).group(1)
        assert config.SERVER_VERSION == declared


class TestRobotAddressHandling:
    """Robot addresses are optional, kept, and never exposed."""

    def test_unset_addresses_are_none(self, env):
        """No address env means no address."""
        settings = Settings.from_env(env)
        assert settings.robot_ip_1 is None
        assert settings.robot_ip_2 is None
        assert settings.robot_ip_single is None

    def test_addresses_are_kept_verbatim(self, env):
        """Set addresses are stored for launch-argument use."""
        env['FRANKA_WEB_ROBOT_IP_1'] = DOC_IP_1
        env['FRANKA_WEB_ROBOT_IP_2'] = DOC_IP_2
        env['FRANKA_WEB_ROBOT_IP'] = DOC_IP_1
        settings = Settings.from_env(env)
        assert settings.robot_ip_1 == DOC_IP_1
        assert settings.robot_ip_2 == DOC_IP_2
        assert settings.robot_ip_single == DOC_IP_1

    def test_repr_never_contains_an_address(self, env):
        """repr/str of Settings must be safe to log."""
        env['FRANKA_WEB_ROBOT_IP_1'] = DOC_IP_1
        env['FRANKA_WEB_ROBOT_IP_2'] = DOC_IP_2
        env['FRANKA_WEB_ROBOT_IP'] = DOC_IP_1
        settings = Settings.from_env(env)
        for rendering in (repr(settings), str(settings)):
            assert DOC_IP_1 not in rendering
            assert DOC_IP_2 not in rendering

    def test_all_address_fields_are_repr_suppressed(self):
        """Every address-carrying field is marked repr=False."""
        assert set(config._redacted_field_names()) == {
            'robot_ip_1', 'robot_ip_2', 'robot_ip_single'}

    @pytest.mark.parametrize('bad', [
        '203.0.113.7 use_rviz:=true',   # would smuggle a second launch token
        '-allow-motion',                 # leading dash
        'a:=b',                          # launch assignment characters
        'host name',                     # whitespace
    ])
    def test_implausible_addresses_refused_without_echo(self, env, bad):
        """Malformed addresses are refused and the value never appears in the error."""
        env['FRANKA_WEB_ROBOT_IP_1'] = bad
        with pytest.raises(ConfigError) as excinfo:
            Settings.from_env(env)
        message = str(excinfo.value)
        assert 'FRANKA_WEB_ROBOT_IP_1' in message
        assert bad not in message

    def test_hostname_shape_accepted(self, env):
        """A plain hostname is as valid as a dotted quad."""
        env['FRANKA_WEB_ROBOT_IP'] = 'robot-1.lab.internal'
        assert Settings.from_env(env).robot_ip_single == 'robot-1.lab.internal'
