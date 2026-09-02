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

"""The config.yaml surface: defaults, unit conversion and teaching refusals."""

from dataclasses import FrozenInstanceError
import json
import math
import os
from pathlib import Path
import re
import stat

from franka_web import config, defaults
import pytest
import yaml


FIXTURES = Path(__file__).parent / 'support' / 'sample_config'
PACKAGE_ROOT = Path(__file__).parents[1]
EXAMPLE = PACKAGE_ROOT / 'config' / 'config.example.yaml'
SENTINEL = '#| '

# The v1 environment names are ASSEMBLED here, never written as a literal: the
# package-wide scan that proves v2 reads no such variable walks this file too.
PREFIX = 'FRANKA_' + 'WEB_'
V1_NAMES = tuple(PREFIX + suffix for suffix in (
    'BIND', 'PORT', 'STATE_DIR', 'RECORDING_ROOT', 'FRANKA_DIR',
    'ROBOT_IP', 'ROBOT_IP_1', 'ROBOT_IP_2',
    'SETTLING_DRIFT_LIMIT_DEG', 'SETTLING_SPAN_LIMIT_DEG',
    'SETTLING_VELOCITY_LIMIT_DEG_S', 'SETTLING_FENCE_MARGIN_DEG',
    'SETTLING_STABLE_WINDOW_S', 'SETTLING_MIN_SAMPLE_COUNT',
    'SETTLING_TIMEOUT_S'))

INVALID_FIXTURE_KEYS = {
    'invalid_unknown_top_key.yaml': 'nonsense',
    'invalid_unknown_settling_key.yaml': 'settling.stable_windows',
    'invalid_unknown_robot_key.yaml': 'robots.panda1.adress',
    'invalid_unknown_arm.yaml': 'profiles.panda3',
    'invalid_root_sequence.yaml': 'must contain a mapping of settings',
    'invalid_yaml_syntax.yaml': 'could not be parsed as YAML',
    'invalid_port_range.yaml': 'port',
    'invalid_settling_drift_string.yaml': 'settling.drift_limit_deg',
    'invalid_settling_drift_six.yaml': 'settling.drift_limit_deg',
    'invalid_timeout_infeasible.yaml': 'settling.timeout_s',
    'invalid_torque_over_ceiling.yaml': 'profiles.panda2.torque_limit_nm[6]',
    'invalid_speed_over_ceiling.yaml': 'profiles.panda1.speed_limit_deg_s[4]',
    'invalid_watchdog_key.yaml': 'profiles.panda1.watchdog_timeout_s',
    'invalid_fence_outside_policy.yaml': 'fence.panda1.lower_deg[3]',
    'invalid_fence_inverted.yaml': 'fence.panda1',
    'invalid_fence_without_enabled.yaml': 'fence.panda1.lower_deg',
    'invalid_fence_below_the_snap.yaml': 'fence.panda1.lower_deg[3]',
}

VALID_FIXTURES = sorted(path.name for path in FIXTURES.glob('valid_*.yaml'))
INVALID_FIXTURES = sorted(path.name for path in FIXTURES.glob('invalid_*.yaml'))

POLICY_LOWER_DEG = tuple(round(math.degrees(bound), 3)
                         for bound in defaults.POLICY_POSITION_LOWER_RAD)
POLICY_UPPER_DEG = tuple(round(math.degrees(bound), 3)
                         for bound in defaults.POLICY_POSITION_UPPER_RAD)


def write(tmp_path, text):
    """Write a config.yaml under tmp_path and return its path."""
    path = tmp_path / 'config.yaml'
    path.write_text(text, encoding='utf-8')
    return str(path)


def env_for(tmp_path, **extra):
    """Return a hermetic environ dict rooted at tmp_path, with no legacy names."""
    # This docstring deliberately names no legacy-variable literal; V1_NAMES is
    # assembled at runtime so no scan of this file can find the old prefix.
    environ = {'HOME': str(tmp_path)}
    environ.update(extra)
    return environ


def load_text(tmp_path, text, **kwargs):
    """Load a literal config body, with make_dirs=False by default."""
    kwargs.setdefault('make_dirs', False)
    kwargs.setdefault('environ', env_for(tmp_path))
    return config.load(path=write(tmp_path, text), **kwargs)


def refusal(tmp_path, text, **kwargs):
    """Assert load() raises ConfigError for this body and return the message."""
    with pytest.raises(config.ConfigError) as caught:
        load_text(tmp_path, text, **kwargs)
    return str(caught.value)


def _dotted_keys(node, prefix=''):
    """Yield every dotted key path of a nested mapping."""
    if not isinstance(node, dict):
        return
    for key, value in node.items():
        dotted = key if not prefix else '{}.{}'.format(prefix, key)
        yield dotted
        for nested in _dotted_keys(value, dotted):
            yield nested


def fence_body(lower=None, upper=None, arm_id='panda1', enabled=True):
    """Return a fence config body for one arm."""
    lines = ['fence:', '  {}:'.format(arm_id), '    enabled: {}'.format(
        'true' if enabled else 'false')]
    if lower is not None:
        lines.append('    lower_deg: {}'.format(list(lower)))
    if upper is not None:
        lines.append('    upper_deg: {}'.format(list(upper)))
    return '\n'.join(lines) + '\n'


class TestDefaultConfigPath:
    """Where the server looks for the file, without touching the disk."""

    def test_xdg_config_home_wins(self, tmp_path):
        """An absolute XDG_CONFIG_HOME decides the location."""
        environ = env_for(tmp_path, XDG_CONFIG_HOME='/xdg')
        assert config.default_config_path(environ) == '/xdg/franka_web/config.yaml'

    def test_relative_xdg_config_home_is_ignored(self, tmp_path):
        """A relative XDG_CONFIG_HOME is ignored, per the XDG basedir spec."""
        environ = env_for(tmp_path, XDG_CONFIG_HOME='relative/dir')
        assert config.default_config_path(environ) == str(
            tmp_path / '.config' / 'franka_web' / 'config.yaml')

    def test_falls_back_to_home_dot_config(self, tmp_path):
        """Without XDG_CONFIG_HOME the path is under HOME/.config."""
        assert config.default_config_path(env_for(tmp_path)) == str(
            tmp_path / '.config' / 'franka_web' / 'config.yaml')

    def test_expands_tilde_when_home_is_absent(self):
        """With no HOME at all the user's real home directory is used."""
        expected = os.path.join(os.path.expanduser('~'), '.config', 'franka_web',
                                'config.yaml')
        assert config.default_config_path({}) == expected

    def test_never_touches_the_filesystem(self, tmp_path, monkeypatch):
        """Resolving the path is pure string work."""
        def explode(*args, **kwargs):
            raise AssertionError('the filesystem was touched')

        monkeypatch.setattr(os.path, 'exists', explode)
        monkeypatch.setattr(os, 'stat', explode)
        config.default_config_path(env_for(tmp_path))


class TestDefaultsOnlyPath:
    """A missing file means pure defaults, silently."""

    def test_missing_file_loads_without_error(self, tmp_path):
        """No file is not an error."""
        assert config.load(path=str(tmp_path / 'absent.yaml'),
                           environ=env_for(tmp_path), make_dirs=False) is not None

    def test_missing_file_is_silent(self, tmp_path, capsys):
        """Nothing is printed to stdout or stderr."""
        config.load(path=str(tmp_path / 'absent.yaml'), environ=env_for(tmp_path),
                    make_dirs=False)
        captured = capsys.readouterr()
        assert captured.out == '' and captured.err == ''

    def test_missing_file_reports_the_path_it_would_have_read(self, tmp_path):
        """config_path is the path consulted, even when it does not exist."""
        path = str(tmp_path / 'absent.yaml')
        settings = config.load(path=path, environ=env_for(tmp_path), make_dirs=False)
        assert settings.config_path == path

    def test_config_present_is_false(self, tmp_path):
        """config_present says the file was missing."""
        settings = config.load(path=str(tmp_path / 'absent.yaml'),
                               environ=env_for(tmp_path), make_dirs=False)
        assert settings.config_present is False

    def test_scalar_defaults_match_the_contract(self, tmp_path):
        """Port, bind, recording and jog step take their documented defaults."""
        settings = load_text(tmp_path, '')
        assert settings.port == 8765
        assert settings.bind == '0.0.0.0'
        assert settings.recording_enabled is True
        assert settings.jog_step_rad == defaults.JOG_STEP_RAD

    def test_robot_ips_are_the_factory_defaults(self, tmp_path):
        """Both arms default to their Franka factory addresses."""
        settings = load_text(tmp_path, '')
        assert dict(settings.robot_ips) == {'panda1': '172.16.0.2',
                                            'panda2': '172.16.0.3'}

    def test_panda1_profile_is_the_standard_profile(self, tmp_path):
        """panda1 gets the standard gains."""
        profile = load_text(tmp_path, '').profile('panda1')
        assert profile.k_gains == defaults.STANDARD_PROFILE['k_gains']
        assert profile.d_gains == defaults.STANDARD_PROFILE['d_gains']

    def test_panda2_profile_is_the_stiff_j2_profile(self, tmp_path):
        """panda2 gets the stiffened joint 2."""
        profile = load_text(tmp_path, '').profile('panda2')
        assert profile.k_gains == defaults.STIFF_J2_PROFILE['k_gains']
        assert profile.d_gains == defaults.STIFF_J2_PROFILE['d_gains']

    def test_torque_ceilings_are_identical_across_the_two_default_profiles(self, tmp_path):
        """Stiffening joint 2 never raised a torque ceiling."""
        settings = load_text(tmp_path, '')
        assert (settings.profile('panda1').max_effort_nm
                == settings.profile('panda2').max_effort_nm)

    def test_default_fence_is_off_and_bounds_are_the_factory_policy(self, tmp_path):
        """Fence off means exactly the factory limits."""
        for arm_id in defaults.ARM_IDS:
            profile = load_text(tmp_path, '').profile(arm_id)
            assert profile.fence_enabled is False
            assert profile.position_lower_rad == defaults.POLICY_POSITION_LOWER_RAD
            assert profile.position_upper_rad == defaults.POLICY_POSITION_UPPER_RAD

    def test_default_settling_is_bit_exact_si(self, tmp_path):
        """A defaulted settling value never makes a degree round-trip."""
        settling = load_text(tmp_path, '').settling
        assert settling.drift_limit_rad == defaults.DEFAULT_SETTLING['drift_limit_rad']
        assert settling.span_limit_rad == defaults.DEFAULT_SETTLING['span_limit_rad']
        assert (settling.velocity_limit_rad_s
                == defaults.DEFAULT_SETTLING['velocity_limit_rad_s'])
        assert settling.fence_margin_rad == defaults.DEFAULT_SETTLING['fence_margin_rad']
        assert settling.stable_window_s == 1.0
        assert settling.min_samples == 6
        assert settling.timeout_s == 5.0

    def test_default_profile_source_is_default(self, tmp_path):
        """An arm nobody configured reports source 'default'."""
        settings = load_text(tmp_path, '')
        for arm_id in defaults.ARM_IDS:
            assert settings.profile(arm_id).public_view()['source'] == 'default'

    def test_recording_can_be_turned_off(self, tmp_path):
        """recording.enabled: false makes the trip from the file to Settings."""
        settings = load_text(tmp_path, 'recording:\n  enabled: false\n')
        assert settings.recording_enabled is False


class TestEnvironmentIsNotAConfigSurface:
    """No environment variable configures anything except the ROS domain."""

    def test_every_v1_franka_web_variable_is_ignored(self, tmp_path):
        """Setting every legacy variable to nonsense changes nothing."""
        path = str(tmp_path / 'absent.yaml')
        plain = config.load(path=path, environ=env_for(tmp_path), make_dirs=False)
        polluted_env = env_for(tmp_path)
        for name in V1_NAMES:
            polluted_env[name] = 'ABSURD-VALUE-/nowhere:99999'
        polluted = config.load(path=path, environ=polluted_env, make_dirs=False)
        assert polluted == plain

    def test_only_three_environment_names_are_read(self, tmp_path):
        """load() reads XDG_CONFIG_HOME, HOME and ROS_DOMAIN_ID and nothing else."""
        class RecordingEnviron(dict):
            """A dict that records which names were looked up."""

            def __init__(self, *args, **kwargs):
                """Start with an empty read log."""
                super().__init__(*args, **kwargs)
                self.read = set()

            def get(self, key, default=None):
                """Record the lookup and delegate."""
                self.read.add(key)
                return super().get(key, default)

            def __getitem__(self, key):
                """Record the lookup and delegate."""
                self.read.add(key)
                return super().__getitem__(key)

        environ = RecordingEnviron(env_for(tmp_path))
        config.load(path=str(tmp_path / 'absent.yaml'), environ=environ,
                    make_dirs=False)
        assert environ.read <= {'XDG_CONFIG_HOME', 'HOME', 'ROS_DOMAIN_ID'}

    def test_load_does_not_mutate_os_environ(self, tmp_path):
        """Expansion uses the given environ and never the process environment."""
        before = dict(os.environ)
        environ = env_for(tmp_path, FIXTURE_ROOT=str(tmp_path / 'var'))
        settings = load_text(
            tmp_path,
            'directories:\n'
            '  state: "$FIXTURE_ROOT/state"\n'
            '  recordings: "~/recordings"\n',
            environ=environ)
        assert dict(os.environ) == before
        assert settings.state_dir == str(tmp_path / 'var' / 'state')
        assert settings.recording_root == str(tmp_path / 'recordings')

    def test_expansion_never_falls_back_to_the_process_environment(
            self, tmp_path, monkeypatch):
        """
        §5.2: `$VAR` resolves in the GIVEN environ, or not at all.

        `_expand` is the one place a name from the file reaches an
        environment lookup, and a fallback to `os.environ` would be
        invisible: the neighbouring tests either supply the name in the
        passed environ or expand no `$VAR` at all. Here the name exists ONLY
        in the process environment, so an unexpanded `$LEAKED_ROOT` -- and
        therefore a non-absolute path refusal -- is the proof.
        """
        monkeypatch.setenv('LEAKED_ROOT', str(tmp_path / 'leak'))
        message = refusal(
            tmp_path,
            'directories:\n  state: "$LEAKED_ROOT/state"\n',
            environ=env_for(tmp_path))
        assert 'directories.state' in message
        assert 'absolute' in message
        assert str(tmp_path / 'leak') not in message

    def test_ros_domain_id_is_taken_from_the_environment_when_the_key_is_null(
            self, tmp_path):
        """A null key falls back to a usable ROS_DOMAIN_ID."""
        settings = load_text(tmp_path, 'ros_domain_id: null\n',
                             environ=env_for(tmp_path, ROS_DOMAIN_ID='42'))
        assert settings.ros_domain_id == 42

    @pytest.mark.parametrize('text', ['', 'eighty', '233', '-1', '8_0', '+80'])
    def test_unusable_ros_domain_id_environment_means_zero(self, tmp_path, text):
        """An unusable environment value means domain 0, never an error."""
        settings = load_text(tmp_path, '', environ=env_for(tmp_path, ROS_DOMAIN_ID=text))
        assert settings.ros_domain_id == 0

    def test_explicit_key_beats_the_environment(self, tmp_path):
        """An explicit ros_domain_id wins over the environment."""
        settings = load_text(tmp_path, 'ros_domain_id: 9\n',
                             environ=env_for(tmp_path, ROS_DOMAIN_ID='42'))
        assert settings.ros_domain_id == 9


class TestFileLifecycle:
    """Reading, parsing and refusing the file itself."""

    def test_empty_file_is_present_and_all_defaults(self, tmp_path):
        """An empty file is present and changes nothing."""
        settings = load_text(tmp_path, '')
        assert settings.config_present is True
        assert settings.port == defaults.DEFAULT_PORT

    def test_comments_only_file_is_all_defaults(self, tmp_path):
        """A comments-only file parses to nothing and changes nothing."""
        settings = load_text(tmp_path, '# just a comment\n# and another\n')
        assert settings.config_present is True
        assert settings.bind == defaults.DEFAULT_BIND

    def test_valid_file_produces_no_output(self, tmp_path, capsys):
        """A valid file is invisible."""
        load_text(tmp_path, 'port: 9000\n')
        captured = capsys.readouterr()
        assert captured.out == '' and captured.err == ''

    @pytest.mark.parametrize('body', ['- a\n- b\n', 'just a string\n', '42\n'])
    def test_root_must_be_a_mapping(self, tmp_path, body):
        """A root that is not a mapping is refused by name."""
        message = refusal(tmp_path, body)
        assert 'must contain a mapping of settings, found ' in message

    def test_yaml_syntax_error_names_line_and_column(self, tmp_path):
        """A parse error names the exact line and column PyYAML reported."""
        with pytest.raises(config.ConfigError) as caught:
            config.load(path=str(FIXTURES / 'invalid_yaml_syntax.yaml'),
                        environ=env_for(tmp_path), make_dirs=False)
        assert ('could not be parsed as YAML at line 4, column 19: mapping values '
                'are not allowed here.') in str(caught.value)

    def test_yaml_error_message_has_no_traceback(self, tmp_path):
        """No stack trace ever reaches the operator."""
        with pytest.raises(config.ConfigError) as caught:
            config.load(path=str(FIXTURES / 'invalid_yaml_syntax.yaml'),
                        environ=env_for(tmp_path), make_dirs=False)
        assert 'Traceback' not in str(caught.value)
        assert caught.value.__cause__ is None

    def test_file_that_is_not_utf8_is_refused(self, tmp_path):
        """A binary file is refused with a plain sentence."""
        path = tmp_path / 'config.yaml'
        path.write_bytes(b'port: \xff\xfe\n')
        with pytest.raises(config.ConfigError) as caught:
            config.load(path=str(path), environ=env_for(tmp_path), make_dirs=False)
        assert str(caught.value) == '{}: is not valid UTF-8 text.'.format(path)

    def test_file_that_is_a_directory_is_refused(self, tmp_path):
        """A directory where the file should be is refused, not crashed on."""
        path = tmp_path / 'config.yaml'
        path.mkdir()
        with pytest.raises(config.ConfigError) as caught:
            config.load(path=str(path), environ=env_for(tmp_path), make_dirs=False)
        assert 'could not be read: Is a directory.' in str(caught.value)

    def test_oversized_file_is_refused(self, tmp_path):
        """A file larger than a mebibyte is refused before it is parsed."""
        body = '# padding padding padding padding padding padding padding\n' * 20000
        message = refusal(tmp_path, body)
        assert 'is larger than 1 MiB; a config file is a few dozen lines.' in message

    @pytest.mark.parametrize('name', INVALID_FIXTURES)
    def test_every_message_starts_with_the_config_path(self, tmp_path, name):
        """Every refusal starts with the file path, exactly once."""
        path = str(FIXTURES / name)
        with pytest.raises(config.ConfigError) as caught:
            config.load(path=path, environ=env_for(tmp_path), make_dirs=False)
        message = str(caught.value)
        assert message.startswith(path + ': ')
        assert message.count(path) == 1

    def test_no_message_contains_the_whole_file(self, tmp_path):
        """A refusal echoes the offending scalar and nothing else of the file."""
        message = refusal(tmp_path, 'bind: "10.9.8.7"\nport: 80\n')
        assert '10.9.8.7' not in message

    def test_long_string_value_is_truncated_in_the_message(self, tmp_path):
        """A very long scalar is truncated inside its quotes."""
        long_value = 'z' * 300
        message = refusal(tmp_path, 'robots:\n  panda1:\n    ip: "{} {}"\n'.format(
            long_value, 'tail'))
        assert '..."' in message
        assert long_value not in message


class TestUnknownKeys:
    """A key that silently does nothing is the failure this schema prevents."""

    def test_unknown_top_level_key_lists_the_allowed_top_level_keys(self, tmp_path):
        """An unknown root key lists every legal root key."""
        message = refusal(tmp_path, 'nonsense: 1\n')
        assert message.endswith(
            'nonsense: unknown key. Allowed top-level keys: bind, directories, '
            'fence, jog, port, profiles, recording, robots, ros_domain_id, settling.')

    def test_unknown_key_under_robots_panda1_matches_the_contract_message(self, tmp_path):
        """The plain unknown-key form is the documented sentence, verbatim."""
        message = refusal(tmp_path, 'robots:\n  panda1:\n    adress: "1.2.3.4"\n')
        assert message.endswith(
            'robots.panda1.adress: unknown key. Allowed keys under robots.panda1: ip.')

    def test_typo_within_distance_two_gets_a_did_you_mean(self, tmp_path):
        """A near miss teaches the right key, verbatim."""
        message = refusal(tmp_path, 'settling:\n  stable_windows: 1.0\n')
        assert message.endswith(
            'settling.stable_windows: unknown key. Did you mean "stable_window_s"? '
            'Allowed keys under settling: drift_limit_deg, span_limit_deg, '
            'velocity_limit_deg_s, fence_margin_deg, stable_window_s, min_samples, '
            'timeout_s.')

    def test_distant_typo_gets_no_suggestion(self, tmp_path):
        """A distant name gets the plain form, no guess."""
        message = refusal(tmp_path, 'settling:\n  completely_different: 1.0\n')
        assert 'Did you mean' not in message
        assert 'Allowed keys under settling:' in message

    def test_the_suggestion_threshold_is_exactly_distance_two(self, tmp_path):
        """
        §4.1/§4.3: distance 2 suggests, distance 3 or more does not.

        The two cases above sit at distance 1 and distance >= 5, so the
        threshold could be tightened to 1 or loosened to 4 without either of
        them noticing. These two sit either side of the real boundary.
        """
        allowed = config._ALLOWED_KEYS['settling']
        assert config._levenshtein('timeout', 'timeout_s') == 2
        assert min(config._levenshtein('drift_limit', candidate)
                   for candidate in allowed) == 4

        near = refusal(tmp_path, 'settling:\n  timeout: 1.0\n')
        assert 'Did you mean "timeout_s"?' in near

        far = refusal(tmp_path, 'settling:\n  drift_limit: 1.0\n')
        assert 'Did you mean' not in far

    def test_unknown_key_under_a_profile(self, tmp_path):
        """An unknown profile key lists the four profile keys."""
        message = refusal(tmp_path, 'profiles:\n  panda1:\n    gains: [1]\n')
        assert message.endswith('Allowed keys under profiles.panda1: stiffness, '
                                'damping, torque_limit_nm, speed_limit_deg_s.')

    def test_watchdog_timeout_key_is_an_unknown_key_with_the_contract_message(
            self, tmp_path):
        """The dropped watchdog key teaches where the value actually lives."""
        message = refusal(tmp_path, 'profiles:\n  panda1:\n    watchdog_timeout_s: 0.1\n')
        assert message.endswith(
            "profiles.panda1.watchdog_timeout_s: unknown key. The controller's "
            'watchdog timing (0.1 s) is fixed by its reviewed timing policy and is '
            'not settable from this file; it is reported read-only in '
            'GET /api/config. Allowed keys under profiles.panda1: stiffness, '
            'damping, torque_limit_nm, speed_limit_deg_s.')

    @pytest.mark.parametrize('key,noun,value', [
        ('max_header_age_s', 'header-age limit', '1.0'),
        ('future_tolerance_s', 'future-tolerance limit', '0.1')])
    def test_the_other_two_dropped_timing_keys_get_the_same_shape(
            self, tmp_path, key, noun, value):
        """The two sibling timing keys carry the same teaching sentence."""
        message = refusal(tmp_path, 'profiles:\n  panda2:\n    {}: {}\n'.format(
            key, value))
        assert message.endswith(
            "profiles.panda2.{}: unknown key. The controller's {} ({} s) is fixed "
            'by its reviewed timing policy and is not settable from this file; it '
            'is reported read-only in GET /api/config. Allowed keys under '
            'profiles.panda2: stiffness, damping, torque_limit_nm, '
            'speed_limit_deg_s.'.format(key, noun, value))

    @pytest.mark.parametrize('key', ['watchdog_timeout_s', 'max_header_age_s',
                                     'future_tolerance_s'])
    def test_dropped_timing_keys_get_no_did_you_mean(self, tmp_path, key):
        """The teaching sentence replaces the suggestion; it does not stack."""
        message = refusal(tmp_path, 'profiles:\n  panda1:\n    {}: 0.1\n'.format(key))
        assert 'Did you mean' not in message

    @pytest.mark.parametrize('section', ['robots', 'profiles', 'fence'])
    def test_unknown_arm_under_robots_profiles_and_fence(self, tmp_path, section):
        """Only panda1 and panda2 exist, in every per-arm section."""
        message = refusal(tmp_path, '{}:\n  panda3:\n    {{}}\n'.format(section))
        assert '{}.panda3: unknown key.'.format(section) in message
        assert 'Allowed keys under {}: panda1, panda2.'.format(section) in message

    def test_a_section_that_is_not_a_mapping_is_refused(self, tmp_path):
        """A scalar where a section belongs names the allowed keys."""
        message = refusal(tmp_path, 'settling: 5\n')
        assert 'settling: expected a mapping of settings, found 5 (a number).' in message

    def test_non_string_key_is_refused(self, tmp_path):
        """A non-name key is refused before any value is read."""
        message = refusal(tmp_path, '3: hello\n')
        assert message.endswith('keys must be names, found 3 (a number).')


class TestBroadcast7:
    """The scalar-or-seven-list workhorse, tested directly."""

    def test_scalar_broadcasts_to_seven(self):
        """One number becomes seven identical numbers."""
        assert config.broadcast7('k', 2.0, unit='deg') == (2.0,) * 7

    def test_seven_list_passes_through_in_order(self):
        """A seven-list keeps its order."""
        values = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        assert config.broadcast7('k', values, unit=None) == tuple(values)

    def test_wrong_length_names_the_key_and_what_is_allowed(self):
        """A six-list is a type error that names the key."""
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('settling.drift_limit_deg', [1.0] * 6, unit='deg',
                              minimum=0.0, maximum=30.0)
        assert str(caught.value).startswith(
            'settling.drift_limit_deg: expected a number or a list of 7 numbers in '
            'degrees, found a list of 6 items.')

    def test_string_value_matches_the_contract_example(self):
        """The canonical wrong-type message is byte-for-byte the documented one."""
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('settling.drift_limit_deg', 'two', unit='deg',
                              minimum=0.0, maximum=30.0)
        assert str(caught.value) == (
            'settling.drift_limit_deg: expected a number or a list of 7 numbers in '
            'degrees, found "two" (a string). Allowed: any value greater than 0, '
            'e.g. 2.0 or [2.0, 5.0, 2.0, 2.0, 2.0, 2.0, 2.0].')

    def test_allowed_clause_omits_the_maximum_for_an_exclusive_minimum(self):
        """A type error's Allowed clause names the minimum only."""
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('k', 'two', unit=None, minimum=0.0, maximum=30.0)
        assert 'Allowed: any value greater than 0.' in str(caught.value)
        assert 'at most' not in str(caught.value)

    def test_range_error_does_carry_the_maximum(self):
        """The ceiling is taught where it is the thing that went wrong."""
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('k', 40.0, unit=None, minimum=0.0, maximum=30.0)
        assert str(caught.value) == 'k: expected a value in 0 < x <= 30.0, found 40.0.'

    @pytest.mark.parametrize('maximum,rendered', [(60.0, '60.0'), (30.0, '30.0'),
                                                  (12.0, '12.0')])
    def test_every_bound_in_a_message_is_num_formatted(self, maximum, rendered):
        """No message renders a bound as a bare integer."""
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('k', maximum + 1.0, unit=None, minimum=0.0,
                              maximum=maximum)
        assert '<= {},'.format(rendered) in str(caught.value)

    def test_string_element_inside_a_list_is_refused(self):
        """One string in a seven-list is a type error for the whole key."""
        with pytest.raises(config.ConfigError):
            config.broadcast7('k', [1.0, 'x', 1.0, 1.0, 1.0, 1.0, 1.0], unit=None)

    @pytest.mark.parametrize('value', [True, False, [True] * 7])
    def test_boolean_is_not_a_number(self, value):
        """YAML booleans are never numbers."""
        with pytest.raises(config.ConfigError):
            config.broadcast7('k', value, unit=None)

    @pytest.mark.parametrize('value', [float('inf'), float('nan'),
                                       [float('inf')] * 7])
    def test_non_finite_is_refused(self, value):
        """Infinity and NaN are type errors."""
        with pytest.raises(config.ConfigError):
            config.broadcast7('k', value, unit=None)

    def test_exclusive_minimum_refuses_zero(self):
        """An exclusive minimum refuses the bound itself."""
        with pytest.raises(config.ConfigError):
            config.broadcast7('k', 0.0, unit=None, minimum=0.0)

    def test_inclusive_minimum_accepts_zero(self):
        """An inclusive minimum accepts the bound itself."""
        assert config.broadcast7('k', 0.0, unit=None, minimum=0.0,
                                 exclusive_minimum=False) == (0.0,) * 7

    def test_maximum_is_inclusive(self):
        """The ceiling itself is a legal value."""
        assert config.broadcast7('k', 30.0, unit=None, minimum=0.0,
                                 maximum=30.0) == (30.0,) * 7

    def test_list_violation_is_reported_with_an_index(self):
        """A list element is named with its index."""
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('k', [1.0, 1.0, 1.0, 99.0, 1.0, 1.0, 1.0], unit=None,
                              minimum=0.0, maximum=30.0)
        assert caught.value.key == 'k[3]'

    def test_scalar_violation_is_reported_without_an_index(self):
        """A scalar keeps the plain key."""
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('k', 99.0, unit=None, minimum=0.0, maximum=30.0)
        assert caught.value.key == 'k'

    @pytest.mark.parametrize('unit,phrase,suffix', [
        ('deg', ' in degrees', ' deg'),
        ('deg/s', ' in degrees per second', ' deg/s'),
        ('N.m', ' in newton-metres', ' N·m'),
        (None, '', '')])
    def test_unit_wording(self, unit, phrase, suffix):
        """Each unit selects its own wording in both message shapes."""
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('k', 'two', unit=unit, minimum=0.0)
        assert 'a list of 7 numbers{}, found'.format(phrase) in str(caught.value)
        with pytest.raises(config.ConfigError) as caught:
            config.broadcast7('k', 99.0, unit=unit, minimum=0.0, maximum=30.0)
        assert 'x <= 30.0{}, found 99.0.'.format(suffix) in str(caught.value)


class TestUnitConversion:
    """Degrees enter the file; radians leave the loader."""

    def test_degrees_to_radians_scalar(self):
        """A scalar converts."""
        assert config.degrees_to_radians(180.0) == math.pi

    def test_degrees_to_radians_seven_sequence(self):
        """A seven-sequence converts elementwise."""
        assert config.degrees_to_radians([0.0, 90.0, 180.0, 0.0, 0.0, 0.0, 0.0]) == (
            0.0, math.radians(90.0), math.pi, 0.0, 0.0, 0.0, 0.0)

    @pytest.mark.parametrize('values', [[1.0] * 6, [1.0] * 8, []])
    def test_degrees_to_radians_rejects_other_lengths(self, values):
        """Only seven-long sequences convert."""
        with pytest.raises(ValueError):
            config.degrees_to_radians(values)

    def test_two_degrees_is_the_jog_step_constant(self):
        """The jog step constant is exactly two degrees."""
        assert config.degrees_to_radians(2.0) == defaults.JOG_STEP_RAD

    def test_drift_limit_scalar_broadcasts_to_seven_joints(self, tmp_path):
        """A scalar drift limit reaches all seven joints."""
        settling = load_text(tmp_path, 'settling:\n  drift_limit_deg: 2.0\n').settling
        assert settling.drift_limit_rad == (math.radians(2.0),) * 7

    def test_drift_limit_per_joint_list_preserves_order(self, tmp_path):
        """A per-joint list keeps its order through the conversion."""
        settling = load_text(
            tmp_path, 'settling:\n  drift_limit_deg: [2, 5, 2, 2, 2, 2, 2]\n').settling
        assert settling.drift_limit_rad == tuple(
            math.radians(value) for value in (2, 5, 2, 2, 2, 2, 2))

    def test_file_supplied_span_uses_math_radians_not_the_default_literal(self, tmp_path):
        """A file-supplied value converts; only an ABSENT key keeps the literal."""
        settling = load_text(tmp_path, 'settling:\n  span_limit_deg: 0.05\n').settling
        assert settling.span_limit_rad == (math.radians(0.05),) * 7
        assert settling.span_limit_rad != defaults.DEFAULT_SETTLING['span_limit_rad']

    def test_jog_step_deg_becomes_radians(self, tmp_path):
        """A configured jog step is stored in radians."""
        settings = load_text(tmp_path, 'jog:\n  step_deg: 3.0\n')
        assert settings.jog_step_rad == math.radians(3.0)

    def test_jog_step_above_fifteen_is_refused(self, tmp_path):
        """The jog step carries its documented ceiling."""
        message = refusal(tmp_path, 'jog:\n  step_deg: 90.0\n')
        assert message.endswith(
            'jog.step_deg: expected a value in 0 < x <= 15.0 deg, found 90.0.')

    def test_jog_step_zero_is_refused(self, tmp_path):
        """A zero jog step is refused."""
        assert 'jog.step_deg' in refusal(tmp_path, 'jog:\n  step_deg: 0\n')

    def test_jog_step_at_fifteen_is_accepted(self, tmp_path):
        """The jog-step ceiling is inclusive."""
        settings = load_text(tmp_path, 'jog:\n  step_deg: 15\n')
        assert settings.jog_step_rad == math.radians(15.0)

    def test_speed_limit_deg_s_becomes_rad_s(self, tmp_path):
        """A speed limit in degrees per second is stored in radians per second."""
        settings = load_text(
            tmp_path, 'profiles:\n  panda1:\n    speed_limit_deg_s: 6.0\n')
        assert settings.profile('panda1').max_target_velocity_rad_s == (
            math.radians(6.0),) * 7

    def test_fence_bounds_convert_to_radians(self, tmp_path):
        """Fence bounds are degrees in the file and radians in the record."""
        lower = [-45.0] * 7
        lower[3] = -150.0
        lower[5] = 10.0
        upper = [45.0] * 7
        upper[3] = -30.0
        upper[5] = 180.0
        profile = load_text(tmp_path, fence_body(lower, upper)).profile('panda1')
        assert profile.position_lower_rad == tuple(math.radians(v) for v in lower)
        assert profile.position_upper_rad == tuple(math.radians(v) for v in upper)

    def test_public_view_carries_no_degree_key(self, tmp_path):
        """Nothing downstream of Settings speaks degrees."""
        def walk(node):
            if isinstance(node, dict):
                for key, value in node.items():
                    assert not key.endswith('_deg') and not key.endswith('_deg_s')
                    walk(value)
            elif isinstance(node, list):
                for value in node:
                    walk(value)

        walk(load_text(tmp_path, '').public_view())

    def test_public_view_is_json_serializable_with_allow_nan_false(self, tmp_path):
        """The projection is plain JSON with no non-finite value in it."""
        json.dumps(load_text(tmp_path, '').public_view(), allow_nan=False)


class TestPortBindDomain:
    """The three network-facing scalars."""

    @pytest.mark.parametrize('port', [1024, 8765, 65535])
    def test_valid_ports(self, tmp_path, port):
        """Every port inside the range loads."""
        assert load_text(tmp_path, 'port: {}\n'.format(port)).port == port

    @pytest.mark.parametrize('port', [80, 1023, 65536, 0, -1])
    def test_port_out_of_range_is_refused(self, tmp_path, port):
        """Every port outside the range is refused with the range in the message."""
        message = refusal(tmp_path, 'port: {}\n'.format(port))
        assert 'port: expected an integer in 1024..65535, found {}.'.format(
            port) in message

    @pytest.mark.parametrize('literal', ['"8765"', '8765.0', 'true', 'null'])
    def test_port_must_be_an_integer(self, tmp_path, literal):
        """A float, a string, a boolean and a null are all type errors."""
        message = refusal(tmp_path, 'port: {}\n'.format(literal))
        assert 'port: expected an integer in 1024..65535, found ' in message

    def test_bind_defaults_to_every_interface(self, tmp_path):
        """The default bind is reachable from the lab network."""
        assert load_text(tmp_path, '').bind == '0.0.0.0'

    @pytest.mark.parametrize('address', ['127.0.0.1', '::', '::1'])
    def test_bind_accepts_loopback_and_ipv6_any(self, tmp_path, address):
        """Loopback and the IPv6 wildcard are available for anyone who wants them."""
        assert load_text(tmp_path, 'bind: "{}"\n'.format(address)).bind == address

    def test_bind_rejects_a_hostname(self, tmp_path):
        """A name is not an address; the message says what is allowed."""
        message = refusal(tmp_path, 'bind: "localhost"\n')
        assert 'bind: expected an IPv4 or IPv6 address to listen on, found ' in message
        assert '0.0.0.0 (every interface)' in message

    @pytest.mark.parametrize('value,ok', [(0, True), (232, True), (233, False),
                                          (-1, False)])
    def test_ros_domain_id_range(self, tmp_path, value, ok):
        """The Fast-DDS ceiling is enforced."""
        body = 'ros_domain_id: {}\n'.format(value)
        if ok:
            assert load_text(tmp_path, body).ros_domain_id == value
        else:
            assert 'expected an integer in 0..232 or null' in refusal(tmp_path, body)

    def test_ros_domain_id_null_is_allowed(self, tmp_path):
        """An explicit null is the documented 'use the environment' spelling."""
        assert load_text(tmp_path, 'ros_domain_id: null\n').ros_domain_id == 0


class TestRobots:
    """Per-arm addresses; there is no shared address key."""

    def test_per_arm_ips_are_independent(self, tmp_path):
        """Each arm carries its own address."""
        settings = load_text(
            tmp_path,
            'robots:\n  panda1:\n    ip: "10.0.0.1"\n  panda2:\n    ip: "10.0.0.2"\n')
        assert settings.robot_ip('panda1') == '10.0.0.1'
        assert settings.robot_ip('panda2') == '10.0.0.2'

    def test_only_one_arm_configured_leaves_the_other_default(self, tmp_path):
        """Configuring one arm never moves the other."""
        settings = load_text(tmp_path, 'robots:\n  panda2:\n    ip: "10.0.0.2"\n')
        assert settings.robot_ip('panda1') == '172.16.0.2'
        assert settings.robot_ip('panda2') == '10.0.0.2'

    def test_no_shared_address_key_exists(self, tmp_path):
        """The single-address key that caused a live mislabel does not exist."""
        message = refusal(tmp_path, 'robots:\n  ip: "10.0.0.1"\n')
        assert 'robots.ip: unknown key.' in message

    @pytest.mark.parametrize('literal', ['" 1.2.3.4"', '"-lead"', '"a b"', '""',
                                         '"1.2.3.4/8"', '"1.2.3.4:9"'])
    def test_address_shape_is_refused(self, tmp_path, literal):
        """Anything that could smuggle a second launch token is refused."""
        message = refusal(tmp_path,
                          'robots:\n  panda1:\n    ip: {}\n'.format(literal))
        assert 'robots.panda1.ip: expected a hostname or IPv4 address' in message

    def test_long_address_is_refused(self, tmp_path):
        """An address longer than 254 characters is refused."""
        message = refusal(tmp_path, 'robots:\n  panda1:\n    ip: "{}"\n'.format(
            'a' * 300))
        assert 'robots.panda1.ip: expected a hostname or IPv4 address' in message

    def test_address_is_echoed_in_the_message(self, tmp_path):
        """Robot addresses are not secrets; the refusal shows what was written."""
        message = refusal(tmp_path, 'robots:\n  panda1:\n    ip: "1.2.3.4:9"\n')
        assert '"1.2.3.4:9"' in message

    def test_robot_ip_lookup_rejects_an_unknown_arm(self, tmp_path):
        """Asking for an arm that does not exist is a programming error."""
        with pytest.raises(ValueError):
            load_text(tmp_path, '').robot_ip('panda3')


class TestDirectories:
    """State and recordings are created, never gated."""

    def test_defaults_are_under_home(self, tmp_path):
        """Both directories default under the operator's home."""
        settings = load_text(tmp_path, '')
        assert settings.state_dir == str(tmp_path / '.local' / 'state' / 'franka_web')
        assert settings.recording_root == str(tmp_path / 'franka_web_recordings')

    def test_tilde_is_expanded_from_the_given_environ(self, tmp_path):
        """A leading ~ uses the environ's HOME."""
        settings = load_text(tmp_path, 'directories:\n  state: "~/elsewhere"\n')
        assert settings.state_dir == str(tmp_path / 'elsewhere')

    def test_variable_is_expanded_from_the_given_environ(self, tmp_path):
        """A $VAR uses the environ, not the process environment."""
        settings = load_text(tmp_path, 'directories:\n  state: "$ROOT/state"\n',
                             environ=env_for(tmp_path, ROOT=str(tmp_path / 'r')))
        assert settings.state_dir == str(tmp_path / 'r' / 'state')

    def test_relative_path_is_refused(self, tmp_path):
        """A relative path is refused with the allowed shapes."""
        message = refusal(tmp_path, 'directories:\n  state: "relative/dir"\n')
        assert 'directories.state: expected an absolute directory path' in message
        assert 'a path starting with /, ~ or $VAR' in message

    def test_nul_byte_is_refused(self, tmp_path):
        """An embedded NUL never reaches an os call."""
        message = refusal(tmp_path, 'directories:\n  state: "/tmp/a\\u0000b"\n')
        assert 'directories.state: expected an absolute directory path' in message

    def test_path_that_is_a_file_is_refused(self, tmp_path):
        """A file where a directory belongs is named in the refusal."""
        target = tmp_path / 'afile'
        target.write_text('x', encoding='utf-8')
        message = refusal(tmp_path, 'directories:\n  state: "{}"\n'.format(target),
                          make_dirs=True)
        assert 'directories.state: expected a directory, found a file at {}.'.format(
            target) in message

    def test_missing_directories_are_created_private(self, tmp_path):
        """Both directories are created at mode 0700 when they are missing."""
        settings = load_text(tmp_path, '', make_dirs=True)
        for path in (settings.state_dir, settings.recording_root):
            assert os.path.isdir(path)
            assert stat.S_IMODE(os.stat(path).st_mode) == 0o700

    def test_make_dirs_false_creates_nothing(self, tmp_path):
        """--check-config creates nothing."""
        settings = load_text(tmp_path, '')
        assert not os.path.exists(settings.state_dir)
        assert not os.path.exists(settings.recording_root)

    def test_existing_directory_mode_is_left_alone(self, tmp_path):
        """An existing directory is never chmodded; there are no permission mandates."""
        target = tmp_path / 'state'
        target.mkdir(mode=0o755)
        os.chmod(target, 0o755)
        load_text(tmp_path, 'directories:\n  state: "{}"\n'.format(target),
                  make_dirs=True)
        assert stat.S_IMODE(os.stat(target).st_mode) == 0o755

    def test_creation_failure_message_names_the_path_and_the_reason(self, tmp_path):
        """A creation failure names the path and the operating system's reason."""
        if os.geteuid() == 0:
            pytest.skip('root can write into a read-only directory')
        parent = tmp_path / 'readonly'
        parent.mkdir(mode=0o500)
        os.chmod(parent, 0o500)
        target = parent / 'state'
        try:
            message = refusal(tmp_path, 'directories:\n  state: "{}"\n'.format(target),
                              make_dirs=True)
        finally:
            os.chmod(parent, 0o700)
        assert 'directories.state: could not be created at {}: '.format(target) in message
        assert 'Allowed: any absolute path this user can create.' in message

    def test_franka_dir_defaults_to_null_and_is_never_created(self, tmp_path):
        """The libfranka build directory is unset by default."""
        assert load_text(tmp_path, '', make_dirs=True).franka_dir is None

    def test_franka_dir_explicit_null_is_accepted(self, tmp_path):
        """An explicit null means unset, exactly like an absent key."""
        settings = load_text(tmp_path, 'directories:\n  franka_dir: null\n',
                             make_dirs=True)
        assert settings.franka_dir is None

    def test_franka_dir_is_expanded_and_never_created(self, tmp_path):
        """A set franka_dir is expanded, required absolute, and never created."""
        settings = load_text(tmp_path, 'directories:\n  franka_dir: "~/libfranka"\n',
                             make_dirs=True)
        assert settings.franka_dir == str(tmp_path / 'libfranka')
        assert not os.path.exists(settings.franka_dir)

    @pytest.mark.parametrize('key', ['state', 'recordings'])
    def test_state_and_recordings_do_not_accept_null(self, tmp_path, key):
        """Only franka_dir is optional; a null elsewhere is the ordinary type error."""
        message = refusal(tmp_path, 'directories:\n  {}: null\n'.format(key))
        assert 'directories.{}: expected an absolute directory path, found nothing ' \
               '(null).'.format(key) in message


class TestSettling:
    """Each settling key defaults independently and teaches its own bound."""

    def test_each_key_defaults_independently(self, tmp_path):
        """The v1 all-or-none policy rule is gone."""
        settling = load_text(tmp_path, 'settling:\n  min_samples: 3\n').settling
        assert settling.min_samples == 3
        assert settling.drift_limit_rad == defaults.DEFAULT_SETTLING['drift_limit_rad']
        assert settling.timeout_s == defaults.DEFAULT_SETTLING['timeout_s']

    def test_scalar_and_list_forms_both_accepted(self, tmp_path):
        """Every angular key takes one number or seven."""
        scalar = load_text(tmp_path, 'settling:\n  fence_margin_deg: 4.0\n').settling
        listed = load_text(
            tmp_path,
            'settling:\n  fence_margin_deg: [4, 4, 4, 4, 4, 4, 4]\n').settling
        assert scalar.fence_margin_rad == listed.fence_margin_rad

    def test_drift_limit_zero_is_refused(self, tmp_path):
        """Zero drift is not a policy anyone can satisfy."""
        assert 'settling.drift_limit_deg' in refusal(
            tmp_path, 'settling:\n  drift_limit_deg: 0\n')

    def test_drift_limit_above_thirty_is_refused(self, tmp_path):
        """The drift ceiling is enforced and rendered as 30.0."""
        message = refusal(tmp_path, 'settling:\n  drift_limit_deg: 40\n')
        assert message.endswith(
            'settling.drift_limit_deg: expected a value in 0 < x <= 30.0 deg, '
            'found 40.0.')

    def test_fence_margin_zero_is_allowed(self, tmp_path):
        """A zero fence margin is legal; its minimum is inclusive."""
        settling = load_text(tmp_path, 'settling:\n  fence_margin_deg: 0\n').settling
        assert settling.fence_margin_rad == (0.0,) * 7

    @pytest.mark.parametrize('key', ['span_limit_deg', 'velocity_limit_deg_s'])
    def test_span_and_velocity_must_be_positive(self, tmp_path, key):
        """Both must be strictly greater than zero."""
        assert 'settling.{}'.format(key) in refusal(
            tmp_path, 'settling:\n  {}: 0\n'.format(key))

    def test_min_samples_below_two_is_refused(self, tmp_path):
        """A single sample is not a window."""
        message = refusal(tmp_path, 'settling:\n  min_samples: 1\n')
        assert message.endswith(
            'settling.min_samples: expected an integer of 2 or more, found 1.')

    def test_stable_window_must_be_positive(self, tmp_path):
        """A zero stable window is refused."""
        message = refusal(tmp_path, 'settling:\n  stable_window_s: 0\n')
        assert message.endswith(
            'settling.stable_window_s: expected a number greater than 0, found 0.0.')

    def test_timeout_must_exceed_the_stable_window(self, tmp_path):
        """The timeout must leave room for the window it contains."""
        message = refusal(tmp_path,
                          'settling:\n  stable_window_s: 2.0\n  timeout_s: 1.5\n')
        assert message.endswith(
            'settling.timeout_s: expected a value greater than '
            'settling.stable_window_s (2.0 s), found 1.5.')

    def test_timeout_above_sixty_is_refused(self, tmp_path):
        """The timeout ceiling is the supervisor's own budget, rendered as 60.0."""
        message = refusal(tmp_path, 'settling:\n  timeout_s: 90\n')
        assert message.endswith(
            'settling.timeout_s: expected a value in 0 < x <= 60.0 s, found 90.0.')

    def test_infeasible_timeout_message_matches_the_contract(self, tmp_path):
        """The feasibility refusal is the documented sentence, verbatim."""
        path = str(FIXTURES / 'invalid_timeout_infeasible.yaml')
        with pytest.raises(config.ConfigError) as caught:
            config.load(path=path, environ=env_for(tmp_path), make_dirs=False)
        assert str(caught.value) == (
            path + ': settling.timeout_s: 6 samples and a 1.0 s stable window need '
            "at least 1.2 s at the supervisor's 0.1 s cadence, but timeout_s is 1.1. "
            'Raise settling.timeout_s, or lower settling.min_samples / '
            'settling.stable_window_s.')

    def test_a_timeout_equal_to_the_stable_window_is_a_range_error_not_a_feasibility_error(
            self, tmp_path):
        """The range check runs first, which is why the fixture uses 1.1 and not 1.0."""
        message = refusal(
            tmp_path,
            'settling:\n  stable_window_s: 1.0\n  min_samples: 6\n  timeout_s: 1.0\n')
        assert message.endswith(
            'settling.timeout_s: expected a value greater than '
            'settling.stable_window_s (1.0 s), found 1.0.')
        assert 'need at least' not in message

    def test_the_feasibility_comparison_refuses_its_own_boundary(self, tmp_path):
        """
        §4.4: 1.2 itself is refused, because the comparison is `>=`.

        Both sides are 1_200_000_000 ns at this policy, so equality is the
        only thing separating a just-infeasible timeout from a just-feasible
        one. The shipped fixture uses 1.1, which `>` and `>=` both refuse.
        """
        message = refusal(
            tmp_path,
            'settling:\n  stable_window_s: 1.0\n  min_samples: 6\n'
            '  timeout_s: 1.2\n')
        assert message.endswith(
            'settling.timeout_s: 6 samples and a 1.0 s stable window need at '
            "least 1.2 s at the supervisor's 0.1 s cadence, but timeout_s is "
            '1.2. Raise settling.timeout_s, or lower settling.min_samples / '
            'settling.stable_window_s.')

    def test_one_nanosecond_past_the_boundary_is_feasible(self, tmp_path):
        """The refusal is exactly at the boundary, not above it."""
        settling = load_text(
            tmp_path,
            'settling:\n  stable_window_s: 1.0\n  min_samples: 6\n'
            '  timeout_s: 1.2000001\n').settling
        assert settling.timeout_s == 1.2000001

    def test_feasible_tight_policy_is_accepted(self, tmp_path):
        """A tight but feasible policy loads."""
        settling = load_text(
            tmp_path,
            'settling:\n  stable_window_s: 0.2\n  min_samples: 3\n'
            '  timeout_s: 1.0\n').settling
        assert settling.timeout_s == 1.0

    def test_policy_maps_onto_the_activation_settling_policy(self, tmp_path):
        """Every field maps onto the reviewed gate's own names."""
        settling = load_text(tmp_path, '').settling
        policy = settling.policy()
        assert policy.max_watch_delta_rad == settling.drift_limit_rad
        assert policy.max_position_span_rad == settling.span_limit_rad
        assert policy.max_abs_velocity_rad_s == settling.velocity_limit_rad_s
        assert policy.min_fence_margin_rad == settling.fence_margin_rad
        assert policy.stable_window_s == settling.stable_window_s
        assert policy.min_sample_count == settling.min_samples
        assert policy.timeout_s == settling.timeout_s

    def test_policy_is_memoized(self, tmp_path):
        """The policy object is built once."""
        settling = load_text(tmp_path, '').settling
        assert settling.policy() is settling.policy()

    def test_policy_sha256_appears_in_the_public_view(self, tmp_path):
        """The projection carries the policy digest."""
        settling = load_text(tmp_path, '').settling
        assert settling.public_view()['policy_sha256'] == settling.policy().sha256

    def test_two_identical_configs_have_the_same_policy_sha256(self, tmp_path):
        """The digest is content-addressed."""
        body = 'settling:\n  drift_limit_deg: 2.5\n'
        first = load_text(tmp_path, body).settling.policy().sha256
        second = load_text(tmp_path, body).settling.policy().sha256
        assert first == second

    def test_a_changed_drift_limit_changes_the_policy_sha256(self, tmp_path):
        """A different policy is a different digest."""
        first = load_text(tmp_path, 'settling:\n  drift_limit_deg: 2.0\n')
        second = load_text(tmp_path, 'settling:\n  drift_limit_deg: 2.5\n')
        assert (first.settling.policy().sha256 != second.settling.policy().sha256)


class TestProfiles:
    """Per-arm impedance profiles, with the reviewed timing carried, not read."""

    @pytest.mark.parametrize('key', ['stiffness', 'damping'])
    def test_stiffness_and_damping_must_be_seven_long(self, tmp_path, key):
        """Both are list-only keys of exactly seven values."""
        message = refusal(tmp_path,
                          'profiles:\n  panda1:\n    {}: [1, 2, 3]\n'.format(key))
        assert 'profiles.panda1.{}: expected a list of 7 numbers, found a list of ' \
               '3 items.'.format(key) in message

    def test_negative_stiffness_is_refused_with_an_index(self, tmp_path):
        """A negative gain names its joint index."""
        message = refusal(
            tmp_path,
            'profiles:\n  panda1:\n    stiffness: [20, 20, 20, -1.0, 10, 10, 60]\n')
        assert message.endswith(
            'profiles.panda1.stiffness[3]: expected a value of 0 or more, found -1.0.')

    def test_zero_stiffness_is_allowed(self, tmp_path):
        """A zero gain is legal (the joint is left free)."""
        settings = load_text(
            tmp_path,
            'profiles:\n  panda1:\n    stiffness: [0, 20, 20, 20, 10, 10, 60]\n')
        assert settings.profile('panda1').k_gains[0] == 0.0

    def test_torque_over_the_joint_ceiling_matches_the_contract_message(self, tmp_path):
        """The torque refusal is the documented sentence, verbatim."""
        message = refusal(
            tmp_path,
            'profiles:\n  panda2:\n'
            '    torque_limit_nm: [10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 20.0]\n')
        assert message.endswith(
            'profiles.panda2.torque_limit_nm[6]: expected a value in 0 < x <= 12.0 '
            'N·m (the Panda joint-7 hardware ceiling), found 20.0.')

    def test_torque_zero_is_refused(self, tmp_path):
        """A zero torque ceiling would make the arm uncommandable."""
        message = refusal(
            tmp_path,
            'profiles:\n  panda1:\n'
            '    torque_limit_nm: [0, 10.0, 10.0, 10.0, 5.0, 5.0, 3.0]\n')
        assert 'profiles.panda1.torque_limit_nm[0]' in message

    def test_speed_limit_zero_is_refused(self, tmp_path):
        """
        §4.2's strict `0 < x` on speed_limit_deg_s, exercised.

        A zero speed limit loads an arm whose internal target can never
        slew: the impedance controller accepts every jog and the arm never
        moves. That silently-inert setting is exactly what this schema
        exists to refuse, and only broadcast7's exclusive minimum stops it.
        """
        message = refusal(
            tmp_path, 'profiles:\n  panda1:\n    speed_limit_deg_s: 0\n')
        assert 'profiles.panda1.speed_limit_deg_s' in message

        indexed = refusal(
            tmp_path,
            'profiles:\n  panda1:\n'
            '    speed_limit_deg_s: [10.0, 0, 10.0, 10.0, 10.0, 10.0, 10.0]\n')
        assert 'profiles.panda1.speed_limit_deg_s[1]' in indexed

    def test_torque_at_the_joint_ceiling_is_accepted(self, tmp_path):
        """The hardware ceiling itself is a legal value."""
        ceiling = list(defaults.POLICY_EFFORT_CEILING_NM)
        settings = load_text(
            tmp_path,
            'profiles:\n  panda1:\n    torque_limit_nm: {}\n'.format(ceiling))
        assert settings.profile('panda1').max_effort_nm == tuple(ceiling)

    def test_speed_limit_over_the_urdf_ceiling_is_refused_naming_the_joint(self, tmp_path):
        """The velocity refusal quotes the inward-rounded ceiling, never 149.542."""
        message = refusal(
            tmp_path,
            'profiles:\n  panda1:\n'
            '    speed_limit_deg_s: [5, 5, 5, 5, 200.0, 5, 5]\n')
        assert message.endswith(
            'profiles.panda1.speed_limit_deg_s[4]: expected a value in '
            '0 < x <= 149.541 deg/s (the factory URDF velocity ceiling for joint 5), '
            'found 200.0.')
        assert '149.542' not in message

    @pytest.mark.parametrize('index', list(range(7)))
    def test_speed_limit_at_the_quoted_degree_ceiling_is_accepted(self, tmp_path, index):
        """Every ceiling a refusal quotes loads on the very next attempt."""
        quoted = config._deg_upper(defaults.POLICY_VELOCITY_CEILING_RAD_S[index])
        values = [1.0] * 7
        values[index] = quoted
        settings = load_text(
            tmp_path,
            'profiles:\n  panda1:\n    speed_limit_deg_s: {}\n'.format(values))
        stored = settings.profile('panda1').max_target_velocity_rad_s[index]
        assert stored <= defaults.POLICY_VELOCITY_CEILING_RAD_S[index]

    def test_speed_limit_just_above_the_ceiling_is_snapped_not_refused(self, tmp_path):
        """A value a few micro-radians over the ceiling snaps onto it."""
        settings = load_text(
            tmp_path,
            'profiles:\n  panda1:\n'
            '    speed_limit_deg_s: [1, 1, 1, 1, 149.542, 1, 1]\n')
        assert settings.profile('panda1').max_target_velocity_rad_s[4] == 2.61

    def test_speed_limit_well_above_the_ceiling_is_still_refused(self, tmp_path):
        """The snap is a boundary tolerance, not a licence."""
        message = refusal(
            tmp_path,
            'profiles:\n  panda1:\n'
            '    speed_limit_deg_s: [1, 1, 1, 1, 149.6, 1, 1]\n')
        assert 'profiles.panda1.speed_limit_deg_s[4]' in message

    def test_speed_limit_scalar_and_list_forms(self, tmp_path):
        """Both spellings produce the same vector."""
        scalar = load_text(
            tmp_path, 'profiles:\n  panda1:\n    speed_limit_deg_s: 4.0\n')
        listed = load_text(
            tmp_path,
            'profiles:\n  panda1:\n    speed_limit_deg_s: [4, 4, 4, 4, 4, 4, 4]\n')
        assert (scalar.profile('panda1').max_target_velocity_rad_s
                == listed.profile('panda1').max_target_velocity_rad_s)

    @pytest.mark.parametrize('body', [
        '', 'profiles:\n  panda1:\n    stiffness: [1, 1, 1, 1, 1, 1, 1]\n'])
    @pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
    def test_the_three_timing_fields_always_come_from_reviewed_timing(
            self, tmp_path, body, arm_id):
        """The reviewed timing is carried, never read from the file."""
        profile = load_text(tmp_path, body).profile(arm_id)
        assert profile.watchdog_timeout_s == defaults.REVIEWED_TIMING_S['watchdog_timeout']
        assert profile.max_header_age_s == defaults.REVIEWED_TIMING_S['max_header_age']
        assert profile.future_tolerance_s == defaults.REVIEWED_TIMING_S['future_tolerance']

    @pytest.mark.parametrize('body,field,expected', [
        ('profiles:\n  panda1:\n    stiffness: [1, 2, 3, 4, 5, 6, 7]\n',
         'k_gains', (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
        ('profiles:\n  panda1:\n    damping: [1, 2, 3, 4, 5, 6, 7]\n',
         'd_gains', (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
        ('profiles:\n  panda1:\n'
         '    torque_limit_nm: [1, 2, 3, 4, 5, 6, 7]\n',
         'max_effort_nm', (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
        ('profiles:\n  panda1:\n    speed_limit_deg_s: 3.0\n',
         'max_target_velocity_rad_s', (math.radians(3.0),) * 7),
        ('fence:\n  panda1:\n    enabled: true\n', 'fence_enabled', True),
    ])
    def test_profile_file_keys_map_to_the_contract_fields(
            self, tmp_path, body, field, expected):
        """Each file key changes exactly its counterpart field."""
        baked = defaults.DEFAULT_PROFILES['panda1']
        profile = load_text(tmp_path, body).profile('panda1')
        assert getattr(profile, field) == expected
        for other, key in (('k_gains', 'k_gains'), ('d_gains', 'd_gains'),
                           ('max_effort_nm', 'max_effort_nm'),
                           ('max_target_velocity_rad_s', 'max_target_velocity_rad_s')):
            if other == field:
                continue
            assert getattr(profile, other) == baked[key]

    def test_fence_file_keys_map_to_the_position_fields(self, tmp_path):
        """The fence bound keys land on the position fields, converted."""
        lower = [-45.0, -45.0, -45.0, -150.0, -45.0, 10.0, -45.0]
        upper = [45.0, 45.0, 45.0, -30.0, 45.0, 180.0, 45.0]
        profile = load_text(tmp_path, fence_body(lower, upper)).profile('panda1')
        assert profile.position_lower_rad == tuple(math.radians(v) for v in lower)
        assert profile.position_upper_rad == tuple(math.radians(v) for v in upper)

    def test_a_torque_ceiling_below_the_hardware_bound_loads_without_complaint(
            self, tmp_path, capsys):
        """A legal-but-unproven ceiling loads silently; the startup WARN is elsewhere."""
        settings = load_text(
            tmp_path,
            'profiles:\n  panda1:\n'
            '    torque_limit_nm: [20.0, 20.0, 20.0, 20.0, 8.0, 8.0, 8.0]\n')
        assert settings.profile('panda1').max_effort_nm[0] == 20.0
        captured = capsys.readouterr()
        assert captured.out == '' and captured.err == ''

    def test_source_is_config_when_the_file_supplies_any_profile_key(self, tmp_path):
        """One profile key is enough to mark the arm as file-sourced."""
        settings = load_text(
            tmp_path, 'profiles:\n  panda1:\n    speed_limit_deg_s: 4.0\n')
        assert settings.profile('panda1').public_view()['source'] == 'config'
        assert settings.profile('panda2').public_view()['source'] == 'default'

    def test_source_is_config_when_only_the_fence_is_configured(self, tmp_path):
        """The fence feeds the same record, so it also marks the arm."""
        settings = load_text(tmp_path, 'fence:\n  panda1:\n    enabled: false\n')
        assert settings.profile('panda1').public_view()['source'] == 'config'

    def test_configuring_one_arm_leaves_the_other_default(self, tmp_path):
        """Configuring panda1 never moves panda2."""
        settings = load_text(
            tmp_path,
            'profiles:\n  panda1:\n    stiffness: [1, 1, 1, 1, 1, 1, 1]\n')
        assert settings.profile('panda2').k_gains == defaults.STIFF_J2_PROFILE['k_gains']

    def test_profile_lookup_rejects_an_unknown_arm(self, tmp_path):
        """Asking for an arm that does not exist is a programming error."""
        with pytest.raises(ValueError):
            load_text(tmp_path, '').profile('panda3')

    def test_public_view_key_set_matches_the_contract(self, tmp_path):
        """The wire object carries exactly the eleven documented keys."""
        view = load_text(tmp_path, '').profile('panda1').public_view()
        assert set(view) == {
            'k_gains', 'd_gains', 'max_effort_nm', 'max_target_velocity_rad_s',
            'watchdog_timeout_s', 'max_header_age_s', 'future_tolerance_s',
            'fence_enabled', 'position_lower_rad', 'position_upper_rad', 'source'}


class TestFence:
    """The optional per-arm position sandbox, off by default."""

    def test_disabled_by_default(self, tmp_path):
        """There is no fence unless the operator asks for one."""
        assert load_text(tmp_path, '').profile('panda1').fence_enabled is False

    def test_disabled_bounds_are_the_factory_policy_bit_exact(self, tmp_path):
        """Fence off means exactly the factory bounds, bit for bit."""
        profile = load_text(tmp_path, 'fence:\n  panda1:\n    enabled: false\n').profile(
            'panda1')
        assert profile.position_lower_rad == defaults.POLICY_POSITION_LOWER_RAD
        assert profile.position_upper_rad == defaults.POLICY_POSITION_UPPER_RAD

    def test_enabled_sandbox_narrows_the_bounds(self, tmp_path):
        """A sandbox is a tighter box inside the factory limits."""
        lower = [-45.0, -45.0, -45.0, -150.0, -45.0, 10.0, -45.0]
        upper = [45.0, 45.0, 45.0, -30.0, 45.0, 180.0, 45.0]
        profile = load_text(tmp_path, fence_body(lower, upper)).profile('panda1')
        assert profile.fence_enabled is True
        for index in range(7):
            assert (profile.position_lower_rad[index]
                    > defaults.POLICY_POSITION_LOWER_RAD[index])
            assert (profile.position_upper_rad[index]
                    < defaults.POLICY_POSITION_UPPER_RAD[index])

    def test_lower_below_the_policy_is_refused_naming_the_joint(self, tmp_path):
        """A bound outside the factory box names the joint and the legal bound."""
        lower = list(POLICY_LOWER_DEG)
        lower[3] = -180.0
        message = refusal(tmp_path, fence_body(lower, POLICY_UPPER_DEG))
        assert message.endswith(
            'fence.panda1.lower_deg[3]: expected a value of at least -176.001 deg '
            '(the Panda joint-4 factory lower limit), found -180.0.')

    def test_upper_above_the_policy_is_refused_naming_the_joint(self, tmp_path):
        """An upper bound above the factory box names the joint."""
        upper = list(POLICY_UPPER_DEG)
        upper[3] = 10.0
        message = refusal(tmp_path, fence_body(POLICY_LOWER_DEG, upper))
        assert message.endswith(
            'fence.panda1.upper_deg[3]: expected a value of at most -4.0 deg '
            '(the Panda joint-4 factory upper limit), found 10.0.')

    def test_inverted_bounds_are_refused_naming_the_joint(self, tmp_path):
        """A lower bound above its upper bound is refused."""
        lower = list(POLICY_LOWER_DEG)
        lower[3] = -3.0
        upper = list(POLICY_UPPER_DEG)
        upper[3] = -4.0
        message = refusal(tmp_path, fence_body(lower, upper))
        assert message.endswith(
            'fence.panda1: joint 4 lower_deg (-3.0) must be less than '
            'upper_deg (-4.0).')

    def test_bounds_without_enabled_are_refused_with_a_teaching_message(self, tmp_path):
        """An inert bound is the failure mode this validation exists to prevent."""
        message = refusal(tmp_path, fence_body(POLICY_LOWER_DEG, POLICY_UPPER_DEG,
                                               enabled=False))
        assert message.endswith(
            'fence.panda1.lower_deg: set fence.panda1.enabled: true to use these '
            'bounds, or remove lower_deg and upper_deg. With the fence off the arm '
            'uses the Panda factory limits.')

    def test_enabled_without_bounds_is_the_factory_box(self, tmp_path):
        """Turning the fence on without bounds is explicit and harmless."""
        profile = load_text(tmp_path, 'fence:\n  panda1:\n    enabled: true\n').profile(
            'panda1')
        assert profile.fence_enabled is True
        assert profile.position_lower_rad == defaults.POLICY_POSITION_LOWER_RAD
        assert profile.position_upper_rad == defaults.POLICY_POSITION_UPPER_RAD

    @pytest.mark.parametrize('index', list(range(7)))
    @pytest.mark.parametrize('side', ['lower', 'upper'])
    def test_policy_boundary_in_degrees_is_accepted_on_every_joint(
            self, tmp_path, index, side):
        """A 3-decimal degree spelling of a policy bound loads on every joint."""
        lower = list(POLICY_LOWER_DEG)
        upper = list(POLICY_UPPER_DEG)
        profile = load_text(tmp_path, fence_body(lower, upper)).profile('panda1')
        if side == 'lower':
            stored = profile.position_lower_rad[index]
            policy = defaults.POLICY_POSITION_LOWER_RAD[index]
            assert stored >= policy
        else:
            stored = profile.position_upper_rad[index]
            policy = defaults.POLICY_POSITION_UPPER_RAD[index]
            assert stored <= policy
        assert abs(stored - policy) <= config._SNAP_RAD

    @pytest.mark.parametrize('index,side,value', [(5, 'lower', -1.003),
                                                  (3, 'upper', -3.999)])
    def test_a_bound_just_outside_the_policy_snaps_onto_it(self, tmp_path, index,
                                                           side, value):
        """The two bounds that round outward snap exactly onto the policy."""
        lower = list(POLICY_LOWER_DEG)
        upper = list(POLICY_UPPER_DEG)
        if side == 'lower':
            lower[index] = value
        else:
            upper[index] = value
        profile = load_text(tmp_path, fence_body(lower, upper)).profile('panda1')
        if side == 'lower':
            assert profile.position_lower_rad[index] == (
                defaults.POLICY_POSITION_LOWER_RAD[index])
        else:
            assert profile.position_upper_rad[index] == (
                defaults.POLICY_POSITION_UPPER_RAD[index])

    def test_a_bound_just_inside_the_policy_is_left_alone(self, tmp_path):
        """The snap is inward-only; it never widens an operator's sandbox."""
        lower = list(POLICY_LOWER_DEG)
        lower[5] = -1.002
        profile = load_text(tmp_path, fence_body(lower, POLICY_UPPER_DEG)).profile(
            'panda1')
        assert profile.position_lower_rad[5] == math.radians(-1.002)
        assert profile.position_lower_rad[5] != defaults.POLICY_POSITION_LOWER_RAD[5]

    @pytest.mark.parametrize('side,value', [('lower', -166.0028),
                                            ('upper', 166.0028)])
    def test_a_bound_inside_the_snap_window_is_never_widened(
            self, tmp_path, side, value):
        """
        §4.3: the snap only ever moves a bound onto or INSIDE the policy.

        The case above sits 1.18e-05 rad from its policy value -- outside the
        8.73e-06 rad snap window -- so a bidirectional snap would leave it
        alone too. These two sit INSIDE the window on the inner side, where a
        bidirectional guard would move them outward onto the factory limit
        and silently widen the operator's sandbox.
        """
        lower = list(POLICY_LOWER_DEG)
        upper = list(POLICY_UPPER_DEG)
        if side == 'lower':
            lower[0] = value
        else:
            upper[0] = value
        profile = load_text(tmp_path, fence_body(lower, upper)).profile('panda1')
        stored = (profile.position_lower_rad[0] if side == 'lower'
                  else profile.position_upper_rad[0])
        policy = (defaults.POLICY_POSITION_LOWER_RAD[0] if side == 'lower'
                  else defaults.POLICY_POSITION_UPPER_RAD[0])
        assert abs(stored - policy) <= config._SNAP_RAD, 'not in the window'
        assert stored == math.radians(value)
        assert stored != policy

    @pytest.mark.parametrize('index', list(range(7)))
    @pytest.mark.parametrize('side', ['lower', 'upper'])
    def test_every_quoted_fence_bound_is_itself_accepted(self, tmp_path, index, side):
        """A refusal never names a value the very next load would reject."""
        lower = list(POLICY_LOWER_DEG)
        upper = list(POLICY_UPPER_DEG)
        if side == 'lower':
            lower[index] = POLICY_LOWER_DEG[index] - 10.0
        else:
            upper[index] = POLICY_UPPER_DEG[index] + 10.0
        message = refusal(tmp_path, fence_body(lower, upper))
        quoted = re.search(r'(-?\d+(?:\.\d+)?) deg', message)
        assert quoted is not None, message
        if side == 'lower':
            lower[index] = float(quoted.group(1))
        else:
            upper[index] = float(quoted.group(1))
        load_text(tmp_path, fence_body(lower, upper))

    def test_a_bound_outside_the_snap_is_still_refused(self, tmp_path):
        """One hundredth of a degree outside is refused, not snapped."""
        path = str(FIXTURES / 'invalid_fence_below_the_snap.yaml')
        with pytest.raises(config.ConfigError) as caught:
            config.load(path=path, environ=env_for(tmp_path), make_dirs=False)
        assert caught.value.key == 'fence.panda1.lower_deg[3]'
        assert 'joint-4' in str(caught.value)

    def test_per_arm_fences_are_independent(self, tmp_path):
        """A sandbox on one arm leaves the other at the factory limits."""
        lower = [-45.0, -45.0, -45.0, -150.0, -45.0, 10.0, -45.0]
        upper = [45.0, 45.0, 45.0, -30.0, 45.0, 180.0, 45.0]
        settings = load_text(tmp_path, fence_body(lower, upper, arm_id='panda2'))
        assert settings.profile('panda1').fence_enabled is False
        assert (settings.profile('panda1').position_lower_rad
                == defaults.POLICY_POSITION_LOWER_RAD)
        assert settings.profile('panda2').fence_enabled is True


class TestFixtures:
    """The shipped fixtures and the shipped example file."""

    @pytest.mark.parametrize('name', VALID_FIXTURES)
    def test_valid_fixtures_load(self, tmp_path, name):
        """Every valid fixture loads."""
        settings = config.load(path=str(FIXTURES / name), environ=env_for(tmp_path),
                               make_dirs=False)
        assert settings.config_present is True

    @pytest.mark.parametrize('name', INVALID_FIXTURES)
    def test_invalid_fixtures_are_refused(self, tmp_path, name):
        """Every invalid fixture is refused, naming its own defect."""
        path = str(FIXTURES / name)
        with pytest.raises(config.ConfigError) as caught:
            config.load(path=path, environ=env_for(tmp_path), make_dirs=False)
        message = str(caught.value)
        assert message.startswith(path + ': ')
        assert INVALID_FIXTURE_KEYS[name] in message
        assert 'Traceback' not in message

    def test_every_fixture_is_covered(self):
        """No fixture file is orphaned by the two parametrized sets."""
        every = sorted(path.name for path in FIXTURES.glob('*.yaml'))
        assert every == sorted(VALID_FIXTURES + INVALID_FIXTURES)
        assert sorted(INVALID_FIXTURE_KEYS) == INVALID_FIXTURES

    def test_example_config_loads_when_uncommented(self, tmp_path):
        """Every commented-out block of the shipped example loads when uncommented."""
        if not PACKAGE_ROOT.is_dir():  # pragma: no cover - installed tree
            pytest.skip('package root is not present in this layout')
        assert EXAMPLE.is_file(), 'the shipped example config is missing'
        lines = EXAMPLE.read_text(encoding='utf-8').splitlines()
        body = [line[len(SENTINEL):] for line in lines if line.startswith(SENTINEL)]
        assert body, 'the example lost its "#| " prefixes'
        path = tmp_path / 'uncommented.yaml'
        path.write_text('\n'.join(body) + '\n', encoding='utf-8')
        settings = config.load(path=str(path), environ=env_for(tmp_path),
                               make_dirs=False)
        assert isinstance(settings, config.Settings)
        assert settings.profile('panda1').fence_enabled is True

        # ... and it documents the WHOLE §4.2 surface. Without this, a key
        # added to the schema -- or one silently dropped from the example --
        # leaves the operator-facing documentation short and nothing fails.
        documented = set(_dotted_keys(yaml.safe_load('\n'.join(body))))
        required = set()
        for section, names in config._ALLOWED_KEYS.items():
            for name in names:
                required.add(name if not section
                             else '{}.{}'.format(section, name))
        # The one documented exemption: the example says in prose that "the
        # same three keys work under panda2:" rather than repeating them.
        required = {dotted for dotted in required
                    if not dotted.startswith('fence.panda2')}
        assert required - documented == set(), (
            'the shipped example documents no {}'.format(
                sorted(required - documented)))

    def test_example_config_uses_the_sentinel_for_every_key_line(self):
        """Prose is prose and settings carry the sentinel; nothing is ambiguous."""
        if not PACKAGE_ROOT.is_dir():  # pragma: no cover - installed tree
            pytest.skip('package root is not present in this layout')
        key_like = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*:(\s|$)')
        for line in EXAMPLE.read_text(encoding='utf-8').splitlines():
            if line.startswith(SENTINEL) or not line.startswith('#'):
                continue
            prose = line[2:] if line.startswith('# ') else line[1:]
            assert not key_like.match(prose), line

    def test_example_config_shows_no_refused_combination(self):
        """The example never shows a fence bound under a disabled fence."""
        text = EXAMPLE.read_text(encoding='utf-8')
        assert 'enabled: false' not in text
        for key in ('watchdog_timeout_s', 'max_header_age_s', 'future_tolerance_s'):
            assert '{}{}:'.format(SENTINEL.rstrip(), key) not in text


class TestConfigError:
    """The one exception type PART2 catches."""

    def test_single_argument_form_renders_verbatim(self):
        """A one-argument error renders its sentence unchanged."""
        assert str(config.ConfigError('something went wrong')) == 'something went wrong'

    def test_with_path_prefixes_a_bare_error(self):
        """with_path adds the file path in front of an existing problem."""
        error = config.ConfigError('port', 'expected an integer.')
        assert str(error.with_path('/etc/x.yaml')) == (
            '/etc/x.yaml: port: expected an integer.')

    def test_is_a_value_error(self):
        """The error stays a ValueError so existing handlers keep working."""
        assert issubclass(config.ConfigError, ValueError)

    def test_key_and_problem_are_available_as_attributes(self):
        """The parts stay reachable for a caller that wants them."""
        error = config.ConfigError('settling.timeout_s', 'too small.', '/x.yaml')
        assert error.key == 'settling.timeout_s'
        assert error.problem == 'too small.'
        assert error.path == '/x.yaml'


class TestSettingsRecord:
    """The record itself: frozen, read-only inside, and its wire projection."""

    def test_settings_is_frozen(self, tmp_path):
        """Nobody rewrites a loaded Settings."""
        settings = load_text(tmp_path, '')
        with pytest.raises(FrozenInstanceError):
            settings.port = 1

    def test_robot_ips_mapping_is_read_only(self, tmp_path):
        """The interior address map cannot be rewritten by a consumer."""
        settings = load_text(tmp_path, '')
        with pytest.raises(TypeError):
            settings.robot_ips['panda1'] = '10.0.0.1'

    def test_profiles_mapping_is_read_only(self, tmp_path):
        """The interior profile map cannot be rewritten by a consumer."""
        settings = load_text(tmp_path, '')
        with pytest.raises(TypeError):
            settings.profiles['panda1'] = None

    def test_public_view_key_set_matches_contract_1_2_2(self, tmp_path):
        """The projection carries exactly the documented top-level keys."""
        assert set(load_text(tmp_path, '').public_view()) == {
            'config_path', 'config_present', 'port', 'bind', 'ros_domain_id',
            'state_dir', 'recording_root', 'recording_enabled', 'jog_step_rad',
            'robots', 'settling', 'profiles'}

    def test_public_view_has_no_franka_dir_key(self, tmp_path):
        """The libfranka build directory is an install detail, not a setting."""
        view = load_text(tmp_path, 'directories:\n  franka_dir: "/opt/libfranka"\n',
                         make_dirs=False).public_view()
        assert 'franka_dir' not in view
        assert 'franka_dir' not in view.get('profiles', {})

    def test_public_view_includes_robot_addresses(self, tmp_path):
        """Robot addresses are not secrets and are reported."""
        view = load_text(tmp_path, '').public_view()
        assert view['robots'] == {'panda1': '172.16.0.2', 'panda2': '172.16.0.3'}
