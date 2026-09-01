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
Tests for franka_web.preflight: argv, report parsing, and the blocking rule.

Most tests drive a fake runner with canned stdout. A handful deliberately use
the real :func:`subprocess.run` against a throwaway script written into
``tmp_path`` -- a plain, non-ROS child -- so the invocation contract
(``capture_output``/``text``/``timeout``) is proven against the real thing and
not only against the fake.

Robot addresses in these fixtures are RFC 5737 documentation addresses
(203.0.113.0/24).
"""

from datetime import datetime, timedelta, timezone
import json
import re
import subprocess

from ament_index_python.packages import PackageNotFoundError
from franka_web import defaults, preflight
from franka_web.preflight import PreflightResult
import pytest

DOC_IP_1 = '203.0.113.7'
DOC_IP_2 = '203.0.113.8'

FAKE_PREFIX = '/opt/ws/install/franka_bringup'
FAKE_BINARY = FAKE_PREFIX + '/lib/franka_bringup/franka_rt_preflight'
FAKE_FRANKA_DIR = '/opt/libfranka/build'

RFC3339_UTC = re.compile(r'\A\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{6}Z\Z')

FIXED_NOW = datetime(2026, 8, 30, 14, 15, 0, 123456, tzinfo=timezone.utc)
FIXED_STAMP = '2026-08-30T14:15:00.123456Z'


def _check(name, status, summary='', evidence=''):
    """Build one check entry in the tool's own key layout."""
    return {'name': name, 'status': status, 'summary': summary, 'evidence': evidence}


def _report(overall, checks):
    """Build a report with the same surrounding keys the real tool emits."""
    return {
        'schema_version': 1,
        'overall': overall,
        'read_only': True,
        'network_access': False,
        'checks': checks,
        'versions': {'ros': {'distro': 'jazzy'}},
    }


PASS_CHECKS = [
    _check('kernel', 'pass', 'active kernel has PREEMPT_RT', '6.8.1-1058-realtime'),
    _check('rt_priority_limit', 'pass', 'RTPRIO soft limit covers priority 50', 'soft=99'),
    _check('ros_distro', 'pass', 'ROS 2 Jazzy is sourced', 'distro=jazzy'),
]

FAIL_CHECKS = [
    _check('kernel', 'fail', 'active kernel is not proven PREEMPT_RT', '6.8.0-generic'),
    _check('memlock_limit', 'warn', 'memlock soft limit is not unlimited', 'soft=8388608'),
    _check('ros_distro', 'pass', 'ROS 2 Jazzy is sourced', 'distro=jazzy'),
]

PASS_STDOUT = json.dumps(_report('pass', PASS_CHECKS), indent=2, sort_keys=True)
FAIL_STDOUT = json.dumps(_report('fail', FAIL_CHECKS), indent=2, sort_keys=True)


class FakeRunner:
    """
    Stand-in for :func:`subprocess.run` that records its call.

    It returns a canned :class:`subprocess.CompletedProcess`, or raises the
    exception it was given, so every invocation-level failure mode is
    reachable without touching a real process.
    """

    def __init__(self, stdout='', stderr='', returncode=0, raises=None):
        """Prepare a runner returning ``stdout``, or raising ``raises``."""
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode
        self.raises = raises
        self.calls = []

    def __call__(self, argv, capture_output=False, timeout=None, text=False):
        """Record one invocation and produce the canned result."""
        self.calls.append({
            'argv': argv,
            'capture_output': capture_output,
            'timeout': timeout,
            'text': text,
        })
        if self.raises is not None:
            raise self.raises
        return subprocess.CompletedProcess(argv, self.returncode, self.stdout, self.stderr)


class _Settings:
    """
    The two fields the preflight actually reads, and nothing else.

    ``build_argv`` reads ``franka_dir`` and ``run_preflight`` reads nothing
    else, so the whole module is testable without a configuration file.
    """

    def __init__(self, franka_dir=None):
        """Bind the one setting this module consumes."""
        self.franka_dir = franka_dir
        self.robot_ips = {'panda1': DOC_IP_1, 'panda2': DOC_IP_2}

    def robot_ip(self, arm_id):
        """Return one arm's documentation address."""
        return self.robot_ips[arm_id]


def _settings(franka_dir=None):
    """Build the minimal settings stand-in the preflight consumes."""
    return _Settings(franka_dir)


@pytest.fixture()
def settings():
    """Return valid settings with no libfranka directory configured."""
    return _settings()


@pytest.fixture()
def fake_binary(monkeypatch):
    """Pin ``preflight_binary`` so argv does not depend on the install tree."""
    monkeypatch.setattr(preflight, 'preflight_binary', lambda: FAKE_BINARY)
    return FAKE_BINARY


def _write_script(tmp_path, body):
    """Write an executable throwaway python script and return its path."""
    script = tmp_path / 'fake_rt_preflight'
    script.write_text('#!/usr/bin/env python3\n' + body)
    script.chmod(0o700)
    return str(script)


class TestPreflightBinary:
    """The binary is located through the ament index, never hard-coded."""

    def test_path_is_the_bringup_libexec_layout(self, monkeypatch):
        """The path is <prefix>/lib/franka_bringup/franka_rt_preflight."""
        monkeypatch.setattr(preflight, 'get_package_prefix', lambda name: FAKE_PREFIX)
        assert preflight.preflight_binary() == FAKE_BINARY

    def test_prefix_is_looked_up_for_franka_bringup(self, monkeypatch):
        """The package queried is franka_bringup, which installs the tool."""
        asked = []

        def _prefix(name):
            asked.append(name)
            return FAKE_PREFIX

        monkeypatch.setattr(preflight, 'get_package_prefix', _prefix)
        preflight.preflight_binary()
        assert asked == ['franka_bringup']

    def test_installed_binary_path_is_absolute(self):
        """Against the real index the result is an absolute path."""
        try:
            path = preflight.preflight_binary()
        except PackageNotFoundError:
            pytest.skip('franka_bringup is not on the ament index in this environment')
        assert path.startswith('/')
        assert path.endswith('/lib/franka_bringup/franka_rt_preflight')


class TestBuildArgv:
    """argv is exactly --json, plus --franka-dir only when one is configured."""

    def test_without_franka_dir(self, settings, fake_binary):
        """An unset libfranka directory leaves the tool on its own default."""
        assert preflight.build_argv(settings) == (FAKE_BINARY, '--json')

    def test_with_franka_dir(self, fake_binary):
        """A configured libfranka directory is passed through verbatim."""
        argv = preflight.build_argv(_settings(FAKE_FRANKA_DIR))
        assert argv == (FAKE_BINARY, '--json', '--franka-dir', FAKE_FRANKA_DIR)

    def test_blank_franka_dir_is_omitted(self, fake_binary):
        """A blank directory is treated as unset rather than passed empty."""
        assert preflight.build_argv(_settings('')) == (FAKE_BINARY, '--json')

    def test_argv_is_an_immutable_tuple(self, settings, fake_binary):
        """The argv is a tuple, so no caller can append to the command line."""
        assert isinstance(preflight.build_argv(settings), tuple)

    def test_priority_is_left_to_the_tool(self, fake_binary):
        """The reviewed default priority lives in the tool, not here."""
        assert '--priority' not in preflight.build_argv(_settings(FAKE_FRANKA_DIR))


class TestReportParsing:
    """A well-formed report becomes a verdict plus the non-PASS checks."""

    def test_pass_report(self, settings, fake_binary):
        """A passing run is PASS, passed, with nothing in failed_checks."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        result = preflight.run_preflight(settings, 'watch', runner=runner)
        assert result.overall == 'PASS'
        assert result.passed is True
        assert result.failed_checks == []
        assert result.error is None

    def test_fail_report(self, settings, fake_binary):
        """A failing run is FAIL and not passed."""
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=FAIL_STDOUT))
        assert result.overall == 'FAIL'
        assert result.passed is False
        assert result.error is None

    def test_failed_checks_extraction(self, settings, fake_binary):
        """Every non-PASS check is carried across with all four fields."""
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=FAIL_STDOUT))
        assert result.failed_checks == [
            {
                'status': 'FAIL',
                'name': 'kernel',
                'summary': 'active kernel is not proven PREEMPT_RT',
                'evidence': '6.8.0-generic',
            },
            {
                'status': 'WARN',
                'name': 'memlock_limit',
                'summary': 'memlock soft limit is not unlimited',
                'evidence': 'soft=8388608',
            },
        ]

    def test_passing_checks_are_not_listed(self, settings, fake_binary):
        """The passing ros_distro check is not reported as a failure."""
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=FAIL_STDOUT))
        assert [check['name'] for check in result.failed_checks] == ['kernel', 'memlock_limit']

    def test_warn_overall_is_not_a_pass(self, settings, fake_binary):
        """Only PASS passes: a WARN verdict is reported and does not pass."""
        stdout = json.dumps(_report('warn', [
            _check('memlock_limit', 'warn', 'memlock soft limit is not unlimited', 'soft=8388608'),
        ]))
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=stdout))
        assert result.overall == 'WARN'
        assert result.passed is False
        assert result.failed_checks[0]['status'] == 'WARN'

    def test_already_upper_case_report_is_accepted(self, settings, fake_binary):
        """An upper-case verdict parses identically (case is normalized once)."""
        stdout = json.dumps(_report('PASS', [_check('kernel', 'PASS', 'ok', '')]))
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=stdout))
        assert (result.overall, result.passed, result.failed_checks) == ('PASS', True, [])

    def test_missing_check_text_becomes_empty_strings(self, settings, fake_binary):
        """A check without summary/evidence still yields all four fields."""
        stdout = json.dumps(_report('fail', [{'name': 'kernel', 'status': 'fail'}]))
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=stdout))
        assert result.failed_checks == [
            {'status': 'FAIL', 'name': 'kernel', 'summary': '', 'evidence': ''},
        ]

    def test_empty_checks_list_is_a_valid_report(self, settings, fake_binary):
        """A report with no checks parses; it simply has no failures."""
        stdout = json.dumps(_report('pass', []))
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=stdout))
        assert result.passed is True
        assert result.failed_checks == []

    def test_non_zero_exit_does_not_override_the_report(self, settings, fake_binary):
        """The tool exits 2 on fail; the report on stdout is the authority."""
        runner = FakeRunner(stdout=PASS_STDOUT, returncode=2)
        assert preflight.run_preflight(settings, 'watch', runner=runner).passed is True

    def test_runner_receives_the_documented_call(self, settings, fake_binary):
        """The runner is called with capture_output, text, and the timeout."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        preflight.run_preflight(settings, 'watch', runner=runner, timeout_s=12.5)
        assert runner.calls == [{
            'argv': (FAKE_BINARY, '--json'),
            'capture_output': True,
            'timeout': 12.5,
            'text': True,
        }]

    def test_default_timeout_is_the_config_budget(self, settings, fake_binary):
        """The default timeout is defaults.PREFLIGHT_TIMEOUT_S, not a local number."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        preflight.run_preflight(settings, 'watch', runner=runner)
        assert runner.calls[0]['timeout'] == defaults.PREFLIGHT_TIMEOUT_S


class TestBlockingByMode:
    """watch and motion block on a non-PASS verdict; simulate only warns."""

    @pytest.mark.parametrize('mode,blocking', [
        ('watch', True),
        ('motion', True),
        ('simulate', False),
    ])
    def test_blocking_flag_follows_the_mode(self, settings, fake_binary, mode, blocking):
        """The blocking flag records the mode rule, independent of the outcome."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        assert preflight.run_preflight(settings, mode, runner=runner).blocking is blocking

    @pytest.mark.parametrize('mode', ['watch', 'motion'])
    def test_failure_blocks_production_modes(self, settings, fake_binary, mode):
        """A FAIL stops a session that would drive real hardware."""
        result = preflight.run_preflight(settings, mode, runner=FakeRunner(stdout=FAIL_STDOUT))
        assert result.blocks_start() is True

    def test_failure_in_simulate_is_only_a_warning(self, settings, fake_binary):
        """A FAIL in simulate is fully reported but does not stop the start."""
        runner = FakeRunner(stdout=FAIL_STDOUT)
        result = preflight.run_preflight(settings, 'simulate', runner=runner)
        assert result.overall == 'FAIL'
        assert result.passed is False
        assert result.blocking is False
        assert result.blocks_start() is False
        assert len(result.failed_checks) == 2

    def test_error_in_simulate_is_only_a_warning(self, settings, fake_binary):
        """A missing or broken tool does not stop a simulate session either."""
        runner = FakeRunner(raises=FileNotFoundError(2, 'No such file or directory'))
        result = preflight.run_preflight(settings, 'simulate', runner=runner)
        assert result.overall == 'ERROR'
        assert result.blocks_start() is False

    def test_error_blocks_watch(self, settings, fake_binary):
        """A tool that cannot be run is an unproven host: watch is blocked."""
        runner = FakeRunner(raises=FileNotFoundError(2, 'No such file or directory'))
        assert preflight.run_preflight(settings, 'watch', runner=runner).blocks_start() is True

    def test_pass_never_blocks(self, settings, fake_binary):
        """A PASS starts every mode."""
        for mode in ('simulate', 'watch', 'motion'):
            runner = FakeRunner(stdout=PASS_STDOUT)
            assert preflight.run_preflight(settings, mode, runner=runner).blocks_start() is False


class TestBlocksStart:
    """blocks_start blocks production modes on FAIL/ERROR only, never WARN."""

    @pytest.mark.parametrize('overall,blocking,blocks', [
        ('PASS', True, False),
        ('PASS', False, False),
        ('WARN', True, False),   # the tool itself exits 0 on warn
        ('WARN', False, False),
        ('FAIL', True, True),
        ('FAIL', False, False),
        ('ERROR', True, True),   # an unrunnable tool leaves the host unproven
        ('ERROR', False, False),
    ])
    def test_truth_table(self, overall, blocking, blocks):
        """All combinations of overall x blocking."""
        result = PreflightResult(
            overall=overall,
            passed=overall == 'PASS',
            blocking=blocking,
            failed_checks=[],
            ran_at=FIXED_STAMP,
            error=None,
        )
        assert result.blocks_start() is blocks


class TestFrame:
    """frame() is the plan section 6.11 preflight block, and nothing else."""

    def test_shape_on_pass(self, settings, fake_binary):
        """A passing frame carries the four documented keys."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        frame = preflight.run_preflight(settings, 'watch', runner=runner, now=FIXED_NOW).frame()
        assert frame == {
            'ran_at': FIXED_STAMP,
            'overall': 'PASS',
            'blocking': True,
            'failed_checks': [],
        }

    def test_key_set_is_closed(self, settings, fake_binary):
        """No extra keys leak into the frame, on failure or on error."""
        expected = {'ran_at', 'overall', 'blocking', 'failed_checks'}
        for runner in (FakeRunner(stdout=FAIL_STDOUT), FakeRunner(stdout='not json')):
            frame = preflight.run_preflight(settings, 'watch', runner=runner).frame()
            assert set(frame) == expected

    def test_frame_is_json_serializable(self, settings, fake_binary):
        """The frame goes out over SSE, so it must serialize as-is."""
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=FAIL_STDOUT))
        assert json.loads(json.dumps(result.frame())) == result.frame()

    def test_frame_checks_are_copies(self, settings, fake_binary):
        """Mutating a returned frame cannot corrupt the stored result."""
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=FAIL_STDOUT))
        frame = result.frame()
        frame['failed_checks'][0]['summary'] = 'tampered'
        assert result.failed_checks[0]['summary'] == 'active kernel is not proven PREEMPT_RT'

    def test_error_frame_reports_error_with_no_checks(self, settings, fake_binary):
        """An ERROR frame shows the verdict; the reason travels in .error."""
        result = preflight.run_preflight(settings, 'motion', runner=FakeRunner(stdout='{'))
        assert result.frame()['overall'] == 'ERROR'
        assert result.frame()['failed_checks'] == []
        assert result.error


class TestInvocationFailures:
    """Every failure mode ends in an ERROR verdict; nothing propagates out."""

    @pytest.mark.parametrize('stdout', [
        '',
        '   ',
        'not json at all',
        '{"overall": "pass", "checks": [}',
        '<html>a proxy ate the output</html>',
    ])
    def test_malformed_json(self, settings, fake_binary, stdout):
        """Anything that is not JSON is an ERROR, not an exception."""
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=stdout))
        assert result.overall == 'ERROR'
        assert result.passed is False
        assert 'JSON' in result.error or 'output' in result.error

    @pytest.mark.parametrize('stdout', ['null', '[]', '"pass"', '42'])
    def test_json_that_is_not_a_report_object(self, settings, fake_binary, stdout):
        """Valid JSON of the wrong shape is still an ERROR."""
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(stdout=stdout))
        assert result.overall == 'ERROR'

    @pytest.mark.parametrize('report', [
        {'checks': []},
        {'overall': None, 'checks': []},
        {'overall': '', 'checks': []},
        {'overall': 2, 'checks': []},
    ])
    def test_missing_or_unusable_overall(self, settings, fake_binary, report):
        """A report without a usable verdict is an ERROR."""
        runner = FakeRunner(stdout=json.dumps(report))
        result = preflight.run_preflight(settings, 'watch', runner=runner)
        assert result.overall == 'ERROR'
        assert 'overall' in result.error

    @pytest.mark.parametrize('report', [
        {'overall': 'pass'},
        {'overall': 'pass', 'checks': None},
        {'overall': 'pass', 'checks': {'kernel': 'pass'}},
    ])
    def test_missing_or_unusable_checks(self, settings, fake_binary, report):
        """A report without a checks list is an ERROR."""
        runner = FakeRunner(stdout=json.dumps(report))
        result = preflight.run_preflight(settings, 'watch', runner=runner)
        assert result.overall == 'ERROR'
        assert 'checks' in result.error

    @pytest.mark.parametrize('checks', [
        ['kernel is fine'],
        [{'name': 'kernel'}],
        [{'name': 'kernel', 'status': None}],
        [{'name': 'kernel', 'status': ''}],
    ])
    def test_unusable_check_entry(self, settings, fake_binary, checks):
        """A check without a readable status is an ERROR, not a silent pass."""
        runner = FakeRunner(stdout=json.dumps(_report('fail', checks)))
        result = preflight.run_preflight(settings, 'watch', runner=runner)
        assert result.overall == 'ERROR'
        assert result.passed is False

    def test_timeout(self, settings, fake_binary):
        """A hung tool is an ERROR naming the budget it overran."""
        runner = FakeRunner(raises=subprocess.TimeoutExpired(cmd=(FAKE_BINARY,), timeout=30.0))
        result = preflight.run_preflight(settings, 'watch', runner=runner, timeout_s=30.0)
        assert result.overall == 'ERROR'
        assert result.passed is False
        assert '30 s' in result.error

    @pytest.mark.parametrize('failure', [
        FileNotFoundError(2, 'No such file or directory'),
        PermissionError(13, 'Permission denied'),
        OSError(8, 'Exec format error'),
    ])
    def test_os_errors(self, settings, fake_binary, failure):
        """A missing, unexecutable, or broken binary is an ERROR."""
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner(raises=failure))
        assert result.overall == 'ERROR'
        assert type(failure).__name__ in result.error

    def test_unexpected_runner_exception(self, settings, fake_binary):
        """Even an unanticipated failure is caught: the gate never raises."""
        result = preflight.run_preflight(
            settings, 'watch', runner=FakeRunner(raises=RuntimeError('boom')))
        assert result.overall == 'ERROR'
        assert 'RuntimeError' in result.error

    def test_binary_not_found_in_the_index(self, settings, monkeypatch):
        """An uninstalled franka_bringup is an ERROR, not a traceback."""
        def _missing():
            raise PackageNotFoundError('franka_bringup')

        monkeypatch.setattr(preflight, 'preflight_binary', _missing)
        result = preflight.run_preflight(settings, 'watch', runner=FakeRunner())
        assert result.overall == 'ERROR'
        assert 'franka_bringup' in result.error

    def test_result_without_stdout(self, settings, fake_binary):
        """A runner that returns no stdout at all is an ERROR."""
        result = preflight.run_preflight(
            settings, 'watch', runner=lambda *a, **k: subprocess.CompletedProcess((), 0, None))
        assert result.overall == 'ERROR'

    def test_error_results_are_fully_formed(self, settings, fake_binary):
        """An ERROR still carries a timestamp, no checks, and a reason."""
        result = preflight.run_preflight(
            settings, 'watch', runner=FakeRunner(stdout='{'), now=FIXED_NOW)
        assert result.ran_at == FIXED_STAMP
        assert result.failed_checks == []
        assert isinstance(result.error, str) and result.error


class TestTimestamp:
    """ran_at is RFC 3339 UTC with microseconds, stamped when the run starts."""

    def test_injected_clock(self, settings, fake_binary):
        """An injected clock is formatted exactly as documented."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        result = preflight.run_preflight(settings, 'watch', runner=runner, now=lambda: FIXED_NOW)
        assert result.ran_at == FIXED_STAMP

    def test_datetime_instead_of_callable(self, settings, fake_binary):
        """A plain datetime is accepted as well as a callable."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        result = preflight.run_preflight(settings, 'watch', runner=runner, now=FIXED_NOW)
        assert result.ran_at == FIXED_STAMP

    def test_other_zone_is_converted_to_utc(self, settings, fake_binary):
        """A non-UTC aware timestamp is converted, never truncated."""
        moment = FIXED_NOW.astimezone(timezone(timedelta(hours=2)))
        runner = FakeRunner(stdout=PASS_STDOUT)
        result = preflight.run_preflight(settings, 'watch', runner=runner, now=moment)
        assert result.ran_at == FIXED_STAMP

    def test_naive_datetime_is_treated_as_utc(self, settings, fake_binary):
        """A naive datetime is read as UTC rather than as local time."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        result = preflight.run_preflight(
            settings, 'watch', runner=runner, now=FIXED_NOW.replace(tzinfo=None))
        assert result.ran_at == FIXED_STAMP

    def test_default_clock_shape(self, settings, fake_binary):
        """The wall clock produces the same parsable UTC shape."""
        runner = FakeRunner(stdout=PASS_STDOUT)
        ran_at = preflight.run_preflight(settings, 'watch', runner=runner).ran_at
        assert RFC3339_UTC.match(ran_at)
        assert datetime.strptime(ran_at, '%Y-%m-%dT%H:%M:%S.%fZ')

    def test_broken_clock_does_not_break_the_gate(self, settings, fake_binary):
        """A clock that raises falls back to the wall clock, not an exception."""
        def _broken():
            raise RuntimeError('no clock')

        runner = FakeRunner(stdout=PASS_STDOUT)
        result = preflight.run_preflight(settings, 'watch', runner=runner, now=_broken)
        assert result.passed is True
        assert RFC3339_UTC.match(result.ran_at)


class TestAgainstARealChild:
    """The invocation contract holds against a real (plain, non-ROS) process."""

    def test_pass_from_a_real_child(self, settings, tmp_path, monkeypatch):
        """A child printing a PASS report on stdout parses end to end."""
        script = _write_script(
            tmp_path, 'import sys\nsys.stdout.write({!r})\n'.format(PASS_STDOUT))
        monkeypatch.setattr(preflight, 'preflight_binary', lambda: script)
        result = preflight.run_preflight(settings, 'watch')
        assert result.overall == 'PASS'
        assert result.blocks_start() is False

    def test_fail_from_a_real_child(self, settings, tmp_path, monkeypatch):
        """A child exiting 2 with a FAIL report blocks a watch session."""
        script = _write_script(
            tmp_path,
            'import sys\nsys.stdout.write({!r})\nsys.exit(2)\n'.format(FAIL_STDOUT))
        monkeypatch.setattr(preflight, 'preflight_binary', lambda: script)
        result = preflight.run_preflight(settings, 'watch')
        assert result.overall == 'FAIL'
        assert result.blocks_start() is True
        assert result.failed_checks[0]['name'] == 'kernel'

    def test_timeout_of_a_real_child(self, settings, tmp_path, monkeypatch):
        """A child that hangs is killed by the timeout and reported as ERROR."""
        script = _write_script(tmp_path, 'import time\ntime.sleep(30)\n')
        monkeypatch.setattr(preflight, 'preflight_binary', lambda: script)
        result = preflight.run_preflight(settings, 'watch', timeout_s=0.5)
        assert result.overall == 'ERROR'
        assert '0.5 s' in result.error

    def test_missing_real_binary(self, settings, tmp_path, monkeypatch):
        """A path that does not exist surfaces as ERROR, not FileNotFoundError."""
        monkeypatch.setattr(
            preflight, 'preflight_binary', lambda: str(tmp_path / 'not_installed'))
        result = preflight.run_preflight(settings, 'motion')
        assert result.overall == 'ERROR'
        assert 'FileNotFoundError' in result.error
        assert result.blocks_start() is True


