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

import errno
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys

from franka_bringup import recorder
import pytest


class _FakeProcess:
    def __init__(self, timeout_count):
        self.timeout_count = timeout_count
        self.wait_timeouts = []
        self.signals = []

    def wait(self, timeout):
        self.wait_timeouts.append(timeout)
        if self.timeout_count > 0:
            self.timeout_count -= 1
            raise subprocess.TimeoutExpired('ros2', timeout)
        return 0

    def send_signal(self, candidate):
        self.signals.append(candidate)

    def terminate(self):
        self.signals.append('terminate')

    def kill(self):
        self.signals.append('kill')


class _InjectedFailureProcess:
    def __init__(self, failure_operation, injected_error, exit_stage):
        self.failure_operation = failure_operation
        self.injected_error = injected_error
        self.exit_stage = exit_stage
        self.failure_delivered = False
        self.last_stage = 'initial'
        self.live = True
        self.reaped = False
        self.wait_timeouts = []
        self.signals = []

    def _inject(self, operation):
        if operation == self.failure_operation and not self.failure_delivered:
            self.failure_delivered = True
            raise self.injected_error

    def wait(self, timeout):
        self.wait_timeouts.append(timeout)
        self._inject(self.last_stage + '_wait')
        if self.live:
            raise subprocess.TimeoutExpired('ros2', timeout)
        self.reaped = True
        return 0

    def send_signal(self, candidate):
        self.last_stage = 'sigint'
        self.signals.append(candidate)
        self._inject('sigint_signal')
        if self.exit_stage == 'sigint':
            self.live = False

    def terminate(self):
        self.last_stage = 'terminate'
        self.signals.append('terminate')
        self._inject('terminate_signal')
        if self.exit_stage == 'terminate':
            self.live = False

    def kill(self):
        self.last_stage = 'kill'
        self.signals.append('kill')
        self._inject('kill_signal')
        if self.exit_stage == 'kill':
            self.live = False


def test_topic_allowlist_is_exact_fixed_and_has_no_broad_selectors():
    assert recorder.ALLOWED_TOPICS == (
        '/controller_manager/activity',
        '/controller_manager/introspection_data',
        '/controller_manager/statistics',
        '/diagnostics',
        '/franka/joint_states',
        '/franka_panda1_robot_state_broadcaster/robot_state',
        '/franka_panda2_robot_state_broadcaster/robot_state',
    )
    argv = recorder.recorder_argv('/proc/self/fd/7/bag')
    assert argv[:3] == ('ros2', 'bag', 'record')
    assert argv[-7:] == recorder.ALLOWED_TOPICS
    for forbidden in ('-a', '--all', '--all-topics', '--all-services', '--regex', '--services'):
        assert forbidden not in argv


@pytest.mark.parametrize('name', [
    '', '.', '..', '../escape', 'nested/name', 'white space', '--all', 'x;touch', 'x$(id)',
    'x' * 65,
])
def test_rejects_name_and_argv_injection(name, tmp_path):
    with pytest.raises(recorder.RecorderError):
        recorder.dry_run_plan(tmp_path, name, 1)
    assert list(tmp_path.iterdir()) == []


def test_dry_run_validates_root_but_creates_nothing(tmp_path):
    before = list(tmp_path.iterdir())
    result = recorder.dry_run_plan(tmp_path, 'safe_session', 60)
    assert list(tmp_path.iterdir()) == before
    assert result['dry_run']
    assert result['pass_fds'] == ['<session-fd>']
    assert '/proc/self/fd/<session-fd>/bag' in result['argv']


def test_component_wise_walk_rejects_parent_symlink_even_when_target_is_valid(tmp_path):
    real_root = tmp_path / 'real-root'
    real_root.mkdir()
    link = tmp_path / 'linked-root'
    link.symlink_to(real_root, target_is_directory=True)
    with pytest.raises(recorder.RecorderError, match='symlink'):
        recorder.dry_run_plan(link, 'session', 1)
    assert list(real_root.iterdir()) == []


def test_requires_existing_normalized_absolute_owned_directory(tmp_path, monkeypatch):
    with pytest.raises(recorder.RecorderError):
        recorder.dry_run_plan(Path('relative'), 'session', 1)
    with pytest.raises(recorder.RecorderError):
        recorder.dry_run_plan(tmp_path / 'missing', 'session', 1)
    os.chmod(tmp_path, 0o750)
    with pytest.raises(recorder.RecorderError, match='private'):
        recorder.dry_run_plan(tmp_path, 'session', 1)
    os.chmod(tmp_path, 0o700)
    actual_uid = os.geteuid()
    monkeypatch.setattr(recorder.os, 'geteuid', lambda: actual_uid + 1)
    with pytest.raises(recorder.RecorderError, match='owned'):
        recorder.dry_run_plan(tmp_path, 'session', 1)


def test_session_is_mode_0700_nonexisting_and_pinned_across_rename(tmp_path):
    session = recorder.create_pinned_session(tmp_path, 'session')
    try:
        original = os.fstat(session.descriptor)
        assert stat.S_IMODE(original.st_mode) == 0o700
        moved = tmp_path / 'renamed-after-open'
        os.rename(tmp_path / 'session', moved)
        assert (os.fstat(session.descriptor).st_dev, os.fstat(session.descriptor).st_ino) == (
            original.st_dev, original.st_ino)
        marker = os.open(
            'marker', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600,
            dir_fd=session.descriptor)
        os.close(marker)
        assert (moved / 'marker').is_file()
        assert session.bag_path.startswith('/proc/self/fd/')
    finally:
        session.close()


def test_existing_session_is_rejected_without_overwrite(tmp_path):
    existing = tmp_path / 'existing'
    existing.mkdir()
    marker = existing / 'marker'
    marker.write_text('preserve', encoding='utf-8')
    with pytest.raises(recorder.RecorderError, match='must not already exist'):
        recorder.create_pinned_session(tmp_path, 'existing')
    assert marker.read_text(encoding='utf-8') == 'preserve'


@pytest.mark.parametrize('timeouts,expected_signals,expected_shutdown', [
    (1, [signal.SIGINT], 'sigint'),
    (2, [signal.SIGINT, 'terminate'], 'terminate'),
    (3, [signal.SIGINT, 'terminate', 'kill'], 'kill'),
])
def test_bounded_flush_escalation_and_exact_subprocess_contract(
        tmp_path, timeouts, expected_signals, expected_shutdown):
    fake = _FakeProcess(timeouts)
    captured = {}

    def factory(argv, **kwargs):
        captured['argv'] = argv
        captured['kwargs'] = kwargs
        return fake

    result = recorder.run_recording(tmp_path, 'session', 3, process_factory=factory)
    assert result['shutdown'] == expected_shutdown
    assert fake.signals == expected_signals
    expected_timeouts = [3]
    if timeouts >= 1:
        expected_timeouts.append(recorder.SIGINT_FLUSH_TIMEOUT_SECONDS)
    if timeouts >= 2:
        expected_timeouts.append(recorder.TERMINATE_TIMEOUT_SECONDS)
    if timeouts >= 3:
        expected_timeouts.append(recorder.KILL_TIMEOUT_SECONDS)
    assert fake.wait_timeouts == expected_timeouts
    assert captured['kwargs']['shell'] is False
    assert captured['kwargs']['pass_fds']
    descriptor = captured['kwargs']['pass_fds'][0]
    assert '/proc/self/fd/{}/bag'.format(descriptor) in captured['argv']
    assert '--topics' in captured['argv']


def test_final_kill_wait_is_bounded_and_reported(tmp_path):
    fake = _FakeProcess(5)
    with pytest.raises(recorder.RecorderError, match='final bounded kill'):
        recorder.run_recording(
            tmp_path, 'session', 1, process_factory=lambda *args, **kwargs: fake)
    assert fake.wait_timeouts == [
        1, recorder.SIGINT_FLUSH_TIMEOUT_SECONDS, recorder.TERMINATE_TIMEOUT_SECONDS,
        recorder.KILL_TIMEOUT_SECONDS, recorder.KILL_TIMEOUT_SECONDS]
    assert fake.signals == [signal.SIGINT, 'terminate', 'kill', 'kill']


@pytest.mark.parametrize(('failure_operation', 'exit_stage', 'exception_type'), (
    ('initial_wait', 'sigint', KeyboardInterrupt),
    ('sigint_signal', 'terminate', OSError),
    ('sigint_wait', 'sigint', RuntimeError),
    ('terminate_signal', 'kill', OSError),
    ('terminate_wait', 'terminate', RuntimeError),
    ('kill_signal', 'kill', OSError),
    ('kill_wait', 'kill', RuntimeError),
))
def test_every_post_start_wait_or_signal_failure_still_reaps_and_preserves_original(
        tmp_path, failure_operation, exit_stage, exception_type):
    injected_error = exception_type('injected {}'.format(failure_operation))
    fake = _InjectedFailureProcess(failure_operation, injected_error, exit_stage)
    passed_descriptor = []

    def factory(_argv, **kwargs):
        passed_descriptor.extend(kwargs['pass_fds'])
        return fake

    with pytest.raises(exception_type) as caught:
        recorder.run_recording(tmp_path, 'session', 1, process_factory=factory)
    assert caught.value is injected_error
    assert fake.failure_delivered
    assert fake.reaped
    assert not fake.live
    assert len(passed_descriptor) == 1
    with pytest.raises(OSError) as closed:
        os.fstat(passed_descriptor[0])
    assert closed.value.errno == errno.EBADF


def test_cleanup_error_is_not_allowed_to_replace_initial_wait_exception(tmp_path):
    initial_error = RuntimeError('initial wait failed')
    cleanup_error = OSError('SIGINT failed')

    class DoublyFailingProcess(_InjectedFailureProcess):
        def wait(self, timeout):
            if self.last_stage == 'initial':
                self.last_stage = 'initial-complete'
                raise initial_error
            return super().wait(timeout)

        def send_signal(self, candidate):
            self.last_stage = 'sigint'
            self.signals.append(candidate)
            raise cleanup_error

    fake = DoublyFailingProcess('unused', RuntimeError('unused'), 'terminate')
    with pytest.raises(RuntimeError) as caught:
        recorder.run_recording(
            tmp_path, 'session', 1, process_factory=lambda *_args, **_kwargs: fake)
    assert caught.value is initial_error
    assert fake.reaped
    assert not fake.live
    assert any('SIGINT failed' in note for note in getattr(initial_error, '__notes__', ()))


def test_real_local_child_times_out_handles_sigint_and_is_reaped(tmp_path):
    captured = {}
    program = (
        'import signal,time\n'
        'def stopped(_signal,_frame):\n'
        ' raise SystemExit(0)\n'
        'signal.signal(signal.SIGINT,stopped)\n'
        'time.sleep(30)')

    def factory(_argv, **kwargs):
        captured['descriptor'] = kwargs['pass_fds'][0]
        process = subprocess.Popen(
            [sys.executable, '-c', program], shell=False,
            pass_fds=kwargs['pass_fds'])
        captured['process'] = process
        return process

    result = recorder.run_recording(tmp_path, 'real-child', 1, process_factory=factory)
    process = captured['process']
    assert result['shutdown'] == 'sigint'
    assert process.poll() == 0
    with pytest.raises(ChildProcessError):
        os.waitpid(process.pid, os.WNOHANG)
    with pytest.raises(OSError) as closed:
        os.fstat(captured['descriptor'])
    assert closed.value.errno == errno.EBADF


def test_real_local_child_is_reaped_when_initial_wait_is_interrupted(tmp_path):
    injected = KeyboardInterrupt('injected private /tmp/operator-secret')
    captured = {}
    program = (
        'import signal,time\n'
        'def stopped(_signal,_frame):\n'
        ' raise SystemExit(0)\n'
        'signal.signal(signal.SIGINT,stopped)\n'
        'time.sleep(30)')

    class InterruptedProcess:
        def __init__(self, process):
            self.process = process
            self.first_wait = True

        def wait(self, timeout):
            if self.first_wait:
                self.first_wait = False
                raise injected
            return self.process.wait(timeout=timeout)

        def send_signal(self, candidate):
            self.process.send_signal(candidate)

        def terminate(self):
            self.process.terminate()

        def kill(self):
            self.process.kill()

    def factory(_argv, **kwargs):
        captured['descriptor'] = kwargs['pass_fds'][0]
        child = subprocess.Popen(
            [sys.executable, '-c', program], shell=False,
            pass_fds=kwargs['pass_fds'])
        captured['process'] = child
        return InterruptedProcess(child)

    with pytest.raises(KeyboardInterrupt) as caught:
        recorder.run_recording(tmp_path, 'interrupted-child', 1, process_factory=factory)
    assert caught.value is injected
    process = captured['process']
    assert process.poll() in (0, -signal.SIGINT)
    with pytest.raises(ChildProcessError):
        os.waitpid(process.pid, os.WNOHANG)
    with pytest.raises(OSError) as closed:
        os.fstat(captured['descriptor'])
    assert closed.value.errno == errno.EBADF


@pytest.mark.parametrize(('error', 'expected_code', 'expected_message'), (
    (KeyboardInterrupt('private /tmp/wait'), 3, 'recording operation failed'),
    (RuntimeError('private /tmp/unexpected-wait'), 3, 'recording operation failed'),
    (OSError('private /tmp/cleanup'), 3, 'recording operation failed'),
    (recorder.RecorderError('private /tmp/request'), 2, 'recording request failed'),
))
def test_cli_sanitizes_api_failures_without_traceback_or_detail(
        monkeypatch, capsys, tmp_path, error, expected_code, expected_message):
    def fail(*_args, **_kwargs):
        raise error

    monkeypatch.setattr(recorder, 'run_recording', fail)
    assert recorder.main([
        '--output-root', str(tmp_path), '--name', 'session', '--duration', '1']) == expected_code
    captured = capsys.readouterr()
    assert captured.out == ''
    assert json.loads(captured.err) == {'error': expected_message, 'ok': False}
    assert 'Traceback' not in captured.err
    assert '/tmp/' not in captured.err


def test_cli_does_not_swallow_system_exit(monkeypatch, tmp_path):
    def stop(*_args, **_kwargs):
        raise SystemExit(19)

    monkeypatch.setattr(recorder, 'run_recording', stop)
    with pytest.raises(SystemExit) as caught:
        recorder.main([
            '--output-root', str(tmp_path), '--name', 'session', '--duration', '1'])
    assert caught.value.code == 19


@pytest.mark.parametrize('duration', [0, 3601])
def test_cli_rejects_out_of_range_duration_without_creation(duration, tmp_path):
    assert recorder.main([
        '--output-root', str(tmp_path), '--name', 'session', '--duration', str(duration),
        '--dry-run']) == 2
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('duration', [False, 1.0, 0, 3601])
def test_direct_api_rejects_invalid_duration_before_creation(duration, tmp_path):
    with pytest.raises(recorder.RecorderError, match='duration'):
        recorder.dry_run_plan(tmp_path, 'session', duration)
    with pytest.raises(recorder.RecorderError, match='duration'):
        recorder.run_recording(tmp_path, 'session', duration)
    assert list(tmp_path.iterdir()) == []
