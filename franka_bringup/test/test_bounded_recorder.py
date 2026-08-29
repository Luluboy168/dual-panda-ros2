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
import threading
import time

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
    # ros2_control's introspection/statistics registries are pal_statistics registries: the only
    # real topics are the `<base>/names` (transient-local StatisticsNames) and `<base>/values`
    # (per-cycle StatisticsValues) children, plus a `<base>/full` that repeats the names on every
    # sample. The bare `<base>` names this allowlist carried before 2026-08-28 matched no
    # publisher and silently recorded nothing in every Phase 9 and Phase 10 bag.
    assert recorder.DUAL_ALLOWED_TOPICS == (
        '/controller_manager/activity',
        '/controller_manager/introspection_data/names',
        '/controller_manager/introspection_data/values',
        '/controller_manager/statistics/names',
        '/controller_manager/statistics/values',
        '/diagnostics',
        '/franka/joint_states',
        '/franka_panda1_robot_state_broadcaster/robot_state',
        '/franka_panda2_robot_state_broadcaster/robot_state',
    )
    # ALLOWED_TOPICS is the byte-identical default (dual) set relied on by existing callers.
    assert recorder.ALLOWED_TOPICS == recorder.DUAL_ALLOWED_TOPICS
    argv = recorder.recorder_argv('/proc/self/fd/7/bag')
    assert argv[:3] == ('ros2', 'bag', 'record')
    assert argv[-9:] == recorder.DUAL_ALLOWED_TOPICS
    for forbidden in ('-a', '--all', '--all-topics', '--all-services', '--regex', '--services'):
        assert forbidden not in argv


def test_single_arm_mode_topic_allowlist_is_exact_fixed_and_has_no_broad_selectors():
    # Derived from launch/real/one_arm_franka.launch.py and config/real/one_arm_controllers.yaml,
    # which register the state broadcaster under a fixed, unprefixed instance name (the arm ID is
    # a launch-time argument, not known when that config is written) -- confirmed against a live
    # fake_single_state_only bringup (arm_id:=panda2).
    assert recorder.SINGLE_ALLOWED_TOPICS == (
        '/controller_manager/activity',
        '/controller_manager/introspection_data/names',
        '/controller_manager/introspection_data/values',
        '/controller_manager/statistics/names',
        '/controller_manager/statistics/values',
        '/diagnostics',
        '/franka/joint_states',
        '/franka_robot_state_broadcaster/robot_state',
    )
    argv = recorder.recorder_argv('/proc/self/fd/7/bag', 'single')
    assert argv[:3] == ('ros2', 'bag', 'record')
    assert argv[-8:] == recorder.SINGLE_ALLOWED_TOPICS
    for forbidden in ('-a', '--all', '--all-topics', '--all-services', '--regex', '--services'):
        assert forbidden not in argv


def test_recorder_argv_defaults_to_dual_arm_mode():
    assert recorder.recorder_argv('/proc/self/fd/7/bag') == recorder.recorder_argv(
        '/proc/self/fd/7/bag', 'dual')


def test_recorder_argv_rejects_unknown_arm_mode():
    with pytest.raises(recorder.RecorderError, match='arm mode'):
        recorder.recorder_argv('/proc/self/fd/7/bag', 'both')


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
    # Default dual output must stay byte-identical to the pre-single-arm contract:
    # the arm_mode key appears only for non-default modes (F-9d review finding).
    assert 'arm_mode' not in result
    assert result['topics'] == list(recorder.DUAL_ALLOWED_TOPICS)
    assert result['pass_fds'] == ['<session-fd>']
    assert '/proc/self/fd/<session-fd>/bag' in result['argv']


def test_dry_run_selects_single_arm_mode_topic_set(tmp_path):
    result = recorder.dry_run_plan(tmp_path, 'safe_session', 60, arm_mode='single')
    assert result['arm_mode'] == 'single'
    assert result['topics'] == list(recorder.SINGLE_ALLOWED_TOPICS)
    assert list(tmp_path.iterdir()) == []


def test_dry_run_rejects_unknown_arm_mode_without_creation(tmp_path):
    with pytest.raises(recorder.RecorderError, match='arm mode'):
        recorder.dry_run_plan(tmp_path, 'safe_session', 60, arm_mode='triple')
    assert list(tmp_path.iterdir()) == []


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
    # Default dual output must stay byte-identical to the pre-single-arm contract:
    # the arm_mode key appears only for non-default modes (F-9d review finding).
    assert 'arm_mode' not in result
    assert result['topics'] == list(recorder.DUAL_ALLOWED_TOPICS)
    assert captured['argv'][-9:] == recorder.DUAL_ALLOWED_TOPICS


def test_run_recording_selects_single_arm_mode_topic_set(tmp_path):
    fake = _FakeProcess(0)
    captured = {}

    def factory(argv, **kwargs):
        captured['argv'] = argv
        return fake

    result = recorder.run_recording(
        tmp_path, 'session', 1, process_factory=factory, arm_mode='single')
    assert result['arm_mode'] == 'single'
    assert result['topics'] == list(recorder.SINGLE_ALLOWED_TOPICS)
    assert captured['argv'][-8:] == recorder.SINGLE_ALLOWED_TOPICS


def test_run_recording_rejects_unknown_arm_mode_without_creation(tmp_path):
    with pytest.raises(recorder.RecorderError, match='arm mode'):
        recorder.run_recording(tmp_path, 'session', 1, arm_mode='triple')
    assert list(tmp_path.iterdir()) == []


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


def test_cli_defaults_to_dual_arm_mode(monkeypatch, tmp_path):
    captured = {}

    def fake_run_recording(_output_root, _name, _duration, arm_mode='dual'):
        captured['arm_mode'] = arm_mode
        return {'arm_mode': arm_mode, 'ok': True}

    monkeypatch.setattr(recorder, 'run_recording', fake_run_recording)
    assert recorder.main([
        '--output-root', str(tmp_path), '--name', 'session', '--duration', '1']) == 0
    assert captured['arm_mode'] == 'dual'


def test_cli_passes_through_explicit_single_arm_mode(monkeypatch, tmp_path):
    captured = {}

    def fake_run_recording(_output_root, _name, _duration, arm_mode='dual'):
        captured['arm_mode'] = arm_mode
        return {'arm_mode': arm_mode, 'ok': True}

    monkeypatch.setattr(recorder, 'run_recording', fake_run_recording)
    assert recorder.main([
        '--output-root', str(tmp_path), '--name', 'session', '--duration', '1',
        '--arm-mode', 'single']) == 0
    assert captured['arm_mode'] == 'single'


def test_cli_rejects_unknown_arm_mode_via_argparse(tmp_path, capsys):
    with pytest.raises(SystemExit) as caught:
        recorder.main([
            '--output-root', str(tmp_path), '--name', 'session', '--duration', '1',
            '--arm-mode', 'triple'])
    assert caught.value.code == 2
    assert list(tmp_path.iterdir()) == []


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


# --- Signal-initiated shutdown (F-10i) -----------------------------------------------------
#
# Before this fix SIGTERM skipped the finally: block entirely and ORPHANED the `ros2 bag record`
# child -- three such orphans survived Phase 10 and cross-captured later sessions' traffic into
# already-sealed bags. SIGINT did seal, but reported the sealed recording to the operator as
# exit 3 / "recording operation failed". Both signals now stop the child through the same bounded
# escalation, seal, and exit 0 with "stopped_by" naming the signal.


@pytest.fixture
def stop_signal_guard():
    """
    Make self-signalling safe: harmless handlers that outlive the recorder's own.

    The recorder restores whatever handlers it found, so these are what a late or stray stop
    signal lands on -- never SIG_DFL, which would kill the test session.
    """
    received = []
    previous = {
        number: signal.signal(number, lambda number, _frame: received.append(number))
        for number in recorder.STOP_SIGNALS
    }
    try:
        yield received
    finally:
        for number, handler in previous.items():
            signal.signal(number, signal.SIG_DFL if handler is None else handler)


def _sleeping_child_program():
    return (
        'import signal,time\n'
        'def stopped(_signal,_frame):\n'
        ' raise SystemExit(0)\n'
        'signal.signal(signal.SIGINT,stopped)\n'
        'time.sleep(30)')


def test_stop_signals_are_exactly_sigint_and_sigterm():
    assert recorder.STOP_SIGNALS == (signal.SIGINT, signal.SIGTERM)


@pytest.mark.parametrize('number', recorder.STOP_SIGNALS)
def test_watch_defers_while_unarmed_raises_once_armed_and_restores_handlers(
        stop_signal_guard, number):
    installed_before = {
        candidate: signal.getsignal(candidate) for candidate in recorder.STOP_SIGNALS}
    with recorder._StopSignalWatch() as watch:
        assert watch.installed
        os.kill(os.getpid(), number)
        assert watch.signal_number == number
        assert stop_signal_guard == []
        watch.arm()
        with pytest.raises(recorder._StopSignal) as caught:
            os.kill(os.getpid(), number)
        assert caught.value.signal_number == number
        # One raise only: a second signal during cleanup must be deferred, not raised.
        os.kill(os.getpid(), number)
    assert {candidate: signal.getsignal(candidate)
            for candidate in recorder.STOP_SIGNALS} == installed_before


def test_watch_degrades_to_a_noop_off_the_main_thread():
    observed = {}

    def probe():
        with recorder._StopSignalWatch() as watch:
            watch.arm()
            observed['installed'] = watch.installed
            observed['raising'] = watch._raising

    thread = threading.Thread(target=probe)
    thread.start()
    thread.join(timeout=10)
    assert not thread.is_alive()
    assert observed == {'installed': False, 'raising': False}


def test_stop_signal_labels_are_lowercase_signal_names():
    assert recorder._stop_signal_label(signal.SIGTERM) == 'sigterm'
    assert recorder._stop_signal_label(signal.SIGINT) == 'sigint'


def test_signal_delivered_before_arming_stops_the_child_without_waiting_out_the_duration(
        tmp_path, stop_signal_guard):
    fake = _FakeProcess(0)

    def factory(_argv, **_kwargs):
        os.kill(os.getpid(), signal.SIGTERM)
        return fake

    result = recorder.run_recording(
        tmp_path, 'early-signal', 3600, process_factory=factory)
    assert result['stopped_by'] == 'sigterm'
    assert result['ok']
    assert fake.signals == [signal.SIGINT]
    # The 3600 s duration wait was never entered.
    assert fake.wait_timeouts == [recorder.SIGINT_FLUSH_TIMEOUT_SECONDS]


@pytest.mark.parametrize('number', recorder.STOP_SIGNALS)
def test_real_child_is_sealed_reaped_and_reported_when_a_stop_signal_arrives(
        tmp_path, stop_signal_guard, number):
    captured = {}

    def factory(_argv, **kwargs):
        captured['descriptor'] = kwargs['pass_fds'][0]
        process = subprocess.Popen(
            [sys.executable, '-c', _sleeping_child_program()], shell=False,
            pass_fds=kwargs['pass_fds'])
        captured['process'] = process
        return process

    timer = threading.Timer(0.5, os.kill, args=(os.getpid(), number))
    timer.start()
    try:
        result = recorder.run_recording(
            tmp_path, 'signalled-child', 3600, process_factory=factory)
    finally:
        timer.cancel()
    process = captured['process']
    assert result['ok']
    assert result['stopped_by'] == _EXPECTED_LABELS[number]
    # The child was stopped through the ordinary bounded escalation, flushing SIGINT first.
    assert result['shutdown'] == 'sigint'
    assert process.poll() == 0
    with pytest.raises(ChildProcessError):
        os.waitpid(process.pid, os.WNOHANG)
    with pytest.raises(OSError) as closed:
        os.fstat(captured['descriptor'])
    assert closed.value.errno == errno.EBADF


_EXPECTED_LABELS = {signal.SIGINT: 'sigint', signal.SIGTERM: 'sigterm'}


def test_a_stop_signal_arriving_during_cleanup_is_deferred_and_still_reaps_the_child(
        tmp_path, stop_signal_guard):
    class SignallingDuringCleanupProcess(_FakeProcess):
        def send_signal(self, candidate):
            super().send_signal(candidate)
            os.kill(os.getpid(), signal.SIGTERM)

    fake = SignallingDuringCleanupProcess(1)
    result = recorder.run_recording(
        tmp_path, 'signal-in-cleanup', 1, process_factory=lambda *_a, **_k: fake)
    # The recording ended on its duration; the signal arrived afterwards, so it is not reported
    # as the cause. What matters is that cleanup completed rather than being cut short.
    assert 'stopped_by' not in result
    assert result['shutdown'] == 'sigint'
    assert fake.signals == [signal.SIGINT]


def test_child_is_started_with_a_parent_death_preexec_hook(tmp_path):
    captured = {}

    def factory(_argv, **kwargs):
        captured.update(kwargs)
        return _FakeProcess(0)

    recorder.run_recording(tmp_path, 'preexec', 1, process_factory=factory)
    hook = captured['preexec_fn']
    assert callable(hook)
    assert hook.func is recorder._set_parent_death_signal
    assert hook.args == (os.getpid(),)


def test_parent_death_setup_requests_pdeathsig_and_keeps_a_live_parent():
    calls = []
    recorder._set_parent_death_signal(os.getppid(), prctl=lambda *args: calls.append(args))
    assert calls == [(recorder._PR_SET_PDEATHSIG, int(signal.SIGKILL), 0, 0, 0)]


def test_parent_death_setup_exits_when_the_parent_already_died(monkeypatch):
    class _Exited(BaseException):
        pass

    def fake_exit(code):
        raise _Exited(code)

    monkeypatch.setattr(recorder.os, '_exit', fake_exit)
    with pytest.raises(_Exited) as caught:
        recorder._set_parent_death_signal(os.getpid() + 1, prctl=lambda *_args: 0)
    assert caught.value.args == (recorder._ORPHANED_CHILD_EXIT_STATUS,)


def test_prctl_is_resolved_in_the_parent_before_any_fork():
    assert recorder._PRCTL is not None
    assert recorder._PRCTL(recorder._PR_SET_PDEATHSIG, 0, 0, 0, 0) == 0


_SIGNAL_DRIVER = '''
import functools
import subprocess
import sys

from franka_bringup import recorder

BAG_STUB = """
import os, signal, sys, time
bag = sys.argv[1]
os.makedirs(bag)
with open(os.path.join(bag, 'bag_0.mcap'), 'w', encoding='utf-8') as handle:
    handle.write('recorded-data')
def seal(_signal, _frame):
    with open(os.path.join(bag, 'metadata.yaml'), 'w', encoding='utf-8') as handle:
        handle.write('rosbag2_bagfile_information:\\\\n  version: 9\\\\n')
    raise SystemExit(0)
signal.signal(signal.SIGINT, seal)
with open(os.path.join(os.path.dirname(bag), 'ready'), 'w', encoding='utf-8') as handle:
    handle.write('ready')
time.sleep(300)
"""


def factory(argv, **kwargs):
    bag_path = argv[argv.index('--output') + 1]
    process = subprocess.Popen(
        [sys.executable, '-c', BAG_STUB, bag_path], shell=False,
        pass_fds=kwargs['pass_fds'], preexec_fn=kwargs['preexec_fn'],
        start_new_session=kwargs.get('start_new_session', False),
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    sys.stderr.write('child-pid {}\\n'.format(process.pid))
    sys.stderr.flush()
    return process


recorder.run_recording = functools.partial(recorder.run_recording, process_factory=factory)
raise SystemExit(recorder.main(sys.argv[1:]))
'''


def _run_signal_driver(
        tmp_path, duration, stop_signal=None, name='sealed', delivery='process'):
    """
    Drive franka_record end to end in a real process and stop it with a real signal.

    The stub child gets DEVNULL stdio and reports readiness through a file rather than a pipe:
    an orphaned child holding this harness's pipes open would otherwise turn a regression into a
    hang instead of a failure -- which is exactly how it behaved before the fix.

    ``delivery`` selects who the signal goes to. ``'process'`` signals the driver's PID, the way a
    script or a service manager stops it. ``'group'`` signals the driver's whole PROCESS GROUP,
    which is what a terminal Ctrl-C does; the driver is always started in its own session so that
    a group signal here can never escape into pytest's own group. ``'kill'`` SIGKILLs the driver,
    the one stop no handler can intercept.
    """
    session = tmp_path / name
    driver = subprocess.Popen(
        [sys.executable, '-c', _SIGNAL_DRIVER,
         '--output-root', str(tmp_path), '--name', name, '--duration', str(duration)],
        shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        start_new_session=True)
    try:
        child_pid = int(driver.stderr.readline().split()[1])
        deadline = time.monotonic() + 30
        while not (session / 'ready').is_file() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert (session / 'ready').is_file(), 'stub child never signalled readiness'
        if delivery == 'group':
            os.killpg(os.getpgid(driver.pid), stop_signal)
        elif delivery == 'kill':
            driver.kill()
        elif stop_signal is not None:
            driver.send_signal(stop_signal)
        stdout, _stderr = driver.communicate(timeout=60)
    finally:
        if driver.poll() is None:
            driver.kill()
            driver.communicate(timeout=30)
    return driver.returncode, stdout, child_pid


def _wait_until_process_is_gone(pid, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _process_is_live(pid):
            return True
        time.sleep(0.02)
    return not _process_is_live(pid)


def _process_is_live(pid):
    try:
        os.readlink('/proc/{}/exe'.format(pid))
    except (FileNotFoundError, ProcessLookupError):
        return False
    except PermissionError:
        return True
    return True


def test_real_subprocess_sigterm_seals_the_bag_exits_zero_and_leaves_no_orphan(tmp_path):
    return_code, stdout, child_pid = _run_signal_driver(
        tmp_path, 3600, stop_signal=signal.SIGTERM)
    assert return_code == 0
    result = json.loads(stdout)
    assert result['ok'] is True
    assert result['stopped_by'] == 'sigterm'
    assert result['shutdown'] == 'sigint'
    # Sealed: the child got its flush signal and wrote the bag's metadata before exiting.
    assert (tmp_path / 'sealed' / 'bag' / 'metadata.yaml').is_file()
    # No orphan: /proc, not pgrep -- `ros2 bag record` truncates past pgrep -x's 15-char limit.
    assert not _process_is_live(child_pid)


def test_real_subprocess_sigint_seals_the_bag_and_also_exits_zero(tmp_path):
    return_code, stdout, child_pid = _run_signal_driver(
        tmp_path, 3600, stop_signal=signal.SIGINT, name='interrupted')
    assert return_code == 0
    result = json.loads(stdout)
    assert result['stopped_by'] == 'sigint'
    assert (tmp_path / 'interrupted' / 'bag' / 'metadata.yaml').is_file()
    assert not _process_is_live(child_pid)


def test_the_bag_recorder_child_is_started_in_its_own_session(tmp_path):
    """
    Pin the own-session contract for the bag-record child.

    A terminal Ctrl-C signals the whole foreground PROCESS GROUP. While the ``ros2 bag record``
    child shared this process's group it received that SIGINT directly and exited with the ros2
    CLI's KeyboardInterrupt status 2, which run_recording reported as a failed recording on a
    fully sealed bag (tools-verify finding 8). start_new_session puts the child in its own
    session, so only franka_record sees the terminal's signal and the ordinary bounded
    stop-and-reap seals the bag.
    """
    captured = {}

    def factory(_argv, **kwargs):
        captured.update(kwargs)
        return _FakeProcess(0)

    recorder.run_recording(tmp_path, 'session', 1, process_factory=factory)
    assert captured['start_new_session'] is True
    # The parent-death guard must survive the session change: CPython's child_exec runs setsid()
    # before preexec_fn, so PR_SET_PDEATHSIG is armed after it.
    assert captured['preexec_fn'].func is recorder._set_parent_death_signal


def test_a_process_group_sigint_seals_the_bag_exits_zero_and_leaves_no_orphan(tmp_path):
    """
    Stop the recorder the way an operator does, with a process-group SIGINT.

    Before start_new_session this exited 2 with {"error":"recording request failed"} on a sealed
    bag; the existing SIGINT test never caught it because it signals only the parent PID.
    """
    return_code, stdout, child_pid = _run_signal_driver(
        tmp_path, 3600, stop_signal=signal.SIGINT, name='ctrlc', delivery='group')
    assert return_code == 0, stdout
    result = json.loads(stdout)
    assert result['ok'] is True
    assert result['stopped_by'] == 'sigint'
    assert result['shutdown'] == 'sigint'
    assert (tmp_path / 'ctrlc' / 'bag' / 'metadata.yaml').is_file()
    assert not _process_is_live(child_pid)


def test_sigkill_of_the_recorder_leaves_no_orphan_and_an_unsealed_bag(tmp_path):
    """
    Pin the documented SIGKILL contract (tools-verify findings 9 and 11).

    PR_SET_PDEATHSIG covers the ORPHAN half only: no recorder survives, but nothing seals the bag,
    so the directory holds its data file with no metadata.yaml and needs ``ros2 bag reindex``.
    Deleting PR_SET_PDEATHSIG was previously caught by a single assertion on a module constant;
    this exercises the behaviour end to end in real processes.
    """
    return_code, _stdout, child_pid = _run_signal_driver(
        tmp_path, 3600, name='killed', delivery='kill')
    assert return_code == -signal.SIGKILL
    assert _wait_until_process_is_gone(child_pid), (
        'the ros2 bag record child outlived a SIGKILLed recorder: PR_SET_PDEATHSIG is not armed')
    bag = tmp_path / 'killed' / 'bag'
    assert (bag / 'bag_0.mcap').is_file()
    assert not (bag / 'metadata.yaml').exists(), (
        'a SIGKILLed recorder cannot seal; if this ever passes the docstring contract is wrong')


def test_real_subprocess_duration_expiry_keeps_its_previous_zero_exit_and_key_set(tmp_path):
    return_code, stdout, child_pid = _run_signal_driver(tmp_path, 1, name='expired')
    assert return_code == 0
    result = json.loads(stdout)
    assert 'stopped_by' not in result
    assert result['duration_seconds'] == 1
    assert (tmp_path / 'expired' / 'bag' / 'metadata.yaml').is_file()
    assert not _process_is_live(child_pid)


@pytest.mark.parametrize(('timeout_count', 'expected_stopped_by', 'expected_shutdown'), (
    (0, None, 'none'),
    (1, 'sigterm', 'sigint'),
))
def test_a_stop_signal_landing_while_the_watch_disarms_does_not_escape(
        tmp_path, monkeypatch, timeout_count, expected_stopped_by, expected_shutdown):
    """
    Close the last window: the handler can fire between the wait returning and disarm().

    Signal handlers run between bytecodes, so a stop signal delivered in the few instructions
    between ``process.wait`` returning and ``watch.disarm()`` raises out of that finally: clause,
    past every except: that would otherwise have caught it. Simulated deterministically by making
    disarm() itself raise. The child must still be reaped and the signal must not escape as an
    unhandled BaseException.
    """
    monkeypatch.setattr(
        recorder._StopSignalWatch, 'disarm',
        lambda _self: (_ for _ in ()).throw(recorder._StopSignal(signal.SIGTERM)))
    fake = _FakeProcess(timeout_count)
    result = recorder.run_recording(
        tmp_path, 'disarm-race', 1, process_factory=lambda *_a, **_k: fake)
    assert result['ok']
    assert result['shutdown'] == expected_shutdown
    assert result.get('stopped_by') == expected_stopped_by
    # Reaped either way: the duration-expiry case needs no escalation, the still-running case
    # went through the ordinary bounded stop.
    assert fake.signals == ([] if timeout_count == 0 else [signal.SIGINT])
