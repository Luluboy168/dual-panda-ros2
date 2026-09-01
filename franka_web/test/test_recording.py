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
Tests for franka_web.recording: naming, argv, segment chaining and shutdown.

Nothing here executes a process. Every child is a scripted :class:`FakeChild`
driven by :class:`support.fake_clock.FakeClock`, so the 30 s SIGINT budget and
the hour-long segment rollover are exercised in microseconds and deterministically.

The two audit corrections have tests of their own:

* D20 -- the argv must be the installed binary, never ``ros2 run``;
* D19 -- SIGINT must come first and must be given at least as long as
  ``franka_bringup.recorder``'s own worst-case inner ladder before escalating.

Two more v2 behaviours are covered here: ``recording.enabled: false`` turns
the whole chain off (nothing spawned, ``disabled: true`` in the frame), and
every line the recorder child prints reaches the log bus, which is the only
way the recorder appears in the operator's log drawer.
"""

from datetime import datetime, timedelta, timezone
import os
import re
import signal

from franka_bringup import recorder as bringup_recorder
from franka_web import defaults, recording
from franka_web.logbus import LogBus
from franka_web.launcher import LauncherError
from franka_web.recording import (
    build_argv,
    recorder_binary,
    RecordingError,
    RecordingSupervisor,
    segment_name,
    session_name,
    topics_for,
)
import pytest
from support.fake_clock import FakeClock

# The name gate as the recorder's docstring documents it, written out here so a
# silent change to the imported object is caught rather than mirrored.
DOCUMENTED_NAME_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')
BASE_NAME = 'web-20260830-141501'


class FakeChild:
    """
    A scripted stand-in for one ``franka_record`` child process.

    ``exits_after`` models ``--duration`` elapsing on its own, ``seals_on`` lists
    the signals this child actually honours, and ``seal_delay`` is how long it
    then spends sealing the bag. Waits advance the injected clock, so a test that
    drives the stop ladder ends with the fake time the ladder really consumed.
    """

    def __init__(self, clock, pid, exits_after=None, returncode=0, tail='',
                 seals_on=(signal.SIGINT, signal.SIGTERM, signal.SIGKILL),
                 seal_delay=0.0, on_line=None):
        """Create a child that is running at the clock's current time."""
        self.clock = clock
        self.pid = pid
        self.on_line = on_line
        self.signals = []
        self.waits = []
        self.tail = tail
        self._returncode = returncode
        self._seals_on = tuple(seals_on)
        self._seal_delay = float(seal_delay)
        self._exit_at = None if exits_after is None else clock.monotonic() + float(exits_after)
        self._exited = False

    def _settle(self):
        """Mark the child exited once the clock has reached its exit time."""
        if self._exited or self._exit_at is None:
            return
        if self.clock.monotonic() >= self._exit_at:
            self._exited = True

    def alive(self):
        """Return whether the child is still running."""
        self._settle()
        return not self._exited

    def returncode(self):
        """Return the exit status, or None while the child runs."""
        self._settle()
        return self._returncode if self._exited else None

    def send_signal(self, number):
        """Record a delivered signal and arm the seal if this child honours it."""
        self.signals.append(number)
        self._settle()
        if self._exited or number not in self._seals_on:
            return
        self._exit_at = self.clock.monotonic() + self._seal_delay
        self._settle()

    def wait_exited(self, timeout):
        """Record the budget, advance the clock over it, and report the outcome."""
        self.waits.append(timeout)
        self._settle()
        if self._exited:
            return True
        now = self.clock.monotonic()
        if self._exit_at is not None and self._exit_at <= now + timeout:
            self.clock.advance(self._exit_at - now)
        else:
            self.clock.advance(timeout)
        self._settle()
        return self._exited

    def stop(self, sigint_wait, sigterm_wait, sigkill_wait):
        """Mirror ChildProcess.stop, including its complete-ladder failure."""
        if not self.alive():
            return 'already-exited'
        for step, number, budget in (
                ('sigint', signal.SIGINT, sigint_wait),
                ('sigterm', signal.SIGTERM, sigterm_wait),
                ('sigkill', signal.SIGKILL, sigkill_wait)):
            self.send_signal(number)
            if self.wait_exited(budget) or not self.alive():
                return step
        raise LauncherError('scripted recorder process group survived SIGKILL')

    def output_tail(self):
        """Return the child's short diagnostic tail."""
        return self.tail

    def emit(self, text):
        """Simulate the child printing one line to its merged output."""
        if self.on_line is not None:
            self.on_line(text)


class FakeSpawner:
    """A ``spawn`` callable handing out scripted children, newest specification last."""

    def __init__(self, clock, *specs):
        """Script one child per specification; the last one repeats forever."""
        self.clock = clock
        self.calls = []
        self.children = []
        self._specs = list(specs)

    def __call__(self, argv, env, name, on_line=None):
        """
        Record the spawn request and return the next scripted child.

        ``on_line`` is accepted and threaded into the child: the supervisor
        passes the log bus's sink here, and a fake that could not take the
        keyword would be the one thing able to break that wiring silently.
        """
        index = len(self.children)
        spec = self._specs[min(index, len(self._specs) - 1)] if self._specs else {}
        child = FakeChild(self.clock, 4200 + index, on_line=on_line, **spec)
        self.calls.append((tuple(argv), env, name))
        self.children.append(child)
        return child

    @property
    def names(self):
        """Return the segment name of every spawn, in order."""
        return [call[2] for call in self.calls]


class _Settings:
    """The two fields the recording supervisor reads."""

    def __init__(self, recording_root, recording_enabled=True):
        """Bind the recording root and the on/off policy."""
        self.recording_root = recording_root
        self.recording_enabled = recording_enabled


@pytest.fixture()
def settings(tmp_path):
    """Return settings whose recording root is a private directory."""
    root = tmp_path / 'recordings'
    root.mkdir(mode=0o700)
    return _Settings(str(root))


@pytest.fixture()
def disabled_settings(tmp_path):
    """Return settings with session recording turned off by policy."""
    root = tmp_path / 'recordings'
    root.mkdir(mode=0o700, exist_ok=True)
    return _Settings(str(root), recording_enabled=False)


@pytest.fixture()
def clock():
    """Return a deterministic monotonic clock for the supervisor and its children."""
    return FakeClock()


def _started(settings, clock, *specs, arm_mode='dual'):
    """Start a supervisor on scripted children and return (supervisor, spawner)."""
    spawner = FakeSpawner(clock, *specs)
    supervisor = RecordingSupervisor(settings, spawner, monotonic=clock.monotonic)
    supervisor.start(BASE_NAME, arm_mode, {'ROS_DOMAIN_ID': '80'})
    return supervisor, spawner


class TestRecorderBinary:
    """The recorder is exec'd directly; the ros2 run wrapper is never used (D20)."""

    def test_resolves_the_installed_binary(self):
        """recorder_binary points at <prefix>/lib/franka_bringup/franka_record."""
        path = recorder_binary()
        assert os.path.isabs(path)
        assert path.endswith(os.path.join('lib', 'franka_bringup', 'franka_record'))
        assert os.path.isfile(path) and os.access(path, os.X_OK)

    def test_is_the_pid_that_can_be_signalled(self):
        """The resolved path is a binary, not a wrapper command line."""
        path = recorder_binary()
        assert 'ros2' not in os.path.basename(path)
        assert ' ' not in path


class TestSessionName:
    """Session names are UTC and always acceptable to the recorder."""

    def test_format_is_web_utc_timestamp(self):
        """A UTC instant renders as web-YYYYmmdd-HHMMSS."""
        moment = datetime(2026, 8, 30, 14, 15, 1, tzinfo=timezone.utc)
        assert session_name(moment) == 'web-20260830-141501'

    def test_aware_times_are_converted_to_utc(self):
        """A non-UTC aware time is converted, not formatted where it stands."""
        moment = datetime(2026, 8, 30, 16, 15, 1, tzinfo=timezone(timedelta(hours=2)))
        assert session_name(moment) == 'web-20260830-141501'

    def test_naive_times_are_taken_as_utc(self):
        """A naive datetime is formatted as the UTC time it claims to be."""
        assert session_name(datetime(2026, 8, 30, 14, 15, 1)) == 'web-20260830-141501'

    def test_default_is_now(self):
        """The default name is a well-formed timestamp for the current time."""
        assert re.fullmatch(r'web-\d{8}-\d{6}', session_name())

    def test_matches_the_recorders_own_regex(self):
        """The generated name passes franka_bringup.recorder's gate, object and pattern."""
        name = session_name(datetime(2026, 8, 30, 14, 15, 1, tzinfo=timezone.utc))
        assert bringup_recorder._SAFE_NAME.fullmatch(name)
        assert DOCUMENTED_NAME_PATTERN.fullmatch(name)
        bringup_recorder._validate_name(name)

    def test_the_imported_gate_is_the_documented_one(self):
        """The regex this module validates against has not drifted upstream."""
        assert bringup_recorder._SAFE_NAME.pattern == DOCUMENTED_NAME_PATTERN.pattern


class TestSegmentName:
    """Segment 1 keeps the session name; later segments carry a padded suffix."""

    def test_first_segment_is_the_base_name(self):
        """A session that never rolls over is named exactly as the plan says."""
        assert segment_name(BASE_NAME, 1) == BASE_NAME

    @pytest.mark.parametrize('sequence,expected', [
        (2, 'web-20260830-141501-002'),
        (3, 'web-20260830-141501-003'),
        (10, 'web-20260830-141501-010'),
        (100, 'web-20260830-141501-100'),
        (1000, 'web-20260830-141501-1000'),
    ])
    def test_later_segments_are_suffixed(self, sequence, expected):
        """Rollovers append -002, -003, ... without truncating the base."""
        assert segment_name(BASE_NAME, sequence) == expected

    @pytest.mark.parametrize('sequence', [1, 2, 3, 10, 100, 1000])
    def test_every_segment_name_is_recorder_valid(self, sequence):
        """Every name in the chain passes the recorder's own name gate."""
        name = segment_name(BASE_NAME, sequence)
        assert DOCUMENTED_NAME_PATTERN.fullmatch(name)
        bringup_recorder._validate_name(name)

    @pytest.mark.parametrize('sequence', [0, -1, '2', 2.0, True, None])
    def test_bad_sequence_refused(self, sequence):
        """A sequence that is not an integer of at least 1 is refused."""
        with pytest.raises(RecordingError) as excinfo:
            segment_name(BASE_NAME, sequence)
        assert 'sequence' in str(excinfo.value)

    @pytest.mark.parametrize('base', ['', '-leading-dash', 'has space', 'slash/name', 'dot.name',
                                      'x' * 65, None])
    def test_bad_base_refused(self, base):
        """A base the recorder would refuse is refused here, before spawning."""
        with pytest.raises(RecordingError):
            segment_name(base, 1)

    def test_overlong_base_is_refused_for_a_suffixed_segment(self):
        """A base valid alone but unable to carry -002 is refused, and says why."""
        base = 'w' * 61
        assert DOCUMENTED_NAME_PATTERN.fullmatch(base)
        assert segment_name(base, 1) == base
        with pytest.raises(RecordingError) as excinfo:
            segment_name(base, 2)
        assert 'too long' in str(excinfo.value)

    def test_the_longest_chainable_base_is_accepted(self):
        """A 60-character base still fits its suffix inside the 64-character bound."""
        base = 'w' * 60
        assert len(segment_name(base, 2)) == 64
        bringup_recorder._validate_name(segment_name(base, 2))


class TestTopicsFor:
    """The frame reports the recorder's real topic sets, not a second copy."""

    def test_dual_and_single_are_the_recorders_tuples(self):
        """topics_for returns franka_bringup.recorder's own allowlists."""
        assert topics_for('dual') is bringup_recorder.DUAL_ALLOWED_TOPICS
        assert topics_for('single') is bringup_recorder.SINGLE_ALLOWED_TOPICS

    def test_matches_the_recorders_arm_mode_table(self):
        """Both modes agree with the recorder's own arm-mode table."""
        for mode in ('dual', 'single'):
            assert topics_for(mode) == bringup_recorder.ARM_MODE_TOPICS[mode]

    @pytest.mark.parametrize('mode', ['both', 'DUAL', '', None, 'panda1'])
    def test_unknown_arm_mode_refused(self, mode):
        """Anything but dual or single is refused."""
        with pytest.raises(RecordingError):
            topics_for(mode)


class TestBuildArgv:
    """The spawn argv is exact, and it is the binary rather than the wrapper."""

    def test_dual_argv_is_exact(self, settings):
        """The dual argv matches the plan's command, element for element."""
        assert build_argv(settings, BASE_NAME, 'dual') == (
            recorder_binary(),
            '--output-root', settings.recording_root,
            '--name', BASE_NAME,
            '--duration', '3600',
            '--arm-mode', 'dual',
        )

    def test_single_argv_is_exact(self, settings):
        """The single-arm argv differs only in --arm-mode."""
        assert build_argv(settings, BASE_NAME, 'single')[-2:] == ('--arm-mode', 'single')

    def test_never_spawns_through_ros2_run(self, settings):
        """Audit D20: no ros2 run wrapper appears anywhere in the argv."""
        argv = build_argv(settings, BASE_NAME, 'dual')
        assert argv[0] == recorder_binary()
        assert 'ros2' not in argv
        assert 'run' not in argv
        assert 'franka_record' not in argv[1:]

    def test_duration_is_the_recorders_hard_cap(self, settings):
        """--duration is 3600, the recorder's maximum, which is why chaining exists."""
        argv = build_argv(settings, BASE_NAME, 'dual')
        duration = argv[argv.index('--duration') + 1]
        assert duration == str(defaults.RECORDING_SEGMENT_DURATION_S)
        assert int(duration) == bringup_recorder.MAXIMUM_DURATION_SECONDS
        bringup_recorder._validate_duration(int(duration))

    def test_argv_values_pass_the_recorders_own_validators(self, settings):
        """Name and arm mode are accepted by the recorder that will receive them."""
        argv = build_argv(settings, BASE_NAME, 'single')
        bringup_recorder._validate_name(argv[argv.index('--name') + 1])
        bringup_recorder._validate_arm_mode(argv[argv.index('--arm-mode') + 1])

    def test_bad_name_or_mode_refused(self, settings):
        """An invalid name or arm mode never becomes an argv."""
        with pytest.raises(RecordingError):
            build_argv(settings, 'not a name', 'dual')
        with pytest.raises(RecordingError):
            build_argv(settings, BASE_NAME, 'both')


class TestStart:
    """Every session is recorded: a recorder that fails to come up fails the start."""

    def test_spawns_segment_one(self, settings, clock):
        """Start spawns exactly one child, with the segment-1 argv and the given env."""
        env = {'ROS_DOMAIN_ID': '80'}
        spawner = FakeSpawner(clock)
        supervisor = RecordingSupervisor(settings, spawner, monotonic=clock.monotonic)
        supervisor.start(BASE_NAME, 'dual', env)
        assert len(spawner.calls) == 1
        argv, spawn_env, name = spawner.calls[0]
        assert argv == build_argv(settings, BASE_NAME, 'dual')
        assert spawn_env == env
        assert name == BASE_NAME
        assert supervisor.active is True
        assert supervisor.restarts == 0

    def test_grace_window_is_one_second(self, settings, clock):
        """The child is watched for a second before the session is declared started."""
        supervisor, spawner = _started(settings, clock)
        assert spawner.children[0].waits == [recording.START_GRACE_S]
        assert recording.START_GRACE_S == 1.0
        assert supervisor.active is True

    def test_refuses_when_the_child_dies_immediately(self, settings, clock):
        """A recorder that exits inside the grace window fails the start."""
        spawner = FakeSpawner(clock, {'exits_after': 0.2, 'returncode': 2,
                                      'tail': 'output root must be private'})
        supervisor = RecordingSupervisor(settings, spawner, monotonic=clock.monotonic)
        with pytest.raises(RecordingError) as excinfo:
            supervisor.start(BASE_NAME, 'dual', {})
        message = str(excinfo.value)
        assert 'exited immediately' in message
        assert 'exit status 2' in message
        assert supervisor.active is False
        assert len(spawner.calls) == 1

    def test_refusal_leaves_the_never_started_frame(self, settings, clock):
        """After a refused start nothing claims to be recording anywhere."""
        spawner = FakeSpawner(clock, {'exits_after': 0.0, 'returncode': 3})
        supervisor = RecordingSupervisor(settings, spawner, monotonic=clock.monotonic)
        with pytest.raises(RecordingError):
            supervisor.start(BASE_NAME, 'dual', {})
        assert supervisor.frame(topics_for('dual')) == {
            'active': False, 'disabled': False, 'name': None, 'sequence': 0,
            'path': None, 'arm_mode': None, 'topics': [],
        }

    def test_a_refused_start_does_not_restart_on_tick(self, settings, clock):
        """A failed start is final; the supervisor loop must not resurrect it."""
        spawner = FakeSpawner(clock, {'exits_after': 0.0, 'returncode': 3})
        supervisor = RecordingSupervisor(settings, spawner, monotonic=clock.monotonic)
        with pytest.raises(RecordingError):
            supervisor.start(BASE_NAME, 'dual', {})
        supervisor.tick({})
        assert len(spawner.calls) == 1

    def test_refuses_a_second_start_while_active(self, settings, clock):
        """One supervisor records one session at a time."""
        supervisor, spawner = _started(settings, clock)
        with pytest.raises(RecordingError) as excinfo:
            supervisor.start(BASE_NAME, 'dual', {})
        assert 'already running' in str(excinfo.value)
        assert len(spawner.calls) == 1

    def test_refuses_an_unchainable_base_before_spawning(self, settings, clock):
        """A base too long to carry -002 is refused at start, not an hour later."""
        spawner = FakeSpawner(clock)
        supervisor = RecordingSupervisor(settings, spawner, monotonic=clock.monotonic)
        with pytest.raises(RecordingError) as excinfo:
            supervisor.start('w' * 61, 'dual', {})
        assert 'too long' in str(excinfo.value)
        assert spawner.calls == []

    @pytest.mark.parametrize('base,mode', [('bad name', 'dual'), (BASE_NAME, 'both')])
    def test_refuses_bad_arguments_before_spawning(self, settings, clock, base, mode):
        """An invalid name or arm mode never reaches the spawn callable."""
        spawner = FakeSpawner(clock)
        supervisor = RecordingSupervisor(settings, spawner, monotonic=clock.monotonic)
        with pytest.raises(RecordingError):
            supervisor.start(base, mode, {})
        assert spawner.calls == []


class TestSegmentChaining:
    """A session that outlives --duration keeps recording, visibly, as a chain."""

    def test_tick_is_a_no_op_while_the_segment_runs(self, settings, clock):
        """A healthy segment is left alone."""
        supervisor, spawner = _started(settings, clock)
        for _ in range(5):
            supervisor.tick({})
        assert len(spawner.calls) == 1
        assert supervisor.active is True

    def test_self_exit_starts_the_next_segment(self, settings, clock):
        """When --duration elapses the supervisor immediately starts <base>-002."""
        supervisor, spawner = _started(settings, clock, {'exits_after': 3600.0})
        clock.advance(3600.0)
        supervisor.tick({'ROS_DOMAIN_ID': '80'})
        assert spawner.names == [BASE_NAME, BASE_NAME + '-002']
        assert spawner.calls[1][0] == build_argv(settings, BASE_NAME + '-002', 'dual')
        assert supervisor.restarts == 1
        assert supervisor.active is True

    def test_the_chain_keeps_going(self, settings, clock):
        """A third segment follows the second under the same rule."""
        supervisor, spawner = _started(settings, clock, {'exits_after': 3600.0})
        for _ in range(2):
            clock.advance(3600.0)
            supervisor.tick({})
        assert spawner.names == [BASE_NAME, BASE_NAME + '-002', BASE_NAME + '-003']
        assert supervisor.restarts == 2
        assert supervisor.frame(topics_for('dual'))['sequence'] == 3

    def test_a_segment_that_dies_at_once_stops_the_chain(self, settings, clock):
        """A rollover that fails is reported once, not retried on every tick."""
        supervisor, spawner = _started(
            settings, clock, {'exits_after': 3600.0}, {'exits_after': 0.0, 'returncode': 3})
        clock.advance(3600.0)
        supervisor.tick({})
        with pytest.raises(RecordingError) as excinfo:
            supervisor.tick({})
        assert 'instead of recording its segment' in str(excinfo.value)
        assert supervisor.active is False
        supervisor.tick({})
        supervisor.tick({})
        assert len(spawner.calls) == 2

    def test_env_is_passed_to_every_segment(self, settings, clock):
        """Each chained segment is spawned with the environment it is given."""
        supervisor, spawner = _started(settings, clock, {'exits_after': 3600.0})
        clock.advance(3600.0)
        supervisor.tick({'ROS_DOMAIN_ID': '80', 'RCUTILS_COLORIZED_OUTPUT': '0'})
        assert spawner.calls[1][1] == {'ROS_DOMAIN_ID': '80', 'RCUTILS_COLORIZED_OUTPUT': '0'}


class TestStopLadder:
    """SIGINT first, with the patience franka_record needs to seal the bag (D19)."""

    def test_sigint_budget_covers_the_recorders_own_ladder(self):
        """Audit D19: 30 s is at least the recorder's ~25 s worst-case inner ladder."""
        inner_worst_case = (
            bringup_recorder.SIGINT_FLUSH_TIMEOUT_SECONDS +
            bringup_recorder.TERMINATE_TIMEOUT_SECONDS +
            2 * bringup_recorder.KILL_TIMEOUT_SECONDS)
        assert inner_worst_case == 25
        assert defaults.RECORDER_STOP_SIGINT_WAIT_S == 30.0
        assert defaults.RECORDER_STOP_SIGINT_WAIT_S >= inner_worst_case

    def test_sigint_first_and_thirty_seconds_of_patience(self, settings, clock):
        """A recorder that takes 25 s to seal is stopped by SIGINT alone."""
        supervisor, spawner = _started(settings, clock, {'seal_delay': 25.0})
        started_at = clock.monotonic()
        assert supervisor.stop() == 'sigint'
        child = spawner.children[0]
        assert child.signals == [signal.SIGINT]
        assert child.waits == [recording.START_GRACE_S, defaults.RECORDER_STOP_SIGINT_WAIT_S]
        assert clock.monotonic() - started_at == pytest.approx(25.0)
        assert supervisor.active is False

    def test_escalates_to_sigterm_only_after_the_full_budget(self, settings, clock):
        """A child deaf to SIGINT gets SIGTERM, and only after the whole 30 s."""
        supervisor, spawner = _started(
            settings, clock, {'seals_on': (signal.SIGTERM, signal.SIGKILL)})
        started_at = clock.monotonic()
        assert supervisor.stop() == 'sigterm'
        child = spawner.children[0]
        assert child.signals == [signal.SIGINT, signal.SIGTERM]
        assert child.waits == [
            recording.START_GRACE_S,
            defaults.RECORDER_STOP_SIGINT_WAIT_S,
            defaults.RECORDER_STOP_SIGTERM_WAIT_S,
        ]
        assert clock.monotonic() - started_at == pytest.approx(
            defaults.RECORDER_STOP_SIGINT_WAIT_S)

    def test_sigkill_is_the_last_resort(self, settings, clock):
        """SIGKILL is reached only after both sealing signals had their full budget."""
        supervisor, spawner = _started(settings, clock, {'seals_on': (signal.SIGKILL,)})
        started_at = clock.monotonic()
        assert supervisor.stop() == 'sigkill'
        child = spawner.children[0]
        assert child.signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
        assert clock.monotonic() - started_at == pytest.approx(
            defaults.RECORDER_STOP_SIGINT_WAIT_S + defaults.RECORDER_STOP_SIGTERM_WAIT_S)

    def test_a_wedged_child_raises_after_the_bounded_ladder(self, settings, clock):
        """A child that survives SIGKILL is a loud failure, not a silent success."""
        supervisor, spawner = _started(settings, clock, {'seals_on': ()})
        started_at = clock.monotonic()
        with pytest.raises(RecordingError) as excinfo:
            supervisor.stop()
        assert 'did not exit' in str(excinfo.value)
        assert spawner.children[0].signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
        assert clock.monotonic() - started_at == pytest.approx(
            defaults.RECORDER_STOP_SIGINT_WAIT_S +
            defaults.RECORDER_STOP_SIGTERM_WAIT_S +
            defaults.RECORDER_STOP_SIGKILL_WAIT_S)

    def test_failed_stop_retains_exact_child_for_later_retry(self, settings, clock):
        """A failed wrapper proof is retryable; the recorder owner is not lost."""
        supervisor, spawner = _started(settings, clock)
        child = spawner.children[0]
        real_stop = child.stop
        calls = []

        def fail_once(*budgets):
            calls.append(tuple(budgets))
            if len(calls) == 1:
                raise LauncherError('scripted unkillable recorder group')
            return real_stop(*budgets)

        child.stop = fail_once
        with pytest.raises(RecordingError, match='wrapper.*did not exit'):
            supervisor.stop()

        assert supervisor._child is child
        assert len(calls) == 1
        assert supervisor.stop() == 'sigint'
        assert supervisor._child is None
        assert len(calls) == 2

    def test_an_already_finished_child_is_not_signalled(self, settings, clock):
        """A segment that ended on its own needs no ladder at all."""
        supervisor, spawner = _started(settings, clock, {'exits_after': 3600.0})
        clock.advance(3600.0)
        assert supervisor.stop() == 'exited'
        assert spawner.children[0].signals == []

    def test_stop_without_a_start_is_none(self, settings, clock):
        """Stopping a supervisor that never started reports nothing to stop."""
        supervisor = RecordingSupervisor(settings, FakeSpawner(clock), monotonic=clock.monotonic)
        assert supervisor.stop() is None
        assert supervisor.active is False

    def test_stop_is_idempotent(self, settings, clock):
        """A second stop re-reports the same step and signals nothing more."""
        supervisor, spawner = _started(settings, clock)
        assert supervisor.stop() == 'sigint'
        signals_after_first = list(spawner.children[0].signals)
        assert supervisor.stop() == 'sigint'
        assert supervisor.stop() == 'sigint'
        assert spawner.children[0].signals == signals_after_first

    def test_tick_never_restarts_after_stop(self, settings, clock):
        """After stop the chain is over, even when the clock runs past a segment."""
        supervisor, spawner = _started(settings, clock, {'exits_after': 3600.0})
        assert supervisor.stop() == 'sigint'
        clock.advance(7200.0)
        for _ in range(10):
            supervisor.tick({})
        assert len(spawner.calls) == 1
        assert supervisor.active is False

    def test_stop_after_a_rollover_stops_the_current_segment(self, settings, clock):
        """The ladder is aimed at the segment that is actually running."""
        supervisor, spawner = _started(settings, clock, {'exits_after': 3600.0}, {})
        clock.advance(3600.0)
        supervisor.tick({})
        assert supervisor.stop() == 'sigint'
        assert spawner.children[0].signals == []
        assert spawner.children[1].signals == [signal.SIGINT]


class TestFrame:
    """The section 6.11 recording block, before, during and after a session."""

    def test_before_any_start(self, settings, clock):
        """A server that has never recorded reports the empty shape."""
        supervisor = RecordingSupervisor(settings, FakeSpawner(clock), monotonic=clock.monotonic)
        assert supervisor.frame([]) == {
            'active': False, 'disabled': False, 'name': None, 'sequence': 0,
            'path': None, 'arm_mode': None, 'topics': [],
        }

    def test_during_the_first_segment(self, settings, clock):
        """A running session reports its name, path, mode and topic set."""
        supervisor, _ = _started(settings, clock)
        assert supervisor.frame(topics_for('dual')) == {
            'active': True,
            'disabled': False,
            'name': BASE_NAME,
            'sequence': 1,
            'path': os.path.join(settings.recording_root, BASE_NAME),
            'arm_mode': 'dual',
            'topics': list(bringup_recorder.DUAL_ALLOWED_TOPICS),
        }

    def test_single_arm_session(self, settings, clock):
        """A single-arm session reports the single-arm topic set."""
        supervisor, _ = _started(settings, clock, arm_mode='single')
        frame = supervisor.frame(topics_for('single'))
        assert frame['arm_mode'] == 'single'
        assert frame['topics'] == list(bringup_recorder.SINGLE_ALLOWED_TOPICS)

    def test_after_a_rollover(self, settings, clock):
        """Name and path follow the current segment so the growing bag is findable."""
        supervisor, _ = _started(settings, clock, {'exits_after': 3600.0})
        clock.advance(3600.0)
        supervisor.tick({})
        frame = supervisor.frame(topics_for('dual'))
        assert frame['name'] == BASE_NAME + '-002'
        assert frame['sequence'] == 2
        assert frame['path'] == os.path.join(settings.recording_root, BASE_NAME + '-002')
        assert os.path.basename(frame['path']) == frame['name']

    def test_after_stop(self, settings, clock):
        """A stopped session goes inactive but still says where its bag is."""
        supervisor, _ = _started(settings, clock)
        supervisor.stop()
        frame = supervisor.frame(topics_for('dual'))
        assert frame['active'] is False
        assert frame['name'] == BASE_NAME
        assert frame['sequence'] == 1
        assert frame['path'] == os.path.join(settings.recording_root, BASE_NAME)

    def test_keys_are_exactly_the_contract(self, settings, clock):
        """The block carries the seven frozen keys and nothing else."""
        expected = {'active', 'disabled', 'name', 'sequence', 'path',
                    'arm_mode', 'topics'}
        supervisor = RecordingSupervisor(settings, FakeSpawner(clock), monotonic=clock.monotonic)
        assert set(supervisor.frame([])) == expected
        supervisor.start(BASE_NAME, 'dual', {})
        assert set(supervisor.frame(topics_for('dual'))) == expected

    def test_topics_are_a_copy(self, settings, clock):
        """Mutating a frame's topic list cannot corrupt the recorder's tuple."""
        supervisor, _ = _started(settings, clock)
        frame = supervisor.frame(topics_for('dual'))
        frame['topics'].append('/not-recorded')
        assert len(bringup_recorder.DUAL_ALLOWED_TOPICS) == 9
        assert supervisor.frame(topics_for('dual'))['topics'] == list(
            bringup_recorder.DUAL_ALLOWED_TOPICS)


class TestRecordingDisabled:
    """``recording.enabled: false`` turns the whole chain off, quietly."""

    def test_recording_disabled_never_spawns_a_child(self, disabled_settings, clock):
        """Nothing is spawned, and the supervisor is not `active`."""
        spawner = FakeSpawner(clock)
        supervisor = RecordingSupervisor(
            disabled_settings, spawner, monotonic=clock.monotonic)
        supervisor.start(BASE_NAME, 'dual', {})
        supervisor.tick({})
        supervisor.tick({})
        assert spawner.calls == []
        assert supervisor.active is False
        assert supervisor.disabled is True

    def test_recording_disabled_reports_disabled_true_in_the_frame(
            self, disabled_settings, clock):
        """
        The frame says WHY there is no recording.

        `active: false` alone cannot distinguish "no session is running" from
        "policy turned it off", and the console hides its REC chip on the
        second.
        """
        supervisor = RecordingSupervisor(
            disabled_settings, FakeSpawner(clock), monotonic=clock.monotonic)
        assert supervisor.frame([])['disabled'] is True
        supervisor.start(BASE_NAME, 'dual', {})
        frame = supervisor.frame(topics_for('dual'))
        assert frame['disabled'] is True
        assert frame['active'] is False
        # No segment was ever spawned, so there is no bag to name.
        assert frame['name'] is None and frame['path'] is None

    def test_recording_disabled_stop_is_a_no_op(self, disabled_settings, clock):
        """Stopping a chain that never started is not an error."""
        supervisor = RecordingSupervisor(
            disabled_settings, FakeSpawner(clock), monotonic=clock.monotonic)
        supervisor.start(BASE_NAME, 'dual', {})
        assert supervisor.stop() is None
        assert supervisor.stop() is None

    def test_an_enabled_supervisor_reports_disabled_false(self, settings, clock):
        """The key is ALWAYS present and always a boolean."""
        supervisor = RecordingSupervisor(
            settings, FakeSpawner(clock), monotonic=clock.monotonic)
        assert supervisor.disabled is False
        assert supervisor.frame([])['disabled'] is False


class TestRecorderOutputReachesTheLogBus:
    """The recorder is log source two; its spawn site must pass the sink."""

    def test_recorder_output_reaches_the_log_bus(self, settings, clock):
        """
        A line the recorder child prints lands in the bus as `franka_record`.

        This is the wire that is easy to miss, because the recorder's spawn
        is injected rather than called directly -- and its absence is
        invisible until an operator opens the drawer.
        """
        bus = LogBus()
        spawner = FakeSpawner(clock)
        supervisor = RecordingSupervisor(
            settings, spawner, monotonic=clock.monotonic, log_bus=bus)
        supervisor.start(BASE_NAME, 'dual', {})
        assert spawner.children[0].on_line is not None
        spawner.children[0].emit('[INFO] [1.0] [franka_record]: recording started')
        lines = bus.window()['lines']
        assert [line['node'] for line in lines] == ['franka_record']
        assert lines[0]['message'] == 'recording started'

    def test_a_supervisor_without_a_bus_still_spawns(self, settings, clock):
        """`log_bus=None` keeps every existing test double working."""
        spawner = FakeSpawner(clock)
        supervisor = RecordingSupervisor(
            settings, spawner, monotonic=clock.monotonic)
        supervisor.start(BASE_NAME, 'dual', {})
        assert spawner.children[0].on_line is None


class TestRecorderRefusalIsSurfaced:
    """The reviewed recorder is the authority on its own output root."""

    def test_a_permission_refusal_is_surfaced_verbatim_with_a_chmod_hint(
            self, settings, clock):
        """
        The recorder's own sentence, then one command that fixes it.

        The server does not pre-check the mode bits, does not paraphrase and
        does not gate boot on it: it creates the directory 0700 and stops.
        """
        refusal = ('the output root must have no group or other permission '
                   'bits (expected mode 0700)')
        spawner = FakeSpawner(
            clock, {'exits_after': 0.0, 'returncode': 2, 'tail': refusal})
        supervisor = RecordingSupervisor(
            settings, spawner, monotonic=clock.monotonic)
        with pytest.raises(RecordingError) as excinfo:
            supervisor.start(BASE_NAME, 'dual', {})
        message = str(excinfo.value)
        assert refusal in message
        assert 'Run: chmod 700 {}'.format(settings.recording_root) in message

    def test_an_unrelated_refusal_gets_no_chmod_hint(self, settings, clock):
        """The hint is for the cause it fixes, and for nothing else."""
        spawner = FakeSpawner(
            clock, {'exits_after': 0.0, 'returncode': 2,
                    'tail': 'the topic set is not recordable'})
        supervisor = RecordingSupervisor(
            settings, spawner, monotonic=clock.monotonic)
        with pytest.raises(RecordingError) as excinfo:
            supervisor.start(BASE_NAME, 'dual', {})
        assert 'chmod' not in str(excinfo.value)
