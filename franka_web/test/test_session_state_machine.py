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
Every transition of SessionSupervisor, driven with fakes.

Also the v2 additions that live on the state machine rather than on the
motion surface: the verification checklist, the in-session pre-activation
baseline (and its unconditional controller-state gate), the persistent hint
line, the per-arm command source, and the log bus's launch-child wire.
"""

import threading

from franka_web import defaults
from franka_web.gains import ProfileStore
from franka_web.launcher import LauncherError
from franka_web.lock import OperatorLock
from franka_web.logbus import LogBus
from franka_web.preflight import run_preflight
from franka_web.recording import RecordingError
from franka_web.session import SessionError, SessionRequest, SessionSupervisor
import pytest
from sensor_msgs.msg import JointState
from support.config_factory import DOC_IP_1, DOC_IP_2, make_settings
from support.fake_clock import FakeClock
from support.fake_launcher import (
    FakeBridge, FakeBroker, FakeChild, FakePreflightResult,
    FakeRecording, FakeSpawner)


#: The Franka home pose, well inside the factory policy limits at every
#: joint -- which matters now that a Motion session checks its captured
#: baseline against the fence before the controller can take hold.
HOME_POSE = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)


def dual_joint_state(positions=HOME_POSE):
    """Build a complete 14-name dual JointState at ``positions``."""
    msg = JointState()
    for arm in ('panda1', 'panda2'):
        for joint in range(1, 8):
            msg.name.append('{}_joint{}'.format(arm, joint))
            msg.position.append(float(positions[joint - 1]))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
    return msg


def torn_joint_state(short_arm='panda2', keep=4):
    """Build a dual JointState in which one arm carries only ``keep`` joints."""
    msg = JointState()
    for arm in ('panda1', 'panda2'):
        count = keep if arm == short_arm else 7
        for joint in range(1, count + 1):
            msg.name.append('{}_joint{}'.format(arm, joint))
            msg.position.append(float(HOME_POSE[joint - 1]))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
    return msg


def non_finite_joint_state(arm='panda2', joint=3):
    """Build a complete dual JointState carrying one NaN position."""
    msg = dual_joint_state()
    msg.position[('panda1', 'panda2').index(arm) * 7 + (joint - 1)] = \
        float('nan')
    return msg


class Harness:
    """One fully faked supervisor plus its collaborators."""

    def __init__(self, tmp_path, preflight=None, recorder=None, **settings_extra):
        """Wire a supervisor whose collaborators are all inspectable fakes."""
        self.clock = FakeClock()
        self.settings = make_settings(tmp_path, **settings_extra)
        self.bridge = FakeBridge()
        self.broker = FakeBroker()
        self.spawner = FakeSpawner()
        self.events = []
        self.recorder = recorder or FakeRecording(events=self.events)
        self.preflight = preflight or FakePreflightResult()
        self.launch_child = FakeChild(name='launch')
        self.spawner.queue_child(self.launch_child)
        self.logs = LogBus()
        self.lock = OperatorLock(monotonic=self.clock.monotonic)
        self.claim = self.lock.claim()
        self.operator_lease = self.lock.authorize(self.claim.token)
        self.supervisor = SessionSupervisor(
            self.settings, self.bridge, self.lock, self.broker,
            spawn=self._spawn,
            recording_factory=lambda: self.recorder,
            preflight_runner=lambda settings, mode: self.preflight,
            profile_store=ProfileStore(self.settings.state_dir),
            log_bus=self.logs,
            monotonic=self.clock.monotonic,
        )

    def _spawn(self, argv, env, name, **kwargs):
        """Route spawns through the recording FakeSpawner with event order."""
        child = self.spawner(argv, env, name, **kwargs)
        self.events.append('spawn-' + name)
        original_stop = child.stop

        def recording_stop(a, b, c):
            self.events.append('launch-stop')
            return original_stop(a, b, c)
        child.stop = recording_stop
        return child

    def make_ready_simulate(self):
        """Satisfy the §3.5 simulate readiness criteria."""
        self.bridge.controllers = {'joint_state_broadcaster': 'active'}
        self.bridge.types = {
            'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster'}
        self.bridge.joint = (self.clock.monotonic_ns(), dual_joint_state())

    def start(self, arms='both', mode='simulate'):
        """Submit a start request and process it on the supervisor thread."""
        result = {}

        def submit():
            try:
                result['value'] = self.supervisor.request_start(
                    SessionRequest(arms=arms, mode=mode),
                    operator_lease=self.operator_lease)
            except SessionError as error:
                result['error'] = error
        thread = threading.Thread(target=submit)
        thread.start()
        for _ in range(50):
            if 'value' in result or 'error' in result:
                break
            self.supervisor.tick()
        thread.join(timeout=2)
        if 'error' in result:
            raise result['error']
        return result.get('value')

    def stop(self):
        """Submit a stop request and process it."""
        result = {}

        def submit():
            try:
                result['value'] = self.supervisor.request_stop()
            except SessionError as error:
                result['error'] = error
        thread = threading.Thread(target=submit)
        thread.start()
        for _ in range(50):
            if 'value' in result or 'error' in result:
                break
            self.supervisor.tick()
        thread.join(timeout=2)
        if 'error' in result:
            raise result['error']
        return result.get('value')


@pytest.fixture()
def harness(tmp_path):
    """Build a default simulate-capable harness."""
    h = Harness(tmp_path)
    h.make_ready_simulate()
    return h


class TestHappyPath:
    """stopped -> preflight -> starting -> running -> stopping -> stopped."""

    def test_full_simulate_lifecycle(self, harness):
        """The whole §3.4 loop with readiness immediately satisfied."""
        accepted = harness.start()
        assert accepted['state'] == 'preflight'
        assert accepted['session_id'].startswith('web-')
        for _ in range(5):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'running'
        assert harness.events[:2] == ['recorder-start', 'spawn-launch']
        assert harness.bridge.configured == (('panda1', 'panda2'), 'dual')
        stopped = harness.stop()
        assert stopped['state'] == 'stopping'
        for _ in range(3):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'
        assert harness.bridge.cleared >= 1

    def test_recorder_sealed_before_launch_stopped(self, harness):
        """§5.5: the recorder seals BEFORE the launch child is touched."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        harness.stop()
        for _ in range(3):
            harness.supervisor.tick()
        assert 'recorder-stop' in harness.events
        assert 'launch-stop' in harness.events
        assert (harness.events.index('recorder-stop')
                < harness.events.index('launch-stop'))

    def test_frame_in_stopped_has_empty_arms(self, harness):
        """Frame rule 1: arms is {} in stopped."""
        frame = harness.supervisor.frame()
        assert frame['session']['state'] == 'stopped'
        assert frame['arms'] == {}
        assert frame['session']['advisory'].startswith('The physical stop buttons')

    def test_frame_while_running_carries_arms_and_motion_off(self, harness):
        """The running frame carries both arms with motion.available False."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        frame = harness.supervisor.frame()
        assert set(frame['arms']) == {'panda1', 'panda2'}
        for arm in frame['arms'].values():
            assert arm['motion']['available'] is False
            assert arm['motion']['enabled'] is False
        assert frame['schema_version'] == 3


class TestStartRefusals:
    """Command validation refusals (§6.7 error codes)."""

    def test_second_start_refused(self, harness):
        """session_already_active while any session exists."""
        harness.start()
        with pytest.raises(SessionError) as excinfo:
            harness.start()
        assert excinfo.value.code == 'session_already_active'

    def test_invalid_arms(self, harness):
        """invalid_arms for an unknown arm selection."""
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='panda3')
        assert excinfo.value.code == 'invalid_arms'

    def test_unknown_mode_refused(self, harness):
        """A mode outside the closed set is invalid_mode."""
        with pytest.raises(SessionError) as excinfo:
            harness.start(mode='teleop')
        assert excinfo.value.code == 'invalid_mode'

    def test_a_watch_start_no_longer_needs_an_address_to_be_supplied(self, harness):
        """
        Every address defaults, so `robot_addresses_missing` is unreachable.

        The refusal survives as a defensive code only; a Watch start from a
        configuration that names no address still reaches preflight.
        """
        accepted = harness.start(mode='watch')
        assert accepted['state'] == 'preflight'
        assert DOC_IP_1 in ' '.join(harness.supervisor._session['launch_argv'])
        assert DOC_IP_2 in ' '.join(harness.supervisor._session['launch_argv'])

    def test_stop_when_stopped_refused(self, harness):
        """session_not_active when nothing runs."""
        with pytest.raises(SessionError) as excinfo:
            harness.stop()
        assert excinfo.value.code == 'session_not_active'


class TestFailurePaths:
    """preflight/recording/launch failures land in stopped with last_error."""

    def test_blocking_preflight_failure_stops(self, tmp_path):
        """A blocking FAIL never spawns anything."""
        h = Harness(
            tmp_path,
            preflight=FakePreflightResult(overall='FAIL', passed=False, blocking=True))
        h.start(mode='watch')
        for _ in range(5):
            h.supervisor.tick()
        assert h.supervisor.state == 'stopped'
        frame = h.supervisor.frame()
        assert frame['session']['last_error']['code'] == 'preflight_failed'
        assert h.events == []

    def test_a_missing_preflight_tool_says_so_in_last_error(self, tmp_path):
        """
        Finding F-2: the invocation-level reason reaches the operator.

        ``PreflightResult.error`` -- the only thing that distinguishes "the
        tool is not installed" from "it timed out" from "its report was
        unusable" -- was computed and then read nowhere, so a host without
        ``franka_rt_preflight`` refused every watch/motion start with a bare
        ``"RT preflight failed: ERROR"`` and no way to act on it. §6.11 gives
        the frame's preflight block no field for it, so ``last_error.detail``
        is where it belongs.

        The real ``run_preflight`` runs here, with only the subprocess seam
        replaced: this is the genuine missing-tool path, not a hand-built
        result object.
        """
        def missing_tool(argv, **kwargs):
            raise FileNotFoundError(2, 'No such file or directory', argv[0])

        h = Harness(
            tmp_path)
        h.preflight = run_preflight(h.settings, 'watch', runner=missing_tool)
        assert h.preflight.overall == 'ERROR'
        assert h.preflight.error, 'the ERROR verdict must carry a reason'

        h.start(mode='watch')
        for _ in range(5):
            h.supervisor.tick()

        assert h.supervisor.state == 'stopped'
        last_error = h.supervisor.frame()['session']['last_error']
        assert last_error['code'] == 'preflight_failed'
        assert h.preflight.error in last_error['detail']
        assert 'franka_rt_preflight' in last_error['detail']
        assert h.events == []

    def test_a_plain_fail_verdict_carries_no_invented_reason(self, tmp_path):
        """A FAIL has failed_checks, not an invocation error; the detail stays clean."""
        h = Harness(
            tmp_path,
            preflight=FakePreflightResult(overall='FAIL', passed=False, blocking=True))
        h.start(mode='watch')
        for _ in range(5):
            h.supervisor.tick()
        assert (h.supervisor.frame()['session']['last_error']['detail']
                == 'RT preflight failed: FAIL')

    def test_simulate_preflight_failure_is_nonblocking(self, tmp_path):
        """A FAIL in simulate is a warning; the session still starts."""
        h = Harness(
            tmp_path,
            preflight=FakePreflightResult(overall='FAIL', passed=False, blocking=False))
        h.make_ready_simulate()
        h.start()
        for _ in range(5):
            h.supervisor.tick()
        assert h.supervisor.state == 'running'

    def test_recording_failure_prevents_start(self, tmp_path):
        """recording_failed: the recorder is an invariant, not best-effort."""
        h = Harness(tmp_path, recorder=FakeRecording(fail_start=True))
        h.make_ready_simulate()
        h.start()
        for _ in range(5):
            h.supervisor.tick()
        assert h.supervisor.state == 'stopped'
        assert h.supervisor.frame()['session']['last_error']['code'] == 'recording_failed'
        assert not any(s['name'] == 'launch' for s in h.spawner.spawned)

    def test_launch_spawn_failure(self, harness):
        """launch_failed when the spawn itself raises."""
        harness.spawner.children.clear()
        harness.spawner.fail_names.add('launch')
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'
        assert (harness.supervisor.frame()['session']['last_error']['code']
                == 'launch_failed')

    def test_child_exit_during_starting(self, tmp_path):
        """§3.4: starting exits to stopping on child exit."""
        h = Harness(tmp_path)
        h.bridge.controllers = {}
        h.start()
        h.launch_child.die(returncode=1)
        for _ in range(5):
            h.supervisor.tick()
        assert h.supervisor.state == 'stopped'
        assert h.supervisor.frame()['session']['last_error']['code'] == 'launch_failed'

    def test_starting_timeout(self, tmp_path):
        """launch_timeout after STARTING_TIMEOUT_S without readiness."""
        h = Harness(tmp_path)
        h.bridge.controllers = {}
        h.start()
        assert h.supervisor.state == 'starting'
        h.clock.advance(61.0)
        for _ in range(3):
            h.supervisor.tick()
        assert h.supervisor.state == 'stopped'
        assert h.supervisor.frame()['session']['last_error']['code'] == 'launch_timeout'


class TestFaults:
    """running -> fault wiring (the engine itself is tested separately)."""

    def test_launch_death_while_running_faults(self, harness):
        """F7: the launch child dying moves running to fault."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'running'
        harness.launch_child.die(returncode=1)
        harness.supervisor.tick()
        assert harness.supervisor.state == 'fault'
        frame = harness.supervisor.frame()
        assert frame['fault']['active'] is True
        assert any(r['code'] == 'launch_exited' for r in frame['fault']['reasons'])
        assert frame['fault']['recoverable'] is False

    def test_fault_forces_enables_off(self, harness):
        """The §3.4 invariant: entering fault clears every enable flag."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        harness.supervisor._arm_enabled['panda1'] = True
        harness.launch_child.die()
        harness.supervisor.tick()
        assert harness.supervisor._arm_enabled == {'panda1': False, 'panda2': False}

    def test_stop_from_fault(self, harness):
        """Drive fault -> stopping -> stopped."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        harness.launch_child.die()
        harness.supervisor.tick()
        assert harness.supervisor.state == 'fault'
        harness.stop()
        for _ in range(3):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'


class TestStopSemantics:
    """Stop idempotency and shutdown."""

    def test_stop_is_idempotent_while_stopping(self, harness):
        """A second stop during stopping answers stopping, not an error."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        with harness.supervisor._state_lock:
            harness.supervisor._state = 'stopping'
        assert harness.stop() == {'state': 'stopping'}

    def test_shutdown_from_running(self, harness):
        """Process shutdown drives everything to stopped."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        harness.supervisor.shutdown()
        assert harness.supervisor.state == 'stopped'
        assert harness.recorder.stopped is True

    def test_failed_launch_stop_retains_identity_until_shutdown_retry(self, harness):
        """A survivor keeps stopping/pidfile ownership; a later retry can finish."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        real_stop = harness.launch_child.stop
        calls = []

        def fail_once(*budgets):
            calls.append(tuple(budgets))
            if len(calls) == 1:
                raise LauncherError('scripted unkillable target group')
            return real_stop(*budgets)

        harness.launch_child.stop = fail_once
        with harness.supervisor._state_lock:
            harness.supervisor._state = 'stopping'

        harness.supervisor._do_stopping()

        assert harness.supervisor.state == 'stopping'
        assert harness.supervisor._launch is harness.launch_child
        assert len(calls) == 1

        harness.supervisor.shutdown()

        assert harness.supervisor.state == 'stopped'
        assert harness.supervisor._launch is None
        assert len(calls) == 2
        assert harness.bridge.cleared >= 2

    def test_failed_recorder_stop_retains_identity_until_shutdown_retry(self, harness):
        """A recorder survivor blocks stopped and is retried by shutdown."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        real_stop = harness.recorder.stop
        calls = []

        def fail_once():
            calls.append('stop')
            if len(calls) == 1:
                raise RecordingError('scripted unkillable recorder group')
            return real_stop()

        harness.recorder.stop = fail_once
        with harness.supervisor._state_lock:
            harness.supervisor._state = 'stopping'

        harness.supervisor._do_stopping()

        assert harness.supervisor.state == 'stopping'
        assert harness.supervisor._recording is harness.recorder
        assert harness.supervisor._launch is None
        assert harness.recorder.stopped is False
        assert len(calls) == 1

        harness.supervisor.shutdown()

        assert harness.supervisor.state == 'stopped'
        assert harness.supervisor._recording is None
        assert harness.recorder.stopped is True
        assert len(calls) == 2
        assert harness.bridge.cleared >= 2

    def test_transitions_publish_frames(self, harness):
        """Every transition pushes an immediate state frame to the broker."""
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        states = [data['session']['state']
                  for event, data in harness.broker.events if event == 'state']
        assert 'preflight' in states
        assert 'starting' in states
        assert 'running' in states


def step(frame, step_id):
    """Return one entry of the frame's verification checklist."""
    for entry in frame['session']['steps']:
        if entry['id'] == step_id:
            return entry
    raise AssertionError('no step {!r} in {}'.format(
        step_id, [entry['id'] for entry in frame['session']['steps']]))


def step_ids(frame):
    """Return the checklist's step ids in order."""
    return [entry['id'] for entry in frame['session']['steps']]


class TestVerificationChecklist:
    """`session.steps` is the checklist the console renders during startup."""

    def test_steps_are_built_in_the_documented_order_for_each_mode(self, harness):
        """Simulate and Watch stop at `baseline`; Motion adds two more."""
        harness.start(arms='both', mode='simulate')
        assert step_ids(harness.supervisor.frame()) == [
            'preflight', 'connect:panda1', 'connect:panda2', 'health', 'baseline']
        harness.supervisor._steps_init('motion', ('panda2',))
        assert step_ids(harness.supervisor.frame()) == [
            'preflight', 'connect:panda2', 'health', 'baseline',
            'controller', 'settling']

    def test_steps_are_empty_in_stopped(self, harness):
        """A stopped session has no checklist."""
        assert harness.supervisor.frame()['session']['steps'] == []
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        harness.stop()
        for _ in range(3):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'
        assert harness.supervisor.frame()['session']['steps'] == []

    def test_a_fresh_checklist_starts_pending_with_preflight_active(self, harness):
        """The array is built pending, and only `preflight` is active."""
        harness.supervisor._steps_init('simulate', ('panda1', 'panda2'))
        harness.supervisor._step_active('preflight')
        statuses = [(entry['id'], entry['status'])
                    for entry in harness.supervisor.frame()['session']['steps']]
        assert statuses == [('preflight', 'active'), ('connect:panda1', 'pending'),
                            ('connect:panda2', 'pending'), ('health', 'pending'),
                            ('baseline', 'pending')]

    def test_steps_advance_pending_active_done_monotonically(self, harness):
        """No step's status ever goes backwards across published frames."""
        rank = {'pending': 0, 'active': 1, 'done': 2, 'failed': 2}
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        seen = {}
        published = [event[1] for event in harness.broker.events
                     if event[0] == 'state']
        assert published, 'no state frame was ever published'
        for frame in published:
            for entry in frame['session']['steps']:
                previous = seen.get(entry['id'], 0)
                assert rank[entry['status']] >= previous, (
                    '{} went backwards'.format(entry['id']))
                seen[entry['id']] = rank[entry['status']]
        final = harness.supervisor.frame()
        assert [entry['status'] for entry in final['session']['steps']] == (
            ['done'] * 5)
        assert step(final, 'preflight')['duration_s'] is not None

    def test_a_failed_step_records_its_detail_and_duration(self, tmp_path):
        """
        A blocking preflight failure marks its own step failed.

        The checklist is cleared on the way into `stopped`, so the evidence
        is read from the frame the transition published, not from the frame
        after teardown.
        """
        h = Harness(tmp_path, preflight=FakePreflightResult(
            overall='FAIL', passed=False, blocking=True))
        h.make_ready_simulate()
        h.start(mode='watch')
        for _ in range(4):
            h.supervisor.tick()
        failed = [frame for event, frame in h.broker.events
                  if event == 'state' and frame['session']['steps']
                  and frame['session']['steps'][0]['status'] == 'failed']
        assert failed, 'no published frame carried the failed step'
        entry = failed[-1]['session']['steps'][0]
        assert entry['id'] == 'preflight'
        assert entry['detail'] == 'FAIL'
        assert h.supervisor.frame()['session']['steps'] == []

    def test_the_baseline_step_completes_in_simulate_and_watch(self, tmp_path):
        """
        `baseline` completes in EVERY mode, on a fresh complete sample alone.

        This is the test that fails if _capture_baseline is ever put back
        behind an `if mode == 'motion'` guard: `baseline` is the LAST step in
        these modes, so nothing would ever back-fill it and the whole
        start-phase hint row would be stranded.
        """
        for mode in ('simulate', 'watch'):
            h = Harness(tmp_path / mode)
            h.make_ready_simulate()
            h.start(arms='both', mode=mode)
            for _ in range(6):
                h.supervisor.tick()
            frame = h.supervisor.frame()
            assert step(frame, 'baseline')['status'] == 'done'
            assert h.supervisor._baseline_captured is True
            # No fence exists outside Motion, so none was checked.
            assert h.supervisor._profile_record is None
            for arm in frame['arms'].values():
                assert arm['motion']['fence_lower'] is None
                assert arm['motion']['fence_upper'] is None

    def test_recover_replaces_the_steps_array_with_the_recovery_list(self, harness):
        """
        A Recover swaps the checklist for the recovery list.

        The `reconnect:` prefix on steps[0] is the stable discriminator
        between a startup checklist and a recovery checklist.
        """
        harness.supervisor._steps_recovery(('panda1', 'panda2'))
        frame = harness.supervisor.frame()
        assert step_ids(frame) == [
            'reconnect:panda1', 'reconnect:panda2', 'controller', 'verify']
        assert frame['session']['steps'][0]['id'].startswith('reconnect:')
        # `controller` deliberately reads differently in each list.
        assert step(frame, 'controller')['label'] == 'Restart controller'
        harness.supervisor._steps_init('motion', ('panda1',))
        startup = step(harness.supervisor.frame(), 'controller')
        assert startup['label'] == 'Controller active'


class TestBaselineCapture:
    """The in-session pre-activation baseline, and its ordering guarantee."""

    def _motion_harness(self, tmp_path, ready=True):
        """
        Return a Motion harness parked in `starting` with nothing captured.

        The start request is submitted the way an HTTP worker does, but the
        joint sample is withheld until the caller wants it, so the tick that
        captures the baseline is the caller's own -- these tests are about
        exactly what happens on that tick.
        """
        h = Harness(tmp_path)
        h.start(arms='both', mode='motion')
        for _ in range(4):
            h.supervisor.tick()
        assert h.supervisor.state == 'starting'
        assert h.supervisor._baseline_captured is False
        if ready:
            h.make_ready_simulate()
        return h

    def test_baseline_is_captured_from_the_first_fresh_complete_sample(self, tmp_path):
        """The captured pose is the measured one, per arm."""
        h = self._motion_harness(tmp_path)
        h.supervisor.tick()
        assert h.supervisor._baseline_captured is True
        baseline = h.supervisor._activation_baseline
        assert set(baseline) == {'panda1', 'panda2'}
        assert baseline['panda1'] == HOME_POSE

    def test_baseline_capture_waits_for_the_health_step(self, tmp_path):
        """Nothing is captured before the graph is healthy."""
        h = Harness(tmp_path)
        h.start(arms='both', mode='motion')       # no joint sample at all
        h.supervisor.tick()
        h.supervisor.tick()
        assert h.supervisor._baseline_captured is False
        assert h.supervisor.state == 'starting'

    @pytest.mark.parametrize('fresh_sample', [False, True],
                             ids=['stale-sample', 'fresh-sample'])
    def test_baseline_capture_fails_closed_when_the_controller_activates_first(
            self, tmp_path, fresh_sample):
        """
        The controller-state test gates UNCONDITIONALLY, fresh sample included.

        The common race is a FRESH sample: the launch's `spawner
        --switch-asap` activates the controller, a 1 kHz joint sample lands,
        and the same 100 ms tick would otherwise capture a POST-activation
        pose as the pre-activation baseline -- after which the settling gate
        measures drift from an already-torqued pose, the activation jump
        reads as about zero, and the gate passes trivially.

        Reordering _capture_baseline so freshness is tested first must fail
        the fresh case; if it does not, this test is not testing the property.
        """
        h = self._motion_harness(tmp_path)
        h.bridge.controllers[defaults.MOTION_CONTROLLER] = 'active'
        if not fresh_sample:
            # Older than the baseline's freshness window, but still inside
            # the staleness fault window -- so health completes and the
            # controller-state test is genuinely the thing that decides.
            h.bridge.joint = (
                h.clock.monotonic_ns()
                - int((defaults.ENABLE_JOINT_STATE_MAX_AGE_S + 0.3) * 1e9),
                h.bridge.joint[1])
        for _ in range(3):
            h.supervisor.tick()
        assert h.supervisor._baseline_captured is False
        assert h.supervisor.state in ('stopping', 'stopped')
        failed = [frame for event, frame in h.broker.events
                  if event == 'state'
                  and any(entry['id'] == 'baseline' and entry['status'] == 'failed'
                          for entry in frame['session']['steps'])]
        assert failed, 'no published frame carried the failed baseline step'
        assert failed[-1]['session']['last_error']['code'] == (
            'activation_settling_limit')
        # No enable was ever possible: the session never reached `running`.
        assert all(enabled is False
                   for enabled in h.supervisor._arm_enabled.values())

    def _health_done_awaiting_a_fresh_sample(self, tmp_path, mode):
        """
        Park a session in `starting` with `health` done and nothing captured.

        `health` is monotonic once done, while _capture_baseline re-reads the
        bridge sample on every later tick -- so this is the state in which a
        torn sample can reach the capture, and the state these tests need.
        """
        h = Harness(tmp_path)
        h.start(arms='both', mode=mode)
        for _ in range(4):
            h.supervisor.tick()
        h.make_ready_simulate()
        # Older than the baseline's freshness window, but still inside the
        # staleness fault window: `health` completes on this sample while the
        # baseline capture keeps waiting for a fresh one.
        h.bridge.joint = (
            h.clock.monotonic_ns()
            - int((defaults.ENABLE_JOINT_STATE_MAX_AGE_S + 0.3) * 1e9),
            h.bridge.joint[1])
        h.supervisor.tick()
        assert h.supervisor._steps_status('health') == 'done'
        assert h.supervisor._baseline_captured is False
        assert h.supervisor.state == 'starting'
        return h

    @pytest.mark.parametrize('mode', ['motion', 'watch'])
    @pytest.mark.parametrize('build', [torn_joint_state, non_finite_joint_state],
                             ids=['incomplete', 'non-finite'])
    def test_a_torn_or_non_finite_sample_is_never_captured(
            self, tmp_path, mode, build):
        """
        An arm dropping out of the 14-name sample cannot become the baseline.

        The whole activation-settling argument is measured against this pose,
        so a fresh-but-torn sample -- one arm gone from the message, or a NaN
        position -- must leave the capture untaken and the session in
        `starting`, in every mode and without raising out of the tick. The
        next good sample still captures: this refuses a sample, it does not
        latch a failure.
        """
        h = self._health_done_awaiting_a_fresh_sample(tmp_path, mode)
        h.bridge.joint = (h.clock.monotonic_ns(), build())

        h.supervisor.tick()

        assert h.supervisor._baseline_captured is False
        assert h.supervisor._activation_baseline is None
        assert h.supervisor.state == 'starting'
        assert step(h.supervisor.frame(), 'baseline')['status'] != 'done'

        h.bridge.joint = (h.clock.monotonic_ns(), dual_joint_state())
        h.supervisor.tick()
        assert h.supervisor._baseline_captured is True
        assert h.supervisor._activation_baseline['panda2'] == HOME_POSE

    def test_a_baseline_outside_the_fence_faults_before_any_operator_torque(
            self, tmp_path):
        """A pose outside the fence faults at the baseline step."""
        h = self._motion_harness(tmp_path)
        record = h.supervisor._profile_record
        h.supervisor._profile_record = _narrow(record, lower_shift=2.0)
        h.supervisor.tick()
        frame = h.supervisor.frame()
        assert h.supervisor.state == 'fault'
        assert step(frame, 'baseline')['status'] == 'failed'
        assert frame['fault']['cause'] == 'session_wedged'
        assert frame['fault']['action'] == 'restart'
        assert frame['session']['last_error']['code'] == 'pose_outside_fence'

    def test_the_baseline_message_names_the_joint_and_both_bounds_in_degrees(
            self, tmp_path):
        """The operator is told which joint, where it is, and where it may be."""
        h = self._motion_harness(tmp_path)
        h.supervisor._profile_record = _narrow(
            h.supervisor._profile_record, lower_shift=+2.0)
        h.supervisor.tick()
        detail = step(h.supervisor.frame(), 'baseline')['detail']
        assert detail.startswith('panda1 J2 is at ')
        assert '°' in detail
        assert 'outside its limits' in detail

    def test_the_baseline_margin_message_names_both_keys_and_the_summed_reserve(
            self, tmp_path):
        """
        The message names BOTH keys and their sum.

        Naming the margin alone teaches the operator to raise a value that
        will still be refused: the reserve is the margin PLUS the drift limit.
        """
        h = self._motion_harness(tmp_path)
        h.supervisor._profile_record = _snug(h.supervisor._profile_record)
        h.supervisor.tick()
        detail = step(h.supervisor.frame(), 'baseline')['detail']
        assert 'settling.fence_margin_deg' in detail
        assert 'settling.drift_limit_deg' in detail
        assert 'reserves 7.0°' in detail

    def test_motion_start_needs_no_prior_watch_session(self, harness):
        """The attestation prerequisite is gone: Motion starts from stopped."""
        accepted = harness.start(arms='both', mode='motion')
        assert accepted['state'] == 'preflight'

    def test_watch_start_creates_no_evidence_for_a_later_motion_start(self, harness):
        """The pose cache does not exist at all any more."""
        for attribute in ('_pose_cache', '_watch_preview_cache',
                          '_watch_preview_gains', '_stop_requested',
                          '_watch_recovery_barrier_ns'):
            assert not hasattr(harness.supervisor, attribute)


def _replace_fence(record, mutate):
    """Return a copy of a StoredProfile whose fence bounds are rewritten."""
    import dataclasses
    fence = {arm_id: dict(limits) for arm_id, limits in record.fence.items()}
    for limits in fence.values():
        mutate(limits)
    return dataclasses.replace(record, fence=fence)


def _narrow(record, lower_shift):
    """Lift every lower bound above the measured pose."""
    def mutate(limits):
        limits['position_lower'] = tuple(
            value + lower_shift for value in limits['position_lower'])
    return _replace_fence(record, mutate)


def _snug(record):
    """Leave the pose inside its bounds but inside the activation envelope."""
    def mutate(limits):
        # The measured pose is 0.1 rad at joint 1; put the lower bound just
        # under it so the reserve (5 deg margin + 2 deg drift) cannot fit.
        # Joint 1 sits at 0.0 rad; leave it inside its bounds but with
        # less clearance than the 5 deg margin plus the 2 deg drift limit.
        limits['position_lower'] = (-0.10,) + tuple(limits['position_lower'][1:])
    return _replace_fence(record, mutate)


def running_motion(tmp_path, **settings_extra):
    """
    Return a Motion harness parked in `running`, with the baseline captured.

    The activation-settling gate is exercised end to end in
    ``test_motion_guards``; here the subject is the frame the console reads,
    so the session is placed in `running` once the baseline has genuinely
    been captured ahead of the controller going active.
    """
    h = Harness(tmp_path, **settings_extra)
    h.make_ready_simulate()
    h.start(arms='both', mode='motion')
    for _ in range(4):
        h.supervisor.tick()
    assert h.supervisor._baseline_captured is True
    h.bridge.controllers[defaults.MOTION_CONTROLLER] = 'active'
    # A healthy production snapshot, so a tick in `running` does not fault:
    # the hardware component is the only fault input a fake bridge supplies
    # by omission rather than by absence.
    h.bridge.hardware = {
        'name': 'FrankaMultiHardwareInterface',
        'plugin_name': 'franka_hardware/FrankaMultiHardwareInterface',
        'lifecycle_id': 3, 'lifecycle_label': 'active',
    }
    with h.supervisor._state_lock:
        h.supervisor._state = 'running'
    return h


class TestCommandSource:
    """The per-arm Jog | External switch."""

    def test_arm_sources_start_as_jog(self, harness):
        """Every arm starts a Motion session on the jog source."""
        harness.start(arms='both', mode='motion')
        assert harness.supervisor._arm_source == {'panda1': 'jog', 'panda2': 'jog'}

    def test_fault_entry_resets_every_source_to_jog(self, harness):
        """Fault entry revokes every authorization, sources included."""
        harness.start(arms='both', mode='motion')
        harness.supervisor._arm_source['panda1'] = 'external'
        harness.supervisor._transition('fault', reason=None)
        assert harness.supervisor._arm_source == {'panda1': 'jog', 'panda2': 'jog'}

    def test_session_stop_resets_every_source_to_jog(self, harness):
        """So does teardown."""
        harness.start(arms='both', mode='motion')
        harness.supervisor._arm_source['panda2'] = 'external'
        harness.supervisor._transition('stopping', reason=None)
        assert harness.supervisor._arm_source == {'panda1': 'jog', 'panda2': 'jog'}

    def test_operator_revocation_resets_every_source_to_jog(self, harness):
        """The revocation hook resets sources as well as enables."""
        harness.start(arms='both', mode='motion')
        harness.supervisor._arm_source['panda1'] = 'external'
        harness.supervisor._arm_enabled['panda1'] = True
        harness.supervisor.revoke_operator_authorization()
        assert harness.supervisor._arm_source == {'panda1': 'jog', 'panda2': 'jog'}
        assert harness.supervisor._arm_enabled['panda1'] is False

    def test_external_counters_are_reconciled_from_the_supervisor_tick(
            self, tmp_path):
        """
        The ROS-side subscription follows the flags one tick later.

        That is what makes the lock-free reset in the revocation hook
        sufficient: the hook flips the flag the jog stream reads, and the
        subscription catches up on the next tick.
        """
        h = running_motion(tmp_path)
        assert h.supervisor.state == 'running'
        h.supervisor._arm_source['panda1'] = 'external'
        h.supervisor.tick()
        assert h.bridge.external_counters == {'panda1': 1}
        h.supervisor.revoke_operator_authorization()
        assert h.supervisor._arm_source['panda1'] == 'jog'
        h.supervisor.tick()
        assert h.bridge.external_counters == {}

    def test_external_rate_is_zero_not_null_before_the_counter_is_reconciled(
            self, tmp_path):
        """
        A telemetry gap must never take the whole state surface down.

        `_reconcile_external_counters` swallows its own failure by design, so
        there is a real window in which the source is external and the bridge
        is not counting -- and an unguarded round(None, 1) inside frame()
        would raise on the hot path behind every SSE tick.
        """
        h = running_motion(tmp_path)
        h.bridge.external_counter_error = RuntimeError('no middleware today')
        h.supervisor._arm_source['panda1'] = 'external'
        h.supervisor.tick()
        motion = h.supervisor.frame()['arms']['panda1']['motion']
        assert motion['source'] == 'external'
        assert motion['external_rate_hz'] == 0.0
        assert h.supervisor.frame()['arms']['panda2']['motion'][
            'external_rate_hz'] is None


class TestCommandTopicAndTemplate:
    """The External panel's topic and copyable template come from the server."""

    def test_the_command_topic_uses_the_one_based_session_slot(self, tmp_path):
        """A single-arm panda2 session reports arm_1, not arm_2."""
        h = running_motion(tmp_path)
        topics = {arm_id: arm['motion']['command_topic']
                  for arm_id, arm in h.supervisor.frame()['arms'].items()}
        assert topics == {
            'panda1': '/{}/arm_1/joint_target'.format(defaults.MOTION_CONTROLLER),
            'panda2': '/{}/arm_2/joint_target'.format(defaults.MOTION_CONTROLLER),
        }

    def test_the_template_carries_the_latest_measured_pose(self, tmp_path):
        """A copy-paste publishes a no-op rather than a jump."""
        h = running_motion(tmp_path)
        motion = h.supervisor.frame()['arms']['panda1']['motion']
        assert motion['command_template_ready'] is True
        assert motion['command_template'] == (
            '# trajectory_msgs/msg/JointTrajectory — publish at 10 Hz or more\n'
            'header:\n'
            '  stamp: now\n'
            'joint_names: [panda1_joint1, panda1_joint2, panda1_joint3, '
            'panda1_joint4,\n'
            '              panda1_joint5, panda1_joint6, panda1_joint7]\n'
            'points:\n'
            '- positions: [0.000, -0.785, 0.000, -2.356, 0.000, 1.571, 0.785]'
            '   # rad — one point per message\n'
            '  time_from_start: {sec: 0, nanosec: 0}\n')

    def test_the_template_is_null_outside_motion_mode_but_ready_is_a_boolean(
            self, harness):
        """`command_template_ready` is ALWAYS a boolean, never null."""
        harness.start(arms='both', mode='simulate')
        for _ in range(5):
            harness.supervisor.tick()
        motion = harness.supervisor.frame()['arms']['panda1']['motion']
        assert motion['command_topic'] is None
        assert motion['command_template'] is None
        assert motion['command_template_ready'] is False
        assert motion['source'] is None
        assert motion['available'] is False


class TestHintLine:
    """The persistent next-step line, computed once on the server."""

    def test_the_idle_hint_offers_simulate(self, harness):
        """Nothing has run yet."""
        assert harness.supervisor.frame()['hint'] == (
            'Pick arms and press Start — or choose Simulate to try the '
            'console without robots.')

    def test_the_ended_hint_branches_on_recording_policy(self, tmp_path):
        """The hint can never claim a recording that policy disabled."""
        recorded = Harness(tmp_path / 'on')
        recorded.make_ready_simulate()
        recorded.start()
        for _ in range(8):
            recorded.supervisor.tick()
        recorded.stop()
        for _ in range(3):
            recorded.supervisor.tick()
        assert recorded.supervisor.frame()['hint'] == (
            'Session ended. Recording saved. Start a new session anytime.')

        quiet = Harness(tmp_path / 'off', recording_enabled=False)
        quiet.make_ready_simulate()
        quiet.start()
        for _ in range(8):
            quiet.supervisor.tick()
        quiet.stop()
        for _ in range(3):
            quiet.supervisor.tick()
        assert quiet.supervisor.frame()['hint'] == (
            'Session ended. Start a new session anytime.')
        assert quiet.supervisor.frame()['recording']['disabled'] is True

    @pytest.mark.parametrize('step_id, sentence', [
        ('preflight', 'Preflight…'),
        ('connect:panda1', 'Connecting to panda1…'),
        ('connect:panda2', 'Connecting to panda2…'),
        ('health', 'Health check…'),
        ('baseline', 'Capturing baseline…'),
        ('controller', 'Activating controller…'),
        ('settling', 'Settling check…'),
    ])
    def test_start_phase_hints_come_from_the_step_hint_table_not_the_label(
            self, harness, step_id, sentence):
        """
        The hint is NOT a transform of the display label.

        'Connect panda1' against 'Connecting to panda1...' is not a
        transform of anything, and a mechanical one would produce
        'Connect panda1...'. The server reads the step-id table instead.
        """
        harness.supervisor._steps_init('motion', ('panda1', 'panda2'))
        harness.supervisor._step_active(step_id)
        harness.supervisor._state = 'starting'
        hint = harness.supervisor.frame()['hint']
        assert hint == sentence
        assert hint.endswith('…'), 'the ellipsis is one character'
        assert not hint.endswith('...')

    def test_the_hint_falls_back_to_the_first_pending_step_when_none_is_active(
            self, harness):
        """
        Reachable on any tick: `health` goes done before `baseline` is active.

        The hint is never empty while a session is starting.
        """
        harness.supervisor._steps_init('simulate', ('panda1',))
        harness.supervisor._state = 'starting'
        assert harness.supervisor.frame()['hint'] == 'Preflight…'
        harness.supervisor._step_done('health')
        assert harness.supervisor.frame()['hint'] == 'Capturing baseline…'
        harness.supervisor._step_done('baseline')
        assert harness.supervisor.frame()['hint'] == 'Capturing baseline…'

    def test_recovery_step_hints_match_the_contract_table(self):
        """
        Unit-level, because PART2 keeps `state == 'fault'` for a whole Recover.

        These sentences are therefore unreachable through the hint table --
        the two fault rows win -- but they are built and pinned anyway, so
        moving the session out of `fault` during a recovery stays a one-line
        change rather than a string hunt.
        """
        from franka_web.session import _RECOVERY_STEP_HINTS
        assert _RECOVERY_STEP_HINTS == {
            'reconnect': 'Reconnecting to {arm}…',
            'controller': 'Restarting controller…',
            'verify': 'Verifying fresh data…',
        }

    def test_the_hint_during_a_recover_is_the_fault_row(self, harness):
        """
        The session stays in `fault` for the whole recovery, so this is it.

        This test is what pins WHICH of the two readings PART2 built.
        """
        harness.start(arms='both', mode='motion')
        harness.supervisor._steps_recovery(('panda1', 'panda2'))
        harness.supervisor._state = 'fault'
        assert harness.supervisor.frame()['hint'] == (
            'Check that nobody pressed a stop, then press Recover.')

    def test_the_watch_hint_says_motion_is_impossible(self, tmp_path):
        """Watch is observe-only and says so."""
        h = Harness(tmp_path)
        h.make_ready_simulate()
        h.start(arms='both', mode='watch')
        with h.supervisor._state_lock:
            h.supervisor._state = 'running'
        assert h.supervisor.frame()['hint'] == (
            'Observing only — motion is impossible in Watch. The arm can '
            'be moved by hand.')

    def test_the_running_hints_walk_from_enable_to_jog_to_external(self, tmp_path):
        """Enable, then jog, then the two external rows."""
        h = running_motion(tmp_path)
        assert h.supervisor.frame()['hint'] == 'Enable an arm to allow commands.'
        h.supervisor._arm_enabled['panda1'] = True
        assert h.supervisor.frame()['hint'] == (
            'Jog with the − / + buttons, or switch the source to External '
            'to use your own ROS 2 node.')
        h.supervisor._arm_source['panda1'] = 'external'
        h.supervisor.tick()
        topic = '/{}/arm_1/joint_target'.format(defaults.MOTION_CONTROLLER)
        assert h.supervisor.frame()['hint'] == (
            'Waiting for your publisher on {} — 0.0 Hz'.format(topic))
        h.bridge.external_rates['panda1'] = 20.0
        assert h.supervisor.frame()['hint'] == (
            'Receiving 20.0 Hz from your node. The watchdog freezes the arm '
            'if the stream stops.')

    def test_the_stopping_hint_names_the_seal(self, harness):
        """Stopping says what it is waiting for."""
        harness.start()
        harness.supervisor._state = 'stopping'
        assert harness.supervisor.frame()['hint'] == (
            'Stopping — sealing the recording.')

    def test_the_lock_expired_fault_hint_asks_for_a_reclaim_first(self, harness):
        """A Recover press cannot succeed without the lock."""
        harness.start(arms='both', mode='motion')
        harness.supervisor._state = 'fault'
        harness.supervisor._session['operator_claim_id'] = 'deadbeef'
        assert harness.supervisor.frame()['hint'] == (
            'Press Reclaim to take control back, then Recover.')


class TestFrameAdditions:
    """The two new top-level keys, and the two removed session keys."""

    def test_the_frame_carries_logs_counters(self, harness):
        """The badge numbers and the backfill signal ride every frame."""
        harness.logs.emit('warn', 'something')
        counters = harness.supervisor.frame()['logs']
        assert counters['warn_count'] == 1
        assert counters['error_count'] == 0
        assert counters['last_seq'] >= 1

    def test_the_frame_no_longer_carries_controller_name_or_gains_sha256(
            self, harness):
        """Both keys are gone from the session block entirely."""
        harness.start(arms='both', mode='motion')
        block = harness.supervisor.frame()['session']
        assert 'controller_name' not in block
        assert 'gains_sha256' not in block

    def test_recording_disabled_skips_the_recorder_and_reports_disabled_true(
            self, tmp_path):
        """Policy, not failure: the chip is hidden, the session still runs."""
        h = Harness(tmp_path, recording_enabled=False)
        assert h.supervisor.frame()['recording']['disabled'] is True
        assert h.settings.recording_enabled is False


class TestLaunchOutputReachesTheLogBus:
    """Log source one: the launch child's merged output."""

    def test_launch_child_output_reaches_the_log_bus(self, harness):
        """
        The sink is passed at the launch spawn site, or the drawer is blind.

        This fails the moment `_enter_starting` forgets its `on_line=`.
        """
        harness.start()
        harness.supervisor.tick()
        assert harness.launch_child.on_line is not None
        harness.launch_child.emit(
            '[INFO] [1.0] [controller_manager]: Configured and activated')
        lines = harness.logs.window()['lines']
        from_launch = [line for line in lines if line['node'] != 'franka_web']
        assert from_launch, 'no line reached the bus from the launch child'
        assert from_launch[-1]['node'] == 'controller_manager'

    def test_the_server_emits_its_own_operator_lines(self, harness):
        """Session start and stop are operator-facing events."""
        harness.start()
        messages = [line['message'] for line in harness.logs.window()['lines']]
        assert any(message.startswith('session web-') and 'starting' in message
                   for message in messages)


class TestTorqueCeilingWarning:
    """One warn line at startup when the safety bound left the proven set."""

    class _Bus:
        """A bus that records the (level, message) pairs it is handed."""

        def __init__(self):
            """Start with nothing emitted."""
            self.lines = []

        def emit(self, level, message, **_kwargs):
            """Record one emitted line."""
            self.lines.append((level, message))

    def test_a_changed_torque_ceiling_warns_once_at_startup(self, tmp_path):
        """
        Changed ceilings warn once, naming the arm and both vectors.

        Torque ceilings stay editable, bounded by the Panda hardware ceiling.
        The server does not refuse and does not nag per session.
        """
        from franka_web import server
        settings = make_settings(tmp_path, profiles={
            'panda2': {'torque_limit_nm': [8.0, 8.0, 8.0, 8.0, 4.0, 4.0, 2.0]}})
        bus = self._Bus()
        server._warn_on_changed_torque_ceilings(settings, bus)
        assert len(bus.lines) == 1
        level, message = bus.lines[0]
        assert level == 'warn'
        assert message.startswith('panda2: torque ceilings differ')
        assert '8.0' in message and '10.0' in message

    def test_the_proven_set_warns_about_nothing(self, tmp_path):
        """The line exists for a difference, not for every boot."""
        from franka_web import server
        bus = self._Bus()
        server._warn_on_changed_torque_ceilings(make_settings(tmp_path), bus)
        assert bus.lines == []


class TestSettlingMessages:
    """The teaching sentences: which joint, what value, which key to raise."""

    class _Gate:
        """A gate stand-in carrying only the metrics the messages read."""

        def __init__(self, current):
            """Bind the per-arm metric dict."""
            self.current = current

    @staticmethod
    def _session():
        """Return the two session fields the message helpers read."""
        return {'arm_ids': ['panda1', 'panda2']}

    def _metrics(self, **overrides):
        """Build a clean per-arm metric dict with one family perturbed."""
        zeros = [0.0] * 7
        base = {
            'max_abs_delta_rad': list(zeros),
            'position_span_rad': list(zeros),
            'max_abs_velocity_rad_s': list(zeros),
            'lower_margin_rad': [1.0] * 7,
            'upper_margin_rad': [1.0] * 7,
            'delta_rad': list(zeros),
        }
        base.update(overrides)
        return {'panda1': {key: list(value) for key, value in base.items()},
                'panda2': {key: list(value) for key, value in base.items()}}

    def test_settling_trip_message_names_the_drift_joint_value_and_key(
            self, harness):
        """The worst joint, in degrees, and the key that would allow it."""
        metrics = self._metrics()
        metrics['panda2']['max_abs_delta_rad'][1] = 0.0403   # 2.31 deg
        message = harness.supervisor._settling_trip_message(
            self._Gate(metrics), self._session(),
            'activation_settling_limit', 'drift')
        assert message.startswith('panda2 J2 moved 2.31°; the limit is 2.00°')
        assert 'settling.drift_limit_deg' in message
        assert harness.settings.config_path in message
        assert message.endswith(
            'Check that nothing is pushing the arm, or raise that value.')

    @pytest.mark.parametrize('metric, key, verb', [
        ('position_span_rad', 'settling.span_limit_deg', 'drifted over a window of'),
        ('max_abs_velocity_rad_s', 'settling.velocity_limit_deg_s',
         'was still moving at'),
    ])
    def test_settling_trip_message_names_each_family(
            self, harness, metric, key, verb):
        """Every family says what it measured and which key bounds it."""
        metrics = self._metrics()
        metrics['panda1'][metric][3] = 1.0
        message = harness.supervisor._settling_trip_message(
            self._Gate(metrics), self._session(),
            'activation_settling_limit', 'detail')
        assert 'panda1 J4 {}'.format(verb) in message
        assert key in message

    def test_settling_trip_message_names_the_fence_margin_family(self, harness):
        """The margin must stay ABOVE its limit, so its severity inverts."""
        metrics = self._metrics()
        metrics['panda1']['lower_margin_rad'][5] = 0.001
        message = harness.supervisor._settling_trip_message(
            self._Gate(metrics), self._session(),
            'activation_settling_limit', 'detail')
        assert 'panda1 J6 came within' in message
        assert 'settling.fence_margin_deg' in message

    def test_a_timeout_names_the_three_keys_that_widen_the_window(self, harness):
        """A timeout is fixed by time, not by a limit."""
        metrics = self._metrics()
        metrics['panda1']['max_abs_delta_rad'][0] = 0.05
        message = harness.supervisor._settling_trip_message(
            self._Gate(metrics), self._session(),
            'activation_settling_timeout', 'detail')
        assert message.endswith(
            'Raise settling.timeout_s, or lower settling.min_samples / '
            'settling.stable_window_s.')

    def test_settling_trip_message_falls_back_when_no_sample_was_observed(
            self, harness):
        """With no sample there is no worst joint; the detail still teaches."""
        message = harness.supervisor._settling_trip_message(
            self._Gate({}), self._session(),
            'activation_settling_timeout', 'no sample was ever observed')
        assert message.startswith('no sample was ever observed. ')
        assert 'settling.timeout_s' in message

    def test_settling_success_detail_names_the_worst_joint_in_degrees(
            self, harness):
        """The passing sentence reports the SIGNED settle and the limit."""
        metrics = self._metrics()
        metrics['panda2']['max_abs_delta_rad'][1] = 0.00541
        metrics['panda2']['delta_rad'][1] = 0.00541
        detail = harness.supervisor._settling_success_detail(
            self._Gate(metrics), self._session())
        assert detail == 'panda2 J2 settled +0.31° (limit 2.00°)'

    def test_settling_success_detail_falls_back_without_a_sample(self, harness):
        """No metrics, no joint to name."""
        assert harness.supervisor._settling_success_detail(
            self._Gate({}), self._session()) == (
            'settled within the configured limits')
