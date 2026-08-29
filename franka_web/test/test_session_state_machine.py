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

"""Every §3.4 transition of SessionSupervisor, driven with fakes."""

import os
import threading

from franka_web.config import Settings
from franka_web.preflight import run_preflight
from franka_web.session import SessionError, SessionRequest, SessionSupervisor
import pytest
from sensor_msgs.msg import JointState
from support.fake_clock import FakeClock
from support.fake_launcher import (
    FakeBridge, FakeBroker, FakeChild, FakeLock, FakePreflightResult,
    FakeRecording, FakeSpawner)

DOC_IP_1 = '203.0.113.7'
DOC_IP_2 = '203.0.113.8'


def make_settings(tmp_path, **extra):
    """Build a valid Settings against private tmp directories."""
    state_dir = tmp_path / 'state'
    state_dir.mkdir(mode=0o700)
    recording_root = tmp_path / 'recordings'
    recording_root.mkdir(mode=0o700)
    os.chmod(tmp_path, 0o700)
    env = {
        'FRANKA_WEB_STATE_DIR': str(state_dir),
        'FRANKA_WEB_RECORDING_ROOT': str(recording_root),
        'ROS_DOMAIN_ID': '80',
    }
    env.update(extra)
    return Settings.from_env(env)


def dual_joint_state():
    """Build a complete 14-name dual JointState."""
    msg = JointState()
    for arm in ('panda1', 'panda2'):
        for joint in range(1, 8):
            msg.name.append('{}_joint{}'.format(arm, joint))
            msg.position.append(0.1 * joint)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
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
        self.supervisor = SessionSupervisor(
            self.settings, self.bridge, FakeLock(), self.broker,
            spawn=self._spawn,
            recording_factory=lambda: self.recorder,
            preflight_runner=lambda settings, mode: self.preflight,
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
                    SessionRequest(arms=arms, mode=mode))
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
        assert frame['schema_version'] == 1


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

    def test_motion_without_controller_refused(self, harness):
        """Stage 2: motion without a controller_name is refused up front."""
        with pytest.raises(SessionError) as excinfo:
            harness.start(mode='motion')
        assert excinfo.value.code == 'controller_not_reviewed'

    def test_unknown_mode_refused(self, harness):
        """A mode outside the closed set is invalid_mode."""
        with pytest.raises(SessionError) as excinfo:
            harness.start(mode='teleop')
        assert excinfo.value.code == 'invalid_mode'

    def test_watch_without_addresses_refused(self, harness):
        """robot_addresses_missing, and no address in the message."""
        with pytest.raises(SessionError) as excinfo:
            harness.start(mode='watch')
        assert excinfo.value.code == 'robot_addresses_missing'
        assert DOC_IP_1 not in excinfo.value.detail

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
            preflight=FakePreflightResult(overall='FAIL', passed=False, blocking=True),
            **{'FRANKA_WEB_ROBOT_IP_1': DOC_IP_1, 'FRANKA_WEB_ROBOT_IP_2': DOC_IP_2})
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
            tmp_path,
            **{'FRANKA_WEB_ROBOT_IP_1': DOC_IP_1, 'FRANKA_WEB_ROBOT_IP_2': DOC_IP_2})
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
            preflight=FakePreflightResult(overall='FAIL', passed=False, blocking=True),
            **{'FRANKA_WEB_ROBOT_IP_1': DOC_IP_1, 'FRANKA_WEB_ROBOT_IP_2': DOC_IP_2})
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
