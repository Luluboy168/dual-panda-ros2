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

import builtins
import json
import os
import pathlib
import threading
import time
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticStatus
from franka_msgs.msg import FrankaState
from franka_web import defaults, health, travel, workspace
from franka_web.gains import ProfileStore
from franka_web.launcher import LauncherError
from franka_web.lock import OperatorLock
from franka_web.logbus import LogBus
from franka_web.preflight import run_preflight
from franka_web.recording import RecordingError
from franka_web.session import (
    expected_broadcasters, SessionError, SessionRequest, SessionSupervisor)
import pytest
from sensor_msgs.msg import JointState
from support.config_factory import DOC_IP_1, DOC_IP_2, make_settings
from support.fake_checker import (
    checker_holding, CheckResult, Contact, FakeCellModel)
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


def dual_joint_state_pair(panda1, panda2):
    """Build a dual JointState with a different pose on each arm."""
    msg = JointState()
    for arm, positions in (('panda1', panda1), ('panda2', panda2)):
        for joint in range(1, 8):
            msg.name.append('{}_joint{}'.format(arm, joint))
            msg.position.append(float(positions[joint - 1]))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
    return msg


def healthy_robot_state():
    """Build a FrankaState no fault rule fires on (move mode, high CCSR)."""
    message = FrankaState()
    message.robot_mode = 2
    message.control_command_success_rate = 0.998
    return message


def healthy_diagnostic(arm_id):
    """Build the canonical per-arm DiagnosticStatus at level OK."""
    status = DiagnosticStatus()
    status.name = health.canonical_diagnostic_name(arm_id)
    status.hardware_id = arm_id
    status.level = bytes([0])
    status.message = 'backend state is healthy'
    return status


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

    def __init__(self, tmp_path, preflight=None, recorder=None, checker=None,
                 **settings_extra):
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
        self.sample_waits_remaining = 2
        self.sample_wait_calls = []
        self.next_publication = None
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
            checker=checker,
            monotonic=self.clock.monotonic,
            recovery_wait=self.wait_for_samples,
        )
        self.checker = checker
        #: Every switch-spacing slice the supervisor spent, in fake seconds.
        self.dwell_waits = []
        # Assigned rather than passed: `_switch_dwell_wait` is deliberately
        # not a constructor keyword, so this rig can also drive a build
        # without the dwell (see test_motion_guards.TestSwitchDwell).
        self.supervisor._switch_dwell_wait = self.wait_switch_dwell

    def wait_switch_dwell(self, timeout_s):
        """
        Spend one switch-spacing slice of fake time, publishers still running.

        A separate seam from :meth:`wait_for_samples`, and it spends none of
        that scripted budget: the reviewed spacing between two
        ``switch_controller`` calls is time the driver needs, not a
        publication a test is choosing to grant.
        """
        self.dwell_waits.append(timeout_s)
        self.clock.advance(timeout_s)
        stamp = self.clock.monotonic_ns()
        if self.bridge.joint is not None:
            self.bridge.set_joint_sample(stamp, self.bridge.joint[1])
        self.bridge.robot_states = {
            arm_id: (stamp, sample[1])
            for arm_id, sample in self.bridge.robot_states.items()}
        self.bridge.diagnostics = {
            arm_id: (stamp, sample[1])
            for arm_id, sample in self.bridge.diagnostics.items()}
        return True

    def wait_for_samples(self, timeout_s):
        """
        Advance one scripted publisher cycle, then report no further updates.

        The Motion start restages the impedance controller and waits for a
        joint sample strictly newer than its own deactivation, so this seam
        is what a fake clock needs to supply one. ``sample_waits_remaining``
        is the budget; zeroing it is how a test says "the publisher stopped",
        and ``next_publication`` is how one says "and THIS is what arrives in
        the paused window".
        """
        self.sample_wait_calls.append(timeout_s)
        if self.sample_waits_remaining <= 0:
            return False
        self.sample_waits_remaining -= 1
        self.clock.advance(0.001)
        stamp = self.clock.monotonic_ns()
        if self.next_publication is not None:
            self.bridge.set_joint_sample(stamp, self.next_publication())
        elif self.bridge.joint is not None:
            self.bridge.set_joint_sample(stamp, self.bridge.joint[1])
        self.bridge.robot_states = {
            arm_id: (stamp, sample[1])
            for arm_id, sample in self.bridge.robot_states.items()}
        self.bridge.diagnostics = {
            arm_id: (stamp, sample[1])
            for arm_id, sample in self.bridge.diagnostics.items()}
        return True

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

    def make_ready_motion(self, arm_ids=('panda1', 'panda2'), arm_mode='dual'):
        """
        Satisfy every §3.5 Motion readiness criterion, controller included.

        The impedance controller is ACTIVE here from the start, which is what
        the reviewed launch's ``spawner --switch-asap`` really does; the
        server's restage is what creates the torque-free window it measures
        the baseline in.
        """
        controllers = {name: 'active'
                       for name in expected_broadcasters(arm_ids, arm_mode)}
        controllers[defaults.MOTION_CONTROLLER] = 'active'
        self.bridge.controllers = controllers
        self.bridge.types = {name: 'franka_example_controllers/Stub'
                             for name in controllers}
        stamp = self.clock.monotonic_ns()
        for arm_id in arm_ids:
            self.bridge.robot_states[arm_id] = (stamp, healthy_robot_state())
            self.bridge.diagnostics[arm_id] = (stamp, healthy_diagnostic(arm_id))
        self.bridge.hardware = {
            'name': 'FrankaMultiHardwareInterface',
            'plugin_name': 'franka_hardware/FrankaMultiHardwareInterface',
            'lifecycle_id': 3, 'lifecycle_label': 'active',
        }
        self.bridge.joint = (stamp, dual_joint_state())

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

    def drive(self, call, ticks=50):
        """
        Submit one operator command the way an HTTP worker does, and answer it.

        The command is enqueued from another thread; this thread ticks the
        supervisor until the verdict arrives. A refusal is re-raised here, so a
        test reads it with ``pytest.raises`` exactly as the HTTP layer would.
        """
        result = {}

        def submit():
            """Run the call on its own thread and record what it answered."""
            try:
                result['value'] = call()
            except SessionError as error:
                result['error'] = error
        thread = threading.Thread(target=submit)
        thread.start()
        for _ in range(ticks):
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


class TestRecordingRetentionOnStop:
    """The size cap is wired into the stop path, not just into a module."""

    @staticmethod
    def seed(root, name, size_bytes, sealed=True):
        """Write one fake sealed session directory of a known apparent size."""
        bag = os.path.join(root, name, 'bag')
        os.makedirs(bag, exist_ok=True)
        if sealed:
            with open(os.path.join(bag, 'metadata.yaml'), 'w',
                      encoding='utf-8') as handle:
                handle.write('rosbag2_bagfile_information: {}\n')
        with open(os.path.join(bag, 'bag_0.mcap'), 'wb') as handle:
            handle.truncate(size_bytes)

    @staticmethod
    def retention_lines(harness):
        """Return the retention lines the harness's log bus captured."""
        return [line['message'] for line in harness.logs.window()['lines']
                if line['message'].startswith('retention: ')]

    def run_session(self, harness):
        """Start and stop one simulate session on this harness."""
        harness.make_ready_simulate()
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'running'
        harness.stop()
        for _ in range(3):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'

    def test_sealing_a_session_over_the_cap_removes_the_oldest_and_says_so(
            self, tmp_path):
        """
        The pass the operator was promised runs when the bag is sealed.

        Without the call in ``_do_stopping`` the module is correct and the
        disk still fills, which is exactly the bug this pins.
        """
        harness = Harness(tmp_path, max_total_gb=0.001)
        root = harness.settings.recording_root
        self.seed(root, 'web-20200101-000001', 800000)
        self.seed(root, 'web-20200102-000002', 800000)
        self.run_session(harness)
        assert not os.path.exists(os.path.join(root, 'web-20200101-000001'))
        assert os.path.isdir(os.path.join(root, 'web-20200102-000002'))
        lines = self.retention_lines(harness)
        assert lines[0].startswith(
            'retention: removed web-20200101-000001 ('), lines
        assert lines[-1].startswith('retention: 1 sessions hold '), lines

    def test_unlimited_leaves_every_recording_where_it_is(self, tmp_path):
        """The documented off switch reaches the stop path too."""
        harness = Harness(tmp_path, max_total_gb='unlimited')
        root = harness.settings.recording_root
        self.seed(root, 'web-20200101-000001', 800000)
        self.seed(root, 'web-20200102-000002', 800000)
        self.run_session(harness)
        assert sorted(os.listdir(root)) == ['web-20200101-000001',
                                            'web-20200102-000002']
        assert self.retention_lines(harness)[-1].startswith(
            'retention: no size cap is set')

    def test_the_session_just_sealed_survives_its_own_stop(self, tmp_path):
        """
        The recording the operator has this second stopped is protected.

        Nothing else in the pass would spare it: the live directory is safe
        only while `self._recording` holds it, and by the time the pass runs
        the recorder has been dropped. Without the sealed name the stop that
        ends an overnight Watch larger than the cap eats the front of the
        session it has just finished writing -- so this seeds a chain over
        the cap, stops it, and requires every segment to still be there.
        """
        harness = Harness(tmp_path, max_total_gb=0.001)
        root = harness.settings.recording_root
        self.seed(root, 'web-20200101-000001', 800000)
        harness.make_ready_simulate()
        harness.start()
        for _ in range(5):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'running'
        # The session's own chain, on disk under the name the recorder was
        # started with, and twice the cap all by itself.
        name = harness.recorder.started[0]
        self.seed(root, name, 1000000)
        self.seed(root, name + '-002', 1000000)
        harness.stop()
        for _ in range(3):
            harness.supervisor.tick()
        assert harness.supervisor.state == 'stopped'
        assert sorted(os.listdir(root)) == [name, name + '-002']
        lines = self.retention_lines(harness)
        assert lines[0].startswith(
            'retention: removed web-20200101-000001 ('), lines
        assert lines[-1].endswith(
            'still above the cap, and nothing else may be removed'), lines

    def test_a_session_that_sealed_nothing_runs_no_pass(self, tmp_path):
        """
        Recording switched off means there is no new bag and nothing to do.

        The claim is scoped to the STOP path: nothing was sealed, so the
        total cannot have changed, so no pass runs. It is NOT a claim that
        `recording.enabled: false` shields old bags from the cap -- the
        startup pass is unconditional and does not read that key. The cap is
        the cap: a server that boots over it trims to it even when it is
        deliberately recording nothing itself.
        """
        harness = Harness(tmp_path, recorder=FakeRecording(disabled=True),
                          recording_enabled=False, max_total_gb=0.001)
        root = harness.settings.recording_root
        self.seed(root, 'web-20200101-000001', 800000)
        self.seed(root, 'web-20200102-000002', 800000)
        self.run_session(harness)
        assert sorted(os.listdir(root)) == ['web-20200101-000001',
                                            'web-20200102-000002']
        assert self.retention_lines(harness) == []


class TestTheStartupPassIsGuardedByThePidfile:
    """A second server the pidfile refuses must remove nothing at all."""

    def test_a_refused_second_launch_leaves_every_recording_where_it_is(
            self, tmp_path, capsys):
        """
        The guard exists to make a mistyped second start a harmless no-op.

        With the pass ahead of ``pidfile.acquire()`` this is a data-loss bug:
        the doomed process trims the root to the cap -- including the already
        SEALED earlier segments of the chain the real server is recording
        right now -- and only then refuses to start. Seeded three sessions
        against a cap that fits one; the refused launch must return 1 with
        all three still there.
        """
        from franka_web import server
        from franka_web.launcher import PidfileLock
        settings = make_settings(tmp_path, max_total_gb=0.001)
        root = settings.recording_root
        seeds = ['web-20200101-000001', 'web-20200102-000002',
                 'web-20200103-000003']
        for name in seeds:
            TestRecordingRetentionOnStop.seed(root, name, 800000)
        held = PidfileLock(os.path.join(settings.state_dir, 'franka_web.pid'))
        held.acquire()
        try:
            assert server.serve(settings) == 1
        finally:
            held.release()
        assert sorted(os.listdir(root)) == seeds
        assert 'another franka_web server is running' in capsys.readouterr().err


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
        assert frame['schema_version'] == 5


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
        """
        A FAIL with no named checks invents nothing.

        The detail is the verdict sentence and nothing else: no check name, no
        parenthesised invocation error (a FAIL has none), and no advice about
        a config key nothing blamed.
        """
        h = Harness(
            tmp_path,
            preflight=FakePreflightResult(overall='FAIL', passed=False, blocking=True))
        h.start(mode='watch')
        for _ in range(5):
            h.supervisor.tick()
        detail = h.supervisor.frame()['session']['last_error']['detail']
        assert detail == 'the real-time preflight returned FAIL'
        assert 'franka_dir' not in detail

    @staticmethod
    def _scripted_report(name, summary, evidence=''):
        """Return a runner that answers with one failing check, as JSON."""
        report = json.dumps({
            'overall': 'fail',
            'checks': [
                {'status': 'pass', 'name': 'kernel', 'summary': 'PREEMPT_RT'},
                {'status': 'fail', 'name': name, 'summary': summary,
                 'evidence': evidence},
            ],
        })

        def runner(argv, **kwargs):
            return SimpleNamespace(returncode=1, stdout=report, stderr='')

        return runner

    #: The live 2026-09-01 refusal, as the tool actually reported it.
    LIBFRANKA_CHECK = ('build environment',
                       'Franka_DIR not supplied, libfranka could not be '
                       'identified')

    def _refused_watch_start(self, tmp_path, runner, **settings_extra):
        """Run a real preflight over ``runner`` and return the refusal detail."""
        h = Harness(tmp_path, **settings_extra)
        h.preflight = run_preflight(h.settings, 'watch', runner=runner)
        assert h.preflight.blocks_start()
        h.start(mode='watch')
        for _ in range(5):
            h.supervisor.tick()
        assert h.supervisor.state == 'stopped'
        return h.supervisor.frame()['session']['last_error']['detail']

    def test_a_failed_check_is_named_in_the_preflight_refusal(self, tmp_path):
        """
        The refusal names the check and its summary, not just the verdict.

        `"RT preflight failed: FAIL"` told the live operator nothing at all;
        the tool knew exactly which check failed and why.
        """
        name, summary = self.LIBFRANKA_CHECK
        detail = self._refused_watch_start(
            tmp_path, self._scripted_report(name, summary))
        assert detail.startswith('the real-time preflight returned FAIL')
        assert name in detail
        assert summary in detail

    def test_the_preflight_refusal_teaches_the_franka_dir_key_when_it_is_unset(
            self, tmp_path):
        """The one config key that fixes it is named, with where to write it."""
        name, summary = self.LIBFRANKA_CHECK
        detail = self._refused_watch_start(
            tmp_path, self._scripted_report(name, summary))
        assert 'directories.franka_dir' in detail
        assert '~/.config/franka_web/config.yaml' in detail

    def test_the_franka_dir_hint_is_withheld_when_the_key_is_already_set(
            self, tmp_path):
        """Never tell an operator to set what they have already set."""
        name, summary = self.LIBFRANKA_CHECK
        detail = self._refused_watch_start(
            tmp_path, self._scripted_report(name, summary),
            franka_dir=str(tmp_path))
        assert name in detail
        assert 'directories.franka_dir' not in detail

    def test_a_memlock_only_failure_does_not_blame_franka_dir(self, tmp_path):
        """A failure about something else must not name an unrelated key."""
        detail = self._refused_watch_start(
            tmp_path,
            self._scripted_report('memlock limit',
                                  'RLIMIT_MEMLOCK is 64 kB, unlimited required',
                                  evidence='ulimit -l'))
        assert 'memlock limit' in detail
        assert 'franka_dir' not in detail

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
        """Simulate and Watch stop at `baseline`; Motion adds four more."""
        harness.start(arms='both', mode='simulate')
        assert step_ids(harness.supervisor.frame()) == [
            'preflight', 'connect:panda1', 'connect:panda2', 'health', 'baseline']
        harness.supervisor._steps_init('motion', ('panda2',))
        assert step_ids(harness.supervisor.frame()) == [
            'preflight', 'connect:panda2', 'health', 'stack_ready',
            'controller_pause', 'baseline', 'controller', 'settling']

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

        The start request is submitted the way an HTTP worker does, but
        readiness is withheld until the caller wants it, so the tick that
        runs the restage -- pause the controller, measure, hand the arms
        back -- is the caller's own. These tests are about exactly what
        happens on that tick.
        """
        h = Harness(tmp_path)
        h.start(arms='both', mode='motion')
        for _ in range(4):
            h.supervisor.tick()
        assert h.supervisor.state == 'starting'
        assert h.supervisor._baseline_captured is False
        if ready:
            h.make_ready_motion()
        return h

    def test_baseline_is_captured_from_the_first_fresh_complete_sample(self, tmp_path):
        """The captured pose is the measured one, per arm."""
        h = self._motion_harness(tmp_path)
        h.supervisor.tick()
        assert h.supervisor._baseline_captured is True
        baseline = h.supervisor._activation_baseline
        assert set(baseline) == {'panda1', 'panda2'}
        assert baseline['panda1'] == HOME_POSE

    def test_the_baseline_is_measured_while_the_controller_is_inactive(
            self, tmp_path):
        """
        The captured pose is the PAUSED one, never the pose under torque.

        The two poses are made different on purpose: the sample published
        while the impedance controller was still active carries one value,
        and the sample the paused window produces carries another. Capturing
        before the deactivate -- or skipping it -- takes the torqued pose.
        """
        torqued = tuple(value + 0.05 for value in HOME_POSE)
        h = self._motion_harness(tmp_path)
        h.bridge.joint = (h.clock.monotonic_ns(), dual_joint_state(torqued))
        # What the arms are actually resting at once nothing commands them.
        h.next_publication = lambda: dual_joint_state(HOME_POSE)

        h.supervisor.tick()

        assert h.supervisor._baseline_captured is True
        assert h.supervisor._activation_baseline['panda1'] == HOME_POSE
        assert h.supervisor._activation_baseline['panda1'] != torqued
        assert h.bridge.controllers[defaults.MOTION_CONTROLLER] == 'active'

    def test_baseline_capture_waits_for_the_health_step(self, tmp_path):
        """Nothing is captured before the graph is healthy."""
        h = Harness(tmp_path)
        h.start(arms='both', mode='motion')       # no joint sample at all
        h.supervisor.tick()
        h.supervisor.tick()
        assert h.supervisor._baseline_captured is False
        assert h.supervisor.state == 'starting'

    @staticmethod
    def _reactivate_after_the_pause_is_verified(h):
        """
        Model the controller coming back active under the paused server.

        The restage deactivates and verifies; from the NEXT controller-manager
        query onwards the controller reports ``active`` again -- which is the
        one thing that makes any pose measured afterwards a POST-activation
        pose.
        """
        controller = defaults.MOTION_CONTROLLER
        seen = {'verification': False}
        original_deactivate = h.bridge.call_switch_deactivate
        original_query = h.bridge.query_controller_states

        def call_switch_deactivate(controllers, timeout_s=5.0):
            response = original_deactivate(controllers, timeout_s)
            seen['verification'] = None     # the next query verifies the pause
            return response

        def query_controller_states(timeout_s=5.0):
            states = original_query(timeout_s)
            if seen['verification'] is None:
                seen['verification'] = True
            elif seen['verification']:
                states = dict(states)
                states[controller] = 'active'
            return states

        h.bridge.call_switch_deactivate = call_switch_deactivate
        h.bridge.query_controller_states = query_controller_states

    @pytest.mark.parametrize('fresh_sample', [False, True],
                             ids=['stale-sample', 'fresh-sample'])
    def test_the_controller_state_test_gates_unconditionally(
            self, tmp_path, fresh_sample):
        """
        The controller-state test gates UNCONDITIONALLY, fresh sample included.

        It is read and acted on BEFORE the sample is even looked at, let
        alone judged fresh. If the controller came back active under the
        paused server, every pose from then on is a POST-activation pose:
        the settling gate would measure drift from an already-torqued arm,
        the activation jump would read as about zero, and the gate would pass
        trivially.

        Moving the controller-state check after the freshness test must fail
        the STALE case (it would report a missing sample instead); if it does
        not, this test is not testing the property.
        """
        h = self._motion_harness(tmp_path)
        self._reactivate_after_the_pause_is_verified(h)
        if not fresh_sample:
            # No further publication will ever arrive.
            h.sample_waits_remaining = 0
        for _ in range(3):
            h.supervisor.tick()
        assert h.supervisor._baseline_captured is False
        assert h.supervisor.state in ('stopping', 'stopped')
        failed = [frame for event, frame in h.broker.events
                  if event == 'state'
                  and any(entry['id'] == 'baseline' and entry['status'] == 'failed'
                          for entry in frame['session']['steps'])]
        assert failed, 'no published frame carried the failed baseline step'
        last_error = failed[-1]['session']['last_error']
        assert last_error['code'] == 'activation_settling_limit'
        assert last_error['detail'] == (
            'the impedance controller became active before a pre-activation '
            'baseline could be captured')
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

    @pytest.mark.parametrize('build', [torn_joint_state, non_finite_joint_state],
                             ids=['incomplete', 'non-finite'])
    def test_a_torn_or_non_finite_pose_fails_the_restage_closed(
            self, tmp_path, build):
        """
        A torn or non-finite pose can never become the Motion baseline.

        The whole activation-settling argument is measured against this pose,
        so an arm gone from the 14-name message -- or a NaN position -- stops
        the session rather than arming a gate against nonsense. Motion is the
        one mode that refuses rather than waiting: the controller has already
        been paused, so there is no "try again next tick" that leaves the arms
        uncommanded for an unbounded time.

        Readiness itself already requires a complete sample, so the torn one
        is the publication that lands INSIDE the paused window -- the only
        place it can still reach the capture.
        """
        h = self._motion_harness(tmp_path)
        h.next_publication = build

        h.supervisor.tick()

        assert h.supervisor._baseline_captured is False
        assert h.supervisor._activation_baseline is None
        assert h.supervisor.state in ('stopping', 'stopped')
        failed = [frame for event, frame in h.broker.events
                  if event == 'state'
                  and frame['session']['last_error'] is not None]
        assert failed, 'no published frame carried the refusal'
        last_error = failed[-1]['session']['last_error']
        assert last_error['code'] == 'activation_settling_limit'
        assert 'pre-activation pose' in last_error['detail']

    @pytest.mark.parametrize('mode', ['watch'])
    @pytest.mark.parametrize('build', [torn_joint_state, non_finite_joint_state],
                             ids=['incomplete', 'non-finite'])
    def test_a_torn_or_non_finite_sample_is_never_captured(
            self, tmp_path, mode, build):
        """
        An arm dropping out of the 14-name sample cannot become the baseline.

        Outside Motion nothing has been paused, so a fresh-but-torn sample --
        one arm gone from the message, or a NaN position -- leaves the capture
        untaken and the session in `starting`, without raising out of the
        tick. The next good sample still captures: this refuses a sample, it
        does not latch a failure.
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


def running_motion(tmp_path, **kwargs):
    """
    Return a Motion harness parked in `running`, with the baseline captured.

    The activation-settling gate is exercised end to end in
    ``test_motion_guards``; here the subject is the frame the console reads,
    so the session is placed in `running` once the restage has genuinely
    captured the baseline with the controller paused.
    """
    h = Harness(tmp_path, **kwargs)
    # The real launch order: the impedance controller is already active.
    h.make_ready_motion()
    h.start(arms='both', mode='motion')
    for _ in range(4):
        h.supervisor.tick()
    assert h.supervisor._baseline_captured is True
    assert h.bridge.controllers[defaults.MOTION_CONTROLLER] == 'active'
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

    def test_the_ended_hint_branches_on_whether_a_recording_sealed(self, tmp_path):
        """The hint can never claim a recording that was never written."""
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

        quiet = Harness(tmp_path / 'off', recording_enabled=False,
                        recorder=FakeRecording(disabled=True))
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

    def test_the_frame_publishes_the_same_sealed_evidence_the_hint_uses(
            self, tmp_path):
        """
        ``session.recording_sealed`` and the hint branch on ONE fact.

        The console's stopped card used to key its "The recording was saved."
        on ``recording.disabled``, which answers a different question --
        whether recording is switched off in the configuration -- so a start
        refused at preflight, which adopts no recorder at all, was told its
        recording had been saved (live finding V2L-2). The card now reads this
        field, and this test is what keeps the two answers from drifting.
        """
        recorded = Harness(tmp_path / 'sealed')
        recorded.make_ready_simulate()
        recorded.start()
        for _ in range(8):
            recorded.supervisor.tick()
        # Nothing is claimed while the session is still running.
        assert recorded.supervisor.frame()['session']['recording_sealed'] is False
        recorded.stop()
        for _ in range(3):
            recorded.supervisor.tick()
        frame = recorded.supervisor.frame()
        assert frame['session']['recording_sealed'] is True
        assert frame['hint'] == (
            'Session ended. Recording saved. Start a new session anytime.')

        refused = Harness(tmp_path / 'refused', preflight=FakePreflightResult(
            overall='FAIL', passed=False, blocking=True))
        refused.start(mode='watch')
        for _ in range(6):
            refused.supervisor.tick()
        frame = refused.supervisor.frame()
        assert refused.supervisor.state == 'stopped'
        assert frame['session']['recording_sealed'] is False
        assert frame['recording']['disabled'] is False, (
            'the config still has recording enabled; disabled is the WRONG key')
        assert frame['hint'] == 'Session ended. Start a new session anytime.'

    def test_the_ended_hint_says_nothing_about_recording_when_none_sealed(
            self, tmp_path):
        """
        A preflight-refused session saved nothing and must not claim otherwise.

        This is the live case: the console told an operator "Recording saved"
        after a start that was refused before a recorder was ever adopted.
        """
        h = Harness(tmp_path, preflight=FakePreflightResult(
            overall='FAIL', passed=False, blocking=True))
        h.start(mode='watch')
        for _ in range(6):
            h.supervisor.tick()
        assert h.supervisor.state == 'stopped'
        assert h.events == [], 'nothing was ever spawned, recorder included'
        assert h.supervisor.frame()['hint'] == (
            'Session ended. Start a new session anytime.')

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


# ----------------------------------------------------------------------
# Grippers: the absences first, then the wiring that remains
# ----------------------------------------------------------------------

GRIPPER_SERIAL_ID = 'usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0'

HEALTHY_GRIPPER_VALUES = {
    'width_mm': '84.7', 'requested_width_mm': '85.0', 'object': 'at_position',
    'activated': 'true', 'moving': 'false', 'fault_code': '0x00',
    'fault_name': 'no_fault', 'fault_class': 'none', 'current_ma': '120',
    'speed_mm_s': '85.0', 'force_n': '74.0', 'port': GRIPPER_SERIAL_ID,
    'link': 'up',
}


def gripper_status(values=None, *, message='Open 84.7 mm.', level=0):
    """Build one gripper ~/status sample the way the node publishes it."""
    from diagnostic_msgs.msg import KeyValue

    merged = dict(HEALTHY_GRIPPER_VALUES)
    merged.update(values or {})
    status = DiagnosticStatus()
    status.name = 'panda1 Robotiq 2F-85'
    status.hardware_id = merged['port']
    status.level = level
    status.message = message
    status.values = [KeyValue(key=key, value=value)
                     for key, value in merged.items()]
    return status


def enable_grippers(harness, *arm_ids):
    """
    Turn on the given arms' grippers on an already-built harness.

    Applied to the loaded Settings rather than written into the config file:
    enabling one through the loader would reach the guarded franka_robotiq
    import, and this file is about the supervisor.
    """
    from dataclasses import replace

    from franka_web.config import GripperConfig

    grippers = {arm_id: GripperConfig(arm_id=arm_id,
                                      enabled=arm_id in arm_ids,
                                      serial_id=(GRIPPER_SERIAL_ID
                                                 if arm_id in arm_ids else ''),
                                      from_file=True)
                for arm_id in defaults.ARM_IDS}
    settings = replace(harness.settings, grippers=grippers)
    harness.settings = settings
    harness.supervisor._settings = settings
    return settings


def request_gripper(harness, arm_id, action, width_mm=None):
    """Submit one gripper request and process it on the supervisor thread."""
    result = {}

    def submit():
        """Ask the supervisor and record the verdict either way."""
        try:
            result['value'] = harness.supervisor.request_gripper_action(
                arm_id, action, width_mm,
                operator_lease=harness.operator_lease)
        except SessionError as error:
            result['error'] = error
    thread = threading.Thread(target=submit)
    thread.start()
    for _ in range(50):
        if 'value' in result or 'error' in result:
            break
        harness.supervisor.tick()
    thread.join(timeout=2)
    if 'error' in result:
        raise result['error']
    return result.get('value')


def run_to_running(harness, arms='both', mode='simulate'):
    """Satisfy this mode's readiness criteria, start, and tick to ``running``."""
    arm_ids = defaults.ARM_IDS if arms == 'both' else (arms,)
    arm_mode = 'dual' if arms == 'both' else 'single'
    if mode != 'simulate':
        harness.make_ready_motion(arm_ids=arm_ids, arm_mode=arm_mode)
    harness.start(arms=arms, mode=mode)
    for _ in range(20):
        harness.supervisor.tick()
        if harness.supervisor.state == 'running':
            break
    assert harness.supervisor.state == 'running', harness.supervisor.state


class TestGripperAbsences:
    """The standing-node design is mostly a set of absences, and they need tests."""

    def test_no_gripper_child_is_ever_spawned(self, harness):
        """Exactly one child for a session with both grippers configured."""
        enable_grippers(harness, 'panda1', 'panda2')
        run_to_running(harness)
        assert len(harness.spawner.spawned) == 1
        assert harness.spawner.spawned[0]['name'] == 'launch'

    def test_the_supervisor_writes_no_gripper_params_file(self, harness, tmp_path):
        """Nothing appears under <state_dir>/grippers/, because there is no such thing."""
        enable_grippers(harness, 'panda1', 'panda2')
        run_to_running(harness)
        assert not os.path.exists(os.path.join(harness.settings.state_dir,
                                               'grippers'))

    def test_stopping_touches_no_gripper_anything(self, harness):
        """The teardown path makes no gripper call at all."""
        enable_grippers(harness, 'panda1', 'panda2')
        run_to_running(harness)
        harness.bridge.gripper_events = []
        harness.stop()
        for _ in range(20):
            harness.supervisor.tick()
            if harness.supervisor.state == 'stopped':
                break
        assert harness.supervisor.state == 'stopped'
        assert harness.bridge.gripper_events == []

    def test_a_running_gripper_node_is_not_a_survivor(self, harness):
        """A standing node outlives the session BY DESIGN; no scan calls it a leak."""
        enable_grippers(harness, 'panda1', 'panda2')
        run_to_running(harness)
        harness.stop()
        for _ in range(20):
            harness.supervisor.tick()
            if harness.supervisor.state == 'stopped':
                break
        names = [entry['name'] for entry in harness.spawner.spawned]
        assert 'gripper' not in ' '.join(names)
        assert names == ['launch']

    def test_launcher_module_is_not_imported_for_any_gripper_path(self):
        """The guardian is byte-untouched, and session.py never reaches for it."""
        import inspect

        from franka_web import session as session_module

        source = inspect.getsource(session_module)
        head, _marker, tail = source.partition('def _accept_gripper')
        gripper_source = tail.partition('\n    def _reseed_jog_model')[0]
        assert 'launcher' not in gripper_source
        assert 'ChildProcess' not in gripper_source
        assert '_spawn' not in gripper_source
        assert 'gripper' in head


class TestGripperWiring:
    """The state-frame block, the refusal ladder and the dispatch."""

    def test_configure_session_receives_the_gripper_arms_and_only_them(self, harness):
        """Only the arms with a gripper enabled reach the bridge."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, mode='watch')
        assert harness.bridge.gripper_arms == ('panda1',)

    def test_simulate_passes_no_gripper_arms_at_all(self, harness):
        """Simulate gets no gripper surface, whatever the config file says."""
        enable_grippers(harness, 'panda1', 'panda2')
        run_to_running(harness, mode='simulate')
        assert harness.bridge.gripper_arms == ()

    def test_the_frame_carries_a_gripper_block_for_every_arm_configured_or_not(
            self, harness):
        """The block is always present; `configured` is what varies."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, mode='watch')
        arms = harness.supervisor.frame()['arms']
        assert arms['panda1']['gripper']['configured'] is True
        assert arms['panda2']['gripper']['configured'] is False
        assert arms['panda2']['gripper']['status_line'] == (
            'No gripper is configured for panda2.')

    def test_a_simulate_frame_carries_configured_false_even_when_the_file_enables_it(
            self, harness):
        """The short-circuit is at the frame builder, before the bridge."""
        enable_grippers(harness, 'panda1', 'panda2')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        run_to_running(harness, mode='simulate')
        for arm_id in defaults.ARM_IDS:
            block = harness.supervisor.frame()['arms'][arm_id]['gripper']
            assert block['configured'] is False
            assert block['available'] is False
            assert block['width_mm'] is None

    def test_a_configured_arm_with_no_node_running_says_start_it_with_ros2_launch(
            self, harness):
        """The sentence teaches the one command that fixes it."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, mode='watch')
        block = harness.supervisor.frame()['arms']['panda1']['gripper']
        assert block['available'] is False
        assert 'ros2 launch franka_robotiq dual_robotiq.launch.py' in \
            block['status_line']

    def test_the_frame_reads_the_nodes_live_speed_and_force(self, harness):
        """The page shows what the running node has, not this server's copy."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, mode='watch')
        harness.bridge.set_gripper_status(
            'panda1', harness.clock.monotonic_ns(),
            gripper_status({'speed_mm_s': '30.0', 'force_n': '40.0'}))
        block = harness.supervisor.frame()['arms']['panda1']['gripper']
        assert block['speed_mm_s'] == pytest.approx(30.0)
        assert block['force_n'] == pytest.approx(40.0)
        assert harness.settings.gripper('panda1').force_n == pytest.approx(74.0)

    def test_busy_is_set_on_dispatch_and_cleared_by_the_result(self, harness):
        """The frame's busy follows the bridge's in-flight flag."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, mode='watch')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        harness.bridge.gripper_busy_arms.add('panda1')
        assert harness.supervisor.frame()['arms']['panda1']['gripper']['busy'] is True
        harness.bridge.gripper_busy_arms.discard('panda1')
        assert harness.supervisor.frame()['arms']['panda1']['gripper']['busy'] is False

    def test_busy_is_force_cleared_after_the_contract_ceiling(self):
        """The watchdog is the contract's own ceiling plus one second."""
        assert defaults.GRIPPER_BUSY_MAX_S == (
            defaults.GRIPPER_MOTION_TIMEOUT_RANGE_S[1] + 1.0)

    def test_the_six_refusals_fire_in_the_contracted_order(self, harness):
        """Each refusal is the most specific true one."""
        enable_grippers(harness, 'panda1')
        with pytest.raises(SessionError) as stopped:
            request_gripper(harness, 'panda1', 'close')
        assert stopped.value.code == 'session_not_running'

        run_to_running(harness, arms='panda1', mode='watch')
        with pytest.raises(SessionError) as absent:
            request_gripper(harness, 'panda2', 'close')
        assert absent.value.code == 'arm_not_in_session'

        with pytest.raises(SessionError) as unavailable:
            request_gripper(harness, 'panda1', 'close')
        assert unavailable.value.code == 'gripper_unavailable'
        assert 'ros2 launch' in unavailable.value.detail

        harness.bridge.set_gripper_status(
            'panda1', harness.clock.monotonic_ns(),
            gripper_status({'fault_code': '0x0C', 'fault_class': 'major'},
                           message='Internal fault.', level=2))
        with pytest.raises(SessionError) as faulted:
            request_gripper(harness, 'panda1', 'close')
        assert faulted.value.code == 'gripper_faulted'
        assert '/panda1_robotiq/reactivate' in faulted.value.detail

        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        harness.bridge.gripper_busy_arms.add('panda1')
        with pytest.raises(SessionError) as busy:
            request_gripper(harness, 'panda1', 'close')
        assert busy.value.code == 'gripper_busy'

    def test_a_simulate_session_refuses_every_gripper_command_as_not_configured(
            self, harness):
        """The API and the page give ONE answer about Simulate."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, mode='simulate')
        with pytest.raises(SessionError) as caught:
            request_gripper(harness, 'panda1', 'close')
        assert caught.value.code == 'gripper_not_configured'
        assert 'grippers.panda1.enabled' in caught.value.detail

    def test_stop_is_allowed_while_faulted_and_while_busy(self, harness):
        """Stop must always be pressable: it is the gripper's instant disable."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status(
            'panda1', harness.clock.monotonic_ns(),
            gripper_status({'fault_code': '0x0C', 'fault_class': 'major'},
                           message='Internal fault.', level=2))
        harness.bridge.gripper_busy_arms.add('panda1')
        result = request_gripper(harness, 'panda1', 'stop')
        assert result == {'arm_id': 'panda1', 'action': 'stop', 'width_mm': None}
        assert ('trigger', 'panda1', 'stop') in harness.bridge.gripper_events

    def test_reactivate_is_allowed_while_faulted_and_never_waits_for_activation(
            self, harness):
        """It IS the cure for a fault, and the supervisor is never held for it."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status(
            'panda1', harness.clock.monotonic_ns(),
            gripper_status({'fault_code': '0x0C', 'fault_class': 'major'},
                           message='Internal fault.', level=2))
        result = request_gripper(harness, 'panda1', 'reactivate')
        assert result['action'] == 'reactivate'
        assert result['width_mm'] is None
        assert ('trigger_async', 'panda1', 'reactivate') in \
            harness.bridge.gripper_events

    def test_open_and_close_go_through_the_nodes_own_services(self, harness):
        """The widths belong to the node, so the server calls the service."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        assert request_gripper(harness, 'panda1', 'open')['width_mm'] == \
            pytest.approx(harness.settings.gripper('panda1').open_width_mm)
        assert request_gripper(harness, 'panda1', 'close')['width_mm'] == \
            pytest.approx(harness.settings.gripper('panda1').close_width_mm)
        assert [entry for entry in harness.bridge.gripper_events
                if entry[0] == 'goal'] == []

    def test_a_width_goal_sends_zero_max_effort_so_the_node_owns_the_force(
            self, harness):
        """0.0 means "use the node's configured force_n", which is the live one."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        request_gripper(harness, 'panda1', 'width', 30.0)
        goals = [entry for entry in harness.bridge.gripper_events
                 if entry[0] == 'goal']
        assert goals == [('goal', 'panda1', 0.015, 0.0)]

    @pytest.mark.parametrize('width_mm,half_width_m', [
        (0.0, 0.0), (30.0, 0.015), (85.0, 0.0425), (42.5, 0.02125)])
    def test_the_half_width_conversion_is_the_only_gripper_arithmetic_in_franka_web(
            self, harness, width_mm, half_width_m):
        """One documented exception, and it is the action's own convention."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        request_gripper(harness, 'panda1', 'width', width_mm)
        goal = [entry for entry in harness.bridge.gripper_events
                if entry[0] == 'goal'][-1]
        assert goal[2] == pytest.approx(half_width_m)

    def test_a_rejected_goal_is_reported_as_gripper_busy(self, harness):
        """The node's own no-preemption rejection reaches the operator as words."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        harness.bridge.gripper_goal_verdict = 'rejected'
        with pytest.raises(SessionError) as caught:
            request_gripper(harness, 'panda1', 'width', 30.0)
        assert caught.value.code == 'gripper_busy'

    def test_an_unanswered_node_is_reported_as_gripper_unavailable(self, harness):
        """A node that never answers is unavailable, not an internal error."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        harness.bridge.gripper_trigger_response = None
        with pytest.raises(SessionError) as caught:
            request_gripper(harness, 'panda1', 'close')
        assert caught.value.code == 'gripper_unavailable'

    def test_every_dispatch_puts_one_line_in_the_drawer(self, harness):
        """The drawer carries the operator's own gripper actions."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        before = harness.logs.counters()['last_seq']
        request_gripper(harness, 'panda1', 'close')
        lines = harness.logs.window(since=before)['lines']
        assert any('gripper: panda1 close' in json.dumps(line) for line in lines)

    def test_a_lock_revocation_never_moves_a_gripper(self, harness):
        """A browser tab closing must not move a physical device."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.gripper_events = []
        harness.supervisor.revoke_operator_authorization()
        assert harness.bridge.gripper_events == []

    def test_a_width_command_with_no_width_is_a_refusal_not_a_crash(self, harness):
        """A caller reaching the supervisor directly gets the same sentence."""
        enable_grippers(harness, 'panda1')
        run_to_running(harness, arms='panda1', mode='watch')
        harness.bridge.set_gripper_status('panda1', harness.clock.monotonic_ns(),
                                          gripper_status())
        with pytest.raises(SessionError) as caught:
            request_gripper(harness, 'panda1', 'width', None)
        assert caught.value.code == 'invalid_gripper_width'


# ----------------------------------------------------------------------
# Apply — the third per-arm source, on the state machine
# ----------------------------------------------------------------------


#: A goal one comfortable travel away from HOME_POSE on one joint.
APPLY_GOAL = tuple(
    value + (0.2 if index == 3 else 0.0) for index, value in enumerate(HOME_POSE))


#: "no checker argument was given", so a test can wire an ABSENT one.
NO_CHECKER = object()


def apply_ready(tmp_path, checker=NO_CHECKER, arm_id='panda1', **kwargs):
    """
    Return a running Motion harness with one arm enabled and sourced to ghost.

    Enable goes through the real handler, so the epoch bump and the model seed
    are the ones production performs; the source switch does too. Passing
    ``checker=None`` wires a supervisor with no checker at all, which is the
    fail-closed case.
    """
    checker = checker_holding(FakeCellModel()) if checker is NO_CHECKER else checker
    h = running_motion(tmp_path, checker=checker, **kwargs)
    h.drive(lambda: h.supervisor.request_arm_enable(
        arm_id, True, operator_lease=h.operator_lease))
    h.drive(lambda: h.supervisor.request_arm_source(
        arm_id, 'ghost', operator_lease=h.operator_lease))
    assert h.supervisor._arm_enabled[arm_id] is True
    assert h.supervisor._arm_source[arm_id] == 'ghost'
    return h


def start_apply(h, arm_id='panda1', positions=APPLY_GOAL):
    """Run one Apply through the supervisor the way an HTTP worker does."""
    return h.drive(lambda: h.supervisor.request_arm_apply(
        arm_id, 'start', list(positions), operator_lease=h.operator_lease))


def travelling(h, arm_id='panda1'):
    """Return whether this arm has a live plan right now."""
    plan = h.supervisor._arm_travel.get(arm_id)
    return plan is not None and plan.live


def apply_block(h, arm_id='panda1'):
    """Return one arm's ``motion.apply`` block from a fresh frame."""
    return h.supervisor.frame()['arms'][arm_id]['motion']['apply']


class TestApplySource:
    """`ghost` is jog's sibling: server-mediated, and refused a jog."""

    def test_ghost_is_an_accepted_source_and_survives_the_frame(self, tmp_path):
        """T20. The third value reaches the frame the console reads."""
        h = apply_ready(tmp_path)
        assert h.supervisor.frame()['arms']['panda1']['motion']['source'] == 'ghost'
        assert h.supervisor.frame()['arms']['panda2']['motion']['source'] == 'jog'

    def test_a_jog_on_a_ghost_arm_is_refused_in_its_own_words(self, tmp_path):
        """
        T21. The External sentence is simply false about a ghost-sourced arm.

        Nobody else's publisher is involved, so telling the operator to stop
        their own node would teach them the wrong thing about their console.
        """
        h = apply_ready(tmp_path)
        with pytest.raises(SessionError) as caught:
            h.drive(lambda: h.supervisor.request_arm_jog(
                'panda1', 0, 1, operator_lease=h.operator_lease))
        assert caught.value.code == 'not_motion_mode'
        assert caught.value.detail == (
            'this arm is applying a ghost pose; switch the source back to '
            'Jog first')

    def test_the_external_jog_sentence_is_unchanged(self, tmp_path):
        """T21. Byte for byte: only the ghost case is new."""
        h = apply_ready(tmp_path)
        h.drive(lambda: h.supervisor.request_arm_source(
            'panda1', 'external', operator_lease=h.operator_lease))
        with pytest.raises(SessionError) as caught:
            h.drive(lambda: h.supervisor.request_arm_jog(
                'panda1', 0, 1, operator_lease=h.operator_lease))
        assert caught.value.detail == (
            'this arm takes its commands from your own publisher; switch the '
            'source back to Jog first')

    def test_a_ghost_arm_gets_no_counting_subscription(self, tmp_path):
        """The server is the publisher, so there is nothing to count."""
        h = apply_ready(tmp_path)
        h.supervisor.tick()
        assert h.bridge.external_counters == {}


class TestApplyAdmission:
    """The refusal ladder in front of the check, and nothing planned past it."""

    def test_apply_on_a_disabled_arm_records_no_travel(self, tmp_path):
        """T22. ``arm_not_enabled``, and the plan dictionary is untouched."""
        h = apply_ready(tmp_path)
        h.drive(lambda: h.supervisor.request_arm_enable(
            'panda1', False, operator_lease=h.operator_lease))
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'arm_not_enabled'
        assert h.supervisor._arm_travel['panda1'] is None

    def test_apply_is_refused_on_a_disenabled_arm_whose_model_still_holds(
            self, tmp_path):
        """
        M2's tripwire: the enable FLAG, on its own, with nothing masking it.

        A disabled arm's model is normally invalidated too, so a test that
        only pressed the switch would pass even with the flag check deleted --
        the unseeded model would refuse instead. The revocation hook makes
        exactly this state real: it clears the flags lock-free and leaves the
        controller-side disable to a queued command, so there is a genuine
        window in which the flag is false and the model is still seeded.
        """
        h = apply_ready(tmp_path)
        with h.supervisor._state_lock:
            h.supervisor._arm_enabled['panda1'] = False
        assert h.supervisor._jog_models['panda1'].seeded is True, (
            'the model was invalidated, so this would not test the flag')
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'arm_not_enabled'
        assert h.supervisor._arm_travel['panda1'] is None

    @pytest.mark.parametrize('source', ['jog', 'external'])
    def test_apply_on_a_non_ghost_arm_is_refused(self, tmp_path, source):
        """T23. Silently accepting a request that publishes nothing is worse."""
        h = apply_ready(tmp_path)
        h.drive(lambda: h.supervisor.request_arm_source(
            'panda1', source, operator_lease=h.operator_lease))
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'not_motion_mode'
        assert 'switch the source to Ghost first' in caught.value.detail

    def test_a_second_apply_is_refused_while_the_other_arm_travels(self, tmp_path):
        """
        T24. One travel at a time, session-wide, and the refusal names which.

        Two independently timed checked paths do not compose: each was
        approved with the other arm held at a measured constant.
        """
        h = apply_ready(tmp_path)
        h.drive(lambda: h.supervisor.request_arm_enable(
            'panda2', True, operator_lease=h.operator_lease))
        h.drive(lambda: h.supervisor.request_arm_source(
            'panda2', 'ghost', operator_lease=h.operator_lease))
        start_apply(h, 'panda1')
        with pytest.raises(SessionError) as caught:
            start_apply(h, 'panda2')
        assert caught.value.code == 'apply_in_progress'
        assert caught.value.payload == {'arm_id': 'panda1'}
        assert travelling(h, 'panda1') is True

    def test_a_second_apply_on_the_same_arm_is_refused(self, tmp_path):
        """Replacing a running plan is retargeting under another name."""
        h = apply_ready(tmp_path)
        start_apply(h)
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'apply_in_progress'

    def test_an_absent_checker_refuses_with_the_checkers_own_sentence(
            self, tmp_path):
        """
        T25. Fail-closed, and the page holds no copy of the sentence.

        The ghost degrades visibly when the checker cannot run, because the
        ghost commands nothing. Apply refuses, because Apply commands
        everything, and there is no degraded "apply without a check" mode.
        """
        h = apply_ready(tmp_path, checker=None)
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'apply_unavailable'
        assert caught.value.detail == workspace.NOTE_PACKAGE_ABSENT
        assert caught.value.payload == {'checker': 'absent'}
        assert h.supervisor._arm_travel['panda1'] is None

    def test_an_interlock_mismatch_refuses_with_the_interlock_sentence(
            self, tmp_path):
        """T25. A model built for a different description commands nothing."""
        checker = checker_holding(FakeCellModel())
        checker.set_interlock('mismatch')
        h = apply_ready(tmp_path, checker=checker)
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'apply_unavailable'
        assert caught.value.detail == workspace.NOTE_INTERLOCK_MISMATCH
        assert caught.value.payload == {'checker': 'mismatch'}

    def test_a_fouled_path_refuses_and_publishes_nothing_new(self, tmp_path):
        """M1's unit-level tripwire: no plan, and the held target stands."""
        model = FakeCellModel(path_result=CheckResult(
            ok=False, min_clearance=-0.01,
            contacts=(Contact(kind='cross_arm', a='panda1_link5_v1',
                              b='panda2_link6_v1', distance=0.02,
                              required=0.03, arm_id='panda1'),),
            sample_index=40, samples_evaluated=120))
        h = apply_ready(tmp_path, checker=checker_holding(model))
        held = h.supervisor._jog_models['panda1'].target
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'apply_refused'
        assert caught.value.payload['reason_code'] == 'contact'
        assert caught.value.payload['offending_links'] == ['panda1_link5',
                                                           'panda2_link6']
        assert h.supervisor._arm_travel['panda1'] is None
        assert h.supervisor._jog_models['panda1'].target == held

    def test_the_co_arm_pose_comes_from_the_bridge_not_the_request(self, tmp_path):
        """
        M7's unit-level tripwire: the request carries seven floats and no more.

        The solve endpoint may take the client's scene verbatim because there
        it decides only a tint. Here it would decide motion, and only the
        server's own measurements will do.
        """
        model = FakeCellModel()
        h = apply_ready(tmp_path, checker=checker_holding(model))
        fabricated = tuple(value + 1.0 for value in HOME_POSE)
        h.drive(lambda: h.supervisor.request_arm_apply(
            'panda1', 'start', list(APPLY_GOAL),
            operator_lease=h.operator_lease))
        checked = model.paths[0]
        assert [point['panda2'] for point in checked] == [HOME_POSE] * 3
        assert all(point['panda2'] != fabricated for point in checked)


class TestApplyLifecycle:
    """What stops a travel, and what a stopped travel leaves behind."""

    def test_a_travel_advances_the_held_target_along_the_checked_line(
            self, tmp_path):
        """The stream keeps publishing; only the held target moves."""
        h = apply_ready(tmp_path)
        response = start_apply(h)
        model = h.supervisor._jog_models['panda1']
        before = model.target
        h.supervisor.jog_stream_tick()
        assert model.target != before
        assert response['steps_total'] > 1
        assert response['goal'] == list(APPLY_GOAL)
        assert response['start'] == list(before)

    def test_a_disable_clears_the_travel_and_moves_the_enable_epoch(
            self, tmp_path):
        """
        T26. Re-enabling is a new authorization, never a continuation.

        The plan carries the epoch it was authorised under, so even a missed
        clear stops it dead.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        epoch = h.supervisor._enable_epoch['panda1']
        h.drive(lambda: h.supervisor.request_arm_enable(
            'panda1', False, operator_lease=h.operator_lease))
        assert h.supervisor._arm_travel['panda1'] is None
        assert h.supervisor._enable_epoch['panda1'] > epoch
        h.drive(lambda: h.supervisor.request_arm_enable(
            'panda1', True, operator_lease=h.operator_lease))
        assert h.supervisor._arm_travel['panda1'] is None

    def test_a_plan_whose_epoch_moved_is_never_advanced(self, tmp_path):
        """
        T27. M5's tripwire: the epoch alone stops a plan the clears missed.

        The plan is injected directly and the clear deliberately skipped, so
        the ONLY thing that can stop it is gate 5.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        model = h.supervisor._jog_models['panda1']
        h.supervisor._enable_epoch['panda1'] += 1
        before = model.target
        h.supervisor.jog_stream_tick()
        assert model.target == before

    def test_the_advance_is_gated_on_the_cancel_generation_too(self, tmp_path):
        """
        T43. Bumping the generation alone stops the travel.

        That is what makes an operator Cancel bounded: whoever bumps the
        integer has stopped this travel by the next tick without touching a
        queue, a lock or a plan object.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        model = h.supervisor._jog_models['panda1']
        h.supervisor._cancel_gen['panda1'] += 1
        before = model.target
        h.supervisor.jog_stream_tick()
        assert model.target == before

    @pytest.mark.parametrize('state', ['fault', 'stopping'])
    def test_fault_and_teardown_clear_every_travel(self, tmp_path, state):
        """T28. A disbelieved picture of the cell stops every checked path."""
        h = apply_ready(tmp_path)
        start_apply(h)
        h.supervisor._transition(state, reason=None)
        assert h.supervisor._arm_travel == {'panda1': None, 'panda2': None}

    def test_the_revocation_hook_clears_every_travel_without_the_state_lock(
            self, tmp_path):
        """
        T29. It runs inside the operator lock's own mutex and may not block.

        Asserted by holding ``_state_lock`` on another thread and requiring
        the hook to return anyway.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        held = threading.Event()
        release = threading.Event()

        def hold():
            """Sit inside the supervisor's own lock while the hook runs."""
            with h.supervisor._state_lock:
                held.set()
                release.wait(5.0)
        holder = threading.Thread(target=hold)
        holder.start()
        assert held.wait(2.0)
        try:
            done = threading.Event()

            def revoke():
                """Run the hook and record that it returned."""
                h.supervisor.revoke_operator_authorization()
                done.set()
            thread = threading.Thread(target=revoke)
            thread.start()
            assert done.wait(0.05), 'the revocation hook waited on _state_lock'
            thread.join(timeout=2)
        finally:
            release.set()
            holder.join(timeout=2)
        assert h.supervisor._arm_travel['panda1'] is None

    def test_a_source_switch_clears_the_travel_before_it_reseeds(self, tmp_path):
        """
        T30. The switch is a statement about who commands this arm.

        Clearing first means the jog branch's re-seed reads a settled arm
        rather than racing a moving target.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        h.drive(lambda: h.supervisor.request_arm_source(
            'panda1', 'jog', operator_lease=h.operator_lease))
        assert h.supervisor._arm_travel['panda1'] is None
        before = h.supervisor._jog_models['panda1'].target
        h.supervisor.jog_stream_tick()
        assert h.supervisor._jog_models['panda1'].target == before

    def test_an_arrived_travel_holds_the_goal_and_goes_idle(self, tmp_path):
        """The stream keeps publishing; the held target stops changing."""
        h = apply_ready(tmp_path)
        start_apply(h)
        model = h.supervisor._jog_models['panda1']
        for _ in range(h.supervisor._arm_travel['panda1'].steps_total):
            h.supervisor._last_advance['panda1'] = None
            h.supervisor.jog_stream_tick()
        assert model.target == APPLY_GOAL
        assert apply_block(h)['state'] == 'idle'
        assert h.supervisor._arm_source['panda1'] == 'ghost'
        # M3's tripwire. An advance that runs past the end of a finite plan
        # raises off the end of the waypoint list, and the tick swallows one
        # arm's exception so it cannot gap the other -- which means the arm
        # silently STOPS BEING FED and the watchdog freezes it. Arriving is a
        # hold, so the stream must keep running.
        published = len(h.bridge.published_targets)
        for _ in range(4):
            h.supervisor._last_advance['panda1'] = None
            h.supervisor.jog_stream_tick()
        assert len(h.bridge.published_targets) == published + 4, (
            'the stream stopped when the travel arrived')
        assert model.target == APPLY_GOAL


class TestApplyCancel:
    """The stop that must never be behind a queue."""

    def test_cancel_on_an_idle_arm_answers_that_nothing_was_travelling(
            self, tmp_path):
        """
        T31. A stop control that can return an error is one you distrust.

        Idempotent by design, so a double press costs nothing.
        """
        h = apply_ready(tmp_path)
        result = h.supervisor.request_arm_apply('panda1', 'cancel')
        assert result == {'arm_id': 'panda1', 'action': 'cancel',
                          'was_travelling': False, 'fraction': None,
                          'stopped_at': None}

    def test_cancel_after_arrival_reports_that_nothing_was_travelling(
            self, tmp_path):
        """
        The two orders answer differently, and the frame is the arbiter.

        An arrived plan is left in the dict with ``step == steps_total``: the
        tick's arrival branch stores it back and nothing clears it. Every other
        reader filters on ``plan.live`` -- which is why the frame already says
        ``idle`` -- so a cancel that answered ``was_travelling: True`` here
        would be the response and ``apply.state`` disagreeing about whether a
        travel was running. The stale entry is still cleared.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        for _ in range(h.supervisor._arm_travel['panda1'].steps_total):
            h.supervisor._last_advance['panda1'] = None
            h.supervisor.jog_stream_tick()
        assert apply_block(h)['state'] == 'idle'
        assert h.supervisor._arm_travel['panda1'] is not None, (
            'the arrived plan was cleared elsewhere; this case has moved'
        )
        before = h.supervisor._cancel_gen['panda1']
        result = h.supervisor.request_arm_apply('panda1', 'cancel')
        assert result == {'arm_id': 'panda1', 'action': 'cancel',
                          'was_travelling': False, 'fraction': None,
                          'stopped_at': None}
        assert h.supervisor._arm_travel['panda1'] is None
        assert h.supervisor._cancel_gen['panda1'] > before

    def test_cancel_before_arrival_reports_that_one_was(self, tmp_path):
        """The other order: mid-travel, the same press answers True."""
        h = apply_ready(tmp_path)
        start_apply(h)
        h.supervisor._last_advance['panda1'] = None
        h.supervisor.jog_stream_tick()
        assert apply_block(h)['state'] == 'travelling'
        result = h.supervisor.request_arm_apply('panda1', 'cancel')
        assert result['was_travelling'] is True
        assert 0.0 < result['fraction'] < 1.0
        assert apply_block(h)['state'] == 'idle'

    def test_cancel_stops_the_travel_and_reports_where(self, tmp_path):
        """The last waypoint is a point on the CHECKED line, so it is held."""
        h = apply_ready(tmp_path)
        start_apply(h)
        for _ in range(3):
            h.supervisor._last_advance['panda1'] = None
            h.supervisor.jog_stream_tick()
        model = h.supervisor._jog_models['panda1']
        stopped = model.target
        result = h.supervisor.request_arm_apply('panda1', 'cancel')
        assert result['was_travelling'] is True
        assert result['stopped_at'] == list(stopped)
        assert 0.0 < result['fraction'] < 1.0
        h.supervisor.jog_stream_tick()
        assert model.target == stopped

    def test_cancel_never_reaches_the_command_queue(self, tmp_path):
        """
        T41. M9's tripwire: a queue whose put explodes changes nothing.

        Every other operator command is a queued ``_Command``, and ``_submit``
        DISCARDS one the supervisor never reached -- which for a stop would
        mean an error response while the arm kept moving.
        """
        h = apply_ready(tmp_path)
        start_apply(h)

        class Explodes:
            """A queue that refuses to accept anything at all."""

            def put(self, item):
                """Fail the way a queued cancel would have to go through."""
                raise AssertionError('the cancel was queued')

        h.supervisor._commands = Explodes()
        result = h.supervisor.request_arm_apply('panda1', 'cancel')
        assert result['was_travelling'] is True
        assert h.supervisor._arm_travel['panda1'] is None

    def test_cancel_answers_while_the_supervisor_thread_is_blocked(
            self, tmp_path):
        """
        T42. The bound, measured against a BLOCKED supervisor.

        Before Apply, a blocked supervisor left a jogged arm static. A
        travelling arm is not static, so this is the one stop that comes off
        the queue.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        entered = threading.Event()
        release = threading.Event()
        original = h.bridge.call_enable

        def slow_enable(slot, enabled, timeout_s=5.0):
            """Hold the supervisor thread inside a service call."""
            entered.set()
            release.wait(5.0)
            return original(slot, enabled, timeout_s)
        h.bridge.call_enable = slow_enable
        blocked = threading.Thread(target=lambda: h.drive(
            lambda: h.supervisor.request_arm_enable(
                'panda2', True, operator_lease=h.operator_lease, timeout_s=30.0),
            ticks=400))
        blocked.start()
        try:
            assert entered.wait(2.0), 'the supervisor never blocked'
            started = time.monotonic()
            result = h.supervisor.request_arm_apply('panda1', 'cancel')
            elapsed = time.monotonic() - started
            assert result['was_travelling'] is True
            assert elapsed < 0.2, elapsed
            assert h.supervisor._arm_travel['panda1'] is None
        finally:
            release.set()
            blocked.join(timeout=10)

    def test_a_cancel_for_an_unknown_arm_grows_no_key(self, tmp_path):
        """
        T31b. The fixed key set is what makes every lock-free write legal.

        A cancel naming an arm this session does not have would otherwise be
        the thing that grows it.
        """
        h = running_motion(tmp_path, checker=checker_holding(FakeCellModel()))
        before = [set(h.supervisor._cancel_gen), set(h.supervisor._arm_travel),
                  set(h.supervisor._enable_epoch),
                  set(h.supervisor._last_advance)]
        with pytest.raises(SessionError) as caught:
            h.supervisor.request_arm_apply('panda9', 'cancel')
        assert caught.value.code == 'arm_not_in_session'
        after = [set(h.supervisor._cancel_gen), set(h.supervisor._arm_travel),
                 set(h.supervisor._enable_epoch),
                 set(h.supervisor._last_advance)]
        assert before == after

    def test_a_cancel_with_no_session_at_all_grows_no_key(self, tmp_path):
        """T31b, the second half: a stopped server has no keys to create."""
        h = Harness(tmp_path, checker=checker_holding(FakeCellModel()))
        with pytest.raises(SessionError):
            h.supervisor.request_arm_apply('panda1', 'cancel')
        assert h.supervisor._cancel_gen == {}
        assert h.supervisor._arm_travel == {}


class TestApplyRaces:
    """The two interleavings a naive implementation gets wrong."""

    def pause_the_tick(self, h):
        """Return (entered, release) events armed inside ``advanced``."""
        entered = threading.Event()
        release = threading.Event()
        original = travel.TravelPlan.advanced

        def blocking(self):
            """Let another thread run between the read and the store-back."""
            entered.set()
            release.wait(5.0)
            return original(self)
        travel.TravelPlan.advanced = blocking
        return entered, release, original

    def run_interleaved(self, tmp_path, interfere):
        """
        Enter the tick, pause it mid-advance, interfere, then release it.

        The interference runs on its OWN thread: the paused tick is holding
        ``_state_lock``, so a canceller that takes it would otherwise deadlock
        the test rather than exercise it. The lock-free half of every stop --
        the generation bump -- happens before the lock is taken, which is
        exactly the property under test.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        entered, release, original = self.pause_the_tick(h)
        try:
            ticker = threading.Thread(target=h.supervisor.jog_stream_tick)
            ticker.start()
            assert entered.wait(2.0), 'the tick never reached the store-back'
            other = threading.Thread(target=lambda: interfere(h))
            other.start()
            time.sleep(0.1)
            release.set()
            ticker.join(timeout=5)
            other.join(timeout=5)
        finally:
            travel.TravelPlan.advanced = original
        return h

    def test_the_revocation_hook_cannot_be_undone_by_the_store_back(
            self, tmp_path):
        """
        T29b. M10's tripwire, and the finding this test was written for.

        A blind store-back would resurrect a plan authority has already been
        withdrawn from. It could not move the arm -- the enable and source
        gates are false by then -- but nothing else would ever clear it: the
        frame would report a travel that can never move, and every later Apply
        in the session would be refused ``apply_in_progress``.
        """
        h = self.run_interleaved(
            tmp_path, lambda s: s.supervisor.revoke_operator_authorization())
        assert h.supervisor._arm_travel['panda1'] is None
        assert apply_block(h)['state'] == 'idle'

    def test_a_cancel_cannot_be_undone_by_the_store_back(self, tmp_path):
        """T29c. The same interleaving, with the operator's own Cancel."""
        h = self.run_interleaved(
            tmp_path,
            lambda s: s.supervisor.request_arm_apply('panda1', 'cancel'))
        assert h.supervisor._arm_travel['panda1'] is None
        assert apply_block(h)['state'] == 'idle'
        # And the session is not wedged: a later Apply is admitted rather than
        # refused for a travel that no longer exists.
        h.drive(lambda: h.supervisor.request_arm_enable(
            'panda1', True, operator_lease=h.operator_lease))
        h.drive(lambda: h.supervisor.request_arm_source(
            'panda1', 'ghost', operator_lease=h.operator_lease))
        assert start_apply(h)['steps_total'] > 0

    def test_an_enable_that_fails_still_clears_the_travel_first(self, tmp_path):
        """
        T44. The three exits that leave the arm ENABLED are the dangerous ones.

        ``model.seed(measured)`` runs before the service call and therefore
        before every failure exit, so a plan left live beside a re-seeded model
        would resume from a target yanked off the checked line.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        h.bridge.enable_ready = False
        with pytest.raises(SessionError) as caught:
            h.drive(lambda: h.supervisor.request_arm_enable(
                'panda1', True, operator_lease=h.operator_lease))
        assert caught.value.code == 'enable_service_unavailable'
        assert h.supervisor._arm_enabled['panda1'] is True
        assert h.supervisor._arm_travel['panda1'] is None
        model = h.supervisor._jog_models['panda1']
        before = model.target
        h.supervisor.jog_stream_tick()
        assert model.target == before

    def test_an_enable_the_controller_rejects_still_clears_the_travel(
            self, tmp_path):
        """T44, the second exit: ``enable_rejected`` leaves the arm enabled."""
        h = apply_ready(tmp_path)
        start_apply(h)
        h.bridge.enable_response = {'success': False, 'message': 'refused'}
        with pytest.raises(SessionError) as caught:
            h.drive(lambda: h.supervisor.request_arm_enable(
                'panda1', True, operator_lease=h.operator_lease))
        assert caught.value.code == 'enable_rejected'
        assert h.supervisor._arm_travel['panda1'] is None

    def test_an_enable_that_never_answers_still_clears_the_travel(self, tmp_path):
        """T44, the third exit: the compensating-disable path."""
        h = apply_ready(tmp_path)
        start_apply(h)
        h.bridge.enable_response = None
        with pytest.raises(SessionError) as caught:
            h.drive(lambda: h.supervisor.request_arm_enable(
                'panda1', True, operator_lease=h.operator_lease))
        assert caught.value.code == 'enable_service_unavailable'
        assert h.supervisor._arm_travel['panda1'] is None

    def test_a_stop_landing_during_the_check_refuses_rather_than_plans(
            self, tmp_path):
        """
        T45. The commit re-read, which is not decoration.

        ``check_path`` runs between reading the two integers and committing
        the plan. A Cancel landing inside that window must not be overtaken by
        a plan stamped with the value it just invalidated.
        """
        model = FakeCellModel()
        h = apply_ready(tmp_path, checker=checker_holding(model))
        model.path_hook = lambda: h.supervisor._cancel_gen.__setitem__(
            'panda1', h.supervisor._cancel_gen['panda1'] + 1)
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'apply_refused'
        assert caught.value.payload['reason_code'] == 'no_travel'
        assert h.supervisor._arm_travel['panda1'] is None


class TestApplyStopGuards:
    """The two guards that stop a travel because the world moved."""

    def test_a_co_arm_that_moves_stops_the_travel_and_holds(self, tmp_path):
        """The pair-pose the model approved is the pair-pose that executes."""
        h = apply_ready(tmp_path)
        start_apply(h)
        h.supervisor.jog_stream_tick()
        model = h.supervisor._jog_models['panda1']
        held = model.target
        drifted = list(HOME_POSE)
        drifted[0] += 0.05
        h.bridge.set_joint_sample(
            h.clock.monotonic_ns(),
            dual_joint_state_pair(list(model.target), drifted))
        h.supervisor._last_advance['panda1'] = None
        h.supervisor.jog_stream_tick()
        assert h.supervisor._arm_travel['panda1'] is None
        assert model.target == held

    def test_an_arm_that_does_not_follow_stops_the_travel(self, tmp_path):
        """
        The lag brake stops it before the torque ceilings have to.

        The ceilings are the real bound; this is an earlier one that can say
        what happened in words.
        """
        h = apply_ready(tmp_path, arm_id='panda1')
        start_apply(h, positions=tuple(
            value + (0.6 if index == 3 else 0.0)
            for index, value in enumerate(HOME_POSE)))
        model = h.supervisor._jog_models['panda1']
        for _ in range(400):
            if h.supervisor._arm_travel['panda1'] is None:
                break
            h.supervisor._last_advance['panda1'] = None
            # The measured pose never moves: the arm is held by something.
            h.bridge.set_joint_sample(h.clock.monotonic_ns(),
                                      dual_joint_state())
            h.supervisor.jog_stream_tick()
        assert h.supervisor._arm_travel['panda1'] is None
        lag = max(abs(a - b) for a, b in zip(model.target, HOME_POSE))
        assert lag > defaults.APPLY_LAG_LIMIT_RAD

    def test_the_spacing_floor_makes_a_burst_of_ticks_one_step(self, tmp_path):
        """
        M12's unit-level tripwire.

        A stalled executor delivers ticks in a burst when it catches up, and a
        burst is the one schedule that could put more than one step's worth of
        command ahead of the controller's ramp.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        for _ in range(10):
            h.supervisor.jog_stream_tick()
        assert h.supervisor._arm_travel['panda1'].step == 1


class TestApplyFrame:
    """The frame's apply block, and the two hint rows beside it."""

    def test_the_block_is_always_present_and_idle_by_default(self, tmp_path):
        """A v4 consumer is missing a required key, which is why 5 exists."""
        h = apply_ready(tmp_path)
        block = apply_block(h)
        assert block == {'available': True, 'state': 'idle', 'fraction': None,
                         'steps_done': None, 'steps_total': None,
                         'seconds_remaining': None, 'goal': None, 'note': None}

    def test_a_travelling_block_carries_the_progress_and_the_goal(self, tmp_path):
        """The applied pose is a server-side fact every viewer may see."""
        h = apply_ready(tmp_path)
        start_apply(h)
        h.supervisor.jog_stream_tick()
        block = apply_block(h)
        assert block['state'] == 'travelling'
        assert block['goal'] == list(APPLY_GOAL)
        assert block['steps_done'] == 1
        assert block['steps_total'] > 1
        assert 0.0 < block['fraction'] < 1.0
        assert block['seconds_remaining'] > 0.0

    def test_the_note_is_non_null_exactly_when_no_apply_can_start(self, tmp_path):
        """
        T32. G3-G4's "exactly when", asserted in both directions.

        The four checker sentences keep their one author, and the console
        renders whichever one the server sends.
        """
        working = apply_ready(tmp_path)
        assert apply_block(working)['note'] is None
        absent = apply_ready(tmp_path, checker=None)
        assert apply_block(absent)['note'] == workspace.NOTE_PACKAGE_ABSENT
        mismatched = checker_holding(FakeCellModel())
        mismatched.set_interlock('mismatch')
        h = apply_ready(tmp_path, checker=mismatched)
        assert apply_block(h)['note'] == workspace.NOTE_INTERLOCK_MISMATCH

    def test_a_model_that_does_not_describe_this_arm_is_a_note_and_a_refusal(
            self, tmp_path):
        """
        The fourth row, and it is per ARM rather than per session.

        A model can load and still not describe the arm in front of you. If
        the frame said nothing about that, the button would be live and the
        press would fail -- so the note and the refusal are the same answer to
        the same question, read from the same place.
        """
        h = apply_ready(tmp_path,
                        checker=checker_holding(FakeCellModel(arms=('panda2',))))
        assert apply_block(h)['note'] == workspace.NOTE_PROFILE_ARM_MISMATCH
        with pytest.raises(SessionError) as caught:
            start_apply(h)
        assert caught.value.code == 'apply_unavailable'
        assert caught.value.detail == workspace.NOTE_PROFILE_ARM_MISMATCH

    def test_building_a_frame_reads_no_file_at_all(self, tmp_path,
                                                   monkeypatch):
        """
        The note is a cache lookup per arm, not a cell file per arm.

        The frame runs at STATE_FRAME_HZ on the thread every SSE viewer is fed
        from. Asking the checker for its whole scene status block would, on a
        model package too old to report its own measured volume, re-open and
        re-parse the cell YAML twice per frame -- so the frame asks the three
        cheap questions instead. Proved by watching the two doors into a file
        rather than by timing anything.

        A FakeCellModel is exactly such an old package: it offers no
        ``allowed_volume``, so the fallback is the one that would run.
        """
        h = apply_ready(tmp_path, checker=checker_holding(FakeCellModel()))
        opened = []
        real_open = builtins.open
        real_read_text = pathlib.Path.read_text

        def watched_open(*args, **kwargs):
            """Record the path, then open it exactly as before."""
            opened.append(args[0] if args else None)
            return real_open(*args, **kwargs)

        def watched_read_text(self, *args, **kwargs):
            """Record the path, then read it exactly as before."""
            opened.append(str(self))
            return real_read_text(self, *args, **kwargs)
        monkeypatch.setattr(builtins, 'open', watched_open)
        monkeypatch.setattr(pathlib.Path, 'read_text', watched_read_text)
        try:
            for _ in range(3):
                assert h.supervisor.frame()['arms']['panda1']['motion'][
                    'apply']['note'] is None
        finally:
            monkeypatch.undo()
        assert opened == [], opened

    def test_the_note_is_null_where_there_is_no_apply_surface(self, tmp_path):
        """A note beside an absent surface would be an answer to no question."""
        h = Harness(tmp_path, checker=None)
        h.make_ready_simulate()
        h.start(arms='both', mode='simulate')
        for _ in range(5):
            h.supervisor.tick()
        block = h.supervisor.frame()['arms']['panda1']['motion']['apply']
        assert block['available'] is False
        assert block['note'] is None

    def test_the_travelling_hint_names_the_control_that_stops_it(self, tmp_path):
        """The most urgent thing on the screen gets the hint line."""
        h = apply_ready(tmp_path)
        start_apply(h)
        h.supervisor.jog_stream_tick()
        assert h.supervisor.frame()['hint'] == (
            'Applying a pose to panda1 — press Cancel to stop it where it is.')

    def test_an_idle_ghost_arm_is_told_where_the_ghost_lives(self, tmp_path):
        """A source with no visible next step teaches nothing."""
        h = apply_ready(tmp_path)
        assert h.supervisor.frame()['hint'] == (
            'Drag the ghost in the Scene panel, then press Apply on the card.')

    @pytest.mark.parametrize('gate', ['enabled', 'source', 'lock', 'state'])
    def test_the_state_is_travelling_only_while_every_gate_holds(
            self, tmp_path, gate):
        """
        T33. Each gate driven false in isolation stops the advance.

        The plan is a plan, never a permission: every gate that authorised it
        is re-read on every single tick.
        """
        h = apply_ready(tmp_path)
        start_apply(h)
        model = h.supervisor._jog_models['panda1']
        before = model.target
        if gate == 'enabled':
            h.supervisor._arm_enabled['panda1'] = False
        elif gate == 'source':
            h.supervisor._arm_source['panda1'] = 'external'
        elif gate == 'lock':
            h.lock.release(h.claim.token)
        else:
            with h.supervisor._state_lock:
                h.supervisor._state = 'settling'
        h.supervisor.jog_stream_tick()
        assert model.target == before
