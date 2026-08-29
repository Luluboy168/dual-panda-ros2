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
The §8 Stage 2 guard matrix for the motion surface, driven entirely with fakes.

Two levels, both unit:

* **Session level** — a real :class:`~franka_web.session.SessionSupervisor` with
  a real :class:`~franka_web.gains.GainsStore`, a real
  :class:`~franka_web.lock.OperatorLock` and a real
  :class:`~franka_web.jog.JogTargetModel`, wired to the ``support.fake_launcher``
  fakes and a :class:`~support.fake_clock.FakeClock`. Every §6.13 command is
  submitted the way an HTTP worker submits it (queue + wait) and answered by an
  explicit ``tick()``, so the ordering rules the plan states in prose —
  *enable flag set only after the service succeeds*, *enable flag cleared before
  the disable call*, *enables forced off before anything else on fault* — are
  observed rather than assumed.
* **HTTP level** — the real ``ThreadingHTTPServer`` over loopback against a
  scripted supervisor, covering only what the table-driven token test in
  ``test_http_api.py`` cannot: the ``{arm_id}`` path segment, the two motion
  body shapes, and the §6.5 gains upload against a REAL ``GainsStore``.

No robot, no ROS graph, no child process and no launch is executed anywhere in
this file: the spawner is a fake, and the only addresses that appear are RFC
5737 documentation addresses, present so a motion profile can build its argv.
"""

import http.client
import json
import os
import socket
import threading
import time

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus
from franka_msgs.msg import FrankaState
from franka_web import config, health
from franka_web.config import Settings
from franka_web.gains import GainsStore
from franka_web.http_api import App, build_server
from franka_web.lock import OperatorLock
from franka_web.session import (
    expected_state_broadcasters, SessionError, SessionRequest, SessionSupervisor)
from franka_web.sse import Broker
import pytest
from sensor_msgs.msg import JointState
from support.fake_clock import FakeClock
from support.fake_launcher import (
    FakeBridge, FakeBroker, FakeChild, FakePreflightResult, FakeRecording, FakeSpawner)
from support.mock_impedance_controller import (
    ENABLE_DISABLED_MESSAGE, ENABLE_ENABLED_MESSAGE, NO_ERRORS_MESSAGE)

#: RFC 5737 documentation addresses. They exist only so a motion profile can
#: build an argv; nothing in this file ever executes one.
DOC_IP_1 = '203.0.113.7'
DOC_IP_2 = '203.0.113.8'
DOC_IP_SINGLE = '203.0.113.9'

IMPEDANCE = 'dual_arm_joint_impedance_controller'
HOLD = 'dual_arm_joint_hold_controller'
VELOCITY = 'dual_arm_joint_velocity_controller'

HARDWARE_NAME = 'FrankaMultiHardwareInterface'

#: The validated fixtures are READ rather than copied: the fence numbers below
#: are only meaningful against the very bytes ``test_gains.py`` pins, and a
#: copy here would drift the moment the validator's limits change.
SAMPLE_GAINS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'support', 'sample_gains')

STATIC_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')

#: A pose every joint of which sits inside ``valid_dual_impedance.yaml``'s
#: fence -- note joint4, whose fence is [-3.0718, -0.0698] and so excludes 0.0.
IN_FENCE_POSE = (0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.5)

#: The same pose with joint4 one third of a step below its upper fence, so a
#: single ``+`` press has to be clamped.
NEAR_UPPER_POSE = (0.0, -0.3, 0.0, -0.08, 0.0, 1.2, 0.5)

#: joint4 above its upper fence: legal for a JointState, refused by the §5.4
#: precondition and by ``JogTargetModel.seed``.
OUT_OF_FENCE_POSE = (0.0, -0.3, 0.0, 0.4, 0.0, 1.2, 0.5)

#: Wall-clock bound on one queue-and-tick command exchange. Everything here is
#: in-process; anything slower than this is a hang, not slowness.
COMMAND_DEADLINE_S = 10.0

#: Every request in the HTTP half is loopback and answered in-process.
REQUEST_TIMEOUT_S = 10.0


def read_gains(name):
    """Return one ``support/sample_gains`` fixture as the raw bytes an upload carries."""
    with open(os.path.join(SAMPLE_GAINS_DIR, name), 'rb') as handle:
        return handle.read()


def dual_joint_state(pose_1=IN_FENCE_POSE, pose_2=IN_FENCE_POSE):
    """Build a complete 14-name dual JointState carrying the two given poses."""
    message = JointState()
    for arm_id, pose in (('panda1', pose_1), ('panda2', pose_2)):
        for index, position in enumerate(pose, start=1):
            message.name.append('{}_joint{}'.format(arm_id, index))
            message.position.append(float(position))
            message.velocity.append(0.0)
            message.effort.append(0.0)
    return message


def partial_joint_state():
    """Build a JointState missing panda1's joint7 (``complete`` is False)."""
    message = dual_joint_state()
    del message.name[6]
    del message.position[6]
    del message.velocity[6]
    del message.effort[6]
    return message


def healthy_robot_state():
    """Build a FrankaState no fault rule fires on (move mode, high CCSR, no errors)."""
    message = FrankaState()
    message.robot_mode = 2
    message.control_command_success_rate = 0.998
    return message


def diagnostic_at(arm_id, level=0):
    """Build the canonical per-arm DiagnosticStatus at the given level."""
    status = DiagnosticStatus()
    status.name = health.canonical_diagnostic_name(arm_id)
    status.hardware_id = arm_id
    status.level = bytes([level])
    status.message = ('backend state is healthy' if level == 0
                      else 'communication constraints violated')
    return status


def hardware_component(lifecycle_label='active'):
    """Build the §6.11 hardware component dict the bridge caches."""
    return {
        'name': HARDWARE_NAME,
        'plugin_name': 'franka_hardware/FrankaMultiHardwareInterface',
        'lifecycle_id': 3 if lifecycle_label == 'active' else 2,
        'lifecycle_label': lifecycle_label,
    }


def make_settings(tmp_path, **extra):
    """Build valid Settings over private tmp directories and documentation addresses."""
    state_dir = tmp_path / 'state'
    state_dir.mkdir(mode=0o700, exist_ok=True)
    recording_root = tmp_path / 'recordings'
    recording_root.mkdir(mode=0o700, exist_ok=True)
    os.chmod(str(tmp_path), 0o700)
    env = {
        'FRANKA_WEB_STATE_DIR': str(state_dir),
        'FRANKA_WEB_RECORDING_ROOT': str(recording_root),
        'FRANKA_WEB_ROBOT_IP_1': DOC_IP_1,
        'FRANKA_WEB_ROBOT_IP_2': DOC_IP_2,
        'FRANKA_WEB_ROBOT_IP': DOC_IP_SINGLE,
        'ROS_DOMAIN_ID': '80',
    }
    env.update(extra)
    return Settings.from_env(env)


def published(harness):
    """Return how many jog targets the bridge has been handed so far."""
    return len(harness.bridge.published_targets)


class MotionBridge(FakeBridge):
    """
    ``FakeBridge`` plus the recording and hooks the motion tests need.

    Two things the shared fake cannot give: a NON-ZERO clock stamp (a zeroed
    ``builtin_interfaces/Time`` is exactly what ``JogTargetModel.message``
    refuses, and the real bridge reads a live node clock), and a hook that runs
    INSIDE ``call_enable`` -- the only way to observe what the supervisor had
    already done to the enable flag and the held target at the moment it called
    the controller.
    """

    def __init__(self, clock):
        """Wire the fake to ``clock`` and start with nothing recorded."""
        super().__init__()
        self.clock = clock
        self.enable_calls = []
        self.published_targets = []
        self.recovery_calls = []
        self.switch_calls = []
        self.hardware_active_calls = []
        self.on_enable = None
        self.on_recovery_success = None

    def now_msg(self):
        """Return the fake clock as a non-zero ``builtin_interfaces/Time``."""
        now = self.clock.monotonic()
        return Time(sec=int(now), nanosec=int(round((now - int(now)) * 1e9)))

    def call_enable(self, slot, enabled, timeout_s=5.0):
        """Run the observation hook, then answer with the scripted response."""
        if self.on_enable is not None:
            self.on_enable(slot, enabled)
        return super().call_enable(slot, enabled, timeout_s)

    def call_error_recovery(self, arm_id, timeout_s=5.0):
        """
        Record the per-arm recovery call and answer with the scripted response.

        ``on_recovery_success`` is the test's stand-in for the world changing
        underneath a successful call: the real service clears the robot's
        error, and the next diagnostic message therefore comes back healthy.
        §7.3 leaves ``fault`` only if the rules stop firing on the NEXT tick,
        so a fake that answered ok while leaving the snapshot poisoned could
        never reach ``running`` at all.
        """
        self.recovery_calls.append(arm_id)
        response = super().call_error_recovery(arm_id, timeout_s)
        if (response is not None and response['success']
                and self.on_recovery_success is not None):
            self.on_recovery_success(arm_id)
        return response

    def call_switch_activate(self, controllers, timeout_s=5.0):
        """Record the controller re-activation call and answer as scripted."""
        self.switch_calls.append(list(controllers))
        return super().call_switch_activate(controllers, timeout_s)

    def call_hardware_active(self, name, timeout_s=5.0):
        """
        Record the hardware activation call and, when it succeeds, make it true.

        ``list_hardware_components`` reports ``active`` after a successful
        activation; a fake that answered ok and kept reporting ``inactive``
        would be scripting a contradiction, not a robot.
        """
        self.hardware_active_calls.append(name)
        response = super().call_hardware_active(name, timeout_s)
        if response is not None and response['ok'] and self.hardware is not None:
            self.hardware = dict(self.hardware, lifecycle_id=3, lifecycle_label='active')
        return response


class MotionHarness:
    """
    One fully faked supervisor, wired for motion sessions.

    Real: the supervisor, the gains store (over a private tmp state dir), the
    operator lock, the jog models and the fault engine. Fake: the clock, the
    bridge, the spawner, the recorder and the preflight verdict.
    """

    def __init__(self, tmp_path):
        """Wire a stopped supervisor whose every collaborator is inspectable."""
        self.clock = FakeClock()
        self.settings = make_settings(tmp_path)
        self.bridge = MotionBridge(self.clock)
        self.broker = FakeBroker()
        self.spawner = FakeSpawner()
        self.recorder = FakeRecording()
        self.preflight = FakePreflightResult()
        self.lock = OperatorLock(monotonic=self.clock.monotonic)
        self.gains = GainsStore(self.settings.state_dir)
        self.launch_child = FakeChild(name='launch')
        self.spawner.queue_child(self.launch_child)
        self.token = None
        self.supervisor = SessionSupervisor(
            self.settings, self.bridge, self.lock, self.broker,
            spawn=self.spawner,
            recording_factory=lambda: self.recorder,
            preflight_runner=lambda settings, mode: self.preflight,
            gains_store=self.gains,
            monotonic=self.clock.monotonic,
        )

    # -- driving the supervisor ----------------------------------------

    def tick(self):
        """Run exactly one supervisor step."""
        self.supervisor.tick()

    def pump(self, ticks=4):
        """Run ``ticks`` supervisor steps."""
        for _ in range(ticks):
            self.supervisor.tick()

    def drive(self, call):
        """
        Submit one operator command the way an HTTP worker does and answer it.

        The command is enqueued from another thread and this thread waits for
        it to REACH the queue before ticking, so the very next ``tick()``
        answers it: no state work can slip in between the request and its
        verdict, and a guard test therefore observes the state it set up.
        """
        outcome = {}

        def submit():
            try:
                outcome['value'] = call()
            except Exception as error:
                outcome['error'] = error

        thread = threading.Thread(target=submit, daemon=True)
        thread.start()
        deadline = time.monotonic() + COMMAND_DEADLINE_S
        while (self.supervisor._commands.empty() and not outcome
               and time.monotonic() < deadline):
            time.sleep(0.001)
        while not outcome and time.monotonic() < deadline:
            self.supervisor.tick()
        thread.join(timeout=5.0)
        assert outcome, 'the supervisor never answered the command'
        if 'error' in outcome:
            raise outcome['error']
        return outcome['value']

    def start(self, arms, mode, controller_name=None, gains_sha256=None):
        """Submit a §6.7 start request and return its verdict."""
        return self.drive(lambda: self.supervisor.request_start(SessionRequest(
            arms=arms, mode=mode, controller_name=controller_name,
            gains_sha256=gains_sha256)))

    def enable(self, arm_id, enabled=True):
        """Submit a §6.13 enable command and return its verdict."""
        return self.drive(lambda: self.supervisor.request_arm_enable(arm_id, enabled))

    def jog(self, arm_id, joint_index, direction):
        """Submit a §6.13 jog command and return its verdict."""
        return self.drive(
            lambda: self.supervisor.request_arm_jog(arm_id, joint_index, direction))

    def recover(self, arm_id):
        """Submit a §6.13 recover command and return its verdict."""
        return self.drive(lambda: self.supervisor.request_arm_recover(arm_id))

    def force_state(self, state):
        """
        Pin the state machine, the way ``test_session_state_machine`` does.

        Used only to hold the machine in a state a tick would otherwise leave
        (``preflight``/``starting``), so a guard can be asked about it.
        """
        with self.supervisor._state_lock:
            self.supervisor._state = state

    # -- scripting the world -------------------------------------------

    def upload(self, fixture, controller_name, arms):
        """Upload one sample-gains fixture through the real store."""
        return self.gains.upload(read_gains(fixture), controller_name, arms)

    def set_joints(self, message=None):
        """Publish a joint sample stamped at the current fake time."""
        self.bridge.joint = (self.clock.monotonic_ns(),
                             dual_joint_state() if message is None else message)

    def refresh_joints(self):
        """Re-stamp the current joint sample at the current fake time."""
        if self.bridge.joint is not None:
            self.bridge.joint = (self.clock.monotonic_ns(), self.bridge.joint[1])

    def make_ready(self, arm_ids=('panda1', 'panda2'), arm_mode='dual',
                   controller_name=None):
        """Satisfy every §3.5 readiness criterion and every §7.1 health rule."""
        controllers = {'joint_state_broadcaster': 'active'}
        for name in expected_state_broadcasters(arm_ids, arm_mode):
            controllers[name] = 'active'
        if controller_name:
            controllers[controller_name] = 'active'
        self.bridge.controllers = controllers
        self.bridge.types = {name: 'franka_example_controllers/Stub'
                             for name in controllers}
        stamp = self.clock.monotonic_ns()
        for arm_id in arm_ids:
            self.bridge.robot_states[arm_id] = (stamp, healthy_robot_state())
            self.set_diagnostic(arm_id, 0)
        self.bridge.hardware = hardware_component()
        self.set_joints()

    def set_diagnostic(self, arm_id, level):
        """Publish one canonical per-arm diagnostic at ``level`` (F1's input)."""
        self.bridge.diagnostics[arm_id] = (self.clock.monotonic_ns(),
                                           diagnostic_at(arm_id, level))

    def prime_pose_cache(self):
        """
        Seed the §5.4 pose cache as a prior WATCH session would have.

        The cache accepts watch/motion poses only (review finding S4: a
        simulated pose never satisfies the fence gate), so a unit rig with
        no prior production session seeds the supervisor's cache directly
        from the bridge's current joint sample — the exact write a running
        watch session's tick performs.
        """
        from franka_web import health
        sample = self.bridge.joint_sample()
        assert sample is not None, 'prime the bridge joint sample first'
        now = self.clock.monotonic()
        for arm_id in ('panda1', 'panda2'):
            joints = health.extract_joints(arm_id, sample[1])
            if joints['complete']:
                self.supervisor._pose_cache[arm_id] = (
                    now, tuple(joints['positions']))

    def claim_lock(self):
        """Take the operator lock (the jog stream publishes only while it is held)."""
        self.token = self.lock.claim()
        assert self.token is not None
        return self.token

    def model(self, arm_id):
        """Return the arm's live JogTargetModel."""
        return self.supervisor._jog_models[arm_id]

    def enabled_flags(self):
        """Return a copy of the supervisor's per-arm enable flags."""
        with self.supervisor._state_lock:
            return dict(self.supervisor._arm_enabled)

    def fault_codes(self):
        """Return the codes of the currently reported fault reasons."""
        return [reason['code'] for reason in self.supervisor.frame()['fault']['reasons']]


def motion_running(tmp_path, arms='both', controller_name=IMPEDANCE,
                   fixture='valid_dual_impedance.yaml', gains_arms=None):
    """Build a harness whose motion session has reached ``running``."""
    harness = MotionHarness(tmp_path)
    record = harness.upload(fixture, controller_name, gains_arms or arms)
    arm_ids = ('panda1', 'panda2') if arms == 'both' else (arms,)
    harness.make_ready(arm_ids, 'dual' if arms == 'both' else 'single', controller_name)
    harness.prime_pose_cache()
    harness.start(arms=arms, mode='motion', controller_name=controller_name,
                  gains_sha256=record.config_sha256)
    harness.pump()
    assert harness.supervisor.state == 'running', harness.supervisor.frame()['session']
    return harness


def simple_running(tmp_path, mode):
    """Build a harness whose non-motion session (simulate/watch) is ``running``."""
    harness = MotionHarness(tmp_path)
    harness.make_ready()
    harness.start(arms='both', mode=mode)
    harness.pump()
    assert harness.supervisor.state == 'running'
    return harness


# ======================================================================
# §6.13 POST /api/arm/{arm_id}/enable — the guard matrix
# ======================================================================


class TestEnableGuards:
    """Every refusal ``_accept_arm_enable`` can produce, in the plan's order."""

    def test_stopped_session_is_not_running(self, tmp_path):
        """No session at all: session_not_running."""
        harness = MotionHarness(tmp_path)
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'session_not_running'

    def test_preflight_is_not_running(self, tmp_path):
        """A session still in preflight has no motion surface yet."""
        harness = motion_running(tmp_path)
        harness.force_state('preflight')
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'session_not_running'

    def test_starting_is_not_running(self, tmp_path):
        """A session still coming up refuses the enable rather than racing it."""
        harness = motion_running(tmp_path)
        harness.force_state('starting')
        harness.supervisor._starting_deadline = harness.clock.monotonic() + 1000.0
        harness.bridge.controllers = {}
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'session_not_running'

    def test_stopping_is_not_running(self, tmp_path):
        """A session on its way down refuses the enable."""
        harness = motion_running(tmp_path)
        harness.force_state('stopping')
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'session_not_running'

    def test_faulted_session_refuses_enable(self, tmp_path):
        """A faulted session answers session_faulted, never a fresh enable."""
        harness = motion_running(tmp_path)
        harness.launch_child.die(returncode=1)
        harness.pump(1)
        assert harness.supervisor.state == 'fault'
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'session_faulted'

    @pytest.mark.parametrize('mode', ['simulate', 'watch'])
    def test_non_motion_session_has_no_enable(self, tmp_path, mode):
        """Simulate and Watch sessions carry no motion surface at all."""
        harness = simple_running(tmp_path, mode)
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'not_motion_mode'

    def test_hold_controller_has_no_enable_surface(self, tmp_path):
        """§6.13: the hold controller holds the activation pose and offers no enable."""
        harness = motion_running(tmp_path, controller_name=HOLD,
                                 fixture='valid_dual_hold.yaml')
        assert harness.supervisor._jog_models == {}
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'not_motion_mode'
        assert 'hold controller' in excinfo.value.detail

    def test_arm_outside_a_one_arm_session(self, tmp_path):
        """A one-arm session refuses the other arm with arm_not_in_session."""
        harness = motion_running(
            tmp_path, arms='panda1', fixture='valid_single_impedance_panda1.yaml')
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda2')
        assert excinfo.value.code == 'arm_not_in_session'
        assert 'panda2' in excinfo.value.detail

    def test_absent_joint_sample(self, tmp_path):
        """No joint sample at all: joint_state_stale, never a blind enable."""
        harness = motion_running(tmp_path)
        harness.bridge.joint = None
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'joint_state_stale'

    def test_incomplete_joint_sample(self, tmp_path):
        """A sample missing one of the seven joints is stale, not partial."""
        harness = motion_running(tmp_path)
        harness.set_joints(partial_joint_state())
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'joint_state_stale'
        assert 'all 7 joints' in excinfo.value.detail

    def test_sample_older_than_the_enable_window(self, tmp_path):
        """§6.13 step 2: a sample older than 0.2 s cannot authorize an enable."""
        harness = motion_running(tmp_path)
        harness.clock.advance(config.ENABLE_JOINT_STATE_MAX_AGE_S + 0.05)
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'joint_state_stale'
        assert str(config.ENABLE_JOINT_STATE_MAX_AGE_S) in excinfo.value.detail

    def test_sample_just_inside_the_enable_window_is_accepted(self, tmp_path):
        """The window is a limit, not a margin: just under 0.2 s still enables."""
        harness = motion_running(tmp_path)
        harness.clock.advance(config.ENABLE_JOINT_STATE_MAX_AGE_S - 0.05)
        assert harness.enable('panda1')['enabled'] is True

    def test_measured_pose_outside_the_fence(self, tmp_path):
        """§5.4: an out-of-fence measured pose is refused, naming the joints."""
        harness = motion_running(tmp_path)
        harness.set_joints(dual_joint_state(pose_1=OUT_OF_FENCE_POSE))
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'pose_outside_fence'
        assert 'joint4' in excinfo.value.detail
        assert harness.model('panda1').seeded is False

    def test_enable_service_not_ready(self, tmp_path):
        """An unreachable enable service is 503 enable_service_unavailable."""
        harness = motion_running(tmp_path)
        harness.bridge.enable_ready = False
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'enable_service_unavailable'
        assert harness.bridge.enable_calls == []
        assert harness.enabled_flags()['panda1'] is False

    def test_enable_service_does_not_answer(self, tmp_path):
        """A service that never answers is unavailable, not a silent success."""
        harness = motion_running(tmp_path)
        harness.bridge.enable_response = None
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'enable_service_unavailable'
        assert harness.enabled_flags()['panda1'] is False

    def test_controller_refusal_is_passed_through(self, tmp_path):
        """success=false is enable_rejected carrying the controller's own message."""
        harness = motion_running(tmp_path)
        harness.bridge.enable_response = {'success': False,
                                          'message': 'rejected: not stably active'}
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'enable_rejected'
        assert excinfo.value.detail == 'rejected: not stably active'
        assert harness.enabled_flags()['panda1'] is False


class TestEnableHappyPath:
    """The §6.13 ordered enable, observed step by step."""

    def test_enable_returns_the_measured_target_and_message(self, tmp_path):
        """The target is the measured pose and the message is the controller's."""
        harness = motion_running(tmp_path)
        harness.bridge.enable_response = {'success': True,
                                          'message': ENABLE_ENABLED_MESSAGE}
        result = harness.enable('panda1')
        assert result['arm_id'] == 'panda1'
        assert result['enabled'] is True
        assert result['target'] == pytest.approx(list(IN_FENCE_POSE))
        assert result['message'] == ENABLE_ENABLED_MESSAGE
        assert harness.bridge.enable_calls == [(1, True)]
        assert harness.model('panda1').target == pytest.approx(list(IN_FENCE_POSE))

    def test_the_flag_is_set_only_after_the_service_succeeds(self, tmp_path):
        """
        §6.13 step 6, observed from inside the service call.

        The jog timer publishes off this flag, so a flag set before the
        controller has agreed would stream targets at an arm the controller
        still considers disabled.
        """
        harness = motion_running(tmp_path)
        observed = {}

        def watch(slot, enabled):
            observed['slot'] = slot
            observed['flag'] = harness.enabled_flags()['panda1']
            observed['seeded'] = harness.model('panda1').seeded

        harness.bridge.on_enable = watch
        harness.enable('panda1')
        # Seeded BEFORE the call (step 4), flagged only AFTER it (step 6).
        assert observed == {'slot': 1, 'flag': False, 'seeded': True}
        assert harness.enabled_flags()['panda1'] is True

    def test_a_rejected_enable_leaves_no_flag_behind(self, tmp_path):
        """A refused enable is not a partial enable."""
        harness = motion_running(tmp_path)
        harness.bridge.enable_response = {'success': False, 'message': 'no'}
        with pytest.raises(SessionError):
            harness.enable('panda1')
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_the_second_arm_is_independent(self, tmp_path):
        """Enabling one arm never enables the other, and the slots are distinct."""
        harness = motion_running(tmp_path)
        harness.enable('panda2')
        assert harness.enabled_flags() == {'panda1': False, 'panda2': True}
        assert harness.bridge.enable_calls == [(2, True)]

    def test_the_frame_reports_the_motion_block(self, tmp_path):
        """§6.11: the per-arm motion block follows the enable."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        motion = harness.supervisor.frame()['arms']['panda1']['motion']
        assert motion['available'] is True
        assert motion['enabled'] is True
        assert motion['target'] == pytest.approx(list(IN_FENCE_POSE))
        assert motion['pose_inside_fence'] is True
        assert motion['targets_published'] == 0
        assert motion['enable_service_available'] is True


class TestDisableOrdering:
    """``enabled: false`` clears the flag and the target BEFORE it calls out."""

    def test_flag_and_target_drop_before_the_service_call(self, tmp_path):
        """
        Observed from inside ``call_enable``: both are already gone.

        Ordering is the whole point of this path. Calling the controller
        first would leave the 20 Hz publisher running against a target the
        arm is about to stop honouring.
        """
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        observed = {}

        def watch(slot, enabled):
            if not enabled:
                observed['flag'] = harness.enabled_flags()['panda1']
                observed['seeded'] = harness.model('panda1').seeded

        harness.bridge.on_enable = watch
        harness.bridge.enable_response = {'success': True,
                                          'message': ENABLE_DISABLED_MESSAGE}
        result = harness.enable('panda1', enabled=False)
        assert observed == {'flag': False, 'seeded': False}
        assert result == {'arm_id': 'panda1', 'enabled': False, 'target': None,
                          'message': ENABLE_DISABLED_MESSAGE}

    def test_a_failed_disable_still_leaves_the_flag_off(self, tmp_path):
        """A controller that refuses the disable does not re-enable the arm."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        harness.bridge.enable_response = {'success': False, 'message': 'busy'}
        result = harness.enable('panda1', enabled=False)
        assert result['enabled'] is False
        assert result['message'] == 'busy'
        assert harness.enabled_flags()['panda1'] is False

    def test_an_unanswered_disable_still_leaves_the_flag_off(self, tmp_path):
        """No answer is not a reason to keep streaming: the stream stops anyway."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        harness.bridge.enable_response = None
        result = harness.enable('panda1', enabled=False)
        assert result['enabled'] is False
        assert 'did not answer' in result['message']
        assert harness.enabled_flags()['panda1'] is False

    def test_a_disabled_arm_refuses_the_next_jog(self, tmp_path):
        """The disable is complete: the jog surface closes with it."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        harness.enable('panda1', enabled=False)
        with pytest.raises(SessionError) as excinfo:
            harness.jog('panda1', 0, 1)
        assert excinfo.value.code == 'arm_not_enabled'


# ======================================================================
# §6.13 POST /api/arm/{arm_id}/jog
# ======================================================================


class TestJogGuards:
    """The jog matrix at session level (the model itself is pinned elsewhere)."""

    def test_jog_before_enable_is_refused(self, tmp_path):
        """arm_not_enabled: a jog is never an implicit enable."""
        harness = motion_running(tmp_path)
        with pytest.raises(SessionError) as excinfo:
            harness.jog('panda1', 0, 1)
        assert excinfo.value.code == 'arm_not_enabled'

    @pytest.mark.parametrize('joint_index', [-1, 7, 99])
    def test_out_of_range_joint_is_invalid_joint(self, tmp_path, joint_index):
        """A joint index outside 0..6 reaches the model and comes back 400."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        with pytest.raises(SessionError) as excinfo:
            harness.jog('panda1', joint_index, 1)
        assert excinfo.value.code == 'invalid_joint'

    def test_a_jog_moves_the_model_exactly_one_step(self, tmp_path):
        """One press is exactly one ``JOG_STEP_RAD``, nothing more."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        result = harness.jog('panda1', 2, 1)
        expected = list(IN_FENCE_POSE)
        expected[2] += config.JOG_STEP_RAD
        assert result['arm_id'] == 'panda1'
        assert result['target'] == pytest.approx(expected)
        assert result['clamped'] == [False] * config.JOINT_COUNT
        assert harness.model('panda1').target == pytest.approx(expected)

    def test_a_negative_jog_moves_the_other_way(self, tmp_path):
        """A direction of -1 subtracts exactly one step."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        result = harness.jog('panda1', 0, -1)
        assert result['target'][0] == pytest.approx(IN_FENCE_POSE[0] - config.JOG_STEP_RAD)

    def test_a_clamped_jog_reports_the_mask(self, tmp_path):
        """The fence cuts the step short and the mask says which joint it was."""
        harness = motion_running(tmp_path)
        harness.set_joints(dual_joint_state(pose_1=NEAR_UPPER_POSE))
        harness.enable('panda1')
        result = harness.jog('panda1', 3, 1)
        upper = harness.gains.get(
            harness.supervisor.frame()['session']['gains_sha256']
        ).fence['panda1']['position_upper'][3]
        assert result['target'][3] == pytest.approx(upper)
        assert result['clamped'] == [False, False, False, True, False, False, False]

    def test_jog_while_faulted_is_refused(self, tmp_path):
        """A faulted session answers session_faulted, never a step."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        harness.launch_child.die(returncode=1)
        harness.pump(1)
        assert harness.supervisor.state == 'fault'
        with pytest.raises(SessionError) as excinfo:
            harness.jog('panda1', 0, 1)
        assert excinfo.value.code == 'session_faulted'

    def test_jog_on_a_hold_session_has_no_surface(self, tmp_path):
        """The hold controller has no target topic and so no jog."""
        harness = motion_running(tmp_path, controller_name=HOLD,
                                 fixture='valid_dual_hold.yaml')
        with pytest.raises(SessionError) as excinfo:
            harness.jog('panda1', 0, 1)
        assert excinfo.value.code == 'not_motion_mode'


# ======================================================================
# The 20 Hz jog stream
# ======================================================================


class TestJogStreamTick:
    """``jog_stream_tick`` publishes only while every condition holds."""

    def test_the_stream_publishes_when_everything_holds(self, tmp_path):
        """Running + motion + jog controller + enabled + lock held: it streams."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        for _ in range(3):
            harness.supervisor.jog_stream_tick()
        assert published(harness) == 3
        slot, message = harness.bridge.published_targets[-1]
        assert slot == 1
        assert list(message.joint_names) == list(health.joint_names_for('panda1'))
        assert list(message.points[0].positions) == pytest.approx(list(IN_FENCE_POSE))
        motion = harness.supervisor.frame()['arms']['panda1']['motion']
        assert motion['targets_published'] == 3
        assert motion['last_publish_age_s'] == 0.0
        assert harness.supervisor.frame()['arms']['panda2']['motion']['targets_published'] == 0

    def test_each_condition_alone_stops_the_stream(self, tmp_path):
        """
        Drive every publishing condition False in turn; none of them may publish.

        Each condition is restored before the next is broken, and the final
        assertion proves the stream was still capable of publishing all along
        -- otherwise this test would pass with a permanently broken stream.
        """
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.supervisor.jog_stream_tick()
        baseline = published(harness)
        assert baseline == 1
        supervisor = harness.supervisor

        # 1. not running
        harness.force_state('starting')
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        harness.force_state('running')

        # 2. not motion mode
        with supervisor._state_lock:
            supervisor._session['mode'] = 'watch'
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        with supervisor._state_lock:
            supervisor._session['mode'] = 'motion'

        # 3. not a jog controller
        with supervisor._state_lock:
            supervisor._session['controller_name'] = HOLD
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        with supervisor._state_lock:
            supervisor._session['controller_name'] = IMPEDANCE

        # 4. no arm enabled
        with supervisor._state_lock:
            supervisor._arm_enabled['panda1'] = False
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        with supervisor._state_lock:
            supervisor._arm_enabled['panda1'] = True

        # 5. the operator lock is not held
        assert harness.lock.release(harness.token) is True
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        harness.claim_lock()

        # 6. the target is not seeded
        harness.model('panda1').invalidate()
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        harness.model('panda1').seed(IN_FENCE_POSE)

        supervisor.jog_stream_tick()
        assert published(harness) == baseline + 1

    def test_lock_expiry_stops_the_stream(self, tmp_path):
        """§5.6: an expired lock stops publication without anyone releasing it."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.supervisor.jog_stream_tick()
        assert published(harness) == 1
        harness.clock.advance(config.OPERATOR_LOCK_TTL_S + 0.1)
        assert harness.lock.state()['locked'] is False
        harness.supervisor.jog_stream_tick()
        assert published(harness) == 1

    def test_a_jog_is_what_the_stream_carries(self, tmp_path):
        """The stream publishes the jogged target, not the seeded one."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        jogged = harness.jog('panda1', 5, -1)['target']
        harness.supervisor.jog_stream_tick()
        _, message = harness.bridge.published_targets[-1]
        assert list(message.points[0].positions) == pytest.approx(jogged)

    def test_both_arms_stream_independently(self, tmp_path):
        """Two enabled arms publish on their own slots, once each per tick."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.enable('panda2')
        harness.supervisor.jog_stream_tick()
        assert sorted(slot for slot, _ in harness.bridge.published_targets) == [1, 2]


# ======================================================================
# §5.4 fence-vs-pose precondition, at session start
# ======================================================================


class TestFencePosePrecondition:
    """A motion session may not start against a pose nobody has verified."""

    def _prepared(self, tmp_path):
        """Return a harness with gains uploaded and the graph ready, not started."""
        harness = MotionHarness(tmp_path)
        record = harness.upload('valid_dual_impedance.yaml', IMPEDANCE, 'both')
        harness.make_ready(controller_name=IMPEDANCE)
        return harness, record

    def test_no_cached_pose_refuses_the_start(self, tmp_path):
        """fence_pose_unverified: run a Watch session first."""
        harness, record = self._prepared(tmp_path)
        harness.bridge.joint = None
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='both', mode='motion', controller_name=IMPEDANCE,
                          gains_sha256=record.config_sha256)
        assert excinfo.value.code == 'fence_pose_unverified'
        assert harness.supervisor.state == 'stopped'

    def test_a_stale_cached_pose_refuses_the_start(self, tmp_path):
        """A pose older than POSE_CACHE_TTL_S is no longer evidence."""
        harness, record = self._prepared(tmp_path)
        harness.prime_pose_cache()
        # The stream has since gone away (the watch session ended), so nothing
        # refreshes the cache while the clock runs past its TTL.
        harness.bridge.joint = None
        harness.clock.advance(config.POSE_CACHE_TTL_S + 1.0)
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='both', mode='motion', controller_name=IMPEDANCE,
                          gains_sha256=record.config_sha256)
        assert excinfo.value.code == 'fence_pose_unverified'

    def test_a_cached_pose_outside_the_fence_refuses_the_start(self, tmp_path):
        """§5.4: a fence that does not contain the pose commands motion at enable."""
        harness, record = self._prepared(tmp_path)
        harness.set_joints(dual_joint_state(pose_2=OUT_OF_FENCE_POSE))
        harness.prime_pose_cache()
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='both', mode='motion', controller_name=IMPEDANCE,
                          gains_sha256=record.config_sha256)
        assert excinfo.value.code == 'pose_outside_fence'
        assert 'panda2' in excinfo.value.detail
        assert 'joint4' in excinfo.value.detail

    def test_a_good_cached_pose_starts(self, tmp_path):
        """A fresh in-fence pose lets the start proceed to preflight."""
        harness, record = self._prepared(tmp_path)
        harness.prime_pose_cache()
        accepted = harness.start(arms='both', mode='motion', controller_name=IMPEDANCE,
                                 gains_sha256=record.config_sha256)
        # The verdict is 'preflight'; the state itself is not asserted here,
        # because the supervisor keeps ticking while the waiting thread is
        # scheduled and may legitimately be further along already.
        assert accepted['state'] == 'preflight'
        assert accepted['session_id'].startswith('web-')
        harness.pump()
        assert harness.supervisor.state == 'running'
        assert harness.supervisor.frame()['session']['gains_sha256'] == record.config_sha256

    def test_the_hold_controller_needs_no_pose(self, tmp_path):
        """The precondition guards the jog fence; the hold controller has none."""
        harness = MotionHarness(tmp_path)
        record = harness.upload('valid_dual_hold.yaml', HOLD, 'both')
        harness.make_ready(controller_name=HOLD)
        harness.bridge.joint = None
        accepted = harness.start(arms='both', mode='motion', controller_name=HOLD,
                                 gains_sha256=record.config_sha256)
        assert accepted['state'] == 'preflight'


# ======================================================================
# §6.7 gains matching, at session start
# ======================================================================


class TestGainsMatchingAtStart:
    """A motion start is refused unless the uploaded config matches the request."""

    def test_missing_sha_is_gains_required(self, tmp_path):
        """gains_required: motion mode never starts on unreviewed numbers."""
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=IMPEDANCE)
        harness.prime_pose_cache()
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='both', mode='motion', controller_name=IMPEDANCE)
        assert excinfo.value.code == 'gains_required'

    def test_unknown_sha_is_gains_unknown(self, tmp_path):
        """gains_unknown: the page is stale, or the store was restarted."""
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=IMPEDANCE)
        harness.prime_pose_cache()
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='both', mode='motion', controller_name=IMPEDANCE,
                          gains_sha256='0' * 64)
        assert excinfo.value.code == 'gains_unknown'

    def test_controller_mismatch(self, tmp_path):
        """A hold config aimed at the impedance controller is the wrong file."""
        harness = MotionHarness(tmp_path)
        record = harness.upload('valid_dual_hold.yaml', HOLD, 'both')
        harness.make_ready(controller_name=IMPEDANCE)
        harness.prime_pose_cache()
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='both', mode='motion', controller_name=IMPEDANCE,
                          gains_sha256=record.config_sha256)
        assert excinfo.value.code == 'gains_controller_mismatch'

    def test_arms_mismatch(self, tmp_path):
        """A two-arm config aimed at a one-arm session is refused before launch."""
        harness = MotionHarness(tmp_path)
        record = harness.upload('valid_dual_impedance.yaml', IMPEDANCE, 'both')
        harness.make_ready(('panda1',), 'single', IMPEDANCE)
        harness.prime_pose_cache()
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='panda1', mode='motion', controller_name=IMPEDANCE,
                          gains_sha256=record.config_sha256)
        assert excinfo.value.code == 'gains_arms_mismatch'

    def test_the_velocity_controller_is_not_offered(self, tmp_path):
        """
        controller_not_reviewed: the web allowlist is a strict subset.

        ``dual_arm_joint_velocity_controller`` is reviewed in the validator and
        deliberately excluded from this interface, so the refusal comes from
        the session, before the store is ever consulted.
        """
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=VELOCITY)
        harness.prime_pose_cache()
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='both', mode='motion', controller_name=VELOCITY,
                          gains_sha256='0' * 64)
        assert excinfo.value.code == 'controller_not_reviewed'
        assert VELOCITY not in excinfo.value.detail

    def test_no_controller_name_at_all(self, tmp_path):
        """A motion start without a controller is the same refusal."""
        harness = MotionHarness(tmp_path)
        harness.make_ready()
        harness.prime_pose_cache()
        with pytest.raises(SessionError) as excinfo:
            harness.start(arms='both', mode='motion')
        assert excinfo.value.code == 'controller_not_reviewed'


# ======================================================================
# §7.3 recover — the one-click sequence
# ======================================================================


def fault_by_hardware(harness):
    """Drive a running session to ``fault`` through F8 (a recoverable fault)."""
    harness.bridge.hardware = hardware_component('inactive')
    harness.pump(1)
    assert harness.supervisor.state == 'fault'
    return harness


def fault_by_diagnostic(harness, arm_id='panda1'):
    """Drive a running session to ``fault`` through F1 (a recoverable fault)."""
    harness.set_diagnostic(arm_id, 2)
    harness.pump(1)
    assert harness.supervisor.state == 'fault'
    return harness


class TestRecoverSequencing:
    """§6.13/§7.3: the ordered sequence, its steps, and what it never does."""

    def test_recover_on_a_running_session_is_refused(self, tmp_path):
        """not_faulted: recovery is a fault-only action."""
        harness = motion_running(tmp_path)
        with pytest.raises(SessionError) as excinfo:
            harness.recover('panda1')
        assert excinfo.value.code == 'not_faulted'

    def test_recover_on_a_simulate_session_is_refused(self, tmp_path):
        """not_production_mode: mock hardware has nothing to recover."""
        harness = simple_running(tmp_path, 'simulate')
        harness.launch_child.die(returncode=1)
        harness.pump(1)
        assert harness.supervisor.state == 'fault'
        with pytest.raises(SessionError) as excinfo:
            harness.recover('panda1')
        assert excinfo.value.code == 'not_production_mode'

    def test_recover_names_an_arm_of_the_session(self, tmp_path):
        """A one-arm session refuses a recover aimed at the other arm."""
        harness = motion_running(
            tmp_path, arms='panda1', fixture='valid_single_impedance_panda1.yaml')
        fault_by_hardware(harness)
        with pytest.raises(SessionError) as excinfo:
            harness.recover('panda2')
        assert excinfo.value.code == 'arm_not_in_session'

    def test_unreachable_recovery_service(self, tmp_path):
        """503 recovery_service_unavailable when the service never answers."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.recovery_response = None
        with pytest.raises(SessionError) as excinfo:
            harness.recover('panda1')
        assert excinfo.value.code == 'recovery_service_unavailable'
        assert harness.bridge.recovery_calls == ['panda1']

    def test_no_errors_is_informational_not_a_failure(self, tmp_path):
        """
        §0.8: ``success=false, error='No errors'`` is a neutral note.

        The backend answers this when there was nothing to clear. Treating it
        as a failure would put a red error in front of an operator whose robot
        is fine, and would abort the rest of the sequence.
        """
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.recovery_response = {'success': False, 'error': NO_ERRORS_MESSAGE}
        result = harness.recover('panda1')
        assert result['steps'][0] == {'step': 'error_recovery', 'ok': True,
                                      'detail': NO_ERRORS_MESSAGE}
        assert result['enabled_after'] is False

    def test_a_hard_recovery_failure_stops_the_sequence(self, tmp_path):
        """502 recovery_failed, carrying the backend's own error text."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.recovery_response = {'success': False,
                                            'error': 'reflex not cleared'}
        with pytest.raises(SessionError) as excinfo:
            harness.recover('panda1')
        assert excinfo.value.code == 'recovery_failed'
        assert 'reflex not cleared' in excinfo.value.detail
        assert harness.bridge.hardware_active_calls == []
        assert harness.bridge.switch_calls == []

    def test_an_inactive_hardware_component_is_reactivated(self, tmp_path):
        """§7.3: the hardware step runs only when the component is not active."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        fault_by_hardware(harness)
        # The controller is still listed active, so the switch is SKIPPED
        # (STRICT would refuse an already-active activate — finding S1).
        result = harness.recover('panda1')
        assert harness.bridge.hardware_active_calls == [HARDWARE_NAME]
        assert result['steps'] == [
            {'step': 'error_recovery', 'ok': True, 'detail': ''},
            {'step': 'hardware_active', 'ok': True, 'detail': 'active'},
            {'step': 'reactivate_controllers', 'ok': True,
             'detail': '{} (already active)'.format(IMPEDANCE)},
        ]
        assert harness.bridge.switch_calls == []
        assert result['enabled_after'] is False
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_an_active_hardware_component_is_left_alone(self, tmp_path):
        """An already-active component reports ok without being touched."""
        harness = motion_running(tmp_path)
        harness.launch_child.die(returncode=1)
        harness.pump(1)
        result = harness.recover('panda1')
        assert harness.bridge.hardware_active_calls == []
        assert result['steps'][1] == {'step': 'hardware_active', 'ok': True,
                                      'detail': 'active'}

    def test_a_refused_hardware_activation_fails_the_recovery(self, tmp_path):
        """The sequence stops at the step that failed, and says which."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.hardware_response = {'ok': False}
        with pytest.raises(SessionError) as excinfo:
            harness.recover('panda1')
        assert excinfo.value.code == 'recovery_failed'
        assert 'hardware' in excinfo.value.detail
        assert harness.bridge.switch_calls == []

    def test_a_refused_controller_switch_fails_the_recovery(self, tmp_path):
        """A deactivated controller that will not re-activate fails recovery."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        # The switch runs only for a controller that actually left 'active'
        # (finding S1); make it inactive so the refused switch is reached.
        harness.bridge.controllers[IMPEDANCE] = 'inactive'
        harness.bridge.switch_response = {'ok': False}
        with pytest.raises(SessionError) as excinfo:
            harness.recover('panda1')
        assert excinfo.value.code == 'recovery_failed'
        assert 'activation' in excinfo.value.detail

    def test_a_deactivated_controller_is_switched_back(self, tmp_path):
        """A controller that left 'active' is re-activated via the switch."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.controllers[IMPEDANCE] = 'inactive'
        result = harness.recover('panda1')
        assert harness.bridge.switch_calls == [[IMPEDANCE]]
        assert result['steps'][2] == {
            'step': 'reactivate_controllers', 'ok': True, 'detail': IMPEDANCE}

    def test_a_watch_session_has_no_controller_step(self, tmp_path):
        """Only a motion session carries a controller to re-activate."""
        harness = simple_running(tmp_path, 'watch')
        fault_by_hardware(harness)
        result = harness.recover('panda1')
        assert [step['step'] for step in result['steps']] == [
            'error_recovery', 'hardware_active']
        assert harness.bridge.switch_calls == []

    def test_recover_forces_the_enables_off_first(self, tmp_path):
        """
        Recovery starts from a disabled arm, always.

        The fault transition already cleared the flags; recover clears them
        again and invalidates every held target, so nothing can be published
        between the recovery and the operator's fresh authorization.
        """
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        fault_by_hardware(harness)
        with harness.supervisor._state_lock:
            harness.supervisor._arm_enabled['panda1'] = True     # a stuck flag
        harness.recover('panda1')
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.model('panda1').seeded is False
        before = published(harness)
        harness.supervisor.jog_stream_tick()
        assert published(harness) == before

    def test_a_successful_recovery_returns_the_session_to_running(self, tmp_path):
        """§7.3: fault -> running once the rules stop firing, every enable off."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        fault_by_diagnostic(harness)
        assert harness.supervisor.frame()['fault']['recoverable'] is True
        # The clearing the ErrorRecovery service really performs, made visible
        # to the next fault evaluation.
        harness.bridge.on_recovery_success = lambda arm_id: harness.set_diagnostic(arm_id, 0)
        result = harness.recover('panda1')
        assert result['enabled_after'] is False
        harness.pump(2)
        assert harness.supervisor.state == 'running'
        # Recovery returns the arm to state-only reading: still off, every arm.
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.supervisor.frame()['fault']['active'] is False
        before = published(harness)
        harness.supervisor.jog_stream_tick()
        assert published(harness) == before

    def test_a_still_firing_rule_keeps_the_session_faulted(self, tmp_path):
        """A recovery the world does not agree with leaves the session in fault."""
        harness = motion_running(tmp_path)
        fault_by_diagnostic(harness)
        result = harness.recover('panda1')
        assert all(step['ok'] for step in result['steps'])
        harness.refresh_joints()
        harness.pump(3)
        assert harness.supervisor.state == 'fault'
        assert 'diagnostic_error' in harness.fault_codes()


# ======================================================================
# F5 wiring, and the operator-release path
# ======================================================================


class TestControllerDeactivationWiring:
    """F5: the session controller leaving ``active`` is a motion fault."""

    def test_the_session_faults_with_enables_forced_off(self, tmp_path):
        """A deactivated controller faults the session and disarms every arm."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.supervisor.jog_stream_tick()
        published_before = published(harness)
        harness.bridge.controllers[IMPEDANCE] = 'inactive'
        harness.pump(1)
        assert harness.supervisor.state == 'fault'
        assert 'controller_deactivated' in harness.fault_codes()
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        harness.supervisor.jog_stream_tick()
        assert published(harness) == published_before

    def test_the_fault_is_not_recoverable(self, tmp_path):
        """§7.1: F5 alone offers no Recover button — stop and restart instead."""
        harness = motion_running(tmp_path)
        harness.bridge.controllers[IMPEDANCE] = 'inactive'
        harness.pump(1)
        frame = harness.supervisor.frame()
        assert frame['fault']['recoverable'] is False
        assert frame['fault']['recover_hint'] is None

    def test_an_unloaded_controller_faults_too(self, tmp_path):
        """A controller that vanished from list_controllers is not active either."""
        harness = motion_running(tmp_path)
        del harness.bridge.controllers[IMPEDANCE]
        harness.pump(1)
        assert harness.supervisor.state == 'fault'
        assert 'controller_deactivated' in harness.fault_codes()


class TestOperatorRelease:
    """§6.4/§5.6: losing the operator stops the arms, both locally and remotely."""

    def test_release_drops_the_flags_and_queues_a_disable(self, tmp_path):
        """The flags go first (the stream reads them), the service call follows."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.enable('panda2')
        calls_before = len(harness.bridge.enable_calls)

        harness.supervisor.operator_released()
        # Immediate, on the caller's thread: the stream cannot publish again.
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.supervisor._commands.empty() is False
        before = published(harness)
        harness.supervisor.jog_stream_tick()
        assert published(harness) == before

        harness.pump(1)
        disables = harness.bridge.enable_calls[calls_before:]
        assert sorted(disables) == [(1, False), (2, False)]
        assert harness.model('panda1').seeded is False
        assert harness.model('panda2').seeded is False

    def test_release_on_a_non_motion_session_queues_nothing(self, tmp_path):
        """A Watch session has no controller to disable, so nothing is queued."""
        harness = simple_running(tmp_path, 'watch')
        harness.supervisor.operator_released()
        assert harness.supervisor._commands.empty() is True
        assert harness.bridge.enable_calls == []


# ======================================================================
# HTTP level — only what the table-driven token test cannot cover
# ======================================================================
#
# ``test_http_api.py`` already parametrizes every ``needs_token`` route
# against a missing/expired/wrong token (401), the motion rows included.
# What follows is what that table cannot reach: the ``{arm_id}`` path
# segment, the two motion body shapes, and §6.5 against a real store.


class FakeMotionSupervisor:
    """
    Scriptable stand-in for the supervisor's HTTP-thread surface.

    Only the methods ``http_api`` actually calls exist; anything else a
    handler reached for would be a contract change this file should notice.
    """

    def __init__(self):
        """Accept every command and record what arrived."""
        self.enable_calls = []
        self.jog_calls = []
        self.recover_calls = []
        self.releases = 0
        self.error = None
        self.enable_result = {'arm_id': 'panda1', 'enabled': True,
                              'target': [0.0] * config.JOINT_COUNT,
                              'message': ENABLE_ENABLED_MESSAGE}
        self.jog_result = {'arm_id': 'panda1', 'target': [0.0] * config.JOINT_COUNT,
                           'clamped': [False] * config.JOINT_COUNT}
        self.recover_result = {'arm_id': 'panda1', 'steps': [], 'enabled_after': False}

    def _answer(self, result):
        """Return the scripted result, or raise the scripted refusal."""
        if self.error is not None:
            raise self.error
        return dict(result)

    def request_arm_enable(self, arm_id, enabled):
        """Record the §6.13 enable and answer with the scripted verdict."""
        self.enable_calls.append((arm_id, enabled))
        return self._answer(self.enable_result)

    def request_arm_jog(self, arm_id, joint_index, direction):
        """Record the §6.13 jog and answer with the scripted verdict."""
        self.jog_calls.append((arm_id, joint_index, direction))
        return self._answer(self.jog_result)

    def request_arm_recover(self, arm_id):
        """Record the §6.13 recover and answer with the scripted verdict."""
        self.recover_calls.append(arm_id)
        return self._answer(self.recover_result)

    def operator_released(self):
        """Count the §6.4 release notification."""
        self.releases += 1

    def frame(self):
        """Return a placeholder state frame; no test here reads /api/state."""
        return {}


class Response:
    """One completed HTTP response: status, headers and raw body."""

    def __init__(self, status, headers, body):
        """Wrap the status line, the header object and the body bytes."""
        self.status = status
        self.headers = headers
        self.body = body

    def json(self):
        """Decode the body as JSON."""
        return json.loads(self.body.decode('utf-8'))


def free_port():
    """Return a loopback port that was free a moment ago."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class MotionServer:
    """A real HTTP server on loopback, wired to a fake supervisor and a real store."""

    def __init__(self, tmp_path):
        """Bind, wire and start serving on a daemon thread."""
        state_dir = tmp_path / 'state'
        state_dir.mkdir(mode=0o700, exist_ok=True)
        recording_root = tmp_path / 'recordings'
        recording_root.mkdir(mode=0o700, exist_ok=True)
        self.clock = FakeClock()
        self.supervisor = FakeMotionSupervisor()
        self.lock = OperatorLock(monotonic=self.clock.monotonic)
        self.broker = Broker()
        self.gains = GainsStore(str(state_dir))
        self.settings = None
        self.httpd = None
        for _ in range(10):
            settings = Settings(
                bind='127.0.0.1', port=free_port(), state_dir=str(state_dir),
                recording_root=str(recording_root), ros_domain_id=80)
            app = App(settings=settings, supervisor=self.supervisor, lock=self.lock,
                      broker=self.broker, static_root=STATIC_ROOT,
                      gains_store=self.gains)
            try:
                self.httpd = build_server(app)
            except OSError:
                continue        # the probe port was taken in the meantime
            self.settings = settings
            break
        assert self.httpd is not None, 'could not bind a free loopback port'
        self.thread = threading.Thread(
            target=self.httpd.serve_forever, args=(0.02,),
            name='test-motion-http', daemon=True)
        self.thread.start()

    @property
    def port(self):
        """Return the bound port."""
        return self.settings.port

    def request(self, method, path, body=None, headers=None):
        """
        Send one request and return the :class:`Response`.

        ``path`` reaches the wire verbatim -- ``http.client`` never normalizes
        it -- which is what makes the traversal-shaped ``{arm_id}`` cases
        meaningful. A caller-supplied ``Content-Length`` is left alone, so a
        body can be DECLARED and never sent.
        """
        if isinstance(body, str):
            body = body.encode('utf-8')
        connection = http.client.HTTPConnection(
            '127.0.0.1', self.port, timeout=REQUEST_TIMEOUT_S)
        try:
            connection.request(method, path, body=body, headers=dict(headers or {}))
            response = connection.getresponse()
            return Response(response.status, response.headers, response.read())
        finally:
            connection.close()

    def claim(self):
        """Claim the operator lock over HTTP and return the token."""
        response = self.request('POST', '/api/operator/claim')
        assert response.status == 200, response.body
        return response.json()['token']

    def close(self):
        """Stop serving and retire the handler threads."""
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=5.0)


@pytest.fixture()
def server(tmp_path):
    """Serve one motion-wired app for the duration of a test."""
    running = MotionServer(tmp_path)
    yield running
    running.close()


def assert_error(response, code, status):
    """Assert the §6.0 failure envelope, its code and its status."""
    assert response.status == status, response.body
    body = response.json()
    assert set(body) == {'ok', 'error', 'detail'}
    assert body['ok'] is False
    assert body['error'] == code
    assert isinstance(body['detail'], str) and body['detail']
    return body


#: Path values that must never reach the supervisor. '' collapses the segment,
#: 'PANDA1' proves the comparison is not case-folded, and '..' proves the
#: segment is matched as data rather than resolved as a path.
BAD_ARM_IDS = ('panda3', '', 'PANDA1', '..', 'panda1%20')

MOTION_ROUTES = ('enable', 'jog', 'recover')


class TestArmIdPathSegment:
    """The ``{arm_id}`` placeholder is a closed set, checked before any body."""

    @pytest.mark.parametrize('endpoint', MOTION_ROUTES)
    @pytest.mark.parametrize('arm_id', BAD_ARM_IDS)
    def test_a_bad_arm_id_is_404_arm_not_in_session(self, server, endpoint, arm_id):
        """Anything but panda1/panda2 is refused before the supervisor is touched."""
        token = server.claim()
        response = server.request(
            'POST', '/api/arm/{}/{}'.format(arm_id, endpoint),
            body=json.dumps({'enabled': True, 'joint_index': 0, 'direction': 1}),
            headers={'X-Operator-Token': token})
        assert_error(response, 'arm_not_in_session', 404)
        assert server.supervisor.enable_calls == []
        assert server.supervisor.jog_calls == []
        assert server.supervisor.recover_calls == []

    @pytest.mark.parametrize('arm_id', ['panda1', 'panda2'])
    def test_both_real_arms_reach_the_supervisor(self, server, arm_id):
        """The two legal values dispatch, carrying the arm through unchanged."""
        token = server.claim()
        response = server.request('POST', '/api/arm/{}/recover'.format(arm_id),
                                  headers={'X-Operator-Token': token})
        assert response.status == 200, response.body
        assert server.supervisor.recover_calls == [arm_id]

    def test_the_arm_id_is_checked_before_the_body(self, server):
        """A bad arm and a bad body together answer for the arm, not the body."""
        token = server.claim()
        response = server.request('POST', '/api/arm/panda9/enable',
                                  body='{{{{ not json',
                                  headers={'X-Operator-Token': token})
        assert_error(response, 'arm_not_in_session', 404)


class TestEnableBodyShapes:
    """§6.13 enable takes exactly ``{"enabled": true|false}``."""

    @pytest.mark.parametrize('body', [
        None,                                   # no body at all
        '{}',                                   # 'enabled' missing
        '{"enabled": 1}',                       # 1 is not true
        '{"enabled": 0}',
        '{"enabled": "true"}',                  # the string is not the boolean
        '{"enabled": null}',
        '{"enabled": [true]}',
        '[]',                                   # not an object
        'not json at all',
    ])
    def test_a_bad_enable_body_is_invalid_json(self, server, body):
        """Every shape but a JSON boolean is 400 invalid_json."""
        token = server.claim()
        response = server.request('POST', '/api/arm/panda1/enable', body=body,
                                  headers={'X-Operator-Token': token})
        assert_error(response, 'invalid_json', 400)
        assert server.supervisor.enable_calls == []

    @pytest.mark.parametrize('enabled', [True, False])
    def test_a_boolean_reaches_the_supervisor(self, server, enabled):
        """Both booleans dispatch, and the §6.13 body comes back whole."""
        token = server.claim()
        response = server.request('POST', '/api/arm/panda2/enable',
                                  body=json.dumps({'enabled': enabled}),
                                  headers={'X-Operator-Token': token})
        assert response.status == 200, response.body
        assert server.supervisor.enable_calls == [('panda2', enabled)]
        body = response.json()
        assert body['ok'] is True
        assert set(body) == {'ok', 'arm_id', 'enabled', 'target', 'message'}


class TestJogBodyShapes:
    """§6.13 jog takes an integer ``joint_index`` and a ``direction`` of ±1."""

    @pytest.mark.parametrize('body', [
        '{}',
        '{"direction": 1}',                             # joint_index missing
        '{"joint_index": 0}',                           # direction missing
        '{"joint_index": true, "direction": 1}',        # a bool is not an index
        '{"joint_index": 1.0, "direction": 1}',         # a float is not an index
        '{"joint_index": "1", "direction": 1}',
        '{"joint_index": null, "direction": 1}',
        '{"joint_index": 0, "direction": 0}',           # 0 is not a direction
        '{"joint_index": 0, "direction": 2}',
        '{"joint_index": 0, "direction": -2}',
        '{"joint_index": 0, "direction": "1"}',
        '{"joint_index": 0, "direction": true}',        # True is not +1
        '{"joint_index": 0, "direction": false}',
        '{"joint_index": 0, "direction": 1.5}',
    ])
    def test_a_bad_jog_body_is_invalid_json(self, server, body):
        """Shape errors never reach the supervisor, and never move an arm."""
        token = server.claim()
        response = server.request('POST', '/api/arm/panda1/jog', body=body,
                                  headers={'X-Operator-Token': token})
        assert_error(response, 'invalid_json', 400)
        assert server.supervisor.jog_calls == []

    @pytest.mark.parametrize('direction', [-1, 1])
    def test_a_valid_jog_reaches_the_supervisor(self, server, direction):
        """Both directions dispatch with the index and sign intact."""
        token = server.claim()
        response = server.request(
            'POST', '/api/arm/panda1/jog',
            body=json.dumps({'joint_index': 6, 'direction': direction}),
            headers={'X-Operator-Token': token})
        assert response.status == 200, response.body
        assert server.supervisor.jog_calls == [('panda1', 6, direction)]
        assert set(response.json()) == {'ok', 'arm_id', 'target', 'clamped'}

    def test_a_whole_float_direction_is_refused_at_the_http_layer(self, server):
        """
        ``1.0`` is refused as invalid_json before reaching the supervisor.

        ``1.0 == 1`` in Python, so a bare value comparison would have let a
        JSON float through to the jog model; the HTTP guard requires an
        actual int (the gap this test originally pinned, fixed since).
        """
        token = server.claim()
        response = server.request('POST', '/api/arm/panda1/jog',
                                  body='{"joint_index": 0, "direction": 1.0}',
                                  headers={'X-Operator-Token': token})
        assert_error(response, 'invalid_json', 400)
        assert server.supervisor.jog_calls == []

    def test_an_out_of_range_index_is_the_session_s_refusal(self, server):
        """
        A shape-valid index out of range is invalid_joint, not invalid_json.

        The HTTP layer checks the TYPE; the range belongs to the jog model,
        whose fence the request has to be measured against.
        """
        token = server.claim()
        server.supervisor.error = SessionError('invalid_joint',
                                               'joint_index must be an integer in 0..6')
        response = server.request('POST', '/api/arm/panda1/jog',
                                  body=json.dumps({'joint_index': 7, 'direction': 1}),
                                  headers={'X-Operator-Token': token})
        assert_error(response, 'invalid_joint', 400)
        assert server.supervisor.jog_calls == [('panda1', 7, 1)]


class TestGainsUploadOverHttp:
    """§6.5/§6.6 against a REAL GainsStore: query handling, size cap, round trip."""

    def test_missing_controller_name(self, server):
        """No controller_name is not on the allowlist, so controller_not_reviewed."""
        token = server.claim()
        response = server.request('POST', '/api/gains?arms=both',
                                  body=read_gains('valid_dual_impedance.yaml'),
                                  headers={'X-Operator-Token': token})
        assert_error(response, 'controller_not_reviewed', 400)

    def test_unreviewed_controller_name(self, server):
        """The velocity controller is excluded from this interface by decision."""
        token = server.claim()
        response = server.request(
            'POST', '/api/gains?controller_name={}&arms=both'.format(VELOCITY),
            body=read_gains('valid_dual_impedance.yaml'),
            headers={'X-Operator-Token': token})
        assert_error(response, 'controller_not_reviewed', 400)

    def test_missing_arms(self, server):
        """No arms selection is invalid_arms, from the store's own closed set."""
        token = server.claim()
        response = server.request(
            'POST', '/api/gains?controller_name={}'.format(IMPEDANCE),
            body=read_gains('valid_dual_impedance.yaml'),
            headers={'X-Operator-Token': token})
        assert_error(response, 'invalid_arms', 400)

    def test_unknown_arms(self, server):
        """A third arm selection is refused the same way."""
        token = server.claim()
        response = server.request(
            'POST', '/api/gains?controller_name={}&arms=panda3'.format(IMPEDANCE),
            body=read_gains('valid_dual_impedance.yaml'),
            headers={'X-Operator-Token': token})
        assert_error(response, 'invalid_arms', 400)

    def test_an_oversize_declaration_is_refused_before_the_body(self, server):
        """
        A Content-Length past the cap is refused without reading a byte.

        The body is declared and never sent: if the handler read it first,
        this request would hang until the socket timed out instead of
        answering 400 immediately.
        """
        token = server.claim()
        response = server.request(
            'POST', '/api/gains?controller_name={}&arms=both'.format(IMPEDANCE),
            headers={'X-Operator-Token': token,
                     'Content-Length': str(config.MAX_GAINS_BYTES + 1)})
        body = assert_error(response, 'gains_too_large', 400)
        assert str(config.MAX_GAINS_BYTES) in body['detail']
        assert server.gains.entries() == []

    def test_an_invalid_config_carries_the_validator_s_message(self, server):
        """gains_invalid, with the validator's own sentence (§6.5)."""
        token = server.claim()
        response = server.request(
            'POST', '/api/gains?controller_name={}&arms=both'.format(IMPEDANCE),
            body=read_gains('invalid_fence_inverted.yaml'),
            headers={'X-Operator-Token': token})
        body = assert_error(response, 'gains_invalid', 400)
        assert body['detail']
        assert server.gains.entries() == []

    def test_a_valid_upload_round_trips(self, server):
        """The §6.5 body comes back, and §6.6 then lists the very same record."""
        token = server.claim()
        raw = read_gains('valid_dual_impedance.yaml')
        response = server.request(
            'POST', '/api/gains?controller_name={}&arms=both'.format(IMPEDANCE),
            body=raw, headers={'X-Operator-Token': token})
        assert response.status == 200, response.body
        body = response.json()
        assert set(body) == {'ok', 'config_sha256', 'controller_name', 'arms',
                             'uploaded_at', 'path', 'controller_type',
                             'command_interfaces', 'state_interfaces', 'fence'}
        assert body['ok'] is True
        assert body['controller_name'] == IMPEDANCE
        assert body['arms'] == ['panda1', 'panda2']
        assert body['controller_type']
        assert set(body['fence']) == {'panda1', 'panda2'}
        assert len(body['fence']['panda1']['position_lower']) == config.JOINT_COUNT
        assert body['path'].endswith('{}.yaml'.format(body['config_sha256']))

        listed = server.request('GET', '/api/gains')
        assert listed.status == 200, listed.body
        entries = listed.json()['gains']
        assert len(entries) == 1
        assert entries[0] == {
            'config_sha256': body['config_sha256'],
            'controller_name': IMPEDANCE,
            'arms': ['panda1', 'panda2'],
            'uploaded_at': body['uploaded_at'],
            'path': body['path'],
        }

    def test_a_re_upload_is_idempotent_on_the_wire(self, server):
        """The same bytes twice are the same record, listed once."""
        token = server.claim()
        raw = read_gains('valid_dual_impedance.yaml')
        query = '/api/gains?controller_name={}&arms=both'.format(IMPEDANCE)
        first = server.request('POST', query, body=raw,
                               headers={'X-Operator-Token': token})
        second = server.request('POST', query, body=raw,
                                headers={'X-Operator-Token': token})
        assert first.json() == second.json()
        assert len(server.request('GET', '/api/gains').json()['gains']) == 1
