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
The guard matrix for the motion surface, driven entirely with fakes.

Two levels, both unit:

* **Session level** -- a real :class:`~franka_web.session.SessionSupervisor`
  with a real :class:`~franka_web.gains.ProfileStore`, a real
  :class:`~franka_web.lock.OperatorLock` and a real
  :class:`~franka_web.jog.JogTargetModel`, wired to the
  ``support.fake_launcher`` fakes and a
  :class:`~support.fake_clock.FakeClock`. Every operator command is submitted
  the way an HTTP worker submits it (queue + wait) and answered by an explicit
  ``tick()``, so the ordering rules stated in prose -- *enable flag set only
  after the service succeeds*, *enable flag cleared before the disable call*,
  *enables forced off before anything else on fault* -- are observed rather
  than assumed.
* **HTTP level** -- the real ``ThreadingHTTPServer`` over loopback against a
  scripted supervisor, covering only what the table-driven token test in
  ``test_http_api.py`` cannot: the ``{arm_id}`` path segment and the motion
  body shapes.

This is the mutation-scrutiny file for enable/disable, the jog stream and the
activation-settling gate, and it now also carries the per-arm command source.
The fence comes from the CONFIGURATION -- ``fence.<arm>.enabled`` with degree
bounds -- because that is v2's only route to a sandbox tighter than the
factory limits, and the rig depends on it.

No robot, no ROS graph, no child process and no launch is executed anywhere
in this file: the spawner is a fake, and the only addresses that appear are
RFC 5737 documentation addresses, present so a motion profile can build argv.
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
from franka_web import defaults, health
from franka_web.gains import ProfileStore
from franka_web.http_api import App, build_server
from franka_web.lock import OperatorLock
from franka_web.logbus import LogBus
from franka_web.session import (
    expected_broadcasters, RECOVERY_REQUEST_TIMEOUT_S, SessionError, SessionRequest,
    SessionSupervisor, SWITCH_DWELL_S)
from franka_web.sse import Broker
import pytest
from sensor_msgs.msg import JointState
from support.config_factory import make_settings
from support.fake_checker import checker_holding, FakeCellModel
from support.fake_clock import FakeClock
from support.fake_launcher import (
    FakeBridge, FakeBroker, FakeChild, FakePreflightResult, FakeRecording, FakeSpawner)
from support.mock_impedance_controller import (
    ENABLE_DISABLED_MESSAGE, ENABLE_ENABLED_MESSAGE, NO_ERRORS_MESSAGE)

IMPEDANCE = defaults.MOTION_CONTROLLER

HARDWARE_NAME = 'FrankaMultiHardwareInterface'

STATIC_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')

#: A pose every joint of which sits well inside the rig's configured fence --
#: note joint4, whose factory bound is [-3.0718, -0.0698] and excludes 0.0.
IN_FENCE_POSE = (0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.5)

#: Margins for the tests that install a SANDBOX tighter than the factory
#: limits -- v2 reaches one only through ``fence.<arm>`` in the config file.
WIDE_MARGIN = 0.5
TIGHT_MARGIN = 0.4 * defaults.JOG_STEP_RAD
TIGHT_JOINT_INDEX = 3

#: Where the impedance controller captured its internal target at activation.
#: ``IN_FENCE_POSE`` is the same arm 0.03 rad later on joint2: the sag the live
#: Panda 2 showed between ACTIVE and readiness. The restoring PD error the
#: controller must keep is therefore exactly -0.03 rad on joint2.
ACTIVATION_POSE = (0.0, -0.33, 0.0, -1.5, 0.0, 1.2, 0.5)

#: The same pose with joint4 one third of a step below its upper fence, so a
#: single ``+`` press has to be clamped on the wire.
NEAR_UPPER_POSE = (0.0, -0.3, 0.0, -0.08, 0.0, 1.2, 0.5)

#: joint4 above its upper fence: legal for a JointState, refused at enable
#: time and by ``JogTargetModel.seed``.
OUT_OF_FENCE_POSE = (0.0, -0.3, 0.0, 0.4, 0.0, 1.2, 0.5)

#: Wall-clock bound on one queue-and-tick command exchange. Everything here is
#: in-process; anything slower than this is a hang, not slowness.
COMMAND_DEADLINE_S = 10.0

#: Every request in the HTTP half is loopback and answered in-process.
REQUEST_TIMEOUT_S = 10.0

# Synthetic unit-test settling numbers, in the SI the rig thinks in. They are
# roomy on purpose so unrelated motion-guard tests can drive the gate without
# fighting it; the config factory converts them to the degrees the file speaks.
TEST_SETTLING_RAD = {
    'drift_limit_rad': (0.2,) * 7,
    'span_limit_rad': (0.01,) * 7,
    'velocity_limit_rad_s': (0.05,) * 7,
    'fence_margin_rad': (0.01,) * 7,
    'stable_window_s': 0.2,
    'min_samples': 3,
    'timeout_s': 2.0,
}


def fence_around(pose=IN_FENCE_POSE, margins=None):
    """
    Return ``(lower, upper)`` fence arrays centred on ``pose``.

    ``margins`` defaults to :data:`WIDE_MARGIN` at every joint except
    :data:`TIGHT_JOINT_INDEX`, which gets :data:`TIGHT_MARGIN` so a single
    jog press has to be clamped there.
    """
    if margins is None:
        margins = [WIDE_MARGIN] * defaults.JOINT_COUNT
        margins[TIGHT_JOINT_INDEX] = TIGHT_MARGIN
    lower = tuple(float(value) - float(margin)
                  for value, margin in zip(pose, margins))
    upper = tuple(float(value) + float(margin)
                  for value, margin in zip(pose, margins))
    return _inside_policy(lower, upper)


def _inside_policy(lower, upper):
    """Clip a fence onto the factory policy the validator enforces."""
    lower = tuple(max(value, limit) for value, limit
                  in zip(lower, defaults.POLICY_POSITION_LOWER_RAD))
    upper = tuple(min(value, limit) for value, limit
                  in zip(upper, defaults.POLICY_POSITION_UPPER_RAD))
    return lower, upper


def uniform_fences(pose=IN_FENCE_POSE, margins=None):
    """Return the same :func:`fence_around` fence for both arms."""
    lower, upper = fence_around(pose, margins)
    return {'panda1': (lower, upper), 'panda2': (lower, upper)}


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


def diagnostic_at(arm_id, level=0, message=None):
    """Build the canonical per-arm DiagnosticStatus at the given level."""
    status = DiagnosticStatus()
    status.name = health.canonical_diagnostic_name(arm_id)
    status.hardware_id = arm_id
    status.level = bytes([level])
    status.message = message if message is not None else (
        'backend state is healthy' if level == 0
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


def rig_settings(tmp_path, settling=True, fences=None, **overrides):
    """
    Build Settings from a real configuration file over private tmp directories.

    The default fence is the Panda FACTORY policy, which is what "fence
    default-off" means mechanically. ``fences`` installs the per-arm sandbox
    v2 reaches through ``fence.<arm>.enabled`` -- the only route to bounds
    tighter than the factory limits, and the reason those optional keys are
    load-bearing for this suite. ``settling=False`` leaves the shipped
    settling defaults.
    """
    if settling:
        overrides.setdefault('settling_rad', dict(TEST_SETTLING_RAD))
    if fences is not None:
        overrides.setdefault('fences_rad', fences)
    return make_settings(tmp_path, **overrides)


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

    #: How many scripted publisher cycles one sequence (a start-path restage,
    #: or a recovery) may wait through. A stale stop needs one fresh
    #: state-only publication before motion re-activation and another for the
    #: post-activation verification.
    RECOVERY_WAIT_BUDGET = 2

    #: What ``FrankaMultiHardwareInterface`` reported on real hardware when a
    #: second controller mode switch arrived inside the reviewed spacing
    #: (2026-09-01/02, both live faces of V2L-5/6: a read-cycle error ~90 ms
    #: after a verified reactivation, and a write-cycle mode-switch error with
    #: a 205 ms / 206-missed-cycle control-loop stall).
    STALL_DIAGNOSTIC_MESSAGE = (
        'error in the control loop after a controller mode switch; the '
        'read/write cycle missed its deadline and the driver took its '
        'fail-safe stop')

    def __init__(self, clock):
        """Wire the fake to ``clock`` and start with nothing recorded."""
        super().__init__()
        self.clock = clock
        self.enable_calls = []
        self.published_targets = []
        self.recovery_calls = []
        self.switch_calls = []
        self.deactivate_calls = []
        self.hardware_active_calls = []
        self.on_enable = None
        self.on_recovery_success = None
        self.on_recovery_wait = None
        self.on_switch_activate = None
        self.on_finalize_capture = None
        self.recovery_waits_remaining = self.RECOVERY_WAIT_BUDGET
        self.recovery_wait_calls = []
        # -- the modelled driver's mode-switch spacing (V2L-5/6) -----------
        #: Fake-clock time of every switch_controller call this fake answered,
        #: activations and deactivations in one sequence -- the driver does
        #: not distinguish them, and neither does the spacing that protects
        #: it.
        self.switch_stamps = []
        #: One entry per call that arrived inside the reviewed spacing, i.e.
        #: per modelled fail-safe stop. Empty is the only healthy value.
        self.switch_stalls = []
        #: Turn the modelled failure off for a test that is about something
        #: else entirely. ON by default on purpose: a rig that could not
        #: reproduce the live driver is exactly what let V2L-5 ship.
        self.model_switch_stall = True

    def switch_spacings(self):
        """Return the fake-clock gaps between consecutive switch calls."""
        return [second - first for first, second
                in zip(self.switch_stamps, self.switch_stamps[1:])]

    def _model_switch_spacing(self, controllers):
        """
        Model the real driver's response to two mode switches in quick succession.

        A ``switch_controller`` call arriving before ``SWITCH_DWELL_S`` has
        passed since the previous one stalls the driver's real-time cycle. On
        the live cell that stall surfaced as an errored read cycle, an
        immediate fail-safe stop, and a session fault ~90 ms after a
        reactivation the server had already VERIFIED. Reproduced here as the
        two observable consequences the fault engine reads: the hardware
        component leaves ``active`` (rule F8) and every arm's canonical
        diagnostic goes to ERROR carrying the driver's own account (rule F1).

        The comparison is written exactly as the supervisor's dwell writes it
        -- ``now < previous + SWITCH_DWELL_S`` from the same stamp -- so the
        two can never disagree by one floating-point ulp about whether the
        spacing was kept.
        """
        now = self.clock.monotonic()
        previous = self.switch_stamps[-1] if self.switch_stamps else None
        self.switch_stamps.append(now)
        if not self.model_switch_stall or previous is None:
            return
        if not now < previous + SWITCH_DWELL_S:
            return
        self.switch_stalls.append({'controllers': tuple(controllers),
                                   'spacing_s': now - previous})
        if self.hardware is not None:
            self.hardware = dict(self.hardware, lifecycle_id=2,
                                 lifecycle_label='inactive')
        stamp = self.clock.monotonic_ns()
        for arm_id in list(self.diagnostics):
            self.diagnostics[arm_id] = (
                stamp, diagnostic_at(arm_id, 2, self.STALL_DIAGNOSTIC_MESSAGE))

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
        """Record the activation, model ``onActivate()``, then run the hook."""
        self.switch_calls.append(list(controllers))
        self._model_switch_spacing(controllers)
        response = super().call_switch_activate(controllers, timeout_s)
        if response is not None and response['ok']:
            self.model_activation(tuple(controllers))
            if self.on_switch_activate is not None:
                self.on_switch_activate(tuple(controllers))
        return response

    def model_activation(self, controllers):
        """Model what the reviewed controller DOES on activation; a no-op here."""

    def call_switch_deactivate(self, controllers, timeout_s=5.0):
        """Record fail-closed controller deactivation and answer as scripted."""
        self.deactivate_calls.append(list(controllers))
        self._model_switch_spacing(controllers)
        return super().call_switch_deactivate(controllers, timeout_s)

    def finalize_activation_capture(self, observer):
        """Expose the exact post-observer, pre-disarm boundary to a test hook."""
        def observed(capture):
            verdict = observer(capture)
            if self.on_finalize_capture is not None:
                self.on_finalize_capture(verdict)
            return verdict

        return super().finalize_activation_capture(observed)

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

    def wait_for_recovery_samples(self, timeout_s):
        """Advance one scripted publisher cycle, then report no further updates."""
        self.recovery_wait_calls.append(timeout_s)
        if self.recovery_waits_remaining <= 0:
            return False
        self.recovery_waits_remaining -= 1
        callback = self.on_recovery_wait
        self.on_recovery_wait = None
        if callback is not None:
            callback()
            return True
        self.clock.advance(0.001)
        stamp = self.clock.monotonic_ns()
        if self.joint is not None:
            self.set_joint_sample(stamp, self.joint[1])
        self.robot_states = {
            arm_id: (stamp, sample[1])
            for arm_id, sample in self.robot_states.items()
        }
        self.diagnostics = {
            arm_id: (stamp, sample[1])
            for arm_id, sample in self.diagnostics.items()
        }
        return True


class ImpedanceInboxModel:
    """
    A behavioural port of one reviewed ``ArmImpedanceTargetInbox`` slot.

    Only the enable-generation/target-rebase contract is modelled, because
    that is the contract the web supervisor can violate.  Faithful to
    ``dual_arm_joint_impedance_controller.cpp``:

    * ``ArmImpedanceTargetInbox::setEnabled()`` advances ``enable_generation_``
      on EVERY call -- an odd value while the callback runs, even again when
      it settles -- including a redundant false-to-false request.
    * ``onActivate()`` calls ``disableAndInvalidateAll()``, and the owner
      thread's first ``update()`` runs ``captureActivationState()``, which
      assigns measured ``q`` to ``internal_target`` and synchronises
      ``observed_enable_generation``.
    * Every later ``update()`` whose observed generation differs treats it as
      a transition and assigns ``next_targets = positions`` -- the rebase that
      removed Panda 2's restoring J2 torque in ``web-20260831-152313``.
    """

    def __init__(self):
        """Start detached: not activated, nothing captured, nothing enabled."""
        self.enabled = False
        self.enable_generation = 0
        self.observed_enable_generation = 0
        self.internal_target = None
        self.measured = None
        self.rebases = 0
        self.rebases_at_activation = 0

    def set_enabled(self, enabled):
        """Port ``setEnabled``: always advance the generation, false-to-false too."""
        self.enable_generation += 2
        self.enabled = bool(enabled)

    def on_activate(self, measured):
        """Port ``onActivate()`` plus the owner thread's first update cycle."""
        self.set_enabled(False)
        self.measured = tuple(float(value) for value in measured)
        self.internal_target = self.measured
        self.observed_enable_generation = self.enable_generation
        self.rebases_at_activation = self.rebases

    def settle_to(self, measured):
        """Move the MEASURED pose only: the arm creeping under gravity."""
        self.measured = tuple(float(value) for value in measured)

    def update(self):
        """Apply one RT cycle's transition/rebase decision to this slot."""
        if self.enable_generation != self.observed_enable_generation:
            self.internal_target = self.measured
            self.observed_enable_generation = self.enable_generation
            self.rebases += 1

    @property
    def rebases_since_activation(self):
        """Return how many rebases happened after the last activation."""
        return self.rebases - self.rebases_at_activation

    @property
    def spring_error(self):
        """Return ``internal_target - measured``: the restoring PD error."""
        return tuple(target - measured for target, measured
                     in zip(self.internal_target, self.measured))


class RebaseModelBridge(MotionBridge):
    """
    ``MotionBridge`` whose enable service drives real controller semantics.

    Counting ``call_enable`` proves only what the supervisor SAID.  These
    slots additionally model what the reviewed controller would DO with it,
    so a redundant false-to-false request is caught by the target it destroys
    rather than by an assertion about call volume.
    """

    def __init__(self, clock):
        """Wire the fake with no activated slots yet."""
        super().__init__(clock)
        self.inboxes = {}
        self.control_cycles = 0

    def activate_impedance(self, slots, measured):
        """Model the controller reaching ACTIVE and capturing ``measured``."""
        for slot in slots:
            self.inboxes.setdefault(slot, ImpedanceInboxModel()).on_activate(
                measured)

    def settle_to(self, measured):
        """Move every slot's measured pose without touching its target."""
        for inbox in self.inboxes.values():
            inbox.settle_to(measured)

    def run_control_cycle(self, cycles=1):
        """Run ``cycles`` RT update cycles over every activated slot."""
        for _ in range(cycles):
            self.control_cycles += 1
            for inbox in self.inboxes.values():
                inbox.update()

    def call_enable(self, slot, enabled, timeout_s=5.0):
        """Answer as scripted and apply ``setEnabled`` to the modelled slot."""
        response = super().call_enable(slot, enabled, timeout_s)
        inbox = self.inboxes.get(slot)
        if inbox is not None and response is not None and response['success']:
            inbox.set_enabled(enabled)
        return response

    def model_activation(self, controllers):
        """
        Model ``onActivate()`` for every slot when the impedance controller starts.

        The reviewed controller disables and invalidates every inbox and its
        first ``update()`` captures the CURRENTLY measured pose as the
        internal target. The server's restage activation is a real activation,
        so the model must cover the START path too -- not only Recover.
        """
        if IMPEDANCE not in controllers or self.joint is None:
            return
        arm_ids = self.configured[0] if self.configured else ()
        for slot, arm_id in enumerate(arm_ids, start=1):
            joints = health.extract_joints(arm_id, self.joint[1])
            if not joints['complete']:
                continue
            self.inboxes.setdefault(slot, ImpedanceInboxModel()).on_activate(
                joints['positions'])


class MotionHarness:
    """
    One fully faked supervisor, wired for motion sessions.

    Real: the supervisor, the profile store (over a private tmp state dir),
    the operator lock, the jog models and the fault engine. Fake: the clock,
    the bridge, the spawner, the recorder and the preflight verdict.
    """

    def __init__(self, tmp_path, settling=True, bridge_factory=None,
                 fences=None, checker=None, **settings_overrides):
        """Wire a stopped supervisor whose every collaborator is inspectable."""
        self.clock = FakeClock()
        self.settings = rig_settings(
            tmp_path, settling=settling, fences=fences, **settings_overrides)
        self.bridge = (MotionBridge if bridge_factory is None
                       else bridge_factory)(self.clock)
        self.broker = FakeBroker()
        self.spawner = FakeSpawner()
        self.recorder = FakeRecording()
        self.preflight = FakePreflightResult()
        self.lock = OperatorLock(monotonic=self.clock.monotonic)
        self.logs = LogBus()
        self.profiles = ProfileStore(self.settings.state_dir)
        self.launch_child = FakeChild(name='launch')
        self.spawner.queue_child(self.launch_child)
        self.token = None
        self.supervisor = SessionSupervisor(
            self.settings, self.bridge, self.lock, self.broker,
            spawn=self.spawner,
            recording_factory=lambda: self.recorder,
            preflight_runner=lambda settings, mode: self.preflight,
            profile_store=self.profiles,
            log_bus=self.logs,
            checker=checker,
            monotonic=self.clock.monotonic,
            recovery_wait=self.bridge.wait_for_recovery_samples,
        )
        #: Every dwell slice the supervisor spent, in fake seconds.
        self.dwell_waits = []
        # Assigned, not passed: `_switch_dwell_wait` is deliberately NOT a
        # constructor keyword, so this same rig can be pointed at a build
        # WITHOUT the dwell (where the attribute is simply unused) and produce
        # the fail-before half of the V2L-5 regression pair.
        self.supervisor._switch_dwell_wait = self.wait_switch_dwell

    def wait_switch_dwell(self, timeout_s):
        """
        Spend one switch-spacing slice of fake time, publishers still running.

        The graph does not stop while the server waits out the spacing, so
        this re-stamps the live publications the way a real 0.75 s of wall
        clock would. It is a SEPARATE seam from ``wait_for_recovery_samples``
        and spends none of that scripted budget: "let the real-time loop
        settle" and "let the publishers produce another cycle" are different
        questions, and a dwell that ate the sample budget would silently
        change what every recovery test measures.
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

    def start(self, arms, mode):
        """
        Submit a start request the way an HTTP worker does.

        Motion is driven the way the REAL launch sequences it. The reviewed
        guarded-motion launch runs ``spawner --switch-asap``, so the impedance
        controller is already ACTIVE before ``joint_state_broadcaster``
        publishes anything (live evidence 2026-09-01: controller active at
        t+5.96 s, joint states at t+6.38 s). The rig therefore leaves the
        controller active across the whole start and lets the server's own
        restage create the torque-free window it measures in. Nothing is
        withdrawn from ``bridge.controllers`` here: a rig that hid the
        controller until a baseline existed is exactly what let this bug ship.
        """
        result = self.drive(lambda: self.supervisor.request_start(
            SessionRequest(arms=arms, mode=mode),
            operator_lease=self.operator_lease()))
        if mode == 'motion':
            for _ in range(6):
                if self.supervisor.state != 'starting':
                    break
                self.tick()
        return result

    def reset_call_log(self):
        """
        Forget every service call recorded so far, keeping the live state.

        The start path now drives ``switch_controller`` itself, so a recovery
        test asserting an exact call list needs a boundary between "what the
        start did" and "what THIS recovery did". Clearing the recorders keeps
        those assertions verbatim and keeps their meaning.

        The scripted publisher budget is restored with them, and for the same
        reason: it is "how many publication cycles the next sequence gets",
        and the start-path restage now spends one of them waiting for a
        sample newer than its own deactivation.
        """
        self.bridge.enable_calls = []
        self.bridge.switch_calls = []
        self.bridge.deactivate_calls = []
        self.bridge.hardware_active_calls = []
        self.bridge.recovery_calls = []
        self.bridge.recovery_wait_calls = []
        self.bridge.recovery_waits_remaining = MotionBridge.RECOVERY_WAIT_BUDGET

    def source(self, arm_id, source):
        """Submit a command-source switch and return its verdict."""
        return self.drive(lambda: self.supervisor.request_arm_source(
            arm_id, source, operator_lease=self.operator_lease()))

    def enable(self, arm_id, enabled=True):
        """Submit a §6.13 enable command and return its verdict."""
        return self.drive(lambda: self.supervisor.request_arm_enable(
            arm_id, enabled, operator_lease=self.operator_lease()))

    def jog(self, arm_id, joint_index, direction):
        """Submit a §6.13 jog command and return its verdict."""
        return self.drive(
            lambda: self.supervisor.request_arm_jog(
                arm_id, joint_index, direction,
                operator_lease=self.operator_lease()))

    def apply(self, arm_id, positions):
        """Submit one Apply start and return its verdict."""
        return self.drive(lambda: self.supervisor.request_arm_apply(
            arm_id, 'start', list(positions),
            operator_lease=self.operator_lease()))

    def recover(self, settle=True):
        """Submit a §6.13 session recovery and return its verdict."""
        result = self.drive(lambda: self.supervisor.request_session_recover(
            operator_lease=self.operator_lease()))
        if settle and self.supervisor.state == 'settling':
            self.drive_settling()
        return result

    def force_state(self, state):
        """
        Pin the state machine, the way ``test_session_state_machine`` does.

        Used only to hold the machine in a state a tick would otherwise leave
        (``preflight``/``starting``), so a guard can be asked about it.
        """
        with self.supervisor._state_lock:
            self.supervisor._state = state

    # -- scripting the world -------------------------------------------

    def set_joints(self, message=None):
        """Publish a joint sample stamped at the current fake time."""
        self.bridge.set_joint_sample(
            self.clock.monotonic_ns(),
            dual_joint_state() if message is None else message)

    def refresh_joints(self):
        """Re-stamp the current joint sample at the current fake time."""
        if self.bridge.joint is not None:
            self.bridge.set_joint_sample(
                self.clock.monotonic_ns(), self.bridge.joint[1])

    def drive_settling(self, ticks=4):
        """Publish distinct stable samples until an impedance gate is ready."""
        for _ in range(ticks):
            if self.supervisor.state == 'running':
                return
            self.clock.advance(0.1)
            self.set_joints()
            stamp = self.clock.monotonic_ns()
            self.bridge.robot_states = {
                arm_id: (stamp, sample[1])
                for arm_id, sample in self.bridge.robot_states.items()
            }
            self.bridge.diagnostics = {
                arm_id: (stamp, sample[1])
                for arm_id, sample in self.bridge.diagnostics.items()
            }
            self.tick()
        assert self.supervisor.state == 'running', self.supervisor.frame()['session']

    def make_ready(self, arm_ids=('panda1', 'panda2'), arm_mode='dual',
                   controller_name=None):
        """Satisfy every §3.5 readiness criterion and every §7.1 health rule."""
        controllers = {name: 'active'
                       for name in expected_broadcasters(arm_ids, arm_mode)}
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

    def claim_lock(self):
        """Take the operator lock (the jog stream publishes only while it is held)."""
        if self.token is not None and self.lock.validate(self.token):
            return self.token
        claim = self.lock.claim()
        assert claim is not None
        self.token = claim.token
        return self.token

    def operator_lease(self):
        """Return the exact live authorization an HTTP request would carry."""
        lease = self.lock.authorize(self.claim_lock())
        assert lease is not None
        return lease

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


def motion_running(tmp_path, arms='both', bridge_factory=None, **kwargs):
    """Build a harness whose motion session has reached ``running``."""
    harness = MotionHarness(tmp_path, bridge_factory=bridge_factory, **kwargs)
    arm_ids = ('panda1', 'panda2') if arms == 'both' else (arms,)
    harness.make_ready(arm_ids, 'dual' if arms == 'both' else 'single',
                       IMPEDANCE)
    harness.start(arms=arms, mode='motion')
    harness.pump()
    harness.drive_settling()
    # Nothing to clear: a fresh impedance startup must reach Running having
    # sent ZERO controller-side SetBool calls, because onActivate() already
    # disabled every inbox and any further false call would advance
    # enable_generation and rebase the captured target. Asserting it here
    # makes every caller of this fixture a standing guard.
    assert harness.bridge.enable_calls == [], (
        'fresh motion startup sent controller-side enable traffic: '
        '{!r}'.format(harness.bridge.enable_calls))
    assert harness.supervisor.state == 'running', (
        harness.supervisor.frame()['session'])
    # The start path drives switch_controller itself now (the restage), so
    # every recovery assertion downstream is scoped to THIS recovery.
    harness.reset_call_log()
    return harness


def simple_running(tmp_path, mode, arms='both'):
    """Build a harness whose non-motion session (simulate/watch) is ``running``."""
    harness = MotionHarness(tmp_path)
    arm_ids = ('panda1', 'panda2') if arms == 'both' else (arms,)
    harness.make_ready(arm_ids, 'dual' if arms == 'both' else 'single')
    harness.start(arms=arms, mode=mode)
    harness.pump()
    assert harness.supervisor.state == 'running'
    return harness


def submit_async(call):
    """Run one blocking supervisor request and expose its eventual outcome."""
    outcome = {}

    def submit():
        try:
            outcome['value'] = call()
        except Exception as error:
            outcome['error'] = error

    thread = threading.Thread(target=submit, daemon=True)
    thread.start()
    return outcome, thread


def wait_for_queued_command(harness):
    """Wait until an asynchronous request reaches the supervisor queue."""
    deadline = time.monotonic() + COMMAND_DEADLINE_S
    while harness.supervisor._commands.empty() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert harness.supervisor._commands.empty() is False


class TestRealActivationOrder:
    """
    The live blocker of 2026-09-01, reproduced offline.

    The reviewed ``production_*_guarded_motion`` launch activates the
    impedance controller with ``spawner --switch-asap`` BEFORE
    ``joint_state_broadcaster`` is even loaded, so no pre-activation window
    exists for the server to observe. Every real Motion start was refused with
    ``activation_settling_limit``. The rigs used to model the inverse order,
    which is why 2000 green tests never saw it; they now model the real one,
    and the server makes its own torque-free window instead.
    """

    def test_a_motion_start_survives_the_launch_activating_the_controller_first(
            self, tmp_path):
        """
        A start whose controller is already active still reaches Running.

        The session must pause the impedance controller, measure the resting
        pose with nothing commanding the arms, hand them straight back and
        gate THAT activation.
        """
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=IMPEDANCE)
        # The real order: active before the first joint sample is even seen.
        assert harness.bridge.controllers[IMPEDANCE] == 'active'

        harness.start(arms='both', mode='motion')

        assert harness.supervisor.state == 'settling', (
            harness.supervisor.frame()['session'])
        frame = harness.supervisor.frame()
        assert frame['session']['last_error'] is None
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]
        assert harness.bridge.switch_calls == [[IMPEDANCE]]
        assert harness.bridge.enable_calls == []      # zero SetBool on start
        ids = [entry['id'] for entry in frame['session']['steps']]
        assert ids == ['preflight', 'connect:panda1', 'connect:panda2',
                       'health', 'stack_ready', 'controller_pause', 'baseline',
                       'controller', 'settling']
        assert harness.supervisor._activation_baseline['panda1'] == IN_FENCE_POSE
        harness.drive_settling()
        assert harness.supervisor.state == 'running'

    @staticmethod
    def restaged(tmp_path, **kwargs):
        """Build a ready impedance rig, controller active, nothing started."""
        harness = MotionHarness(tmp_path, **kwargs)
        harness.make_ready(controller_name=IMPEDANCE)
        return harness

    @staticmethod
    def assert_failed_closed(harness, step_id, detail_fragment):
        """
        Assert the named step failed, the session stopped and torque is gone.

        The checklist is cleared on the way into `stopped`, so the evidence is
        read from the frame the transition PUBLISHED, the way the console saw
        it, not from the frame after teardown.
        """
        frame = harness.supervisor.frame()
        assert harness.supervisor.state in ('stopping', 'stopped'), frame['session']
        last_error = frame['session']['last_error']
        assert last_error['code'] == 'activation_settling_limit'
        assert detail_fragment in last_error['detail'], last_error['detail']
        published = [payload for event, payload in harness.broker.events
                     if event == 'state'
                     and any(entry['status'] == 'failed'
                             for entry in payload['session']['steps'])]
        assert published, 'no published frame carried a failed step'
        failed = [entry for entry in published[-1]['session']['steps']
                  if entry['status'] == 'failed']
        assert [entry['id'] for entry in failed] == [step_id], (
            published[-1]['session']['steps'])
        assert harness.bridge.enable_calls == []
        assert harness.bridge.published_targets == []
        assert harness.supervisor._activation_gate is None
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_a_refused_deactivate_stops_the_session_before_any_baseline(
            self, tmp_path):
        """A controller that will not pause leaves no window to measure in."""
        harness = self.restaged(tmp_path)
        harness.bridge.deactivate_response = {'ok': False}

        harness.start(arms='both', mode='motion')

        self.assert_failed_closed(
            harness, 'controller_pause',
            'the impedance controller could not be paused')
        assert harness.supervisor._baseline_captured is False

    def test_an_unverified_deactivate_stops_even_when_the_service_says_ok(
            self, tmp_path):
        """The verdict is the observed lifecycle, never the service's word."""
        harness = self.restaged(tmp_path)
        # The service answers ok; controller-manager still reports active.
        harness.bridge.controller_query_response = {IMPEDANCE: 'active'}

        harness.start(arms='both', mode='motion')

        self.assert_failed_closed(
            harness, 'controller_pause',
            'the impedance controller could not be paused')
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]

    def test_a_refused_reactivation_stops_and_says_the_arms_are_uncommanded(
            self, tmp_path):
        """The operator must be told the arms were left with nothing holding them."""
        harness = self.restaged(tmp_path)
        harness.bridge.switch_response = {'ok': False}

        harness.start(arms='both', mode='motion')

        self.assert_failed_closed(
            harness, 'controller',
            'did not come back active after the baseline was measured')
        assert 'left with no controller holding them' in (
            harness.supervisor.frame()['session']['last_error']['detail'])
        # The bounded capture is retired on the way out, never left armed.
        assert harness.bridge._activation_capture is None

    def test_no_fresh_sample_during_the_pause_fails_closed(self, tmp_path):
        """A publisher that stops while the arms are paused is not guessed at."""
        harness = self.restaged(tmp_path)
        harness.bridge.recovery_waits_remaining = 0

        harness.start(arms='both', mode='motion')

        self.assert_failed_closed(
            harness, 'baseline',
            'no fresh joint sample was available before motion re-activation')
        assert harness.supervisor._baseline_captured is False

    def test_a_fault_firing_at_restage_entry_never_releases_torque(self, tmp_path):
        """A fault already firing is not answered by pausing the controller."""
        harness = self.restaged(tmp_path)
        harness.bridge.enable_ready = False       # hold short of readiness
        harness.start(arms='both', mode='motion')
        assert harness.supervisor.state == 'starting'

        harness.set_diagnostic('panda1', 2)       # F1 fires
        harness.bridge.enable_ready = True
        harness.pump()

        assert harness.supervisor.state == 'fault'
        assert 'diagnostic_error' in harness.fault_codes()
        assert harness.bridge.deactivate_calls == []
        assert harness.bridge.enable_calls == []
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_the_restage_sends_no_enable_call_and_rebases_no_target(
            self, tmp_path):
        """The modelled controller keeps the target its onActivate() captured."""
        harness = self.restaged(tmp_path, bridge_factory=RebaseModelBridge)
        harness.start(arms='both', mode='motion')
        harness.drive_settling()
        harness.bridge.run_control_cycle(5)

        assert harness.supervisor.state == 'running'
        assert harness.bridge.enable_calls == []
        for slot in (1, 2):
            inbox = harness.bridge.inboxes[slot]
            assert inbox.rebases_since_activation == 0
            assert inbox.enable_generation == inbox.observed_enable_generation
            assert inbox.internal_target == IN_FENCE_POSE


class TestSwitchDwell:
    """
    V2L-5/6, the live blocker of 2026-09-01/02, reproduced offline.

    Controller mode switches issued in tight succession stall the driver's
    real-time cycle. The Motion start restage put its pause and its hand-back
    ~110 ms apart; ~90 ms after a reactivation the server had already VERIFIED,
    ``FrankaMultiHardwareInterface``'s read cycle errored, the driver took its
    fail-safe stop and the session faulted -- twice, on two consecutive starts.
    The same class bit Recover's own sequence from the write side, with a
    205 ms / 206-missed-cycle control-loop stall, and degraded into a
    "communication with the robot failed" 16 s later. Recover's historic ~1 s
    spacing has always survived; the restage's 110 ms did not.

    ``MotionBridge`` now MODELS that driver: any ``switch_controller`` call
    arriving inside ``SWITCH_DWELL_S`` of the previous one takes the fail-safe
    stop, which the fault engine reads as F8 plus F1. The fix is one bounded
    spacing discipline around every switch call the server makes, restage and
    Recover alike.
    """

    @staticmethod
    def settle(harness, ticks=6):
        """Publish stable samples for up to ``ticks``, asserting nothing."""
        for _ in range(ticks):
            if harness.supervisor.state in ('running', 'fault', 'stopped'):
                return
            harness.clock.advance(0.1)
            harness.set_joints()
            stamp = harness.clock.monotonic_ns()
            harness.bridge.robot_states = {
                arm_id: (stamp, sample[1])
                for arm_id, sample in harness.bridge.robot_states.items()}
            harness.bridge.diagnostics = {
                arm_id: (stamp, sample[1])
                for arm_id, sample in harness.bridge.diagnostics.items()}
            harness.tick()

    def test_a_motion_start_survives_a_driver_that_stalls_on_tight_switches(
            self, tmp_path):
        """
        PASS-AFTER: the restage spaces its two switches and reaches Running.

        Run this same test against a build without the dwell and it fails the
        way the live cell did: the modelled driver records a stall, the
        hardware component leaves ``active`` and the session ends in ``fault``.
        """
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=IMPEDANCE)
        assert harness.bridge.model_switch_stall is True, (
            'this test is meaningless against a fake that cannot stall')

        harness.start(arms='both', mode='motion')

        assert harness.bridge.switch_stalls == [], (
            'the modelled driver took its fail-safe stop: {!r}'.format(
                harness.bridge.switch_stalls))
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]
        assert harness.bridge.switch_calls == [[IMPEDANCE]]
        assert harness.supervisor.state == 'settling', (
            harness.supervisor.frame()['session'])
        self.settle(harness)
        assert harness.supervisor.state == 'running', (
            'state={} faults={} stalls={}'.format(
                harness.supervisor.state, harness.fault_codes(),
                harness.bridge.switch_stalls))
        assert harness.fault_codes() == []

    def test_the_restage_spends_the_reviewed_spacing_between_its_two_switches(
            self, tmp_path):
        """The spacing is real waiting, measured at the fake driver."""
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=IMPEDANCE)
        harness.start(arms='both', mode='motion')
        spacings = harness.bridge.switch_spacings()
        assert len(spacings) == 1, harness.bridge.switch_stamps
        assert spacings[0] >= SWITCH_DWELL_S, spacings
        # The spacing is measured from the deactivation, and the baseline's
        # own fresh-sample wait already spent part of it, so the dwell waits
        # for the REMAINDER -- never for nothing, and never for more than the
        # constant.
        assert harness.dwell_waits, 'the dwell never waited at all'
        assert 0.0 < sum(harness.dwell_waits) <= SWITCH_DWELL_S, (
            harness.dwell_waits)

    def test_the_fake_driver_reproduces_the_live_fault_without_the_spacing(
            self, tmp_path):
        """
        FAIL-BEFORE, pinned in the suite: no spacing reproduces tonight exactly.

        A dwell wait that refuses to wait is the unpatched restage: the two
        switches land at the same instant, the modelled driver errors its read
        cycle into a fail-safe stop, and the fault arrives on the tick after
        the verified reactivation -- F8 with F1 alongside, carrying the
        driver's own account.
        """
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=IMPEDANCE)
        harness.supervisor._switch_dwell_wait = lambda timeout_s: False

        harness.start(arms='both', mode='motion')

        assert len(harness.bridge.switch_stalls) == 1, harness.bridge.switch_stalls
        stall = harness.bridge.switch_stalls[0]
        assert stall['controllers'] == (IMPEDANCE,)
        # Tonight's number was ~0.110 s; with no spacing at all only the
        # baseline's own sample wait separates the two switches.
        assert stall['spacing_s'] < SWITCH_DWELL_S, stall
        # The restage itself SUCCEEDED -- the reactivation was verified -- and
        # only then did the driver fall over. That is the live timeline.
        assert harness.supervisor.state == 'settling', (
            harness.supervisor.frame()['session'])
        harness.pump(1)
        assert harness.supervisor.state == 'fault'
        codes = harness.fault_codes()
        assert 'hardware_inactive' in codes, codes
        assert 'diagnostic_error' in codes, codes
        message = harness.supervisor.frame()['fault']['reasons']
        assert any(MotionBridge.STALL_DIAGNOSTIC_MESSAGE in reason['detail']
                   for reason in message), message

    def test_recovery_spaces_every_switch_it_issues(self, tmp_path):
        """
        Recover gets the SAME discipline, through the same helper.

        A dual recovery issues seven switch calls -- one pre-deactivation,
        five broadcasters, the motion controller -- and the driver cares about
        every gap between them, not only the impedance one.
        """
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        # Make the recovery walk its whole activation ladder, not only the
        # impedance controller: the driver cares about every gap.
        restore = ('franka_panda1_robot_model_broadcaster',
                   'franka_panda2_robot_model_broadcaster')
        for name in restore:
            harness.bridge.controllers[name] = 'inactive'
        harness.bridge.switch_stamps = []
        harness.bridge.switch_stalls = []

        harness.recover()

        assert harness.supervisor.state == 'running'
        # One pre-deactivation, the two broadcasters, the motion controller.
        assert len(harness.bridge.switch_stamps) == 2 + len(restore), (
            harness.bridge.switch_stamps)
        assert harness.bridge.switch_stalls == [], harness.bridge.switch_stalls
        assert all(spacing >= SWITCH_DWELL_S
                   for spacing in harness.bridge.switch_spacings()), (
            harness.bridge.switch_spacings())

    def test_the_rollback_deactivation_is_spaced_too(self, tmp_path):
        """Even the fail-closed rollback path goes through the spaced helper."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        # No post-restore publication: verify_restore fails and the rollback
        # deactivates the controller it had just brought back.
        harness.bridge.recovery_waits_remaining = 1
        harness.bridge.switch_stamps = []
        harness.bridge.switch_stalls = []
        with pytest.raises(SessionError):
            harness.recover()
        assert harness.bridge.deactivate_calls[-1] == [IMPEDANCE]
        assert harness.bridge.switch_stalls == [], harness.bridge.switch_stalls

    def test_the_dwell_is_bounded_when_the_wait_never_advances_the_clock(
            self, tmp_path):
        """A wait callable that does nothing cannot hold the supervisor thread."""
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=IMPEDANCE)
        calls = []

        def stuck_wait(timeout_s):
            calls.append(timeout_s)
            return True                      # "waited", but time did not move

        harness.supervisor._switch_dwell_wait = stuck_wait
        harness.supervisor._last_switch_mono = harness.clock.monotonic()
        assert harness.supervisor._await_switch_dwell() == 0.0
        assert len(calls) == 64, len(calls)


class TestActivationSettlingIntegration:
    """The supervisor keeps every command surface closed around torque activation."""

    @staticmethod
    def prepared(tmp_path, **kwargs):
        """Build a ready impedance rig without starting Motion."""
        harness = MotionHarness(tmp_path, **kwargs)
        harness.make_ready(controller_name=IMPEDANCE)
        return harness

    def test_a_policy_always_exists_now_so_motion_needs_no_extra_gate(
            self, tmp_path):
        """
        There is no `settling_policy_required` refusal any more.

        The configuration always yields a settling policy, so a Motion start
        cannot be blocked for the lack of one; the gate itself is unchanged.
        """
        harness = self.prepared(tmp_path, settling=False)
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state in ('settling', 'running')
        assert harness.supervisor._session['settling_policy'] is not None

    def test_an_insufficient_envelope_margin_faults_at_the_baseline_step(
            self, tmp_path):
        """
        An in-fence pose still needs room for the whole activation envelope.

        The check moved INSIDE the session: it now happens at the `baseline`
        step in `starting`, before the impedance controller can take hold,
        and it faults rather than refusing the request.
        """
        harness = self.prepared(tmp_path)
        harness.set_joints(dual_joint_state(
            pose_1=NEAR_UPPER_POSE, pose_2=NEAR_UPPER_POSE))
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state == 'fault'
        frame = harness.supervisor.frame()
        baseline = [entry for entry in frame['session']['steps']
                    if entry['id'] == 'baseline'][0]
        assert baseline['status'] == 'failed'
        assert 'J4' in baseline['detail']
        assert 'settling.fence_margin_deg' in baseline['detail']
        assert 'settling.drift_limit_deg' in baseline['detail']

    def test_settling_rejects_commands_then_publishes_ready_evidence(self, tmp_path):
        """Distinct stable dual samples alone open Running and the jog surface."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state == 'settling'
        frame = harness.supervisor.frame()
        activation = frame['session']['activation']
        assert activation['required'] is True
        assert activation['status'] == 'settling'
        assert activation['torque_control_active'] is True
        assert activation['policy_sha256'] == \
            harness.settings.settling.policy().sha256
        for arm in frame['arms'].values():
            assert arm['motion']['available'] is False
        calls_before = list(harness.bridge.enable_calls)
        with pytest.raises(SessionError) as enable_error:
            harness.enable('panda1')
        with pytest.raises(SessionError) as jog_error:
            harness.jog('panda1', 6, 1)
        assert enable_error.value.code == 'session_not_running'
        assert jog_error.value.code == 'session_not_running'
        assert harness.bridge.enable_calls == calls_before
        assert harness.bridge.published_targets == []

        harness.drive_settling()
        ready = harness.supervisor.frame()
        assert ready['session']['activation']['status'] == 'ready'
        assert ready['session']['activation']['samples'] >= 3
        for arm_id in ('panda1', 'panda2'):
            motion = ready['arms'][arm_id]['motion']
            assert motion['available'] is True
            assert motion['activation_delta_rad'] == pytest.approx([0.0] * 7)
            assert motion['activation_max_abs_delta_rad'] == pytest.approx([0.0] * 7)
            assert motion['activation_abs_velocity_rad_s'] == pytest.approx([0.0] * 7)

    @pytest.mark.parametrize('offset', ['stale', 'future'])
    def test_the_barrier_refuses_to_arm_without_a_fresh_sample(
            self, tmp_path, offset):
        """
        The activation barrier is placed after a sample it can trust.

        `_readiness_met` requires a joint sample to EXIST but never checks
        its age, so a stale (or future-dated) sample can carry a Motion
        session out of `starting`. Weakening this precondition to a plain
        "is there a sample" lets the session enter `settling` and be refused
        only after the whole timeout elapses, instead of refusing here with
        the barrier's own message.

        The restage guarantees a fresh sample for the BASELINE, so the window
        this guards is the one between the re-activation and the barrier: the
        sample is spoiled from inside the lifecycle call itself.
        """
        harness = self.prepared(tmp_path)

        def spoil_the_sample(controllers):
            _stamp, message = harness.bridge.joint
            if offset == 'stale':
                stamp = harness.clock.monotonic_ns() - int(
                    (defaults.ENABLE_JOINT_STATE_MAX_AGE_S + 0.3) * 1e9)
            else:
                stamp = harness.clock.monotonic_ns() + int(1e9)
            harness.bridge.set_joint_sample(stamp, message)

        harness.bridge.on_switch_activate = spoil_the_sample
        harness.start(arms='both', mode='motion')

        assert harness.supervisor.state in ('stopping', 'stopped')
        frame = harness.supervisor.frame()
        assert frame['session']['last_error']['code'] == \
            'activation_settling_limit'
        assert ('no fresh joint sample was available at the activation barrier'
                in frame['session']['last_error']['detail'])
        assert harness.supervisor._activation_gate is None
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    @pytest.mark.parametrize('surface', ['enable', 'target'])
    def test_an_enable_or_target_during_settling_stops_the_session(
            self, tmp_path, surface):
        """
        The command surface is closed during settling, and says so if it opens.

        Neither surface is reachable from outside today (`_jog_tick` returns
        unless the state is `running`, and an enable is refused with
        `session_not_running`), which is exactly why a regression in this
        backstop would go unnoticed. It is the last thing standing between an
        operator command and the window where torque has just been armed.
        """
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state == 'settling'

        supervisor = harness.supervisor
        with supervisor._state_lock:
            if surface == 'enable':
                supervisor._arm_enabled['panda1'] = True
            else:
                barrier = supervisor._settling_target_counts.get('panda1', 0)
                supervisor._targets_published['panda1'] = barrier + 1

        harness.tick()

        assert harness.supervisor.state in ('stopping', 'stopped')
        frame = harness.supervisor.frame()
        assert frame['session']['last_error']['code'] == \
            'activation_settling_limit'
        assert frame['session']['last_error']['detail'].startswith(
            'an enable or target appeared while activation settling was '
            'closed.')
        messages = [line['message']
                    for line in harness.logs.window()['lines']]
        assert any(message.startswith(
            'an enable or target appeared while activation settling was '
            'closed.') for message in messages)
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.bridge.published_targets == []

    def test_running_commits_inside_final_capture_boundary(self, tmp_path):
        """The supervisor commits Running before the bridge disarms capture."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        observations = []

        def inspect_boundary(verdict):
            observations.append((
                verdict.status,
                harness.supervisor.state,
                harness.bridge._activation_capture is not None))

        harness.bridge.on_finalize_capture = inspect_boundary
        harness.drive_settling()
        assert observations == [('ready', 'running', True)]
        assert harness.bridge._activation_capture is None

    def test_post_finalizer_fault_is_latched_before_running_is_published(
            self, tmp_path):
        """An independent health callback at finalization cannot expose Enable."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()

        def inject_fault(verdict):
            if verdict.status == 'ready':
                harness.set_diagnostic('panda1', 2)

        harness.bridge.on_finalize_capture = inject_fault
        for _ in range(4):
            harness.clock.advance(0.1)
            harness.set_joints()
            stamp = harness.clock.monotonic_ns()
            harness.bridge.robot_states = {
                arm_id: (stamp, sample[1])
                for arm_id, sample in harness.bridge.robot_states.items()
            }
            harness.bridge.diagnostics = {
                arm_id: (stamp, sample[1])
                for arm_id, sample in harness.bridge.diagnostics.items()
            }
            harness.tick()
            if harness.supervisor.state == 'fault':
                break

        assert harness.supervisor.state == 'fault'
        assert 'diagnostic_error' in harness.fault_codes()
        assert all(enabled is not True
                   for _slot, enabled in harness.bridge.enable_calls)

    def test_activation_delta_violation_stops_without_a_target(self, tmp_path):
        """A hard delta wins even if an ordinary session fault arrives with it."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        moved = list(IN_FENCE_POSE)
        moved[1] += 0.201
        harness.clock.advance(0.1)
        harness.set_joints(dual_joint_state(pose_2=tuple(moved)))
        harness.set_diagnostic('panda1', 2)
        harness.tick()
        frame = harness.supervisor.frame()
        assert harness.supervisor.state == 'stopped'
        assert frame['session']['last_error']['code'] == 'activation_settling_limit'
        assert frame['session']['activation']['status'] == 'failed'
        assert harness.bridge.published_targets == []

    def test_the_launch_activation_transient_is_discarded_not_gated(
            self, tmp_path):
        """
        The LAUNCH's own activation cannot be judged, so it is not judged.

        The capture armed before spawn accumulates the transient of the
        launch's `spawner --switch-asap` activation -- an activation with no
        pre-activation baseline, which no gate can honestly assess. The
        restage ends that capture and arms a fresh one immediately before the
        activation it DOES gate, so a pre-readiness excursion neither trips
        the gate nor leaks into it.
        """
        harness = self.prepared(tmp_path)
        harness.bridge.enable_ready = False        # hold short of readiness
        harness.start(arms='both', mode='motion')
        assert harness.supervisor.state == 'starting'
        generation_before = harness.bridge._activation_capture_generation
        moved = list(IN_FENCE_POSE)
        moved[1] += 0.201
        harness.clock.advance(0.001)
        harness.set_joints(dual_joint_state(pose_2=tuple(moved)))
        harness.clock.advance(0.001)
        harness.set_joints()

        harness.bridge.enable_ready = True
        harness.pump()

        frame = harness.supervisor.frame()
        assert harness.supervisor.state == 'settling', frame['session']
        assert frame['session']['last_error'] is None
        assert frame['session']['activation']['status'] == 'settling'
        # A FRESH capture: the launch transient was retired, not carried over.
        assert (harness.bridge._activation_capture_generation
                > generation_before)
        assert harness.bridge.published_targets == []
        harness.drive_settling()
        assert harness.supervisor.state == 'running'

    def test_settling_timeout_cannot_be_won_by_a_late_good_sample(self, tmp_path):
        """A fresh stable-looking receipt at the exact deadline still fails closed."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        harness.clock.advance(2.0)
        harness.set_joints()
        harness.tick()
        frame = harness.supervisor.frame()
        assert harness.supervisor.state == 'stopped'
        assert frame['session']['last_error']['code'] == 'activation_settling_timeout'
        assert frame['session']['activation']['status'] == 'failed'

    def test_final_excursion_has_priority_at_settling_deadline(self, tmp_path):
        """A callback after the poll drain is not discarded as a timeout."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        moved = list(IN_FENCE_POSE)
        moved[1] += 0.201
        original_drain = harness.bridge.drain_activation_capture

        def drain_then_inject():
            capture = original_drain()
            harness.bridge.drain_activation_capture = original_drain
            harness.set_joints(dual_joint_state(pose_2=tuple(moved)))
            return capture

        harness.bridge.drain_activation_capture = drain_then_inject
        harness.clock.advance(2.0)
        gate = harness.supervisor._activation_gate
        harness.tick()
        frame = harness.supervisor.frame()
        assert harness.supervisor.state == 'stopped'
        assert frame['session']['last_error']['code'] == 'activation_settling_limit'
        assert frame['session']['activation']['status'] == 'failed'
        assert gate.max_abs_delta['panda2'][1] > 0.2
        assert frame['fault']['active'] is False
        assert frame['fault']['reasons'] == []
        assert frame['fault']['recoverable'] is False

    def test_fault_exit_cannot_discard_a_concurrent_hard_excursion(self, tmp_path):
        """Activation hard evidence wins over a recoverable ordinary fault."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        harness.set_diagnostic('panda1', 2)
        moved = list(IN_FENCE_POSE)
        moved[1] += 0.201
        original_evaluate = harness.supervisor._fault_engine.evaluate

        def evaluate_then_inject(snapshot):
            reasons = original_evaluate(snapshot)
            harness.set_joints(dual_joint_state(pose_2=tuple(moved)))
            return reasons

        harness.supervisor._fault_engine.evaluate = evaluate_then_inject
        gate = harness.supervisor._activation_gate
        harness.tick()
        frame = harness.supervisor.frame()
        assert harness.supervisor.state == 'stopped'
        assert frame['session']['last_error']['code'] == 'activation_settling_limit'
        assert frame['session']['activation']['status'] == 'failed'
        assert gate.max_abs_delta['panda2'][1] > 0.2
        assert frame['fault']['active'] is False
        assert frame['fault']['reasons'] == []
        assert frame['fault']['recoverable'] is False

    def test_recovery_restarts_the_gate_after_prior_target_traffic(self, tmp_path):
        """Recovery never inherits readiness, enables, or old target counters."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        harness.jog('panda1', 6, 1)
        harness.supervisor.jog_stream_tick()
        published_before = published(harness)
        fault_by_hardware(harness)

        result = harness.recover(settle=False)
        assert result['enabled_after'] is False
        assert harness.supervisor.state == 'settling'
        frame = harness.supervisor.frame()
        assert frame['session']['activation']['status'] == 'settling'
        assert frame['session']['activation']['samples'] == 0
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        harness.supervisor.jog_stream_tick()
        assert published(harness) == published_before

        harness.drive_settling()
        assert harness.supervisor.state == 'running'
        assert harness.supervisor.frame()['session']['activation']['status'] == 'ready'

    def test_recovery_switch_excursion_and_return_is_not_hidden(self, tmp_path):
        """Recovery arms callback extrema before reactivating the controller."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)

        def transient(controllers):
            if IMPEDANCE not in controllers:
                return
            moved = list(IN_FENCE_POSE)
            moved[1] += 0.201
            harness.clock.advance(0.001)
            harness.set_joints(dual_joint_state(pose_2=tuple(moved)))
            harness.clock.advance(0.001)
            harness.set_joints()

        harness.bridge.on_switch_activate = transient
        with pytest.raises(SessionError) as excinfo:
            harness.recover(settle=False)
        assert excinfo.value.code == 'recovery_failed'
        assert 'transition envelope' in excinfo.value.detail
        assert [IMPEDANCE] in harness.bridge.deactivate_calls
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}


class TestActivationTargetRebase:
    """
    The supervisor must never rebase the controller's captured target.

    Live evidence (``web-20260831-145120`` and ``web-20260831-152313``): the
    post-readiness ``SetBool(false)`` the supervisor used to send after a
    fresh impedance activation was a false-to-false request. The reviewed
    inbox advances ``enable_generation`` on every such call, so the next RT
    cycle assigned the then-current measured pose to ``next_targets``, the
    restoring J2 spring torque collapsed from -1.088512 Nm to -0.003052 Nm,
    and a second settling episode began. These tests model that mechanism
    instead of counting calls.
    """

    SLOTS = (1, 2)

    def prepared(self, tmp_path, activation_pose=ACTIVATION_POSE,
                 settled_pose=IN_FENCE_POSE):
        """Build a rig whose controller activated at ``activation_pose``."""
        harness = MotionHarness(tmp_path, bridge_factory=RebaseModelBridge)
        harness.make_ready(controller_name=IMPEDANCE)
        # The controller reached ACTIVE and captured its internal target; the
        # arm then crept to the pose the session actually measures.
        harness.bridge.activate_impedance(self.SLOTS, activation_pose)
        harness.bridge.settle_to(settled_pose)
        return harness

    def inboxes(self, harness):
        """Return the modelled inbox of every configured slot."""
        return [harness.bridge.inboxes[slot] for slot in self.SLOTS]

    def test_the_model_rebases_when_a_false_to_false_call_is_sent(self, tmp_path):
        """
        Guard the guard: the model must be able to FAIL.

        This is the exact call the supervisor used to make after readiness. If
        the model did not react to it, every other test in this class would be
        vacuous, so assert the destruction directly.
        """
        harness = self.prepared(tmp_path)
        for inbox in self.inboxes(harness):
            assert inbox.enabled is False  # already disabled by onActivate()
            assert inbox.spring_error[1] == pytest.approx(-0.03)
        for slot in self.SLOTS:
            harness.bridge.call_enable(slot, False)
        harness.bridge.run_control_cycle()
        for inbox in self.inboxes(harness):
            assert inbox.rebases_since_activation == 1
            assert inbox.internal_target == IN_FENCE_POSE
            assert inbox.spring_error == pytest.approx((0.0,) * 7)

    def test_fresh_startup_reaches_running_without_rebasing_the_target(
            self, tmp_path):
        """
        Startup sends no false call, so the captured target survives.

        The activation the target is captured at is now the RESTAGE's own
        re-activation, so the captured pose is the one the arms were measured
        holding while the controller was paused. The restoring spring is
        created afterwards, by the arm creeping under gravity.
        """
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state == 'settling'
        harness.bridge.run_control_cycle(5)
        harness.drive_settling()
        harness.bridge.run_control_cycle(5)
        assert harness.supervisor.state == 'running'
        assert harness.bridge.enable_calls == []
        # The arm creeps 0.03 rad on joint2 after the restage handed it back.
        harness.bridge.settle_to(ACTIVATION_POSE)
        harness.bridge.run_control_cycle(5)
        for inbox in self.inboxes(harness):
            assert inbox.rebases_since_activation == 0
            assert inbox.enable_generation == inbox.observed_enable_generation
            assert inbox.internal_target == IN_FENCE_POSE
            # The restoring spring the activation capture exists to observe is
            # still there, unlike the collapsed torque the live bag recorded.
            assert inbox.spring_error[1] == pytest.approx(0.03)

    def test_startup_is_command_closed_while_the_target_is_preserved(
            self, tmp_path):
        """No enable, jog, target or generation change before Running."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state == 'settling'
        with pytest.raises(SessionError) as enable_error:
            harness.enable('panda1')
        with pytest.raises(SessionError) as jog_error:
            harness.jog('panda1', 6, 1)
        assert enable_error.value.code == 'session_not_running'
        assert jog_error.value.code == 'session_not_running'
        harness.bridge.run_control_cycle(5)
        assert harness.bridge.enable_calls == []
        assert harness.bridge.published_targets == []
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        for inbox in self.inboxes(harness):
            assert inbox.enabled is False
            assert inbox.rebases_since_activation == 0
            # Captured by the restage's own activation, from the pose measured
            # while the controller was paused.
            assert inbox.internal_target == IN_FENCE_POSE
        for arm in harness.supervisor.frame()['arms'].values():
            assert arm['motion']['available'] is False

    def test_unreachable_enable_service_never_reaches_settling(self, tmp_path):
        """
        Dropping the disables dropped no fail-closed property.

        The removed loop doubled as proof that the enable surface answered.
        ``_readiness_met`` already refuses to leave ``starting`` until every
        jog slot's enable service is reachable, so an unreachable service now
        times the session out instead of entering settling -- still closed,
        and still without commanding the controller.
        """
        harness = self.prepared(tmp_path)
        harness.bridge.enable_ready = False
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state == 'starting'
        harness.clock.advance(defaults.STARTING_TIMEOUT_S + 1.0)
        harness.pump()
        frame = harness.supervisor.frame()
        assert harness.supervisor.state == 'stopped'
        assert frame['session']['last_error']['code'] == 'launch_timeout'
        # Failing closed must not itself command the controller, and the
        # bounded capture must be retired on the way out.
        assert harness.bridge.enable_calls == []
        assert harness.bridge.published_targets == []
        assert harness.bridge._activation_capture is None
        for inbox in self.inboxes(harness):
            assert inbox.rebases_since_activation == 0
            assert inbox.internal_target == ACTIVATION_POSE

    def test_a_reachable_enable_service_is_still_required_before_settling(
            self, tmp_path):
        """The same rig with a reachable service does reach settling."""
        harness = self.prepared(tmp_path)
        harness.bridge.enable_ready = False
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state == 'starting'
        harness.bridge.enable_ready = True
        harness.refresh_joints()
        harness.pump()
        assert harness.supervisor.state == 'settling'
        assert harness.bridge.enable_calls == []

    def test_genuine_operator_disable_still_reaches_the_controller(self, tmp_path):
        """Only the REDUNDANT call was removed; a real disable still commands."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        harness.drive_settling()
        harness.claim_lock()
        harness.enable('panda1')
        assert harness.bridge.enable_calls == [(1, True)]
        harness.enable('panda1', enabled=False)
        assert harness.bridge.enable_calls == [(1, True), (1, False)]
        harness.bridge.run_control_cycle()
        enabled_slot = harness.bridge.inboxes[1]
        untouched_slot = harness.bridge.inboxes[2]
        # A real enable/disable pair IS a transition; rebasing there is the
        # controller behaving correctly, and the untouched arm keeps its
        # activation target.
        assert enabled_slot.rebases_since_activation == 1
        assert enabled_slot.enabled is False
        assert untouched_slot.rebases_since_activation == 0
        assert untouched_slot.internal_target == IN_FENCE_POSE

    def test_recovery_reactivation_target_is_not_rebased(self, tmp_path):
        """Pre-disable, deactivate, restore, capture -- then leave it alone."""
        harness = self.prepared(tmp_path)
        harness.start(arms='both', mode='motion')
        harness.pump()
        harness.drive_settling()
        assert harness.bridge.enable_calls == []
        # Scope every assertion below to THIS recovery: the start path drove
        # its own deactivate/activate pair (the restage).
        harness.reset_call_log()
        fault_by_hardware(harness)

        recovery_pose = tuple(IN_FENCE_POSE)

        def reactivate(controllers):
            # onActivate() -> disableAndInvalidateAll() -> first update()
            # captures the pose the arm holds at re-activation.
            if IMPEDANCE in controllers:
                harness.bridge.activate_impedance(self.SLOTS, recovery_pose)

        harness.bridge.on_switch_activate = reactivate
        result = harness.recover()
        harness.bridge.run_control_cycle(5)

        assert harness.supervisor.state == 'running'
        # Exactly the two meaningful pre-deactivation disables.
        assert harness.bridge.enable_calls == [(1, False), (2, False)]
        assert [(step['phase'], step['arm_id']) for step in result['steps']
                if step['step'] == 'controller_disable'] == [
                    ('pre', 'panda1'), ('pre', 'panda2')]
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]
        assert harness.bridge.switch_calls[-1] == [IMPEDANCE]
        assert result['enabled_after'] is False
        for inbox in self.inboxes(harness):
            assert inbox.rebases_since_activation == 0
            assert inbox.enable_generation == inbox.observed_enable_generation
            assert inbox.internal_target == recovery_pose


class TestOperatorLeaseBinding:
    """Queued mutators belong to the exact operator claim that submitted them."""

    def test_enable_queued_before_release_is_rejected_after_reclaim(self, tmp_path):
        """B merely claiming cannot inherit A's queued Enable or target stream."""
        harness = motion_running(tmp_path)
        first = harness.token
        old_lease = harness.operator_lease()
        outcome, requester = submit_async(lambda: harness.supervisor.request_arm_enable(
            'panda1', True, operator_lease=old_lease))
        wait_for_queued_command(harness)

        assert harness.lock.release(first) is True
        harness.token = harness.lock.claim().token
        assert harness.token is not None
        harness.tick()
        requester.join(timeout=1.0)

        assert isinstance(outcome.get('error'), SessionError)
        assert outcome['error'].code == 'operator_token_invalid'
        assert harness.bridge.enable_calls == []
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        harness.supervisor.jog_stream_tick()
        assert published(harness) == 0

    def test_stale_start_is_rejected_at_take(self, tmp_path):
        """A delayed Start cannot create a session after B takes control."""
        harness = MotionHarness(tmp_path)
        first = harness.claim_lock()
        old_lease = harness.operator_lease()
        assert harness.lock.release(first) is True
        harness.token = harness.lock.claim().token
        with pytest.raises(SessionError) as excinfo:
            harness.drive(lambda: harness.supervisor.request_start(
                SessionRequest(arms='both', mode='simulate'),
                operator_lease=old_lease))
        assert excinfo.value.code == 'operator_token_invalid'
        assert harness.supervisor.state == 'stopped'

    def test_stale_jog_cannot_mutate_a_fresh_operators_enabled_target(self, tmp_path):
        """A's delayed jog is inert even after B independently enables the arm."""
        harness = motion_running(tmp_path)
        first = harness.token
        old_lease = harness.operator_lease()
        assert harness.lock.release(first) is True
        harness.token = harness.lock.claim().token
        harness.enable('panda1')
        before = list(harness.model('panda1').target)

        with pytest.raises(SessionError) as excinfo:
            harness.drive(lambda: harness.supervisor.request_arm_jog(
                'panda1', 6, 1, operator_lease=old_lease))
        assert excinfo.value.code == 'operator_token_invalid'
        assert harness.model('panda1').target == pytest.approx(before)

    def test_stale_recover_is_rejected_before_backend_mutation(self, tmp_path):
        """A delayed Recover cannot activate hardware/controllers for B."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        first = harness.token
        old_lease = harness.operator_lease()
        assert harness.lock.release(first) is True
        harness.token = harness.lock.claim().token

        with pytest.raises(SessionError) as excinfo:
            harness.drive(lambda: harness.supervisor.request_session_recover(
                operator_lease=old_lease))
        assert excinfo.value.code == 'operator_token_invalid'
        assert harness.bridge.recovery_calls == []
        assert harness.bridge.switch_calls == []
        assert harness.supervisor.state == 'fault'


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

    def test_fresh_cached_fault_blocks_enable_before_controller_call(self, tmp_path):
        """A callback since the last poll cannot slip through queued Enable."""
        harness = motion_running(tmp_path)
        harness.set_diagnostic('panda1', 2)
        calls_before = list(harness.bridge.enable_calls)

        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')

        assert excinfo.value.code == 'session_faulted'
        assert harness.supervisor.state == 'fault'
        assert 'diagnostic_error' in harness.fault_codes()
        assert harness.bridge.enable_calls == calls_before

    def test_fault_arriving_during_enable_is_compensated_before_refusal(
            self, tmp_path):
        """A successful true response cannot outrun an in-flight health fault."""
        harness = motion_running(tmp_path)

        def inject_fault(_slot, enabled):
            if enabled:
                harness.set_diagnostic('panda1', 2)

        harness.bridge.on_enable = inject_fault
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')

        assert excinfo.value.code == 'session_faulted'
        assert harness.supervisor.state == 'fault'
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.model('panda1').target is None
        assert harness.bridge.enable_calls == [(1, True), (1, False)]

    def test_fresh_cached_fault_blocks_jog_before_target_changes(self, tmp_path):
        """A queued Jog cannot mutate a target from known-faulted cache state."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        target_before = tuple(harness.model('panda1').target)
        harness.set_diagnostic('panda1', 2)

        with pytest.raises(SessionError) as excinfo:
            harness.jog('panda1', 6, 1)

        assert excinfo.value.code == 'session_faulted'
        assert harness.supervisor.state == 'fault'
        assert 'diagnostic_error' in harness.fault_codes()
        assert target_before == pytest.approx(IN_FENCE_POSE)
        assert harness.model('panda1').target is None
        assert harness.model('panda1').seeded is False

    @pytest.mark.parametrize('mode', ['simulate', 'watch'])
    def test_non_motion_session_has_no_enable(self, tmp_path, mode):
        """Simulate and Watch sessions carry no motion surface at all."""
        harness = simple_running(tmp_path, mode)
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'not_motion_mode'

    def test_arm_outside_a_one_arm_session(self, tmp_path):
        """A one-arm session refuses the other arm with arm_not_in_session."""
        harness = motion_running(
            tmp_path, arms='panda1')
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
        harness.clock.advance(defaults.ENABLE_JOINT_STATE_MAX_AGE_S + 0.05)
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'joint_state_stale'
        assert str(defaults.ENABLE_JOINT_STATE_MAX_AGE_S) in excinfo.value.detail

    def test_sample_just_inside_the_enable_window_is_accepted(self, tmp_path):
        """The window is a limit, not a margin: just under 0.2 s still enables."""
        harness = motion_running(tmp_path)
        harness.clock.advance(defaults.ENABLE_JOINT_STATE_MAX_AGE_S - 0.05)
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

    def test_enable_and_disable_both_reach_the_log_bus(self, tmp_path):
        """
        §3 source 3: granting torque authority is logged, not just withdrawing.

        A log carrying `panda1 disabled` with no matching `panda1 enabled`
        leaves an operator reading it after an incident unable to see when
        the arm became commandable at all.
        """
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        messages = [line['message']
                    for line in harness.logs.window()['lines']]
        assert 'panda1 enabled' in messages
        assert 'panda2 enabled' not in messages

        harness.enable('panda1', enabled=False)
        messages = [line['message']
                    for line in harness.logs.window()['lines']]
        assert messages.index('panda1 enabled') < \
            messages.index('panda1 disabled')

    def test_a_refused_enable_logs_nothing(self, tmp_path):
        """Only a granted enable is logged; a refusal is not an enable."""
        harness = motion_running(tmp_path)
        harness.bridge.enable_response = {'success': False, 'message': 'no'}
        with pytest.raises(SessionError):
            harness.enable('panda1')
        messages = [line['message']
                    for line in harness.logs.window()['lines']]
        assert 'panda1 enabled' not in messages


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
        expected[2] += defaults.JOG_STEP_RAD
        assert result['arm_id'] == 'panda1'
        assert result['target'] == pytest.approx(expected)
        assert result['clamped'] == [False] * defaults.JOINT_COUNT
        assert harness.model('panda1').target == pytest.approx(expected)

    def test_a_negative_jog_moves_the_other_way(self, tmp_path):
        """A direction of -1 subtracts exactly one step."""
        harness = motion_running(tmp_path)
        harness.enable('panda1')
        result = harness.jog('panda1', 0, -1)
        assert result['target'][0] == pytest.approx(IN_FENCE_POSE[0] - defaults.JOG_STEP_RAD)

    def test_a_clamped_jog_reports_the_mask(self, tmp_path):
        """The fence cuts the step short and the mask says which joint it was."""
        harness = motion_running(tmp_path)
        harness.set_joints(dual_joint_state(pose_1=NEAR_UPPER_POSE))
        harness.enable('panda1')
        result = harness.jog('panda1', 3, 1)
        upper = harness.supervisor._profile_record.fence[
            'panda1']['position_upper'][3]
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

        # 3. the arm's command source is External, not Jog
        with supervisor._state_lock:
            supervisor._arm_source['panda1'] = 'external'
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        with supervisor._state_lock:
            supervisor._arm_source['panda1'] = 'jog'

        # 4. no arm enabled
        with supervisor._state_lock:
            supervisor._arm_enabled['panda1'] = False
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        with supervisor._state_lock:
            supervisor._arm_enabled['panda1'] = True

        # 5. the operator lock is not held. Giving the lock up revokes the
        # authorization inside the lock itself (finding F-0), so the fresh
        # claim starts from every enable off: the arm has to be enabled
        # again, because a new operator is never an automatic continuation
        # of the last one (§6.13).
        assert harness.lock.release(harness.token) is True
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        harness.claim_lock()
        assert harness.enabled_flags()['panda1'] is False
        harness.enable('panda1')

        # 6. the target is not seeded
        harness.model('panda1').invalidate()
        supervisor.jog_stream_tick()
        assert published(harness) == baseline
        harness.model('panda1').seed(IN_FENCE_POSE)

        supervisor.jog_stream_tick()
        assert published(harness) == baseline + 1

    def test_lock_expiry_stops_the_stream(self, tmp_path):
        """
        §5.6: an expired lock stops publication without anyone releasing it.

        Nothing may touch the lock between the clock advance and the tick.
        Reading `lock.state()` first is itself a lock entry point: it retires
        the expired lease, which runs the revocation hook and clears every
        enable flag, so the tick would then stop on the enable flag and the
        stream's own lock gate would never be reached. The tick has to be the
        thing that discovers the expiry -- which is also what makes lazy
        expiry fire on the 20 Hz jog timer rather than the 5 Hz frame pump.
        """
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.supervisor.jog_stream_tick()
        assert published(harness) == 1
        harness.clock.advance(defaults.OPERATOR_LOCK_TTL_S + 0.1)
        assert harness.enabled_flags()['panda1'] is True
        harness.supervisor.jog_stream_tick()
        assert published(harness) == 1
        assert harness.lock.state()['locked'] is False

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


class ReleaseOnSecondCheck:
    """
    The real lock, releasing itself strictly before the final commit.

    ``lease_is_current`` is consulted twice on the enable path: once when the
    command is taken off the queue, and once immediately before the local
    flag is committed. Releasing on the SECOND consultation lands the
    revocation in the one window neither of those checks covers -- after the
    last verification, before the commit -- which is the window the closing
    compare-and-set exists for.
    """

    def __init__(self, lock, token):
        """Wrap ``lock``, arming a release of ``token`` on the second check."""
        self._lock = lock
        self._token = token
        self.checks = 0
        self.released = False

    def __getattr__(self, name):
        """Delegate every other lock method to the real lock."""
        return getattr(self._lock, name)

    def lease_is_current(self, lease):
        """Answer truthfully, then release on the second consultation."""
        self.checks += 1
        verdict = self._lock.lease_is_current(lease)
        if self.checks == 2 and not self.released:
            self.released = True
            assert self._lock.release(self._token) is True
        return verdict


class TestEnableCommitCompareAndSet:
    """The closing CAS: a release racing the final local commit loses."""

    def test_a_release_after_the_last_check_cannot_commit_the_enable(
            self, tmp_path):
        """
        §6.13: the enable commits under the lock's own mutex, or not at all.

        Both in-flight revocation tests above land their release while
        SetBool(true) is blocked, so the earlier pre-commit check always
        catches it and the compare-and-set is never the deciding guard. This
        drives the release into the window strictly after that check, where
        only ``run_if_current`` can refuse: without it a successor operator
        would hold an enable they never pressed.
        """
        harness = motion_running(tmp_path)
        token = harness.claim_lock()
        proxy = ReleaseOnSecondCheck(harness.lock, token)
        harness.supervisor._lock_service = proxy

        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')

        assert excinfo.value.code == 'operator_token_invalid'
        assert proxy.checks >= 2, 'the pre-commit check never ran'
        assert proxy.released is True
        assert harness.enabled_flags()['panda1'] is False
        assert harness.bridge.enable_calls == [(1, True), (1, False)]
        harness.supervisor.jog_stream_tick()
        assert published(harness) == 0


class TestEnableInFlightRevocation:
    """A successful controller reply cannot resurrect a revoked authorization."""

    @staticmethod
    def begin_blocked_enable(harness):
        """Block SetBool(true) after command take and return its barriers."""
        entered = threading.Event()
        resume = threading.Event()

        def barrier(_slot, enabled):
            if enabled:
                entered.set()
                assert resume.wait(timeout=COMMAND_DEADLINE_S)

        harness.bridge.on_enable = barrier
        lease = harness.operator_lease()
        outcome, requester = submit_async(lambda: harness.supervisor.request_arm_enable(
            'panda1', True, operator_lease=lease))
        wait_for_queued_command(harness)
        ticker = threading.Thread(target=harness.tick, daemon=True)
        ticker.start()
        assert entered.wait(timeout=COMMAND_DEADLINE_S)
        return outcome, requester, ticker, resume

    @staticmethod
    def finish_blocked_enable(outcome, requester, ticker, resume):
        """Release the service barrier and require a safe stale-token refusal."""
        resume.set()
        ticker.join(timeout=COMMAND_DEADLINE_S)
        requester.join(timeout=COMMAND_DEADLINE_S)
        assert ticker.is_alive() is False
        assert requester.is_alive() is False
        assert isinstance(outcome.get('error'), SessionError)
        assert outcome['error'].code == 'operator_token_invalid'

    def test_pagehide_release_during_enable_compensates_and_cannot_publish(
            self, tmp_path):
        """Explicit release while SetBool(true) waits leaves B fully disabled."""
        harness = motion_running(tmp_path)
        first = harness.token
        outcome, requester, ticker, resume = self.begin_blocked_enable(harness)

        assert harness.lock.release(first) is True
        harness.token = harness.lock.claim().token
        assert harness.token is not None
        self.finish_blocked_enable(outcome, requester, ticker, resume)

        assert harness.bridge.enable_calls == [(1, True), (1, False)]
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.model('panda1').seeded is False
        harness.supervisor.jog_stream_tick()
        assert published(harness) == 0

    def test_ttl_expiry_and_reclaim_during_enable_cannot_resurrect_or_publish(
            self, tmp_path):
        """A successful late reply belongs to expired A, never successor B."""
        harness = motion_running(tmp_path)
        outcome, requester, ticker, resume = self.begin_blocked_enable(harness)

        harness.clock.advance(defaults.OPERATOR_LOCK_TTL_S + 0.001)
        harness.token = harness.lock.claim().token
        assert harness.token is not None
        self.finish_blocked_enable(outcome, requester, ticker, resume)

        assert harness.bridge.enable_calls == [(1, True), (1, False)]
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.model('panda1').seeded is False
        harness.supervisor.jog_stream_tick()
        assert published(harness) == 0

    def test_session_change_during_enable_compensates_and_invalidates_target(
            self, tmp_path):
        """A late controller success cannot commit into a changed session state."""
        harness = motion_running(tmp_path)

        def change_state(_slot, enabled):
            if enabled:
                harness.force_state('fault')

        harness.bridge.on_enable = change_state
        with pytest.raises(SessionError) as excinfo:
            harness.enable('panda1')
        assert excinfo.value.code == 'session_not_running'
        assert harness.bridge.enable_calls == [(1, True), (1, False)]
        assert harness.enabled_flags()['panda1'] is False
        assert harness.model('panda1').seeded is False


# ======================================================================
# Watch-mode fence preview (display-only; never a motion authority)
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
    """§6.13/§7.3: session-wide ordering, verification, and rollback."""

    def test_recover_on_a_running_session_is_refused(self, tmp_path):
        """not_faulted: recovery is a fault-only action."""
        harness = motion_running(tmp_path)
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'not_faulted'

    def test_recover_on_a_simulate_session_is_refused(self, tmp_path):
        """Recovery applies to watch and motion only; Simulate is refused."""
        harness = simple_running(tmp_path, 'simulate')
        harness.launch_child.die(returncode=1)
        harness.pump(1)
        assert harness.supervisor.state == 'fault'
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'session_not_running'
        assert excinfo.value.detail == (
            'recovery applies to watch and motion sessions only')

    def test_newly_dead_launch_revokes_stale_recovery_eligibility(self, tmp_path):
        """Fresh F7 refuses before any backend/controller/hardware mutation."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        assert harness.supervisor.frame()['fault']['recoverable'] is True
        harness.launch_child.die(returncode=1)
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'recovery_not_supported'
        assert harness.bridge.recovery_calls == []
        assert harness.bridge.enable_calls == []
        assert harness.bridge.deactivate_calls == []
        assert harness.bridge.switch_calls == []
        assert harness.bridge.hardware_active_calls == []
        frame = harness.supervisor.frame()
        assert 'launch_exited' in [
            reason['code'] for reason in frame['fault']['reasons']]
        assert frame['fault']['recoverable'] is False

    def test_naturally_cleared_reason_still_requires_and_allows_recover(
            self, tmp_path):
        """An empty current snapshot does not silently clear the latched fault."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.hardware = hardware_component('active')
        assert harness.supervisor.state == 'fault'
        result = harness.recover()
        assert result['enabled_after'] is False
        assert harness.bridge.recovery_calls == ['panda1', 'panda2']
        assert harness.supervisor.state == 'running'

    @pytest.mark.parametrize('first_response', [
        None,
        {'success': False, 'error': 'reflex not cleared'},
    ])
    def test_dual_recovery_visits_both_arms_after_first_failure(
            self, tmp_path, first_response):
        """The sibling backend is never left unvisited by an earlier failure."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.recovery_responses = {
            'panda1': first_response,
            'panda2': {'success': False, 'error': NO_ERRORS_MESSAGE},
        }
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        expected = ('recovery_service_unavailable' if first_response is None
                    else 'recovery_failed')
        assert excinfo.value.code == expected
        assert harness.bridge.recovery_calls == ['panda1', 'panda2']
        assert harness.bridge.hardware_active_calls == []
        assert harness.bridge.switch_calls == []
        recovery_steps = [step for step in excinfo.value.payload['steps']
                          if step['step'] == 'error_recovery']
        assert [step['arm_id'] for step in recovery_steps] == ['panda1', 'panda2']
        assert [step['ok'] for step in recovery_steps] == [False, True]

    def test_no_errors_is_informational_not_a_failure(self, tmp_path):
        """§0.8: ``success=false, error='No errors'`` is neutral per arm."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.recovery_response = {'success': False, 'error': NO_ERRORS_MESSAGE}
        result = harness.recover()
        recovery_steps = [step for step in result['steps']
                          if step['step'] == 'error_recovery']
        assert recovery_steps == [
            {'step': 'error_recovery', 'arm_id': 'panda1', 'ok': True,
             'detail': NO_ERRORS_MESSAGE},
            {'step': 'error_recovery', 'arm_id': 'panda2', 'ok': True,
             'detail': NO_ERRORS_MESSAGE},
        ]
        assert result['enabled_after'] is False

    def test_an_inactive_hardware_component_is_reactivated(self, tmp_path):
        """Hardware is restored before the pre-disabled controller, last."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        result = harness.recover()
        assert harness.bridge.hardware_active_calls == [HARDWARE_NAME]
        hardware_step = next(step for step in result['steps']
                             if step['step'] == 'hardware_active')
        assert hardware_step['ok'] is True
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]
        assert harness.bridge.switch_calls == [[IMPEDANCE]]
        controller_steps = [step for step in result['steps']
                            if step['step'] == 'controller_active']
        assert controller_steps[-1]['controller'] == IMPEDANCE
        assert result['enabled_after'] is False
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_a_controller_active_at_the_activation_loop_refuses_recovery(
            self, tmp_path):
        """
        Recover never arms the gate against a post-torque pose.

        The startup path's counterpart of this branch is mutation-proved
        twice; this is the recovery one. The controller reads INACTIVE at the
        pre-deactivation phase -- so nothing deactivates it -- and ACTIVE
        again by the time the activation loop reaches it, which is exactly
        the race where a recovery could otherwise capture an already-torqued
        pose as the baseline settling is measured from.
        """
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        real_states = harness.bridge.query_controller_states

        def scripted(*args, **kwargs):
            states = real_states(*args, **kwargs)
            if states is None:
                return states
            states = dict(states)
            # The hardware phase sits between the two reads, so this flips
            # exactly once and without counting call sites.
            if harness.bridge.hardware_active_calls:
                states[IMPEDANCE] = 'active'
            else:
                states.pop(IMPEDANCE, None)
            return states

        harness.bridge.query_controller_states = scripted

        with pytest.raises(SessionError) as excinfo:
            harness.recover(settle=False)

        assert excinfo.value.code == 'recovery_failed'
        assert excinfo.value.detail == (
            'the motion controller became active before activation settling '
            'could be armed')
        # Nothing was activated, and the fail-closed rollback deactivated the
        # controller the recovery found active behind its back.
        assert harness.bridge.switch_calls == []
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]
        assert not any(step.get('step') == 'controller_active'
                       and step.get('controller') == IMPEDANCE
                       for step in excinfo.value.payload['steps'])
        assert harness.supervisor._activation_gate is None
        assert harness.supervisor._activation_baseline is None
        assert harness.supervisor.state == 'fault'
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_false_hardware_ack_preserves_steps_and_rolls_motion_back(self, tmp_path):
        """A refused hardware transition activates no controller and stays faulted."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.hardware_response = {'ok': False}
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'recovery_failed'
        assert 'hardware' in excinfo.value.detail
        assert harness.bridge.switch_calls == []
        assert excinfo.value.payload['enabled_after'] is False
        assert any(step['step'] == 'hardware_active' and not step['ok']
                   for step in excinfo.value.payload['steps'])
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]

    def test_missing_controllers_restore_in_live_proven_order(self, tmp_path):
        """JSB, state, model, then motion are activated one at a time."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        expected = list(expected_broadcasters(('panda1', 'panda2'), 'dual'))
        expected.append(IMPEDANCE)
        for name in expected:
            harness.bridge.controllers[name] = 'inactive'
        result = harness.recover()
        assert harness.bridge.switch_calls == [[name] for name in expected]
        activated = [step['controller'] for step in result['steps']
                     if step['step'] == 'controller_active']
        assert activated == expected

    def test_watch_restores_all_five_broadcasters_and_no_motion(self, tmp_path):
        """Dual Watch restoration includes both model broadcasters."""
        harness = simple_running(tmp_path, 'watch')
        fault_by_hardware(harness)
        expected = list(expected_broadcasters(('panda1', 'panda2'), 'dual'))
        for name in expected:
            harness.bridge.controllers[name] = 'inactive'
        result = harness.recover()
        assert harness.bridge.recovery_calls == ['panda1', 'panda2']
        assert harness.bridge.switch_calls == [[name] for name in expected]
        assert all(step.get('controller') != IMPEDANCE for step in result['steps'])

    def test_single_watch_uses_unprefixed_state_and_model_names(self, tmp_path):
        """Single-arm Watch restores JSB and the two unprefixed broadcasters."""
        harness = simple_running(tmp_path, 'watch', arms='panda2')
        fault_by_hardware(harness)
        expected = [
            'joint_state_broadcaster',
            'franka_robot_state_broadcaster',
            'franka_robot_model_broadcaster',
        ]
        for name in expected:
            harness.bridge.controllers[name] = 'inactive'
        result = harness.recover()
        assert result['arm_ids'] == ['panda2']
        assert harness.bridge.recovery_calls == ['panda2']
        assert harness.bridge.switch_calls == [[name] for name in expected]

    def test_single_impedance_uses_unprefixed_broadcasters_then_motion(
            self, tmp_path):
        """Single motion keeps its exact topology and restores motion last."""
        harness = motion_running(tmp_path, arms='panda1')
        fault_by_hardware(harness)
        expected = [
            'joint_state_broadcaster',
            'franka_robot_state_broadcaster',
            'franka_robot_model_broadcaster',
            IMPEDANCE,
        ]
        for name in expected:
            harness.bridge.controllers[name] = 'inactive'
        result = harness.recover()
        assert result['arm_ids'] == ['panda1']
        assert harness.bridge.switch_calls == [[name] for name in expected]
        assert [step['controller'] for step in result['steps']
                if step['step'] == 'controller_active'] == expected

    def test_active_impedance_is_disabled_before_deactivation_only(self, tmp_path):
        """An active controller is disabled, deactivated, then restored last."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        result = harness.recover()
        disables = [step for step in result['steps']
                    if step['step'] == 'controller_disable']
        # ``pre`` only. The controller was ACTIVE across the fault, so its
        # inboxes may really have been enabled and must be commanded off. The
        # restore at the end re-runs onActivate(), which disables and
        # invalidates every inbox on its own; a ``post`` phase would only
        # advance enable_generation and rebase the freshly captured target.
        assert [(step['phase'], step['arm_id']) for step in disables] == [
            ('pre', 'panda1'), ('pre', 'panda2'),
        ]
        assert all(step['ok'] for step in disables)
        assert harness.bridge.enable_calls == [(1, False), (2, False)]
        invariant = next(step for step in result['steps']
                         if step['step'] == 'activation_disable_invariant')
        assert invariant['ok'] is True and invariant['controller'] == IMPEDANCE
        names = [step['step'] for step in result['steps']]
        assert names.index('activation_disable_invariant') > max(
            index for index, step in enumerate(result['steps'])
            if step['step'] == 'controller_active')
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]
        assert harness.bridge.switch_calls[-1] == [IMPEDANCE]
        inactive = next(step for step in result['steps']
                        if step['step'] == 'controller_inactive')
        active = [step for step in result['steps']
                  if step['step'] == 'controller_active'][-1]
        assert inactive['ok'] is True and inactive['controller'] == IMPEDANCE
        assert active['ok'] is True and active['controller'] == IMPEDANCE
        names = [step['step'] for step in result['steps']]
        assert names.index('controller_inactive') < names.index('error_recovery')
        assert max(index for index, step in enumerate(result['steps'])
                   if step['step'] == 'error_recovery') < result['steps'].index(active)

    def test_failed_pre_disable_deactivates_and_never_recovers_backends(self, tmp_path):
        """Unknown controller-side enable state blocks backend/hardware work."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.enable_response = {'success': False, 'message': 'rejected'}
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'recovery_failed'
        assert harness.bridge.recovery_calls == []
        assert harness.bridge.hardware_active_calls == []
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]
        assert harness.bridge.controllers[IMPEDANCE] == 'inactive'

    def test_unreachable_controller_query_still_attempts_fail_closed_rollback(
            self, tmp_path):
        """Unknown lifecycle never skips controller-side disable/deactivation."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.controller_query_response = None
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'recovery_service_unavailable'
        assert harness.bridge.recovery_calls == []
        assert harness.bridge.enable_calls[-2:] == [(1, False), (2, False)]
        assert harness.bridge.deactivate_calls == [[IMPEDANCE]]
        rollback = [step for step in excinfo.value.payload['steps']
                    if step['step'].startswith('rollback_')]
        assert [step['step'] for step in rollback] == [
            'rollback_disable', 'rollback_disable', 'rollback_deactivate']
        assert rollback[-1]['ok'] is False  # no fresh query could prove inactive

    def test_inactive_impedance_recovers_with_no_controller_side_false_call(
            self, tmp_path):
        """
        An already-inactive controller is restored by ``onActivate()`` alone.

        There is no pre-disable (nothing could be enabled through an inactive
        controller) and deliberately no post-disable. The enable service is
        scripted to REFUSE every call, so any surviving ``SetBool(false)``
        would fail this recovery instead of quietly rebasing the target the
        re-activation just captured.
        """
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.bridge.controllers[IMPEDANCE] = 'inactive'
        harness.bridge.enable_response = {'success': False, 'message': 'rejected'}
        result = harness.recover()
        assert harness.bridge.enable_calls == []
        assert harness.bridge.deactivate_calls == []
        assert [IMPEDANCE] in harness.bridge.switch_calls
        assert harness.bridge.controllers[IMPEDANCE] == 'active'
        assert result['enabled_after'] is False
        assert [step for step in result['steps']
                if step['step'] == 'controller_disable'] == []
        assert next(step for step in result['steps']
                    if step['step'] == 'activation_disable_invariant')['ok'] is True
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

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
        harness.recover()
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
        result = harness.recover()
        assert result['enabled_after'] is False
        assert result['arm_ids'] == ['panda1', 'panda2']
        harness.pump(2)
        assert harness.supervisor.state == 'running'
        # The controller may be active, but every target authorization is off.
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.supervisor.frame()['fault']['active'] is False
        before = published(harness)
        harness.supervisor.jog_stream_tick()
        assert published(harness) == before

    def test_healthy_recorder_is_checked_before_recovery_success(self, tmp_path):
        """The synchronous restore verifies its recording chain before 200."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        ticks_before = harness.recorder.ticks

        result = harness.recover()

        assert result['enabled_after'] is False
        # At least one explicit check inside recovery, then the ordinary
        # fault-state poll that publishes the transition back to running.  The
        # harness may tick running again while the requester thread wakes.
        assert harness.recorder.ticks >= ticks_before + 2
        assert harness.supervisor.state == 'running'

    def test_recorder_failure_during_recovery_rolls_back_and_never_succeeds(
            self, tmp_path):
        """A dead recording chain cannot produce a successful restore verdict."""
        from franka_web.recording import RecordingError

        harness = motion_running(tmp_path)
        fault_by_hardware(harness)

        def fail_recorder_after_backend_restore(_arm_id):
            def failed_tick(_env):
                raise RecordingError('scripted recorder failure during recovery')
            harness.recorder.tick = failed_tick

        harness.bridge.on_recovery_success = fail_recorder_after_backend_restore
        with pytest.raises(SessionError) as excinfo:
            harness.recover()

        assert excinfo.value.code == 'recovery_failed'
        assert 'recorder failed during recovery' in excinfo.value.detail
        assert harness.supervisor._recover_succeeded is False
        assert harness.bridge.deactivate_calls[-1] == [IMPEDANCE]
        assert harness.bridge.controllers[IMPEDANCE] == 'inactive'
        assert harness.supervisor.state == 'stopped'
        assert harness.supervisor.frame()['session']['last_error']['code'] == \
            'recording_failed'

    def test_completed_recovery_cannot_run_twice_before_the_next_poll(self, tmp_path):
        """A second queued press cannot repeat backend/controller transitions."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.recover()
        calls = list(harness.bridge.recovery_calls)
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'not_faulted'
        assert harness.bridge.recovery_calls == calls

    def test_next_tick_refault_retires_capture_and_next_recover_works(
            self, tmp_path):
        """A new post-restore fault cannot poison the next one-click recovery."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        prior_reason = harness.supervisor._fault_reasons[0]
        original_evaluate = harness.supervisor._fault_engine.evaluate
        injected = {'done': False}

        def refault_once(snapshot):
            if harness.supervisor._recover_succeeded and not injected['done']:
                injected['done'] = True
                return (prior_reason,)
            return original_evaluate(snapshot)

        harness.supervisor._fault_engine.evaluate = refault_once
        first = harness.recover(settle=False)
        assert first['enabled_after'] is False
        assert harness.supervisor.state == 'fault'
        assert harness.bridge._activation_capture is None
        assert harness.supervisor._activation_gate is None
        first_generation = harness.bridge._activation_capture_generation

        harness.bridge.recovery_waits_remaining = 2
        second = harness.recover(settle=False)
        assert second['enabled_after'] is False
        assert harness.supervisor.state == 'settling'
        assert harness.bridge._activation_capture is not None
        assert harness.bridge._activation_capture_generation == first_generation + 1
        harness.drive_settling()
        assert harness.supervisor.state == 'running'

    def test_a_still_firing_rule_fails_the_request_and_stays_faulted(self, tmp_path):
        """No 200 is returned while the fresh fault snapshot still fires."""
        harness = motion_running(tmp_path)
        fault_by_diagnostic(harness)
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'recovery_failed'
        verification = next(
            step for step in excinfo.value.payload['steps']
            if step['step'] == 'verify_restore')
        assert verification['ok'] is False
        assert [reason['code'] for reason in verification['fault_reasons']] == [
            'diagnostic_error']
        assert harness.supervisor.state == 'fault'
        assert 'diagnostic_error' in harness.fault_codes()

    def test_no_post_restore_samples_fails_even_when_fault_rules_are_clear(
            self, tmp_path):
        """Lifecycle success alone cannot substitute for fresh joints/health."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        # One post-barrier state-only joint receipt is now required before
        # controller activation; leave no second receipt for restore proof.
        harness.bridge.recovery_waits_remaining = 1
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        verification = next(
            step for step in excinfo.value.payload['steps']
            if step['step'] == 'verify_restore')
        assert verification['ok'] is False
        assert verification['samples_fresh'] is False
        assert verification['fault_reasons'] == []
        assert harness.bridge.deactivate_calls[-1] == [IMPEDANCE]
        assert harness.supervisor.state == 'fault'
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_launch_dying_during_fresh_wait_revokes_published_recovery(
            self, tmp_path):
        """Late F7 replaces stale recoverable reasons before failure returns."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)

        def publish_then_die():
            harness.set_joints()
            stamp = harness.clock.monotonic_ns()
            for arm_id in ('panda1', 'panda2'):
                harness.bridge.robot_states[arm_id] = (
                    stamp, healthy_robot_state())
                harness.set_diagnostic(arm_id, 0)
            harness.launch_child.die(returncode=1)

        harness.bridge.on_recovery_wait = publish_then_die
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'recovery_failed'
        frame = harness.supervisor.frame()
        assert 'launch_exited' in [
            reason['code'] for reason in frame['fault']['reasons']]
        assert frame['fault']['recoverable'] is False
        assert frame['fault']['recover_hint'] is None
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_pre_restore_newer_samples_do_not_satisfy_the_freshness_gate(
            self, tmp_path):
        """Traffic before lifecycle restoration is older than the comparison point."""
        harness = motion_running(tmp_path)
        fault_by_diagnostic(harness)

        def publish_during_backend_recovery(_arm_id):
            harness.clock.advance(0.001)
            harness.set_joints()
            stamp = harness.clock.monotonic_ns()
            for arm_id in ('panda1', 'panda2'):
                harness.bridge.robot_states[arm_id] = (
                    stamp, healthy_robot_state())
                harness.set_diagnostic(arm_id, 0)

        harness.bridge.on_recovery_success = publish_during_backend_recovery
        # Let the new pre-activation baseline pass, but still provide no
        # post-restore sample that could satisfy verify_restore.
        harness.bridge.recovery_waits_remaining = 1
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        verification = next(
            step for step in excinfo.value.payload['steps']
            if step['step'] == 'verify_restore')
        assert verification['fault_reasons'] == []
        assert verification['samples_fresh'] is False
        assert harness.supervisor.state == 'fault'
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}

    def test_absent_baseline_rejects_receipts_captured_at_the_barrier(self, tmp_path):
        """A late callback cannot turn pre-barrier data into recovery evidence."""
        harness = motion_running(tmp_path)
        with harness.supervisor._state_lock:
            session = dict(harness.supervisor._session)
        harness.bridge.joint = None
        harness.bridge.robot_states = {}
        harness.bridge.diagnostics = {}

        baseline = harness.supervisor._recovery_sample_stamps(session)
        captured_ns = baseline['barrier_ns']
        harness.bridge.joint = (captured_ns, dual_joint_state())
        for arm_id in session['arm_ids']:
            harness.bridge.robot_states[arm_id] = (
                captured_ns, healthy_robot_state())
            harness.bridge.diagnostics[arm_id] = (
                captured_ns, diagnostic_at(arm_id))

        assert harness.supervisor._recovery_samples_fresh(
            session, baseline) is False
        accepted_ns = captured_ns + 1
        harness.bridge.joint = (accepted_ns, dual_joint_state())
        for arm_id in session['arm_ids']:
            harness.bridge.robot_states[arm_id] = (
                accepted_ns, healthy_robot_state())
            harness.bridge.diagnostics[arm_id] = (
                accepted_ns, diagnostic_at(arm_id))
        assert harness.supervisor._recovery_samples_fresh(
            session, baseline) is True

    def test_final_service_queries_precede_the_last_fresh_sample_wait(self, tmp_path):
        """State lost during final queries must be republished before success."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        phase = {'disables': 0, 'restored': False, 'cleared': False}

        def count_disables(_slot, enabled):
            if not enabled:
                phase['disables'] += 1

        def note_motion_restore(controllers):
            if IMPEDANCE in controllers:
                phase['restored'] = True

        original_hardware_query = harness.bridge.query_hardware_component

        def clear_samples_during_final_query(timeout_s=5.0):
            result = original_hardware_query(timeout_s)
            # The FINAL hardware query is the first one after the motion
            # controller was re-activated; there is no longer a post-disable
            # to count toward, so use the restore itself as the marker.
            if phase['restored'] and not phase['cleared']:
                phase['cleared'] = True
                harness.bridge.joint = None
                harness.bridge.robot_states = {}
                harness.bridge.diagnostics = {}
            return result

        waits = {'count': 0}

        def publish_after_queries():
            harness.clock.advance(0.001)
            harness.set_joints()
            waits['count'] += 1
            if waits['count'] == 1:
                # This first receipt proves the state-only activation baseline.
                # Re-arm the callback for the distinct post-query proof below.
                harness.bridge.on_recovery_wait = publish_after_queries
                return
            stamp = harness.clock.monotonic_ns()
            for arm_id in ('panda1', 'panda2'):
                harness.bridge.robot_states[arm_id] = (
                    stamp, healthy_robot_state())
                harness.set_diagnostic(arm_id, 0)

        harness.bridge.on_enable = count_disables
        harness.bridge.on_switch_activate = note_motion_restore
        harness.bridge.query_hardware_component = clear_samples_during_final_query
        harness.bridge.on_recovery_wait = publish_after_queries
        result = harness.recover()

        verification = next(
            step for step in result['steps'] if step['step'] == 'verify_restore')
        # Exactly the two pre-deactivation disables, never a post pair.
        assert phase == {'disables': 2, 'restored': True, 'cleared': True}
        assert verification['ok'] is True
        assert verification['samples_fresh'] is True
        assert verification['fault_reasons'] == []

    def test_taken_recovery_waits_for_its_real_verdict_past_timeout(self, tmp_path):
        """A begun command never reports timeout while it keeps executing."""
        harness = MotionHarness(tmp_path)
        outcome = {}

        def submit():
            outcome['result'] = harness.supervisor.request_session_recover(
                operator_lease=harness.operator_lease(),
                timeout_s=0.01)

        requester = threading.Thread(target=submit, daemon=True)
        requester.start()
        command = harness.supervisor._commands.get(timeout=1.0)
        assert command.try_begin() is True
        time.sleep(0.03)  # exceed the caller's initial wait after being taken
        command.resolve({'verdict': 'real'})
        requester.join(timeout=1.0)
        assert requester.is_alive() is False
        assert outcome == {'result': {'verdict': 'real'}}
        # The public wait must still cover a worst-case dual recovery: 149 s
        # of bounded service time PLUS the reviewed spacing on each of its
        # seven switch_controller calls, with scheduling margin on top.
        assert RECOVERY_REQUEST_TIMEOUT_S >= 159.0 + 7 * SWITCH_DWELL_S

    def test_unexpected_taken_command_error_resolves_waiter_and_propagates(
            self, tmp_path):
        """The waiter gets safe internal_error while tick still fails outward."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        outcome = {}

        def explode():
            raise RuntimeError('private backend detail')

        def submit():
            try:
                harness.supervisor.request_session_recover(
                    operator_lease=harness.operator_lease())
            except SessionError as error:
                outcome['error'] = error

        harness.bridge.query_controller_states = explode
        requester = threading.Thread(target=submit, daemon=True)
        requester.start()
        deadline = time.monotonic() + 1.0
        while (harness.supervisor._commands.empty()
               and time.monotonic() < deadline):
            time.sleep(0.001)
        with pytest.raises(RuntimeError, match='private backend detail'):
            harness.supervisor.tick()
        requester.join(timeout=1.0)
        assert requester.is_alive() is False
        error = outcome['error']
        assert error.code == 'internal_error'
        assert error.detail == 'internal server error'
        assert 'private backend detail' not in error.detail


def recovery_checklist(harness):
    """Return the frame's steps if they are a recovery checklist, else None."""
    steps = harness.supervisor.frame()['session']['steps']
    first = steps[0]['id'] if steps else ''
    return steps if first.startswith('reconnect:') else None


class TestRecoveryProgressEvidence:
    """
    V2L-7: the frame is the console's ONLY evidence that a recovery is running.

    Live, 2026-09-02: after one successful Recover the session faulted again;
    the operator pressed Recover a second time, NO ``recovery started`` line
    was ever emitted server-side, and the page sat on "Recovering"
    indefinitely with its control disabled. The page had no business showing
    it -- but the frame had handed it the evidence, because the FINISHED
    recovery checklist was still there when the new fault episode began.

    A recovery checklist belongs to the recovery that ran it. These pin that
    the server never publishes one for a recovery it has not started, so
    ``recoveryInProgress()`` in ``app.js`` has nothing to misread.
    """

    def test_a_new_fault_does_not_inherit_the_last_recovery_checklist(
            self, tmp_path):
        """THE live bug: a fresh fault episode starts with no recovery steps."""
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        harness.recover()
        assert harness.supervisor.state == 'running'
        finished = recovery_checklist(harness)
        assert finished is not None, 'the recovery published no checklist'
        assert all(step['status'] == 'done' for step in finished), finished

        # A NEW fault. Nothing about it is a recovery, and nothing in the
        # frame may say otherwise.
        fault_by_hardware(harness)
        assert harness.supervisor.state == 'fault'
        assert recovery_checklist(harness) is None, (
            'the new fault inherited the last recovery checklist: {!r}'.format(
                harness.supervisor.frame()['session']['steps']))
        assert harness.supervisor.frame()['session']['steps'] == []
        # And the Recover offer itself is intact: this fault is still one the
        # operator may act on.
        assert harness.supervisor.frame()['fault']['action'] == 'recover'

    def test_a_start_path_refusal_keeps_its_own_failed_checklist(self, tmp_path):
        """Only a RECOVERY checklist is stale evidence; a start's is teaching."""
        harness = MotionHarness(tmp_path, fences=uniform_fences())
        harness.make_ready(controller_name=IMPEDANCE)
        # The resting pose the restage will measure is outside the fence, so
        # the start is refused INTO fault with its own failed step.
        harness.set_joints(dual_joint_state(pose_1=OUT_OF_FENCE_POSE))
        harness.start(arms='both', mode='motion')
        assert harness.supervisor.state == 'fault'
        steps = harness.supervisor.frame()['session']['steps']
        assert steps, 'the operator lost every account of where the start broke'
        assert recovery_checklist(harness) is None
        assert [step['status'] for step in steps].count('failed') == 1

    def test_a_recover_that_never_starts_publishes_no_recovery_evidence(
            self, tmp_path):
        """
        A queued-but-unstarted Recover leaves the frame silent, and answers.

        This is the shape the page had to survive: the supervisor is busy, the
        command is never taken, and the operator gets a refusal to render in
        the notice bar rather than a progress state nobody confirmed.
        """
        harness = motion_running(tmp_path)
        fault_by_hardware(harness)
        before = harness.supervisor.frame()['session']['steps']
        outcome, thread = submit_async(
            lambda: harness.supervisor.request_session_recover(
                operator_lease=harness.operator_lease(), timeout_s=0.05))
        wait_for_queued_command(harness)
        thread.join(timeout=COMMAND_DEADLINE_S)
        assert thread.is_alive() is False
        assert 'error' in outcome, outcome
        assert outcome['error'].code == 'internal_error'
        assert 'retry once the state settles' in outcome['error'].detail
        # Never taken, so never any recovery evidence -- before or after.
        assert recovery_checklist(harness) is None
        assert harness.supervisor.frame()['session']['steps'] == before
        # And the abandoned command really is skipped, not run late.
        harness.pump(2)
        assert recovery_checklist(harness) is None
        assert harness.supervisor.state == 'fault'

    def test_a_refused_recover_publishes_where_it_broke_and_raises(self, tmp_path):
        """A recovery that STARTED and failed leaves a failed checklist."""
        harness = motion_running(tmp_path)
        fault_by_diagnostic(harness)
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'recovery_failed'
        steps = recovery_checklist(harness)
        assert steps is not None, 'a started recovery published no checklist'
        assert any(step['status'] == 'failed' for step in steps), steps
        assert harness.supervisor.state == 'fault'


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

    def test_impedance_f5_alone_is_recoverable_and_restored(self, tmp_path):
        """The full restore now addresses an inactive impedance controller."""
        harness = motion_running(tmp_path)
        harness.bridge.controllers[IMPEDANCE] = 'inactive'
        harness.pump(1)
        frame = harness.supervisor.frame()
        assert frame['fault']['recoverable'] is True
        assert frame['fault']['recover_hint']
        result = harness.recover()
        assert result['enabled_after'] is False
        assert harness.bridge.switch_calls[-1] == [IMPEDANCE]
        assert harness.supervisor.state == 'running'

    def test_physical_stop_shape_restores_full_stack_and_fresh_state(self, tmp_path):
        """F2+F5+F6+F8 restores broadcasters, motion last, and fresh samples."""
        harness = motion_running(tmp_path)
        stopped = healthy_robot_state()
        stopped.robot_mode = 5
        harness.bridge.robot_states['panda1'] = (
            harness.clock.monotonic_ns(), stopped)
        expected = list(expected_broadcasters(('panda1', 'panda2'), 'dual'))
        expected.append(IMPEDANCE)
        for name in expected:
            harness.bridge.controllers[name] = 'inactive'
        harness.bridge.hardware = hardware_component('inactive')
        harness.clock.advance(defaults.JOINT_STATE_STALE_FAULT_S + 0.01)
        harness.pump(1)
        frame = harness.supervisor.frame()
        codes = [reason['code'] for reason in frame['fault']['reasons']]
        assert {'robot_mode_fault', 'controller_deactivated',
                'joint_state_stale', 'hardware_inactive'} <= set(codes)
        assert frame['fault']['recoverable'] is True

        def publish_fresh_state():
            harness.clock.advance(0.001)
            harness.set_joints()
            stamp = harness.clock.monotonic_ns()
            for arm_id in ('panda1', 'panda2'):
                harness.bridge.robot_states[arm_id] = (
                    stamp, healthy_robot_state())
                harness.set_diagnostic(arm_id, 0)

        harness.bridge.on_recovery_wait = publish_fresh_state
        result = harness.recover()
        assert harness.bridge.recovery_calls == ['panda1', 'panda2']
        assert harness.bridge.hardware_active_calls == [HARDWARE_NAME]
        assert harness.bridge.switch_calls == [[name] for name in expected]
        assert result['steps'][-1]['step'] == 'verify_restore'
        assert result['steps'][-1]['samples_fresh'] is True
        assert result['enabled_after'] is False
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        assert harness.supervisor.state == 'running'

    def test_f1_plus_f7_dead_launch_blocks_full_recovery(self, tmp_path):
        """A diagnostic fault cannot hide the restart-only dead launch."""
        harness = motion_running(tmp_path)
        harness.set_diagnostic('panda1', 2)
        harness.launch_child.die(returncode=1)
        harness.pump(1)
        frame = harness.supervisor.frame()
        assert {'diagnostic_error', 'launch_exited'} <= {
            reason['code'] for reason in frame['fault']['reasons']}
        assert frame['fault']['recoverable'] is False
        assert frame['fault']['recover_hint'] is None
        with pytest.raises(SessionError) as excinfo:
            harness.recover()
        assert excinfo.value.code == 'recovery_not_supported'
        assert harness.bridge.recovery_calls == []

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

        harness.supervisor.revoke_operator_authorization()
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
        harness.supervisor.revoke_operator_authorization()
        assert harness.supervisor._commands.empty() is True
        assert harness.bridge.enable_calls == []


class TestExpiryReclaimRace:
    """§5.6/§6.13: a re-claim inside one frame-pump period (finding F-0)."""

    def test_reclaim_inside_one_pump_period_does_not_inherit_enables(self, tmp_path):
        """
        Expiry followed immediately by a claim still clears every enable.

        Expiry used to be noticed only as a falling edge in the 5 Hz frame
        pump's ``lock.state()['locked']`` sample. A claim landing inside one
        sampling period left ``locked`` True at both samples, so the edge
        never fired, the revocation was skipped, and the jog stream
        went on publishing for a NEW operator who had never pressed Enable.
        The revocation now happens inside the lock, before any successor
        token can exist, so pump timing cannot decide the authorization.

        Nothing observes the lock between the deadline and the claim here:
        the claim is the first entry point after expiry, which is exactly
        the window the pump could not see.
        """
        harness = motion_running(tmp_path)
        first = harness.claim_lock()
        harness.enable('panda1')
        harness.supervisor.jog_stream_tick()
        baseline = published(harness)
        assert baseline == 1
        assert harness.enabled_flags()['panda1'] is True

        harness.clock.advance(defaults.OPERATOR_LOCK_TTL_S + 0.001)
        successor = harness.lock.claim()
        assert successor is not None
        assert successor.token != first
        # What the pump would have sampled on both sides of the window.
        assert harness.lock.state()['locked'] is True

        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}
        harness.supervisor.jog_stream_tick()
        assert published(harness) == baseline
        assert harness.supervisor.frame()['arms']['panda1']['motion']['enabled'] is False

    def test_reclaim_race_queues_the_controller_side_disable(self, tmp_path):
        """The revoked authorization also disables the arms at the controller."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        calls_before = len(harness.bridge.enable_calls)

        harness.clock.advance(defaults.OPERATOR_LOCK_TTL_S + 0.001)
        assert harness.lock.claim() is not None
        harness.pump(1)

        disables = harness.bridge.enable_calls[calls_before:]
        assert sorted(disables) == [(1, False), (2, False)]
        assert harness.model('panda1').seeded is False

    def test_expiry_alone_still_clears_without_any_reclaim(self, tmp_path):
        """The same revocation runs when the lock merely expires and stays free."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda2')
        harness.clock.advance(defaults.OPERATOR_LOCK_TTL_S + 0.001)

        # Any entry point retires the token; state() is what the pump calls.
        assert harness.lock.state()['locked'] is False
        assert harness.enabled_flags() == {'panda1': False, 'panda2': False}


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
                              'target': [0.0] * defaults.JOINT_COUNT,
                              'message': ENABLE_ENABLED_MESSAGE}
        self.jog_result = {'arm_id': 'panda1', 'target': [0.0] * defaults.JOINT_COUNT,
                           'clamped': [False] * defaults.JOINT_COUNT}
        self.recover_result = {'arm_ids': ['panda1', 'panda2'],
                               'steps': [], 'enabled_after': False}
        #: Set to an Event to hang POST /api/session/recover inside the route.
        self.recover_block = None
        self.recover_entered = threading.Event()

    def _answer(self, result):
        """Return the scripted result, or raise the scripted refusal."""
        if self.error is not None:
            raise self.error
        return dict(result)

    def request_arm_enable(self, arm_id, enabled, operator_lease=None):
        """Record the §6.13 enable and answer with the scripted verdict."""
        self.enable_calls.append((arm_id, enabled))
        return self._answer(self.enable_result)

    def request_arm_jog(self, arm_id, joint_index, direction,
                        operator_lease=None):
        """Record the §6.13 jog and answer with the scripted verdict."""
        self.jog_calls.append((arm_id, joint_index, direction))
        return self._answer(self.jog_result)

    def request_session_recover(self, operator_lease=None):
        """
        Record the §6.13 session recover and answer with the verdict.

        ``recover_block``, when set, holds the handler thread inside the route
        the way a wedged supervisor does -- the shape behind live finding
        V2L-7, where a Recover press produced no ``recovery started`` line at
        all and no answer either.
        """
        self.recover_calls.append('session')
        if self.recover_block is not None:
            self.recover_entered.set()
            self.recover_block.wait(REQUEST_TIMEOUT_S * 3)
        return self._answer(self.recover_result)

    def revoke_operator_authorization(self):
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
        self.lock.set_revocation_hook(
            self.supervisor.revoke_operator_authorization)
        self.broker = Broker()

        self.logs = LogBus()
        self.settings = None
        self.httpd = None
        for _ in range(10):
            settings = make_settings(
                tmp_path / 'rig', bind='127.0.0.1', port=free_port())
            app = App(settings=settings, supervisor=self.supervisor,
                      lock=self.lock, broker=self.broker,
                      static_root=STATIC_ROOT, log_bus=self.logs)
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

#: The three per-arm endpoints that carry the ``{arm_id}`` placeholder.
MOTION_ROUTES = ('enable', 'jog', 'source')


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

    def test_session_recovery_reaches_the_supervisor_once(self, server):
        """Recovery has one session endpoint, never one call per URL arm."""
        token = server.claim()
        response = server.request('POST', '/api/session/recover',
                                  headers={'X-Operator-Token': token})
        assert response.status == 200, response.body
        assert response.json()['arm_ids'] == ['panda1', 'panda2']
        assert server.supervisor.recover_calls == ['session']

    def test_session_recovery_error_preserves_partial_and_rollback_evidence(
            self, server):
        """The failure envelope keeps safe per-step evidence for the operator."""
        token = server.claim()
        steps = [
            {'step': 'error_recovery', 'arm_id': 'panda1', 'ok': False,
             'detail': 'service unavailable'},
            {'step': 'rollback_deactivate', 'controller': IMPEDANCE,
             'ok': True, 'detail': 'inactive'},
        ]
        server.supervisor.error = SessionError(
            'recovery_failed', 'full restore was refused',
            payload={'arm_ids': ['panda1', 'panda2'], 'steps': steps,
                     'enabled_after': False})
        response = server.request('POST', '/api/session/recover',
                                  headers={'X-Operator-Token': token})
        assert response.status == 502, response.body
        body = response.json()
        assert body == {
            'ok': False, 'error': 'recovery_failed',
            'detail': 'full restore was refused',
            'arm_ids': ['panda1', 'panda2'], 'steps': steps,
            'enabled_after': False,
        }
        assert server.supervisor.recover_calls == ['session']

    def test_a_refused_recover_answers_with_something_the_page_can_render(
            self, server):
        """
        REFUSE: the busy-supervisor refusal reaches the page as an envelope.

        This is what an abandoned (never-taken) recover command produces, and
        what ``noticeFromError`` renders in the notice bar before the Recover
        control is re-enabled. The page needs a CODE and a DETAIL; anything
        less and it has nothing to say but "Recovering".
        """
        token = server.claim()
        server.supervisor.error = SessionError(
            'internal_error',
            'the supervisor is busy (a long stop or preflight is in '
            'progress); the command was discarded — retry once the '
            'state settles')
        response = server.request('POST', '/api/session/recover',
                                  headers={'X-Operator-Token': token})
        assert response.status == 500, response.body
        body = response.json()
        assert body['ok'] is False
        assert body['error'] == 'internal_error'
        assert 'retry once the state settles' in body['detail']
        assert 'steps' not in body, 'a refusal must not look like progress'

    def test_a_hanging_recover_route_tells_the_page_nothing_at_all(self, server):
        """
        HANG: a blocked route emits no response and no progress claim.

        The point is negative and it is the whole reason the console bounds
        its own pending state: while the server is wedged inside this route
        there is NOTHING for the page to read, so a page that kept
        "Recovering" up on the strength of the click alone would keep it up
        forever -- exactly what happened live. The route is still serving
        other requests, which is what makes the page's own timeout the right
        guard rather than a broken connection.
        """
        token = server.claim()
        server.supervisor.recover_block = threading.Event()
        outcome = {}

        def send():
            try:
                outcome['response'] = server.request(
                    'POST', '/api/session/recover',
                    headers={'X-Operator-Token': token})
            except Exception as error:      # noqa: BLE001 - reported below
                outcome['error'] = error

        caller = threading.Thread(target=send, daemon=True)
        caller.start()
        assert server.supervisor.recover_entered.wait(REQUEST_TIMEOUT_S)
        time.sleep(0.2)
        assert outcome == {}, 'the hung route answered something'
        # The rest of the surface is alive, so the page keeps receiving
        # frames -- none of which says a recovery is running.
        alive = server.request('POST', '/api/operator/heartbeat',
                               headers={'X-Operator-Token': token})
        assert alive.status == 200, alive.body

        server.supervisor.recover_block.set()
        caller.join(timeout=REQUEST_TIMEOUT_S)
        assert caller.is_alive() is False
        assert outcome.get('response') is not None, outcome
        assert outcome['response'].status == 200, outcome['response'].body

    def test_old_arm_recovery_route_is_not_found(self, server):
        """The removed per-arm route cannot silently retain partial semantics."""
        token = server.claim()
        response = server.request('POST', '/api/arm/panda1/recover',
                                  headers={'X-Operator-Token': token})
        assert_error(response, 'not_found', 404)
        assert server.supervisor.recover_calls == []

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


# ======================================================================
# The per-arm command source
# ======================================================================


class TestCommandSourceSwitch:
    """``POST /api/arm/{id}/source`` and what it does to the jog stream."""

    def test_source_switch_is_allowed_while_the_arm_is_disabled(self, tmp_path):
        """
        The switch does not require an enabled arm.

        The console greys its control out until an arm is enabled; the
        backend must not depend on that, and must not grow a restriction the
        page could then be the only thing enforcing.
        """
        harness = motion_running(tmp_path)
        assert harness.enabled_flags()['panda1'] is False
        assert harness.source('panda1', 'external') == {
            'arm_id': 'panda1', 'source': 'external'}

    def test_the_jog_stream_publishes_nothing_for_an_external_arm(self, tmp_path):
        """
        Switching to External silences the server's own publisher.

        That is what makes every message counted on the target topic the
        operator's own.
        """
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.supervisor.jog_stream_tick()
        baseline = published(harness)
        assert baseline >= 1
        harness.source('panda1', 'external')
        for _ in range(20):
            harness.supervisor.jog_stream_tick()
        assert published(harness) == baseline

    def test_switching_back_to_jog_resumes_the_stream(self, tmp_path):
        """And switching back re-seeds and resumes it."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.source('panda1', 'external')
        assert harness.model('panda1').seeded is False
        harness.source('panda1', 'jog')
        assert harness.model('panda1').seeded is True
        before = published(harness)
        harness.supervisor.jog_stream_tick()
        assert published(harness) == before + 1

    def test_a_source_switch_landing_between_snapshot_and_publish_is_honoured(
            self, tmp_path):
        """
        The re-check immediately before publishing closes the last window.

        The snapshot at the top of the tick closes the common case; a switch
        landing AFTER it must silence that arm on this tick, not the next
        one. panda2's source is flipped from inside panda1's message build --
        after the snapshot named both arms, and before panda2's own publish.
        """
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.enable('panda2')
        baseline = published(harness)
        model = harness.model('panda1')
        original = model.message

        def flip_then_build(*args, **kwargs):
            with harness.supervisor._state_lock:
                harness.supervisor._arm_source['panda2'] = 'external'
            return original(*args, **kwargs)

        model.message = flip_then_build
        harness.supervisor.jog_stream_tick()
        # Exactly one publish: panda1's. panda2 was named by the snapshot and
        # silenced by the re-check.
        assert published(harness) == baseline + 1
        assert harness.bridge.published_targets[-1][0] == 1

    def test_jog_is_refused_while_the_source_is_external(self, tmp_path):
        """
        Silently accepting a jog that publishes nothing is the worse failure.

        The refusal carries the sentence that says what to do about it.
        """
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.source('panda1', 'external')
        with pytest.raises(SessionError) as excinfo:
            harness.jog('panda1', 0, 1)
        assert excinfo.value.code == 'not_motion_mode'
        assert 'switch the source back to Jog' in excinfo.value.detail

    def test_disable_does_not_reset_the_source(self, tmp_path):
        """Disabling an arm is not a source decision."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        harness.source('panda1', 'external')
        harness.enable('panda1', enabled=False)
        assert harness.supervisor._arm_source['panda1'] == 'external'

    def test_an_unknown_source_value_is_refused_with_invalid_source(self, tmp_path):
        """The value is a closed set of two."""
        harness = motion_running(tmp_path)
        with pytest.raises(SessionError) as excinfo:
            harness.source('panda1', 'telepathy')
        assert excinfo.value.code == 'invalid_source'
        assert excinfo.value.detail == (
            "source must be 'jog', 'external' or 'ghost'")

    def test_source_switch_is_refused_outside_motion_mode(self, tmp_path):
        """Simulate and Watch have no motion surface to switch."""
        for mode in ('simulate', 'watch'):
            harness = simple_running(tmp_path / mode, mode)
            with pytest.raises(SessionError) as excinfo:
                harness.source('panda1', 'external')
            assert excinfo.value.code == 'not_motion_mode'

    def test_source_switch_is_refused_while_the_session_is_not_running(
            self, tmp_path):
        """A session that has not opened its command surface refuses."""
        harness = MotionHarness(tmp_path)
        harness.make_ready(controller_name=IMPEDANCE)
        harness.start(arms='both', mode='motion')
        harness.pump()
        assert harness.supervisor.state == 'settling'
        with pytest.raises(SessionError) as excinfo:
            harness.source('panda1', 'external')
        assert excinfo.value.code == 'session_not_running'

    def test_source_switch_is_refused_for_an_arm_not_in_the_session(self, tmp_path):
        """A one-arm session refuses the other arm."""
        harness = motion_running(tmp_path, arms='panda1')
        with pytest.raises(SessionError) as excinfo:
            harness.source('panda2', 'external')
        assert excinfo.value.code == 'arm_not_in_session'

    def test_the_frame_reports_the_source_and_the_rate(self, tmp_path):
        """
        `external_rate_hz` is 0.0 while nothing arrives, and null on Jog.

        That distinction is what tells the console to say "waiting for your
        publisher" rather than showing nothing at all.
        """
        harness = motion_running(tmp_path)
        motion = harness.supervisor.frame()['arms']['panda1']['motion']
        assert motion['source'] == 'jog'
        assert motion['external_rate_hz'] is None
        harness.source('panda1', 'external')
        motion = harness.supervisor.frame()['arms']['panda1']['motion']
        assert motion['source'] == 'external'
        assert motion['external_rate_hz'] == 0.0
        harness.bridge.external_rates['panda1'] = 19.96
        assert harness.supervisor.frame()['arms']['panda1']['motion'][
            'external_rate_hz'] == 20.0


class TestJogResponseShape:
    """The jog response is what drives the console's clamp feedback."""

    def test_the_jog_response_still_carries_the_clamped_mask(self, tmp_path):
        """
        Exactly three keys, and `clamped` is a seven-element boolean mask.

        The console's clamp flash is its only consumer, and no other document
        states this response shape -- so this test is its only guard.
        """
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.set_joints(dual_joint_state(pose_1=NEAR_UPPER_POSE))
        harness.enable('panda1')
        result = harness.jog('panda1', 3, 1)
        assert set(result) == {'arm_id', 'target', 'clamped'}
        assert result['arm_id'] == 'panda1'
        assert len(result['clamped']) == defaults.JOINT_COUNT
        assert all(isinstance(flag, bool) for flag in result['clamped'])
        assert result['clamped'] == [
            False, False, False, True, False, False, False]

    def test_an_unclamped_jog_reports_an_all_false_mask(self, tmp_path):
        """The mask is present whether or not anything was clamped."""
        harness = motion_running(tmp_path)
        harness.claim_lock()
        harness.enable('panda1')
        result = harness.jog('panda1', 0, 1)
        assert result['clamped'] == [False] * defaults.JOINT_COUNT


def _app_js():
    """Return the shipped application JavaScript."""
    with open(os.path.join(STATIC_ROOT, 'app.js')) as handle:
        return handle.read()


def _js_function(source, name):
    """
    Return the body of one top-level ``function name(...) {...}`` from app.js.

    CI has no browser and no Node (and this package ships without either), so
    the console's decisions cannot be EXECUTED here. What can be checked is
    which inputs a decision is allowed to read, and that is exactly the
    property V2L-7 turned on: "Recovering" is a claim about the server, so the
    function that decides it must not be able to see this page's own pending
    state. Brace-matched rather than regex-matched so a nested block cannot
    truncate the body being examined.
    """
    marker = '\nfunction {}('.format(name)
    start = source.index(marker)
    opening = source.index('{', source.index(')', start))
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == '{':
            depth += 1
        elif source[index] == '}':
            depth -= 1
            if depth == 0:
                return source[opening:index + 1]
    raise AssertionError('function {} is not brace-balanced'.format(name))


@pytest.mark.skipif(
    'Recover full session' in _app_js(),
    reason='the frontend rewrite has not landed yet; the v1 app.js is still '
           'in the tree. This class arms itself on that merge -- confirm it '
           'is RUNNING, not skipping, once it is in.')
class TestConsoleContract:
    """Pin the v2 console's server-facing surface."""

    def test_one_session_endpoint_replaces_per_arm_recovery(self):
        """Recovery is one session-wide action, not a per-arm one."""
        source = _app_js()
        assert source.count("'/api/session/recover'") == 1
        assert '/api/arm/${armId}/recover' not in source

    def test_the_page_streams_and_can_take_over(self):
        """One EventSource, and the takeover the operator badge offers."""
        source = _app_js()
        assert "EventSource('/api/state/stream')" in source
        assert "'/api/operator/takeover'" in source

    def test_no_innerhtml_and_no_removed_mechanisms(self):
        # TOKEN checks, never a bare 'gains'. Two independent reasons: the
        # English word "against" contains that substring, and `k_gains` /
        # `d_gains` are legitimate GET /api/config keys the profile popover
        # renders. Only the DELETED gains-upload mechanism is forbidden here.
        source = _app_js()
        assert 'innerHTML' not in source
        assert 'gains_sha256' not in source
        assert 'max_gains_bytes' not in source
        assert 'gains_upload' not in source
        assert "'/api/gains'" not in source
        assert 'hold_controller' not in source

    def test_the_release_path_and_the_advisory_survive(self):
        # Release-on-unload must use fetch(..., {keepalive: true}) with the
        # X-Operator-Token header -- sendBeacon cannot set a header. And the
        # persistent stop advisory IS rendered. This is the only place either
        # decision is checked once the frontend's scratch verifier is gone.
        source = _app_js()
        assert 'sendBeacon' not in source
        assert 'keepalive' in source
        assert 'session.advisory' in source

    def test_no_notes_tree_reference(self):
        """The needle is assembled at runtime so it is not itself a match."""
        assert ('multipanda_ros2' + '_jazzy_notes') not in _app_js()

    def test_the_recovering_card_is_keyed_on_server_evidence_only(self):
        """
        V2L-7: "Recovering" may be rendered from the FRAME and nothing else.

        Live, a second Recover press produced no ``recovery started`` line
        server-side at all, and the page nonetheless showed "Recovering"
        indefinitely. The decision therefore lives in one named function that
        is not allowed to see this page's own click state; mutating it to read
        ``ui.pending.recover`` fails here.
        """
        source = _app_js()
        body = _js_function(source, 'recoveryInProgress')
        # The whole point: this decision cannot see the page's own state.
        assert 'ui.' not in body, body
        assert 'net.' not in body, body
        assert "session.state !== 'fault'" in body, body
        assert 'isRecoverySteps(session.steps)' in body, body
        # A checklist whose every step is done is a FINISHED recovery, and the
        # function must distinguish that from a running one.
        assert "step.status === 'active'" in body, body
        # The stage builds that card from this function and from nothing else.
        assert 'if (recoveryInProgress(frame)) {' in source
        assert source.count("buildChecklist(frame, 'rid', 'Recovering')") == 1
        assert "session.state === 'fault' && isRecoverySteps(" not in source

    def test_the_recover_pending_state_is_bounded_and_says_why_it_ended(self):
        """
        A refused, failed or unanswered Recover releases the control.

        The pending state may outlive the request only while
        ``recoveryInProgress`` holds; otherwise the deadline expires, the
        control re-enables and the notice bar says so. The 1 Hz tick is a
        resolution path too, so a page that has stopped receiving frames still
        recovers its UI.
        """
        source = _app_js()
        assert 'RECOVER_PENDING_MS' in source
        body = _js_function(source, 'resolveRecoverPending')
        assert 'recoveryInProgress(net.frame)' in body, body
        assert 'ui.recoverUntil' in body, body
        assert 'delete ui.pending.recover;' in body, body
        assert 'notice(' in body, body
        assert _js_function(source, 'tick').count('resolveRecoverPending()') == 1
        assert _js_function(source, 'onFrame').count('resolveRecoverPending()') == 1

    def test_the_stopped_card_claims_a_recording_only_when_one_sealed(self):
        """
        V2L-2's last corner: the card must use the hint's evidence, not policy.

        ``recording.disabled`` answers "is recording switched off in the
        config?", which is a different question from "did THIS session save
        anything?" -- and a start refused at preflight adopts no recorder at
        all.
        """
        source = _app_js()
        assert 'session.recording_sealed === true' in source
        assert "recording.disabled === true\n      ? 'Session ended.'" not in source
        assert source.count("'Session ended. The recording was saved.'") == 1


class TestApplyGuards:
    """
    T40. The Apply handler runs the same guards, in the same places.

    Apply is a third per-arm source on the arm card, under ALL the existing
    guardrails. That is a claim about where the guards sit in the handler, so
    it is asserted by driving each of them to failure and reading the code
    that comes back -- the same shape the jog handler is held to.
    """

    #: One comfortable travel from the rig's own in-fence pose.
    GOAL = tuple(value + (0.2 if index == 3 else 0.0)
                 for index, value in enumerate(IN_FENCE_POSE))

    def ready(self, tmp_path, **kwargs):
        """Return a running Motion rig with panda1 enabled and sourced ghost."""
        harness = motion_running(
            tmp_path, checker=checker_holding(FakeCellModel()), **kwargs)
        harness.enable('panda1')
        harness.source('panda1', 'ghost')
        return harness

    def test_apply_outside_a_running_session_is_refused(self, tmp_path):
        """``_motion_guards`` first: no session, no Apply."""
        harness = self.ready(tmp_path)
        harness.force_state('settling')
        with pytest.raises(SessionError) as excinfo:
            harness.apply('panda1', self.GOAL)
        assert excinfo.value.code == 'session_not_running'

    def test_apply_on_an_arm_outside_the_session_is_refused(self, tmp_path):
        """``_motion_guards`` again: the arm set is the session's."""
        harness = self.ready(tmp_path, arms='panda1')
        with pytest.raises(SessionError) as excinfo:
            harness.apply('panda2', self.GOAL)
        assert excinfo.value.code == 'arm_not_in_session'

    def test_apply_on_a_faulted_session_is_refused(self, tmp_path):
        """``_motion_guards``: a faulted session commands nothing."""
        harness = self.ready(tmp_path)
        harness.launch_child.die(returncode=1)
        harness.pump(1)
        assert harness.supervisor.state == 'fault'
        with pytest.raises(SessionError) as excinfo:
            harness.apply('panda1', self.GOAL)
        assert excinfo.value.code == 'session_faulted'

    def test_a_fault_observed_between_the_queue_and_the_handler_refuses(
            self, tmp_path):
        """
        ``_guard_no_fresh_fault``, in the position the jog handler puts it.

        Commands are processed before the ordinary running-state poll, so a
        fault callback can land after the previous tick and before this queued
        Apply. Its cache must be re-evaluated at the final local gate.
        """
        harness = self.ready(tmp_path)
        harness.bridge.hardware = dict(harness.bridge.hardware,
                                       lifecycle_id=2,
                                       lifecycle_label='inactive')
        with pytest.raises(SessionError) as excinfo:
            harness.apply('panda1', self.GOAL)
        assert excinfo.value.code == 'session_faulted'
        assert harness.supervisor._arm_travel['panda1'] is None

    def test_apply_needs_the_current_operator_lease(self, tmp_path):
        """
        A start creates live control, so it re-validates on the supervisor.

        The route's token check at the HTTP edge is not enough: control can
        change between the queue and the handler, and a start taken under a
        lease that is no longer current would be authority nobody granted.
        """
        harness = self.ready(tmp_path)
        lease = harness.operator_lease()
        harness.lock.release(harness.token)
        harness.token = None
        harness.claim_lock()
        with pytest.raises(SessionError) as excinfo:
            harness.drive(lambda: harness.supervisor.request_arm_apply(
                'panda1', 'start', list(self.GOAL), operator_lease=lease))
        assert excinfo.value.code == 'operator_token_invalid'

    def test_a_cancel_never_needs_a_current_lease(self, tmp_path):
        """
        A stop that can be refused for want of authority is not a stop.

        This is ``enable(False)``'s own precedent: turning authority OFF must
        never be refused because control moved while the arm was moving.
        """
        harness = self.ready(tmp_path)
        harness.apply('panda1', self.GOAL)
        harness.lock.release(harness.token)
        harness.token = None
        result = harness.supervisor.request_arm_apply('panda1', 'cancel')
        assert result['arm_id'] == 'panda1'
        assert harness.supervisor._arm_travel['panda1'] is None

    def test_the_stream_publishes_a_ghost_arm_exactly_as_it_publishes_a_jog(
            self, tmp_path):
        """
        The message shape does not change with the source.

        Every byte reaching the controller under source ghost is built by the
        same function, from the same held target, under the same gates.
        """
        harness = self.ready(tmp_path)
        harness.apply('panda1', self.GOAL)
        before = published(harness)
        harness.supervisor.jog_stream_tick()
        assert published(harness) == before + 1
        slot, message = harness.bridge.published_targets[-1]
        assert slot == 1
        assert list(message.joint_names) == list(
            health.joint_names_for('panda1'))
        assert len(message.points) == 1
        assert list(message.points[0].velocities) == []
