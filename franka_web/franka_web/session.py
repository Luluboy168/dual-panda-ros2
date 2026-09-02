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
The session state machine: the single source of truth for the web server.

``SessionSupervisor`` owns the plan's §3.4 machine::

    stopped -> preflight -> starting -> (settling) -> running
                                      -> (fault) -> stopping -> stopped

Invariants (asserted here and in ``test_session_state_machine.py``):

* State is mutated only on the supervisor (main) thread, inside :meth:`tick`.
  HTTP handlers enqueue commands via :meth:`request_start` / :meth:`request_stop`
  and wait; they never transition state themselves.
* Entering ``fault`` or ``stopping`` unconditionally forces every arm's enable
  flag off before anything else happens (Stage 1 carries the flags; Stage 2
  attaches the jog stream to them).
* Long operations (preflight, readiness polling, the stop ladder) run outside
  the state lock so frame snapshots and HTTP reads never stall.
* On ``stopping``, the recorder is sealed (SIGINT first, its own generous
  budget) BEFORE the launch child is touched, so the shutdown itself is
  recorded.
* Motion is ONE GO. There is no Watch->Motion attestation: the pre-activation
  baseline is captured IN SESSION, during ``starting``, with the impedance
  controller PROVEN inactive. The reviewed guarded-motion launch activates
  that controller with ``spawner --switch-asap`` before
  ``joint_state_broadcaster`` publishes anything, so no such window exists to
  observe; the server MAKES one (``_restage_activation``) with the same
  ``switch_controller`` machinery Recover uses -- pause, measure, hand the
  arms back -- and gates its own re-activation. No operator-commandable
  torque exists until a settling gate armed against a pose measured with the
  controller inactive reports ready.
* Every ``switch_controller`` call this server issues is spaced by at least
  ``SWITCH_DWELL_S``, restage and Recover alike, through
  ``_switch_activate``/``_switch_deactivate``. Mode switches in tight
  succession stall the driver's real-time cycle into a fail-safe stop; see
  ``SWITCH_DWELL_S`` for the live evidence.
"""

from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
import os
import queue
import signal
import threading
import time

from franka_web import defaults, health, logbus, travel
from franka_web.faults import (
    classify_fault, FaultEngine, FaultReason, FaultSnapshot)
from franka_web.gains import ProfileStoreError
from franka_web.jog import JogError, JogTargetModel
from franka_web.launcher import ChildProcess, LauncherError
from franka_web.preflight import run_preflight
from franka_web.profiles import argv_for, ProfileError, PROFILES
from franka_web.recording import (
    RecordingError, RecordingSupervisor, session_name, topics_for)
from franka_web.settling import ActivationSettlingGate
from franka_web.workspace import NOTE_PACKAGE_ABSENT, NOTE_PROFILE_ARM_MISMATCH

STATES = ('stopped', 'preflight', 'starting', 'settling', 'running', 'fault', 'stopping')

_VALID_ARMS = ('panda1', 'panda2', 'both')
_VALID_MODES = ('simulate', 'watch', 'motion')

#: Minimum spacing between any two ``switch_controller`` calls this server
#: issues, in seconds.
#:
#: A REVIEWED CONSTANT, deliberately not a configuration key: it is a property
#: of the real-time driver, not of a deployment, and an operator who lowered it
#: would be turning a robot-side fail-safe back on.
#:
#: Live evidence, 2026-09-01/02, dual Panda on real hardware. The Motion start
#: restage issued deactivate -> capture -> reactivate inside ~110 ms; ~90 ms
#: after the VERIFIED reactivation ``FrankaMultiHardwareInterface``'s read
#: cycle errored, the driver took its fail-safe stop and the session faulted --
#: twice, on two consecutive starts. The same class shows a second face on the
#: write side: a Recover issued after a stop-press produced "Error while
#: attempting mode switch when deactivating controllers in write cycle!" with a
#: 205 ms / 206-missed-cycle control-loop stall, and a stalled loop goes on to
#: trip robot-side communication ("communication with the robot failed" 16 s
#: later). Recover's historic ~1 s spacing has always survived on this
#: hardware; the restage's 110 ms did not. 0.75 s sits between the two with
#: margin on the side that is known to work.
SWITCH_DWELL_S = 0.75

#: Hard cap on the wait slices one dwell may take, so a wait callable that
#: fails to advance the clock can never hold the supervisor thread. The dwell
#: is additionally bounded by SWITCH_DWELL_S itself.
_SWITCH_DWELL_MAX_SLICES = 64

# Worst-case dual impedance recovery is 149 s of bounded service time with the
# bridge's 5 s bound: fresh controller query (5), two pre-disables (10),
# pre-deactivate and verify (10), both ErrorRecovery calls (10), hardware
# query/set/verify (15), controller query plus six activate/verify pairs (65),
# fresh state wait (5), final hardware/controller queries (10), and fail-closed
# rollback (19). On top of that the reviewed switch spacing costs up to
# SWITCH_DWELL_S per switch_controller call, and a dual recovery issues seven
# of them (one pre-deactivate, five broadcasters, the motion controller), so
# 149 + 7 * 0.75 = 154.25 s. There are no post-reactivation disables:
# onActivate() already left every inbox disabled, so commanding it again would
# only rebase the captured activation target. The public wait includes
# scheduling margin. Once a command is taken, _submit() waits for its actual
# verdict rather than ever reporting a timeout while recovery is still changing
# controller-manager state.
RECOVERY_FRESH_STATE_TIMEOUT_S = 5.0
RECOVERY_REQUEST_TIMEOUT_S = 200.0


class SessionError(Exception):
    """A refused session command, carrying a stable §6.14 error code."""

    def __init__(self, code, detail, payload=None):
        """Store the closed-set code, safe detail, and optional safe evidence."""
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.payload = dict(payload or {})


@dataclass(frozen=True)
class SessionRequest:
    """A validated-shape start request (content validated by the supervisor)."""

    arms: str
    mode: str
    controller_param_file: str = None


@dataclass
class _Command:
    """
    One queued operator command with a synchronous reply slot.

    A command is answered exactly once, OR abandoned exactly once. The
    waiter that times out calls :meth:`abandon`; a command that was
    abandoned is skipped by the supervisor, never executed. Without this,
    a start that the operator was told had failed would still bring a
    robot stack up tens of seconds later (review finding R1).
    """

    kind: str
    request: SessionRequest = None
    operator_lease: object = None
    done: threading.Event = field(default_factory=threading.Event)
    result: dict = None
    error: SessionError = None
    _claim_lock: threading.Lock = field(default_factory=threading.Lock)
    _taken: bool = False
    _abandoned: bool = False

    def try_begin(self):
        """Claim the command for execution; False if the waiter abandoned it."""
        with self._claim_lock:
            if self._abandoned:
                return False
            self._taken = True
            return True

    def abandon(self):
        """Abandon an unanswered command; False if execution already began."""
        with self._claim_lock:
            if self._taken:
                return False
            self._abandoned = True
            return True

    def resolve(self, result):
        """Answer the waiting HTTP thread with a success payload."""
        self.result = result
        self.done.set()

    def reject(self, error):
        """Answer the waiting HTTP thread with a SessionError."""
        self.error = error
        self.done.set()


def rfc3339(moment=None):
    """Render a UTC timestamp as RFC 3339 with microseconds and ``Z``."""
    moment = moment or datetime.now(timezone.utc)
    return moment.strftime('%Y-%m-%dT%H:%M:%S.%f') + 'Z'


def _joints_outside_fence(positions, lower, upper):
    """Name the joints (1-based labels) whose position leaves the fence."""
    outside = []
    for index, position in enumerate(positions):
        if position is None or not lower[index] <= position <= upper[index]:
            outside.append('joint{}'.format(index + 1))
    return outside


def expected_state_broadcasters(arm_ids, arm_mode):
    """
    Name the robot-state broadcasters a Watch session must see active.

    Dual mode prefixes per arm; one-arm mode registers the unprefixed
    instance name (plan §0.3).
    """
    if arm_mode == 'single':
        return ('franka_robot_state_broadcaster',)
    return tuple('franka_{}_robot_state_broadcaster'.format(a) for a in arm_ids)


def expected_model_broadcasters(arm_ids, arm_mode):
    """Name every launch-owned Franka model broadcaster for the topology."""
    if arm_mode == 'single':
        return ('franka_robot_model_broadcaster',)
    return tuple('franka_{}_robot_model_broadcaster'.format(a) for a in arm_ids)


def expected_broadcasters(arm_ids, arm_mode):
    """Return launch-owned broadcasters in the proven safe restore order."""
    return (('joint_state_broadcaster',)
            + expected_state_broadcasters(arm_ids, arm_mode)
            + expected_model_broadcasters(arm_ids, arm_mode))


#: Display labels for the START checklist.
_STEP_LABELS = {
    'preflight': 'Preflight',
    'health': 'Health check',
    'stack_ready': 'Stack ready',
    'controller_pause': 'Controller paused',
    'baseline': 'Baseline captured',
    'controller': 'Controller active',
    'settling': 'Settling check',
}

#: Display labels for the RECOVERY checklist. Never merged with
#: _STEP_LABELS: `controller` deliberately reads differently in each list.
_RECOVERY_STEP_LABELS = {
    'controller': 'Restart controller',
    'verify': 'Verify fresh data',
}

#: Step id -> the persistent next-step sentence. The hint is NOT a transform
#: of the display label -- three of these are not even close ('Connect panda1'
#: against 'Connecting to panda1...'), so `_hint` reads THIS table, keyed by
#: step id, and never the label. The ellipsis is one character, U+2026.
_STEP_HINTS = {
    'preflight': 'Preflight\u2026',
    'connect': 'Connecting to {arm}\u2026',
    'health': 'Health check\u2026',
    'stack_ready': 'Waiting for the stack\u2026',
    'controller_pause': 'Pausing the controller\u2026',
    'baseline': 'Capturing baseline\u2026',
    'controller': 'Activating controller\u2026',
    'settling': 'Settling check\u2026',
}

_RECOVERY_STEP_HINTS = {
    'reconnect': 'Reconnecting to {arm}\u2026',
    'controller': 'Restarting controller\u2026',
    'verify': 'Verifying fresh data\u2026',
}

#: The three command sources an arm can take its targets from. `ghost` is
#: `jog`'s sibling, not `external`'s: both are server-mediated and both go out
#: through `jog_stream_tick`. The difference is only HOW the held target is
#: allowed to change -- one fixed step on one joint per operator press, or one
#: bounded interpolation step on all seven per tick after a whole-path check.
_SOURCES = ('jog', 'external', 'ghost')

#: The sources `jog_stream_tick` publishes for. `external` is deliberately
#: absent: an externally sourced arm is silent HERE, which is what makes every
#: message counted on its target topic the operator's own.
_SERVER_SOURCES = ('jog', 'ghost')

_HINT_IDLE = ('Pick arms and press Start \u2014 or choose Simulate to try the '
              'console without robots.')
_HINT_ENDED_RECORDED = ('Session ended. Recording saved. Start a new session '
                        'anytime.')
_HINT_ENDED = 'Session ended. Start a new session anytime.'
_HINT_STOPPING = 'Stopping \u2014 sealing the recording.'
_HINT_FAULT_RECLAIM = 'Press Reclaim to take control back, then Recover.'
_HINT_FAULT = 'Check that nobody pressed a stop, then press Recover.'
_HINT_WATCH = ('Observing only \u2014 motion is impossible in Watch. The arm '
               'can be moved by hand.')
_HINT_OTHER_OPERATOR = ('Another program holds control. Take over to command '
                        'the arms.')
_HINT_NO_ENABLE = 'Enable an arm to allow commands.'
_HINT_WAITING = 'Waiting for your publisher on {topic} \u2014 {rate}'
_HINT_RECEIVING = ('Receiving {rate} from your node. The watchdog freezes the '
                   'arm if the stream stops.')
_HINT_JOG = ('Jog with the \u2212 / + buttons, or switch the source to '
             'External to use your own ROS 2 node.')
_HINT_APPLYING = ('Applying a pose to {arm} \u2014 press Cancel to stop it '
                  'where it is.')
_HINT_GHOST = ('Drag the ghost in the Scene panel, then press Apply on the '
               'card.')

#: What to do about a preflight that could not identify libfranka, and the
#: words a failing check uses when that is what went wrong. The hint is only
#: ever added when `directories.franka_dir` is actually unset.
_PREFLIGHT_FRANKA_DIR_HINT = (
    'Set directories.franka_dir in ~/.config/franka_web/config.yaml to the '
    'libfranka build directory (the same path colcon was given as '
    '-DFranka_DIR) so the check can identify libfranka.')
_PREFLIGHT_FRANKA_DIR_MARKERS = ('franka_dir', 'libfranka')

#: The rate at which an external publisher satisfies the controller watchdog.
_EXTERNAL_RATE_FLOOR_HZ = 10.0

#: The External panel's copyable message template (the Copy button hands this
#: straight to an operator's editor, so the bytes are pinned).
_TEMPLATE_HEAD = (
    '# trajectory_msgs/msg/JointTrajectory \u2014 publish at 10 Hz or more\n'
    'header:\n'
    '  stamp: now\n'
    'joint_names: [{n0}, {n1}, {n2}, {n3},\n'
    '              {n4}, {n5}, {n6}]\n'
    'points:\n'
    '- positions: [{positions}]{comment}\n'
    '  time_from_start: {{sec: 0, nanosec: 0}}\n'
)
_READY_COMMENT = '   # rad \u2014 one point per message'
_UNREADY_COMMENT = '  # rad \u2014 replace with your target'
_UNREADY_POSITIONS = '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0'

#: (gate metric, SettlingConfig attribute, config key, unit, direction) for
#: the four settling families the teaching message can name.
_SETTLING_KEYS = (
    ('max_abs_delta_rad', 'drift_limit_rad', 'settling.drift_limit_deg',
     '\u00b0', 'below', 'moved'),
    ('position_span_rad', 'span_limit_rad', 'settling.span_limit_deg',
     '\u00b0', 'below', 'drifted over a window of'),
    ('max_abs_velocity_rad_s', 'velocity_limit_rad_s',
     'settling.velocity_limit_deg_s', '\u00b0/s', 'below',
     'was still moving at'),
    ('_fence_margin', 'fence_margin_rad', 'settling.fence_margin_deg',
     '\u00b0', 'above', 'came within'),
)


class SessionSupervisor:
    """
    Owns the session lifecycle, the child processes, and the state frame.

    Every collaborator is injected so the whole machine is unit-testable
    with fakes; production wiring lives in ``server.py``.
    """

    def __init__(self, settings, bridge, lock, broker, *,
                 spawn=ChildProcess.spawn,
                 recording_factory=None,
                 preflight_runner=run_preflight,
                 fault_engine=None,
                 argv_builder=argv_for,
                 session_namer=session_name,
                 profile_store=None,
                 log_bus=None,
                 checker=None,
                 monotonic=time.monotonic,
                 utcnow=None,
                 recovery_wait=None):
        """Wire the supervisor; nothing is started until :meth:`tick` runs."""
        self._profile_store = profile_store
        # The SAME WorkspaceChecker instance the ghost service holds, so the
        # model that approves an Apply is the model that tinted the ghost --
        # one object, one cache, one loaded model. None means Apply is
        # unavailable, which is fail-closed: the ghost degrades visibly when
        # the checker cannot run because the ghost commands nothing, and Apply
        # refuses because Apply commands everything.
        self._checker = checker
        # Never None: the frame's `logs` block must always exist, and no call
        # site can be left without a bus by forgetting the keyword.
        self._logs = log_bus or logbus.LogBus()
        self._settings = settings
        self._bridge = bridge
        self._lock_service = lock
        self._broker = broker
        self._spawn = spawn
        self._recording_factory = (
            recording_factory
            or (lambda: RecordingSupervisor(settings, spawn, monotonic=monotonic,
                                            log_bus=self._logs)))
        self._preflight_runner = preflight_runner
        self._fault_engine = fault_engine or FaultEngine(monotonic=monotonic)
        self._argv_builder = argv_builder
        self._session_namer = session_namer
        self._monotonic = monotonic
        self._utcnow = utcnow or (lambda: datetime.now(timezone.utc))
        self._recovery_wait = recovery_wait or self._wait_wall_time
        # The reviewed switch spacing waits on the SAME bounded-wait primitive
        # the recovery sample waits use, but on its own seam: a sample wait
        # means "let the publishers produce another cycle" and carries a test
        # budget, while a dwell means "let the real-time loop settle" and must
        # not spend that budget. Production keeps the wall-clock sleep; a
        # fake-clock rig replaces this attribute after construction (there is
        # no constructor keyword on purpose, so the same rig can drive an
        # unpatched build for a fail-before/pass-after comparison).
        self._switch_dwell_wait = self._wait_wall_time
        # Monotonic time of the last switch_controller call this server
        # issued, or None when it has issued none. Never reset per session:
        # the driver does not forget a mode switch because a session ended.
        self._last_switch_mono = None

        self._state_lock = threading.RLock()
        self._commands = queue.Queue()
        self._state = 'stopped'
        self._started_mono = monotonic()
        self._session = None            # dict of session-block fields
        self._launch = None             # ChildProcess
        self._recording = None          # RecordingSupervisor
        self._preflight_result = None
        self._fault_since = None
        self._fault_reasons = ()
        self._fault_recoverable = False
        self._arm_enabled = {}
        self._starting_deadline = None
        self._preflight_pending = False
        self._profile_record = None     # StoredProfile for the motion session
        self._jog_models = {}           # arm_id -> JogTargetModel
        self._arm_slots = {}            # arm_id -> controller slot (1-based)
        # arm_id -> 'jog' | 'external' | 'ghost'. The KEY SET is created once
        # at _accept_start and never grows or shrinks during a session, which
        # is what makes the lock-free per-key reset in the revocation hook
        # legal.
        self._arm_source = {}
        # The Apply travel, and the two integers that make stopping one
        # lock-free. The same fixed-key-set rule as _arm_source applies to all
        # four, and for the same reason.
        #
        # _arm_travel is the ONE field whose new value is computed from its old
        # one (`plan.advanced()`), so it is the one that needs more than
        # per-key assignment: the tick's store-back is a compare-and-set on
        # object identity, or a lock-free clear landing inside the tick's
        # read-modify-write would be overwritten and resurrect a travel that
        # authority has already been withdrawn from.
        self._arm_travel = {}       # arm_id -> travel.TravelPlan | None
        # Bumped on EVERY enable and EVERY disable, including the compensating
        # ones: a travel must never survive a disable/enable cycle, and
        # re-enabling is a new authorization rather than a continuation.
        self._enable_epoch = {}     # arm_id -> int
        # Bumped by whoever stops a travel: the HTTP thread on Cancel, the
        # revocation hook, the source switch, fault and teardown. Plain integer
        # assignment, which is what lets Cancel run off the command queue.
        self._cancel_gen = {}       # arm_id -> int
        self._last_advance = {}     # arm_id -> mono_s of the last advance
        self._steps = []                # the frame's session.steps array
        self._baseline_captured = False
        self._targets_published = {}    # arm_id -> count
        self._last_publish_mono = {}    # arm_id -> mono_s
        self._recover_succeeded = False
        self._activation_baseline = None
        self._activation_gate = None
        self._settling_target_counts = {}
        # Whether THIS session actually sealed a recording, which is not the
        # same question as whether recording is enabled: a start refused at
        # preflight never adopted a recorder and must not claim one.
        self._recording_sealed = False

        # Register LAST: the hook reads the fields above, and the lock may
        # call it the moment it is registered from another thread. Wiring it
        # here rather than in server.py is deliberate -- the authorization
        # invariant must not depend on each construction site (production,
        # the test rigs) remembering to connect it (finding F-0).
        register = getattr(lock, 'set_revocation_hook', None)
        if register is not None:
            register(self.revoke_operator_authorization)

    # ------------------------------------------------------------------
    # HTTP-thread surface (enqueue + wait; never mutates state directly)
    # ------------------------------------------------------------------

    def request_start(self, request, operator_lease=None, timeout_s=5.0):
        """Ask the supervisor to start a session; blocks for the verdict."""
        return self._submit(
            _Command(kind='start', request=request, operator_lease=operator_lease),
            timeout_s)

    def request_stop(self, timeout_s=5.0):
        """Ask the supervisor to stop the session; blocks for the verdict."""
        return self._submit(_Command(kind='stop'), timeout_s)

    def request_arm_enable(self, arm_id, enabled, operator_lease=None,
                           timeout_s=10.0):
        """Toggle one arm's enable (§6.13); blocks for the verdict."""
        return self._submit(
            _Command(kind='enable', request=(arm_id, bool(enabled)),
                     operator_lease=operator_lease), timeout_s)

    def request_arm_jog(self, arm_id, joint_index, direction,
                        operator_lease=None, timeout_s=5.0):
        """Jog one joint by one fixed step (§6.13); blocks for the verdict."""
        return self._submit(
            _Command(kind='jog', request=(arm_id, joint_index, direction),
                     operator_lease=operator_lease), timeout_s)

    def request_arm_source(self, arm_id, source, operator_lease=None,
                           timeout_s=5.0):
        """Switch one arm between the jog, external and ghost sources."""
        return self._submit(
            _Command(kind='source', request=(arm_id, source),
                     operator_lease=operator_lease), timeout_s)

    def request_arm_apply(self, arm_id, action, positions=None,
                          operator_lease=None, timeout_s=5.0):
        """
        Start one ghost travel (queued), or stop one (immediately).

        The split is the one place G3 departs from this server's "every
        operator command is a ``_Command``" shape, and it is deliberate.
        :meth:`_submit` ABANDONS a command the supervisor never reached, and
        the supervisor genuinely blocks -- up to
        ``defaults.SERVICE_CALL_TIMEOUT_S`` inside one ``call_enable``, far
        longer inside a recovery. A queued Cancel would therefore answer
        ``internal_error`` after five seconds having never run, while the arm
        kept travelling. Before G3 a blocked supervisor left a jogged arm
        static; a travelling arm is not static, so the stop for the motion this
        server started comes off the queue.
        """
        if action == 'cancel':
            return self._cancel_travel_now(arm_id)
        return self._submit(
            _Command(kind='apply', request=(arm_id, action, positions),
                     operator_lease=operator_lease), timeout_s)

    def _cancel_travel_now(self, arm_id):
        """
        Stop one travel from the HTTP request thread, bounded by one tick.

        Runs on the request thread on purpose (see :meth:`request_arm_apply`).
        The bound it holds: *from the moment the POST body is parsed, the
        on-wire target stops changing within one stream period -- 50 ms
        nominal, 100 ms worst case -- and it does so regardless of what the
        supervisor thread is doing.*

        Contract, because of where it runs: take no lock but ``_state_lock``,
        hold it for assignments only, call nothing that can block, and emit to
        the log bus AFTER releasing it. The frame builder already reads this
        state under the same lock from whichever thread calls :meth:`frame`, so
        this adds no new sharing, and no critical section in this file holds
        ``_state_lock`` across a blocking call.

        Idempotent, and it never fails: cancelling an idle arm answers
        ``was_travelling: False``. A stop control that can return an error is a
        stop control an operator learns to press twice and then distrust.
        """
        generation = self._cancel_gen.get(arm_id)
        if generation is None:
            # Not a key of this session. Refuse WITHOUT creating one: the fixed
            # key set is the whole reason the lock-free writes are legal, and a
            # cancel for an unknown arm must not be the thing that grows it.
            raise SessionError('arm_not_in_session',
                               '{} is not part of this session'.format(arm_id))
        # First, outside the lock: a tick already inside its critical section
        # stops at the generation gate on its very next pass, whatever else
        # happens below.
        self._cancel_gen[arm_id] = generation + 1
        with self._state_lock:
            plan = self._arm_travel.get(arm_id)
            if plan is not None:
                self._arm_travel[arm_id] = None
            model = self._jog_models.get(arm_id)
            stopped_at = (list(model.target)
                          if (plan is not None and model is not None
                              and model.seeded) else None)
        if plan is None:
            return {'arm_id': arm_id, 'action': 'cancel',
                    'was_travelling': False, 'fraction': None,
                    'stopped_at': None}
        self._logs.emit('info', '{} apply cancelled at {:.0%}'.format(
            arm_id, plan.fraction))
        return {'arm_id': arm_id, 'action': 'cancel', 'was_travelling': True,
                'fraction': round(plan.fraction, 4), 'stopped_at': stopped_at}

    def _clear_travel_locked(self, arm_id, plan, reason=None):
        """
        Clear one travel; the CALLER already holds ``_state_lock``.

        Clears only if the plan it was handed is still the one in the dict
        (section 3.1 gate 8), bumps the cancel generation so even a resurrected
        plan object can never advance, and RETURNS the sentence for the caller
        to hand the log bus after releasing the lock -- ``LogBus.emit`` takes
        its own lock, and holding two to write a sentence would be a new lock
        order for no reason.
        """
        if self._arm_travel.get(arm_id) is not plan:
            return None
        self._arm_travel[arm_id] = None
        self._cancel_gen[arm_id] = self._cancel_gen.get(arm_id, 0) + 1
        return reason

    def _clear_travel(self, arm_id):
        """Clear one arm's travel and bump its generation, taking the lock."""
        with self._state_lock:
            self._arm_travel[arm_id] = None
            self._cancel_gen[arm_id] = self._cancel_gen.get(arm_id, 0) + 1

    def _clear_every_travel_locked(self):
        """
        Clear every travel; the CALLER already holds ``_state_lock``.

        Used by fault entry and teardown, beside the enables and the sources.
        """
        for arm_id in self._arm_travel:
            self._arm_travel[arm_id] = None
        for arm_id in self._cancel_gen:
            self._cancel_gen[arm_id] = self._cancel_gen[arm_id] + 1

    def _clear_every_travel_lockfree(self):
        """
        Clear every travel without taking any supervisor lock.

        Runs inside ``OperatorLock``'s own mutex (the revocation hook), beside
        :meth:`_force_sources_jog_lockfree` and for the same reason. Both are
        plain per-key assignments into dicts whose key sets are fixed for the
        life of the session.

        The generation bump is the one that matters: the store alone can be
        undone by a tick's store-back, and the tick's compare-and-set plus the
        generation are what make the pair sufficient.
        """
        generations = self._cancel_gen
        for arm_id in list(generations):
            generations[arm_id] = generations[arm_id] + 1
        travels = self._arm_travel
        for arm_id in list(travels):
            travels[arm_id] = None

    def request_gripper_action(self, arm_id, action, width_mm=None,
                               operator_lease=None, timeout_s=5.0):
        """
        Ask the supervisor to command one gripper; blocks for the verdict.

        This entry point stays here because the operator lease and "is this
        arm in the running session" live in the supervisor and nowhere else.
        The DISPATCH and the busy bookkeeping live in ros_bridge.py; only the
        admission decision is here.
        """
        return self._submit(
            _Command(kind='gripper', request=(arm_id, action, width_mm),
                     operator_lease=operator_lease), timeout_s)

    def request_session_recover(self, operator_lease=None,
                                timeout_s=RECOVERY_REQUEST_TIMEOUT_S):
        """Run the bounded session-wide recovery; blocks for the verdict."""
        return self._submit(
            _Command(kind='recover', operator_lease=operator_lease), timeout_s)

    def revoke_operator_authorization(self):
        """
        Force every enable off the instant control leaves an operator.

        Registered with the :class:`~franka_web.lock.OperatorLock` in this
        object's constructor, so it runs inside the lock's own mutex at the
        moment a held token is dropped -- lazy expiry or explicit release --
        and therefore always *before* a successor token can exist. That is
        what makes section 5.6's "lock expiry forces every enable off" and
        section 6.13's "re-enabling is a new authorization, never an
        automatic continuation" true of every path to a new token.

        Previously the only expiry-driven clearing was the frame pump
        noticing a falling edge at 5 Hz, so a claim landing inside one
        sampling period inherited live enables and the jog stream kept
        publishing for an operator who had never pressed Enable
        (verification finding F-0).

        Contract, because of where it runs: never block, never call back
        into the lock. The work here is one snapshot of the enable flags,
        per-key assignment into that same dict (atomic under the GIL, and no
        key is added or removed, so a concurrent iteration stays valid), the
        same per-key reset of the command sources, and at most one queue put.
        The controller-side disable is left to the supervisor thread, and is
        queued only when something was actually enabled, so a repeated
        observation cannot flood the queue.

        The source reset uses the LOCK-FREE variant on purpose. Calling the
        ``_state_lock``-held ``_force_sources_jog`` from here would create the
        lock-order inversion ``OperatorLock._mutex -> _state_lock``, opposite
        to the supervisor's own order: any future code reading the lock while
        holding ``_state_lock`` would deadlock the server, and even today it
        would stall every heartbeat behind a supervisor critical section. The
        ROS-side teardown of the counting subscriptions is reconciled on the
        next supervisor tick instead.
        """
        flags = self._arm_enabled
        enabled = [arm_id for arm_id, on in list(flags.items()) if on]
        for arm_id in enabled:
            flags[arm_id] = False
        # Authority left; motion stops; nothing resumes on the next claim.
        # The source reset carries every travel with it.
        self._force_sources_jog_lockfree()
        if enabled and self._jog_models:
            self._commands.put(_Command(kind='disable_all'))

    def _submit(self, command, timeout_s):
        """Queue a command for the supervisor thread and await its answer."""
        self._commands.put(command)
        if not command.done.wait(timeout_s):
            if command.abandon():
                # The supervisor never began it; it will be skipped, so the
                # refusal the operator sees is the truth.
                raise SessionError(
                    'internal_error',
                    'the supervisor is busy (a long stop or preflight is in '
                    'progress); the command was discarded — retry once the '
                    'state settles')
            # Execution already began and cannot be cancelled.  A timeout
            # response here would be false while a recovery/start/stop kept
            # changing the stack.  Wait for the command's real verdict; every
            # external operation in those handlers is independently bounded.
            command.done.wait()
        if command.error is not None:
            raise command.error
        return command.result

    # ------------------------------------------------------------------
    # Supervisor thread
    # ------------------------------------------------------------------

    def run_forever(self, shutdown_event):
        """
        Tick at the configured cadence until ``shutdown_event`` is set.

        An unexpected exception from a tick must never kill the loop —
        a dead supervisor is a live robot stack with no owner (review
        finding R8): it is recorded and the machine is driven to
        ``stopping`` so the children are reaped.
        """
        while not shutdown_event.is_set():
            try:
                self.tick()
            except Exception as error:
                self._record_error('internal_error',
                                   'supervisor tick failed: {}'.format(error))
                if self.state not in ('stopped', 'stopping'):
                    self._transition('stopping', reason='internal_error')
            shutdown_event.wait(defaults.SUPERVISOR_TICK_S)
        self.shutdown()

    def tick(self):
        """Run one supervisor step: commands first, then state work."""
        self._process_commands()
        self._reconcile_external_counters()
        state = self.state
        if state == 'preflight':
            self._do_preflight()
        elif state == 'starting':
            self._poll_starting()
        elif state == 'settling':
            self._poll_settling()
        elif state == 'running':
            self._poll_running()
        elif state == 'fault':
            self._poll_fault()
        if self.state == 'stopping':
            self._do_stopping()

    def shutdown(self):
        """
        Drive whatever is running down to ``stopped`` (process exit path).

        A guarded launch that survived even the bounded SIGKILL stage is an
        unkillable robot stack, not a stopped session. Keep retrying at the
        supervisor cadence so the server retains its pidfile and no replacement
        owner can start. A D-state survivor intentionally prevents clean server
        exit until the kernel reports that exact target group gone.
        """
        if self.state != 'stopped':
            self._transition('stopping', reason=None)
        while self.state != 'stopped':
            self._do_stopping()
            if self.state != 'stopped':
                time.sleep(defaults.SUPERVISOR_TICK_S)

    @property
    def state(self):
        """Return the current state name."""
        with self._state_lock:
            return self._state

    # ------------------------------------------------------------------
    # Command processing (supervisor thread only)
    # ------------------------------------------------------------------

    def _process_commands(self):
        """Drain queued operator commands, answering each synchronously."""
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            if not command.try_begin():
                continue    # abandoned by its waiter; never execute it
            try:
                if (self._command_requires_current_lease(command)
                        and not self._lock_service.lease_is_current(
                            command.operator_lease)):
                    raise SessionError(
                        'operator_token_invalid',
                        'operator control changed before the command began; '
                        'repeat the action under the current lock')
                if command.kind == 'start':
                    command.resolve(self._accept_start(
                        command.request,
                        operator_lease=command.operator_lease))
                elif command.kind == 'stop':
                    command.resolve(self._accept_stop())
                elif command.kind == 'enable':
                    command.resolve(self._accept_arm_enable(
                        *command.request, operator_lease=command.operator_lease))
                elif command.kind == 'jog':
                    command.resolve(self._accept_arm_jog(*command.request))
                elif command.kind == 'source':
                    command.resolve(self._accept_arm_source(*command.request))
                elif command.kind == 'apply':
                    command.resolve(self._accept_arm_apply(*command.request))
                elif command.kind == 'gripper':
                    command.resolve(self._accept_gripper(*command.request))
                elif command.kind == 'recover':
                    command.resolve(self._accept_session_recover())
                elif command.kind == 'disable_all':
                    command.resolve(self._disable_all_arms())
                else:
                    command.reject(SessionError('internal_error', 'unknown command'))
            except SessionError as error:
                command.reject(error)
            except Exception:
                # A taken command may no longer be abandoned. Resolve its
                # waiter with a safe verdict, then propagate so run_forever's
                # outer fail-safe records the fault and tears the stack down.
                command.reject(SessionError(
                    'internal_error', 'internal server error'))
                raise

    @staticmethod
    def _command_requires_current_lease(command):
        """
        Return whether taking ``command`` can create or change live control.

        The rule is about what the command DOES, not which route it arrived
        on: ``enable`` needs a current lease only when it is turning authority
        ON, because refusing to turn it off for want of authority is the wrong
        failure. ``apply`` takes the same shape -- a ``start`` needs one, and a
        ``cancel`` never reaches this queue at all (it is answered on the HTTP
        thread), so the same principle is kept in the same place.
        """
        if command.kind in ('start', 'jog', 'recover', 'source', 'gripper'):
            return True
        if command.kind == 'apply':
            return (command.request is not None
                    and command.request[1] == 'start')
        return (command.kind == 'enable' and command.request is not None
                and bool(command.request[1]))

    def _accept_start(self, request, operator_lease=None):
        """Validate a start request and enter ``preflight``."""
        with self._state_lock:
            if self._state != 'stopped':
                raise SessionError('session_already_active',
                                   'a session is already active; stop it first')
        if request.arms not in _VALID_ARMS:
            raise SessionError('invalid_arms',
                               "arms must be one of 'panda1', 'panda2', 'both'")
        if request.mode not in _VALID_MODES:
            raise SessionError('invalid_mode',
                               'mode must be one of {}'.format(
                                   ', '.join(repr(m) for m in _VALID_MODES)))
        profile = PROFILES[(request.arms, request.mode)]
        motion = request.mode == 'motion'
        stored = None
        param_file = None
        if motion:
            stored = self._materialize_profile(request.arms, profile.arm_ids)
            param_file = stored.path
        try:
            launch_argv = self._argv_builder(
                request.arms, request.mode, self._settings,
                controller_param_file=param_file)
        except ProfileError as error:
            raise SessionError('robot_addresses_missing', str(error)) from None
        settling_policy = self._settings.settling.policy() if motion else None
        session_id = self._session_namer()
        claim_id = None
        claim_id_of = getattr(self._lock_service, 'claim_id_of', None)
        if claim_id_of is not None:
            claim_id = claim_id_of(operator_lease)
        with self._state_lock:
            self._session = {
                'session_id': session_id,
                'arms': request.arms,
                'arm_ids': list(profile.arm_ids),
                'arm_mode': profile.arm_mode,
                'mode': request.mode,
                # Internal only; never emitted. A motion session always runs
                # the one reviewed impedance controller, so this is no longer
                # request-derived -- but _fault_snapshot, _readiness_met and
                # the recovery path all still read it by name.
                'controller_name': defaults.MOTION_CONTROLLER if motion else None,
                'profile_sha256': (stored.config_sha256
                                   if stored is not None else None),
                'settling_policy': settling_policy,
                'activation_policy_sha256': (
                    settling_policy.sha256 if settling_policy is not None else None),
                # The LIVE operator identity, refreshed by claim adoption. A
                # frozen start-time id would pin `lock_expired` forever.
                'operator_claim_id': claim_id,
                'started_at': rfc3339(self._utcnow()),
                'started_mono': self._monotonic(),
                'launch_argv': launch_argv,
                'last_error': None,
            }
            self._arm_enabled = {arm: False for arm in profile.arm_ids}
            self._arm_source = {arm: 'jog' for arm in profile.arm_ids}
            # The key sets must exist BEFORE any lock-free writer can run: the
            # revocation hook, `_cancel_travel_now` and `_force_sources_jog_
            # lockfree` all assign per key and none of them may create one.
            self._arm_travel = {arm: None for arm in profile.arm_ids}
            self._enable_epoch = {arm: 0 for arm in profile.arm_ids}
            self._cancel_gen = {arm: 0 for arm in profile.arm_ids}
            self._last_advance = {arm: None for arm in profile.arm_ids}
            self._preflight_result = None
            self._preflight_pending = True
            self._profile_record = stored
            self._arm_slots = {arm: slot for slot, arm
                               in enumerate(profile.arm_ids, start=1)}
            self._jog_models = {}
            if motion:
                for arm_id in profile.arm_ids:
                    fence = stored.fence[arm_id]
                    self._jog_models[arm_id] = JogTargetModel(
                        arm_id, fence['position_lower'], fence['position_upper'],
                        step_rad=self._settings.jog_step_rad)
            self._targets_published = {arm: 0 for arm in profile.arm_ids}
            self._last_publish_mono = {}
            self._recover_succeeded = False
            self._activation_baseline = None
            self._baseline_captured = False
            self._activation_gate = None
            self._settling_target_counts = {}
            self._recording_sealed = False
            self._fault_engine.reset()
            self._clear_fault()
        self._steps_init(request.mode, profile.arm_ids)
        self._step_active('preflight')
        self._logs.emit('info', 'session {} starting: {} / {}'.format(
            session_id, request.arms, request.mode))
        self._transition('preflight', reason=None)
        return {'session_id': session_id, 'state': 'preflight'}

    def _materialize_profile(self, arms, arm_ids):
        """Render, validate and content-address this session's profile."""
        if self._profile_store is None:
            raise SessionError('internal_error', 'no profile store is wired')
        profiles = {arm_id: self._settings.profile(arm_id) for arm_id in arm_ids}
        try:
            return self._profile_store.materialize(arms, profiles)
        except ProfileStoreError as error:
            raise SessionError(error.code, error.detail) from None

    def adopt_operator_claim(self, claim_id):
        """
        Record ``claim_id`` as this session's operator claim.

        Called from the HTTP thread on a successful claim, takeover or
        heartbeat. Without it a ``lock_expired`` fault is a dead end: the
        Reclaim the fault's own steps prescribe mints a NEW claim id, which
        would still differ from a frozen start-time id, so the cause would
        stay ``lock_expired`` and the action would stay ``reclaim`` forever --
        and the page, which draws its primary button from the action, would
        never offer Recover.

        It takes ``_state_lock`` and nothing else, does no I/O and never
        touches the operator lock, so it is safe on an HTTP worker thread. It
        is never called from the revocation hook.
        """
        if not isinstance(claim_id, str) or not claim_id:
            return
        with self._state_lock:
            if self._state == 'stopped' or self._session is None:
                return
            self._session['operator_claim_id'] = claim_id

    # ------------------------------------------------------------------
    # The verification checklist (session.steps)
    # ------------------------------------------------------------------

    def _steps_init(self, mode, arm_ids):
        """Build the ordered start checklist; every entry pending."""
        entries = [('preflight', _STEP_LABELS['preflight'])]
        entries += [('connect:' + arm, 'Connect ' + arm) for arm in arm_ids]
        entries += [('health', _STEP_LABELS['health'])]
        if mode == 'motion':
            # `stack_ready` names the wait the launch owns (its broadcasters
            # and its own controller activation); `controller_pause` names the
            # moment the arms go briefly hand-movable, which the console must
            # say BEFORE it happens.
            entries += [('stack_ready', _STEP_LABELS['stack_ready']),
                        ('controller_pause', _STEP_LABELS['controller_pause']),
                        ('baseline', _STEP_LABELS['baseline']),
                        ('controller', _STEP_LABELS['controller']),
                        ('settling', _STEP_LABELS['settling'])]
        else:
            entries += [('baseline', _STEP_LABELS['baseline'])]
        self._steps_replace(entries)

    def _steps_recovery(self, arm_ids):
        """Replace the checklist with the recovery list."""
        entries = [('reconnect:' + arm, 'Reconnect ' + arm) for arm in arm_ids]
        entries += [('controller', _RECOVERY_STEP_LABELS['controller']),
                    ('verify', _RECOVERY_STEP_LABELS['verify'])]
        self._steps_replace(entries)

    def _discard_stale_recovery_steps(self):
        """
        Drop a recovery checklist left over from an earlier fault episode.

        A recovery checklist is the frame's ONLY evidence that a recovery is
        under way, and the console renders "Recovering" from it. Leaving a
        finished one in place across a NEW fault made the page claim a
        recovery the server had never started: live finding V2L-7, where a
        second Recover press produced no ``recovery started`` line at all and
        the page nonetheless showed "Recovering" indefinitely. A fault
        episode begins with no recovery in progress, so it begins with no
        recovery checklist; ``_steps_recovery`` installs a fresh one the
        moment a real recovery starts.

        The START checklist is deliberately left alone: a start-path refusal
        marks its failing step before entering fault, and that is the only
        account the operator gets of where the start broke.
        """
        with self._state_lock:
            if not self._steps:
                return
            first = self._steps[0].get('id')
            if isinstance(first, str) and first.startswith('reconnect:'):
                self._steps = []

    def _steps_replace(self, entries):
        """Install a fresh checklist, every entry pending."""
        with self._state_lock:
            self._steps = [
                {'id': step_id, 'label': label, 'status': 'pending',
                 'duration_s': None, 'detail': None, 'started_mono': None}
                for step_id, label in entries]

    def _step_index(self, step_id):
        """Return the index of ``step_id`` in the checklist, or None."""
        for index, step in enumerate(self._steps):
            if step['id'] == step_id:
                return index
        return None

    def _steps_status(self, step_id):
        """
        Return one step's status, or None when the id is not in the array.

        Asking about a step this mode does not have is legitimate (a Simulate
        session has no ``controller`` step), so this never raises.
        """
        with self._state_lock:
            index = self._step_index(step_id)
            return None if index is None else self._steps[index]['status']

    def _first_pending_step(self):
        """
        Return the id of the first ``pending``-or-``active`` step.

        When every step is done, return the LAST step's id, so a caller that
        marks "whatever we were doing" failed always has a real target.
        """
        with self._state_lock:
            if not self._steps:
                return None
            for step in self._steps:
                if step['status'] in ('pending', 'active'):
                    return step['id']
            return self._steps[-1]['id']

    def _step_active(self, step_id):
        """Mark one step active; idempotent for an active or finished step."""
        with self._state_lock:
            index = self._step_index(step_id)
            if index is None:
                return
            step = self._steps[index]
            if step['status'] != 'pending':
                return
            step['status'] = 'active'
            step['started_mono'] = self._monotonic()

    def _step_done(self, step_id, detail=None):
        """Mark one step done, back-filling any earlier step still pending."""
        self._step_finish(step_id, 'done', detail)

    def _step_fail(self, step_id, detail):
        """Mark one step failed, back-filling any earlier step still pending."""
        self._step_finish(step_id, 'failed', detail)

    def _step_finish(self, step_id, status, detail):
        """Close one step and keep the array monotone left to right."""
        if step_id is None:
            return
        with self._state_lock:
            index = self._step_index(step_id)
            if index is None:
                return
            for earlier in self._steps[:index]:
                if earlier['status'] in ('pending', 'active'):
                    earlier['status'] = 'done'
                    earlier['duration_s'] = self._step_duration(earlier)
            step = self._steps[index]
            step['status'] = status
            step['duration_s'] = self._step_duration(step)
            if detail is not None:
                step['detail'] = detail

    def _step_duration(self, step):
        """Return a step's elapsed seconds, or None if it was never active."""
        started = step.get('started_mono')
        if started is None:
            return None
        return round(self._monotonic() - started, 3)

    def _steps_frame(self):
        """Return the checklist in the shape the frame publishes it."""
        with self._state_lock:
            return [{'id': step['id'], 'label': step['label'],
                     'status': step['status'], 'duration_s': step['duration_s'],
                     'detail': step['detail']}
                    for step in self._steps]

    def _accept_stop(self):
        """Handle an operator stop: idempotent while shutting down (§6.8)."""
        with self._state_lock:
            state = self._state
            session = dict(self._session) if self._session else None
            if state == 'stopped':
                raise SessionError('session_not_active', 'no session is active')
            if state == 'stopping':
                return {'state': 'stopping'}
        if session is not None:
            self._logs.emit('info', 'stopping session {}'.format(
                session['session_id']))
        self._transition('stopping', reason=None)
        return {'state': 'stopping'}

    def _motion_guards(self, arm_id, allow_fault=False):
        """Run the common §6.13 guards and return the session snapshot."""
        with self._state_lock:
            state = self._state
            session = dict(self._session) if self._session else None
        if session is None or state in ('stopped', 'stopping'):
            raise SessionError('session_not_running', 'no session is running')
        if state == 'fault' and not allow_fault:
            raise SessionError('session_faulted',
                               'the session is faulted; recover or stop first')
        if state not in ('running', 'fault'):
            raise SessionError('session_not_running',
                               'the session is not running yet')
        if session['mode'] != 'motion':
            raise SessionError('not_motion_mode', 'this session is not in motion mode')
        if arm_id not in session['arm_ids']:
            raise SessionError('arm_not_in_session',
                               '{} is not part of this session'.format(arm_id))
        return session, state

    def _guard_no_fresh_fault(self, session, action):
        """Refuse one command-increasing action if the latest cache is faulted."""
        reasons = self._fault_engine.evaluate(self._fault_snapshot(session))
        if not reasons:
            return
        self._enter_fault(session, reasons)
        raise SessionError(
            'session_faulted',
            'a fresh session fault was observed before {}; recover or stop '
            'first'.format(action))

    def _accept_arm_enable(self, arm_id, enabled, operator_lease=None):
        """
        §6.13 POST /api/arm/{arm_id}/enable, executed in exact order.

        The FIRST thing this handler does on BOTH branches is clear this arm's
        travel and bump its enable epoch. That placement is load-bearing rather
        than tidy: ``model.seed(measured)`` below runs before the service call
        and therefore before every one of this handler's failure exits, and
        three of those exits (``enable_service_unavailable``,
        ``enable_rejected``, and the no-answer path) leave the arm still
        enabled and still on its source. A plan left live beside a re-seeded
        model would resume from a target that had just been yanked off the
        checked line, by as much as ``APPLY_LAG_LIMIT_RAD``. Clearing at the
        top makes the rule path-independent: no exit from this handler,
        including one added later, can leave a live plan beside a re-seeded
        model.
        """
        session, state = self._motion_guards(arm_id)
        if state != 'running':
            raise SessionError('session_not_running', 'the session is not running')
        with self._state_lock:
            self._arm_travel[arm_id] = None
            self._cancel_gen[arm_id] = self._cancel_gen.get(arm_id, 0) + 1
            self._enable_epoch[arm_id] = self._enable_epoch.get(arm_id, 0) + 1
            model = self._jog_models.get(arm_id)
            slot = self._arm_slots.get(arm_id)
        if model is None:
            raise SessionError('not_motion_mode',
                               'this session has no enable surface')
        if not enabled:
            # Clear the flag FIRST, then tell the controller: never leave the
            # publisher running against a disabled arm's stale state.
            with self._state_lock:
                self._arm_enabled[arm_id] = False
            model.invalidate()
            response = self._bridge.call_enable(slot, False)
            message = response['message'] if response else (
                'disable requested; the controller did not answer -- the jog '
                'stream is stopped either way')
            self._logs.emit('info', '{} disabled'.format(arm_id))
            return {'arm_id': arm_id, 'enabled': False, 'target': None,
                    'message': message}
        sample = self._bridge.joint_sample()
        now_ns = int(self._monotonic() * 1e9)
        if (sample is None or
                (now_ns - sample[0]) / 1e9 > defaults.ENABLE_JOINT_STATE_MAX_AGE_S):
            raise SessionError('joint_state_stale',
                               'no joint sample newer than {} s'.format(
                                   defaults.ENABLE_JOINT_STATE_MAX_AGE_S))
        joints = health.extract_joints(arm_id, sample[1])
        if not joints['complete']:
            raise SessionError('joint_state_stale',
                               'the joint sample does not carry all 7 joints')
        measured = joints['positions']
        with self._state_lock:
            fence = self._profile_record.fence[arm_id]
        outside = _joints_outside_fence(
            measured, fence['position_lower'], fence['position_upper'])
        if outside:
            raise SessionError('pose_outside_fence',
                               '{} measured joints outside the fence: {}'.format(
                                   arm_id, ', '.join(outside)))
        try:
            model.seed(measured)
        except JogError as error:
            raise SessionError('pose_outside_fence', str(error)) from None
        if not self._bridge.enable_service_ready(slot):
            raise SessionError('enable_service_unavailable',
                               'the enable service is not reachable')
        # Commands are processed before the ordinary running-state poll. A
        # fault callback can therefore land after the previous tick yet before
        # this queued Enable. Re-evaluate the freshest complete snapshot at
        # the final local gate; never call SetBool(true) from cached-faulted
        # state merely because the prior published frame still said Running.
        self._guard_no_fresh_fault(session, 'enable')
        response = self._bridge.call_enable(slot, True)
        if response is None:
            # The request may have reached the controller even though the
            # answer never arrived: without compensation the arm could sit
            # enabled at the controller while the UI says disabled (review
            # finding S3). Best-effort disable before reporting failure.
            self._bridge.call_enable(slot, False, timeout_s=2.0)
            raise SessionError('enable_service_unavailable',
                               'the enable service did not answer in time; a '
                               'compensating disable was sent')
        if not response['success']:
            raise SessionError('enable_rejected', response['message'])

        # The controller call ran outside our locks.  It may have succeeded
        # after pagehide released this request's operator token, after the
        # lease expired/reclaimed, or after the session changed.  Do not turn
        # that stale success into a fresh local authorization.
        with self._state_lock:
            current = self._session
            context_current = (
                self._state == 'running'
                and current is not None
                and current.get('session_id') == session.get('session_id')
                and current.get('mode') == 'motion'
                and arm_id in current.get('arm_ids', ())
                and self._jog_models.get(arm_id) is model
                and self._arm_slots.get(arm_id) == slot)
        if not context_current:
            with self._state_lock:
                self._arm_enabled[arm_id] = False
            model.invalidate()
            try:
                self._bridge.call_enable(slot, False, timeout_s=2.0)
            except Exception:  # noqa: BLE001 - compensation is best effort
                pass
            raise SessionError(
                'session_not_running',
                'the session changed while enable was in flight; the jog '
                'stream remains off and a compensating disable was sent')

        # Preserve the established stale-authority verdict before evaluating
        # health against a clock that may have advanced while the service was
        # blocked. The atomic run_if_current check below is still required to
        # close a release racing the final local commit.
        if not self._lock_service.lease_is_current(operator_lease):
            with self._state_lock:
                self._arm_enabled[arm_id] = False
            model.invalidate()
            try:
                self._bridge.call_enable(slot, False, timeout_s=2.0)
            except Exception:  # noqa: BLE001 - compensation is best effort
                pass
            raise SessionError(
                'operator_token_invalid',
                'operator control changed while enable was in flight; the '
                'jog stream remains off and a compensating disable was sent')

        # The service itself is an external scheduling window. A diagnostic,
        # robot-state, or lifecycle callback can become faulting while
        # SetBool(true) is in flight. Re-check before creating any local
        # authorization; on every refusal/uncertainty, compensate the
        # controller-side enable first.
        try:
            self._guard_no_fresh_fault(session, 'enable completion')
        except Exception:
            with self._state_lock:
                self._arm_enabled[arm_id] = False
            model.invalidate()
            try:
                self._bridge.call_enable(slot, False, timeout_s=2.0)
            except Exception:  # noqa: BLE001 - compensation is best effort
                pass
            raise

        def commit_local_enable():
            # Runs inside OperatorLock's mutex.  Never block or call back into
            # that lock: the revocation hook uses this same flag as its CAS
            # counterpart and will clear it if release lands afterward.
            self._arm_enabled[arm_id] = True

        if not self._lock_service.run_if_current(
                operator_lease, commit_local_enable):
            self._arm_enabled[arm_id] = False
            model.invalidate()
            try:
                self._bridge.call_enable(slot, False, timeout_s=2.0)
            except Exception:  # noqa: BLE001 - compensation is best effort
                pass
            raise SessionError(
                'operator_token_invalid',
                'operator control changed while enable was in flight; the '
                'jog stream remains off and a compensating disable was sent')
        # §3 source 3: the drawer records every operator-facing action, and
        # granting torque command authority is the most safety-significant
        # one there is. Without this the log shows a `disabled` with no
        # matching `enabled`, and an operator reading it after an incident
        # cannot see when the arm became commandable.
        self._logs.emit('info', '{} enabled'.format(arm_id))
        return {'arm_id': arm_id, 'enabled': True,
                'target': list(model.target), 'message': response['message']}

    def _accept_arm_jog(self, arm_id, joint_index, direction):
        """§6.13 POST /api/arm/{arm_id}/jog — one fixed step on one joint."""
        session, _state = self._motion_guards(arm_id)
        with self._state_lock:
            model = self._jog_models.get(arm_id)
            enabled = self._arm_enabled.get(arm_id, False)
        if model is None:
            raise SessionError('not_motion_mode',
                               'this session has no jog surface')
        if not enabled:
            raise SessionError('arm_not_enabled',
                               'enable {} before jogging it'.format(arm_id))
        source = self._arm_source.get(arm_id)
        if source != 'jog':
            # Silently accepting a jog that publishes nothing is the worse
            # failure; `not_motion_mode` is the closest code the closed set
            # has, and the sentence says exactly what to do about it. It is
            # source-aware because the External sentence is simply false about
            # a ghost-sourced arm: nobody else's publisher is involved.
            raise SessionError(
                'not_motion_mode',
                'this arm is applying a ghost pose; switch the source back to '
                'Jog first'
                if source == 'ghost' else
                'this arm takes its commands from your own publisher; switch '
                'the source back to Jog first')
        self._guard_no_fresh_fault(session, 'jog')
        try:
            result = model.step(joint_index, direction)
        except JogError as error:
            raise SessionError('invalid_joint', str(error)) from None
        self._logs.emit('info', '{} jog joint{} {}'.format(
            arm_id, int(joint_index) + 1, '+' if direction > 0 else '-'))
        return {'arm_id': arm_id, 'target': list(result.target),
                'clamped': list(result.clamped)}

    def _accept_arm_source(self, arm_id, source):
        """Switch one arm between the jog, external and ghost sources."""
        session, state = self._motion_guards(arm_id)
        if state != 'running':
            raise SessionError('session_not_running', 'the session is not running')
        if source not in _SOURCES:
            raise SessionError(
                'invalid_source', "source must be 'jog', 'external' or 'ghost'")
        # Deliberately NO arm_not_enabled check: the switch is legal whether
        # or not the arm is enabled. The console greys the control out until
        # an arm is enabled; the backend must not depend on that.
        with self._state_lock:
            # The travel is cleared BEFORE the switch semantics run: the switch
            # is an explicit statement about who commands this arm, and
            # clearing first means the `jog` branch's re-seed reads a settled
            # arm rather than racing a moving target.
            self._arm_travel[arm_id] = None
            self._cancel_gen[arm_id] = self._cancel_gen.get(arm_id, 0) + 1
            previous = self._arm_source.get(arm_id)
            self._arm_source[arm_id] = source
            model = self._jog_models.get(arm_id)
        if source == 'external':
            if model is not None:
                # Re-seeded from the measured pose on the way back, so the
                # resumed stream can never republish a pre-external target.
                model.invalidate()
        else:
            self._reseed_jog_model(arm_id, model)
        self._reconcile_external_counters()
        if previous != source:
            self._logs.emit('info', '{} command source is now {}'.format(
                arm_id, source))
        return {'arm_id': arm_id, 'source': source}

    # ------------------------------------------------------------------
    # Apply -- executing a ghost pose on the real arm
    # ------------------------------------------------------------------

    @staticmethod
    def _checker_profile(session):
        """Return the cell-model profile this session calls for."""
        return 'single' if len(session['arm_ids']) == 1 else 'dual'

    def _checker_refusal(self, profile, arm_id=None):
        """
        Return ``(sentence, checker)`` when no Apply can run, else ``None``.

        Fail-closed, in full, and the three rows were written down in
        advance of this consumer existing: the package is absent, the cell file
        will not load, or the interlock says the model was built for a
        different description. In every one of them nothing moves, and the
        sentence the operator sees is the CHECKER's own -- there is no second
        copy of any of them in this file or on the page.

        The ghost degrades visibly here and Apply refuses, because the ghost
        commands nothing and Apply commands everything. There is no degraded
        "apply without a check" mode, and this is the one place that could
        have created one.
        """
        if self._checker is None:
            return NOTE_PACKAGE_ABSENT, 'absent'
        status = self._checker.status(profile)
        if not status['available']:
            return status['cell_note'] or NOTE_PACKAGE_ABSENT, 'absent'
        if status['interlock'] == 'mismatch':
            return status['checker_note'], 'mismatch'
        # A fourth row, and it is per ARM rather than per session: a model
        # that loaded may still not describe THIS arm, and an Apply it cannot
        # judge is an Apply it must refuse. It lives here rather than beside
        # the plan so that the frame's note and the handler's refusal are the
        # same answer to the same question -- otherwise the button would be
        # live and the press would fail.
        if arm_id is not None:
            model = self._checker.model_for(profile)
            if model is not None and arm_id not in tuple(model.arm_ids()):
                return NOTE_PROFILE_ARM_MISMATCH, 'absent'
        return None

    def _any_travel_live(self):
        """Return the arm id of a live travel anywhere in this session, or None."""
        with self._state_lock:
            for arm_id, plan in self._arm_travel.items():
                if plan is not None and plan.live:
                    return arm_id
        return None

    def _accept_arm_apply(self, arm_id, action, positions):
        """
        Start one ghost travel: check the whole path, then install the plan.

        ``action == 'cancel'`` never reaches here -- it is answered on the HTTP
        thread by :meth:`_cancel_travel_now`, off the command queue, because a
        stop must not be behind a queue whose consumer can be blocked for
        seconds. This handler is ``start`` and nothing else.

        What it deliberately does NOT do: call the IK service, read the
        client's scene, or trust anything about the request except seven
        floats. The co-arm's pose comes from THE SERVER'S OWN measured state.
        The solve endpoint may take the client's scene verbatim because there
        it decides only a tint; here it would decide motion, and only the
        server's measurements will do.
        """
        if action != 'start':
            raise SessionError('invalid_json',
                               "'action' must be 'start' or 'cancel'")
        session, state = self._motion_guards(arm_id)
        if state != 'running':
            raise SessionError('session_not_running', 'the session is not running')
        with self._state_lock:
            model = self._jog_models.get(arm_id)
            source = self._arm_source.get(arm_id)
            enabled = self._arm_enabled.get(arm_id, False)
            fence = (self._profile_record.fence[arm_id]
                     if self._profile_record is not None else None)
        if model is None or fence is None:
            raise SessionError('not_motion_mode',
                               'this session has no apply surface')
        if source != 'ghost':
            raise SessionError(
                'not_motion_mode',
                'this arm is not taking its commands from the ghost; switch '
                'the source to Ghost first')
        if not enabled:
            raise SessionError('arm_not_enabled',
                               'enable {} before applying a pose to it'.format(arm_id))
        busy = self._any_travel_live()
        if busy is not None:
            # One travel at a time, session-wide. Two independently timed
            # checked paths do not compose: each was approved against the other
            # arm held at a measured constant, and during execution neither
            # assumption holds. Refusing is the honest answer.
            raise SessionError(
                'apply_in_progress',
                '{} is travelling; wait for it to arrive or cancel it'.format(busy),
                {'arm_id': busy})
        # Commands are processed before the ordinary running-state poll, so a
        # fault callback can land between the queue and here. Same position the
        # jog handler puts it in: immediately before the action becomes real.
        self._guard_no_fresh_fault(session, 'apply')

        profile = self._checker_profile(session)
        refusal = self._checker_refusal(profile, arm_id)
        if refusal is not None:
            raise SessionError('apply_unavailable', refusal[0],
                               {'checker': refusal[1]})
        cell_model = self._checker.model_for(profile)

        sample = self._bridge.joint_sample()
        now_ns = int(self._monotonic() * 1e9)
        if (sample is None
                or (now_ns - int(sample[0])) / 1e9
                > defaults.ENABLE_JOINT_STATE_MAX_AGE_S):
            raise SessionError('joint_state_stale',
                               'no joint sample newer than {} s'.format(
                                   defaults.ENABLE_JOINT_STATE_MAX_AGE_S))
        joints = health.extract_joints(arm_id, sample[1])
        if not joints['complete']:
            raise SessionError('joint_state_stale',
                               'the joint sample does not carry all 7 joints')
        q_measured = tuple(joints['positions'])
        q_held = model.target
        if q_held is None:
            raise SessionError('arm_not_enabled',
                               'enable {} before applying a pose to it'.format(arm_id))
        co_arm_id = next((other for other in session['arm_ids']
                          if other != arm_id), None)
        co_arm_q = None
        if co_arm_id is not None:
            co_joints = health.extract_joints(co_arm_id, sample[1])
            co_arm_q = (tuple(co_joints['positions'])
                        if co_joints['complete'] else None)
        with self._state_lock:
            epoch = self._enable_epoch.get(arm_id, 0)
            generation = self._cancel_gen.get(arm_id, 0)
        try:
            plan = travel.plan_travel(
                arm_id=arm_id, q_held=q_held, q_measured=q_measured,
                q_goal=positions,
                fence_lower=fence['position_lower'],
                fence_upper=fence['position_upper'],
                max_target_velocity=fence['max_target_velocity'],
                model=cell_model, co_arm_id=co_arm_id, co_arm_q=co_arm_q,
                enable_epoch=epoch, cancel_gen=generation)
        except travel.TravelError as error:
            raise SessionError(error.code, error.detail, error.payload) from None

        # THE RE-READ IS NOT DECORATION. check_path ran between the read above
        # and this commit, and is budgeted at up to APPLY_CHECK_BUDGET_S. A
        # Cancel, a lock revocation or a disable landing inside that window
        # bumps one of the two integers; committing a plan stamped with the old
        # value would install a travel the operator has already stopped. The
        # tick's gates 5 and 6 would catch it next tick anyway -- this makes
        # the RESPONSE honest too.
        with self._state_lock:
            if (self._enable_epoch.get(arm_id) != epoch
                    or self._cancel_gen.get(arm_id) != generation
                    or self._arm_source.get(arm_id) != 'ghost'
                    or not self._arm_enabled.get(arm_id, False)):
                raise SessionError(
                    'apply_refused', travel.STOPPED_WHILE_CHECKING,
                    {'reason_code': 'no_travel'})
            self._arm_travel[arm_id] = plan
            # None, so the very first tick may advance at once: the spacing
            # floor is a floor between two advances, never a delay before one.
            self._last_advance[arm_id] = None
        self._logs.emit('info', '{} applying a ghost pose ({} steps, {:.1f} s)'.format(
            arm_id, plan.steps_total, plan.steps_total / defaults.JOG_STREAM_HZ))
        return {
            'arm_id': arm_id, 'action': 'start',
            'goal': list(plan.q1), 'start': list(plan.q0),
            'steps_total': plan.steps_total,
            'duration_s': round(plan.steps_total / defaults.JOG_STREAM_HZ, 3),
            'checked': dict(plan.checked),
        }

    def gripper_arm_ids(self, session):
        """
        Return the session's arms that have a configured gripper.

        EMPTY in Simulate, regardless of the config file: Simulate observes,
        moving fingers is motion, and a device that exists only inside this
        process would be a demonstration of the server rather than of the
        cell.
        """
        if session is None or session['mode'] == 'simulate':
            return ()
        return tuple(arm_id for arm_id in session['arm_ids']
                     if self._settings.gripper(arm_id).enabled)

    def _gripper_projection(self, arm_id, configured):
        """Return the frame's gripper block for one arm, right now."""
        return health.project_gripper(
            arm_id, int(self._monotonic() * 1e9),
            (self._bridge.gripper_status_sample(arm_id) if configured else None),
            configured=configured,
            busy=self._bridge.gripper_busy(arm_id))

    def _gripper_done(self, arm_id, action):
        """Return the callback that puts one dispatch's outcome in the drawer."""
        def _finished(outcome):
            """Record what the node answered, or that it never did."""
            if outcome is None:
                self._logs.emit('warn', 'gripper: {} {} got no answer'.format(
                    arm_id, action))
                return
            message = outcome.get('message') if isinstance(outcome, dict) else ''
            success = outcome.get('success') if isinstance(outcome, dict) else True
            self._logs.emit('info' if success else 'error',
                            'gripper: {} {}: {}'.format(arm_id, action,
                                                        message or 'done'))
        return _finished

    def _accept_gripper(self, arm_id, action, width_mm=None):
        """
        Admit or refuse one gripper command, most specific refusal first.

        There is deliberately NO session-mode row in this ladder. The gripper
        is a standing node commandable from ROS in every mode, so an API mode
        gate would refuse the web button while the identical motion stayed one
        `ros2 action send_goal` away. The page disables the row's buttons
        outside Motion, and that is an affordance against an accidental click,
        not a boundary. Simulate is a DIFFERENT rule and is enforced below,
        because there the gripper is not a mode gate but a fact about the
        session: there is no gripper there to command.
        """
        with self._state_lock:
            state = self._state
            session = dict(self._session) if self._session else None
        if session is None or state in ('stopped', 'stopping'):
            raise SessionError('session_not_running', 'no session is running')
        if arm_id not in session['arm_ids']:
            raise SessionError('arm_not_in_session',
                               '{} is not part of this session'.format(arm_id))
        configured = arm_id in self.gripper_arm_ids(session)
        if not configured:
            raise SessionError(
                'gripper_not_configured',
                'no gripper is configured for {}; set grippers.{}.enabled in '
                '{}'.format(arm_id, arm_id, self._settings.config_path))
        projection = self._gripper_projection(arm_id, True)
        if not projection['available']:
            raise SessionError('gripper_unavailable', projection['status_line'])
        if action in ('open', 'close', 'width'):
            if projection['fault_code']:
                raise SessionError('gripper_faulted', '{} Call /{}{}/reactivate.'.format(
                    projection['status_line'], arm_id,
                    defaults.GRIPPER_NODE_SUFFIX))
            if projection['busy']:
                raise SessionError(
                    'gripper_busy',
                    'Another gripper goal is running; cancel it first.')
        elif action == 'reactivate' and projection['busy']:
            # reactivate skips the fault row -- it IS the cure for a fault --
            # but keeps this one: it refuses while a goal is active.
            raise SessionError('gripper_busy',
                               'Another gripper goal is running; cancel it first.')
        return self._dispatch_gripper(arm_id, action, width_mm)

    def _dispatch_gripper(self, arm_id, action, width_mm):
        """Send one gripper command and return the endpoint's echo."""
        gripper = self._settings.gripper(arm_id)
        effective_mm = None
        if action in ('open', 'close', 'stop'):
            if action == 'open':
                effective_mm = gripper.open_width_mm
            elif action == 'close':
                effective_mm = gripper.close_width_mm
            result = self._bridge.call_gripper_trigger(arm_id, action)
            if result is None:
                raise SessionError(
                    'gripper_unavailable',
                    'the {} gripper node did not answer within {:.1f} s'.format(
                        arm_id, defaults.GRIPPER_REQUEST_TIMEOUT_S))
            if not result['success']:
                raise SessionError('gripper_unavailable', result['message'])
        elif action == 'reactivate':
            # This service returns only when the rACT cycle finishes, up to
            # the node's activation_timeout_s. Never hold the supervisor for
            # that; the outcome arrives in ~/status and in one log line.
            self._bridge.send_gripper_trigger_async(
                arm_id, 'reactivate', done=self._gripper_done(arm_id, action))
        else:
            if width_mm is None:
                # The endpoint already refuses this, but a caller reaching the
                # supervisor directly must get the same sentence rather than
                # an internal error out of float(None).
                raise SessionError(
                    'invalid_gripper_width',
                    "action 'width' requires a width in millimetres")
            effective_mm = float(width_mm)
            verdict = self._bridge.send_gripper_goal(
                # mm -> half-width in metres. This is the ONE arithmetic line
                # in franka_web that touches gripper units, and it is the
                # action's own documented convention rather than the
                # interpolated count mapping the driver owns. It stays here
                # because franka_web must not import franka_robotiq for
                # arithmetic; do not "move it into units.py".
                arm_id, effective_mm / 2000.0,
                # 0.0 means "use the node's configured force_n", which is the
                # one actually in force.
                0.0,
                done=self._gripper_done(arm_id, action))
            if verdict == 'rejected':
                raise SessionError(
                    'gripper_busy',
                    'the gripper refused the goal; another goal is running, or '
                    'the target is outside its stroke')
            if verdict is None:
                raise SessionError(
                    'gripper_unavailable',
                    'the {} gripper node did not answer within {:.1f} s'.format(
                        arm_id, defaults.GRIPPER_REQUEST_TIMEOUT_S))
        self._logs.emit('info', 'gripper: {} {}'.format(arm_id, action))
        return {'arm_id': arm_id, 'action': action,
                'width_mm': (None if effective_mm is None
                             else float(effective_mm))}

    def _reseed_jog_model(self, arm_id, model):
        """Seed one jog model from the latest measured pose; never raise."""
        if model is None:
            return
        try:
            sample = self._bridge.joint_sample()
            now_ns = int(self._monotonic() * 1e9)
            if (sample is None or int(sample[0]) > now_ns
                    or now_ns - int(sample[0])
                    > int(defaults.ENABLE_JOINT_STATE_MAX_AGE_S * 1e9)):
                return
            joints = health.extract_joints(arm_id, sample[1])
            if not joints['complete']:
                return
            model.seed(joints['positions'])
        except Exception:  # noqa: BLE001 - leaving it invalid is safe
            # An unseeded model publishes nothing until the operator
            # re-enables, which is the fail-closed direction.
            return

    def _reconcile_external_counters(self):
        """
        Make the bridge's counting subscriptions match ``_arm_source`` exactly.

        Called from the source switch, from every supervisor tick and from
        teardown. Ticking it is what makes the lock-free reset in the
        revocation hook sufficient: the hook only flips flags -- the thing the
        jog stream actually reads -- and the ROS-side subscriptions are
        reconciled on the very next tick.
        """
        with self._state_lock:
            running = (self._state == 'running' and self._session is not None
                       and self._session['mode'] == 'motion')
            wanted = ({arm_id: self._arm_slots[arm_id]
                       for arm_id, source in self._arm_source.items()
                       if source == 'external' and arm_id in self._arm_slots}
                      if running else {})
        try:
            self._bridge.set_external_counters(wanted)
        except Exception:  # noqa: BLE001 - a counter is telemetry, never a gate
            pass

    def _force_sources_jog(self):
        """
        Reset every arm to ``jog`` under the supervisor's own lock.

        Every travel goes with them. A source reset is a statement that this
        server no longer believes its own picture of who commands these arms,
        and a checked path executed on a disbelieved picture is the worst
        available option.
        """
        with self._state_lock:
            for arm_id in self._arm_source:
                self._arm_source[arm_id] = 'jog'
            self._clear_every_travel_locked()

    def _force_sources_jog_lockfree(self):
        """
        Reset every arm to ``jog`` without taking any supervisor lock.

        Runs inside OperatorLock's own mutex (the revocation hook). Per-key
        assignment into a dict whose key set is fixed for the life of the
        session is atomic under the GIL, and no key is added or removed, so a
        concurrent iteration under ``_state_lock`` stays valid. This is the
        same discipline the enable flags already follow.

        Every travel goes with them, through
        :meth:`_clear_every_travel_lockfree`, which is the same discipline
        again with one addition: the generation bump is what makes the clear
        stick, because the store alone could be undone by a tick's store-back.
        """
        sources = self._arm_source
        for arm_id in list(sources):
            sources[arm_id] = 'jog'
        self._clear_every_travel_lockfree()

    def _accept_session_recover(self):
        """Run the single operator-authorized recovery for the whole session."""
        with self._state_lock:
            state = self._state
            session = dict(self._session) if self._session else None
            recovery_complete = self._recover_succeeded
        if session is None or state != 'fault':
            raise SessionError('not_faulted', 'the session is not faulted')
        if recovery_complete:
            raise SessionError(
                'not_faulted',
                'recovery already completed; awaiting the running-state frame')
        if session['mode'] not in ('watch', 'motion'):
            raise SessionError('session_not_running',
                               'recovery applies to watch and motion sessions only')
        if not self._session_supports_recovery(session):
            raise SessionError(
                'recovery_not_supported',
                'this session cannot be recovered in place; stop and restart it')
        self._logs.emit('warn', 'recovery started')
        self._steps_recovery(session['arm_ids'])
        self._step_active('reconnect:' + session['arm_ids'][0])

        # A recovery is a new torque activation, never a continuation of a
        # previously ready gate.  Clear the old evidence before the first
        # backend/controller mutation so every exit path remains fail closed.
        if session['mode'] == 'motion':
            self._end_activation_capture()
            with self._state_lock:
                self._activation_baseline = None
                self._activation_gate = None
                self._settling_target_counts = {}

        # Eligibility is not a historical permission. A launch can die after
        # the transition into fault, so classify a fresh snapshot before any
        # disable/service/lifecycle mutation. An empty snapshot is allowed:
        # recover remains the operator action that clears the latched fault.
        current_reasons = self._fault_engine.evaluate(
            self._fault_snapshot(session))
        if current_reasons:
            current_recoverable = self._session_fault_recoverable(
                session, current_reasons)
            with self._state_lock:
                self._fault_reasons = tuple(current_reasons)
                self._fault_recoverable = current_recoverable
            if not current_recoverable:
                raise SessionError(
                    'recovery_not_supported',
                    'this fault requires stopping and restarting the session')

        self._force_enables_off()
        self._force_sources_jog()
        for model in self._jog_models.values():
            model.invalidate()
        steps = []
        controller = session['controller_name']

        # Local flags only silence the publisher. If an impedance controller
        # stayed active, explicitly clear every controller-side enable before
        # any backend or hardware operation can make it command-capable again.
        if session['mode'] == 'motion':
            controllers = self._bridge.query_controller_states()
            if controllers is None:
                self._recovery_failure(
                    session, steps, 'recovery_service_unavailable',
                    'controller-manager state is not reachable')
            if controllers.get(controller) == 'active':
                if not self._disable_controller_arms(session, steps, 'pre'):
                    self._recovery_failure(
                        session, steps, 'recovery_failed',
                        'the active impedance controller could not be disabled')
                if not self._deactivate_controller_verified(
                        controller, steps, 'pre'):
                    self._recovery_failure(
                        session, steps, 'recovery_failed',
                        'the active impedance controller did not reach inactive')

        # A dual global backend fault clears only when every backend is safe.
        # Visit every arm even after an earlier hard/unreachable result; each
        # ErrorRecovery is independently bounded and itself reaches state-only.
        all_recovered = True
        unavailable = False
        for arm_id in session['arm_ids']:
            recovery = self._bridge.call_error_recovery(arm_id)
            if recovery is None:
                ok = False
                detail = 'service unavailable'
                unavailable = True
            else:
                informational = (not recovery['success']
                                 and recovery['error'] == 'No errors')
                ok = bool(recovery['success'] or informational)
                detail = recovery['error'] if not recovery['success'] else ''
            steps.append({'step': 'error_recovery', 'arm_id': arm_id,
                          'ok': ok, 'detail': detail})
            if ok:
                self._step_done('reconnect:' + arm_id)
                index = session['arm_ids'].index(arm_id) + 1
                if index < len(session['arm_ids']):
                    self._step_active('reconnect:' + session['arm_ids'][index])
            else:
                self._step_fail('reconnect:' + arm_id, detail or 'recovery failed')
            all_recovered = all_recovered and ok
        if not all_recovered:
            code = ('recovery_service_unavailable' if unavailable
                    else 'recovery_failed')
            self._recovery_failure(
                session, steps, code,
                'one or more arm error-recovery operations failed')

        component = self._bridge.query_hardware_component()
        if component is None:
            self._recovery_failure(
                session, steps, 'recovery_service_unavailable',
                'controller-manager hardware state is not reachable')
        if component.get('lifecycle_label') == 'active':
            steps.append({'step': 'hardware_active', 'ok': True,
                          'detail': 'FrankaMultiHardwareInterface (already active)'})
        else:
            name = component.get('name') or 'FrankaMultiHardwareInterface'
            response = self._bridge.call_hardware_active(name)
            observed = self._bridge.query_hardware_component()
            ok = bool(response and response['ok'] and observed is not None
                      and observed.get('lifecycle_label') == 'active')
            steps.append({'step': 'hardware_active', 'ok': ok,
                          'detail': (name if ok else
                                     '{} was not verified active'.format(name))})
            if not ok:
                self._recovery_failure(
                    session, steps, 'recovery_failed',
                    'the hardware component did not reach active')

        required = list(expected_broadcasters(
            session['arm_ids'], session['arm_mode']))
        if session['mode'] == 'motion':
            required.append(controller)  # motion controller is always last
        controllers = self._bridge.query_controller_states()
        if controllers is None:
            self._recovery_failure(
                session, steps, 'recovery_service_unavailable',
                'controller-manager state is not reachable')
        recovery_activation_baseline = None
        for name in required:
            if controllers.get(name) == 'active':
                if name == controller and session['mode'] == 'motion':
                    self._recovery_failure(
                        session, steps, 'recovery_failed',
                        'the motion controller became active before activation '
                        'settling could be armed')
                steps.append({'step': 'controller_active', 'controller': name,
                              'ok': True, 'detail': 'already active'})
                continue
            if name == controller and session['mode'] == 'motion':
                recovery_activation_baseline = self._capture_recovery_activation_baseline(
                    session, steps)
                # Spend the reviewed switch spacing BEFORE arming, so the
                # capture still opens immediately before the lifecycle call
                # rather than accumulating a dwell of pre-activation rest.
                # _switch_activate's own dwell below is then already satisfied.
                self._await_switch_dwell()
                try:
                    # Arm under the bridge cache lock immediately before the
                    # lifecycle call that can reactivate torque control.
                    self._bridge.begin_activation_capture(session['arm_ids'])
                except Exception as error:  # noqa: BLE001 - fail closed
                    self._recovery_failure(
                        session, steps, 'recovery_failed',
                        'motion re-activation observation could not be armed: '
                        '{}'.format(error))
            response = self._switch_activate([name])
            controllers = self._bridge.query_controller_states()
            ok = bool(response and response['ok'] and controllers is not None
                      and controllers.get(name) == 'active')
            steps.append({'step': 'controller_active', 'controller': name,
                          'ok': ok,
                          'detail': ('active' if ok else
                                     'activation was not verified')})
            if not ok:
                self._recovery_failure(
                    session, steps, 'recovery_failed',
                    '{} did not reach active'.format(name))
        # NO post-reactivation SetBool(false).  Reaching this point in motion
        # mode always means the motion controller was just activated by the
        # loop above: an already-active motion controller fails the recovery
        # before it (a redundant re-activation could not be observed), so
        # onActivate()'s disableAndInvalidateAll() has run and the first RT
        # update has captured measured q as the internal target.  A false call
        # here would advance enable_generation and rebase that freshly captured
        # target onto whatever pose the arm had drifted to -- the same
        # false-to-false rebase that produced Panda 2's second J2 settling
        # episode on the fresh-startup path.  Record the invariant instead of
        # commanding it; the pre-deactivation disables above remain the real
        # proof for a controller that WAS active.
        if session['mode'] == 'motion':
            steps.append({
                'step': 'activation_disable_invariant',
                'controller': controller, 'ok': True,
                'detail': ('onActivate() disabled and invalidated every '
                           'impedance inbox; no redundant SetBool(false) was '
                           'sent, so the captured activation target stands'),
            })
        self._step_done('controller')
        self._step_active('verify')
        self._force_enables_off()
        self._force_sources_jog()
        for model in self._jog_models.values():
            model.invalidate()

        # Finish the potentially blocking lifecycle queries before the final
        # sample gate.  Otherwise joints/FrankaState/diagnostics that passed a
        # wait could become stale during the two service calls and still be
        # credited at the verdict.
        final_hardware = self._bridge.query_hardware_component()
        final_controllers = self._bridge.query_controller_states()

        # Only publications received after the complete restore and those
        # final lifecycle observations may certify the recovered stack.  The
        # local enable/target closure above is now the last mutation before
        # this barrier.  The absolute barrier also closes the missing-baseline
        # race: a callback that captured a pre-barrier message but stored it
        # after this snapshot cannot count as fresh.
        sample_baseline = self._recovery_sample_stamps(session)
        samples_fresh, fresh_reasons = self._wait_for_recovery_state(
            session, sample_baseline)
        verified = bool(
            final_hardware is not None
            and final_hardware.get('lifecycle_label') == 'active'
            and final_controllers is not None
            and all(final_controllers.get(name) == 'active' for name in required)
            and self._readiness_met(session)
            and samples_fresh
            and not fresh_reasons)
        steps.append({'step': 'verify_restore', 'ok': verified,
                      'detail': ('full session ready' if verified
                                 else 'fresh readiness/fault verification failed'),
                      'samples_fresh': samples_fresh,
                      'fault_reasons': [reason.as_dict()
                                        for reason in fresh_reasons]})
        if not verified:
            # A new fault (especially F7) may appear while the bounded fresh
            # sample wait is in progress. Publish that final snapshot before
            # rollback/failure so the UI cannot keep offering Recover from a
            # stale earlier reason set. If readiness alone failed with no
            # firing rule, retain the prior latched reasons as useful evidence.
            if fresh_reasons:
                with self._state_lock:
                    self._fault_reasons = tuple(fresh_reasons)
                    self._fault_recoverable = self._session_fault_recoverable(
                        session, fresh_reasons)
            self._recovery_failure(
                session, steps, 'recovery_failed',
                'the full session did not pass fresh readiness verification')

        # Recovery runs synchronously on the supervisor thread and can spend a
        # long time in bounded services.  Its ordinary fault-state recorder
        # poll therefore cannot run until after the command returns.  Check the
        # recording chain here, before publishing success, so a recorder that
        # failed during restoration yields rollback/stopping rather than a
        # false successful recovery with no durable evidence.
        with self._state_lock:
            recorder = self._recording
        if recorder is None:
            self._record_error(
                'recording_failed', 'the session recorder disappeared during recovery')
            self._transition('stopping', reason='recording_failed')
            recording_ok = False
        else:
            recording_ok = self._recorder_tick(recorder)
        if not recording_ok:
            self._recovery_failure(
                session, steps, 'recovery_failed',
                'the session recorder failed during recovery; recovery was '
                'failed closed instead of reporting success')
        if session['mode'] == 'motion':
            try:
                # Match initial startup: begin the bounded stability window only
                # after the active stack and recorder are verified.  The pose is
                # still compared with the state-only baseline captured before
                # controller activation.
                gate = self._install_activation_gate(
                    session, recovery_activation_baseline,
                    barrier_ns=int(self._monotonic() * 1e9))
            except ValueError as error:
                self._recovery_failure(
                    session, steps, 'recovery_failed',
                    'motion re-activation settling could not be armed: '
                    '{}'.format(error))
            capture_verdict = self._observe_activation_capture(gate)
            if capture_verdict.status == 'failed':
                self._recovery_failure(
                    session, steps, 'recovery_failed',
                    'motion re-activation exceeded the reviewed transition '
                    'envelope: {}'.format(capture_verdict.detail))
        self._step_done('verify')
        self._logs.emit('info', 'recovery complete')
        with self._state_lock:
            self._recover_succeeded = True
        return self._recovery_payload(session, steps)

    @staticmethod
    def _session_supports_recovery(session):
        """Watch and Motion are recoverable; Simulate is not."""
        return session['mode'] in ('watch', 'motion')

    def _session_fault_recoverable(self, session, reasons):
        """Combine fault-rule eligibility with the session controller policy."""
        return (self._session_supports_recovery(session)
                and FaultEngine.recoverable(session['mode'], reasons))

    def _activation_fences(self, session):
        """Return the selected arms' reviewed position fences."""
        with self._state_lock:
            record = self._profile_record
        if record is None:
            raise ValueError('the Motion session has no rendered profile')
        try:
            return {
                arm_id: (
                    tuple(record.fence[arm_id]['position_lower']),
                    tuple(record.fence[arm_id]['position_upper']))
                for arm_id in session['arm_ids']
            }
        except (KeyError, TypeError):
            raise ValueError(
                'the Motion session has no complete joint-position fence') from None

    def _install_activation_gate(self, session, baseline, *, barrier_ns):
        """Install a fresh immutable-policy gate for one torque activation."""
        policy = session.get('settling_policy')
        if policy is None or baseline is None:
            raise ValueError('no complete reviewed activation policy/baseline')
        # A defensive assertion, not a drift detector: both sides now read the
        # session's own frozen policy, so in correct code this is a tautology.
        # It costs one comparison and fails loudly if a future edit ever
        # reintroduces a second policy source.
        if session.get('activation_policy_sha256') != policy.sha256:
            raise ValueError('the session activation-policy identity changed')
        fences = self._activation_fences(session)
        ActivationSettlingGate.validate_baseline(
            policy, session['arm_ids'], baseline, fences)
        gate = ActivationSettlingGate(
            policy, session['arm_ids'], baseline, fences,
            started_mono_ns=barrier_ns, barrier_ns=barrier_ns,
            sample_max_age_s=defaults.ENABLE_JOINT_STATE_MAX_AGE_S)
        with self._state_lock:
            self._activation_baseline = {
                arm_id: tuple(baseline[arm_id]) for arm_id in session['arm_ids']}
            self._activation_gate = gate
            self._settling_target_counts = dict(self._targets_published)
        return gate

    def _observe_activation_capture(self, gate):
        """Drain callback extrema into ``gate`` without exposing a command surface."""
        try:
            capture = self._bridge.drain_activation_capture()
        except Exception:  # noqa: BLE001 - gate turns this into a closed failure
            capture = None
        with self._state_lock:
            return gate.observe_capture(capture)

    def _finalize_activation_capture(self, gate):
        """Atomically validate capture and commit Running at the same boundary."""
        def observe_and_commit(capture):
            verdict = gate.finalize_capture(
                capture, int(self._monotonic() * 1e9))
            if verdict.status == 'ready':
                # This is the internal state linearization point. The bridge
                # has closed callback admission, waited for every callback
                # admitted before that boundary, and still holds its cache
                # lock. The ordinary transition helper publishes the already-
                # committed state immediately afterward.
                self._state = 'running'
            return verdict

        try:
            with self._state_lock:
                return self._bridge.finalize_activation_capture(
                    observe_and_commit)
        except Exception:  # noqa: BLE001 - missing atomicity is a closed failure
            with self._state_lock:
                return gate.observe_capture(None)

    def _close_activation_capture(
            self, gate, *, next_state=None, commit=None):
        """Atomically validate/disarm capture and optionally commit an exit state."""
        def observe_and_commit(capture):
            verdict = gate.finalize_capture(
                capture, int(self._monotonic() * 1e9))
            if verdict.status != 'failed' and next_state is not None:
                if commit is not None:
                    commit()
                self._state = next_state
            return verdict

        try:
            with self._state_lock:
                return self._bridge.close_activation_capture(
                    observe_and_commit)
        except Exception:  # noqa: BLE001 - missing atomicity is a closed failure
            with self._state_lock:
                return gate.observe_capture(None)

    def _end_activation_capture(self):
        """Best-effort capture teardown used only on an already closed path."""
        try:
            self._bridge.end_activation_capture()
        except Exception:  # noqa: BLE001 - stopping/fault remains authoritative
            pass

    def _capture_recovery_activation_baseline(self, session, steps):
        """
        Capture the pre-reactivation pose for a Recover (§7.3).

        Every refusal is collapsed to ``recovery_failed`` on purpose: §1.4's
        closed set maps recovery refusals to that code, and
        ``POST /api/session/recover`` must not start answering with new ones.
        The DETAIL improves -- the fence message and the "became active"
        sentence both arrive here now.
        """
        return self._capture_activation_baseline(
            session, steps, phase='recovery',
            fail=lambda code, detail: self._recovery_failure(
                session, steps, 'recovery_failed', detail))

    def _capture_activation_baseline(self, session, steps, *, fail, phase):
        """
        Capture a fresh pose with the impedance controller PROVEN inactive.

        ``fail(code, detail)`` disposes of the session; the start-path sink
        returns (and this helper then returns ``None``), while Recover's
        raises into the waiting HTTP thread. Used by both, because the pose
        the settling gate measures against must be one the arm held while
        nothing was commanding it.
        """
        controller = session['controller_name']
        # Read and acted on BEFORE the sample is even looked at, let alone
        # judged fresh: a controller that came back active under us makes any
        # pose a POST-activation pose, after which the settling gate would
        # measure drift from an already-torqued arm and pass trivially.
        controllers = self._bridge.query_controller_states()
        if controllers is None:
            fail('activation_settling_limit',
                 'controller-manager state is not reachable, so the '
                 'controller could not be proven inactive before the baseline')
            return None
        if controllers.get(controller) == 'active':
            fail('activation_settling_limit',
                 'the impedance controller became active before a '
                 'pre-activation baseline could be captured')
            return None
        previous = self._bridge.joint_sample()
        previous_ns = previous[0] if previous is not None else None
        barrier_ns = int(self._monotonic() * 1e9)
        deadline = time.monotonic() + RECOVERY_FRESH_STATE_TIMEOUT_S
        while True:
            sample = self._bridge.joint_sample()
            now_ns = int(self._monotonic() * 1e9)
            freshness_barrier = (barrier_ns if previous_ns is None
                                 else max(barrier_ns, int(previous_ns)))
            sample_fresh = bool(
                sample is not None and int(sample[0]) <= now_ns
                and int(sample[0]) > freshness_barrier
                and now_ns - int(sample[0])
                <= int(defaults.ENABLE_JOINT_STATE_MAX_AGE_S * 1e9))
            if sample_fresh:
                break
            remaining = deadline - time.monotonic()
            if (remaining <= 0.0
                    or not self._recovery_wait(min(
                        defaults.SUPERVISOR_TICK_S, remaining))):
                fail('activation_settling_limit',
                     'no fresh joint sample was available before motion '
                     're-activation')
                return None
        baseline = {}
        for arm_id in session['arm_ids']:
            joints = health.extract_joints(arm_id, sample[1])
            try:
                positions = tuple(float(value) for value in joints['positions'])
            except (TypeError, ValueError):
                positions = ()
            if (not joints['complete'] or len(positions) != defaults.JOINT_COUNT
                    or not all(math.isfinite(value) for value in positions)):
                fail('activation_settling_limit',
                     '{} has no complete finite pre-activation pose'.format(
                         arm_id))
                return None
            baseline[arm_id] = positions
        fences = None
        try:
            policy = session.get('settling_policy')
            if policy is None or session.get('activation_policy_sha256') != policy.sha256:
                raise ValueError('the reviewed activation policy is unavailable')
            # Computed INSIDE the guard: _activation_fences raises the same
            # ValueError when the profile record is missing, and the teaching
            # message builder needs the fences it produced. Recovery gains
            # that message here; it used to emit the raw ValueError string.
            fences = self._activation_fences(session)
            ActivationSettlingGate.validate_baseline(
                policy, session['arm_ids'], baseline, fences)
        except ValueError as error:
            detail = (str(error) if fences is None else
                      self._baseline_fence_message(
                          session, baseline, fences, error))
            fail('pose_outside_fence', detail)
            return None
        steps.append({'step': 'baseline_captured', 'phase': phase, 'ok': True,
                      'detail': 'captured with the impedance controller '
                                'inactive'})
        return baseline

    @staticmethod
    def _wait_wall_time(timeout_s):
        """Wait for publishers without holding state locks; report that time passed."""
        time.sleep(timeout_s)
        return True

    def _recovery_sample_stamps(self, session):
        """Snapshot receipt stamps, followed by an absolute recovery barrier."""
        joint = self._bridge.joint_sample()
        robot = {
            arm_id: (sample[0] if sample is not None else None)
            for arm_id in session['arm_ids']
            for sample in (self._bridge.robot_state_sample(arm_id),)
        }
        diagnostic = {
            arm_id: (sample[0] if sample is not None else None)
            for arm_id in session['arm_ids']
            for sample in (self._bridge.diagnostic_sample(arm_id),)
        }
        return {
            # Capture this last. Every sample read above is deliberately on the
            # old side of the gate, including one concurrently stored while
            # the snapshot was being assembled.
            'barrier_ns': int(self._monotonic() * 1e9),
            'joint': joint[0] if joint is not None else None,
            'robot': robot,
            'diagnostic': diagnostic,
        }

    def _recovery_samples_fresh(self, session, baseline):
        """Require every receipt to be newer than the absolute recovery gate."""
        barrier_ns = baseline['barrier_ns']

        def is_new(sample, previous):
            threshold = barrier_ns if previous is None else max(barrier_ns, previous)
            return sample is not None and sample[0] > threshold

        joint = self._bridge.joint_sample()
        if not is_new(joint, baseline['joint']):
            return False
        for arm_id in session['arm_ids']:
            sample = self._bridge.robot_state_sample(arm_id)
            previous = baseline['robot'][arm_id]
            if not is_new(sample, previous):
                return False
            diagnostic = self._bridge.diagnostic_sample(arm_id)
            previous_diagnostic = baseline['diagnostic'][arm_id]
            if not is_new(diagnostic, previous_diagnostic):
                return False
        return True

    def _wait_for_recovery_state(self, session, baseline):
        """Wait boundedly for fresh samples and a non-firing fault snapshot."""
        deadline = time.monotonic() + RECOVERY_FRESH_STATE_TIMEOUT_S
        while True:
            fresh = self._recovery_samples_fresh(session, baseline)
            reasons = self._fault_engine.evaluate(self._fault_snapshot(session))
            if fresh and not reasons:
                return True, reasons
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return fresh, reasons
            if not self._recovery_wait(min(defaults.SUPERVISOR_TICK_S, remaining)):
                return fresh, reasons

    @staticmethod
    def _recovery_payload(session, steps):
        """Build the success/error evidence shared by HTTP and the UI."""
        return {'arm_ids': list(session['arm_ids']), 'steps': list(steps),
                'enabled_after': False}

    def _disable_controller_arms(self, session, steps, phase):
        """
        Visit every impedance slot and require an acknowledged disable.

        Only the ``pre`` phase uses this: an impedance controller that stayed
        ACTIVE across the fault may really be enabled, so its inboxes must be
        commanded off before any backend/hardware work can make them
        command-capable again.  There is deliberately no post-reactivation
        phase; see ``_accept_session_recover()`` for why a false-to-false call
        after a fresh onActivate() would rebase the captured target.
        """
        all_ok = True
        for arm_id in session['arm_ids']:
            slot = self._arm_slots.get(arm_id)
            response = self._bridge.call_enable(slot, False)
            ok = bool(response and response['success'])
            detail = (response['message'] if response is not None
                      else 'enable service unavailable')
            steps.append({'step': 'controller_disable', 'phase': phase,
                          'arm_id': arm_id, 'ok': ok, 'detail': detail})
            all_ok = all_ok and ok
        return all_ok

    def _await_switch_dwell(self):
        """
        Hold off until ``SWITCH_DWELL_S`` has passed since the last switch call.

        THE spacing discipline for every ``switch_controller`` call this
        server makes -- the start-path restage and Recover alike, through the
        shared ``_switch_activate``/``_switch_deactivate`` helpers below.
        Controller mode switches issued in tight succession stall the driver's
        real-time cycle: see ``SWITCH_DWELL_S`` for the live evidence.

        Bounded three ways, because this runs on the supervisor thread: by
        ``SWITCH_DWELL_S`` itself, by ``_SWITCH_DWELL_MAX_SLICES`` iterations,
        and by a wait callable that reports it can wait no further. Returns
        the seconds actually spent waiting, for the caller's evidence.
        """
        with self._state_lock:
            last = self._last_switch_mono
        if last is None:
            return 0.0
        started = self._monotonic()
        deadline = last + SWITCH_DWELL_S
        for _ in range(_SWITCH_DWELL_MAX_SLICES):
            remaining = deadline - self._monotonic()
            if remaining <= 0.0:
                break
            if not self._switch_dwell_wait(min(remaining, SWITCH_DWELL_S)):
                break
        return max(0.0, self._monotonic() - started)

    def _note_switch_issued(self):
        """Stamp the moment a ``switch_controller`` call returned."""
        with self._state_lock:
            self._last_switch_mono = self._monotonic()

    def _switch_activate(self, controllers):
        """Activate controllers with the reviewed spacing before and after."""
        self._await_switch_dwell()
        try:
            return self._bridge.call_switch_activate(controllers)
        finally:
            # In ``finally`` so a raising service call still spaces the NEXT
            # one: the driver saw the switch either way.
            self._note_switch_issued()

    def _switch_deactivate(self, controllers):
        """Deactivate controllers with the reviewed spacing before and after."""
        self._await_switch_dwell()
        try:
            return self._bridge.call_switch_deactivate(controllers)
        finally:
            self._note_switch_issued()

    def _deactivate_controller_verified(self, controller, steps, phase):
        """
        Deactivate one controller and prove it left ``active``.

        Shared by Recover's pre-deactivation and the start-path restage: both
        need the same evidence, and the appended step dict is byte-identical
        to the one recovery emitted before this was extracted, so
        ``POST /api/session/recover``'s payload is unchanged.
        """
        response = self._switch_deactivate([controller])
        observed = self._bridge.query_controller_states()
        inactive = bool(response and response['ok'] and observed is not None
                        and observed.get(controller) != 'active')
        steps.append({'step': 'controller_inactive', 'phase': phase,
                      'controller': controller, 'ok': inactive,
                      'detail': ('inactive' if inactive else
                                 'deactivation was not verified')})
        return inactive

    def _rollback_motion_controller(self, session, steps):
        """Best-effort compensation after an incomplete impedance restore."""
        if session['mode'] != 'motion':
            return
        self._force_enables_off()
        for model in self._jog_models.values():
            model.invalidate()
        for arm_id in session['arm_ids']:
            slot = self._arm_slots.get(arm_id)
            response = self._bridge.call_enable(slot, False, timeout_s=2.0)
            ok = bool(response and response['success'])
            steps.append({'step': 'rollback_disable', 'arm_id': arm_id,
                          'ok': ok,
                          'detail': (response['message'] if response is not None
                                     else 'enable service unavailable')})
        controller = session['controller_name']
        states = self._bridge.query_controller_states()
        if states is not None and states.get(controller) != 'active':
            steps.append({'step': 'rollback_deactivate',
                          'controller': controller, 'ok': True,
                          'detail': 'already inactive'})
            return
        response = self._switch_deactivate([controller])
        states = self._bridge.query_controller_states()
        ok = bool(response and response['ok'] and states is not None
                  and states.get(controller) != 'active')
        steps.append({'step': 'rollback_deactivate', 'controller': controller,
                      'ok': ok,
                      'detail': ('inactive' if ok
                                 else 'deactivation was not verified')})

    def _recovery_failure(self, session, steps, code, detail):
        """Remain faulted and raise while preserving every completed step."""
        self._logs.emit('error', 'recovery failed: {}'.format(detail))
        self._step_fail(self._first_pending_step(), detail)
        self._end_activation_capture()
        self._rollback_motion_controller(session, steps)
        self._force_enables_off()
        self._force_sources_jog()
        for model in self._jog_models.values():
            model.invalidate()
        with self._state_lock:
            self._activation_baseline = None
            self._activation_gate = None
            self._settling_target_counts = {}
        raise SessionError(code, detail,
                           payload=self._recovery_payload(session, steps))

    def _disable_all_arms(self):
        """Best-effort controller-side disable after a lock loss."""
        with self._state_lock:
            slots = dict(self._arm_slots)
            models = dict(self._jog_models)
            enabled_now = dict(self._arm_enabled)
        disabled = []
        for arm_id, slot in slots.items():
            if arm_id not in models:
                continue
            if enabled_now.get(arm_id):
                # Re-enabled under a fresh lock claim since this command was
                # queued: that is a new authorization — leave it alone.
                continue
            models[arm_id].invalidate()
            self._bridge.call_enable(slot, False, timeout_s=1.0)
            disabled.append(arm_id)
        return {'disabled': sorted(disabled)}

    def jog_stream_tick(self):
        """
        One 20 Hz jog-timer tick (runs on the bridge's executor thread).

        Publishes each enabled arm's held target -- ONLY while the session is
        running in motion mode, the arm's command source is ``jog`` or
        ``ghost``, and the operator lock is held and unexpired. Every other
        condition publishes nothing, which the controller answers with its
        0.1 s watchdog freeze. An arm whose source is ``external`` is therefore
        silent HERE, which is what makes every message counted on its target
        topic the operator's.

        The ONE thing an Apply adds is a branch that moves the held target one
        waypoint along an already-checked line before the message is built.
        Every byte that reaches the controller under source ``ghost`` is built
        by the same function, from the same held target, under the same four
        gates, as every byte that reaches it under source ``jog``. That is the
        whole safety argument, and it is checkable by reading this method.
        """
        with self._state_lock:
            if self._state != 'running' or self._session is None:
                return
            if self._session['mode'] != 'motion':
                return
            arms = [(arm_id, self._arm_slots.get(arm_id),
                     self._jog_models.get(arm_id),
                     self._arm_source.get(arm_id))
                    for arm_id, enabled in self._arm_enabled.items()
                    if enabled and self._arm_source.get(arm_id) in _SERVER_SOURCES]
        if not arms:
            return
        if not self._lock_service.state()['locked']:
            return
        ghosting = any(source == 'ghost' for _a, _s, _m, source in arms)
        # ONE joint_sample read per tick, and only when a ghost-sourced arm is
        # streaming: the stop guards compare measured against commanded, and a
        # bridge read per arm would double the cost for no new information.
        sample = self._bridge.joint_sample() if ghosting else None
        announcements = []
        for arm_id, slot, model, source in arms:
            if slot is None or model is None or not model.seeded:
                continue
            try:
                with self._state_lock:
                    # Re-check right before publishing: a _force_enables_off
                    # or a source switch landing after the snapshot must
                    # silence this arm now, not one tick later.
                    if (not self._arm_enabled.get(arm_id)
                            or self._arm_source.get(arm_id) != source):
                        continue
                    if source == 'ghost':
                        announced = self._advance_travel_locked(
                            arm_id, model, sample)
                        if announced is not None:
                            announcements.append(announced)
                message = model.message(self._bridge.now_msg(),
                                        health.joint_names_for(arm_id))
                self._bridge.publish_target(slot, message)
                with self._state_lock:
                    self._targets_published[arm_id] = (
                        self._targets_published.get(arm_id, 0) + 1)
                    self._last_publish_mono[arm_id] = self._monotonic()
            except Exception:  # noqa: BLE001 - one arm must not gap the other
                continue
        # Outside the critical section on purpose: LogBus.emit takes its own
        # lock, and holding two to write a sentence would be a new lock order.
        for level, sentence in announcements:
            self._logs.emit(level, sentence)

    def _advance_travel_locked(self, arm_id, model, sample):
        """
        Move one travelling arm's held target one waypoint; CALLER holds the lock.

        Returns a ``(level, sentence)`` pair for the caller to put on the log
        bus after releasing the lock, or None. Runs inside the tick's existing
        pre-publish critical section, so it is a handful of dict reads, a few
        float comparisons and one tuple rebind -- it must be atomic with
        respect to the clears that stop it, and it must cost nothing.

        The eight gates of the design live here and in the caller. Gates 1-4
        and 7 (running, enabled, source, operator lock, seeded) are the caller's
        and are shared with the jog path unchanged. This method adds:

        * **gate 5**, the enable epoch: a travel must never survive a
          disable/enable cycle, so even a missed clear stops the plan dead;
        * **gate 6**, the cancel generation: whoever bumped it -- the HTTP
          thread on Cancel, the revocation hook on a lost lock -- has stopped
          this travel by the next tick without touching a queue, a lock or a
          plan object;
        * **gate 8**, the compare-and-set: the store-back happens only if the
          plan read at the top is still the one in the dict, so a clear can
          never be undone by an advance computed before it.

        Steps are COUNTED, not timed, with one floor between them: a late tick
        makes the travel longer and can never make it take a bigger step. The
        floor exists because a stalled executor delivers ticks in a burst when
        it catches up, and a burst is the one schedule that could put more than
        one step's worth of command ahead of the controller's ramp.
        """
        plan = self._arm_travel.get(arm_id)
        if plan is None or not plan.live:
            return None                      # nothing to do; hold the target
        if (self._enable_epoch.get(arm_id) != plan.enable_epoch      # gate 5
                or self._cancel_gen.get(arm_id) != plan.cancel_gen):  # gate 6
            self._clear_travel_locked(arm_id, plan)          # stopped elsewhere
            return None
        measured, fresh = self._measured_for(arm_id, sample)
        co_arm_q, co_fresh = ((None, False) if plan.co_arm_id is None
                              else self._measured_for(plan.co_arm_id, sample))
        reason = travel.stop_reason(
            plan=plan, co_arm_q_now=co_arm_q, co_arm_fresh=co_fresh,
            q_measured=(measured if fresh else None), q_target=model.target)
        if reason is not None:
            # Stop-and-hold. The held target is not touched, so the arm sits at
            # the last CHECKED waypoint and the stream keeps feeding the
            # watchdog; only the advance stops.
            cleared = self._clear_travel_locked(arm_id, plan, reason)
            return None if cleared is None else ('warn', cleared)
        now = self._monotonic()
        previous = self._last_advance.get(arm_id)
        if (previous is not None
                and now - previous < defaults.APPLY_MIN_ADVANCE_PERIOD_S):
            return None                      # too soon; a hold, not a step
        try:
            model.set_target(plan.waypoint(plan.step + 1))
        except JogError as error:
            # The convexity lemma says this cannot happen: both endpoints were
            # fence-validated and every waypoint is between them. If it does,
            # it is a bug in the plan and the honest answer is to stop rather
            # than to bend the path.
            cleared = self._clear_travel_locked(
                arm_id, plan, '{} apply stopped: {}'.format(arm_id, error))
            return None if cleared is None else ('error', cleared)
        # The advanced plan is BUILT before the identity check, so the only
        # thing between the check and the store is the store itself. Building
        # it after the check would reopen exactly the window the check exists
        # to close: a lock-free clear landing inside it would be overwritten,
        # and the resurrected plan would be permanent -- nothing else clears
        # it, the frame would report a travel that can never move, and every
        # later Apply in the session would be refused as already in progress.
        following = plan.advanced()
        if self._arm_travel.get(arm_id) is plan:                 # gate 8: CAS
            self._arm_travel[arm_id] = following
            self._last_advance[arm_id] = now
            if not following.live:
                return ('info', '{} reached the applied pose'.format(arm_id))
        return None

    def _measured_for(self, arm_id, sample):
        """Return ``(positions, fresh)`` for one arm from one joint sample."""
        if sample is None:
            return None, False
        now_ns = int(self._monotonic() * 1e9)
        age_ns = now_ns - int(sample[0])
        if age_ns > int(defaults.ENABLE_JOINT_STATE_MAX_AGE_S * 1e9):
            return None, False
        joints = health.extract_joints(arm_id, sample[1])
        if not joints['complete']:
            return None, False
        return tuple(joints['positions']), True

    # ------------------------------------------------------------------
    # State work (supervisor thread only; heavy work outside the lock)
    # ------------------------------------------------------------------

    def _do_preflight(self):
        """Run the RT preflight once and act on its verdict (§3.4)."""
        with self._state_lock:
            if not self._preflight_pending:
                return
            self._preflight_pending = False
            mode = self._session['mode']
        result = self._preflight_runner(self._settings, mode)
        with self._state_lock:
            self._preflight_result = result
        if result.blocks_start():
            detail = self._preflight_failure_detail(result)
            self._record_error('preflight_failed', detail)
            self._step_fail('preflight', result.overall)
            self._logs.emit('error', detail)
            self._transition('stopping', reason='preflight_failed')
            return
        self._enter_starting()

    def _preflight_failure_detail(self, result):
        """
        Name the failing checks, and the config key when it is the cause.

        A bare verdict is not actionable. An ERROR means the run itself could
        not be made or understood, and only ``PreflightResult.error`` says why
        (tool missing, timed out, unusable report) -- the frame's preflight
        block has no field for it by §6.11, so ``last_error`` is where the
        operator reads it (verification finding F-2). A FAIL names checks
        instead: only ``name`` and ``summary`` are rendered, because those are
        the operator-facing halves and neither can carry a robot address.
        """
        checks = [check for check in (result.failed_checks or [])
                  if check.get('status') == 'FAIL'] or list(
                      result.failed_checks or [])
        detail = 'the real-time preflight returned {}'.format(result.overall)
        named = '; '.join(
            '{} \u2014 {}'.format(
                check.get('name') or 'unnamed check',
                check.get('summary') or 'no summary given')
            for check in checks[:2])
        if named:
            detail = '{}: {}'.format(detail, named)
            if len(checks) > 2:
                detail = '{} (and {} more)'.format(detail, len(checks) - 2)
        if result.error:
            detail = '{} ({})'.format(detail, result.error)
        if (not self._settings.franka_dir
                and self._preflight_blames_franka_dir(checks)):
            detail = '{}. {}'.format(detail, _PREFLIGHT_FRANKA_DIR_HINT)
        return detail

    @staticmethod
    def _preflight_blames_franka_dir(checks):
        """
        Say whether any failing check is about identifying libfranka.

        ``evidence`` is read HERE and only here -- for the blame test, never
        for the rendered sentence.
        """
        for check in checks:
            text = ' '.join(str(check.get(field) or '')
                            for field in ('name', 'summary', 'evidence')).lower()
            if any(marker in text for marker in _PREFLIGHT_FRANKA_DIR_MARKERS):
                return True
        return False

    def _enter_starting(self):
        """Spawn the recorder then the launch child; arm the deadline."""
        with self._state_lock:
            session = dict(self._session)
        recorder = self._recording_factory()
        # Adopt the recorder before start. If its refusal cleanup cannot prove
        # the exact process group gone, stopping/shutdown must retain and retry
        # that same owner rather than lose it through a raised start().
        with self._state_lock:
            self._recording = recorder
        try:
            recorder.start(session['session_id'], session['arm_mode'], self._child_env())
        except (RecordingError, LauncherError, OSError) as error:
            # LauncherError/OSError: the spawn layer failed underneath the
            # recorder; same verdict, the session must not start (§5.5).
            self._record_error('recording_failed', str(error))
            self._transition('stopping', reason='recording_failed')
            return
        if session['mode'] == 'motion':
            # Start acceptance and the launch spawn are separated by the
            # potentially 30-second RT preflight plus recorder startup. Re-open
            # the content-addressed file at the last web-owned boundary so a
            # same-user edit in that interval cannot make the console's fence
            # describe A while operator_launch seals valid-but-different B.
            # Keep the failure text constant: filesystem paths and lower-level
            # errors are terminal diagnostics, not state-frame/API material.
            with self._state_lock:
                accepted_record = self._profile_record
            try:
                if accepted_record is None:
                    raise OSError('no rendered profile was accepted')
                self._profile_store.verify(accepted_record)
            except (ProfileStoreError, OSError, AttributeError):
                self._record_error(
                    'profile_invalid',
                    'the controller profile file changed after start '
                    'acceptance; launch was refused')
                self._step_fail(self._first_pending_step(),
                                'the controller profile file changed')
                self._transition('stopping', reason='profile_invalid')
                return
        # The bridge subscriptions must exist before the launch child can
        # publish any activation evidence. Wiring them after spawn left the
        # most important part of the transition unobservable.
        try:
            self._bridge.configure_session(
                session['arm_ids'], session['arm_mode'],
                gripper_arm_ids=self.gripper_arm_ids(session))
            if session['mode'] in ('watch', 'motion'):
                self._bridge.configure_motion(
                    session['arm_ids'], session['controller_name'])
        except Exception as error:  # noqa: BLE001 - refuse before launch
            self._record_error(
                'internal_error',
                'session observation wiring failed before launch: {}'.format(error))
            self._transition('stopping', reason='internal_error')
            return
        if session['mode'] == 'motion':
            try:
                # Fixed-size callback extrema are armed at the last boundary
                # before spawn can activate torque control.
                self._bridge.begin_activation_capture(session['arm_ids'])
            except Exception as error:  # noqa: BLE001 - refuse before launch
                self._record_error(
                    'activation_settling_limit',
                    'activation observation could not be armed before launch: '
                    '{}'.format(error))
                self._transition('stopping', reason='activation_settling_limit')
                return
        try:
            # This established launch call shape selects launcher.py's
            # non-ROS guardian. The guardian gets a parent-death SIGTERM, then
            # broadcasts terminal-equivalent SIGINT to the complete target
            # group before its bounded TERM/KILL escalation.
            launch = self._spawn(session['launch_argv'], self._child_env(), 'launch',
                                 parent_death_signal=signal.SIGINT,
                                 on_line=self._logs.sink('launch'))
        except (LauncherError, OSError) as error:
            self._record_error('launch_failed', 'launch spawn failed: {}'.format(error))
            self._transition('stopping', reason='launch_failed')
            return
        with self._state_lock:
            self._launch = launch
            self._starting_deadline = self._monotonic() + defaults.STARTING_TIMEOUT_S
        self._step_done('preflight')
        self._step_active('connect:' + session['arm_ids'][0])
        self._transition('starting', reason=None)

    def _recorder_tick(self, recorder):
        """
        Tick the recorder; a broken recording chain stops the session.

        "Every session auto-recorded" is an invariant (§5.5): if the
        recorder cannot keep its chain alive, the session ends rather than
        running unrecorded — and the error must never crash the supervisor
        loop (review findings R6/R8).
        """
        try:
            recorder.tick(self._child_env())
            return True
        except (RecordingError, LauncherError, OSError) as error:
            self._record_error('recording_failed', str(error))
            self._transition('stopping', reason='recording_failed')
            return False

    def _poll_starting(self):
        """Advance ``starting``: steps, baseline, readiness, death, timeout."""
        with self._state_lock:
            launch = self._launch
            recorder = self._recording
            deadline = self._starting_deadline
            session = dict(self._session)
        if recorder is not None and not self._recorder_tick(recorder):
            return
        if launch is None or not launch.alive():
            self._record_error('launch_failed', 'the launch child exited during startup')
            self._step_fail(self._first_pending_step(), 'the launch child exited')
            self._transition('stopping', reason='launch_failed')
            return
        self._advance_start_steps(session)
        # Motion captures its baseline inside the restage, where the impedance
        # controller can be PROVEN inactive; every other mode captures from
        # the first fresh complete sample.
        if session['mode'] != 'motion' and not self._capture_baseline(session):
            return
        if self._readiness_met(session):
            if session['mode'] == 'motion':
                # Always leaves `starting`: settling on success, stopping or
                # fault on refusal.
                self._restage_activation(session)
            else:
                self._transition('running', reason=None)
            return
        if self._monotonic() > deadline:
            self._record_error('launch_timeout',
                               'the stack did not become ready within {:.0f} s'.format(
                                   defaults.STARTING_TIMEOUT_S))
            self._step_fail(self._first_pending_step(),
                            'the stack did not become ready in time')
            self._transition('stopping', reason='launch_timeout')

    def _advance_start_steps(self, session):
        """
        Advance the display-only startup checklist. This never gates anything.

        ``health`` completes when every selected arm has a fresh, complete
        joint sample and, in a production mode, no arm reports a diagnostic at
        level >= ERROR. A NEVER-SEEN diagnostic passes: requiring an actual
        ``/diagnostics`` message here would lose a race against the launch's
        own controller spawner on every Motion session. The real gate is
        ``_readiness_met``, which is unchanged and DOES require an actual
        diagnostic sample before ``starting`` can be left.
        """
        arm_ids = session['arm_ids']
        production = session['mode'] in ('watch', 'motion')
        sample = self._bridge.joint_sample()
        now_ns = int(self._monotonic() * 1e9)
        fresh = bool(
            sample is not None and int(sample[0]) <= now_ns
            and now_ns - int(sample[0])
            <= int(defaults.JOINT_STATE_STALE_FAULT_S * 1e9))
        complete = {}
        for arm_id in arm_ids:
            joints = (health.extract_joints(arm_id, sample[1])
                      if sample is not None else {'complete': False})
            complete[arm_id] = bool(joints['complete'])

        for index, arm_id in enumerate(arm_ids):
            connected = complete[arm_id]
            if connected and production:
                connected = self._bridge.robot_state_sample(arm_id) is not None
            if connected:
                self._step_done('connect:' + arm_id)
                if index + 1 < len(arm_ids):
                    self._step_active('connect:' + arm_ids[index + 1])
            else:
                self._step_active('connect:' + arm_id)
                break

        healthy = fresh and all(complete.values())
        if healthy and production:
            for arm_id in arm_ids:
                diagnostic = self._bridge.diagnostic_sample(arm_id)
                if diagnostic is None:
                    continue
                projection = health.project_arm(
                    arm_id, now_ns, sample,
                    self._bridge.robot_state_sample(arm_id), diagnostic)
                level = projection.get('diagnostic', {}).get('level')
                if isinstance(level, int) and not isinstance(level, bool) and level >= 2:
                    healthy = False
                    break
        if healthy:
            self._step_done('health')
        elif self._steps_status('health') == 'pending':
            self._step_active('health')

        # The step that follows `health` is marked active as soon as health is
        # done, so the start-phase hint always has an active step to name.
        # In Motion that step is `stack_ready` -- the launch is still bringing
        # its broadcasters and its own controller up, and marking `baseline`
        # active here would make the hint claim an action that has not
        # started. `controller` is likewise no longer an observation of the
        # launch: it is the server's OWN verified re-activation, marked done
        # by _begin_settling.
        if self._steps_status('health') == 'done':
            self._step_active(
                'stack_ready' if session['mode'] == 'motion' else 'baseline')

    def _capture_baseline(self, session):
        """
        Capture the Simulate/Watch pre-activation pose from a fresh sample.

        Returns True while the session may proceed. Called from every
        ``starting`` tick of a NON-motion session; the step completes on a
        fresh complete sample and nothing else. Motion does not come here at
        all: its baseline is captured inside :meth:`_restage_activation`,
        which first proves the impedance controller inactive -- the
        controller-state test that used to live here now guards the only
        window it can meaningfully guard (see
        :meth:`_capture_activation_baseline`).
        """
        with self._state_lock:
            if self._baseline_captured:
                return True
            if self._steps_status('health') != 'done':
                return True
        sample = self._bridge.joint_sample()
        now_ns = int(self._monotonic() * 1e9)
        fresh = (sample is not None and int(sample[0]) <= now_ns
                 and now_ns - int(sample[0])
                 <= int(defaults.ENABLE_JOINT_STATE_MAX_AGE_S * 1e9))
        if not fresh:
            return True
        baseline = {}
        for arm_id in session['arm_ids']:
            joints = health.extract_joints(arm_id, sample[1])
            positions = (tuple(float(value) for value in joints['positions'])
                         if joints['complete'] else ())
            if (len(positions) != defaults.JOINT_COUNT
                    or not all(math.isfinite(value) for value in positions)):
                return True
            baseline[arm_id] = positions
        with self._state_lock:
            self._activation_baseline = baseline
            self._baseline_captured = True
        self._step_done('baseline')
        return True

    def _baseline_fence_message(self, session, baseline, fences, error):
        """
        Name the offending joint, its measured value and both bounds, in degrees.

        Two shapes: a joint genuinely outside its bounds, and a joint inside
        them but without the reserved activation envelope. The reserve is
        ``fence_margin + drift_limit``, not ``fence_margin`` alone -- naming
        only the margin teaches the operator to raise a value that will still
        be refused.
        """
        policy = session['settling_policy']
        worst = None
        tightest = None
        for arm_id in session['arm_ids']:
            lower, upper = fences[arm_id]
            for joint in range(defaults.JOINT_COUNT):
                value = baseline[arm_id][joint]
                if not lower[joint] <= value <= upper[joint]:
                    if worst is None:
                        worst = (arm_id, joint, value, lower[joint], upper[joint])
                    continue
                clearance = min(value - lower[joint], upper[joint] - value)
                if tightest is None or clearance < tightest[0]:
                    tightest = (clearance, arm_id, joint, value,
                                lower[joint], upper[joint])
        if worst is not None:
            arm_id, joint, value, lower_bound, upper_bound = worst
            return ('{} J{} is at {:.1f}\u00b0, outside its limits '
                    '({:.1f}\u00b0 \u2026 {:.1f}\u00b0). Move the arm back '
                    'inside its range and start again.').format(
                        arm_id, joint + 1, math.degrees(value),
                        math.degrees(lower_bound), math.degrees(upper_bound))
        if tightest is None:
            return str(error)
        clearance, arm_id, joint, value, lower_bound, upper_bound = tightest
        margin = policy.min_fence_margin_rad[joint]
        drift = policy.max_watch_delta_rad[joint]
        return ('{} J{} is at {:.1f}\u00b0, only {:.1f}\u00b0 from its limit '
                '({:.1f}\u00b0 \u2026 {:.1f}\u00b0); the activation envelope '
                'reserves {:.1f}\u00b0 (settling.fence_margin_deg {:.1f}\u00b0 '
                '+ settling.drift_limit_deg {:.1f}\u00b0).').format(
                    arm_id, joint + 1, math.degrees(value),
                    math.degrees(clearance), math.degrees(lower_bound),
                    math.degrees(upper_bound), math.degrees(margin + drift),
                    math.degrees(margin), math.degrees(drift))

    def _restage_activation(self, session):
        """
        Make a torque-free window, measure the resting pose, hand the arms back.

        The reviewed guarded-motion launch activates the impedance controller
        with ``spawner --switch-asap`` BEFORE joint_state_broadcaster
        publishes anything (live evidence 2026-09-01: controller active at
        t+5.96 s, joint states at t+6.38 s), so there is no pre-activation
        window to observe. The server therefore creates one with the same
        ``switch_controller`` machinery Recover uses: deactivate, measure with
        nothing commanding the arms, reactivate, and gate THAT activation.

        No operator command is possible anywhere in here -- the session is
        still ``starting``, every enable flag is false and ``_motion_guards``
        refuses every mutator -- so the launch's own ungated activation can
        never be commanded through this server. The torque-free window is not
        new either: between hardware load and the launch's own activation the
        real arms already stand with nothing writing commands, every session.
        This recreates that same condition deliberately and for a bounded
        time, through calls Recover already makes on the same hardware.

        Always leaves ``starting``: settling on success, stopping or fault on
        refusal.
        """
        controller = session['controller_name']
        steps = []
        self._step_done('stack_ready')

        # A fault that is already firing must not be answered by releasing
        # torque.
        reasons = self._fault_engine.evaluate(self._fault_snapshot(session))
        if reasons:
            self._enter_fault(session, reasons)
            return

        self._step_active('controller_pause')
        self._logs.emit(
            'warn',
            'pausing the impedance controller to measure the resting pose; '
            'the arms hold their position and are briefly movable by hand')

        # The capture armed before launch has been accumulating the LAUNCH's
        # own activation transient. That transient has no pre-activation
        # baseline and cannot be judged; discard it rather than mixing it into
        # the gate that judges ours.
        self._end_activation_capture()

        states = self._bridge.query_controller_states()
        if states is None:
            self._restage_failure(
                session, steps, 'activation_settling_limit',
                'controller-manager state is not reachable, so the impedance '
                'controller could not be paused for the baseline')
            return
        if states.get(controller) == 'active':
            if not self._deactivate_controller_verified(
                    controller, steps, 'restage'):
                self._restage_failure(
                    session, steps, 'activation_settling_limit',
                    'the impedance controller could not be paused, so the '
                    'resting pose could not be measured before torque control '
                    'resumed')
                return
        else:
            steps.append({'step': 'controller_inactive', 'phase': 'restage',
                          'controller': controller, 'ok': True,
                          'detail': 'already inactive'})
        self._step_done('controller_pause')

        self._step_active('baseline')
        baseline = self._capture_activation_baseline(
            session, steps, phase='restage',
            fail=lambda code, detail: self._restage_failure(
                session, steps, code, detail))
        if baseline is None:    # _restage_failure returned instead of raising
            return
        with self._state_lock:
            self._activation_baseline = baseline
            self._baseline_captured = True
        self._step_done('baseline')
        self._logs.emit('info',
                        'pre-activation baseline captured with the impedance '
                        'controller inactive')

        self._step_active('controller')
        # THE live fix (V2L-5): the pause and the hand-back are two controller
        # mode switches, and issuing them ~110 ms apart stalled the driver's
        # read cycle into a fail-safe stop on real hardware. Wait the reviewed
        # spacing out HERE, before the capture is armed, so the capture still
        # opens immediately before the lifecycle call; _switch_activate's own
        # dwell below is then already satisfied and returns at once.
        dwelled_s = self._await_switch_dwell()
        if dwelled_s > 0.0:
            self._logs.emit(
                'info',
                'holding {:.2f} s before restarting the impedance controller '
                'so the control loop settles after the pause'.format(dwelled_s))
        try:
            # Armed under the bridge's callback boundary immediately before
            # the lifecycle call that can resume torque control.
            self._bridge.begin_activation_capture(session['arm_ids'])
        except Exception as error:  # noqa: BLE001 - fail closed
            self._restage_failure(
                session, steps, 'activation_settling_limit',
                'activation observation could not be re-armed before the '
                'controller was restarted: {}'.format(error))
            return
        response = self._switch_activate([controller])
        states = self._bridge.query_controller_states()
        ok = bool(response and response['ok'] and states is not None
                  and states.get(controller) == 'active')
        steps.append({'step': 'controller_active', 'phase': 'restage',
                      'controller': controller, 'ok': ok,
                      'detail': 'active' if ok else 'activation was not verified'})
        if not ok:
            self._restage_failure(
                session, steps, 'activation_settling_limit',
                'the impedance controller did not come back active after the '
                'baseline was measured; the arms were left with no controller '
                'holding them and the session was stopped')
            return

        # NO SetBool(false) here, on purpose: onActivate() has just run
        # disableAndInvalidateAll() and the first RT update captured measured
        # q as the internal target. A redundant false-to-false call would
        # advance enable_generation and rebase it.
        for entry in steps:
            self._logs.emit('info' if entry['ok'] else 'error',
                            'restage: {} {}'.format(entry['step'],
                                                    entry['detail']))
        self._begin_settling(session, baseline=baseline)

    def _restage_failure(self, session, steps, code, detail):
        """
        Fail the start-path restage closed, leaving the arms uncommanded.

        RETURNS rather than raising, unlike ``_recovery_failure``: there is no
        waiting HTTP thread here, and every ``fail(...)`` call site in the
        restage checks for the sentinel and returns.
        """
        self._logs.emit('error', detail)
        for entry in steps:
            self._logs.emit('info' if entry['ok'] else 'error',
                            'restage: {} {}'.format(entry['step'],
                                                    entry['detail']))
        self._record_error(code, detail)
        self._step_fail(self._first_pending_step(), detail)
        self._force_enables_off()
        for model in self._jog_models.values():
            model.invalidate()
        self._end_activation_capture()
        with self._state_lock:
            self._activation_baseline = None
            self._activation_gate = None
            self._settling_target_counts = {}
        if code == 'pose_outside_fence':
            # Same verdict as before the fix: the operator must READ this, so
            # it faults and Stop is the only exit. The launch keeps running
            # and its controller holds the measured pose; what is guaranteed
            # is that no OPERATOR-commanded torque is possible.
            self._enter_fault(session, (FaultReason(
                code='baseline_outside_fence', arm_id=None, detail=detail),))
            return
        self._transition('stopping', reason=code)

    def _begin_settling(self, session, baseline=None):
        """Enter the post-torque-activation gate with every local enable off."""
        with self._state_lock:
            accepted_baseline = baseline or self._activation_baseline
        if accepted_baseline is None:
            # Defensive: config always yields a policy now, but the baseline
            # could still be missing if _capture_baseline never fired.
            self._record_error(
                'activation_settling_limit',
                'Motion activation settling has no captured baseline')
            self._step_fail('settling', 'no pre-activation baseline was captured')
            self._transition('stopping', reason='activation_settling_limit')
            return
        self._step_done('controller')
        self._step_active('settling')

        self._force_enables_off()
        for model in self._jog_models.values():
            model.invalidate()
        # NO controller-side SetBool(false) here, and no probe that could
        # become one.  The reviewed controller's onActivate() already ran
        # disableAndInvalidateAll(), and its first RT update captured measured
        # q as the arm's internal target while synchronising
        # observed_enable_generation.  ArmImpedanceTargetInbox::setEnabled()
        # advances that generation on EVERY call, including false-to-false, so
        # a redundant disable here would make the next RT cycle treat it as a
        # transition and rebase next_targets onto the then-current measured
        # pose -- discarding the restoring spring the activation capture
        # exists to observe (live evidence: Panda 2 J2 in sessions
        # web-20260831-145120 and web-20260831-152313, where desired J2 torque
        # collapsed from -1.088512 Nm to -0.003052 Nm about 40 ms after
        # readiness).  The enable surface is still proven to EXIST before this
        # point: _readiness_met() refuses to leave `starting` at all until
        # every jog slot's enable service is reachable, so the fail-closed
        # property is kept without commanding the controller.

        sample = self._bridge.joint_sample()
        now_ns = int(self._monotonic() * 1e9)
        if (sample is None or int(sample[0]) > now_ns
                or now_ns - int(sample[0])
                > int(defaults.ENABLE_JOINT_STATE_MAX_AGE_S * 1e9)):
            self._fail_settling(
                None, session, 'activation_settling_limit',
                'no fresh joint sample was available at the activation barrier')
            return
        try:
            # The barrier is captured after the current readiness sample.
            # Only a later bridge receipt may count.
            gate = self._install_activation_gate(
                session, accepted_baseline, barrier_ns=now_ns)
        except ValueError as error:
            self._fail_settling(
                None, session, 'activation_settling_limit', str(error))
            return
        capture_verdict = self._observe_activation_capture(gate)
        if capture_verdict.status == 'failed':
            self._end_activation_capture()
            self._fail_settling(gate, session, capture_verdict.code,
                                capture_verdict.detail)
            return
        self._transition('settling', reason=None)

    def _poll_settling(self):
        """Keep every command surface closed until fresh measured motion is stable."""
        with self._state_lock:
            recorder = self._recording
            session = dict(self._session)
            gate = self._activation_gate
            enabled = any(self._arm_enabled.values())
            targets_published = dict(self._targets_published)
            target_barrier = dict(self._settling_target_counts)
        if recorder is not None and not self._recorder_tick(recorder):
            return
        if gate is None:
            self._fail_settling(
                None, session, 'activation_settling_limit',
                'activation settling state has no gate')
            return
        target_traffic = any(
            count > target_barrier.get(arm_id, 0)
            for arm_id, count in targets_published.items())
        if enabled or target_traffic:
            self._force_enables_off()
            self._fail_settling(
                gate, session, 'activation_settling_limit',
                'an enable or target appeared while activation settling was closed')
            return

        sample = self._bridge.joint_sample()
        now_ns = int(self._monotonic() * 1e9)
        # Drain hard transition extrema before applying the time budget. This
        # preserves the most specific safety evidence at the exact deadline.
        capture_verdict = self._observe_activation_capture(gate)
        if capture_verdict.status == 'failed':
            self._end_activation_capture()
            self._fail_settling(gate, session, capture_verdict.code,
                                capture_verdict.detail)
            return
        with self._state_lock:
            deadline_reached = gate.deadline_reached(now_ns)
        if deadline_reached:
            # Close admission, drain once more, apply hard limits, then apply
            # the inclusive deadline as one indivisible exit decision.
            timeout = self._close_activation_capture(gate)
            self._fail_settling(gate, session, timeout.code, timeout.detail)
            return
        verdict = None
        if sample is not None:
            joints = {
                arm_id: health.extract_joints(arm_id, sample[1])
                for arm_id in session['arm_ids']
            }
            with self._state_lock:
                verdict = gate.observe(sample[0], now_ns, joints)
            if verdict.status == 'failed':
                self._end_activation_capture()
                self._fail_settling(gate, session, verdict.code, verdict.detail)
                return

        # A ready-looking gate never overrides an ordinary session fault.  Run
        # the fault engine after the gate's hard-limit checks, then open the
        # command surface only if both verdicts are clean.
        reasons = self._fault_engine.evaluate(self._fault_snapshot(session))
        if reasons:
            self._force_enables_off()
            fault_since = rfc3339(self._utcnow())
            fault_recoverable = self._session_fault_recoverable(
                session, reasons)

            def commit_fault():
                """Publish fault metadata only if capture permits this exit."""
                self._fault_since = fault_since
                self._fault_reasons = tuple(reasons)
                self._fault_recoverable = fault_recoverable

            # A fault snapshot cannot discard a joint callback that was
            # admitted while that snapshot was being evaluated. Close and
            # validate capture before atomically committing the fault state.
            exit_verdict = self._close_activation_capture(
                gate, next_state='fault', commit=commit_fault)
            if exit_verdict.status == 'failed':
                self._fail_settling(gate, session, exit_verdict.code,
                                    exit_verdict.detail)
                return
            self._transition('fault', reason=reasons[0].code)
            return
        if verdict is not None and verdict.status == 'ready':
            # Validate callback extrema and decide close-vs-continue under the
            # bridge's callback lock. No callback can land in a close/re-arm
            # gap, and hard-limit evidence stops the session.
            final_verdict = self._finalize_activation_capture(gate)
            if final_verdict.status == 'failed':
                self._fail_settling(gate, session, final_verdict.code,
                                    final_verdict.detail)
                return
            if final_verdict.status != 'ready':
                return
            # Joint finalization is atomic, but diagnostics, FrankaState, and
            # controller/hardware activity have independent callbacks. Check
            # them again before publishing Running. No supervisor command can
            # interleave inside this tick; the Enable path repeats this guard
            # against a callback arriving after this check.
            post_commit_reasons = self._fault_engine.evaluate(
                self._fault_snapshot(session))
            if post_commit_reasons:
                self._enter_fault(session, post_commit_reasons)
                return
            detail = self._settling_success_detail(gate, session)
            self._step_done('settling', detail)
            self._logs.emit('info', detail)
            self._transition('running', reason=None)

    # ------------------------------------------------------------------
    # Settling messages: teach the operator which key to raise
    # ------------------------------------------------------------------

    def _fail_settling(self, gate, session, code, detail):
        """Record a teaching settling refusal, fail the step, and stop."""
        message = self._settling_trip_message(gate, session, code, detail)
        self._record_error(code, message)
        self._step_fail('settling', message)
        self._logs.emit('error', message)
        self._transition('stopping', reason=code)

    def _settling_metrics(self, gate):
        """Return the gate's per-arm metric dict, or an empty one."""
        if gate is None:
            return {}
        try:
            with self._state_lock:
                return {arm_id: {name: list(values)
                                 for name, values in metrics.items()}
                        for arm_id, metrics in gate.current.items()}
        except Exception:  # noqa: BLE001 - a message never breaks the machine
            return {}

    def _settling_worst(self, gate, session):
        """
        Return ``(arm_id, joint, family, measured, limit)`` for the worst joint.

        Severity is ``measured / limit`` for the three "must stay below"
        families and ``limit / measured`` for the fence margin, which must
        stay ABOVE its limit. Ties break by arm order, then joint, then the
        family order of ``_SETTLING_KEYS``.
        """
        metrics = self._settling_metrics(gate)
        if not metrics:
            return None
        settling = self._settings.settling
        best = None
        for arm_id in session['arm_ids']:
            values = metrics.get(arm_id)
            if not values:
                continue
            for joint in range(defaults.JOINT_COUNT):
                for order, family in enumerate(_SETTLING_KEYS):
                    metric, attribute, _key, _unit, direction, _verb = family
                    limits = getattr(settling, attribute, None)
                    if limits is None:
                        continue
                    limit = float(limits[joint])
                    measured = self._settling_measured(values, metric, joint)
                    if measured is None:
                        continue
                    if direction == 'below':
                        severity = measured / limit if limit > 0.0 else float('inf')
                    else:
                        severity = (limit / measured if measured > 0.0
                                    else float('inf'))
                    candidate = (severity, arm_id, joint, order, measured, limit)
                    if best is None or severity > best[0]:
                        best = candidate
        if best is None:
            return None
        _severity, arm_id, joint, order, measured, limit = best
        return arm_id, joint, _SETTLING_KEYS[order], measured, limit

    @staticmethod
    def _settling_measured(values, metric, joint):
        """Return one metric's value for one joint, or None when absent."""
        if metric == '_fence_margin':
            lower = values.get('lower_margin_rad')
            upper = values.get('upper_margin_rad')
            candidates = [entry[joint] for entry in (lower, upper)
                          if entry is not None and entry[joint] is not None]
            return min(candidates) if candidates else None
        entry = values.get(metric)
        if entry is None or entry[joint] is None:
            return None
        return abs(float(entry[joint]))

    def _settling_where(self, key):
        """Name the config key and the file the operator should edit."""
        path = getattr(self._settings, 'config_path', None)
        if not getattr(self._settings, 'config_present', False):
            return 'create {} and set {}'.format(path, key)
        return '{} in {}'.format(key, path)

    def _settling_trip_message(self, gate, session, code, detail):
        """Compose the teaching sentence for one settling refusal."""
        timeout = code == 'activation_settling_timeout'
        worst = self._settling_worst(gate, session)
        if worst is None:
            return ('{}. If the arms need longer to settle, raise {}.'.format(
                detail, self._settling_where('settling.timeout_s')))
        arm_id, joint, family, measured, limit = worst
        _metric, _attribute, key, unit, direction, verb = family
        scale = math.degrees(1.0)
        if direction == 'below':
            body = '{} J{} {} {:.2f}{}; the limit is {:.2f}{} ({}).'.format(
                arm_id, joint + 1, verb, measured * scale, unit,
                limit * scale, unit, self._settling_where(key))
        else:
            body = ('{} J{} {} {:.2f}{} of its limit; at least {:.2f}{} is '
                    'required ({}).').format(
                        arm_id, joint + 1, verb, measured * scale, unit,
                        limit * scale, unit, self._settling_where(key))
        if timeout:
            tail = ('Raise settling.timeout_s, or lower settling.min_samples / '
                    'settling.stable_window_s.')
        else:
            tail = 'Check that nothing is pushing the arm, or raise that value.'
        return '{} {}'.format(body, tail)

    def _settling_success_detail(self, gate, session):
        """Return the worst-joint sentence a passing settling check reports."""
        metrics = self._settling_metrics(gate)
        settling = self._settings.settling
        best = None
        for arm_id in session['arm_ids']:
            values = metrics.get(arm_id)
            if not values:
                continue
            deltas = values.get('max_abs_delta_rad')
            latest = values.get('delta_rad')
            if deltas is None:
                continue
            for joint in range(defaults.JOINT_COUNT):
                magnitude = deltas[joint]
                if magnitude is None:
                    continue
                if best is None or abs(magnitude) > best[0]:
                    signed = (latest[joint] if latest is not None
                              and latest[joint] is not None else magnitude)
                    best = (abs(magnitude), arm_id, joint, signed)
        if best is None:
            return 'settled within the configured limits'
        _magnitude, arm_id, joint, signed = best
        return '{} J{} settled {:+.2f}\u00b0 (limit {:.2f}\u00b0)'.format(
            arm_id, joint + 1, math.degrees(signed),
            math.degrees(settling.drift_limit_rad[joint]))

    def _readiness_met(self, session):
        """Check the §3.5 readiness criteria for the session's mode."""
        controllers = self._bridge.controller_states()
        if controllers.get('joint_state_broadcaster') != 'active':
            return False
        joint_sample = self._bridge.joint_sample()
        if joint_sample is None:
            return False
        for arm_id in session['arm_ids']:
            if not health.extract_joints(arm_id, joint_sample[1])['complete']:
                return False
        if session['mode'] == 'simulate':
            return True
        for name in (expected_state_broadcasters(
                session['arm_ids'], session['arm_mode'])
                + expected_model_broadcasters(
                    session['arm_ids'], session['arm_mode'])):
            if controllers.get(name) != 'active':
                return False
        for arm_id in session['arm_ids']:
            if self._bridge.robot_state_sample(arm_id) is None:
                return False
            if self._bridge.diagnostic_sample(arm_id) is None:
                return False
        if session['mode'] == 'motion':
            if controllers.get(session['controller_name']) != 'active':
                return False
            with self._state_lock:
                slots = list(self._arm_slots.values())
            for slot in slots:
                if not self._bridge.enable_service_ready(slot):
                    return False
        return True

    def _poll_running(self):
        """Advance ``running``: recorder chaining and fault evaluation."""
        with self._state_lock:
            recorder = self._recording
            session = dict(self._session)
        if recorder is not None and not self._recorder_tick(recorder):
            return
        snapshot = self._fault_snapshot(session)
        reasons = self._fault_engine.evaluate(snapshot)
        if reasons:
            self._enter_fault(session, reasons)

    def _enter_fault(self, session, reasons):
        """Latch one fresh fault snapshot with every local enable already off."""
        self._force_enables_off()
        for model in self._jog_models.values():
            model.invalidate()
        # A NEW fault episode starts with no recovery in progress, so it must
        # start with no recovery checklist for the console to read as one.
        self._discard_stale_recovery_steps()
        with self._state_lock:
            self._fault_since = rfc3339(self._utcnow())
            self._fault_reasons = tuple(reasons)
            recoverable = self._session_fault_recoverable(session, reasons)
            self._fault_recoverable = recoverable
        self._logs.emit('error', 'fault: {}'.format(
            self._fault_headline(session, reasons, recoverable)))
        self._transition('fault', reason=reasons[0].code)

    def _fault_headline(self, session, reasons, recoverable):
        """Return the plain-words headline the console will show."""
        try:
            operator = self._lock_service.state()
            return classify_fault(
                reasons=reasons, active=True,
                arm_ids=session.get('arm_ids', ()),
                recoverable=recoverable,
                operator_locked=bool(operator.get('locked')),
                operator_claim_id=operator.get('claim_id'),
                session_claim_id=session.get('operator_claim_id'),
            )['headline']
        except Exception:  # noqa: BLE001 - a log line never breaks a fault
            return reasons[0].detail if reasons else 'the session faulted'

    def _poll_fault(self):
        """Keep the recorder alive while faulted; leave fault after recovery."""
        with self._state_lock:
            recorder = self._recording
            recovered = self._recover_succeeded
            session = dict(self._session) if self._session else None
        if recorder is not None:
            self._recorder_tick(recorder)
        if not recovered or session is None or self.state != 'fault':
            return
        # §7.3: leave fault only when the whole recover sequence succeeded AND
        # the fault rules stop firing on the next tick.  Re-activating an
        # impedance controller must pass the same closed-command settling gate
        # before it can return to running.
        reasons = self._fault_engine.evaluate(self._fault_snapshot(session))
        with self._state_lock:
            self._recover_succeeded = False
            if reasons:
                self._fault_reasons = tuple(reasons)
                self._fault_recoverable = self._session_fault_recoverable(
                    session, reasons)
        if reasons:
            # The synchronous recovery proved a healthy snapshot, but a new
            # fault won the mandatory next-tick check. Retire that attempt's
            # transition capture and gate now. Otherwise another authorized
            # Recover would collide with the still-armed old capture instead
            # of getting one fresh, continuous activation identity.
            self._end_activation_capture()
            with self._state_lock:
                self._activation_baseline = None
                self._activation_gate = None
                self._settling_target_counts = {}
            return
        if session['mode'] == 'motion':
            with self._state_lock:
                gate_ready = self._activation_gate is not None
            if not gate_ready:
                self._record_error(
                    'recovery_failed',
                    'recovery completed without a fresh activation-settling gate')
                return
            with self._state_lock:
                self._clear_fault()
            self._transition('settling', reason=None)
            return
        with self._state_lock:
            self._clear_fault()
        self._transition('running', reason=None)

    def _fault_snapshot(self, session):
        """Assemble the FaultSnapshot the engine evaluates each tick."""
        now_ns = int(self._monotonic() * 1e9)
        arms = {
            arm_id: health.project_arm(
                arm_id, now_ns,
                self._bridge.joint_sample(),
                self._bridge.robot_state_sample(arm_id),
                self._bridge.diagnostic_sample(arm_id))
            for arm_id in session['arm_ids']
        }
        component = self._bridge.hardware_component()
        with self._state_lock:
            launch = self._launch
        return FaultSnapshot(
            mode=session['mode'],
            arms=arms,
            controller_name=session['controller_name'],
            controller_states=self._bridge.controller_states(),
            hardware_available=component is not None,
            hardware_lifecycle_label=(
                component['lifecycle_label'] if component else None),
            launch_alive=bool(launch is not None and launch.alive()),
        )

    def _do_stopping(self):
        """
        Tear down: enables off, seal the recorder, stop the launch, reap.

        Every step is individually guarded: a recorder that will not seal
        must never leave the launch child running with the machine stuck in
        ``stopping`` (review finding R7). Failures are recorded, teardown
        continues.
        """
        self._force_enables_off()
        self._force_sources_jog()
        self._reconcile_external_counters()
        with self._state_lock:
            recorder = self._recording
            launch = self._launch
        failures = []
        recorder_stopped = recorder is None
        if recorder is not None:
            try:
                recorder.stop()
                recorder_stopped = True
                # A name exists only once a segment was really started, so
                # this is the one honest answer to "was anything saved?".
                sealed = bool(recorder.frame(()).get('name'))
                with self._state_lock:
                    self._recording_sealed = sealed
            except Exception as error:
                failures.append('recorder stop failed: {}'.format(error))
        launch_stopped = launch is None
        if launch is not None:
            try:
                # Always call stop, even when the guardian leader already
                # exited: guarded ChildProcess.stop independently proves the
                # immutable target process group is empty.
                launch.stop(defaults.STOP_SIGINT_WAIT_S,
                            defaults.STOP_SIGTERM_WAIT_S,
                            defaults.STOP_SIGKILL_WAIT_S)
                launch_stopped = True
            except Exception as error:
                failures.append('launch stop failed: {}'.format(error))
        try:
            self._bridge.clear_motion()
        except Exception as error:
            failures.append('motion teardown failed: {}'.format(error))
        try:
            self._bridge.clear_session()
        except Exception as error:
            failures.append('bridge teardown failed: {}'.format(error))
        if failures:
            self._record_error('internal_error', '; '.join(failures))
        with self._state_lock:
            # A failed recorder ladder retains the exact child and keeps the
            # session in stopping. This prevents both pidfile release and a
            # replacement session that could be cross-captured by a survivor.
            if recorder_stopped:
                self._recording = None
            # Never discard the exact target identity on a failed stop. Keeping
            # it also keeps the machine in stopping, so a later tick/shutdown
            # retries and no new session can be accepted through a survivor.
            if launch_stopped:
                self._launch = None
            self._starting_deadline = None
            # A stopped session has no motion surface: keeping the models
            # alive let the lock-expiry path enqueue disable commands for a
            # dead session forever (review finding S2b).
            self._jog_models = {}
            self._arm_slots = {}
            if self._session is not None:
                # Freeze the session clock: a stopped session's uptime must
                # not keep counting (review finding R19).
                self._session.setdefault('ended_mono', self._monotonic())
        if not recorder_stopped or not launch_stopped:
            return
        with self._state_lock:
            session_id = (self._session['session_id']
                          if self._session is not None else None)
            self._steps = []
        if session_id is not None:
            self._logs.emit('info', 'session {} stopped'.format(session_id))
        self._transition('stopped', reason=None)

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    def _child_env(self):
        """Build the child environment per §3.3 (never invents DDS settings)."""
        env = dict(os.environ)
        env['ROS_DOMAIN_ID'] = str(self._settings.ros_domain_id)
        state_dir = self._settings.state_dir
        session_id = ''
        with self._state_lock:
            if self._session:
                session_id = self._session['session_id']
        if session_id:
            env['ROS_HOME'] = '{}/ros_home/{}'.format(state_dir, session_id)
            env['ROS_LOG_DIR'] = '{}/ros_logs/{}'.format(state_dir, session_id)
            os.makedirs(env['ROS_HOME'], mode=0o700, exist_ok=True)
            os.makedirs(env['ROS_LOG_DIR'], mode=0o700, exist_ok=True)
        return env

    def _force_enables_off(self):
        """Force every arm's enable flag off (first action of fault/stopping)."""
        with self._state_lock:
            for arm_id in self._arm_enabled:
                self._arm_enabled[arm_id] = False

    def _clear_fault(self):
        """Reset the fault block (new session)."""
        self._fault_since = None
        self._fault_reasons = ()
        self._fault_recoverable = False

    def _record_error(self, code, detail):
        """Record the session's last_error for the frame and the UI."""
        with self._state_lock:
            if self._session is not None:
                self._session['last_error'] = {'code': code, 'detail': detail}

    def _transition(self, new_state, reason):
        """Commit a state change and push an immediate frame (§6.11)."""
        with self._state_lock:
            if new_state not in STATES:
                raise ValueError('unknown state {!r}'.format(new_state))
            if new_state in ('fault', 'stopping'):
                # Fault entry and teardown both revoke every authorization:
                # enables off, every command source back to jog, and every
                # travel cleared. A fault means the console no longer believes
                # its own picture of the cell, and continuing a checked path on
                # a disbelieved picture is the worst available option.
                for arm_id in self._arm_enabled:
                    self._arm_enabled[arm_id] = False
                for arm_id in self._arm_source:
                    self._arm_source[arm_id] = 'jog'
                self._clear_every_travel_locked()
            self._state = new_state
        if self._broker is not None:
            self._broker.publish('state', self.frame())

    # ------------------------------------------------------------------
    # The state frame (§6.11) — callable from any thread
    # ------------------------------------------------------------------

    def frame(self):
        """Build the full normative state frame."""
        now = self._utcnow()
        with self._state_lock:
            state = self._state
            session = dict(self._session) if self._session else None
            preflight = self._preflight_result
            recorder = self._recording
            launch = self._launch
            fault_since = self._fault_since
            fault_reasons = self._fault_reasons
            fault_recoverable = self._fault_recoverable
        operator = self._lock_service.state()
        session_block = self._session_block(state, session, launch)
        arms_block = self._arms_block(state, session)
        classification = classify_fault(
            reasons=fault_reasons,
            active=state == 'fault',
            arm_ids=(session['arm_ids'] if session else ()),
            recoverable=fault_recoverable,
            operator_locked=bool(operator.get('locked')),
            operator_claim_id=operator.get('claim_id'),
            session_claim_id=(session.get('operator_claim_id')
                              if session else None),
        )
        fault_block = {
            'active': state == 'fault',
            'since': fault_since,
            'reasons': [reason.as_dict() for reason in fault_reasons],
            'recoverable': fault_recoverable,
            'recover_hint': ('release the physical stop first, then recover'
                             if (state == 'fault' and fault_recoverable) else None),
        }
        fault_block.update(classification)
        return {
            'schema_version': defaults.SCHEMA_VERSION,
            'server_time': rfc3339(now),
            'server_uptime_s': round(self._monotonic() - self._started_mono, 3),
            'session': session_block,
            'operator': operator,
            'preflight': (preflight.frame() if preflight is not None else
                          {'ran_at': None, 'overall': None,
                           'blocking': False, 'failed_checks': []}),
            'recording': (recorder.frame(topics_for(session['arm_mode']))
                          if (recorder is not None and session) else
                          {'active': False,
                           'disabled': not self._settings.recording_enabled,
                           'name': None, 'sequence': 0,
                           'path': None, 'arm_mode': None, 'topics': []}),
            'controllers': self._controllers_block(state),
            'hardware': self._hardware_block(state),
            'fault': fault_block,
            'arms': arms_block,
            'hint': self._hint(state, session, fault_block, arms_block, operator),
            'logs': self._logs.counters(),
        }

    def _session_block(self, state, session, launch):
        """Build the §6.11 session sub-object."""
        with self._state_lock:
            recording_sealed = self._recording_sealed
        block = {
            'state': state,
            'session_id': None,
            'arms': None,
            'arm_ids': [],
            'arm_mode': None,
            'mode': None,
            'started_at': None,
            'uptime_s': None,
            'launch_running': bool(launch is not None and launch.alive()),
            'last_error': None,
            # THE SAME evidence `_hint` branches on, published so the stopped
            # card cannot say "the recording was saved" about a session that
            # sealed nothing. `recording.disabled` answers a different
            # question (is recording switched off in the config?) and was the
            # wrong key: a start refused at preflight adopts no recorder at
            # all, so it saves nothing while recording stays enabled.
            'recording_sealed': bool(recording_sealed),
            'advisory': defaults.STOP_ADVISORY,
            'activation': self._activation_block(state, session),
            'steps': self._steps_frame(),
        }
        if session is not None:
            block.update({
                'session_id': session['session_id'],
                'arms': session['arms'],
                'arm_ids': list(session['arm_ids']),
                'arm_mode': session['arm_mode'],
                'mode': session['mode'],
                'started_at': session['started_at'],
                'uptime_s': round(
                    session.get('ended_mono', self._monotonic())
                    - session['started_mono'], 3),
                'last_error': session['last_error'],
            })
        return block

    def _activation_block(self, state, session):
        """Build bounded evidence for the fail-closed torque-activation gate."""
        required = bool(session is not None and session['mode'] == 'motion')
        policy = session.get('settling_policy') if session else None
        block = {
            'required': required,
            'status': 'waiting' if required else 'not_applicable',
            'torque_control_active': bool(
                required and state in (
                    'starting', 'settling', 'running', 'fault', 'stopping')),
            'policy_sha256': (
                session.get('activation_policy_sha256') if required else None),
            'samples': 0,
            'transition_samples': 0,
            'stable_samples': 0,
            'stable_for_s': 0.0,
            'required_stable_s': (
                policy.stable_window_s if (required and policy is not None)
                else None),
            'required_samples': (
                policy.min_sample_count if (required and policy is not None)
                else None),
        }
        with self._state_lock:
            gate = self._activation_gate
            if required and gate is not None:
                block.update(gate.frame())
        return block

    def _controllers_block(self, state):
        """Build the §6.11 controllers list from the bridge caches."""
        if state == 'stopped':
            return []
        states = self._bridge.controller_states()
        types = self._bridge.controller_types()
        return [{'name': name, 'type': types.get(name), 'state': lifecycle}
                for name, lifecycle in sorted(states.items())]

    def _hardware_block(self, state):
        """Build the §6.11 hardware sub-object."""
        component = self._bridge.hardware_component() if state != 'stopped' else None
        if component is None:
            return {'available': False, 'name': None, 'plugin_name': None,
                    'lifecycle_id': None, 'lifecycle_label': None}
        return {
            'available': True,
            'name': component.get('name'),
            'plugin_name': component.get('plugin_name'),
            'lifecycle_id': component.get('lifecycle_id'),
            'lifecycle_label': component.get('lifecycle_label'),
        }

    def _arms_block(self, state, session):
        """Build the per-arm §6.11 dict; empty in ``stopped`` (frame rule 1)."""
        if state == 'stopped' or session is None:
            return {}
        joint_sample = self._bridge.joint_sample()
        now = self._monotonic()
        now_ns = int(now * 1e9)
        # The bridge timestamps receipt with this same monotonic clock.  By
        # sampling the bridge first, any later timestamp is genuinely
        # impossible rather than a frame/ROS-thread ordering artifact.  Fail
        # closed in the public frame so neither the summary nor per-joint UI
        # can render a future sample as verified.
        projected_joint_sample = joint_sample
        if joint_sample is not None and int(joint_sample[0]) > now_ns:
            projected_joint_sample = None
        arms = {}
        with self._state_lock:
            enabled = dict(self._arm_enabled)
            sources = dict(self._arm_source)
            travels = dict(self._arm_travel)
            record = self._profile_record
            models = dict(self._jog_models)
            slots = dict(self._arm_slots)
            published = dict(self._targets_published)
            last_publish = dict(self._last_publish_mono)
            activation_metrics = {
                arm_id: {name: list(values) for name, values in metrics.items()}
                for arm_id, metrics in (
                    self._activation_gate.current.items()
                    if self._activation_gate is not None else ())
            }
        # The same question the Apply handler asks, asked here per arm: "note
        # is non-null exactly when no Apply can start" is only one rule if
        # both sides read the same answer. The checker caches its loaded
        # model, so this costs a dict lookup per frame.
        profile = self._checker_profile(session)
        motion_mode = session['mode'] == 'motion'
        for arm_id in session['arm_ids']:
            refusal = (self._checker_refusal(profile, arm_id)
                       if motion_mode else None)
            apply_note = None if refusal is None else refusal[0]
            projection = health.project_arm(
                arm_id, now_ns,
                projected_joint_sample,
                self._bridge.robot_state_sample(arm_id),
                self._bridge.diagnostic_sample(arm_id))
            model = models.get(arm_id)
            # Only a Motion session has a materialized profile, so Simulate
            # and Watch frames report null fences by construction.
            fence = (record.fence.get(arm_id)
                     if (record is not None and session['mode'] == 'motion')
                     else None)
            pose_inside = None
            if (fence and fence.get('position_lower') and projection['positions']
                    and not projection['positions_stale']):
                if all(p is not None for p in projection['positions']):
                    pose_inside = not _joints_outside_fence(
                        projection['positions'], fence['position_lower'],
                        fence['position_upper'])
            last_age = None
            if arm_id in last_publish:
                last_age = round(now - last_publish[arm_id], 3)
            projection['motion'] = {
                'available': model is not None and state == 'running',
                'enabled': enabled.get(arm_id, False),
                'target': list(model.target) if (model and model.target) else None,
                'fence_lower': list(fence['position_lower'])
                if fence and fence.get('position_lower') else None,
                'fence_upper': list(fence['position_upper'])
                if fence and fence.get('position_upper') else None,
                'max_target_velocity': list(fence['max_target_velocity'])
                if fence and fence.get('max_target_velocity') else None,
                'pose_inside_fence': pose_inside,
                'targets_published': published.get(arm_id, 0),
                'last_publish_age_s': last_age,
                'enable_service_available': (
                    self._bridge.enable_service_ready(slots[arm_id])
                    if (model is not None and arm_id in slots) else False),
                'activation_delta_rad': activation_metrics.get(
                    arm_id, {}).get('delta_rad'),
                'activation_max_abs_delta_rad': activation_metrics.get(
                    arm_id, {}).get('max_abs_delta_rad'),
                'activation_abs_velocity_rad_s': activation_metrics.get(
                    arm_id, {}).get('abs_velocity_rad_s'),
                'activation_max_abs_velocity_rad_s': activation_metrics.get(
                    arm_id, {}).get('max_abs_velocity_rad_s'),
                'activation_position_span_rad': activation_metrics.get(
                    arm_id, {}).get('position_span_rad'),
                'activation_lower_margin_rad': activation_metrics.get(
                    arm_id, {}).get('lower_margin_rad'),
                'activation_upper_margin_rad': activation_metrics.get(
                    arm_id, {}).get('upper_margin_rad'),
            }
            in_motion = session['mode'] == 'motion'
            source = sources.get(arm_id) if in_motion else None
            rate = (self._bridge.external_rate_hz(arm_id, now_ns)
                    if source == 'external' else None)
            ready = bool(
                in_motion and projection['positions']
                and not projection['positions_stale']
                and all(value is not None for value in projection['positions']))
            projection['motion'].update({
                'source': source,
                # 0.0, not null, for a configured-but-silent arm: that
                # distinction is what drives the console's "waiting for your
                # publisher ... 0 Hz" hint. The None guard is load-bearing --
                # _reconcile_external_counters swallows its own failure by
                # design, so there is a real window in which the source is
                # external and the bridge is not yet counting, and an
                # unguarded round(None, 1) would take down every frame.
                'external_rate_hz': (
                    round(rate, 1) if rate is not None
                    else (0.0 if source == 'external' else None)),
                'command_topic': (self._command_topic(slots.get(arm_id))
                                  if in_motion else None),
                'command_template': (
                    self._command_template(arm_id, projection, ready)
                    if in_motion else None),
                'command_template_ready': ready,
                'apply': self._apply_block(
                    projection['motion']['available'],
                    travels.get(arm_id), apply_note),
            })
            # Simulate gets NO gripper surface -- not read-only, absent.
            # `configured` is false for every arm in Simulate REGARDLESS of
            # the config file, and the page renders no row at all when it is
            # false. The short-circuit is here, before the bridge is
            # consulted, so there is one place and one rule.
            configured = (session['mode'] != 'simulate'
                          and self._settings.gripper(arm_id).enabled)
            # `busy` is the OR of the bridge's in-flight flag (with its
            # GRIPPER_BUSY_MAX_S force-clear) and the projected `moving`; the
            # OR itself is specified and computed inside project_gripper, so
            # the frame has exactly one authority for the key. This line only
            # says what the caller hands in.
            projection['gripper'] = health.project_gripper(
                arm_id, now_ns,
                (self._bridge.gripper_status_sample(arm_id)
                 if configured else None),
                configured=configured,
                busy=self._bridge.gripper_busy(arm_id, now))
            arms[arm_id] = projection
        return arms

    @staticmethod
    def _apply_block(available, plan, note):
        """
        Build one arm's ``motion.apply`` block: the Apply surface, in the frame.

        ``available`` says the surface EXISTS for this arm -- exactly
        ``motion.available`` -- and says nothing about whether a particular
        Apply would be accepted. ``note`` is a SERVER-AUTHORED sentence,
        non-null exactly when the surface exists but no Apply can start for a
        checker-side reason, so the four checker sentences keep their one
        author and the page holds no copy of any of them.

        ``state`` has two values and there is deliberately no ``"cancelling"``:
        Cancel clears the plan synchronously on the HTTP thread before it
        answers, so by the time the browser has its 200 the next frame already
        says ``idle``. A transient third state would be a state the operator
        could see but not act on.

        ``goal`` is the APPLIED pose, frozen at Apply time. It is not the
        ghost, which is client-side only and which G3 neither stores nor
        broadcasts; it is a server-side fact about a motion in progress, and it
        is what makes "reality is going THERE, my ghost is HERE" legible.
        """
        live = plan is not None and plan.live
        return {
            'available': bool(available),
            'state': 'travelling' if live else 'idle',
            'fraction': round(plan.fraction, 4) if live else None,
            'steps_done': plan.step if live else None,
            'steps_total': plan.steps_total if live else None,
            # An ESTIMATE, and the copy never calls it anything else: ticks can
            # be late, and a late tick makes the travel longer.
            'seconds_remaining': (round(plan.seconds_remaining, 2)
                                  if live else None),
            'goal': list(plan.q1) if live else None,
            'note': note if available else None,
        }

    @staticmethod
    def _command_topic(slot):
        """Return the arm's ``joint_target`` topic, or None without a slot."""
        if slot is None:
            return None
        return '/{}/arm_{}/joint_target'.format(defaults.MOTION_CONTROLLER, slot)

    @staticmethod
    def _command_template(arm_id, projection, ready):
        """
        Build the copyable JointTrajectory template for one arm.

        The ``positions`` line carries the LATEST MEASURED pose, rounded to
        three decimals, so a copy-paste publishes a no-op rather than a jump.
        """
        names = health.joint_names_for(arm_id)
        if ready:
            positions = ', '.join(
                '{:.3f}'.format(float(value)) for value in projection['positions'])
            comment = _READY_COMMENT
        else:
            positions = _UNREADY_POSITIONS
            comment = _UNREADY_COMMENT
        return _TEMPLATE_HEAD.format(
            n0=names[0], n1=names[1], n2=names[2], n3=names[3],
            n4=names[4], n5=names[5], n6=names[6],
            positions=positions, comment=comment)

    # ------------------------------------------------------------------
    # The persistent next-step hint
    # ------------------------------------------------------------------

    def _hint(self, state, session, fault_block, arms_block, operator):
        """
        Return the persistent next-step sentence, top to bottom, first match.

        Computed here and rendered verbatim by the page: one owner, no drift.
        """
        if state == 'stopped':
            if session is None:
                return _HINT_IDLE
            # Keyed on whether a recording SEALED, not on the policy: a start
            # refused at preflight adopted no recorder and saved nothing, and
            # telling that operator "Recording saved" is simply false.
            with self._state_lock:
                sealed = self._recording_sealed
            return _HINT_ENDED_RECORDED if sealed else _HINT_ENDED
        if state in ('preflight', 'starting', 'settling'):
            return self._step_hint()
        if state == 'stopping':
            return _HINT_STOPPING
        if state == 'fault':
            return (_HINT_FAULT_RECLAIM
                    if fault_block.get('cause') == 'lock_expired' else _HINT_FAULT)
        if state != 'running' or session is None:
            return _HINT_IDLE
        if session['mode'] == 'watch':
            return _HINT_WATCH
        if not operator.get('locked'):
            # Nobody holds control: the page claims lazily on the first
            # mutating action, so the enable row is still the next step.
            pass
        elif (session.get('operator_claim_id') is not None
                and operator.get('claim_id') != session.get('operator_claim_id')):
            return _HINT_OTHER_OPERATOR
        enabled = [arm_id for arm_id in session['arm_ids']
                   if arms_block.get(arm_id, {}).get('motion', {}).get('enabled')]
        if not enabled:
            return _HINT_NO_ENABLE
        # The ghost rows come FIRST, and the travelling one first of those: an
        # arm that is moving under a checked path is the most urgent thing on
        # the screen, and the sentence names the control that stops it.
        for arm_id in enabled:
            apply_block = arms_block[arm_id]['motion'].get('apply') or {}
            if apply_block.get('state') == 'travelling':
                return _HINT_APPLYING.format(arm=arm_id)
        for arm_id in enabled:
            if arms_block[arm_id]['motion'].get('source') == 'ghost':
                return _HINT_GHOST
        for arm_id in enabled:
            motion = arms_block[arm_id]['motion']
            if motion.get('source') != 'external':
                continue
            rate = motion.get('external_rate_hz') or 0.0
            rendered = '{:.1f} Hz'.format(rate)
            if rate < _EXTERNAL_RATE_FLOOR_HZ:
                return _HINT_WAITING.format(
                    topic=motion.get('command_topic'), rate=rendered)
            return _HINT_RECEIVING.format(rate=rendered)
        return _HINT_JOG

    def _step_hint(self):
        """
        Return the sentence of the step the checklist is currently on.

        Keyed by step ID against the normative table, never by display label:
        three of the sentences are not a transform of their label at all
        ('Connect panda1' against 'Connecting to panda1...'). When no step is
        active, the first pending step's sentence is used; when every step is
        done, the last step's.
        """
        with self._state_lock:
            steps = list(self._steps)
        if not steps:
            return _HINT_IDLE
        chosen = None
        for step in steps:
            if step['status'] == 'active':
                chosen = step
                break
        if chosen is None:
            for step in steps:
                if step['status'] == 'pending':
                    chosen = step
                    break
        if chosen is None:
            chosen = steps[-1]
        # The 'reconnect:' prefix on steps[0] is the stable, contract-
        # guaranteed discriminator between a startup checklist and a recovery
        # checklist. PART2 keeps state == 'fault' for the whole of a Recover,
        # so the two fault rows of the hint table win and these recovery
        # sentences are in practice unreachable through _hint -- they are
        # built and unit-tested anyway, so the day somebody moves the session
        # out of 'fault' during a recovery is a one-line change.
        recovery = steps[0]['id'].startswith('reconnect:')
        return self._step_sentence(chosen['id'], recovery=recovery)

    @staticmethod
    def _step_sentence(step_id, recovery=False):
        """Render one step id as its hint sentence."""
        prefix, _, arm_id = step_id.partition(':')
        table = _RECOVERY_STEP_HINTS if recovery else _STEP_HINTS
        sentence = table.get(prefix) or _STEP_HINTS.get(prefix) or _HINT_IDLE
        return sentence.format(arm=arm_id) if '{arm}' in sentence else sentence
