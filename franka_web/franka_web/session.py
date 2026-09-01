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
"""

from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
import os
import queue
import signal
import threading
import time

from franka_web import config, health
from franka_web.faults import FaultEngine, FaultSnapshot
from franka_web.jog import JogError, JogTargetModel
from franka_web.launcher import ChildProcess, LauncherError
from franka_web.preflight import run_preflight
from franka_web.profiles import argv_for, ProfileError, PROFILES
from franka_web.recording import (
    RecordingError, RecordingSupervisor, session_name, topics_for)
from franka_web.settling import ActivationSettlingGate

STATES = ('stopped', 'preflight', 'starting', 'settling', 'running', 'fault', 'stopping')

_VALID_ARMS = ('panda1', 'panda2', 'both')
_VALID_MODES = ('simulate', 'watch', 'motion')

# Worst-case dual impedance recovery is 149 s with the bridge's 5 s service
# bound: fresh controller query (5), two pre-disables (10), pre-deactivate and
# verify (10), both ErrorRecovery calls (10), hardware query/set/verify (15),
# controller query plus six activate/verify pairs (65), fresh state wait (5),
# final hardware/controller queries (10), and fail-closed rollback (19). There
# are no post-reactivation disables: onActivate() already left every inbox
# disabled, so commanding it again would only rebase the captured activation
# target. The public wait includes scheduling margin. Once a command is
# taken, _submit() waits for its actual verdict rather than ever reporting a
# timeout while recovery is still changing controller-manager state.
RECOVERY_FRESH_STATE_TIMEOUT_S = 5.0
RECOVERY_REQUEST_TIMEOUT_S = 180.0


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
    controller_name: str = None
    gains_sha256: str = None
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
                 gains_store=None,
                 monotonic=time.monotonic,
                 utcnow=None,
                 recovery_wait=None):
        """Wire the supervisor; nothing is started until :meth:`tick` runs."""
        self._gains_store = gains_store
        self._settings = settings
        self._bridge = bridge
        self._lock_service = lock
        self._broker = broker
        self._spawn = spawn
        self._recording_factory = (
            recording_factory
            or (lambda: RecordingSupervisor(settings, spawn, monotonic=monotonic)))
        self._preflight_runner = preflight_runner
        self._fault_engine = fault_engine or FaultEngine(monotonic=monotonic)
        self._argv_builder = argv_builder
        self._session_namer = session_namer
        self._monotonic = monotonic
        self._utcnow = utcnow or (lambda: datetime.now(timezone.utc))
        self._recovery_wait = recovery_wait or self._wait_wall_time

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
        self._stop_requested = False
        self._gains = None              # StoredGains for the motion session
        self._watch_preview_gains = None  # display-only Watch fence
        self._jog_models = {}           # arm_id -> JogTargetModel
        self._arm_slots = {}            # arm_id -> controller slot (1-based)
        self._pose_cache = {}           # arm_id -> (mono_s, positions tuple)
        # arm_id -> (mono_s, sha256, controller_name, exact arm_ids tuple).
        # This is evidence that the pose was observed during a RUNNING Watch
        # session against the same content-addressed fence a later Motion
        # request names.  It is never a motion surface in its own right.
        self._watch_preview_cache = {}
        self._watch_recovery_barrier_ns = None
        self._targets_published = {}    # arm_id -> count
        self._last_publish_mono = {}    # arm_id -> mono_s
        self._recover_succeeded = False
        self._activation_baseline = None
        self._activation_gate = None
        self._settling_target_counts = {}

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

    def request_session_recover(self, operator_lease=None,
                                timeout_s=RECOVERY_REQUEST_TIMEOUT_S):
        """Run the bounded session-wide recovery; blocks for the verdict."""
        return self._submit(
            _Command(kind='recover', operator_lease=operator_lease), timeout_s)

    def operator_released(self):
        """
        React to an operator release or lock expiry (§6.4, §5.6).

        Forces every enable off immediately (the jog timer checks the flags
        and the lock on every tick, so the stream stops within one period);
        a best-effort controller-side disable is queued for the supervisor
        thread. The session itself keeps running.
        """
        with self._state_lock:
            any_enabled = any(self._arm_enabled.values()) and bool(self._jog_models)
        self._force_enables_off()
        # Enqueue the controller-side disable only when something was
        # actually enabled — a level-triggered caller (the frame pump) must
        # not be able to flood the command queue (review finding S2).
        if any_enabled:
            self._commands.put(_Command(kind='disable_all'))

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
        key is added or removed, so a concurrent iteration stays valid) and
        at most one queue put. The controller-side disable is left to the
        supervisor thread, and is queued only when something was actually
        enabled, so a repeated observation cannot flood the queue.
        """
        flags = self._arm_enabled
        enabled = [arm_id for arm_id, on in list(flags.items()) if on]
        for arm_id in enabled:
            flags[arm_id] = False
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
            shutdown_event.wait(config.SUPERVISOR_TICK_S)
        self.shutdown()

    def tick(self):
        """Run one supervisor step: commands first, then state work."""
        self._process_commands()
        self._update_pose_cache()
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
                time.sleep(config.SUPERVISOR_TICK_S)

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
                    command.resolve(self._accept_start(command.request))
                elif command.kind == 'stop':
                    command.resolve(self._accept_stop())
                elif command.kind == 'enable':
                    command.resolve(self._accept_arm_enable(
                        *command.request, operator_lease=command.operator_lease))
                elif command.kind == 'jog':
                    command.resolve(self._accept_arm_jog(*command.request))
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
        """Return whether taking ``command`` can create or change live control."""
        if command.kind in ('start', 'jog', 'recover'):
            return True
        return (command.kind == 'enable' and command.request is not None
                and bool(command.request[1]))

    def _accept_start(self, request):
        """Validate a start request and enter ``preflight`` (§6.7)."""
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
        if request.mode == 'watch':
            # A new Watch attempt supersedes the prior observation for the
            # selected arms immediately.  Do this before config/address
            # validation and preflight: even a rejected or failed attempt
            # must not leave old evidence available to a later Motion start.
            self._invalidate_watch_evidence(profile.arm_ids)
        gains = None
        watch_preview = None
        activation_baseline = None
        param_file = None
        # A Watch request may name an impedance config for display-only fence
        # preview.  Its controller identity is deliberately NOT a session
        # controller and is never forwarded to the state-only launch.
        # Simulate continues to ignore stray controller fields (finding S9).
        controller_name = request.controller_name if request.mode == 'motion' else None
        if request.mode == 'motion':
            gains = self._validate_motion_request(request, profile)
            param_file = gains.path
            if request.controller_name in config.JOG_CONTROLLERS:
                if self._settings.activation_settling_policy is None:
                    raise SessionError(
                        'settling_policy_required',
                        'Motion is blocked until a reviewed activation-settling '
                        'policy is supplied at server startup')
                for arm_id in profile.arm_ids:
                    fence = gains.fence.get(arm_id)
                    if (fence is None or fence.get('position_lower') is None
                            or fence.get('position_upper') is None):
                        raise SessionError(
                            'settling_fence_required',
                            'activation settling requires reviewed joint-position '
                            'bounds for every selected arm')
        elif request.mode == 'watch':
            watch_preview = self._validate_watch_preview_request(request, profile)
        try:
            launch_argv = self._argv_builder(
                request.arms, request.mode, self._settings,
                controller_name=controller_name,
                controller_param_file=param_file)
        except ProfileError as error:
            raise SessionError('robot_addresses_missing', str(error)) from None
        # The fence-vs-pose gate runs AFTER the address check: telling the
        # operator to run a Watch session is useless advice while the robot
        # addresses are not even configured (§6.7 orders the 412s this way).
        if gains is not None and request.controller_name in config.JOG_CONTROLLERS:
            activation_baseline = self._check_fence_pose_precondition(
                profile.arm_ids, gains)
            try:
                ActivationSettlingGate.validate_baseline(
                    self._settings.activation_settling_policy,
                    profile.arm_ids,
                    activation_baseline,
                    {arm_id: (
                        gains.fence[arm_id]['position_lower'],
                        gains.fence[arm_id]['position_upper'])
                     for arm_id in profile.arm_ids})
            except ValueError as error:
                raise SessionError(
                    'settling_margin_unavailable', str(error)) from None
        session_id = self._session_namer()
        session_gains = gains if gains is not None else watch_preview
        with self._state_lock:
            self._session = {
                'session_id': session_id,
                'arms': request.arms,
                'arm_ids': list(profile.arm_ids),
                'arm_mode': profile.arm_mode,
                'mode': request.mode,
                'controller_name': controller_name,
                # In Watch this hash identifies only the displayed preview;
                # controller_name stays null because no controller is loaded.
                'gains_sha256': (session_gains.config_sha256
                                 if session_gains is not None else None),
                'activation_policy_sha256': (
                    self._settings.activation_settling_policy.sha256
                    if (request.mode == 'motion'
                        and request.controller_name in config.JOG_CONTROLLERS)
                    else None),
                'started_at': rfc3339(self._utcnow()),
                'started_mono': self._monotonic(),
                'launch_argv': launch_argv,
                'last_error': None,
            }
            self._arm_enabled = {arm: False for arm in profile.arm_ids}
            self._preflight_result = None
            self._preflight_pending = True
            self._stop_requested = False
            self._gains = gains
            self._watch_preview_gains = watch_preview
            self._watch_recovery_barrier_ns = None
            self._arm_slots = {arm: slot for slot, arm
                               in enumerate(profile.arm_ids, start=1)}
            self._jog_models = {}
            if (request.mode == 'motion' and gains is not None
                    and request.controller_name in config.JOG_CONTROLLERS):
                for arm_id in profile.arm_ids:
                    fence = gains.fence[arm_id]
                    self._jog_models[arm_id] = JogTargetModel(
                        arm_id, fence['position_lower'], fence['position_upper'])
            self._targets_published = {arm: 0 for arm in profile.arm_ids}
            self._last_publish_mono = {}
            self._recover_succeeded = False
            self._activation_baseline = activation_baseline
            self._activation_gate = None
            self._settling_target_counts = {}
            self._fault_engine.reset()
            self._clear_fault()
            if request.mode == 'motion':
                # A Watch attestation describes the pose before this Motion
                # session.  Once the request is fully validated and accepted,
                # that evidence is one-shot: Motion may change the pose, even
                # if it later faults before an operator jog.  Consume only the
                # selected arms here, inside the same state transaction that
                # accepts the session; every earlier refusal preserves truthful
                # evidence so the operator may correct and retry the request.
                for arm_id in profile.arm_ids:
                    self._pose_cache.pop(arm_id, None)
                    self._watch_preview_cache.pop(arm_id, None)
        self._transition('preflight', reason=None)
        return {'session_id': session_id, 'state': 'preflight'}

    def _validate_motion_request(self, request, profile):
        """Run the §6.7 motion checks and the §5.4 fence-vs-pose precondition."""
        if not request.controller_name or request.controller_name not in config.WEB_CONTROLLERS:
            raise SessionError(
                'controller_not_reviewed',
                'motion mode requires controller_name, one of: {}'.format(
                    ', '.join(config.WEB_CONTROLLERS)))
        if not request.gains_sha256:
            raise SessionError('gains_required',
                               'motion mode requires gains_sha256 of an uploaded config')
        if self._gains_store is None:
            raise SessionError('internal_error', 'no gains store is wired')
        from franka_web.gains import GainsError
        try:
            gains = self._gains_store.match(
                request.gains_sha256, request.controller_name, request.arms)
        except GainsError as error:
            raise SessionError(error.code, error.detail) from None
        return gains

    def _validate_watch_preview_request(self, request, profile):
        """Validate an optional display-only joint fence for a Watch session."""
        if request.controller_name is None and request.gains_sha256 is None:
            return None
        if request.controller_name not in config.JOG_CONTROLLERS:
            raise SessionError(
                'controller_not_reviewed',
                'Watch fence preview supports the reviewed impedance '
                'controller only; it never loads that controller')
        if not request.gains_sha256:
            raise SessionError(
                'gains_required',
                'Watch fence preview requires gains_sha256 of an uploaded config')
        if self._gains_store is None:
            raise SessionError('internal_error', 'no gains store is wired')
        from franka_web.gains import GainsError
        try:
            preview = self._gains_store.match(
                request.gains_sha256, request.controller_name, request.arms)
        except GainsError as error:
            raise SessionError(error.code, error.detail) from None
        for arm_id in profile.arm_ids:
            fence = preview.fence.get(arm_id)
            if (fence is None or fence.get('position_lower') is None
                    or fence.get('position_upper') is None):
                raise SessionError(
                    'controller_not_reviewed',
                    'Watch fence preview requires reviewed joint-position bounds')
        return preview

    def _invalidate_watch_evidence(self, arm_ids):
        """Forget prior Watch pose/preview evidence for exactly ``arm_ids``."""
        with self._state_lock:
            for arm_id in arm_ids:
                self._pose_cache.pop(arm_id, None)
                self._watch_preview_cache.pop(arm_id, None)

    def _check_fence_pose_precondition(self, arm_ids, gains):
        """
        §5.4: refuse to start motion unless a recent pose sits inside the fence.

        A fence that does not contain the arm's actual pose commands motion
        the instant the controller is enabled. A fresh motion launch has no
        pose yet, so the check runs against the pose cache fed by a prior
        matching, successfully running Watch preview (TTL
        ``POSE_CACHE_TTL_S``).  Simulated poses never count.
        """
        now = self._monotonic()
        identity = (
            gains.config_sha256, gains.controller_name, tuple(arm_ids))
        baselines = {}
        for arm_id in arm_ids:
            with self._state_lock:
                cached = self._pose_cache.get(arm_id)
                preview = self._watch_preview_cache.get(arm_id)
            pose_age = None if cached is None else now - cached[0]
            if (pose_age is None or pose_age < 0.0
                    or pose_age > config.POSE_CACHE_TTL_S):
                raise SessionError(
                    'fence_pose_unverified',
                    'no recent REAL pose for {}: run a Watch session first so the '
                    'measured pose can be checked against the fence (simulated '
                    'poses never satisfy this gate)'.format(arm_id))
            preview_age = None if preview is None else now - preview[0]
            if (preview_age is None or preview_age < 0.0
                    or preview_age > config.POSE_CACHE_TTL_S
                    or preview[0] != cached[0]):
                raise SessionError(
                    'fence_pose_unverified',
                    'no recent Watch fence preview for {}: run Watch with this '
                    'reviewed config before starting Motion'.format(arm_id))
            if preview[1:] != identity:
                raise SessionError(
                    'gains_preview_mismatch',
                    'the recent Watch preview for {} used a different config, '
                    'controller, or arm selection; run Watch again with this '
                    'exact reviewed config'.format(arm_id))
            fence = gains.fence[arm_id]
            outside = _joints_outside_fence(
                cached[1], fence['position_lower'], fence['position_upper'])
            if outside:
                raise SessionError(
                    'pose_outside_fence',
                    '{} joints outside the uploaded fence: {}'.format(
                        arm_id, ', '.join(outside)))
            baselines[arm_id] = tuple(cached[1])
        return baselines

    def _accept_stop(self):
        """Handle an operator stop: idempotent while shutting down (§6.8)."""
        with self._state_lock:
            state = self._state
            session = dict(self._session) if self._session else None
            if state == 'stopped':
                raise SessionError('session_not_active', 'no session is active')
            if state == 'stopping':
                return {'state': 'stopping'}
            self._stop_requested = True

        if (state == 'running' and session is not None
                and session['mode'] == 'watch'):
            # Commands run before the normal state poll.  Without this check, a
            # queued Stop could skip a simultaneously detectable Watch fault
            # and preserve evidence from a launch/readiness state that was
            # already unhealthy.  Stop itself must remain available even if a
            # collaborator throws, so uncertainty revokes rather than refuses.
            try:
                ready = self._readiness_met(session)
                reasons = self._fault_engine.evaluate(
                    self._fault_snapshot(session))
                preserve_attestation = ready and not reasons
            except Exception:  # noqa: BLE001 - Stop is fail-safe and idempotent
                preserve_attestation = False
            if not preserve_attestation:
                self._invalidate_watch_evidence(session['arm_ids'])
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
        """§6.13 POST /api/arm/{arm_id}/enable, executed in exact order."""
        session, state = self._motion_guards(arm_id)
        if state != 'running':
            raise SessionError('session_not_running', 'the session is not running')
        with self._state_lock:
            model = self._jog_models.get(arm_id)
            slot = self._arm_slots.get(arm_id)
        if model is None:
            raise SessionError(
                'not_motion_mode',
                'the hold controller has no enable surface (arms are held at '
                'their activation pose)')
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
            return {'arm_id': arm_id, 'enabled': False, 'target': None,
                    'message': message}
        sample = self._bridge.joint_sample()
        now_ns = int(self._monotonic() * 1e9)
        if (sample is None or
                (now_ns - sample[0]) / 1e9 > config.ENABLE_JOINT_STATE_MAX_AGE_S):
            raise SessionError('joint_state_stale',
                               'no joint sample newer than {} s'.format(
                                   config.ENABLE_JOINT_STATE_MAX_AGE_S))
        joints = health.extract_joints(arm_id, sample[1])
        if not joints['complete']:
            raise SessionError('joint_state_stale',
                               'the joint sample does not carry all 7 joints')
        measured = joints['positions']
        with self._state_lock:
            fence = self._gains.fence[arm_id]
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
                               'the hold controller has no jog surface')
        if not enabled:
            raise SessionError('arm_not_enabled',
                               'enable {} before jogging it'.format(arm_id))
        self._guard_no_fresh_fault(session, 'jog')
        try:
            result = model.step(joint_index, direction)
        except JogError as error:
            raise SessionError('invalid_joint', str(error)) from None
        return {'arm_id': arm_id, 'target': list(result.target),
                'clamped': list(result.clamped)}

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
            raise SessionError('not_production_mode',
                               'recovery applies to watch/motion sessions only')
        if not self._session_supports_recovery(session):
            raise SessionError(
                'recovery_not_supported',
                'Hold sessions cannot be recovered in place because activating '
                'Hold immediately engages measured-pose effort control; stop and '
                'restart the session')

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
                response = self._bridge.call_switch_deactivate([controller])
                observed = self._bridge.query_controller_states()
                inactive = bool(
                    response and response['ok'] and observed is not None
                    and observed.get(controller) != 'active')
                steps.append({
                    'step': 'controller_inactive', 'phase': 'pre',
                    'controller': controller, 'ok': inactive,
                    'detail': ('inactive' if inactive else
                               'deactivation was not verified'),
                })
                if not inactive:
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
                try:
                    # Arm under the bridge cache lock immediately before the
                    # lifecycle call that can reactivate torque control.
                    self._bridge.begin_activation_capture(session['arm_ids'])
                except Exception as error:  # noqa: BLE001 - fail closed
                    self._recovery_failure(
                        session, steps, 'recovery_failed',
                        'motion re-activation observation could not be armed: '
                        '{}'.format(error))
            response = self._bridge.call_switch_activate([name])
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
        self._force_enables_off()
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
        with self._state_lock:
            self._recover_succeeded = True
        return self._recovery_payload(session, steps)

    @staticmethod
    def _session_supports_recovery(session):
        """Watch and impedance are recoverable; Hold activation is motion."""
        return (session['mode'] == 'watch'
                or (session['mode'] == 'motion'
                    and session['controller_name'] in config.JOG_CONTROLLERS))

    def _session_fault_recoverable(self, session, reasons):
        """Combine fault-rule eligibility with the session controller policy."""
        return (self._session_supports_recovery(session)
                and FaultEngine.recoverable(session['mode'], reasons))

    def _activation_fences(self, session):
        """Return the selected arms' reviewed position fences."""
        with self._state_lock:
            gains = self._gains
        if gains is None:
            raise ValueError('the Motion session has no reviewed configuration')
        try:
            return {
                arm_id: (
                    tuple(gains.fence[arm_id]['position_lower']),
                    tuple(gains.fence[arm_id]['position_upper']))
                for arm_id in session['arm_ids']
            }
        except (KeyError, TypeError):
            raise ValueError(
                'the Motion session has no complete joint-position fence') from None

    def _install_activation_gate(self, session, baseline, *, barrier_ns):
        """Install a fresh immutable-policy gate for one torque activation."""
        policy = self._settings.activation_settling_policy
        if policy is None or baseline is None:
            raise ValueError('no complete reviewed activation policy/baseline')
        if session.get('activation_policy_sha256') != policy.sha256:
            raise ValueError('the session activation-policy identity changed')
        fences = self._activation_fences(session)
        ActivationSettlingGate.validate_baseline(
            policy, session['arm_ids'], baseline, fences)
        gate = ActivationSettlingGate(
            policy, session['arm_ids'], baseline, fences,
            started_mono_ns=barrier_ns, barrier_ns=barrier_ns,
            sample_max_age_s=config.ENABLE_JOINT_STATE_MAX_AGE_S)
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
        """Capture a fresh state-only pose immediately before re-activation."""
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
                <= int(config.ENABLE_JOINT_STATE_MAX_AGE_S * 1e9))
            if sample_fresh:
                break
            remaining = deadline - time.monotonic()
            if (remaining <= 0.0
                    or not self._recovery_wait(min(
                        config.SUPERVISOR_TICK_S, remaining))):
                self._recovery_failure(
                    session, steps, 'recovery_failed',
                    'no fresh joint sample was available before motion '
                    're-activation')
        baseline = {}
        for arm_id in session['arm_ids']:
            joints = health.extract_joints(arm_id, sample[1])
            try:
                positions = tuple(float(value) for value in joints['positions'])
            except (TypeError, ValueError):
                positions = ()
            if (not joints['complete'] or len(positions) != config.JOINT_COUNT
                    or not all(math.isfinite(value) for value in positions)):
                self._recovery_failure(
                    session, steps, 'recovery_failed',
                    '{} has no complete finite pre-activation pose'.format(arm_id))
            baseline[arm_id] = positions
        try:
            policy = self._settings.activation_settling_policy
            if policy is None or session.get('activation_policy_sha256') != policy.sha256:
                raise ValueError('the reviewed activation policy is unavailable')
            ActivationSettlingGate.validate_baseline(
                policy, session['arm_ids'], baseline,
                self._activation_fences(session))
        except ValueError as error:
            self._recovery_failure(
                session, steps, 'recovery_failed',
                'motion re-activation cannot be gated: {}'.format(error))
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
            if not self._recovery_wait(min(config.SUPERVISOR_TICK_S, remaining)):
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
        response = self._bridge.call_switch_deactivate([controller])
        states = self._bridge.query_controller_states()
        ok = bool(response and response['ok'] and states is not None
                  and states.get(controller) != 'active')
        steps.append({'step': 'rollback_deactivate', 'controller': controller,
                      'ok': ok,
                      'detail': ('inactive' if ok
                                 else 'deactivation was not verified')})

    def _recovery_failure(self, session, steps, code, detail):
        """Remain faulted and raise while preserving every completed step."""
        self._end_activation_capture()
        self._rollback_motion_controller(session, steps)
        self._force_enables_off()
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
        running in motion mode with a jog controller and the operator lock is
        held and unexpired. Every other condition publishes nothing, which
        the controller answers with its 0.1 s watchdog freeze.
        """
        with self._state_lock:
            if self._state != 'running' or self._session is None:
                return
            if self._session['mode'] != 'motion':
                return
            if self._session['controller_name'] not in config.JOG_CONTROLLERS:
                return
            arms = [(arm_id, self._arm_slots.get(arm_id), self._jog_models.get(arm_id))
                    for arm_id, enabled in self._arm_enabled.items() if enabled]
        if not arms:
            return
        if not self._lock_service.state()['locked']:
            return
        for arm_id, slot, model in arms:
            if slot is None or model is None or not model.seeded:
                continue
            try:
                with self._state_lock:
                    # Re-check right before publishing: a _force_enables_off
                    # landing after the snapshot must silence this arm now,
                    # not one tick later (review finding S23).
                    if not self._arm_enabled.get(arm_id):
                        continue
                message = model.message(self._bridge.now_msg(),
                                        health.joint_names_for(arm_id))
                self._bridge.publish_target(slot, message)
                with self._state_lock:
                    self._targets_published[arm_id] = (
                        self._targets_published.get(arm_id, 0) + 1)
                    self._last_publish_mono[arm_id] = self._monotonic()
            except Exception:  # noqa: BLE001 - one arm must not gap the other
                continue

    def _update_pose_cache(self):
        """
        Keep the §5.4 pose cache fresh — from REAL hardware only.

        A simulated pose must never satisfy the fence-vs-pose gate: the fake
        stack's positions say nothing about where the physical arms are, and
        the gate exists precisely because a fence that does not contain the
        actual pose commands motion the instant the controller is enabled
        (review finding S4).
        """
        with self._state_lock:
            session = dict(self._session) if self._session else None
            state = self._state
            preview_gains = self._watch_preview_gains
            recovery_barrier_ns = self._watch_recovery_barrier_ns
        # Only a Watch session that actually reached RUNNING may create
        # evidence for a later Motion start.  Preflight/starting data can be a
        # leftover sample from an earlier graph and is display-only at most.
        if session is None or session['mode'] != 'watch' or state != 'running':
            return
        sample = self._bridge.joint_sample()
        if sample is None:
            self._invalidate_watch_evidence(session['arm_ids'])
            return
        if (recovery_barrier_ns is not None
                and int(sample[0]) <= recovery_barrier_ns):
            self._invalidate_watch_evidence(session['arm_ids'])
            return
        now = self._monotonic()
        sample_time = sample[0] / 1e9
        sample_age = now - sample_time
        sample_fresh = 0.0 <= sample_age <= config.JOINT_STATE_STALE_FAULT_S
        preview_identity = None
        if preview_gains is not None:
            preview_identity = (
                preview_gains.config_sha256,
                preview_gains.controller_name,
                tuple(session['arm_ids']),
            )
        updates = {}
        for arm_id in session['arm_ids']:
            joints = health.extract_joints(arm_id, sample[1])
            updates[arm_id] = (
                tuple(joints['positions'])
                if sample_fresh and joints['complete'] else None)
        with self._state_lock:
            if recovery_barrier_ns is not None:
                self._watch_recovery_barrier_ns = None
            for arm_id, positions in updates.items():
                if positions is None:
                    self._pose_cache.pop(arm_id, None)
                    self._watch_preview_cache.pop(arm_id, None)
                    continue
                # Preserve the sample's real timestamp.  Re-reading one stale
                # JointState must never refresh its age indefinitely.
                self._pose_cache[arm_id] = (sample_time, positions)
                if preview_identity is not None:
                    self._watch_preview_cache[arm_id] = (
                        sample_time, *preview_identity)
                else:
                    self._watch_preview_cache.pop(arm_id, None)

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
            # The verdict alone is not actionable: an ERROR means the run
            # itself could not be made or understood, and only
            # PreflightResult.error says why (tool missing, timed out,
            # unusable report). The frame's preflight block has no field for
            # it by §6.11, so last_error is where the operator can read it
            # (verification finding F-2).
            detail = 'RT preflight failed: {}'.format(result.overall)
            if result.error:
                detail = '{} ({})'.format(detail, result.error)
            self._record_error('preflight_failed', detail)
            self._transition('stopping', reason='preflight_failed')
            return
        self._enter_starting()

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
            # potentially 30-second RT preflight plus recorder startup.  Re-open
            # the content-addressed file at the last web-owned boundary so a
            # same-user edit in that interval cannot make the Watch/UI fence
            # describe A while operator_launch seals valid-but-different B.
            # Keep the failure text constant: filesystem paths and lower-level
            # errors are terminal diagnostics, not state-frame/API material.
            from franka_web.gains import GainsError
            try:
                checked_gains = self._gains_store.match(
                    session['gains_sha256'], session['controller_name'],
                    session['arms'])
            except (GainsError, OSError):
                checked_gains = None
            with self._state_lock:
                accepted_gains = self._gains
            if (checked_gains is None or accepted_gains is None
                    or checked_gains.config_sha256 != accepted_gains.config_sha256
                    or checked_gains.controller_name != accepted_gains.controller_name
                    or tuple(checked_gains.arms) != tuple(accepted_gains.arms)
                    or checked_gains.path != accepted_gains.path):
                self._record_error(
                    'gains_invalid',
                    'the selected controller configuration changed after start '
                    'acceptance; launch was refused')
                self._transition('stopping', reason='gains_invalid')
                return
        # The bridge subscriptions must exist before the launch child can
        # publish any activation evidence. Wiring them after spawn left the
        # most important part of the transition unobservable.
        try:
            self._bridge.configure_session(session['arm_ids'], session['arm_mode'])
            if session['mode'] in ('watch', 'motion'):
                jog_controller = (session['controller_name']
                                  if (session['mode'] == 'motion'
                                      and session['controller_name']
                                      in config.JOG_CONTROLLERS)
                                  else None)
                self._bridge.configure_motion(session['arm_ids'], jog_controller)
        except Exception as error:  # noqa: BLE001 - refuse before launch
            self._record_error(
                'internal_error',
                'session observation wiring failed before launch: {}'.format(error))
            self._transition('stopping', reason='internal_error')
            return
        if (session['mode'] == 'motion'
                and session['controller_name'] in config.JOG_CONTROLLERS):
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
                                 parent_death_signal=signal.SIGINT)
        except (LauncherError, OSError) as error:
            self._record_error('launch_failed', 'launch spawn failed: {}'.format(error))
            self._transition('stopping', reason='launch_failed')
            return
        with self._state_lock:
            self._launch = launch
            self._starting_deadline = self._monotonic() + config.STARTING_TIMEOUT_S
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
        """Advance ``starting``: readiness, child death, or timeout."""
        with self._state_lock:
            launch = self._launch
            recorder = self._recording
            deadline = self._starting_deadline
            session = dict(self._session)
        if recorder is not None and not self._recorder_tick(recorder):
            return
        if launch is None or not launch.alive():
            self._record_error('launch_failed', 'the launch child exited during startup')
            self._transition('stopping', reason='launch_failed')
            return
        if self._readiness_met(session):
            if (session['mode'] == 'motion'
                    and session['controller_name'] in config.JOG_CONTROLLERS):
                self._begin_settling(session)
            else:
                self._transition('running', reason=None)
            return
        if self._monotonic() > deadline:
            self._record_error('launch_timeout',
                               'the stack did not become ready within {:.0f} s'.format(
                                   config.STARTING_TIMEOUT_S))
            self._transition('stopping', reason='launch_timeout')

    def _begin_settling(self, session, baseline=None):
        """Enter the post-torque-activation gate with every local enable off."""
        policy = self._settings.activation_settling_policy
        with self._state_lock:
            accepted_baseline = baseline or self._activation_baseline
        if policy is None or accepted_baseline is None:
            self._record_error(
                'settling_policy_required',
                'Motion activation settling has no complete reviewed policy/baseline')
            self._transition('stopping', reason='settling_policy_required')
            return

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
                > int(config.ENABLE_JOINT_STATE_MAX_AGE_S * 1e9)):
            self._record_error(
                'activation_settling_limit',
                'no fresh joint sample was available at the activation barrier')
            self._transition('stopping', reason='activation_settling_limit')
            return
        try:
            # The barrier is captured after the current readiness sample.
            # Only a later bridge receipt may count.
            gate = self._install_activation_gate(
                session, accepted_baseline, barrier_ns=now_ns)
        except ValueError as error:
            self._record_error('activation_settling_limit', str(error))
            self._transition('stopping', reason='activation_settling_limit')
            return
        capture_verdict = self._observe_activation_capture(gate)
        if capture_verdict.status == 'failed':
            self._record_error(capture_verdict.code, capture_verdict.detail)
            self._end_activation_capture()
            self._transition('stopping', reason=capture_verdict.code)
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
            self._record_error(
                'activation_settling_limit', 'activation settling state has no gate')
            self._transition('stopping', reason='activation_settling_limit')
            return
        target_traffic = any(
            count > target_barrier.get(arm_id, 0)
            for arm_id, count in targets_published.items())
        if enabled or target_traffic:
            self._force_enables_off()
            self._record_error(
                'activation_settling_limit',
                'an enable or target appeared while activation settling was closed')
            self._transition('stopping', reason='activation_settling_limit')
            return

        sample = self._bridge.joint_sample()
        now_ns = int(self._monotonic() * 1e9)
        # Drain hard transition extrema before applying the time budget. This
        # preserves the most specific safety evidence at the exact deadline.
        capture_verdict = self._observe_activation_capture(gate)
        if capture_verdict.status == 'failed':
            self._record_error(capture_verdict.code, capture_verdict.detail)
            self._end_activation_capture()
            self._transition('stopping', reason=capture_verdict.code)
            return
        with self._state_lock:
            deadline_reached = gate.deadline_reached(now_ns)
        if deadline_reached:
            # Close admission, drain once more, apply hard limits, then apply
            # the inclusive deadline as one indivisible exit decision.
            timeout = self._close_activation_capture(gate)
            self._record_error(timeout.code, timeout.detail)
            self._transition('stopping', reason=timeout.code)
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
                self._record_error(verdict.code, verdict.detail)
                self._end_activation_capture()
                self._transition('stopping', reason=verdict.code)
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
                self._record_error(exit_verdict.code, exit_verdict.detail)
                self._transition('stopping', reason=exit_verdict.code)
                return
            self._transition('fault', reason=reasons[0].code)
            return
        if verdict is not None and verdict.status == 'ready':
            # Validate callback extrema and decide close-vs-continue under the
            # bridge's callback lock. No callback can land in a close/re-arm
            # gap, and hard-limit evidence stops the session.
            final_verdict = self._finalize_activation_capture(gate)
            if final_verdict.status == 'failed':
                self._record_error(final_verdict.code, final_verdict.detail)
                self._transition('stopping', reason=final_verdict.code)
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
            self._transition('running', reason=None)

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
            if session['controller_name'] in config.JOG_CONTROLLERS:
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
        with self._state_lock:
            self._fault_since = rfc3339(self._utcnow())
            self._fault_reasons = tuple(reasons)
            self._fault_recoverable = self._session_fault_recoverable(
                session, reasons)
        self._transition('fault', reason=reasons[0].code)

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
        if (session['mode'] == 'motion'
                and session['controller_name'] in config.JOG_CONTROLLERS):
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
        if session['mode'] == 'watch':
            with self._state_lock:
                self._watch_recovery_barrier_ns = int(self._monotonic() * 1e9)
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
        with self._state_lock:
            recorder = self._recording
            launch = self._launch
        failures = []
        recorder_stopped = recorder is None
        if recorder is not None:
            try:
                recorder.stop()
                recorder_stopped = True
            except Exception as error:
                failures.append('recorder stop failed: {}'.format(error))
        launch_stopped = launch is None
        if launch is not None:
            try:
                # Always call stop, even when the guardian leader already
                # exited: guarded ChildProcess.stop independently proves the
                # immutable target process group is empty.
                launch.stop(config.STOP_SIGINT_WAIT_S,
                            config.STOP_SIGTERM_WAIT_S,
                            config.STOP_SIGKILL_WAIT_S)
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
            self._watch_preview_gains = None
            if self._session is not None:
                # Freeze the session clock: a stopped session's uptime must
                # not keep counting (review finding R19).
                self._session.setdefault('ended_mono', self._monotonic())
        if not recorder_stopped or not launch_stopped:
            return
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
            if (new_state == 'fault' and self._state != 'fault'
                    and self._session is not None
                    and self._session['mode'] == 'watch'):
                # A fault breaks the continuous observation that made this
                # Watch sample trustworthy.  Forget its pose and exact-config
                # attestation before publishing the fault frame; recovery may
                # recreate them only from a later RUNNING Watch sample.
                for arm_id in self._session['arm_ids']:
                    self._pose_cache.pop(arm_id, None)
                    self._watch_preview_cache.pop(arm_id, None)
            if (new_state == 'stopping' and self._state != 'stopping'
                    and self._session is not None
                    and self._session['mode'] == 'watch'
                    and not self._stop_requested):
                # Only _accept_stop sets _stop_requested, after synchronously
                # checking a RUNNING Watch's readiness and fault snapshot.
                # Recorder failures, internal exceptions, startup failures and
                # process shutdown all reach stopping without that proof and
                # must not leave an attestation for a later Motion session.
                for arm_id in self._session['arm_ids']:
                    self._pose_cache.pop(arm_id, None)
                    self._watch_preview_cache.pop(arm_id, None)
            if new_state in ('fault', 'stopping'):
                for arm_id in self._arm_enabled:
                    self._arm_enabled[arm_id] = False
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
        frame = {
            'schema_version': config.SCHEMA_VERSION,
            'server_time': rfc3339(now),
            'server_uptime_s': round(self._monotonic() - self._started_mono, 3),
            'session': self._session_block(state, session, launch),
            'operator': self._lock_service.state(),
            'preflight': (preflight.frame() if preflight is not None else
                          {'ran_at': None, 'overall': None,
                           'blocking': False, 'failed_checks': []}),
            'recording': (recorder.frame(topics_for(session['arm_mode']))
                          if (recorder is not None and session) else
                          {'active': False, 'name': None, 'sequence': 0,
                           'path': None, 'arm_mode': None, 'topics': []}),
            'controllers': self._controllers_block(state),
            'hardware': self._hardware_block(state),
            'fault': {
                'active': state == 'fault',
                'since': fault_since,
                'reasons': [r.as_dict() for r in fault_reasons],
                'recoverable': fault_recoverable,
                'recover_hint': ('release the physical stop first, then recover'
                                 if (state == 'fault' and fault_recoverable) else None),
            },
            'arms': self._arms_block(state, session),
        }
        return frame

    def _session_block(self, state, session, launch):
        """Build the §6.11 session sub-object."""
        block = {
            'state': state,
            'session_id': None,
            'arms': None,
            'arm_ids': [],
            'arm_mode': None,
            'mode': None,
            'controller_name': None,
            'gains_sha256': None,
            'started_at': None,
            'uptime_s': None,
            'launch_running': bool(launch is not None and launch.alive()),
            'last_error': None,
            'advisory': config.STOP_ADVISORY,
            'activation': self._activation_block(state, session),
        }
        if session is not None:
            block.update({
                'session_id': session['session_id'],
                'arms': session['arms'],
                'arm_ids': list(session['arm_ids']),
                'arm_mode': session['arm_mode'],
                'mode': session['mode'],
                'controller_name': session['controller_name'],
                'gains_sha256': session['gains_sha256'],
                'started_at': session['started_at'],
                'uptime_s': round(
                    session.get('ended_mono', self._monotonic())
                    - session['started_mono'], 3),
                'last_error': session['last_error'],
            })
        return block

    def _activation_block(self, state, session):
        """Build bounded evidence for the fail-closed torque-activation gate."""
        required = bool(
            session is not None
            and session['mode'] == 'motion'
            and session['controller_name'] in config.JOG_CONTROLLERS)
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
                self._settings.activation_settling_policy.stable_window_s
                if required and self._settings.activation_settling_policy is not None
                else None),
            'required_samples': (
                self._settings.activation_settling_policy.min_sample_count
                if required and self._settings.activation_settling_policy is not None
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
            gains = self._gains
            watch_preview = self._watch_preview_gains
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
        for arm_id in session['arm_ids']:
            projection = health.project_arm(
                arm_id, now_ns,
                projected_joint_sample,
                self._bridge.robot_state_sample(arm_id),
                self._bridge.diagnostic_sample(arm_id))
            model = models.get(arm_id)
            display_gains = gains if session['mode'] == 'motion' else watch_preview
            fence = (display_gains.fence.get(arm_id)
                     if display_gains is not None else None)
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
            arms[arm_id] = projection
        return arms
