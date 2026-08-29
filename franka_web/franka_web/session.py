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

    stopped -> preflight -> starting -> running -> (fault) -> stopping -> stopped

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

STATES = ('stopped', 'preflight', 'starting', 'running', 'fault', 'stopping')

_VALID_ARMS = ('panda1', 'panda2', 'both')
_VALID_MODES = ('simulate', 'watch', 'motion')


class SessionError(Exception):
    """A refused session command, carrying a stable §6.14 error code."""

    def __init__(self, code, detail):
        """Store the closed-set ``code`` and the operator-safe ``detail``."""
        super().__init__(detail)
        self.code = code
        self.detail = detail


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
                 utcnow=None):
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
        self._jog_models = {}           # arm_id -> JogTargetModel
        self._arm_slots = {}            # arm_id -> controller slot (1-based)
        self._pose_cache = {}           # arm_id -> (mono_s, positions tuple)
        self._targets_published = {}    # arm_id -> count
        self._last_publish_mono = {}    # arm_id -> mono_s
        self._recover_succeeded = False

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

    def request_start(self, request, timeout_s=5.0):
        """Ask the supervisor to start a session; blocks for the verdict."""
        return self._submit(_Command(kind='start', request=request), timeout_s)

    def request_stop(self, timeout_s=5.0):
        """Ask the supervisor to stop the session; blocks for the verdict."""
        return self._submit(_Command(kind='stop'), timeout_s)

    def request_arm_enable(self, arm_id, enabled, timeout_s=10.0):
        """Toggle one arm's enable (§6.13); blocks for the verdict."""
        return self._submit(
            _Command(kind='enable', request=(arm_id, bool(enabled))), timeout_s)

    def request_arm_jog(self, arm_id, joint_index, direction, timeout_s=5.0):
        """Jog one joint by one fixed step (§6.13); blocks for the verdict."""
        return self._submit(
            _Command(kind='jog', request=(arm_id, joint_index, direction)), timeout_s)

    def request_arm_recover(self, arm_id, timeout_s=25.0):
        """Run the one-click §7.3 recover sequence; blocks for the verdict."""
        return self._submit(_Command(kind='recover', request=arm_id), timeout_s)

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
            # Execution had already begun: the answer is imminent. Wait it
            # out briefly rather than lying about a command that DID run.
            command.done.wait(10.0)
            if not command.done.is_set():
                raise SessionError('internal_error', 'the supervisor did not answer')
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
        elif state == 'running':
            self._poll_running()
        elif state == 'fault':
            self._poll_fault()
        if self.state == 'stopping':
            self._do_stopping()

    def shutdown(self):
        """Drive whatever is running down to ``stopped`` (process exit path)."""
        if self.state != 'stopped':
            self._transition('stopping', reason=None)
            self._do_stopping()

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
                if command.kind == 'start':
                    command.resolve(self._accept_start(command.request))
                elif command.kind == 'stop':
                    command.resolve(self._accept_stop())
                elif command.kind == 'enable':
                    command.resolve(self._accept_arm_enable(*command.request))
                elif command.kind == 'jog':
                    command.resolve(self._accept_arm_jog(*command.request))
                elif command.kind == 'recover':
                    command.resolve(self._accept_arm_recover(command.request))
                elif command.kind == 'disable_all':
                    command.resolve(self._disable_all_arms())
                else:
                    command.reject(SessionError('internal_error', 'unknown command'))
            except SessionError as error:
                command.reject(error)

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
        gains = None
        param_file = None
        # Controller/gains identity is meaningful only in motion mode; stray
        # values on a simulate/watch request are ignored, never forwarded to
        # the launch argv or the frame (review finding S9).
        controller_name = request.controller_name if request.mode == 'motion' else None
        if request.mode == 'motion':
            gains = self._validate_motion_request(request, profile)
            param_file = gains.path
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
            self._check_fence_pose_precondition(profile.arm_ids, gains)
        session_id = self._session_namer()
        with self._state_lock:
            self._session = {
                'session_id': session_id,
                'arms': request.arms,
                'arm_ids': list(profile.arm_ids),
                'arm_mode': profile.arm_mode,
                'mode': request.mode,
                'controller_name': controller_name,
                'gains_sha256': (request.gains_sha256
                                 if request.mode == 'motion' else None),
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
            self._arm_slots = {arm: slot for slot, arm
                               in enumerate(profile.arm_ids, start=1)}
            self._jog_models = {}
            if gains is not None and request.controller_name in config.JOG_CONTROLLERS:
                for arm_id in profile.arm_ids:
                    fence = gains.fence[arm_id]
                    self._jog_models[arm_id] = JogTargetModel(
                        arm_id, fence['position_lower'], fence['position_upper'])
            self._targets_published = {arm: 0 for arm in profile.arm_ids}
            self._last_publish_mono = {}
            self._recover_succeeded = False
            self._fault_engine.reset()
            self._clear_fault()
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

    def _check_fence_pose_precondition(self, arm_ids, gains):
        """
        §5.4: refuse to start motion unless a recent pose sits inside the fence.

        A fence that does not contain the arm's actual pose commands motion
        the instant the controller is enabled. A fresh motion launch has no
        pose yet, so the check runs against the pose cache fed by a prior
        watch/simulate session (TTL ``POSE_CACHE_TTL_S``).
        """
        now = self._monotonic()
        for arm_id in arm_ids:
            cached = self._pose_cache.get(arm_id)
            if cached is None or now - cached[0] > config.POSE_CACHE_TTL_S:
                raise SessionError(
                    'fence_pose_unverified',
                    'no recent REAL pose for {}: run a Watch session first so the '
                    'measured pose can be checked against the fence (simulated '
                    'poses never satisfy this gate)'.format(arm_id))
            fence = gains.fence[arm_id]
            outside = _joints_outside_fence(
                cached[1], fence['position_lower'], fence['position_upper'])
            if outside:
                raise SessionError(
                    'pose_outside_fence',
                    '{} joints outside the uploaded fence: {}'.format(
                        arm_id, ', '.join(outside)))

    def _accept_stop(self):
        """Handle an operator stop: idempotent while shutting down (§6.8)."""
        with self._state_lock:
            if self._state == 'stopped':
                raise SessionError('session_not_active', 'no session is active')
            if self._state == 'stopping':
                return {'state': 'stopping'}
            self._stop_requested = True
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

    def _accept_arm_enable(self, arm_id, enabled):
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
        with self._state_lock:
            self._arm_enabled[arm_id] = True
        return {'arm_id': arm_id, 'enabled': True,
                'target': list(model.target), 'message': response['message']}

    def _accept_arm_jog(self, arm_id, joint_index, direction):
        """§6.13 POST /api/arm/{arm_id}/jog — one fixed step on one joint."""
        self._motion_guards(arm_id)
        with self._state_lock:
            model = self._jog_models.get(arm_id)
            enabled = self._arm_enabled.get(arm_id, False)
        if model is None:
            raise SessionError('not_motion_mode',
                               'the hold controller has no jog surface')
        if not enabled:
            raise SessionError('arm_not_enabled',
                               'enable {} before jogging it'.format(arm_id))
        try:
            result = model.step(joint_index, direction)
        except JogError as error:
            raise SessionError('invalid_joint', str(error)) from None
        return {'arm_id': arm_id, 'target': list(result.target),
                'clamped': list(result.clamped)}

    def _accept_arm_recover(self, arm_id):
        """
        §6.13 POST /api/arm/{arm_id}/recover — the ONE-CLICK §7.3 sequence.

        User-authorized 2026-08-29 ('one click'): error recovery, hardware
        re-activation when needed, and controller re-activation run from a
        single press. ``enabled_after`` is always False -- re-enabling is a
        fresh authorization, never an automatic continuation.
        """
        with self._state_lock:
            state = self._state
            session = dict(self._session) if self._session else None
        if session is None or state != 'fault':
            raise SessionError('not_faulted', 'the session is not faulted')
        if session['mode'] not in ('watch', 'motion'):
            raise SessionError('not_production_mode',
                               'recovery applies to watch/motion sessions only')
        if arm_id not in session['arm_ids']:
            raise SessionError('arm_not_in_session',
                               '{} is not part of this session'.format(arm_id))
        self._force_enables_off()
        for model in self._jog_models.values():
            model.invalidate()
        steps = []
        recovery = self._bridge.call_error_recovery(arm_id)
        if recovery is None:
            raise SessionError('recovery_service_unavailable',
                               'the error-recovery service is not reachable')
        informational = (not recovery['success']
                         and recovery['error'] == 'No errors')
        steps.append({'step': 'error_recovery',
                      'ok': bool(recovery['success'] or informational),
                      'detail': recovery['error'] if not recovery['success'] else ''})
        if not steps[-1]['ok']:
            raise SessionError('recovery_failed',
                               'error recovery failed: {}'.format(recovery['error']))
        component = self._bridge.hardware_component()
        if component is None:
            # The component is unobserved (exactly what an F8 fault looks
            # like): never certify it active — drive it by its canonical
            # name and report what actually happened (review finding S17).
            hardware = self._bridge.call_hardware_active('FrankaMultiHardwareInterface')
            ok = bool(hardware and hardware['ok'])
            steps.append({'step': 'hardware_active', 'ok': ok,
                          'detail': 'activated (component was unobserved)' if ok
                          else 'component unobserved and activation refused'})
            if not ok:
                raise SessionError('recovery_failed',
                                   'the hardware component could not be observed '
                                   'or re-activated')
        elif component.get('lifecycle_label') != 'active':
            hardware = self._bridge.call_hardware_active(component['name'])
            ok = bool(hardware and hardware['ok'])
            steps.append({'step': 'hardware_active', 'ok': ok,
                          'detail': 'active' if ok else 'activation refused'})
            if not ok:
                raise SessionError('recovery_failed',
                                   'the hardware component did not reach active')
        else:
            steps.append({'step': 'hardware_active', 'ok': True, 'detail': 'active'})
        if session['mode'] == 'motion' and session['controller_name']:
            controller = session['controller_name']
            if self._bridge.controller_states().get(controller) == 'active':
                # STRICT switch_controller REFUSES an activate naming an
                # already-active controller, which would wedge recovery for
                # every fault that left it active (review finding S1).
                steps.append({'step': 'reactivate_controllers', 'ok': True,
                              'detail': '{} (already active)'.format(controller)})
            else:
                switched = self._bridge.call_switch_activate([controller])
                ok = bool(switched and switched['ok'])
                steps.append({'step': 'reactivate_controllers', 'ok': ok,
                              'detail': controller if ok
                              else 'switch_controller refused'})
                if not ok:
                    raise SessionError('recovery_failed',
                                       'controller re-activation was refused')
        with self._state_lock:
            self._recover_succeeded = True
        return {'arm_id': arm_id, 'steps': steps, 'enabled_after': False}

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
            session = self._session
            mode = session['mode'] if session else None
        if mode not in ('watch', 'motion'):
            return
        sample = self._bridge.joint_sample()
        if sample is None:
            return
        now = self._monotonic()
        for arm_id in ('panda1', 'panda2'):
            joints = health.extract_joints(arm_id, sample[1])
            if joints['complete']:
                self._pose_cache[arm_id] = (now, tuple(joints['positions']))

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
        try:
            recorder.start(session['session_id'], session['arm_mode'], self._child_env())
        except (RecordingError, LauncherError, OSError) as error:
            # LauncherError/OSError: the spawn layer failed underneath the
            # recorder; same verdict, the session must not start (§5.5).
            self._record_error('recording_failed', str(error))
            self._transition('stopping', reason='recording_failed')
            return
        with self._state_lock:
            self._recording = recorder
        try:
            # SIGINT, not SIGTERM, as the parent-death signal: `ros2 launch`
            # tears its whole tree down only on SIGINT (see launcher.py).
            launch = self._spawn(session['launch_argv'], self._child_env(), 'launch',
                                 parent_death_signal=signal.SIGINT)
        except (LauncherError, OSError) as error:
            self._record_error('launch_failed', 'launch spawn failed: {}'.format(error))
            self._transition('stopping', reason='launch_failed')
            return
        with self._state_lock:
            self._launch = launch
            self._starting_deadline = self._monotonic() + config.STARTING_TIMEOUT_S
        self._bridge.configure_session(session['arm_ids'], session['arm_mode'])
        if session['mode'] in ('watch', 'motion'):
            # Recovery clients exist for every production session (§6.13
            # recover applies to watch too — review finding S6); the jog
            # publishers/enable clients only for a jog controller.
            jog_controller = (session['controller_name']
                              if session['controller_name'] in config.JOG_CONTROLLERS
                              else None)
            self._bridge.configure_motion(session['arm_ids'], jog_controller)
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
            self._transition('running', reason=None)
            return
        if self._monotonic() > deadline:
            self._record_error('launch_timeout',
                               'the stack did not become ready within {:.0f} s'.format(
                                   config.STARTING_TIMEOUT_S))
            self._transition('stopping', reason='launch_timeout')

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
        for name in expected_state_broadcasters(session['arm_ids'], session['arm_mode']):
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
            self._force_enables_off()
            with self._state_lock:
                self._fault_since = rfc3339(self._utcnow())
                self._fault_reasons = tuple(reasons)
                self._fault_recoverable = FaultEngine.recoverable(session['mode'], reasons)
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
        # §7.3: fault -> running only when the whole recover sequence
        # succeeded AND the fault rules stop firing on the next tick.
        reasons = self._fault_engine.evaluate(self._fault_snapshot(session))
        with self._state_lock:
            self._recover_succeeded = False
            if reasons:
                self._fault_reasons = tuple(reasons)
                self._fault_recoverable = FaultEngine.recoverable(
                    session['mode'], reasons)
                return
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
        with self._state_lock:
            recorder = self._recording
            launch = self._launch
        failures = []
        if recorder is not None:
            try:
                recorder.stop()
            except Exception as error:
                failures.append('recorder stop failed: {}'.format(error))
        if launch is not None:
            try:
                if launch.alive():
                    launch.stop(config.STOP_SIGINT_WAIT_S,
                                config.STOP_SIGTERM_WAIT_S,
                                config.STOP_SIGKILL_WAIT_S)
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
            self._recording = None
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
        now_ns = int(self._monotonic() * 1e9)
        now = self._monotonic()
        arms = {}
        with self._state_lock:
            enabled = dict(self._arm_enabled)
            gains = self._gains
            models = dict(self._jog_models)
            slots = dict(self._arm_slots)
            published = dict(self._targets_published)
            last_publish = dict(self._last_publish_mono)
        for arm_id in session['arm_ids']:
            projection = health.project_arm(
                arm_id, now_ns,
                self._bridge.joint_sample(),
                self._bridge.robot_state_sample(arm_id),
                self._bridge.diagnostic_sample(arm_id))
            model = models.get(arm_id)
            fence = gains.fence.get(arm_id) if gains is not None else None
            pose_inside = None
            if fence and fence.get('position_lower') and projection['positions']:
                if all(p is not None for p in projection['positions']):
                    pose_inside = not _joints_outside_fence(
                        projection['positions'], fence['position_lower'],
                        fence['position_upper'])
            last_age = None
            if arm_id in last_publish:
                last_age = round(now - last_publish[arm_id], 3)
            projection['motion'] = {
                'available': model is not None,
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
            }
            arms[arm_id] = projection
        return arms
