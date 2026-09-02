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
The fault rules F1-F8 that move a running session to ``fault`` (plan section 7.1).

The supervisor builds a :class:`FaultSnapshot` on every 100 ms tick and hands it
to :meth:`FaultEngine.evaluate`, which returns one :class:`FaultReason` per
firing rule -- per arm for the per-arm rules. The engine is pure: it never
reads a clock other than the injected ``monotonic``, never touches ROS, and
holds exactly one piece of state, the per-arm window that makes F4's "sustained"
qualifier meaningful.

Rules, and the modes they are evaluated in:

===  =============================================================  ================
F1   canonical diagnostic ``level >= 2``                            watch, motion
F2   ``FrankaState.robot_mode`` in {REFLEX(4), USER_STOPPED(5)}     watch, motion
F3   any ``current_errors`` field true                              watch, motion
F4   ``control_command_success_rate`` below the gate, sustained     motion
F5   the session controller leaves ``active``                       motion
F6   ``/franka/joint_states`` stale                                 all
F7   the ``ros2 launch`` child exited                               all
F8   the hardware component is not ``active``                       watch, motion
===  =============================================================  ================

F1-F3 and F8 are evaluated **only** in the production modes, and that gate is
structural rather than defensive: mock hardware publishes no diagnostics and no
``FrankaState`` (plan section 0.2), so in ``simulate`` the inputs those rules read
are absent by construction and a value appearing there means the snapshot is
wrong, not that the robot is faulted. Poisoned simulate data therefore cannot
raise a production fault -- see the tests of the same name.

F4 is Motion-only. Phase 11 live state-only evidence showed that a healthy Watch
session legitimately reports CCSR 0.0, because no command stream exists to
succeed or fail. In Motion, the gate remains the user-signed one:
``defaults.CCSR_FAULT_THRESHOLD`` (0.95) sustained strictly longer than
``defaults.CCSR_FAULT_SUSTAIN_S`` (5.0 s), from the Phase 10 stop procedure. The
window is per arm. A sample at or above the threshold is the only thing that
resets it: an *absent* sample (no ``FrankaState`` at all) neither fires nor
resets, because silence is not evidence of recovery. :meth:`FaultEngine.reset`
clears every window and is what a new session calls.

Full session recovery addresses F1-F3/F6/F8 in Watch and F1-F6/F8 in Motion.
Eligibility requires *every* firing reason to be addressed, so F7 blocks the
button even in a mixed snapshot. F4 and F5 are Motion-only; a poisoned Watch
reason carrying either code is refused. F5 is recoverable for impedance because
the restore deactivates it first and activates it last; F6 is addressed by
restoring the joint-state broadcaster and waiting for a fresh sample.

:func:`classify_fault` turns a firing reason set plus the operator-lock state
into the five plain-words CAUSES the console renders -- a headline, the
recovery steps, and which primary button to draw. It is pure and it never
parses a detail string: the machine-readable evidence it needs
(:attr:`FaultReason.label`, :attr:`FaultReason.names`) is carried on the
reason itself and deliberately never reaches the wire.
"""

from dataclasses import dataclass
import math
import time

from franka_web import defaults

# --- mode names (the session's three modes; see plan section 3.4) ------------

MODE_SIMULATE = 'simulate'
MODE_WATCH = 'watch'
MODE_MOTION = 'motion'

# The modes backed by real Franka hardware, and so the only ones in which the
# diagnostic/FrankaState/hardware-component rules have inputs at all.
PRODUCTION_MODES = (MODE_WATCH, MODE_MOTION)

# --- thresholds pinned to their sources -------------------------------------

DIAGNOSTIC_ERROR_LEVEL = 2       # diagnostic_msgs level ERROR; plan section 0.5
ROBOT_MODE_REFLEX = 4            # franka_msgs/FrankaState; plan section 0.4
ROBOT_MODE_USER_STOPPED = 5
ACTIVE_STATE = 'active'          # lifecycle label shared by controllers and hardware

_ROBOT_MODE_FAULT_LABELS = {
    ROBOT_MODE_REFLEX: 'reflex',
    ROBOT_MODE_USER_STOPPED: 'user_stopped',
}

# --- the closed code set ----------------------------------------------------

FAULT_CODES = frozenset({
    'diagnostic_error',          # F1
    'robot_mode_fault',          # F2
    'robot_errors',              # F3
    'ccsr_low',                  # F4
    'controller_deactivated',    # F5
    'joint_state_stale',         # F6
    'launch_exited',             # F7
    'hardware_inactive',         # F8
    # Not a tick rule: the in-session pre-activation baseline check raises it
    # once, before any operator-commanded torque is possible. Deliberately
    # NOT recoverable -- a pose outside the fence is fixed by moving the arm.
    'baseline_outside_fence',
})

#: Five causes, not four: `protective_stop` (a reflex or limits violation) is
#: broken out from the robot-side family because a reflex stop has a genuinely
#: different recovery path from an external stop button.
FAULT_CAUSES = ('external_stop', 'protective_stop', 'robot_unreachable',
                'lock_expired', 'session_wedged')

#: The action the console draws its primary button from.
FAULT_ACTIONS = ('recover', 'reclaim', 'restart', 'none')

_PROTECTIVE_ERROR_SUFFIXES = ('_reflex', '_limits_violation')
_UNREACHABLE_ERRORS = ('communication_constraints_violation',)
_UNREACHABLE_CODES = ('joint_state_stale', 'ccsr_low', 'hardware_inactive')

# The codes the full recovery path actually addresses. F5 is included because
# impedance recovery now restores every broadcaster and activates the motion
# controller last. F6 is addressed by restoring JSB and requiring fresh joint
# data. F7 alone describes a dead launch child and blocks every mixed recovery.
RECOVERABLE_FAULT_CODES = frozenset({
    'diagnostic_error',
    'robot_mode_fault',
    'robot_errors',
    'ccsr_low',
    'controller_deactivated',
    'joint_state_stale',
    'hardware_inactive',
})


@dataclass(frozen=True)
class FaultReason:
    """
    One firing rule, in the shape the state frame publishes it.

    ``label`` and ``names`` are MACHINE-READABLE EVIDENCE for
    :func:`classify_fault` -- F2's robot-mode label and F3's current-error
    names. They never ship: :meth:`as_dict` carries the same three keys it
    always did, so classification never has to parse a detail sentence.
    """

    code: str
    arm_id: str | None
    detail: str
    label: str | None = None
    names: tuple = ()

    def as_dict(self):
        """Return the wire shape: code, arm_id, detail -- and nothing else."""
        return {'code': self.code, 'arm_id': self.arm_id, 'detail': self.detail}


@dataclass
class FaultSnapshot:
    """
    Everything the fault rules read, gathered by the supervisor on one tick.

    No field has a default: a rule that silently sees ``launch_alive=True``
    because the caller forgot to fill it in is a fault rule that does not fire,
    so the supervisor is made to state every input explicitly.

    ``arms`` maps ``arm_id`` to the per-arm dict of the health projection
    (``health.project_arm``); ``controller_states`` maps controller name to the
    lifecycle string reported by ``list_controllers``.
    """

    mode: str
    arms: dict
    controller_name: str | None
    controller_states: dict
    hardware_available: bool
    hardware_lifecycle_label: str | None
    launch_alive: bool


def _mapping(value):
    """Return ``value`` when it is a dict, otherwise an empty dict."""
    return value if isinstance(value, dict) else {}


def _section(arm, name):
    """Return the named sub-section of an arm projection, never ``None``."""
    return _mapping(_mapping(arm).get(name))


def _finite(value):
    """Return ``value`` as a finite float, or ``None`` if it is not a number."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _names(value):
    """Return a tuple of names from a list/tuple field, or an empty tuple."""
    if isinstance(value, (list, tuple)):
        return tuple(str(name) for name in value)
    return ()


def _text(value, fallback):
    """Return a non-empty string field, or ``fallback``."""
    return value if isinstance(value, str) and value else fallback


def _code_of(reason):
    """Return the code of a FaultReason or of its dict form."""
    if isinstance(reason, FaultReason):
        return reason.code
    if isinstance(reason, dict):
        return reason.get('code')
    return getattr(reason, 'code', None)


class FaultEngine:
    """
    Stateless fault rules plus the one stateful window F4 needs.

    One engine belongs to one session: the ccsr windows it accumulates are only
    meaningful within a single run of the stack, so a new session must either
    build a new engine or call :meth:`reset`.
    """

    def __init__(self, monotonic=time.monotonic):
        """Take the monotonic clock the F4 sustain window is measured against."""
        self._monotonic = monotonic
        self._ccsr_low_since = {}

    def reset(self):
        """Forget every F4 sustain window (a new session starts clean)."""
        self._ccsr_low_since.clear()

    def evaluate(self, snapshot):
        """
        Return one :class:`FaultReason` for every rule firing on ``snapshot``.

        The list is ordered by rule (F1 first, F8 last) and, within a per-arm
        rule, by the snapshot's own arm order -- the order the state frame and
        the Health card already use. An empty list means the session stays in
        ``running``.
        """
        arms = _mapping(snapshot.arms)
        production = snapshot.mode in PRODUCTION_MODES
        reasons = []
        if production:
            reasons.extend(self._diagnostic_errors(arms))
            reasons.extend(self._robot_mode_faults(arms))
            reasons.extend(self._robot_errors(arms))
        if snapshot.mode == MODE_MOTION:
            reasons.extend(self._ccsr_low(arms))
        reasons.extend(self._controller_deactivated(snapshot))
        reasons.extend(self._joint_state_stale(arms))
        reasons.extend(self._launch_exited(snapshot))
        if production:
            reasons.extend(self._hardware_inactive(snapshot))
        return reasons

    @staticmethod
    def recoverable(mode, reasons):
        """
        Say whether the Recover button may be offered for these reasons.

        True only in a production mode when every firing reason is addressed
        by the full restore sequence: F1-F3/F6/F8 in Watch, F1-F6/F8 in
        Motion. F7 makes the entire snapshot non-recoverable, including a
        mixed snapshot: a dead launch child requires stop-and-restart. The
        supervisor separately blocks Hold, whose activation itself commands
        effort.
        """
        if mode not in PRODUCTION_MODES:
            return False
        codes = tuple(_code_of(reason) for reason in reasons or ())
        addressed = RECOVERABLE_FAULT_CODES
        if mode == MODE_WATCH:
            # F4/F5 are structurally motion-only. Treat poisoned Watch reasons
            # conservatively instead of crediting command quality or a
            # controller that state-only Watch cannot own.
            addressed = addressed - {'ccsr_low', 'controller_deactivated'}
        return bool(codes) and all(code in addressed for code in codes)

    # --- F1 -----------------------------------------------------------------

    def _diagnostic_errors(self, arms):
        """Yield F1 for each arm whose canonical diagnostic is at level >= 2."""
        for arm_id, arm in arms.items():
            diagnostic = _section(arm, 'diagnostic')
            level = diagnostic.get('level')
            if isinstance(level, bool) or not isinstance(level, int):
                # None (no canonical status seen) or a non-numeric level: the
                # rule has no input, which is not the same as a healthy arm.
                continue
            if level < DIAGNOSTIC_ERROR_LEVEL:
                continue
            message = _text(diagnostic.get('message'), 'no diagnostic summary reported')
            yield FaultReason(
                code='diagnostic_error',
                arm_id=arm_id,
                detail='canonical diagnostic reports level {} (>= {}): {}'.format(
                    level, DIAGNOSTIC_ERROR_LEVEL, message),
            )

    # --- F2 -----------------------------------------------------------------

    def _robot_mode_faults(self, arms):
        """Yield F2 for each arm reporting REFLEX or USER_STOPPED."""
        for arm_id, arm in arms.items():
            robot_mode = _section(arm, 'robot_state').get('robot_mode')
            if isinstance(robot_mode, bool) or robot_mode not in _ROBOT_MODE_FAULT_LABELS:
                continue
            yield FaultReason(
                code='robot_mode_fault',
                arm_id=arm_id,
                detail='robot_mode is {} ({})'.format(
                    _ROBOT_MODE_FAULT_LABELS[robot_mode], robot_mode),
                label=_ROBOT_MODE_FAULT_LABELS[robot_mode],
            )

    # --- F3 -----------------------------------------------------------------

    def _robot_errors(self, arms):
        """Yield F3 for each arm reporting at least one current error."""
        for arm_id, arm in arms.items():
            names = _names(_section(arm, 'robot_state').get('current_errors'))
            if not names:
                continue
            yield FaultReason(
                code='robot_errors',
                arm_id=arm_id,
                detail='robot reports current errors: {}'.format(', '.join(names)),
                names=tuple(names),
            )

    # --- F4 -----------------------------------------------------------------

    def _ccsr_low(self, arms):
        """
        Return F4 for each arm below the ccsr gate for longer than the window.

        The window opens on the first below-threshold sample and closes only on
        a sample at or above the threshold; an absent ``FrankaState`` holds it
        open, since no sample is not evidence of recovery. Windows for arms that
        left the snapshot are dropped so a restarted arm starts clean.

        This is the one rule that mutates engine state, so it is a plain
        function rather than a generator: the window bookkeeping must happen
        whether or not the caller consumes every reason.
        """
        now = self._monotonic()
        reasons = []
        for arm_id, arm in arms.items():
            ccsr = _finite(_section(arm, 'robot_state').get('control_command_success_rate'))
            if ccsr is None:
                continue
            if ccsr >= defaults.CCSR_FAULT_THRESHOLD:
                self._ccsr_low_since.pop(arm_id, None)
                continue
            since = self._ccsr_low_since.setdefault(arm_id, now)
            elapsed = now - since
            if elapsed <= defaults.CCSR_FAULT_SUSTAIN_S:
                continue
            detail = 'control command success rate {:.3f} stayed below {:.2f} for {:.1f} s'
            reasons.append(FaultReason(
                code='ccsr_low',
                arm_id=arm_id,
                detail=detail.format(ccsr, defaults.CCSR_FAULT_THRESHOLD, elapsed),
            ))
        for stale_arm in [key for key in self._ccsr_low_since if key not in arms]:
            del self._ccsr_low_since[stale_arm]
        return reasons

    # --- F5 -----------------------------------------------------------------

    def _controller_deactivated(self, snapshot):
        """
        Yield F5 when the session controller is not active in ``motion``.

        This is how an effort-envelope ``ERROR`` return from the impedance
        controller surfaces (plan section 0.6). With no session controller named
        there is nothing to check -- only ``motion`` sessions carry one, and the
        supervisor always names it there.
        """
        if snapshot.mode != MODE_MOTION or not snapshot.controller_name:
            return
        state = _mapping(snapshot.controller_states).get(snapshot.controller_name)
        if state == ACTIVE_STATE:
            return
        yield FaultReason(
            code='controller_deactivated',
            arm_id=None,
            detail='session controller {} is {} (expected {})'.format(
                snapshot.controller_name, _text(state, 'not loaded'), ACTIVE_STATE),
        )

    # --- F6 -----------------------------------------------------------------

    def _joint_state_stale(self, arms):
        """Yield F6 for each arm the health projection marked stale."""
        for arm_id, arm in arms.items():
            if not _mapping(arm).get('positions_stale'):
                continue
            age = _finite(_mapping(arm).get('positions_age_s'))
            limit = defaults.JOINT_STATE_STALE_FAULT_S
            if age is None:
                detail = 'joint states are stale (no sample within {:.1f} s)'.format(limit)
            else:
                template = 'joint states are stale (last sample {:.2f} s old, limit {:.1f} s)'
                detail = template.format(age, limit)
            yield FaultReason(code='joint_state_stale', arm_id=arm_id, detail=detail)

    # --- F7 -----------------------------------------------------------------

    def _launch_exited(self, snapshot):
        """Yield F7 when the ``ros2 launch`` child is no longer running."""
        if snapshot.launch_alive:
            return
        yield FaultReason(
            code='launch_exited',
            arm_id=None,
            detail='the ros2 launch child exited while the session was running',
        )

    # --- F8 -----------------------------------------------------------------

    def _hardware_inactive(self, snapshot):
        """Yield F8 when the hardware component is missing or not active."""
        if not snapshot.hardware_available:
            yield FaultReason(
                code='hardware_inactive',
                arm_id=None,
                detail='the hardware component is not present in list_hardware_components',
            )
            return
        if snapshot.hardware_lifecycle_label == ACTIVE_STATE:
            return
        yield FaultReason(
            code='hardware_inactive',
            arm_id=None,
            detail='the hardware component lifecycle is {} (expected {})'.format(
                _text(snapshot.hardware_lifecycle_label, 'unknown'), ACTIVE_STATE),
        )


# ---------------------------------------------------------------------------
# Cause classification: reasons + lock state -> the console's fault banner
# ---------------------------------------------------------------------------

_NO_CAUSE = {'cause': None, 'arm_id': None, 'headline': None,
             'steps': [], 'action': 'none'}

_HEADLINES = {
    'lock_expired': 'Your control expired while the fault was handled.',
    'external_stop': '{arm} stopped: an external stop button is pressed.',
    'protective_stop': '{arm} stopped itself: a protective limit was reached.',
    'robot_unreachable': '{arm} stopped: communication with the robot failed.',
    'session_wedged': 'The session stopped and cannot continue.',
}

_STEPS = {
    'lock_expired': ('Press Reclaim, then Recover.',),
    'external_stop': ('Release the stop button on the robot.', 'Press Recover.'),
    'protective_stop': ('Check nothing is obstructing the arm.', 'Press Recover.'),
    'robot_unreachable': (
        'Check that nobody pressed a stop button.',
        "Press Recover. If it fails again, check the robot's Desk page."),
    'session_wedged': ('Press Stop, then start a new session.',
                       'Open the logs to see what failed.'),
}


def display_arm(arm_id):
    """Return the operator-facing name of an arm (``panda2`` -> ``Panda 2``)."""
    if not isinstance(arm_id, str) or not arm_id:
        return 'The arm'
    return arm_id.replace('panda', 'Panda ')


def _reason_fields(reason):
    """Return ``(code, arm_id, label, names)`` for a reason or its dict form."""
    if isinstance(reason, FaultReason):
        return reason.code, reason.arm_id, reason.label, tuple(reason.names)
    if isinstance(reason, dict):
        return (reason.get('code'), reason.get('arm_id'),
                reason.get('label'), _names(reason.get('names')))
    return (getattr(reason, 'code', None), getattr(reason, 'arm_id', None),
            getattr(reason, 'label', None), _names(getattr(reason, 'names', ())))


def _is_external_stop(code, _arm_id, label, _names_):
    """Priority 2: an external stop button is pressed on this arm."""
    return code == 'robot_mode_fault' and label == 'user_stopped'


def _is_protective_stop(code, _arm_id, label, names):
    """Priority 3: a reflex or a limits violation stopped the arm itself."""
    if code == 'robot_mode_fault' and label == 'reflex':
        return True
    return code == 'robot_errors' and any(
        name.endswith(_PROTECTIVE_ERROR_SUFFIXES) for name in names)


def _is_unreachable(code, _arm_id, _label, names):
    """Priority 4: the robot cannot be talked to (or is not publishing)."""
    if code == 'robot_errors' and any(
            name in _UNREACHABLE_ERRORS for name in names):
        return True
    return code in _UNREACHABLE_CODES


#: (cause, predicate) in the contract's exact priority order. `lock_expired`
#: is handled ahead of these because it reads the lock rather than a reason,
#: and `session_wedged` after them because it is the fallback.
_CAUSE_PREDICATES = (
    ('external_stop', _is_external_stop),
    ('protective_stop', _is_protective_stop),
    ('robot_unreachable', _is_unreachable),
)


def _first_matching_arm(reasons, arm_ids, predicate):
    """Return the first arm in session order whose reason matches, or None."""
    matching = set()
    for reason in reasons:
        fields = _reason_fields(reason)
        if predicate(*fields):
            matching.add(fields[1])
    for arm_id in arm_ids or ():
        if arm_id in matching:
            return arm_id
    for candidate in matching:
        if candidate is not None:
            return candidate
    return None


def classify_fault(*, reasons, active, arm_ids, recoverable,
                   operator_locked, operator_claim_id, session_claim_id):
    """
    Return the frame's ``cause``/``arm_id``/``headline``/``steps``/``action``.

    Evaluated in the contract's exact priority order; the FIRST match wins.
    ``lock_expired`` outranks everything because a Recover press cannot
    succeed without the lock, so the page must ask for a Reclaim first --
    that stacked-cause confusion is exactly what this ordering removes. When
    several arms fault at once the cause is that of the highest-priority row
    and ``arm_id`` names the first arm in ``arm_ids`` order matching it.
    """
    if not active:
        return dict(_NO_CAUSE, steps=[])
    reasons = tuple(reasons or ())
    arm_ids = tuple(arm_ids or ())

    if not operator_locked or (
            session_claim_id is not None
            and operator_claim_id != session_claim_id):
        return _block('lock_expired', None, 'reclaim')

    for cause, predicate in _CAUSE_PREDICATES:
        arm_id = _first_matching_arm(reasons, arm_ids, predicate)
        if arm_id is not None or any(
                predicate(*_reason_fields(reason)) for reason in reasons):
            return _block(cause, arm_id, 'recover')

    return _block('session_wedged', None,
                  'recover' if recoverable else 'restart')


def _block(cause, arm_id, action):
    """Render one classified cause into the frame's fault sub-object."""
    return {
        'cause': cause,
        'arm_id': arm_id,
        'headline': _HEADLINES[cause].format(arm=display_arm(arm_id)),
        'steps': list(_STEPS[cause]),
        'action': action,
    }
