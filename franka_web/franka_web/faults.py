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
F4   ``control_command_success_rate`` below the gate, sustained     watch, motion
F5   the session controller leaves ``active``                       motion
F6   ``/franka/joint_states`` stale                                 all
F7   the ``ros2 launch`` child exited                               all
F8   the hardware component is not ``active``                       watch, motion
===  =============================================================  ================

F1-F4 and F8 are evaluated **only** in the production modes, and that gate is
structural rather than defensive: mock hardware publishes no diagnostics and no
``FrankaState`` (plan section 0.2), so in ``simulate`` the inputs those rules read
are absent by construction and a value appearing there means the snapshot is
wrong, not that the robot is faulted. Poisoned simulate data therefore cannot
raise a production fault -- see the tests of the same name.

F4's gate is the user-signed one: ``config.CCSR_FAULT_THRESHOLD`` (0.95)
sustained strictly longer than ``config.CCSR_FAULT_SUSTAIN_S`` (5.0 s), from the
Phase 10 stop procedure. The window is per arm. A sample at or above the
threshold is the only thing that resets it: an *absent* sample (no
``FrankaState`` at all) neither fires nor resets, because silence is not
evidence of recovery. :meth:`FaultEngine.reset` clears every window and is what
a new session calls.

Nothing here composes an operator-facing string from a robot address; details
are built from health-projection fields only, none of which carry one.
"""

from dataclasses import dataclass
import math
import time

from franka_web import config

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
    'diagnostic_error',        # F1
    'robot_mode_fault',        # F2
    'robot_errors',            # F3
    'ccsr_low',                # F4
    'controller_deactivated',  # F5
    'joint_state_stale',       # F6
    'launch_exited',           # F7
    'hardware_inactive',       # F8
})

# The codes the recovery path (plan section 7.3) actually addresses. F5/F6/F7
# describe a stack that is gone or wedged; for those the UI says "stop and
# restart the session" instead of offering Recover.
RECOVERABLE_FAULT_CODES = frozenset({
    'diagnostic_error',
    'robot_mode_fault',
    'robot_errors',
    'ccsr_low',
    'hardware_inactive',
})


@dataclass(frozen=True)
class FaultReason:
    """One firing rule, in the shape the state frame publishes it."""

    code: str
    arm_id: str | None
    detail: str

    def as_dict(self):
        """Return the plan section 7.1 wire shape: code, arm_id, detail."""
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

        True only in a production mode and only when at least one reason is one
        the recovery sequence addresses (F1-F4, F8). A fault made up solely of
        F5/F6/F7 is not recoverable: the controller, the joint stream or the
        launch child is gone, and only a stop-and-restart fixes that.
        """
        if mode not in PRODUCTION_MODES:
            return False
        return any(_code_of(reason) in RECOVERABLE_FAULT_CODES for reason in reasons or ())

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
            if ccsr >= config.CCSR_FAULT_THRESHOLD:
                self._ccsr_low_since.pop(arm_id, None)
                continue
            since = self._ccsr_low_since.setdefault(arm_id, now)
            elapsed = now - since
            if elapsed <= config.CCSR_FAULT_SUSTAIN_S:
                continue
            detail = 'control command success rate {:.3f} stayed below {:.2f} for {:.1f} s'
            reasons.append(FaultReason(
                code='ccsr_low',
                arm_id=arm_id,
                detail=detail.format(ccsr, config.CCSR_FAULT_THRESHOLD, elapsed),
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
            limit = config.JOINT_STATE_STALE_FAULT_S
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
