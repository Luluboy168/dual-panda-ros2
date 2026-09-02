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
Ghost pose authoring, from a dragged hand position to seven joint angles.

Validation, the IK field mapping, the checker adapter, the verdict sentence
and the rate bucket all live here.

This module commands nothing. It can send nothing to a robot, it calls no
service that turns an arm on, it touches no session state, and it imports no
module that does -- including transitively, which is the property that
actually matters and the one test_ghost_solve.py's TestNoMotionPath asserts.
It imports no ROS client library either: the solver arrives as a callable, so
the whole surface can be driven from a unit test on a machine with no ROS
installed at all.

That last sentence is why franka_web.health is NOT imported here. It reaches
franka_bringup.status, which pulls in the ROS client library and three
controller-manager services, so importing it would drag the entire ROS stack
into ``import franka_web.ghost``. The one name wanted from there,
joint_names_for, is four lines; health.py owns the canonical spelling, it is
spelled a second time in ghost_copy.py, and a test asserts the two agree.

The Copy payload lives in ghost_copy.py, not here: its templates contain the
words a motion-vocabulary scan must forbid in this file.
"""

from dataclasses import dataclass, field
import math
import re
import threading
import time

from franka_web import defaults
from franka_web.ghost_copy import build_copy, joint_names_for

#: The solver's own enumeration, exactly as its contract states it.
RESULT_SUCCESS = 0
TIP_FLANGE = 0
SOLVER_DEFAULT = 0
REDUNDANCY_FROM_SEED = 0
REDUNDANCY_FIXED = 1

#: One plain sentence per way the solver can decline, and no branching logic.
_SOLVE_REASON = {
    6: "That point is outside this arm's reach.",
    10: 'No arm posture reaches that point from here. Try moving in a '
        'smaller step.',
    7: 'Reaching that point would push a joint past its limit.',
    8: 'The solver could not settle on that point. Try moving in a '
       'smaller step.',
    9: 'The solver could not settle on that point. Try moving in a '
       'smaller step.',
    5: 'The ghost is in a pose the solver will not start from. Press '
       'Reset ghost.',
}

#: BAD_REQUEST / UNKNOWN_ARM / UNKNOWN_FRAME / UNSUPPORTED_TIP /
#: INTERNAL_ERROR: a server bug or a mismatched IK description. One short
#: sentence for the operator; the service's own message goes to the log bus
#: at warn, where a developer can read which arm or frame it actually refused.
_SOLVE_REASON_REFUSED = 'The IK service refused this request.'
_REFUSED_RESULTS = (1, 2, 3, 4, 11)

_DETAIL_UNKNOWN_ARM = 'arm_id must be one of: {}'.format(
    ', '.join(defaults.ARM_IDS))


class GhostError(Exception):
    """A refused ghost request, carrying a closed-set code and its evidence."""

    def __init__(self, code, detail, payload=None):
        """Store the code, the operator-facing detail and its payload."""
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.payload = dict(payload or {})


@dataclass(frozen=True)
class IkCall:
    """The solver field mapping, in plain Python with no ROS types."""

    frame_id: str            # "<arm_id>_link0" -- never ""
    arm_id: str
    tip_frame: int           # the flange; the dual description has no hand
    position: tuple          # 3 floats, metres, in frame_id
    orientation: tuple       # (x, y, z, w), normalised
    seed_positions: tuple    # 7 floats, rad
    redundancy_mode: int
    redundancy_value: float  # finite EVEN in from-seed mode: the service
    #                          refuses a non-finite value before it looks at
    #                          the mode at all
    max_solutions: int = 1   # v1 returns at most one however this is set
    solver: int = SOLVER_DEFAULT
    position_tolerance: float = 0.0      # 0.0 selects the node default 1e-4 m
    orientation_tolerance: float = 0.0   # 0.0 selects 1e-3 rad
    joint_limit_margin: float = 0.0


@dataclass(frozen=True)
class IkReply:
    """The solver result fields this endpoint reads, and nothing else."""

    result: int
    message: str = ''
    positions: tuple = ()
    redundancy_value: float = 0.0
    position_error: float = 0.0
    orientation_error: float = 0.0


@dataclass(frozen=True)
class SolveRequest:
    """One validated solve request."""

    arm_id: str
    seed: tuple
    position: tuple
    orientation: tuple
    redundancy_mode: int
    redundancy_value: float
    scene: dict = field(default_factory=dict)


# ----------------------------------------------------------------------
# The verdict adapter
# ----------------------------------------------------------------------

_LINK_WORDS = {'link0': 'base', 'link1': 'shoulder', 'link2': 'upper arm',
               'link3': 'elbow', 'link4': 'forearm', 'link5': 'forearm',
               'link6': 'wrist', 'link7': 'hand mount', 'link8': 'flange'}
_FACE_WORDS = {'z_min': 'through the table top', 'z_max': 'through the ceiling',
               'x_min': 'past the near edge', 'x_max': 'past the far edge',
               'y_min': 'past the panda 2 side', 'y_max': 'past the panda 1 side'}
_DISPLAY = {'panda1': 'Panda 1', 'panda2': 'Panda 2'}

#: Which of a contact's two identifier fields actually name a link. `a` is a
#: JOINT name for joint_limit; `b` is a "<box>.<face>" string for
#: containment, a solid id for environment and a zone id for keep_out. A
#: blanket union over both fields would put panda1_joint4 or work_area.z_min
#: into a list the renderer feeds to a link-mesh lookup.
_LINK_FIELDS = {
    'self': ('a', 'b'),
    'cross_arm': ('a', 'b'),
    'containment': ('a',),
    'environment': ('a',),
    'keep_out': ('a',),
    'joint_limit': (),
}
_LINK_NAME = re.compile(r'^panda[12]_link[0-8]$')


def link_of(volume_id):
    """Turn 'panda1_link5_v1' into 'panda1_link5'; a link name passes through."""
    head, separator, tail = str(volume_id).rpartition('_v')
    return head if separator and tail.isdigit() else str(volume_id)


def offending_links_for(contacts):
    """Return the sorted link names to tint, and only link names."""
    found = set()
    for contact in contacts:
        for name in _LINK_FIELDS.get(contact.kind, ()):
            candidate = link_of(getattr(contact, name))
            if _LINK_NAME.match(candidate):
                found.add(candidate)
    return sorted(found)


def _plain_part(identifier):
    """Return the operator's word for a link, or the identifier itself."""
    name = link_of(identifier)
    return _LINK_WORDS.get(name.rpartition('_')[2], name)


def _display(arm_id):
    """Return the console's display name for an arm id."""
    return _DISPLAY.get(arm_id, arm_id)


def _millimetres(contact):
    """Return abs(distance - required) in whole millimetres."""
    return int(round(abs(float(contact.distance) - float(contact.required)) * 1000.0))


def verdict_sentence(contact):
    """Return one plain sentence naming what would hit what, and by how much."""
    who = _display(contact.arm_id)
    if contact.kind == 'joint_limit':
        degrees = abs(math.degrees(float(contact.distance)))
        return '{} joint {} is {:.1f}° past its limit.'.format(
            who, str(contact.a)[-1], degrees)
    gap = _millimetres(contact)
    tail = ' — just touching.' if gap == 0 else ' — {} mm too close.'.format(gap)
    part = _plain_part(contact.a)
    if contact.kind == 'containment':
        _box, _dot, face = str(contact.b).rpartition('.')
        by = ' by {} mm.'.format(gap) if gap else '.'
        return "{}'s {} would leave the work area {}{}".format(
            who, part, _FACE_WORDS.get(face, 'through its edge'), by)
    if contact.kind == 'keep_out':
        return "{}'s {} would enter the keep-out zone {}.".format(
            who, part, contact.b)
    if contact.kind == 'cross_arm':
        other = link_of(contact.b)
        return "{}'s {} would hit {}'s {}{}".format(
            who, part, _display(other.split('_')[0]), _plain_part(other), tail)
    if contact.kind == 'self':
        return "{}'s {} would hit its own {}{}".format(
            who, part, _plain_part(contact.b), tail)
    return "{}'s {} would hit {}{}".format(who, part, contact.b, tail)


def _finite_or_none(value):
    """Return a float the JSON envelope can carry, or None."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


# ----------------------------------------------------------------------
# The rate bucket
# ----------------------------------------------------------------------


class TokenBucket:
    """One global bucket over both ghost POSTs; capacity 60, 30/s refill."""

    def __init__(self, capacity, refill_hz, monotonic=time.monotonic):
        """Start full, so the first drag of a session is never refused."""
        self._lock = threading.Lock()
        self._capacity = float(capacity)
        self._refill = float(refill_hz)
        self._monotonic = monotonic
        self._tokens = float(capacity)
        self._stamp = monotonic()

    def take(self, cost):
        """
        Return None when the cost was paid, else the wait in whole milliseconds.

        The wait is computed for THIS request's own cost, which is why the
        argument is used rather than a flat constant: a solve costs one token
        and a 25-sample sweep costs 25, so one flat retry would send a
        refused sweep straight into a second refusal.
        """
        with self._lock:
            now = self._monotonic()
            self._tokens = min(
                self._capacity,
                self._tokens + (now - self._stamp) * self._refill)
            self._stamp = now
            if self._tokens >= cost:
                self._tokens -= cost
                return None
            return max(1, int(math.ceil(
                (cost - self._tokens) / self._refill * 1000.0)))


# ----------------------------------------------------------------------
# Validation helpers
# ----------------------------------------------------------------------


def _invalid(detail):
    """Return the refusal a malformed field produces."""
    return GhostError('invalid_json', detail)


def _finite(value):
    """Return True for a real number that is not a bool and not a NaN."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def _vector(value, length, name):
    """Return ``length`` finite floats from ``value``, or raise."""
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise _invalid('{} must be {} numbers'.format(name, length))
    if not all(_finite(item) for item in value):
        raise _invalid('{} must be {} finite numbers'.format(name, length))
    return tuple(float(item) for item in value)


def _session_view_defaults():
    """Return the empty session view, used when nothing supplies one."""
    return {'arm_ids': [], 'command_topics': {}}


def session_view_from(frame_source, monotonic=time.monotonic,
                      ttl_s=defaults.GHOST_SESSION_VIEW_TTL_S):
    """
    Return a cached reader of the two session facts a solve needs.

    The state frame is rebuilt at 5 Hz anyway and a drag may solve at 30 Hz,
    so this caches for one frame period. The only values read change at
    session boundaries, so a value up to one frame old can at worst put a
    just-stopped session's topic into a pasted comment line.
    """
    state = {'stamp': None, 'value': _session_view_defaults()}
    lock = threading.Lock()

    def read():
        now = monotonic()
        with lock:
            stamp = state['stamp']
            if stamp is not None and now - stamp < ttl_s:
                return state['value']
        try:
            frame = frame_source()
            session = frame.get('session') or {}
            arms = frame.get('arms') or {}
            value = {
                'arm_ids': list(session.get('arm_ids') or []),
                'command_topics': {
                    arm_id: ((arms.get(arm_id) or {}).get('motion') or {}).get(
                        'command_topic')
                    for arm_id in defaults.ARM_IDS},
            }
        except Exception:      # noqa: BLE001 - a read-only peek never refuses
            value = _session_view_defaults()
        with lock:
            state['stamp'] = now
            state['value'] = value
        return value

    return read


class GhostService:
    """The whole ghost endpoint surface, with ROS and HTTP kept outside."""

    def __init__(self, *, solver, checker, session_view=None, ik_ready=None,
                 monotonic=time.monotonic, log_bus=None):
        """
        Wire the three collaborators, all of them plain callables or objects.

        ``solver`` is ``(IkCall, timeout_s) -> IkReply | None``, where None
        means "no answer". ``ik_ready`` is the point-in-time readiness probe
        the scene reports and the timeout/not-ready discriminator reads.
        """
        self._solver = solver
        self._checker = checker
        self._session_view = session_view or _session_view_defaults
        self._ik_ready = ik_ready or (lambda: True)
        self._monotonic = monotonic
        self._log_bus = log_bus
        self._bucket = TokenBucket(defaults.GHOST_RATE_CAPACITY,
                                   defaults.GHOST_RATE_REFILL_HZ,
                                   monotonic=monotonic)

    # -- the read endpoint ---------------------------------------------

    def profile(self):
        """
        Return the checker profile this session calls for.

        Reconciled from the session view rather than pushed in at session
        start: reading the session is something a ghost may do, and writing
        one is not.
        """
        arm_ids = self._session_view().get('arm_ids') or []
        return 'single' if len(arm_ids) == 1 else 'dual'

    def scene(self, assets=None):
        """Return the static scene facts: assets, cell, checker and IK state."""
        status = self._checker.status(self.profile())
        available = bool(self._ik_ready())
        return {
            'assets': assets,
            'cell': status['cell'],
            'cell_source': status['cell_source'],
            'cell_note': status['cell_note'],
            'model': status['model'],
            'arms': list(defaults.ARM_IDS),
            'ik': {'available': available,
                   'arm_ids': list(defaults.ARM_IDS),
                   'tip_frame': 'flange'},
            'checker': {'available': status['available'],
                        'profile': status['profile'],
                        'interlock': status['interlock'],
                        'note': status['checker_note']},
            # The ghost is editable exactly when IK is reachable; the checker
            # being absent degrades the verdict, never the editing.
            'ghost_available': available,
        }

    # -- the two compute endpoints -------------------------------------

    def solve(self, body):
        """Solve one hand pose into joint angles, check it, and describe it."""
        request = self._validate_solve(body)
        self._charge(1)
        profile = self.profile()
        reply = self._call(self._ik_call(request, request.redundancy_mode,
                                         request.redundancy_value),
                           defaults.GHOST_SOLVE_TIMEOUT_S)
        positions, reason = self._read_reply(reply)
        if positions is None:
            return {'arm_id': request.arm_id, 'solved': False,
                    'positions': None, 'positions_deg': None,
                    'redundancy_value': None,
                    'position_error': None, 'orientation_error': None,
                    'solve_reason': reason,
                    # There is no new pose to judge, so there is no verdict.
                    # "unchecked" would flip the ghost to the neutral
                    # treatment although nothing about it changed, and
                    # "clear" would be a lie about a pose never evaluated.
                    'verdict': None, 'copy': None}
        verdict = self._verdict(profile, request, positions)
        topic = self._session_view().get('command_topics', {}).get(request.arm_id)
        return {
            'arm_id': request.arm_id,
            'solved': True,
            'positions': [float(value) for value in positions],
            'positions_deg': [round(math.degrees(float(value)), 2)
                              for value in positions],
            'redundancy_value': _finite_or_none(reply.redundancy_value),
            'position_error': _finite_or_none(reply.position_error),
            'orientation_error': _finite_or_none(reply.orientation_error),
            'solve_reason': None,
            'verdict': verdict,
            'copy': build_copy(request.arm_id, positions, verdict, topic),
        }

    def redundancy(self, body):
        """Return the q7 sample table for one fixed hand pose."""
        request, samples = self._validate_redundancy(body)
        self._charge(samples)
        if not self._ik_ready():
            raise self._unavailable(False)
        lower = defaults.POLICY_POSITION_LOWER_RAD[defaults.JOINT_COUNT - 1]
        upper = defaults.POLICY_POSITION_UPPER_RAD[defaults.JOINT_COUNT - 1]
        deadline = self._monotonic() + defaults.GHOST_REDUNDANCY_TIMEOUT_S
        table = []
        for index in range(samples):
            remaining = deadline - self._monotonic()
            if remaining <= 0.0:
                break
            value = lower + (upper - lower) * index / float(samples - 1)
            # A sample that does not answer is OMITTED rather than refused:
            # a partial table is a state the client is built for, and a
            # sweep that gave up halfway is still useful.
            reply = self._solver(
                self._ik_call(request, REDUNDANCY_FIXED, value),
                min(defaults.GHOST_SOLVE_TIMEOUT_S, remaining))
            if reply is None or reply.result != RESULT_SUCCESS or not reply.positions:
                continue
            table.append({'q7': value,
                          'positions': [float(item) for item in reply.positions]})
        return {'arm_id': request.arm_id,
                # The REQUESTED count after clamping, deliberately not
                # len(table): the client compares the two to decide whether
                # the ring can be driven from this table at all.
                'samples': samples,
                'table': table}

    # -- internals -----------------------------------------------------

    def _charge(self, cost):
        """Take ``cost`` tokens or refuse with the wait this request needs."""
        wait = self._bucket.take(cost)
        if wait is not None:
            raise GhostError('ghost_rate_limited',
                             'too many ghost requests; slow the drag',
                             {'retry_after_ms': wait})

    def _ik_call(self, request, mode, value):
        """Build the solver call for one target, in the arm's own base frame."""
        return IkCall(
            frame_id='{}_link0'.format(request.arm_id),
            arm_id=request.arm_id,
            tip_frame=TIP_FLANGE,
            position=request.position,
            orientation=request.orientation,
            seed_positions=request.seed,
            redundancy_mode=mode,
            redundancy_value=float(value),
            max_solutions=1,
            solver=SOLVER_DEFAULT,
            position_tolerance=0.0,
            orientation_tolerance=0.0,
            joint_limit_margin=0.0)

    def _call(self, call, timeout_s):
        """
        Call the solver once, or refuse the request when it cannot answer.

        The readiness probe is taken BEFORE the call and only ever used to
        say WHICH of the two failures happened: a client that reported
        not-ready and a client that answered nothing are one return value
        away from each other, and the panel teaches a different sentence for
        each.
        """
        ready = bool(self._ik_ready())
        reply = self._solver(call, timeout_s)
        if reply is None:
            raise self._unavailable(ready)
        return reply

    @staticmethod
    def _unavailable(ready):
        """Return the 503 for an IK service that is absent or silent."""
        return GhostError(
            'ghost_unavailable',
            'the IK service is not running; start it with '
            '"ros2 launch franka_ik franka_ik.launch.py"',
            {'ik_state': 'timeout' if ready else 'not_ready'})

    def _read_reply(self, reply):
        """Return (positions, None) on success, else (None, sentence)."""
        if reply.result == RESULT_SUCCESS and reply.positions:
            return tuple(float(value) for value in reply.positions), None
        if reply.result == RESULT_SUCCESS:
            # A success with no solution is a service bug, not a pose the
            # operator can do anything about. It must not become a null
            # dereference in the happy path.
            self._warn(reply.message or 'the IK service answered with no solution')
            return None, _SOLVE_REASON_REFUSED
        if reply.result in _REFUSED_RESULTS:
            self._warn(reply.message)
            return None, _SOLVE_REASON_REFUSED
        return None, _SOLVE_REASON.get(reply.result, _SOLVE_REASON_REFUSED)

    def _warn(self, message):
        """Put the service's own words on the log bus, where a developer reads."""
        if self._log_bus is not None and message:
            self._log_bus.emit('warn', str(message))

    def _verdict(self, profile, request, positions):
        """
        Return the verdict for the pose the user is ABOUT to see.

        The checked scene is the request's own scene with THIS arm replaced
        by the solution the solver just returned. Checking the pose the
        client sent would tint the arm one drag frame late in both
        directions: green while the ghost enters the table, red after it has
        left. Every other arm comes from the client verbatim and is never
        re-derived here.
        """
        checked = dict(request.scene)
        checked[request.arm_id] = tuple(positions)
        result, sentence, code = self._checker.check(profile, checked)
        if result is None:
            return {'status': 'unchecked', 'min_clearance': None,
                    'offending_links': [], 'reason': sentence,
                    'reason_code': code,
                    # Nothing looked at this pose, so no cell model answered.
                    'checker': 'absent'}
        clearance = _finite_or_none(result.min_clearance)
        if result.ok:
            return {'status': 'clear', 'min_clearance': clearance,
                    'offending_links': [], 'reason': None,
                    'reason_code': None, 'checker': 'cell_model'}
        contacts = tuple(result.contacts)
        return {'status': 'collision', 'min_clearance': clearance,
                'offending_links': offending_links_for(contacts),
                'reason': verdict_sentence(contacts[0]),
                'reason_code': 'contact', 'checker': 'cell_model'}

    # -- validation ----------------------------------------------------

    def _validate_target(self, body):
        """Return the validated (position, orientation) of one target."""
        target = body.get('target')
        if not isinstance(target, dict):
            raise _invalid("'target' must be an object with a position and an "
                           'orientation')
        position = _vector(target.get('position'), 3, 'target.position')
        orientation = _vector(target.get('orientation'), 4, 'target.orientation')
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm < 1e-6:
            raise _invalid('target.orientation must not be a zero quaternion')
        return position, tuple(value / norm for value in orientation)

    def _validate_arm(self, body):
        """Return the request's arm id, or refuse by naming the two that exist."""
        arm_id = body.get('arm_id')
        if arm_id not in defaults.ARM_IDS:
            raise GhostError('invalid_arms', _DETAIL_UNKNOWN_ARM)
        return arm_id

    def _validate_scene(self, body):
        """
        Return the rendered scene the client sent, dropping unknown arms.

        A null value for a known arm is an OMITTED arm, not a malformed
        request: refusing a whole gesture because one arm's pose was
        momentarily unknown would be the worst available failure of an
        interactive surface. Only a non-null value that is not seven finite
        numbers is a refusal.
        """
        scene = body.get('scene')
        if scene is None:
            return {}
        if not isinstance(scene, dict):
            raise _invalid("'scene' must be an object of arm poses")
        checked = {}
        for arm_id, value in scene.items():
            if arm_id not in defaults.ARM_IDS or value is None:
                continue
            checked[arm_id] = _vector(
                value, defaults.JOINT_COUNT, 'scene.{}'.format(arm_id))
        return checked

    def _validate_solve(self, body):
        """Return one validated solve request, or raise its refusal."""
        if not isinstance(body, dict):
            raise _invalid('the request body must be a JSON object')
        arm_id = self._validate_arm(body)
        seed = _vector(body.get('seed'), defaults.JOINT_COUNT, "'seed'")
        position, orientation = self._validate_target(body)
        redundancy = body.get('redundancy')
        if redundancy is None:
            redundancy = {'mode': 'from_seed'}
        if not isinstance(redundancy, dict):
            raise _invalid("'redundancy' must be an object")
        mode = redundancy.get('mode', 'from_seed')
        if mode not in ('from_seed', 'fixed'):
            raise _invalid("redundancy.mode must be 'from_seed' or 'fixed'")
        if mode == 'fixed':
            if not _finite(redundancy.get('value')):
                raise _invalid("redundancy.mode 'fixed' needs a finite "
                               'redundancy.value in radians')
            value = float(redundancy['value'])
        else:
            # Finite even here: the service refuses a non-finite value before
            # it looks at the mode, so the seed's own joint 7 is what goes on
            # the wire.
            value = seed[defaults.JOINT_COUNT - 1]
        return SolveRequest(
            arm_id=arm_id, seed=seed, position=position,
            orientation=orientation,
            redundancy_mode=(REDUNDANCY_FIXED if mode == 'fixed'
                             else REDUNDANCY_FROM_SEED),
            redundancy_value=value,
            scene=self._validate_scene(body))

    def _validate_redundancy(self, body):
        """Return (request, clamped sample count) for one sweep."""
        if not isinstance(body, dict):
            raise _invalid('the request body must be a JSON object')
        arm_id = self._validate_arm(body)
        seed = _vector(body.get('seed'), defaults.JOINT_COUNT, "'seed'")
        position, orientation = self._validate_target(body)
        samples = body.get('samples', defaults.GHOST_REDUNDANCY_SAMPLES)
        if samples is None:
            samples = defaults.GHOST_REDUNDANCY_SAMPLES
        if isinstance(samples, bool) or not isinstance(samples, int):
            raise _invalid("'samples' must be a whole number")
        samples = max(defaults.GHOST_REDUNDANCY_SAMPLES_MIN,
                      min(defaults.GHOST_REDUNDANCY_SAMPLES_MAX, samples))
        request = SolveRequest(
            arm_id=arm_id, seed=seed, position=position,
            orientation=orientation, redundancy_mode=REDUNDANCY_FIXED,
            redundancy_value=seed[defaults.JOINT_COUNT - 1], scene={})
        return request, samples


__all__ = [
    'GhostError', 'GhostService', 'IkCall', 'IkReply', 'SolveRequest',
    'TokenBucket', 'joint_names_for', 'link_of', 'offending_links_for',
    'session_view_from', 'verdict_sentence',
]
