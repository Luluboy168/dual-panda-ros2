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
Supervision of the ``franka_record`` child that records every web session.

"Every session auto-recorded" is an invariant, not a best effort (plan section
5.5): if the recorder does not come up, the session does not start. Because a
segment is capped at ``--duration 3600``, a longer session is a CHAIN of
segments -- ``<name>``, ``<name>-002``, ``<name>-003``, ... -- and the sequence
number rides in every state frame so a rollover is visible rather than silent.

Two corrections to the plan text are baked in here. Both come from the citation
audit (session log D19/D20) and both are load-bearing.

NEVER spawn through ``ros2 run`` (D20)
--------------------------------------
``ros2 run franka_bringup franka_record`` yields a WRAPPER pid, and
``franka_bringup.recorder`` deliberately starts its own ``ros2 bag record``
child with ``start_new_session=True``. Signalling the wrapper therefore stops
NOTHING: both processes keep running, the bag is never sealed, and the orphan
cross-captures the next session's traffic. That is the documented failure class
in the recorder's own module docstring ("Stopping a recording started through
``ros2 run``": signal the ``franka_record`` pid, never the wrapper), and it
really happened -- three orphans survived Phase 10. :func:`recorder_binary`
resolves the installed ``<prefix>/lib/franka_bringup/franka_record`` through the
ament index, :func:`build_argv` execs it directly, and the stop ladder signals
THAT pid.

SIGINT first, and thirty seconds of patience (D19)
--------------------------------------------------
``franka_record`` answers SIGINT/SIGTERM by running its own bounded
SIGINT (10 s) -> SIGTERM (5 s) -> SIGKILL (5 s + 5 s) ladder against
``ros2 bag record`` and only then sealing the bag and reporting: up to ~25 s of
legitimate work. Escalating after the server's ordinary 10 s would kill it
mid-seal and leave exactly the metadata-less, ``ros2 bag reindex``-required bag
the recorder exists to prevent. The ladder here is therefore SIGINT
(``config.RECORDER_STOP_SIGINT_WAIT_S`` = 30 s) -> SIGTERM (10 s, still handled,
still seals) -> SIGKILL only as a loud last resort.

The child-process protocol
--------------------------
``spawn(argv, env, name)`` returns a child-process object with:

* ``pid`` -- the pid signalled by the stop ladder (the ``franka_record`` pid);
* ``alive()`` -- ``True`` while the child is running;
* ``returncode()`` -- the exit status, or ``None`` while it runs;
* ``send_signal(number)`` -- deliver one signal to that pid;
* ``wait_exited(timeout)`` -- block up to ``timeout`` seconds, ``True`` iff the
  child has exited;
* ``output_tail()`` -- a short diagnostic tail of the child's own output.

Nothing in this module executes a process, opens a socket or touches ROS, which
is what makes the whole rollover-and-shutdown contract unit-testable against a
scripted fake child.
"""

from datetime import datetime, timezone
import os
import signal
import threading
import time

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from franka_bringup.recorder import _SAFE_NAME, DUAL_ALLOWED_TOPICS, SINGLE_ALLOWED_TOPICS
from franka_web import config
from franka_web.launcher import LauncherError

# The recorder's own name gate, imported rather than copied so the two can
# never drift: a name this module accepts but franka_record refuses would fail
# the session at spawn time instead of at validation time.
_RECORDER_SAFE_NAME = _SAFE_NAME

# franka_web offers exactly the two arm modes the operator launch profiles have.
# Keyed off the recorder's own tuples so the frame reports what is really
# recorded rather than a second, drifting list.
_ARM_MODE_TOPICS = {
    'dual': DUAL_ALLOWED_TOPICS,
    'single': SINGLE_ALLOWED_TOPICS,
}

# `web-YYYYmmdd-HHMMSS` is 19 characters, well inside the recorder's 64-character
# name bound, and leaves room for the `-002`.. segment suffix.
_SESSION_NAME_FORMAT = 'web-%Y%m%d-%H%M%S'
_SEGMENT_SUFFIX_FORMAT = '{}-{:03d}'

# The recorder validates its output root, its name and its topic set before it
# records anything, and exits non-zero within milliseconds when any of that is
# wrong. One second is a generous window in which to notice that and refuse the
# session (plan section 5.5: a session whose recorder failed does not start).
START_GRACE_S = 1.0

# A healthy segment runs its full `--duration` (3600 s). One that ends inside
# ten seconds did not roll over, it failed -- and re-spawning it from the next
# 10 Hz supervisor tick would be a spawn storm. The chain stops instead, loudly.
MINIMUM_SEGMENT_LIFETIME_S = 10.0

# Longest tail of the child's own output carried into an error message. The
# recorder never sees a robot address (none appears in its argv, its
# environment contract or its output), so this cannot leak one; it is bounded
# and whitespace-collapsed anyway so an error string stays one readable line.
_OUTPUT_TAIL_LIMIT = 200

_STOP_LADDER = (
    ('sigint', signal.SIGINT, config.RECORDER_STOP_SIGINT_WAIT_S),
    ('sigterm', signal.SIGTERM, config.RECORDER_STOP_SIGTERM_WAIT_S),
    ('sigkill', signal.SIGKILL, config.RECORDER_STOP_SIGKILL_WAIT_S),
)


class RecordingError(RuntimeError):
    """The session recorder could not be named, started, chained or stopped."""


def recorder_binary():
    """
    Resolve the installed ``franka_record`` executable through the ament index.

    This is the binary itself, never ``ros2 run franka_bringup franka_record``:
    the wrapper's pid cannot stop the recorder (see the module docstring, audit
    D20). Raises :class:`RecordingError` if franka_bringup is not on the ament
    prefix path or its binary is missing or not executable.
    """
    try:
        prefix = get_package_prefix('franka_bringup')
    except (PackageNotFoundError, ValueError) as error:
        raise RecordingError(
            'franka_bringup is not installed; source the workspace before starting a '
            'session') from error
    path = os.path.join(prefix, 'lib', 'franka_bringup', 'franka_record')
    if not os.path.isfile(path) or not os.access(path, os.X_OK):
        raise RecordingError('the installed franka_record binary is missing or not executable')
    return path


def session_name(now=None):
    """
    Return the ``web-YYYYmmdd-HHMMSS`` session name for ``now`` (UTC).

    ``now`` is a ``datetime`` (aware values are converted to UTC, naive ones are
    taken as UTC already) and defaults to the current UTC time. The result is
    validated against the recorder's own name gate before it is returned, so a
    name this function produces can never be refused at spawn time.
    """
    if now is None:
        now = datetime.now(timezone.utc)
    if now.tzinfo is not None:
        now = now.astimezone(timezone.utc)
    name = now.strftime(_SESSION_NAME_FORMAT)
    _validate_name(name)
    return name


def segment_name(base, sequence):
    """
    Return the name of segment ``sequence`` of the session named ``base``.

    Segment 1 records under ``base`` itself, so a session that never rolls over
    is named exactly as the plan says; segments 2 and up append ``-002``,
    ``-003``, ... Raises :class:`RecordingError` if ``base`` is not a valid
    recorder name, if ``sequence`` is not an integer of at least 1, or if the
    suffixed name would exceed the recorder's 64-character bound.
    """
    _validate_name(base)
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 1:
        raise RecordingError('recording segment sequence must be an integer of at least 1')
    if sequence == 1:
        return base
    name = _SEGMENT_SUFFIX_FORMAT.format(base, sequence)
    if not _RECORDER_SAFE_NAME.fullmatch(name):
        raise RecordingError(
            'recording base name is too long to carry a segment suffix; '
            'it must leave room within 64 characters')
    return name


def topics_for(arm_mode):
    """
    Return the exact topic tuple ``franka_record`` records in ``arm_mode``.

    The tuples come from :mod:`franka_bringup.recorder`, so the state frame
    reports what is actually being recorded. Raises :class:`RecordingError` for
    anything other than ``dual`` or ``single``.
    """
    return _ARM_MODE_TOPICS[_validated_arm_mode(arm_mode)]


def build_argv(settings, name, arm_mode):
    """
    Build the exact argv that starts one recording segment.

    The first element is the installed binary from :func:`recorder_binary` --
    the whole point of audit D20 -- and the duration is the recorder's hard
    3600 s cap, which is what makes a long session a chain of segments. Raises
    :class:`RecordingError` on an invalid name or arm mode.
    """
    mode = _validated_arm_mode(arm_mode)
    _validate_name(name)
    return (
        recorder_binary(),
        '--output-root', settings.recording_root,
        '--name', name,
        '--duration', str(config.RECORDING_SEGMENT_DURATION_S),
        '--arm-mode', mode,
    )


class RecordingSupervisor:
    """
    Own one session's ``franka_record`` child, across segment rollovers.

    The supervisor never blocks on the recorder except inside :meth:`start` (the
    one-second refusal window) and :meth:`stop` (the bounded signal ladder);
    :meth:`tick` is a non-blocking poll meant to be called from the server's
    10 Hz supervisor loop.

    ``spawn`` is a callable ``(argv, env, name)`` returning a child-process
    object with the protocol in the module docstring, and ``monotonic`` is the
    clock used to bound segment lifetimes -- both injected so this class is
    testable without executing anything.
    """

    def __init__(self, settings, spawn, monotonic=time.monotonic):
        """Bind the supervisor to ``settings``, a ``spawn`` callable and a clock."""
        self._settings = settings
        self._spawn = spawn
        self._monotonic = monotonic
        # Guards the (name, sequence, child, arm_mode, stopped) tuple that
        # frame() snapshots from the pump/HTTP threads while the supervisor
        # thread rolls segments over.
        self._frame_lock = threading.Lock()
        self._child = None
        self._base_name = None
        self._name = None
        self._arm_mode = None
        self._sequence = 0
        self._restarts = 0
        self._segment_started = None
        self._stopped = False
        self._stop_step = None

    @property
    def active(self):
        """Return ``True`` while a segment is spawned and the chain is not stopped."""
        return self._child is not None and not self._stopped

    @property
    def restarts(self):
        """Return how many times a finished segment has been chained to the next one."""
        return self._restarts

    def start(self, base_name, arm_mode, env):
        """
        Spawn segment 1 of the session named ``base_name`` and prove it came up.

        The child is given :data:`START_GRACE_S` to fail: the recorder validates
        its output root, name and topics before recording anything, so a child
        that is already gone means the recording never started. That raises
        :class:`RecordingError` with the supervisor reset to its never-started
        shape -- and per plan section 5.5 the SESSION must then not start
        either. Raises :class:`RecordingError` if a recording is already active.
        """
        if self.active:
            raise RecordingError('a recording is already running')
        mode = _validated_arm_mode(arm_mode)
        _validate_name(base_name)
        # Prove up front that this base can carry a rollover suffix, rather than
        # discovering it an hour later when segment 2 is due.
        segment_name(base_name, 2)
        self._reset()
        self._base_name = base_name
        self._arm_mode = mode
        try:
            self._spawn_segment(1, env)
        except RecordingError:
            self._reset()
            raise
        child = self._child
        if child.wait_exited(START_GRACE_S) or not child.alive():
            detail = _exit_detail(child)
            self._reset()
            raise RecordingError('the session recorder exited immediately ({})'.format(detail))

    def tick(self, env):
        """
        Chain the next segment if the active one finished on its own.

        A segment that reaches its ``--duration`` exits 0 with the bag sealed;
        the supervisor immediately starts ``<base>-002``, ``-003``, ... so the
        session stays recorded. Does nothing once :meth:`stop` has run. A
        segment that ends inside :data:`MINIMUM_SEGMENT_LIFETIME_S` did not roll
        over, it failed: the chain stops and :class:`RecordingError` is raised
        rather than re-spawning a doomed child on every tick.
        """
        if self._stopped or self._child is None:
            return
        child = self._child
        if child.alive():
            return
        lifetime = self._monotonic() - self._segment_started
        if lifetime < MINIMUM_SEGMENT_LIFETIME_S:
            detail = _exit_detail(child)
            self._abandon()
            raise RecordingError(
                'the session recorder ended after {:.1f} s instead of recording its '
                'segment ({})'.format(lifetime, detail))
        try:
            self._spawn_segment(self._sequence + 1, env)
        except RecordingError:
            self._abandon()
            raise
        self._restarts += 1

    def stop(self):
        """
        Stop the active segment with the bounded SIGINT -> SIGTERM -> SIGKILL ladder.

        SIGINT comes first and gets ``config.RECORDER_STOP_SIGINT_WAIT_S``
        (30 s), because ``franka_record`` runs its own ~25 s ladder against
        ``ros2 bag record`` before it seals the bag (audit D19). SIGTERM is
        still handled and still seals; SIGKILL does not, and leaves a bag that
        needs ``ros2 bag reindex``, so it is the last resort only.

        Returns which step ended the child -- ``'exited'`` (it had already
        finished), ``'sigint'``, ``'sigterm'`` or ``'sigkill'`` -- or ``None``
        if there was never anything to stop. Idempotent: a second call re-reports
        the same step without signalling anything. After a stop, :meth:`tick`
        never starts another segment. Raises :class:`RecordingError` if even
        SIGKILL did not reap the child inside its budget.
        """
        self._stopped = True
        child = self._child
        if child is None:
            return self._stop_step
        self._child = None
        if not child.alive():
            return self._record_stop('exited')
        first_error = None
        for step, number, budget in _STOP_LADDER:
            try:
                child.send_signal(number)
            except ProcessLookupError:
                return self._record_stop(step)
            except OSError as error:
                # Keep escalating: a failure to deliver one signal is far less
                # bad than leaving a recorder running and cross-capturing.
                if first_error is None:
                    first_error = error
            if child.wait_exited(budget) or not child.alive():
                return self._record_stop(step)
        failure = RecordingError(
            'the session recorder (pid {}) did not exit after the bounded SIGINT, SIGTERM and '
            'SIGKILL ladder'.format(getattr(child, 'pid', 'unknown')))
        if first_error is not None:
            raise failure from first_error
        raise failure

    def frame(self, topics):
        """
        Return the ``recording`` block of the plan section 6.11 state frame.

        ``name`` and ``path`` describe the CURRENT segment, so the path's last
        component always equals the name and the operator can find the bag that
        is growing right now; ``sequence`` says which segment of the chain it is.
        ``topics`` is supplied by the caller (normally :func:`topics_for`) and is
        empty until a recording has been started.
        """
        with self._frame_lock:
            name = self._name
            sequence = self._sequence
            arm_mode = self._arm_mode
            active = self._child is not None and not self._stopped
        if name is None:
            return {
                'active': False,
                'name': None,
                'sequence': 0,
                'path': None,
                'arm_mode': None,
                'topics': [],
            }
        return {
            'active': active,
            'name': name,
            'sequence': sequence,
            'path': os.path.join(self._settings.recording_root, name),
            'arm_mode': arm_mode,
            'topics': list(topics),
        }

    def _spawn_segment(self, sequence, env):
        """Spawn segment ``sequence`` and adopt it as the active child."""
        name = segment_name(self._base_name, sequence)
        argv = build_argv(self._settings, name, self._arm_mode)
        try:
            child = self._spawn(argv, env, name)
        except (OSError, LauncherError) as error:
            # LauncherError is what ChildProcess.spawn raises for a failed
            # Popen; without catching it here a recorder spawn failure would
            # escape as a crash instead of a recording_failed refusal.
            raise RecordingError('unable to start the session recorder') from error
        if child is None:
            raise RecordingError('the session recorder spawn returned no child process')
        with self._frame_lock:
            self._child = child
            self._name = name
            self._sequence = sequence
        self._segment_started = self._monotonic()

    def _record_stop(self, step):
        """Remember and return the ladder step that ended the child."""
        self._stop_step = step
        return step

    def _abandon(self):
        """Give up the chain after a failed rollover, without restarting it."""
        with self._frame_lock:
            self._child = None
            self._stopped = True

    def _reset(self):
        """Return every field to the never-started shape."""
        with self._frame_lock:
            self._child = None
            self._base_name = None
            self._name = None
            self._arm_mode = None
            self._sequence = 0
            self._restarts = 0
            self._segment_started = None
            self._stopped = False
        self._stop_step = None


def _validate_name(name):
    """Raise :class:`RecordingError` unless ``name`` passes the recorder's gate."""
    if not isinstance(name, str) or not _RECORDER_SAFE_NAME.fullmatch(name):
        raise RecordingError('recording name must be one safe 1..64 character path component')


def _validated_arm_mode(arm_mode):
    """Return ``arm_mode`` if franka_web records it, else raise :class:`RecordingError`."""
    if arm_mode not in _ARM_MODE_TOPICS:
        raise RecordingError('recording arm mode must be one of: {}'.format(
            ', '.join(sorted(_ARM_MODE_TOPICS))))
    return arm_mode


def _exit_detail(child):
    """
    Describe how a child ended, for one line of an operator-facing error.

    The tail is bounded and whitespace-collapsed. It cannot carry a robot
    address: no address reaches the recorder's argv, environment or output.
    """
    try:
        code = child.returncode()
    except Exception:  # noqa: BLE001 - a diagnostic string must never mask the real failure
        code = None
    try:
        tail = child.output_tail() or ''
    except Exception:  # noqa: BLE001 - same: diagnostics are best effort
        tail = ''
    tail = ' '.join(str(tail).split())[:_OUTPUT_TAIL_LIMIT]
    if not tail:
        return 'exit status {}'.format(code)
    return 'exit status {}: {}'.format(code, tail)
