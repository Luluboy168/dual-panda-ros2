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
Child-process supervision for the franka_web server (plan sections 3.1-3.2).

The server owns exactly two kinds of child -- the guarded ``ros2 launch`` tree
that brings the robot stack up and the ``franka_record`` that bags it -- and
both are started from the main thread only, because ``PR_SET_PDEATHSIG`` fires
when the *thread* that forked dies. Spawning from the main thread is what makes
"the forking thread died" mean "the server process died".

Four rules here are load-bearing and must survive any edit:

* The preexec does only ``setpgid``, libc ``signal(SIG_DFL)``, ``prctl`` and the
  mandatory post-arm ``getppid`` race check. It allocates or raises only on a
  fatal setup failure. Resetting the inherited handler before arming closes the
  last pre-exec race: a parent-death signal must terminate the not-yet-guardian,
  not run a copied server handler that can merely set a copied Event.
  ``preexec_fn`` is documented-unsafe in a threaded parent, and this server is
  threaded (HTTP connections, a ROS executor). It is acceptable only because
  the normal path stays this small; keep it that way.
* Liveness is ``os.path.exists('/proc/<pid>')`` plus ``Popen.poll()``, never
  ``pgrep -x``: ``/proc/<pid>/comm`` is truncated to 15 characters, so a
  ``pgrep -x ros2_control_node`` matches nothing at all and reports a running
  stack as gone.
* A small non-ROS guardian owns the launch process group. It broadcasts SIGINT
  to the whole group (the terminal-equivalent signal shape), then bounded
  SIGTERM and SIGKILL to that same group, and does not exit until the group is
  gone. A lone parent-death SIGINT to ``ros2 launch`` is insufficient: measured
  end to end, launch can stay alive indefinitely with all of its nodes.
* The guardian inherits the server's existing pidfile-lock description, but its
  ROS target does not. Abrupt server death therefore cannot admit a replacement
  server until the guardian has proved the old target group gone.
* Every exit is reaped. A leaked zombie is a leaked robot stack in disguise.

No message raised from this module contains any part of ``argv``: a launch argv
carries ``robot_ip:=<address>``, and an address must never reach a log line, an
API response or a tracked file. The child's own stdout (see
:meth:`ChildProcess.output_tail`) is subject to the same rule at the call site.
"""

import collections
import ctypes
import fcntl
import json
import os
import select
import signal
import subprocess
import sys
import threading
import time

# <linux/prctl.h>: PR_SET_PDEATHSIG. The only defence that survives a SIGKILL
# of the server, which by definition no handler of ours can intercept.
_PR_SET_PDEATHSIG = 1
_PR_SET_CHILD_SUBREAPER = 36

# Never SIGKILL the recorder: the server dying is not a reason to corrupt a
# recording. ``franka_record`` gets SIGTERM and seals its bag. The launch
# guardian also gets SIGTERM, but translates either SIGINT or SIGTERM into its
# own bounded group ladder; its target gets terminal-shaped group SIGINT first.

# Bounded child-output ring. 500 lines is the same order as franka_bringup's
# ring and is enough to show why a launch died; the per-line cap keeps a
# pathological child from turning the ring into a memory leak.
OUTPUT_RING_LINES = 500
OUTPUT_LINE_CHARS = 2000
DEFAULT_TAIL_LINES = 50

# Stop ladders are driven by polling rather than blocking waits so that a stage
# boundary is honoured within one poll interval whatever the child does.
POLL_INTERVAL_S = 0.05

# The launch guardian's internal ladder mirrors the server's production
# 10/5/5 second launch ladder. The outer owner waits for the whole nested ladder
# plus a small scheduling/reap margin before taking over cleanup itself.
_GUARDIAN_SIGINT_WAIT_S = 10.0
_GUARDIAN_SIGTERM_WAIT_S = 5.0
_GUARDIAN_SIGKILL_WAIT_S = 5.0
_GUARDIAN_OUTER_MARGIN_S = 2.0
_GUARDIAN_START_TIMEOUT_S = 10.0
_GUARDIAN_MODE = '--guard-launch'
_GUARDIAN_READY = b'READY '
_GUARDIAN_ERROR = b'ERROR\n'

# If post-Popen adoption fails, there is no caller-visible ChildProcess that
# could retry later. Give recorder-shaped children their full sealing window,
# then retain this stack frame and repeat bounded ladders until the exact owned
# group is proven gone. An unkillable kernel task intentionally blocks spawn
# (and keeps the inherited pidfile exclusion) rather than becoming untracked.
_ADOPTION_SIGINT_WAIT_S = 30.0
_ADOPTION_SIGTERM_WAIT_S = 10.0
_ADOPTION_SIGKILL_WAIT_S = 5.0

# Bound on joining the stdout reader thread after the child is gone. The thread
# is a daemon and the pipe is at EOF by then; this is hygiene, not correctness.
_READER_JOIN_TIMEOUT_S = 1.0

_STOP_ALREADY_EXITED = 'already-exited'
_STOP_SIGINT = 'sigint'
_STOP_SIGTERM = 'sigterm'
_STOP_SIGKILL = 'sigkill'

# ``flock`` belongs to the open file description. The server registers its
# held descriptor here; a launch guardian inherits a duplicate, while the ROS
# target does not. Thus a SIGKILLed server cannot release single-owner
# exclusion until its old target group is actually gone.
_PIDFILE_GUARD_DESCRIPTOR = None

# Defined after the stop labels so the exit protocol stays readable in tests.
_GUARDIAN_EXIT_BY_OUTCOME = {
    _STOP_SIGINT: 0,
    _STOP_SIGTERM: 10,
    _STOP_SIGKILL: 11,
}
_GUARDIAN_NATURAL_EXIT = 20
_GUARDIAN_OUTCOME_BY_EXIT = {
    value: key for key, value in _GUARDIAN_EXIT_BY_OUTCOME.items()
}
# Natural exit is also a guardian proof: guardian_main does not return it
# until it has driven/reaped the complete target group.
_GUARDIAN_OUTCOME_BY_EXIT[_GUARDIAN_NATURAL_EXIT] = _STOP_ALREADY_EXITED


class LauncherError(RuntimeError):
    """A child process could not be started, stopped, or locked against."""


def _load_prctl():
    """
    Resolve prctl(2) in THIS process, before any fork.

    Doing the dynamic-library work here rather than in the forked child keeps
    the post-fork, pre-exec code path down to two already-resolved calls.
    """
    try:
        prctl = ctypes.CDLL(None, use_errno=True).prctl
    except (AttributeError, OSError):
        return None
    prctl.restype = ctypes.c_int
    return prctl


def _load_libc_signal():
    """Resolve libc signal(2) before fork so preexec can restore SIG_DFL."""
    try:
        libc_signal = ctypes.CDLL(None, use_errno=True).signal
    except (AttributeError, OSError):
        return None
    libc_signal.argtypes = (ctypes.c_int, ctypes.c_void_p)
    libc_signal.restype = ctypes.c_void_p
    return libc_signal


def _prctl_unavailable(*_args):
    """Stand in for prctl(2) where libc could not be resolved (never on Linux)."""
    return -1


_PRCTL = _load_prctl()
if _PRCTL is None:
    _PRCTL = _prctl_unavailable
_LIBC_SIGNAL = _load_libc_signal()
_SIG_DFL = ctypes.c_void_p(0)
_SIG_ERR = ctypes.c_void_p(-1).value


def _make_child_preexec(parent_death_signal, parent_pid=None,
                        establish_process_group=True, prctl=None,
                        libc_signal=None):
    """
    Build the minimal preexec for one spawn, including the parent-race check.

    ``prctl`` reports failure through its return value, so silently ignoring it
    would turn a claimed safety invariant into a best effort. The ``getppid``
    check closes the kernel-documented race where the parent dies between fork
    and arming PDEATHSIG. Raising is restricted to these fatal failure paths;
    ``Popen`` transports the generic preexec failure back to the parent.

    ``establish_process_group`` is false only for the guardian's target: that
    target is already placed in a fresh session/group by ``start_new_session``.
    """
    death_signal = int(parent_death_signal)
    expected_parent = os.getpid() if parent_pid is None else int(parent_pid)
    resolved_prctl = _PRCTL if prctl is None else prctl
    resolved_signal = _LIBC_SIGNAL if libc_signal is None else libc_signal

    def _child_preexec():
        if establish_process_group:
            os.setpgid(0, 0)
        # SIGKILL and SIGSTOP cannot be caught or ignored, and signal(2)
        # rejects attempts to change their dispositions.  They are therefore
        # already safe from a copied Python handler; reset only catchable
        # parent-death signals before arming prctl.
        if death_signal not in (int(signal.SIGKILL), int(signal.SIGSTOP)):
            if (resolved_signal is None
                    or resolved_signal(death_signal, _SIG_DFL) == _SIG_ERR):
                raise OSError(
                    'unable to reset parent-death signal disposition')
        if resolved_prctl(_PR_SET_PDEATHSIG, death_signal, 0, 0, 0) != 0:
            raise OSError('unable to arm parent-death signal')
        if os.getppid() != expected_parent:
            raise OSError('parent exited before parent-death signal was armed')
    return _child_preexec


def _process_group_exists(process_group_id):
    """Observe a numeric process group; never use this as ownership proof."""
    try:
        os.killpg(int(process_group_id), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _process_starttime(process_id):
    """Return Linux /proc stat field 22, or None when identity is gone."""
    identity = _process_identity(process_id)
    return None if identity is None else identity[3]


def _process_identity(process_id):
    """Return ``(state, ppid, pgrp, starttime)`` from Linux /proc."""
    try:
        with open('/proc/{}/stat'.format(int(process_id)), 'rb') as handle:
            raw = handle.read()
        fields = raw[raw.rindex(b')') + 1:].split()
        return fields[0], int(fields[1]), int(fields[2]), int(fields[19])
    except (OSError, ValueError, IndexError):
        return None


def _same_process_is_live(process_id, expected_starttime):
    """Prove PID+starttime identity is present and not an unreaped zombie."""
    identity = _process_identity(process_id)
    return (identity is not None
            and identity[3] == int(expected_starttime)
            and identity[0] != b'Z')


def _same_process_exists(process_id, expected_starttime):
    """Prove PID+starttime identity is present, including an owned zombie."""
    identity = _process_identity(process_id)
    return identity is not None and identity[3] == int(expected_starttime)


def _direct_guardian_child_in_group(process_group_id):
    """Return True/False for an adopted group member, or None if unprovable."""
    guardian_pid = os.getpid()
    children_path = '/proc/{0}/task/{0}/children'.format(guardian_pid)
    try:
        with open(children_path, 'rb') as handle:
            child_pids = handle.read().split()
    except OSError:
        return None

    for raw_pid in child_pids:
        try:
            child_pid = int(raw_pid)
        except ValueError:
            return None
        identity = _process_identity(child_pid)
        # This guardian is the only process that can reap its direct children.
        # A listed child whose stat cannot be read is therefore an ownership
        # ambiguity, not evidence that the original target group is gone.
        if identity is None:
            return None
        if identity[1] == guardian_pid and identity[2] == int(process_group_id):
            return True
    return False


def _guardian_group_owned(process_group_id, target=None,
                          target_starttime=None):
    """Prove the numeric PGID is still pinned by this guardian's target tree."""
    if (target is not None and target_starttime is not None
            and _same_process_exists(target.pid, target_starttime)):
        return True
    # Once the leader exits, subreaper adoption makes at least one surviving
    # top-level member of its original process group our direct child. A clean
    # negative means the old PGID is no longer ours; never inspect or signal a
    # group merely because that recyclable number now exists again.
    return _direct_guardian_child_in_group(process_group_id)


def _signal_process_group(process_group_id, signum):
    """Signal a group whose immutable owned member was just proved present."""
    try:
        os.killpg(int(process_group_id), signum)
    except ProcessLookupError:
        pass


def _reap_guardian_children(target):
    """Reap exited descendants adopted by the guardian subreaper."""
    while True:
        try:
            pid, status = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return
        if pid == 0:
            return
        if pid == target.pid:
            target.returncode = os.waitstatus_to_exitcode(status)


def _wait_process_group_gone(process_group_id, timeout_s, target=None,
                             target_starttime=None):
    """Wait until no immutable guardian-owned member pins the target PGID."""
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while True:
        if target is not None:
            target.poll()
            _reap_guardian_children(target)
        owned = _guardian_group_owned(
            process_group_id, target=target,
            target_starttime=target_starttime)
        if owned is False:
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            if target is not None:
                target.poll()
                _reap_guardian_children(target)
            return _guardian_group_owned(
                process_group_id, target=target,
                target_starttime=target_starttime) is False
        time.sleep(min(POLL_INTERVAL_S, remaining))


def _stop_process_group(process_group_id, sigint_wait_s, sigterm_wait_s,
                        sigkill_wait_s, target=None,
                        target_starttime=None):
    """Drive one provably guardian-owned group through INT -> TERM -> KILL."""
    stages = (
        (_STOP_SIGINT, signal.SIGINT, sigint_wait_s),
        (_STOP_SIGTERM, signal.SIGTERM, sigterm_wait_s),
        (_STOP_SIGKILL, signal.SIGKILL, sigkill_wait_s),
    )
    last_outcome = _STOP_ALREADY_EXITED
    for outcome, signum, budget in stages:
        if target is not None:
            target.poll()
            _reap_guardian_children(target)
        owned = _guardian_group_owned(
            process_group_id, target=target,
            target_starttime=target_starttime)
        if owned is False:
            return last_outcome
        if owned is None:
            raise LauncherError(
                'unable to prove launch target-group ownership')
        _signal_process_group(process_group_id, signum)
        last_outcome = outcome
        if _wait_process_group_gone(
                process_group_id, budget, target=target,
                target_starttime=target_starttime):
            return outcome
    raise LauncherError('the launch target group did not exit after SIGKILL')


def _stop_process_group_until_gone(process_group_id, target=None,
                                   target_starttime=None):
    """Guardian-only: retain ownership and retry bounded ladders until gone."""
    while True:
        try:
            return _stop_process_group(
                process_group_id,
                _GUARDIAN_SIGINT_WAIT_S,
                _GUARDIAN_SIGTERM_WAIT_S,
                _GUARDIAN_SIGKILL_WAIT_S,
                target=target,
                target_starttime=target_starttime,
            )
        except (LauncherError, OSError):
            # An unkillable D-state is precisely when the guardian and its
            # inherited pidfile lock must NOT exit. Each attempt is bounded;
            # pace the next one instead of spinning.
            time.sleep(POLL_INTERVAL_S)


def _write_guardian_status(descriptor, payload):
    """Write one short startup verdict without ever including target argv."""
    try:
        os.write(descriptor, payload)
    except OSError:
        pass
    finally:
        try:
            os.close(descriptor)
        except OSError:
            pass


def _guardian_main(status_descriptor):
    """Own one launch group and guarantee bounded group teardown."""
    stopped_by = [None]
    target = None
    target_group = None
    status_open = True

    def _request_stop(signum, _frame):
        if stopped_by[0] is None:
            stopped_by[0] = signum

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    if _PRCTL(_PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) != 0:
        _write_guardian_status(status_descriptor, _GUARDIAN_ERROR)
        return 127
    try:
        encoded = sys.stdin.readline()
        try:
            argv = json.loads(encoded)
        except (TypeError, ValueError):
            argv = None
        if (not isinstance(argv, list) or not argv
                or not all(isinstance(value, str) and value for value in argv)
                or stopped_by[0] is not None):
            _write_guardian_status(status_descriptor, _GUARDIAN_ERROR)
            status_open = False
            return 127
        try:
            target = subprocess.Popen(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=None,
                stderr=subprocess.STDOUT,
                env=dict(os.environ),
                close_fds=True,
                start_new_session=True,
                preexec_fn=_make_child_preexec(
                    signal.SIGKILL, parent_pid=os.getpid(),
                    establish_process_group=False),
            )
        except (OSError, subprocess.SubprocessError):
            _write_guardian_status(status_descriptor, _GUARDIAN_ERROR)
            status_open = False
            return 127
        target_group = target.pid
        target_starttime = _process_starttime(target_group)
        if target_starttime is None:
            _write_guardian_status(status_descriptor, _GUARDIAN_ERROR)
            status_open = False
            _stop_process_group_until_gone(
                target_group, target=target,
                target_starttime=target_starttime)
            return 127
        _write_guardian_status(
            status_descriptor,
            _GUARDIAN_READY + '{} {}\n'.format(
                target_group, target_starttime).encode('ascii'))
        status_open = False

        while stopped_by[0] is None:
            target.poll()
            _reap_guardian_children(target)
            if target.returncode is not None:
                break
            time.sleep(POLL_INTERVAL_S)

        stop_outcome = _stop_process_group_until_gone(
            target_group, target=target,
            target_starttime=target_starttime)
        target.poll()
        _reap_guardian_children(target)
        if stopped_by[0] is not None:
            return _GUARDIAN_EXIT_BY_OUTCOME.get(stop_outcome, 0)
        return _GUARDIAN_NATURAL_EXIT
    except (OSError, subprocess.SubprocessError):
        return 127
    finally:
        if status_open:
            _write_guardian_status(status_descriptor, _GUARDIAN_ERROR)
        if target_group is not None:
            _stop_process_group_until_gone(
                target_group, target=target,
                target_starttime=target_starttime)


def _read_guardian_status(descriptor, timeout_s):
    """Read the guardian's address-free READY line within a fixed bound."""
    ready, _, _ = select.select([descriptor], [], [], max(0.0, float(timeout_s)))
    if not ready:
        return b''
    try:
        return os.read(descriptor, 128)
    except OSError:
        return b''


class ChildProcess:
    """
    One supervised child: its pipe reader, its liveness, and its stop ladder.

    Construct with :meth:`spawn`; the constructor exists for tests that want to
    wrap a ``Popen`` they made themselves.
    """

    def __init__(self, process, name, target_process_group=None,
                 target_process_starttime=None):
        """Adopt an already-started ``Popen`` and start draining its stdout."""
        self._process = process
        self._name = str(name)
        self._pid = int(process.pid)
        self._target_process_group = (
            None if target_process_group is None else int(target_process_group))
        self._target_process_starttime = (
            None if target_process_starttime is None
            else int(target_process_starttime))
        # Cached: the stop ladder checks this path on every 50 ms poll.
        self._proc_path = '/proc/{}'.format(self._pid)
        self._lines = collections.deque(maxlen=OUTPUT_RING_LINES)
        self._lines_lock = threading.Lock()
        self._reader = threading.Thread(
            target=self._drain_output,
            name='child-out-{}'.format(self._name),
            daemon=True,
        )
        self._reader_started = False
        try:
            self._reader.start()
            self._reader_started = True
        except BaseException as error:
            # Popen/READY already transferred ownership to this process. Never
            # propagate an adoption failure while that exact group (or its
            # pidfile-lock copy) can still exist: no SessionSupervisor object
            # has been returned that could remember and retry it.
            self._reader_started = self._reader.ident is not None
            self._stop_after_adoption_failure()
            raise LauncherError(
                'unable to supervise the {} child process'.format(
                    self._name)) from error

    @classmethod
    def spawn(cls, argv, env, name, parent_death_signal=signal.SIGTERM):
        """
        Start ``argv`` as a supervised child in its own process group.

        ``env`` is the child's complete environment (see plan section 3.3; the
        server passes its own environment plus an explicit allowlist). ``name``
        is a short role label used in messages and the reader thread's name --
        it must never be derived from ``argv``, which can carry an address.
        ``parent_death_signal`` is delivered by the kernel if this process
        dies. The established launch call shape (role ``launch`` plus SIGINT)
        selects the guardian: the guardian itself gets PDEATHSIG=SIGTERM, then
        broadcasts SIGINT to its separately owned target group. Other roles
        remain direct children; ``franka_record`` keeps its SIGTERM default so
        it seals its bag. A direct child inherits the current pidfile lock's
        open-file description, when present. Its own ``close_fds`` spawn policy
        decides which descendants receive it; the reviewed recorder passes
        only its pinned bag-directory descriptor to ``ros2 bag``, so exclusion
        lasts through recorder teardown without leaking into the bagger.
        """
        if name == 'launch' and int(parent_death_signal) == int(signal.SIGINT):
            return cls._spawn_guarded_launch(argv, env, name)
        try:
            # No shell, ever: argv is a built list (profiles.py), so nothing in
            # it can be reinterpreted as a command.
            pass_descriptors = ()
            if (_PIDFILE_GUARD_DESCRIPTOR is not None
                    and _PIDFILE_GUARD_DESCRIPTOR >= 0):
                pass_descriptors = (_PIDFILE_GUARD_DESCRIPTOR,)
            process = subprocess.Popen(
                list(argv),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                close_fds=True,
                pass_fds=pass_descriptors,
                preexec_fn=_make_child_preexec(
                    parent_death_signal, parent_pid=os.getpid()),
                text=True,
                encoding='utf-8',
                errors='replace',
                bufsize=1,
            )
        except (OSError, subprocess.SubprocessError) as error:
            # str(error) carries argv[0] (a program name) at most, never the
            # full argv, so no address can reach the message this way.
            raise LauncherError('unable to start the {} child process'.format(name)) from error
        return cls(process, name)

    @classmethod
    def _spawn_guarded_launch(cls, argv, env, name):
        """Start the address-blind guardian and pass target argv over a pipe."""
        status_read, status_write = os.pipe2(os.O_CLOEXEC)
        process = None
        pass_descriptors = [status_write]
        if (_PIDFILE_GUARD_DESCRIPTOR is not None
                and _PIDFILE_GUARD_DESCRIPTOR >= 0):
            pass_descriptors.append(_PIDFILE_GUARD_DESCRIPTOR)
        try:
            process = subprocess.Popen(
                [sys.executable, '-m', 'franka_web.launcher',
                 _GUARDIAN_MODE, str(status_write)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                close_fds=True,
                pass_fds=tuple(pass_descriptors),
                preexec_fn=_make_child_preexec(
                    signal.SIGTERM, parent_pid=os.getpid()),
                text=True,
                encoding='utf-8',
                errors='replace',
                bufsize=1,
            )
        except (OSError, subprocess.SubprocessError) as error:
            os.close(status_read)
            os.close(status_write)
            raise LauncherError(
                'unable to start the {} child process'.format(name)) from error
        finally:
            if process is not None:
                os.close(status_write)

        try:
            payload = json.dumps(
                [str(value) for value in argv],
                ensure_ascii=True,
                allow_nan=False,
                separators=(',', ':'),
            )
            process.stdin.write(payload + '\n')
            process.stdin.close()
            status = _read_guardian_status(status_read, _GUARDIAN_START_TIMEOUT_S)
        except (BrokenPipeError, OSError, TypeError, ValueError):
            status = b''
        finally:
            try:
                os.close(status_read)
            except OSError:
                pass

        target_process_group = None
        target_process_starttime = None
        if status.startswith(_GUARDIAN_READY) and status.endswith(b'\n'):
            try:
                identity = status[len(_GUARDIAN_READY):-1].decode('ascii').split()
                if len(identity) == 2:
                    target_process_group = int(identity[0])
                    target_process_starttime = int(identity[1])
            except (UnicodeDecodeError, ValueError):
                target_process_group = None
                target_process_starttime = None
        if (target_process_group is None or target_process_group <= 0
                or target_process_starttime is None
                or target_process_starttime <= 0):
            # The guardian either never spawned a target or owns any target it
            # did spawn. Give its fail-safe handler the complete nested budget
            # before a final kill of the guardian's own (target-free) group.
            try:
                process.send_signal(signal.SIGTERM)
            except ProcessLookupError:
                pass
            while process.poll() is None:
                try:
                    process.wait(timeout=(
                        _GUARDIAN_SIGINT_WAIT_S + _GUARDIAN_SIGTERM_WAIT_S
                        + _GUARDIAN_SIGKILL_WAIT_S + _GUARDIAN_OUTER_MARGIN_S))
                except subprocess.TimeoutExpired:
                    # If a target is unkillable, the address-blind guardian is
                    # the only remaining owner of both it and the pidfile lock.
                    # Never kill that owner merely because READY was lost.
                    continue
            raise LauncherError(
                'unable to start the {} child process'.format(name))
        return cls(
            process, name,
            target_process_group=target_process_group,
            target_process_starttime=target_process_starttime,
        )

    @property
    def pid(self):
        """Return the child's PID (also its process-group id, from setpgid)."""
        return self._pid

    @property
    def name(self):
        """Return the short role label this child was spawned under."""
        return self._name

    @property
    def target_process_group(self):
        """Return the guarded target PGID, or ``None`` for a direct child."""
        return self._target_process_group

    @property
    def target_process_starttime(self):
        """Return the guarded target's immutable /proc identity token."""
        return self._target_process_starttime

    def alive(self):
        """
        Report whether the child is still running.

        ``/proc/<pid>`` first, then ``Popen.poll()``: the directory disappears
        only once an exited child has been reaped, and ``poll`` distinguishes a
        running child from an unreaped zombie. ``pgrep`` is never used (the
        module docstring says why).
        """
        guardian_live = (
            os.path.exists(self._proc_path) and self._process.poll() is None)
        if self._target_process_group is None:
            return guardian_live
        # Readiness/fault logic tracks the actual launch leader, not the
        # guardian that may remain alive for bounded resistant-descendant
        # cleanup. PID+starttime prevents a recycled PID from looking healthy.
        return (guardian_live and _same_process_is_live(
            self._target_process_group, self._target_process_starttime))

    def returncode(self):
        """Return the child's exit status, or ``None`` while it is running."""
        return self._process.poll()

    def output_tail(self, limit=DEFAULT_TAIL_LINES):
        """
        Return up to ``limit`` most recent lines of the child's merged output.

        These are the child's own words. A ``ros2 launch`` echoes its arguments,
        so a line here can contain a robot address: the tail is for the server's
        stderr and for operator diagnosis in the terminal, and must never be
        copied into an API response, an SSE frame or a tracked file.
        """
        if limit <= 0:
            return []
        with self._lines_lock:
            lines = list(self._lines)
        return lines[-limit:]

    def send_signal(self, signum):
        """
        Send ``signum`` to the child PID exactly -- never to its group.

        A no-op once the child has been reaped, so a stale supervisor tick can
        never signal a recycled PID.
        """
        try:
            self._process.send_signal(signum)
        except ProcessLookupError:
            pass

    def signal_group(self, signum):
        """
        Send ``signum`` to a direct child's group while its leader pins the id.

        The direct stop ladder calls this only after ``Popen.poll()`` proved the
        exact leader had not exited. If it exits immediately afterward it stays
        our unreaped child, so the numeric PGID cannot be recycled before this
        signal. This method must never be used after the leader was reaped.
        """
        try:
            os.killpg(self._pid, signum)
        except ProcessLookupError:
            pass

    def wait_exited(self, timeout_s):
        """
        Poll until the child has exited, up to ``timeout_s``; report success.

        Polling (rather than ``Popen.wait(timeout=...)``) keeps every stage of
        the stop ladder interruptible on a fixed 50 ms grain and reaps the child
        as a side effect of ``poll()``.
        """
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while True:
            if self._process.poll() is not None:
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return self._process.poll() is not None
            time.sleep(min(POLL_INTERVAL_S, remaining))

    def stop(self, sigint_wait_s, sigterm_wait_s, sigkill_wait_s):
        """
        Stop the child through the bounded ladder and reap it.

        Returns ``'already-exited'``, ``'sigint'``, ``'sigterm'`` or
        ``'sigkill'`` -- the stage at which the child actually died. Production
        direct children are only ``franka_record``: its reviewed contract does
        not let the wrapper exit until its separately-sessioned bagger has been
        stopped and reaped. Thus the Popen leader is the ownership proof for
        the direct path; once it exits, never probe or signal its recyclable
        numeric PGID. Raises :class:`LauncherError` if the exact leader survives
        SIGKILL, which means an unkillable (D-state) child and a session that
        must not be reported as stopped.
        """
        if self._target_process_group is not None:
            return self._stop_guarded(
                sigint_wait_s, sigterm_wait_s, sigkill_wait_s)
        if self._process.poll() is not None:
            self._finish()
            return _STOP_ALREADY_EXITED

        # SIGINT goes to the reviewed recorder wrapper so it can seal/reap its
        # bagger. Later escalation is group-shaped, but only while the exact
        # Popen leader remains live and therefore pins this PGID against reuse.
        self.send_signal(signal.SIGINT)
        if self.wait_exited(sigint_wait_s):
            self._finish()
            return _STOP_SIGINT

        if self._process.poll() is not None:
            self._finish()
            return _STOP_SIGINT
        self.signal_group(signal.SIGTERM)
        if self.wait_exited(sigterm_wait_s):
            self._finish()
            return _STOP_SIGTERM

        if self._process.poll() is not None:
            self._finish()
            return _STOP_SIGTERM
        self.signal_group(signal.SIGKILL)
        if self.wait_exited(sigkill_wait_s):
            self._finish()
            return _STOP_SIGKILL

        raise LauncherError(
            'the {} child process did not exit after SIGKILL'.format(self._name))

    def _stop_guarded(self, sigint_wait_s, sigterm_wait_s, sigkill_wait_s):
        """Trust guardian proof; never signal the target PGID from the parent."""
        if self._process.poll() is None:
            self.send_signal(signal.SIGINT)
            self.wait_exited(
                _GUARDIAN_SIGINT_WAIT_S + _GUARDIAN_SIGTERM_WAIT_S
                + _GUARDIAN_SIGKILL_WAIT_S + _GUARDIAN_OUTER_MARGIN_S)

        guardian_returncode = self._process.poll()
        proof_outcome = _GUARDIAN_OUTCOME_BY_EXIT.get(guardian_returncode)
        if proof_outcome is not None:
            # These exit codes are the guardian's protocol-level proof that its
            # target group is gone. Never re-probe the now-recyclable numeric
            # PGID: it may already name an unrelated new process group.
            self._finish()
            return proof_outcome

        # A stuck/crashed guardian supplied no group-death proof. It is the
        # sole owner of group cleanup and its target has PDEATHSIG=SIGKILL if
        # the guardian itself crashes. Parent-side fallback would still have a
        # check-to-killpg reuse race, so never probe or signal the numeric target
        # PGID here. Retain this ChildProcess and let a later stop observe a
        # recognized proof; an unknown guardian death intentionally blocks
        # clean server exit rather than risking an unrelated process group.
        raise LauncherError(
            'the launch guardian did not provide target-group teardown proof')

    def _finish(self):
        """Reap the exited child and let its output reader drain to EOF."""
        try:
            self._process.wait(timeout=0)
        except subprocess.TimeoutExpired:  # pragma: no cover - poll() said it exited
            pass
        if self._reader_started or self._reader.ident is not None:
            self._reader.join(_READER_JOIN_TIMEOUT_S)
        elif self._process.stdout is not None:
            # Thread.start() failed before the reader began; close the now-EOF
            # pipe explicitly instead of asking Python to join an unstarted
            # Thread (which itself raises RuntimeError).
            try:
                self._process.stdout.close()
            except OSError:
                pass

    def _stop_after_adoption_failure(self):
        """Fail closed until an unreturnable child is synchronously gone."""
        while True:
            try:
                self.stop(
                    _ADOPTION_SIGINT_WAIT_S,
                    _ADOPTION_SIGTERM_WAIT_S,
                    _ADOPTION_SIGKILL_WAIT_S,
                )
                return
            except BaseException:
                # The only alternative is returning with a live, untracked
                # process. Keep the exact Popen/PGID and inherited flock owner
                # here, pace retries, and let a D-state intentionally block.
                time.sleep(POLL_INTERVAL_S)

    def _drain_output(self):
        """Reader thread: copy the child's merged output into the bounded ring."""
        stream = self._process.stdout
        if stream is None:
            return
        try:
            for line in iter(stream.readline, ''):
                text = line.rstrip('\r\n')[:OUTPUT_LINE_CHARS]
                with self._lines_lock:
                    self._lines.append(text)
        except (OSError, ValueError):
            # The pipe was closed under us (child killed, interpreter shutting
            # down). Nothing to report: the exit status is the real signal.
            pass
        finally:
            try:
                stream.close()
            except OSError:
                pass


class PidfileLock:
    """
    The single-server guard: an exclusive ``flock`` on the state-dir pidfile.

    Two servers sharing one robot stack would fight over the same children, so
    the second one must refuse to start rather than negotiate. A launch guardian
    inherits the same locked open-file description, so abrupt server death does
    NOT release exclusion while its old target group is being removed. The ROS
    target never inherits that descriptor; the kernel releases it when the last
    legitimate owner exits, with no stale-file cleanup protocol.
    """

    def __init__(self, path):
        """Prepare a lock on ``path``; nothing is opened until :meth:`acquire`."""
        self._path = str(path)
        self._fd = None

    @property
    def path(self):
        """Return the pidfile path this lock guards."""
        return self._path

    def acquire(self):
        """
        Take the lock and write this PID, keeping the descriptor open.

        Raises :class:`LauncherError` if another server already holds it.
        """
        if self._fd is not None:
            raise LauncherError('this franka_web pidfile lock is already held')
        # O_NOFOLLOW: the state dir is private, but a pidfile is a classic
        # symlink target and refusing costs nothing.
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW
        try:
            fd = os.open(self._path, flags, 0o600)
        except OSError as error:
            raise LauncherError('unable to open the franka_web pidfile') from error
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            os.close(fd)
            raise LauncherError('another franka_web server is running') from error
        try:
            os.ftruncate(fd, 0)
            os.write(fd, '{}\n'.format(os.getpid()).encode('ascii'))
        except OSError as error:
            os.close(fd)
            raise LauncherError('unable to write the franka_web pidfile') from error
        self._fd = fd
        global _PIDFILE_GUARD_DESCRIPTOR
        _PIDFILE_GUARD_DESCRIPTOR = fd

    def release(self):
        """Release the lock and close the descriptor; a no-op if not held."""
        global _PIDFILE_GUARD_DESCRIPTOR
        fd = self._fd
        if fd is None:
            return
        self._fd = None
        if _PIDFILE_GUARD_DESCRIPTOR == fd:
            _PIDFILE_GUARD_DESCRIPTOR = None
        try:
            # Close, never explicit LOCK_UN: flock belongs to the shared open
            # file description, so an inherited guardian descriptor must keep
            # exclusion even if a non-SIGKILL server unwind reaches release().
            os.close(fd)
        except OSError:
            pass
        # The pidfile itself is deliberately left behind: unlinking it would
        # race a server that has already opened it and is about to lock.


def _module_main(argv):
    """Run only the private launch guardian module mode."""
    if len(argv) != 2 or argv[0] != _GUARDIAN_MODE:
        return 2
    try:
        status_descriptor = int(argv[1])
    except ValueError:
        return 2
    if status_descriptor < 0:
        return 2
    return _guardian_main(status_descriptor)


if __name__ == '__main__':
    raise SystemExit(_module_main(sys.argv[1:]))
