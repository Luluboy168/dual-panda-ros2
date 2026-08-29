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

The server owns exactly two kinds of child -- the ``ros2 launch`` that brings
the robot stack up and the ``franka_record`` that bags it -- and both are
started from the main thread only, because ``PR_SET_PDEATHSIG`` fires when the
*thread* that forked dies. Spawning from the main thread is what makes "the
forking thread died" mean "the server process died".

Four rules here are load-bearing and must survive any edit:

* The preexec performs exactly two syscalls, ``setpgid(0, 0)`` and
  ``prctl(PR_SET_PDEATHSIG, <per-child signal>)``, with no allocation, locking
  or logging inside the forked child (the closure is built before the fork).
  ``preexec_fn`` is documented-unsafe in a threaded parent, and this server is
  threaded (HTTP connections, a ROS executor). It is acceptable only because
  the function stays this small; keep it that way.
* Liveness is ``os.path.exists('/proc/<pid>')`` plus ``Popen.poll()``, never
  ``pgrep -x``: ``/proc/<pid>/comm`` is truncated to 15 characters, so a
  ``pgrep -x ros2_control_node`` matches nothing at all and reports a running
  stack as gone.
* The stop ladder sends ``SIGINT`` to the child PID alone -- ``ros2 launch``
  handles ``SIGINT`` itself and tears its whole tree down in order -- and only
  escalates to the process GROUP with ``SIGTERM`` and then ``SIGKILL``. This is
  the shape of ``franka_bringup.recorder._bounded_stop_and_reap``, reviewed
  there for the same reason.
* Every exit is reaped. A leaked zombie is a leaked robot stack in disguise.

No message raised from this module contains any part of ``argv``: a launch argv
carries ``robot_ip:=<address>``, and an address must never reach a log line, an
API response or a tracked file. The child's own stdout (see
:meth:`ChildProcess.output_tail`) is subject to the same rule at the call site.
"""

import collections
import ctypes
import fcntl
import os
import signal
import subprocess
import threading
import time

# <linux/prctl.h>: PR_SET_PDEATHSIG. The only defence that survives a SIGKILL
# of the server, which by definition no handler of ours can intercept.
_PR_SET_PDEATHSIG = 1

# Never SIGKILL: the server dying is not a reason to corrupt a recording.
# The signal is chosen PER CHILD at spawn (see _make_child_preexec): SIGTERM
# for `franka_record` (clean stop, seals the bag) and SIGINT for `ros2
# launch`, whose SIGTERM handler cancels WITHOUT terminating its subprocess
# tree (upstream launch_service.py TODO) and would orphan the whole stack —
# measured live as finding D-E2E-1.

# Bounded child-output ring. 500 lines is the same order as franka_bringup's
# ring and is enough to show why a launch died; the per-line cap keeps a
# pathological child from turning the ring into a memory leak.
OUTPUT_RING_LINES = 500
OUTPUT_LINE_CHARS = 2000
DEFAULT_TAIL_LINES = 50

# Stop ladders are driven by polling rather than blocking waits so that a stage
# boundary is honoured within one poll interval whatever the child does.
POLL_INTERVAL_S = 0.05

# Bound on joining the stdout reader thread after the child is gone. The thread
# is a daemon and the pipe is at EOF by then; this is hygiene, not correctness.
_READER_JOIN_TIMEOUT_S = 1.0

_STOP_ALREADY_EXITED = 'already-exited'
_STOP_SIGINT = 'sigint'
_STOP_SIGTERM = 'sigterm'
_STOP_SIGKILL = 'sigkill'


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


def _prctl_unavailable(*_args):
    """Stand in for prctl(2) where libc could not be resolved (never on Linux)."""
    return -1


_PRCTL = _load_prctl()
if _PRCTL is None:
    _PRCTL = _prctl_unavailable


def _make_child_preexec(parent_death_signal):
    """
    Build the preexec for one spawn: exactly two syscalls, nothing else.

    The parent-death signal is per-child because the two children want
    different clean-stop signals: upstream ``ros2 launch`` tears its whole
    tree down only on SIGINT — its SIGTERM handler cancels WITHOUT
    terminating subprocesses (launch_service.py ``_on_sigterm``, a known
    upstream TODO), which orphans ros2_control_node and friends — while
    ``franka_record`` treats SIGTERM as a clean stop and seals its bag.
    """
    death_signal = int(parent_death_signal)

    def _child_preexec():
        os.setpgid(0, 0)
        _PRCTL(_PR_SET_PDEATHSIG, death_signal, 0, 0, 0)
    return _child_preexec


class ChildProcess:
    """
    One supervised child: its pipe reader, its liveness, and its stop ladder.

    Construct with :meth:`spawn`; the constructor exists for tests that want to
    wrap a ``Popen`` they made themselves.
    """

    def __init__(self, process, name):
        """Adopt an already-started ``Popen`` and start draining its stdout."""
        self._process = process
        self._name = str(name)
        self._pid = int(process.pid)
        # Cached: the stop ladder checks this path on every 50 ms poll.
        self._proc_path = '/proc/{}'.format(self._pid)
        self._lines = collections.deque(maxlen=OUTPUT_RING_LINES)
        self._lines_lock = threading.Lock()
        self._reader = threading.Thread(
            target=self._drain_output,
            name='child-out-{}'.format(self._name),
            daemon=True,
        )
        self._reader.start()

    @classmethod
    def spawn(cls, argv, env, name, parent_death_signal=signal.SIGTERM):
        """
        Start ``argv`` as a supervised child in its own process group.

        ``env`` is the child's complete environment (see plan section 3.3; the
        server passes its own environment plus an explicit allowlist). ``name``
        is a short role label used in messages and the reader thread's name --
        it must never be derived from ``argv``, which can carry an address.
        ``parent_death_signal`` is delivered by the kernel if this process
        dies; pass ``signal.SIGINT`` for a ``ros2 launch`` child (the only
        signal on which it tears its whole tree down -- see
        ``_make_child_preexec``) and leave the SIGTERM default for
        ``franka_record`` (which seals its bag on SIGTERM).
        """
        try:
            # No shell, ever: argv is a built list (profiles.py), so nothing in
            # it can be reinterpreted as a command.
            process = subprocess.Popen(
                list(argv),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                close_fds=True,
                preexec_fn=_make_child_preexec(parent_death_signal),
                text=True,
                encoding='utf-8',
                errors='replace',
                bufsize=1,
            )
        except OSError as error:
            # str(error) carries argv[0] (a program name) at most, never the
            # full argv, so no address can reach the message this way.
            raise LauncherError('unable to start the {} child process'.format(name)) from error
        return cls(process, name)

    @property
    def pid(self):
        """Return the child's PID (also its process-group id, from setpgid)."""
        return self._pid

    @property
    def name(self):
        """Return the short role label this child was spawned under."""
        return self._name

    def alive(self):
        """
        Report whether the child is still running.

        ``/proc/<pid>`` first, then ``Popen.poll()``: the directory disappears
        only once an exited child has been reaped, and ``poll`` distinguishes a
        running child from an unreaped zombie. ``pgrep`` is never used (the
        module docstring says why).
        """
        return os.path.exists(self._proc_path) and self._process.poll() is None

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
        Send ``signum`` to the child's whole process group.

        The group id equals the child's PID (``setpgid(0, 0)`` in the preexec),
        and the kernel keeps that id reserved while any member of the group is
        alive, so this stays correct after the group leader itself is reaped --
        which is the case that matters, because an orphaned ``ros2_control_node``
        is exactly what the escalation exists to remove.
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
        ``'sigkill'`` -- the stage at which the child actually died. Raises
        :class:`LauncherError` only if it survived ``SIGKILL`` to its group,
        which means an unkillable (D-state) child and a session that must not be
        reported as stopped.
        """
        if self._process.poll() is not None:
            self._finish()
            return _STOP_ALREADY_EXITED

        # Step 1: SIGINT to the PID alone. `ros2 launch` traps it and shuts its
        # tree down in order; signalling the group here would race that.
        self.send_signal(signal.SIGINT)
        if self.wait_exited(sigint_wait_s):
            self._finish()
            return _STOP_SIGINT

        # Step 2 and 3: the group, because by now the tree is what is left.
        self.signal_group(signal.SIGTERM)
        if self.wait_exited(sigterm_wait_s):
            self._finish()
            return _STOP_SIGTERM

        self.signal_group(signal.SIGKILL)
        if self.wait_exited(sigkill_wait_s):
            self._finish()
            return _STOP_SIGKILL

        raise LauncherError(
            'the {} child process did not exit after SIGKILL'.format(self._name))

    def _finish(self):
        """Reap the exited child and let its output reader drain to EOF."""
        try:
            self._process.wait(timeout=0)
        except subprocess.TimeoutExpired:  # pragma: no cover - poll() said it exited
            pass
        self._reader.join(_READER_JOIN_TIMEOUT_S)

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
    the second one must refuse to start rather than negotiate. The lock is held
    by the open file description, which means the kernel releases it when the
    server dies however it dies -- no stale-lock cleanup path to get wrong.
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

    def release(self):
        """Release the lock and close the descriptor; a no-op if not held."""
        fd = self._fd
        if fd is None:
            return
        self._fd = None
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        except OSError:
            pass
        finally:
            # The pidfile itself is deliberately left behind: unlinking it would
            # race a server that has already opened it and is about to lock.
            os.close(fd)
