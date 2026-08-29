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
Tests for franka_web.launcher: stop ladder, liveness, output ring, pidfile.

Every child here is a throwaway Python script written under ``tmp_path`` -- no
ROS, no node, no DDS. The stop-ladder waits are deliberately short (0.3-0.5 s
rather than the production 10/5/5) so the escalation is exercised in about a
second; the ladder's *shape* is what is under test, not its production numbers.

Every child is reaped: the ``spawner`` fixture stops whatever it handed out,
and the two tests that leave a grandchild behind kill it by matching
``/proc/<pid>/cmdline`` (never by PID alone -- a reaped PID can be recycled,
and never with ``pgrep`` -- ``comm`` is truncated to 15 characters).

The one address in this file is an RFC 5737 documentation address; no real
robot address appears in any fixture, per the session rules.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from franka_web import launcher
from franka_web.launcher import ChildProcess, LauncherError, PidfileLock
import pytest

DOC_IP = '203.0.113.7'

# The grandchild of the PDEATHSIG and process-group tests, matched in
# /proc/<pid>/cmdline so cleanup can never kill a recycled PID.
SLEEPER_SOURCE = 'import time; time.sleep(120)'
SLEEPER_MARK = SLEEPER_SOURCE.encode('ascii')

# Exits 0 on SIGINT, like a well-behaved `ros2 launch`.
COOPERATIVE_CHILD = """\
import signal
import sys
import time


def _exit_cleanly(signum, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, _exit_cleanly)
sys.stdout.write('ready\\n')
sys.stdout.flush()
time.sleep(60)
"""

# Ignores SIGINT, dies on the default SIGTERM disposition.
SIGINT_PROOF_CHILD = """\
import signal
import sys
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)
sys.stdout.write('ready\\n')
sys.stdout.flush()
time.sleep(60)
"""

# Ignores both catchable stop signals; only SIGKILL removes it.
UNSTOPPABLE_CHILD = """\
import signal
import sys
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
sys.stdout.write('ready\\n')
sys.stdout.flush()
time.sleep(60)
"""

# Ignores both stop signals AND holds a grandchild in its own process group,
# the shape of a `ros2 launch` tree that has to be killed as a group.
GROUP_LEADER_CHILD = """\
import signal
import subprocess
import sys
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
grandchild = subprocess.Popen(
    [sys.executable, '-c', {sleeper!r}],
    stdin=subprocess.DEVNULL,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
)
sys.stdout.write('grandchild {{}}\\n'.format(grandchild.pid))
sys.stdout.flush()
time.sleep(60)
""".format(sleeper=SLEEPER_SOURCE)

# Writes more lines than the ring holds, then exits on its own.
CHATTY_CHILD = """\
import sys

for index in range(600):
    sys.stdout.write('line-{}\\n'.format(index))
sys.stdout.flush()
"""

# Arms the same two syscalls as the production preexec, spawns a long sleeper
# under them, then dies abruptly: the sleeper must follow it out.
PDEATHSIG_PARENT = """\
import ctypes
import os
import signal
import subprocess
import sys

_prctl = ctypes.CDLL(None, use_errno=True).prctl


def _preexec():
    os.setpgid(0, 0)
    _prctl(1, int(signal.SIGTERM), 0, 0, 0)


sleeper = subprocess.Popen(
    [sys.executable, '-c', {sleeper!r}],
    stdin=subprocess.DEVNULL,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
    close_fds=True,
    preexec_fn=_preexec,
)
sys.stdout.write('sleeper {{}}\\n'.format(sleeper.pid))
sys.stdout.flush()
os._exit(0)
""".format(sleeper=SLEEPER_SOURCE)

PIDFILE_CLAIMANT = """\
import sys

from franka_web.launcher import LauncherError, PidfileLock

lock = PidfileLock(sys.argv[1])
try:
    lock.acquire()
except LauncherError as error:
    sys.stdout.write('conflict:{}\\n'.format(error))
    raise SystemExit(0)
lock.release()
sys.stdout.write('acquired\\n')
"""


def _script(tmp_path, name, source):
    """Write ``source`` under tmp_path and return an argv that runs it."""
    path = tmp_path / name
    path.write_text(source)
    return [sys.executable, str(path)]


def _process_gone(pid):
    """Report whether ``pid`` is gone or a zombie awaiting its reaper."""
    try:
        with open('/proc/{}/stat'.format(pid), 'rb') as handle:
            after_comm = handle.read().rsplit(b')', 1)[1]
    except OSError:
        return True
    return after_comm.split()[0] == b'Z'


def _kill_sleeper(pid):
    """Kill a leaked test sleeper, identified by cmdline so a recycled PID is safe."""
    try:
        with open('/proc/{}/cmdline'.format(pid), 'rb') as handle:
            cmdline = handle.read()
    except OSError:
        return
    if SLEEPER_MARK not in cmdline:
        return
    try:
        os.kill(pid, signal.SIGKILL)
    except OSError:
        pass


def _wait_until(predicate, timeout_s):
    """Poll ``predicate`` on a 20 ms grain; report whether it became true."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def _await_output(child, prefix, timeout_s=10.0):
    """Return the first output line starting with ``prefix``, or fail the test."""
    found = []

    def _seen():
        for line in child.output_tail(launcher.OUTPUT_RING_LINES):
            if line.startswith(prefix):
                found.append(line)
                return True
        return False

    assert _wait_until(_seen, timeout_s), 'child never printed a line starting with ' + prefix
    return found[0]


@pytest.fixture()
def spawner():
    """Yield a spawn helper that stops and reaps every child it handed out."""
    children = []

    def _spawn(argv, name='child', env=None):
        child = ChildProcess.spawn(argv, dict(os.environ) if env is None else env, name)
        children.append(child)
        return child

    yield _spawn

    for child in children:
        try:
            child.stop(0.2, 0.2, 2.0)
        except Exception:  # pragma: no cover - teardown must never mask a failure
            pass


class TestStopLadder:
    """The SIGINT -> SIGTERM -> SIGKILL escalation and what each stage means."""

    def test_cooperative_child_stops_at_sigint(self, spawner, tmp_path):
        """A child that handles SIGINT exits in stage one, well before SIGTERM."""
        child = spawner(_script(tmp_path, 'cooperative.py', COOPERATIVE_CHILD))
        _await_output(child, 'ready')

        started = time.monotonic()
        outcome = child.stop(5.0, 0.5, 0.5)
        elapsed = time.monotonic() - started

        assert outcome == 'sigint'
        assert child.returncode() == 0
        assert elapsed < 2.0

    def test_sigint_proof_child_stops_at_sigterm(self, spawner, tmp_path):
        """A child that ignores SIGINT falls through to the group SIGTERM."""
        child = spawner(_script(tmp_path, 'sigint_proof.py', SIGINT_PROOF_CHILD))
        _await_output(child, 'ready')

        started = time.monotonic()
        outcome = child.stop(0.5, 5.0, 0.5)
        elapsed = time.monotonic() - started

        assert outcome == 'sigterm'
        assert child.returncode() == -signal.SIGTERM
        assert elapsed >= 0.5, 'stage one must be given its full window'
        assert elapsed < 3.0

    def test_unstoppable_child_stops_at_sigkill(self, spawner, tmp_path):
        """A child that ignores both catchable signals reaches the group SIGKILL."""
        child = spawner(_script(tmp_path, 'unstoppable.py', UNSTOPPABLE_CHILD))
        _await_output(child, 'ready')

        started = time.monotonic()
        outcome = child.stop(0.5, 0.5, 5.0)
        elapsed = time.monotonic() - started

        assert outcome == 'sigkill'
        assert child.returncode() == -signal.SIGKILL
        assert elapsed >= 1.0, 'both catchable stages must be given their windows'
        assert elapsed < 4.0

    def test_escalation_kills_the_whole_process_group(self, spawner, tmp_path):
        """SIGKILL goes to the group, so a launch tree's children die with it."""
        child = spawner(_script(tmp_path, 'group_leader.py', GROUP_LEADER_CHILD))
        grandchild_pid = int(_await_output(child, 'grandchild ').split()[1])
        try:
            assert os.getpgid(grandchild_pid) == child.pid

            assert child.stop(0.3, 0.3, 5.0) == 'sigkill'

            assert _wait_until(lambda: _process_gone(grandchild_pid), 5.0), \
                'the grandchild survived a SIGKILL aimed at the process group'
        finally:
            _kill_sleeper(grandchild_pid)

    def test_stop_reports_a_child_that_had_already_exited(self, spawner, tmp_path):
        """A child that exited on its own is reaped, not signalled again."""
        child = spawner(_script(tmp_path, 'chatty_exit.py', CHATTY_CHILD))

        assert child.wait_exited(10.0)
        assert child.stop(0.5, 0.5, 0.5) == 'already-exited'
        assert child.returncode() == 0

    def test_stop_is_idempotent(self, spawner, tmp_path):
        """Stopping twice is safe: the second call reports the already-dead child."""
        child = spawner(_script(tmp_path, 'cooperative_twice.py', COOPERATIVE_CHILD))
        _await_output(child, 'ready')

        assert child.stop(5.0, 0.5, 0.5) == 'sigint'
        assert child.stop(5.0, 0.5, 0.5) == 'already-exited'


class TestLiveness:
    """Liveness comes from /proc plus poll(), and children own their group."""

    def test_alive_tracks_proc_before_and_after_exit(self, spawner, tmp_path):
        """alive() is true while /proc/<pid> exists, false once the child is reaped."""
        child = spawner(_script(tmp_path, 'liveness.py', COOPERATIVE_CHILD))
        _await_output(child, 'ready')

        assert child.alive()
        assert os.path.exists('/proc/{}'.format(child.pid))
        assert child.returncode() is None

        assert child.stop(5.0, 0.5, 0.5) == 'sigint'

        assert not child.alive()
        assert not os.path.exists('/proc/{}'.format(child.pid)), 'the child was not reaped'
        assert child.returncode() == 0

    def test_child_leads_its_own_process_group(self, spawner, tmp_path):
        """The preexec setpgid(0, 0) makes the child its own group leader."""
        child = spawner(_script(tmp_path, 'own_group.py', COOPERATIVE_CHILD))
        _await_output(child, 'ready')

        assert os.getpgid(child.pid) == child.pid
        assert os.getpgid(child.pid) != os.getpgrp()

    def test_wait_exited_reports_a_running_child(self, spawner, tmp_path):
        """wait_exited returns False (not an exception) when the window expires."""
        child = spawner(_script(tmp_path, 'still_running.py', COOPERATIVE_CHILD))
        _await_output(child, 'ready')

        assert child.wait_exited(0.2) is False
        assert child.alive()


class TestOutputRing:
    """The reader thread captures child output into a bounded ring."""

    def test_output_tail_is_captured_and_bounded(self, spawner, tmp_path):
        """The tail holds the last lines only, capped at the ring size."""
        child = spawner(_script(tmp_path, 'chatty.py', CHATTY_CHILD))
        assert child.wait_exited(10.0)
        assert child.stop(0.5, 0.5, 0.5) == 'already-exited'
        assert _wait_until(
            lambda: len(child.output_tail(10 * launcher.OUTPUT_RING_LINES))
            == launcher.OUTPUT_RING_LINES,
            5.0,
        ), 'the reader thread did not drain the pipe to EOF'

        default_tail = child.output_tail()
        assert len(default_tail) == launcher.DEFAULT_TAIL_LINES
        assert default_tail[-1] == 'line-599'
        assert default_tail[0] == 'line-550'

        whole_ring = child.output_tail(10 * launcher.OUTPUT_RING_LINES)
        assert len(whole_ring) == launcher.OUTPUT_RING_LINES
        assert whole_ring[0] == 'line-100', 'the ring must drop the oldest lines'
        assert whole_ring[-1] == 'line-599'

        assert child.output_tail(3) == ['line-597', 'line-598', 'line-599']
        assert child.output_tail(0) == []


class TestSpawnFailure:
    """A child that cannot start reports it without leaking argv."""

    def test_missing_program_raises_launcher_error_without_the_address(self):
        """The refusal names the role, never the launch arguments."""
        with pytest.raises(LauncherError) as excinfo:
            ChildProcess.spawn(
                ['/nonexistent/franka_web_test_binary', 'robot_ip:={}'.format(DOC_IP)],
                dict(os.environ),
                'launch',
            )
        message = str(excinfo.value)
        assert 'launch' in message
        assert DOC_IP not in message


class TestPidfileLock:
    """The single-server guard, verified across processes as flock demands."""

    def _claim_in_subprocess(self, tmp_path, pidfile):
        """Run a second, independent PidfileLock claimant and return its stdout."""
        script = tmp_path / 'claimant.py'
        script.write_text(PIDFILE_CLAIMANT)
        package_root = str(Path(launcher.__file__).resolve().parents[1])
        env = dict(os.environ)
        existing = env.get('PYTHONPATH', '')
        env['PYTHONPATH'] = package_root + os.pathsep + existing if existing else package_root
        completed = subprocess.run(
            [sys.executable, str(script), str(pidfile)],
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
            env=env,
        )
        assert completed.returncode == 0, completed.stderr
        return completed.stdout.strip()

    def test_second_server_is_refused_and_the_first_pid_survives(self, tmp_path):
        """A second claimant is refused by name, and never touches the held file."""
        pidfile = tmp_path / 'franka_web.pid'
        lock = PidfileLock(str(pidfile))
        lock.acquire()
        try:
            assert pidfile.read_text().strip() == str(os.getpid())

            result = self._claim_in_subprocess(tmp_path, pidfile)

            assert result == 'conflict:another franka_web server is running'
            assert pidfile.read_text().strip() == str(os.getpid()), \
                'a refused claimant must not truncate the running server pidfile'
        finally:
            lock.release()

    def test_release_frees_the_lock_for_the_next_server(self, tmp_path):
        """After release, a fresh process can take the lock."""
        pidfile = tmp_path / 'franka_web.pid'
        lock = PidfileLock(str(pidfile))
        lock.acquire()
        lock.release()

        assert self._claim_in_subprocess(tmp_path, pidfile) == 'acquired'

    def test_release_is_a_no_op_when_unheld(self, tmp_path):
        """Release before acquire (and twice after) must not raise."""
        lock = PidfileLock(str(tmp_path / 'franka_web.pid'))
        lock.release()
        lock.acquire()
        lock.release()
        lock.release()

    def test_double_acquire_by_one_owner_is_refused(self, tmp_path):
        """Re-acquiring a held lock is a programming error, not a silent no-op."""
        lock = PidfileLock(str(tmp_path / 'franka_web.pid'))
        lock.acquire()
        try:
            with pytest.raises(LauncherError):
                lock.acquire()
        finally:
            lock.release()


class TestParentDeathSignal:
    """PR_SET_PDEATHSIG: a child cannot outlive the process that forked it."""

    def test_child_dies_when_its_spawning_process_dies(self, spawner, tmp_path):
        """A sleeper spawned under the two-syscall preexec dies with its parent."""
        child = spawner(_script(tmp_path, 'pdeathsig_parent.py', PDEATHSIG_PARENT), 'pdeathsig')
        sleeper_pid = int(_await_output(child, 'sleeper ').split()[1])
        try:
            assert child.wait_exited(10.0), 'the intermediate parent should have exited at once'
            assert child.stop(0.5, 0.5, 0.5) == 'already-exited'

            assert _wait_until(lambda: _process_gone(sleeper_pid), 5.0), \
                'the sleeper outlived the process that forked it'
        finally:
            _kill_sleeper(sleeper_pid)
