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

# A launch-shaped group whose leader and grandchild both exit cleanly only
# when terminal-shaped SIGINT reaches the WHOLE process group.
GUARDED_GROUP_TARGET = """\
import signal
import subprocess
import sys
import time

grandchild_source = '''\
import signal
import sys
import time


def stop(signum, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
print('ready', flush=True)
time.sleep(120)
'''
grandchild = subprocess.Popen(
    [sys.executable, '-c', grandchild_source],
    stdin=subprocess.DEVNULL,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
)
assert grandchild.stdout.readline().strip() == 'ready'
print('grandchild {}'.format(grandchild.pid), flush=True)


def stop(signum, frame):
    try:
        grandchild.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    sys.exit(0)


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
time.sleep(120)
"""

# The leader exits immediately after creating a cooperative group member. A
# guardian that checked only Popen.poll() would leak that group member.
EXITING_GROUP_LEADER = """\
import signal
import subprocess
import sys

child_source = '''\
import signal
import sys
import time


def stop(signum, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, stop)
print('ready', flush=True)
time.sleep(120)
'''
child = subprocess.Popen(
    [sys.executable, '-c', child_source],
    stdin=subprocess.DEVNULL,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
)
assert child.stdout.readline().strip() == 'ready'
print('grandchild {}'.format(child.pid), flush=True)
"""

# The target leader exits, but its remaining group member ignores SIGINT. The
# guardian must stay for escalation while launch liveness turns false at once.
EXITING_SIGINT_RESISTANT_LEADER = """\
import signal
import subprocess
import sys

child_source = '''\
import signal
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)
print('ready', flush=True)
time.sleep(120)
'''
child = subprocess.Popen(
    [sys.executable, '-c', child_source],
    stdin=subprocess.DEVNULL,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
)
assert child.stdout.readline().strip() == 'ready'
print('grandchild {}'.format(child.pid), flush=True)
"""

# A short-lived server-like owner for the guarded-launch parent-death test.
GUARDIAN_OWNER = """\
import os
import signal
import sys
import time

from franka_web.launcher import ChildProcess

child = ChildProcess.spawn(
    [sys.executable, sys.argv[1]], dict(os.environ), 'launch',
    parent_death_signal=signal.SIGINT)
grandchild = None
deadline = time.monotonic() + 10
while time.monotonic() < deadline and grandchild is None:
    for line in child.output_tail(500):
        if line.startswith('grandchild '):
            grandchild = int(line.split()[1])
            break
    time.sleep(0.02)
assert grandchild is not None
print('guarded {} {} {}'.format(
    child.pid, child.target_process_group, grandchild), flush=True)
time.sleep(120)
"""

# Same shape, but first acquires the real single-server flock. The direct
# guardian inherits that open-file description across owner SIGKILL/exit.
LOCK_HOLDING_OWNER = """\
import os
import signal
import sys
import time

from franka_web.launcher import ChildProcess, PidfileLock

lock = PidfileLock(sys.argv[1])
lock.acquire()
child = ChildProcess.spawn(
    [sys.executable, sys.argv[2]], dict(os.environ), 'launch',
    parent_death_signal=signal.SIGINT)
deadline = time.monotonic() + 10
while time.monotonic() < deadline:
    if any(line.startswith('grandchild ') for line in child.output_tail(500)):
        break
    time.sleep(0.02)
else:
    raise RuntimeError('target group did not become ready')
print('guarded {} {}'.format(child.pid, child.target_process_group), flush=True)
time.sleep(120)
"""

# A recorder-shaped direct child deliberately survives its server parent's
# PDEATHSIG=SIGTERM long enough to prove that it, but not its bagger-shaped
# descendant, retains the shared pidfile open-file description.
DIRECT_RECORDER_GROUP = """\
import signal
import subprocess
import sys
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
bagger_source = {bagger!r}
bagger = subprocess.Popen(
    [sys.executable, '-c', bagger_source],
    stdin=subprocess.DEVNULL,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
    close_fds=True,
)
print('bagger {{}}'.format(bagger.pid), flush=True)
time.sleep(120)
""".format(bagger="""\
import signal
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
time.sleep(120)
""")

DIRECT_RECORDER_LOCK_OWNER = """\
import os
import sys
import time

from franka_web.launcher import ChildProcess, PidfileLock

lock = PidfileLock(sys.argv[1])
lock.acquire()
child = ChildProcess.spawn(
    [sys.executable, sys.argv[2]], dict(os.environ), 'recorder')
deadline = time.monotonic() + 10
bagger = None
while time.monotonic() < deadline and bagger is None:
    for line in child.output_tail(100):
        if line.startswith('bagger '):
            bagger = int(line.split()[1])
            break
    time.sleep(0.02)
if bagger is None:
    raise RuntimeError('recorder descendant did not become ready')
print('direct {} {}'.format(child.pid, bagger), flush=True)
time.sleep(120)
"""

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


class _ScriptedProcess:
    """Minimal Popen double whose poll results expose stop-stage races."""

    def __init__(self, poll_results, pid=424242):
        self.pid = pid
        self._poll_results = list(poll_results)

    def poll(self):
        """Return each scripted result, then repeat the final result."""
        if len(self._poll_results) > 1:
            return self._poll_results.pop(0)
        return self._poll_results[0]


def _scripted_child(poll_results, guarded=False):
    """Build a reader-free ChildProcess double for stop ownership tests."""
    child = object.__new__(ChildProcess)
    child._process = _ScriptedProcess(poll_results)
    child._name = 'scripted'
    child._pid = child._process.pid
    child._target_process_group = 434343 if guarded else None
    child._target_process_starttime = 12345 if guarded else None
    return child


@pytest.fixture()
def spawner():
    """Yield a spawn helper that stops and reaps every child it handed out."""
    children = []

    def _spawn(argv, name='child', env=None, **options):
        child = ChildProcess.spawn(
            argv, dict(os.environ) if env is None else env, name, **options)
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

    def test_recycled_group_is_not_signalled_after_leader_already_exited(
            self, monkeypatch):
        """An exited/reaped direct leader makes its numeric PGID untouchable."""
        child = _scripted_child([0])
        calls = []
        monkeypatch.setattr(child, '_finish', lambda: calls.append('finish'))
        monkeypatch.setattr(
            child, 'send_signal',
            lambda signum: pytest.fail('an exited PID was signalled'))
        monkeypatch.setattr(
            child, 'signal_group',
            lambda signum: pytest.fail('a recycled PGID was signalled'))
        monkeypatch.setattr(
            launcher, '_process_group_exists',
            lambda pgid: pytest.fail('a recycled PGID was probed'))

        assert child.stop(0.0, 0.0, 0.0) == 'already-exited'
        assert calls == ['finish']

    def test_exit_during_sigint_never_reaches_group_escalation(
            self, monkeypatch):
        """A SIGINT-stage leader exit returns without TERM/KILL to its PGID."""
        child = _scripted_child([None])
        calls = []
        monkeypatch.setattr(child, '_finish', lambda: calls.append('finish'))
        monkeypatch.setattr(
            child, 'send_signal', lambda signum: calls.append(('pid', signum)))
        monkeypatch.setattr(child, 'wait_exited', lambda timeout_s: True)
        monkeypatch.setattr(
            child, 'signal_group',
            lambda signum: pytest.fail('group escalation followed leader exit'))

        assert child.stop(1.0, 1.0, 1.0) == 'sigint'
        assert calls == [('pid', signal.SIGINT), 'finish']

    def test_exit_at_sigint_boundary_never_signals_recycled_group(
            self, monkeypatch):
        """The post-wait poll catches exit before any numeric PGID signal."""
        child = _scripted_child([None, 0])
        calls = []
        monkeypatch.setattr(child, '_finish', lambda: calls.append('finish'))
        monkeypatch.setattr(
            child, 'send_signal', lambda signum: calls.append(('pid', signum)))
        monkeypatch.setattr(child, 'wait_exited', lambda timeout_s: False)
        monkeypatch.setattr(
            child, 'signal_group',
            lambda signum: pytest.fail('a recycled PGID was signalled'))

        assert child.stop(1.0, 1.0, 1.0) == 'sigint'
        assert calls == [('pid', signal.SIGINT), 'finish']

    def test_exit_after_sigterm_never_reaches_sigkill(
            self, monkeypatch):
        """A TERM-stage leader exit suppresses the final group KILL."""
        child = _scripted_child([None, None])
        calls = []
        waits = iter([False, True])
        monkeypatch.setattr(child, '_finish', lambda: calls.append('finish'))
        monkeypatch.setattr(
            child, 'send_signal', lambda signum: calls.append(('pid', signum)))
        monkeypatch.setattr(
            child, 'signal_group', lambda signum: calls.append(('group', signum)))
        monkeypatch.setattr(
            child, 'wait_exited', lambda timeout_s: next(waits))

        assert child.stop(1.0, 1.0, 1.0) == 'sigterm'
        assert calls == [
            ('pid', signal.SIGINT),
            ('group', signal.SIGTERM),
            'finish',
        ]

    def test_exit_at_sigterm_boundary_never_signals_recycled_group(
            self, monkeypatch):
        """A final TERM-boundary poll prevents KILL of a reused PGID."""
        child = _scripted_child([None, None, 0])
        calls = []
        waits = iter([False, False])
        monkeypatch.setattr(child, '_finish', lambda: calls.append('finish'))
        monkeypatch.setattr(
            child, 'send_signal', lambda signum: calls.append(('pid', signum)))
        monkeypatch.setattr(
            child, 'signal_group', lambda signum: calls.append(('group', signum)))
        monkeypatch.setattr(
            child, 'wait_exited', lambda timeout_s: next(waits))

        assert child.stop(1.0, 1.0, 1.0) == 'sigterm'
        assert calls == [
            ('pid', signal.SIGINT),
            ('group', signal.SIGTERM),
            'finish',
        ]

    def test_live_leader_reaches_group_sigkill(self, monkeypatch):
        """A leader proven live at every boundary receives the complete ladder."""
        child = _scripted_child([None, None, None])
        calls = []
        waits = iter([False, False, True])
        monkeypatch.setattr(child, '_finish', lambda: calls.append('finish'))
        monkeypatch.setattr(
            child, 'send_signal', lambda signum: calls.append(('pid', signum)))
        monkeypatch.setattr(
            child, 'signal_group', lambda signum: calls.append(('group', signum)))
        monkeypatch.setattr(
            child, 'wait_exited', lambda timeout_s: next(waits))

        assert child.stop(1.0, 1.0, 1.0) == 'sigkill'
        assert calls == [
            ('pid', signal.SIGINT),
            ('group', signal.SIGTERM),
            ('group', signal.SIGKILL),
            'finish',
        ]


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


class TestLaunchGuardian:
    """The address-blind guardian owns and proves the whole launch group."""

    @pytest.mark.parametrize(
        'returncode, expected',
        [(0, 'sigint'), (10, 'sigterm'), (11, 'sigkill'),
         (20, 'already-exited')],
    )
    def test_proof_exit_never_probes_a_recycled_target_group(
            self, monkeypatch, returncode, expected):
        """A recognized guardian rc is complete proof, not a PGID hint."""
        child = _scripted_child([returncode], guarded=True)
        finished = []
        monkeypatch.setattr(child, '_finish', lambda: finished.append(True))
        monkeypatch.setattr(
            launcher, '_process_group_exists',
            lambda pgid: pytest.fail('a proven-gone PGID was probed'))
        monkeypatch.setattr(
            launcher, '_signal_process_group',
            lambda pgid, signum: pytest.fail('a proven-gone PGID was signalled'))
        monkeypatch.setattr(
            launcher, '_stop_process_group',
            lambda *args, **kwargs: pytest.fail('parent target fallback ran'))

        assert child.stop(0.0, 0.0, 0.0) == expected
        assert finished == [True]

    @pytest.mark.parametrize('returncode', [127, -signal.SIGKILL])
    def test_unknown_guardian_exit_fails_closed_without_target_group_access(
            self, monkeypatch, returncode):
        """Unknown guardian death retains ownership and never touches its PGID."""
        child = _scripted_child([returncode], guarded=True)
        monkeypatch.setattr(
            launcher, '_process_group_exists',
            lambda pgid: pytest.fail('an unproved PGID was probed'))
        monkeypatch.setattr(
            launcher, '_signal_process_group',
            lambda pgid, signum: pytest.fail('an unproved PGID was signalled'))
        monkeypatch.setattr(
            launcher, '_stop_process_group',
            lambda *args, **kwargs: pytest.fail('parent target fallback ran'))

        with pytest.raises(LauncherError, match='teardown proof'):
            child.stop(0.0, 0.0, 0.0)

    def test_live_guardian_past_budget_is_retained_without_target_fallback(
            self, monkeypatch):
        """A still-running guardian stays sole cleanup and flock owner."""
        child = _scripted_child([None, None], guarded=True)
        guardian_signals = []
        monkeypatch.setattr(
            child, 'send_signal',
            lambda signum: guardian_signals.append(signum))
        monkeypatch.setattr(child, 'wait_exited', lambda timeout_s: False)
        monkeypatch.setattr(
            launcher, '_process_group_exists',
            lambda pgid: pytest.fail('a live guardian target PGID was probed'))
        monkeypatch.setattr(
            launcher, '_signal_process_group',
            lambda pgid, signum: pytest.fail('a live guardian target was signalled'))

        with pytest.raises(LauncherError, match='teardown proof'):
            child.stop(0.0, 0.0, 0.0)
        assert guardian_signals == [signal.SIGINT]

    @pytest.mark.parametrize(
        'ownership, expected_outcome, expected_signals',
        [
            ([True, False], 'sigint', [signal.SIGINT]),
            ([True, True, True, True, False], 'sigterm',
             [signal.SIGINT, signal.SIGTERM]),
            ([True, True, True, True, True, True, True, False], 'sigkill',
             [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]),
        ],
    )
    def test_guardian_ladder_never_escalates_after_owned_group_disappears(
            self, monkeypatch, ownership, expected_outcome, expected_signals):
        """A recycled visible PGID is not TERM/KILLed at any stage boundary."""
        ownership_results = iter(ownership)
        signals = []
        raw_group_probes = []

        def group_owned(*_args, **_kwargs):
            try:
                return next(ownership_results)
            except StopIteration:
                pytest.fail('the ownership predicate was called unexpectedly')

        def recycled_group_exists(process_group_id):
            raw_group_probes.append(process_group_id)
            return True

        monkeypatch.setattr(launcher, '_guardian_group_owned', group_owned)
        monkeypatch.setattr(
            launcher, '_signal_process_group',
            lambda pgid, signum: signals.append(signum))
        monkeypatch.setattr(
            launcher, '_process_group_exists', recycled_group_exists)

        outcome = launcher._stop_process_group(
            434343, 0.0, 0.0, 0.0,
            target=None, target_starttime=12345)

        assert outcome == expected_outcome
        assert signals == expected_signals
        assert raw_group_probes == [], (
            'numeric group visibility is not guardian ownership evidence')

    def test_normal_stop_group_broadcasts_sigint_and_hides_target_argv(
            self, spawner, tmp_path):
        """SIGINT reaches leader+grandchild; guardian argv has no address."""
        target = tmp_path / 'guarded_group.py'
        target.write_text(GUARDED_GROUP_TARGET)
        child = spawner(
            [sys.executable, str(target), DOC_IP],
            name='launch',
            parent_death_signal=signal.SIGINT,
        )
        grandchild_pid = int(_await_output(child, 'grandchild ').split()[1])
        target_group = child.target_process_group
        try:
            with open('/proc/{}/cmdline'.format(child.pid), 'rb') as handle:
                guardian_cmdline = handle.read()
            assert DOC_IP.encode('ascii') not in guardian_cmdline
            assert os.getpgid(target_group) == target_group

            assert child.stop(0.5, 0.5, 2.0) == 'sigint'

            assert not launcher._process_group_exists(target_group)
            assert _process_gone(target_group)
            assert _process_gone(grandchild_pid)
        finally:
            if launcher._process_group_exists(target_group):
                os.killpg(target_group, signal.SIGKILL)

    def test_guardian_cleans_group_after_target_leader_exits(
            self, spawner, tmp_path):
        """A leader exit is not success while another target-group member lives."""
        target = tmp_path / 'exiting_group_leader.py'
        target.write_text(EXITING_GROUP_LEADER)
        child = spawner(
            [sys.executable, str(target)],
            name='launch',
            parent_death_signal=signal.SIGINT,
        )
        grandchild_pid = int(_await_output(child, 'grandchild ').split()[1])
        target_group = child.target_process_group
        try:
            assert child.wait_exited(5.0), 'guardian did not finish group cleanup'
            assert child.stop(0.5, 0.5, 2.0) == 'already-exited'
            assert not launcher._process_group_exists(target_group)
            assert _process_gone(grandchild_pid)
        finally:
            if launcher._process_group_exists(target_group):
                os.killpg(target_group, signal.SIGKILL)

    def test_target_exit_fails_liveness_while_guardian_cleans_resistant_member(
            self, spawner, tmp_path):
        """F7 sees target death promptly; guardian still owns TERM escalation."""
        target = tmp_path / 'exiting_resistant_leader.py'
        target.write_text(EXITING_SIGINT_RESISTANT_LEADER)
        child = spawner(
            [sys.executable, str(target)],
            name='launch',
            parent_death_signal=signal.SIGINT,
        )
        grandchild_pid = int(_await_output(child, 'grandchild ').split()[1])
        target_group = child.target_process_group
        identity = child.target_process_starttime
        try:
            assert _wait_until(lambda: not child.alive(), 2.0)
            assert not launcher._same_process_is_live(target_group, identity)
            assert child._process.poll() is None, (
                'guardian exited before its resistant descendant was gone')
            assert launcher._process_group_exists(target_group)

            assert child.stop(0.5, 0.5, 2.0) == 'sigterm'

            assert not launcher._process_group_exists(target_group)
            assert _process_gone(grandchild_pid)
        finally:
            if launcher._process_group_exists(target_group):
                os.killpg(target_group, signal.SIGKILL)

    def test_guardian_crash_without_proof_blocks_parent_group_fallback(
            self, spawner, tmp_path):
        """A dead guardian never makes an unproved numeric target PGID safe."""
        target = tmp_path / 'fallback_group.py'
        target.write_text(GUARDED_GROUP_TARGET)
        child = spawner(
            [sys.executable, str(target)],
            name='launch',
            parent_death_signal=signal.SIGINT,
        )
        grandchild_pid = int(_await_output(child, 'grandchild ').split()[1])
        target_group = child.target_process_group
        target_starttime = child.target_process_starttime
        grandchild_starttime = launcher._process_starttime(grandchild_pid)
        try:
            os.kill(child.pid, signal.SIGKILL)
            assert child.wait_exited(5.0)
            assert _wait_until(
                lambda: not launcher._same_process_exists(
                    target_group, target_starttime), 5.0), (
                'target leader did not receive guardian-death SIGKILL')

            with pytest.raises(LauncherError, match='teardown proof'):
                child.stop(0.5, 0.5, 2.0)
        finally:
            for process_id, starttime in (
                    (target_group, target_starttime),
                    (grandchild_pid, grandchild_starttime)):
                if (starttime is not None
                        and launcher._same_process_exists(process_id, starttime)):
                    os.kill(process_id, signal.SIGKILL)
            assert _wait_until(lambda: _process_gone(grandchild_pid), 5.0)

    def test_target_group_ladder_reaches_sigkill(self, spawner, tmp_path):
        """The real subreaper guardian removes a resistant group with SIGKILL."""
        argv = _script(tmp_path, 'guarded_unstoppable.py', GROUP_LEADER_CHILD)
        child = spawner(
            argv,
            name='launch',
            parent_death_signal=signal.SIGINT,
        )
        grandchild_pid = int(_await_output(child, 'grandchild ').split()[1])
        target_pid = child.target_process_group
        target_starttime = child.target_process_starttime
        grandchild_starttime = launcher._process_starttime(grandchild_pid)
        try:
            outcome = child.stop(0.1, 0.1, 2.0)

            assert outcome == 'sigkill'
            assert not launcher._same_process_exists(
                target_pid, target_starttime)
            assert _process_gone(grandchild_pid)
        finally:
            for process_id, starttime in (
                    (target_pid, target_starttime),
                    (grandchild_pid, grandchild_starttime)):
                if (starttime is not None
                        and launcher._same_process_exists(process_id, starttime)):
                    os.kill(process_id, signal.SIGKILL)

    def test_parent_death_guardian_removes_target_and_grandchild(
            self, spawner, tmp_path):
        """SIGKILL-shaped owner loss still runs the guardian's group ladder."""
        target = tmp_path / 'parent_death_group.py'
        target.write_text(GUARDED_GROUP_TARGET)
        owner = tmp_path / 'guardian_owner.py'
        owner.write_text(GUARDIAN_OWNER)
        parent = spawner([sys.executable, str(owner), str(target)], 'owner')
        guarded = _await_output(parent, 'guarded ').split()
        guardian_pid = int(guarded[1])
        target_group = int(guarded[2])
        grandchild_pid = int(guarded[3])
        try:
            os.kill(parent.pid, signal.SIGKILL)
            assert parent.wait_exited(5.0)
            assert parent.stop(0.5, 0.5, 0.5) == 'already-exited'
            assert _wait_until(lambda: _process_gone(guardian_pid), 10.0)
            assert _wait_until(
                lambda: not launcher._process_group_exists(target_group), 10.0)
            assert _process_gone(target_group)
            assert _process_gone(grandchild_pid)
        finally:
            if launcher._process_group_exists(target_group):
                os.killpg(target_group, signal.SIGKILL)
            try:
                os.kill(guardian_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def test_missing_guardian_ready_fails_closed_and_cleans_target(
            self, monkeypatch, tmp_path):
        """A lost startup handshake cannot return a live untracked launch."""
        pid_path = tmp_path / 'target.pid'
        target = tmp_path / 'handshake_target.py'
        target.write_text(
            'import os, pathlib, time\n'
            'pathlib.Path({!r}).write_text(str(os.getpid()))\n'
            'time.sleep(120)\n'.format(str(pid_path)))
        monkeypatch.setattr(launcher, '_read_guardian_status',
                            lambda descriptor, timeout_s: b'')

        with pytest.raises(LauncherError):
            ChildProcess.spawn(
                [sys.executable, str(target)], dict(os.environ), 'launch',
                parent_death_signal=signal.SIGINT)

        if pid_path.exists():
            target_pid = int(pid_path.read_text())
            assert _wait_until(lambda: _process_gone(target_pid), 5.0)


class TestParentDeathSetup:
    """The preexec must prove both prctl success and a still-current parent."""

    def test_preexec_arms_signal_then_checks_parent(self, monkeypatch):
        """The successful path performs setpgid, prctl, then the race check."""
        calls = []
        monkeypatch.setattr(launcher.os, 'setpgid',
                            lambda pid, group: calls.append(('setpgid', pid, group)))
        monkeypatch.setattr(launcher.os, 'getppid',
                            lambda: calls.append(('getppid',)) or 1234)

        def prctl(*arguments):
            calls.append(('prctl',) + arguments)
            return 0

        def reset_signal(*arguments):
            calls.append(('signal',) + arguments)
            return None

        preexec = launcher._make_child_preexec(
            signal.SIGTERM, parent_pid=1234, prctl=prctl,
            libc_signal=reset_signal)
        preexec()

        assert calls == [
            ('setpgid', 0, 0),
            ('signal', int(signal.SIGTERM), launcher._SIG_DFL),
            ('prctl', launcher._PR_SET_PDEATHSIG, int(signal.SIGTERM), 0, 0, 0),
            ('getppid',),
        ]

    def test_preexec_rejects_prctl_failure(self, monkeypatch):
        """A failed safety syscall is a spawn failure, never best effort."""
        monkeypatch.setattr(launcher.os, 'setpgid', lambda pid, group: None)
        monkeypatch.setattr(launcher.os, 'getppid', lambda: 1234)
        preexec = launcher._make_child_preexec(
            signal.SIGTERM, parent_pid=1234, prctl=lambda *_args: -1,
            libc_signal=lambda *_args: None)

        with pytest.raises(OSError, match='parent-death'):
            preexec()

    def test_preexec_rejects_parent_death_race(self, monkeypatch):
        """A child reparented before prctl was armed refuses to exec target."""
        monkeypatch.setattr(launcher.os, 'setpgid', lambda pid, group: None)
        monkeypatch.setattr(launcher.os, 'getppid', lambda: 1)
        preexec = launcher._make_child_preexec(
            signal.SIGTERM, parent_pid=1234, prctl=lambda *_args: 0,
            libc_signal=lambda *_args: None)

        with pytest.raises(OSError, match='parent exited'):
            preexec()

    def test_preexec_rejects_signal_reset_failure(self, monkeypatch):
        """A copied server handler may never survive into the pre-exec race."""
        monkeypatch.setattr(launcher.os, 'setpgid', lambda pid, group: None)
        monkeypatch.setattr(launcher.os, 'getppid', lambda: 1234)
        preexec = launcher._make_child_preexec(
            signal.SIGTERM, parent_pid=1234, prctl=lambda *_args: 0,
            libc_signal=lambda *_args: launcher._SIG_ERR)

        with pytest.raises(OSError, match='signal disposition'):
            preexec()

    def test_preexec_does_not_reset_uncatchable_signal(self, monkeypatch):
        """SIGKILL needs no disposition reset and signal(2) would reject it."""
        calls = []
        monkeypatch.setattr(launcher.os, 'setpgid', lambda pid, group: None)
        monkeypatch.setattr(launcher.os, 'getppid', lambda: 1234)

        def unexpected_reset(*_arguments):
            calls.append('signal')
            return launcher._SIG_ERR

        def prctl(*arguments):
            calls.append(('prctl',) + arguments)
            return 0

        preexec = launcher._make_child_preexec(
            signal.SIGKILL, parent_pid=1234, prctl=prctl,
            libc_signal=unexpected_reset)
        preexec()

        assert calls == [
            ('prctl', launcher._PR_SET_PDEATHSIG,
             int(signal.SIGKILL), 0, 0, 0),
        ]


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

    def test_direct_reader_start_failure_proves_group_gone(
            self, monkeypatch, tmp_path):
        """A post-Popen direct-child adoption failure cannot leak its group."""
        processes = []
        real_popen = launcher.subprocess.Popen

        def track_popen(*args, **kwargs):
            process = real_popen(*args, **kwargs)
            processes.append(process)
            return process

        def fail_reader_start(_thread):
            raise RuntimeError('scripted reader start failure')

        monkeypatch.setattr(launcher.subprocess, 'Popen', track_popen)
        monkeypatch.setattr(launcher.threading.Thread, 'start', fail_reader_start)
        with pytest.raises(LauncherError) as excinfo:
            ChildProcess.spawn(
                _script(tmp_path, 'adoption_direct.py', COOPERATIVE_CHILD)
                + ['robot_ip:={}'.format(DOC_IP)],
                dict(os.environ), 'recorder')

        assert len(processes) == 1
        process = processes[0]
        assert process.poll() is not None
        assert not launcher._process_group_exists(process.pid)
        assert 'recorder' in str(excinfo.value)
        assert DOC_IP not in str(excinfo.value)

    def test_guarded_reader_start_failure_proves_target_and_guardian_gone(
            self, monkeypatch, tmp_path):
        """A post-READY launch adoption failure tears down both ownership layers."""
        processes = []
        statuses = []
        real_popen = launcher.subprocess.Popen
        real_read_status = launcher._read_guardian_status

        def track_popen(*args, **kwargs):
            process = real_popen(*args, **kwargs)
            processes.append(process)
            return process

        def capture_status(descriptor, timeout_s):
            status = real_read_status(descriptor, timeout_s)
            statuses.append(status)
            return status

        def fail_reader_start(_thread):
            raise RuntimeError('scripted reader start failure')

        monkeypatch.setattr(launcher.subprocess, 'Popen', track_popen)
        monkeypatch.setattr(launcher, '_read_guardian_status', capture_status)
        monkeypatch.setattr(launcher.threading.Thread, 'start', fail_reader_start)
        with pytest.raises(LauncherError) as excinfo:
            ChildProcess.spawn(
                _script(tmp_path, 'adoption_guarded.py', COOPERATIVE_CHILD)
                + ['robot_ip:={}'.format(DOC_IP)],
                dict(os.environ), 'launch',
                parent_death_signal=signal.SIGINT)

        assert len(processes) == 1
        assert len(statuses) == 1 and statuses[0].startswith(launcher._GUARDIAN_READY)
        target_group = int(statuses[0].split()[1])
        guardian = processes[0]
        assert guardian.poll() is not None
        assert not launcher._process_group_exists(target_group)
        assert 'launch' in str(excinfo.value)
        assert DOC_IP not in str(excinfo.value)


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

    def test_server_death_guardian_retains_lock_until_target_group_gone(
            self, spawner, tmp_path):
        """A replacement owner is refused throughout orphan cleanup."""
        pidfile = tmp_path / 'franka_web.pid'
        target = tmp_path / 'resistant_launch_group.py'
        target.write_text(GROUP_LEADER_CHILD)
        owner = tmp_path / 'lock_holding_owner.py'
        owner.write_text(LOCK_HOLDING_OWNER)
        parent = spawner(
            [sys.executable, str(owner), str(pidfile), str(target)],
            'lock-owner')
        guarded = _await_output(parent, 'guarded ').split()
        guardian_pid = int(guarded[1])
        target_group = int(guarded[2])
        try:
            os.kill(parent.pid, signal.SIGKILL)
            assert parent.wait_exited(5.0)

            guardian_fds = []
            for entry in os.listdir('/proc/{}/fd'.format(guardian_pid)):
                try:
                    guardian_fds.append(os.readlink(
                        '/proc/{}/fd/{}'.format(guardian_pid, entry)))
                except OSError:
                    pass
            target_fds = []
            for entry in os.listdir('/proc/{}/fd'.format(target_group)):
                try:
                    target_fds.append(os.readlink(
                        '/proc/{}/fd/{}'.format(target_group, entry)))
                except OSError:
                    pass
            assert str(pidfile) in guardian_fds
            assert str(pidfile) not in target_fds
            assert self._claim_in_subprocess(tmp_path, pidfile).startswith('conflict:')

            assert _wait_until(
                lambda: not launcher._process_group_exists(target_group), 30.0)
            assert _wait_until(lambda: _process_gone(guardian_pid), 5.0)
            assert self._claim_in_subprocess(tmp_path, pidfile) == 'acquired'
        finally:
            if launcher._process_group_exists(target_group):
                os.killpg(target_group, signal.SIGKILL)
            try:
                os.kill(guardian_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def test_server_death_direct_recorder_retains_lock_but_bagger_does_not(
            self, spawner, tmp_path):
        """Recorder cleanup holds exclusion; its close-fds child gets no lock."""
        pidfile = tmp_path / 'franka_web.pid'
        target = tmp_path / 'resistant_recorder_group.py'
        target.write_text(DIRECT_RECORDER_GROUP)
        owner = tmp_path / 'direct_recorder_lock_owner.py'
        owner.write_text(DIRECT_RECORDER_LOCK_OWNER)
        parent = spawner(
            [sys.executable, str(owner), str(pidfile), str(target)],
            'direct-lock-owner')
        direct = _await_output(parent, 'direct ').split()
        recorder_pid = int(direct[1])
        bagger_pid = int(direct[2])
        try:
            os.kill(parent.pid, signal.SIGKILL)
            assert parent.wait_exited(5.0)
            assert not _process_gone(recorder_pid)

            recorder_fds = []
            for entry in os.listdir('/proc/{}/fd'.format(recorder_pid)):
                try:
                    recorder_fds.append(os.readlink(
                        '/proc/{}/fd/{}'.format(recorder_pid, entry)))
                except OSError:
                    pass
            bagger_fds = []
            for entry in os.listdir('/proc/{}/fd'.format(bagger_pid)):
                try:
                    bagger_fds.append(os.readlink(
                        '/proc/{}/fd/{}'.format(bagger_pid, entry)))
                except OSError:
                    pass
            assert str(pidfile) in recorder_fds
            assert str(pidfile) not in bagger_fds
            assert self._claim_in_subprocess(tmp_path, pidfile).startswith('conflict:')

            os.killpg(recorder_pid, signal.SIGKILL)
            assert _wait_until(lambda: _process_gone(recorder_pid), 5.0)
            assert _wait_until(lambda: _process_gone(bagger_pid), 5.0)
            assert self._claim_in_subprocess(tmp_path, pidfile) == 'acquired'
        finally:
            if launcher._process_group_exists(recorder_pid):
                os.killpg(recorder_pid, signal.SIGKILL)

    def test_direct_adoption_failure_releases_child_flock_copy_after_cleanup(
            self, monkeypatch, tmp_path):
        """A failed reader start cannot strand a hidden pidfile-lock owner."""
        pidfile = tmp_path / 'franka_web.pid'
        target = tmp_path / 'adoption_lock_recorder.py'
        target.write_text(COOPERATIVE_CHILD)
        lock = PidfileLock(str(pidfile))
        lock.acquire()

        def fail_reader_start(_thread):
            raise RuntimeError('scripted reader start failure')

        monkeypatch.setattr(launcher.threading.Thread, 'start', fail_reader_start)
        try:
            with pytest.raises(LauncherError):
                ChildProcess.spawn(
                    [sys.executable, str(target)], dict(os.environ), 'recorder')
        finally:
            lock.release()

        assert self._claim_in_subprocess(tmp_path, pidfile) == 'acquired'


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
