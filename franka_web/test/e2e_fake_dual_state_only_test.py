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
Stage 1 end-to-end: the real server, the fake dual stack, nothing left behind.

This is the plan's section 8 Stage 1 e2e spec. It starts the INSTALLED
``franka_web_server`` as a subprocess and drives it with ``http.client`` only,
so the supervision code under test is the shipped code and not a fake.

FAKE HARDWARE ONLY
    The only session this file can ever start is ``{"arms": "both", "mode":
    "simulate"}``, which the frozen profile table maps to
    ``fake_dual_state_only.launch.py``. The three ``FRANKA_WEB_ROBOT_IP*``
    variables are scrubbed out of the server's environment before it is
    spawned (and the scrub is asserted), so even an operator shell that has
    them exported cannot turn this test into a production launch.

Domain isolation
    The stack this test brings up is a real ROS graph. It therefore runs on
    the ID this package reserves for it -- ``ROS_DOMAIN_ID=219``, see the
    allocation table in ``CMakeLists.txt`` -- and SKIPS itself when the
    environment does not say so, so a bare ``pytest`` invocation outside the
    CMake registration can never publish onto somebody else's domain.

Process accounting uses ``/proc`` only
    Never ``pgrep``: ``/proc/<pid>/comm`` is truncated to 15 characters, so
    ``pgrep -x ros2_control_node`` matches nothing at all and would report a
    running stack as gone. Everything here reads ``/proc/<pid>/cmdline``,
    ``/proc/<pid>/task/*/children`` and ``/proc/<pid>/environ`` directly, skips
    this test's own pid and its ancestors, and ignores any process that is not
    on domain 219.

Finding D-E2E-1 (FIXED) -- server SIGKILL used to orphan the launch subtree
    First measured on this branch with ``PR_SET_PDEATHSIG = SIGTERM`` for
    every child: upstream ``launch`` answers SIGTERM by cancelling WITHOUT
    tearing its tree down (``launch/launch_service.py`` ``_on_sigterm``, a
    known upstream TODO), so ``ros2 launch`` itself died in ~0.2 s while
    ``robot_state_publisher``, ``franka_joint_state_publisher`` and
    ``ros2_control_node`` survived indefinitely. Fixed in ``launcher.py`` by
    making the parent-death signal per-child: the launch child now gets
    ``SIGINT`` (the one signal ``ros2 launch`` answers with a full ordered
    tree teardown) while ``franka_record`` keeps ``SIGTERM`` (its clean,
    bag-sealing stop). Case 2 therefore asserts the STRONG property: after a
    server SIGKILL, every tracked descendant -- launch subtree included --
    is gone, and the bag is sealed.

The clean-stop path has no such gap: ``SessionSupervisor._do_stopping`` sends
SIGINT to the launch pid, which ``ros2 launch`` handles properly, and case 1
asserts that zero franka processes survive it.
"""

import http.client
import json
import os
import signal
import socket
import stat
import subprocess
import sys
import time
import warnings

from ament_index_python.packages import get_package_prefix
from franka_web.config import STATE_FRAME_HZ, STOP_ADVISORY
import jsonschema
import pytest

#: The ID this package reserves for the Stage 1 e2e (CMakeLists.txt table).
REQUIRED_DOMAIN_ID = '219'

_SKIP_REASON = (
    'the Stage 1 e2e brings up a real ROS graph and must stay on its reserved '
    'domain: run it through the CMake registration, or export '
    'ROS_DOMAIN_ID={} (plus FASTDDS_BUILTIN_TRANSPORTS=SHM and '
    'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST) yourself'.format(REQUIRED_DOMAIN_ID))

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != REQUIRED_DOMAIN_ID, reason=_SKIP_REASON)

#: The normative frame schema handed to Session C.
SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'support', 'state_frame_schema.json')

#: Substrings identifying a process this test's session may have created.
#: ``franka_joint_state_publisher`` is matched by ``joint_state_publisher``.
PROCESS_MARKERS = (
    'ros2 launch',
    'ros2_control_node',
    'robot_state_publisher',
    'joint_state_publisher',
    'ros2 bag',
    'franka_record',
)

#: Never let an ambient operator shell turn this test into a real-robot run.
ADDRESS_VARIABLES = (
    'FRANKA_WEB_ROBOT_IP_1', 'FRANKA_WEB_ROBOT_IP_2', 'FRANKA_WEB_ROBOT_IP')

_POLL_S = 0.2
_SETTLE_S = 0.5


# ----------------------------------------------------------------------
# /proc helpers (never pgrep)
# ----------------------------------------------------------------------


def read_cmdline(pid):
    """Return a process's argv as one space-joined string, or None if it is gone."""
    try:
        with open('/proc/{}/cmdline'.format(pid), 'rb') as handle:
            raw = handle.read()
    except OSError:
        return None
    return raw.decode('utf-8', 'replace').replace('\x00', ' ').strip()


def read_domain_id(pid):
    """Return a process's ROS_DOMAIN_ID from /proc, or None when unreadable."""
    try:
        with open('/proc/{}/environ'.format(pid), 'rb') as handle:
            raw = handle.read()
    except OSError:
        return None
    for entry in raw.split(b'\x00'):
        if entry.startswith(b'ROS_DOMAIN_ID='):
            return entry.split(b'=', 1)[1].decode('utf-8', 'replace')
    return None


def read_parent_pid(pid):
    """Return a process's parent pid from /proc/<pid>/stat, or None."""
    try:
        with open('/proc/{}/stat'.format(pid), 'rb') as handle:
            raw = handle.read()
    except OSError:
        return None
    # comm sits in parentheses and may itself contain spaces and parentheses,
    # so the fields are counted from the LAST ')' rather than split naively.
    try:
        fields = raw[raw.rindex(b')') + 1:].split()
        return int(fields[1])
    except (IndexError, ValueError):
        return None


def alive(pid):
    """Return whether /proc still has an entry for ``pid``."""
    return os.path.exists('/proc/{}'.format(pid))


def own_lineage():
    """Return this test's pid and every ancestor pid, so the scan can skip them."""
    lineage = set()
    pid = os.getpid()
    while pid and pid not in lineage:
        lineage.add(pid)
        pid = read_parent_pid(pid)
    return lineage


def child_pids(pid):
    """Return the direct children of ``pid``, read from every one of its tasks."""
    found = []
    try:
        tasks = os.listdir('/proc/{}/task'.format(pid))
    except OSError:
        return found
    for task in tasks:
        try:
            with open('/proc/{}/task/{}/children'.format(pid, task),
                      encoding='ascii') as handle:
                text = handle.read()
        except OSError:
            continue
        found.extend(int(entry) for entry in text.split())
    return found


def descendant_pids(pid):
    """Return every descendant of ``pid``, breadth first."""
    seen = []
    pending = list(child_pids(pid))
    while pending:
        candidate = pending.pop(0)
        if candidate in seen:
            continue
        seen.append(candidate)
        pending.extend(child_pids(candidate))
    return seen


def scan_processes(lineage):
    """
    Return ``{pid: cmdline}`` for every domain-219 process matching a marker.

    This test's own pid and its ancestors are skipped (a shell command line
    can easily contain one of the markers), and so is any process whose
    ``ROS_DOMAIN_ID`` is readable and is not ours -- another workspace's stack
    on another domain is none of this test's business.
    """
    found = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in lineage:
            continue
        line = read_cmdline(pid)
        if not line or not any(marker in line for marker in PROCESS_MARKERS):
            continue
        if read_domain_id(pid) not in (None, REQUIRED_DOMAIN_ID):
            continue
        found[pid] = line
    return found


def reap(pids):
    """SIGKILL every pid that is still alive; return the ones that needed it."""
    reaped = {}
    for pid, line in pids.items():
        if not alive(pid):
            continue
        reaped[pid] = line
        try:
            os.kill(pid, signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and any(alive(pid) for pid in reaped):
        time.sleep(_POLL_S)
    return reaped


def describe(processes):
    """Render a ``{pid: cmdline}`` mapping as readable lines."""
    return '\n'.join(
        '  {} {}'.format(pid, (line or '')[:160]) for pid, line in sorted(processes.items()))


def wait_until_gone(pid, timeout_s):
    """Poll /proc until ``pid`` is gone; return the seconds it took, or None."""
    started = time.monotonic()
    while time.monotonic() - started < timeout_s:
        if not alive(pid):
            return time.monotonic() - started
        time.sleep(_POLL_S)
    return None


def wait_for_file(path, timeout_s):
    """Poll until ``path`` is a non-empty regular file; return whether it appeared."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if os.path.isfile(path) and os.path.getsize(path) > 0:
            return True
        time.sleep(_POLL_S)
    return False


# ----------------------------------------------------------------------
# Schema and SSE helpers
# ----------------------------------------------------------------------


def load_validator():
    """Build a draft 2020-12 validator for the normative state-frame schema."""
    with open(SCHEMA_PATH, encoding='utf-8') as handle:
        schema = json.load(handle)
    validator_class = jsonschema.validators.validator_for(schema)
    validator_class.check_schema(schema)
    return validator_class(schema)


def schema_errors(validator, frame):
    """Return every schema violation of ``frame`` as readable lines."""
    problems = sorted(validator.iter_errors(frame), key=str)
    return '\n'.join(
        '  {}: {}'.format('/'.join(str(part) for part in error.absolute_path) or '<root>',
                          error.message)
        for error in problems)


def parse_sse(payload):
    """
    Split a raw SSE byte stream into ``(event_name, data_object)`` pairs.

    Only the LAST block may be a partial frame (the read window closed
    mid-write); anything else that fails to parse is a real wire-format
    violation and is raised rather than skipped.
    """
    records = []
    blocks = payload.split(b'\n\n')
    for index, block in enumerate(blocks):
        name = None
        data = []
        for line in block.split(b'\n'):
            if line.startswith(b'event: '):
                name = line[len(b'event: '):].decode('utf-8')
            elif line.startswith(b'data: '):
                data.append(line[len(b'data: '):].decode('utf-8'))
        if name is None or not data:
            continue
        try:
            records.append((name, json.loads('\n'.join(data))))
        except ValueError:
            if index == len(blocks) - 1:
                continue
            raise
    return records


def collect_sse(port, window_s):
    """
    Read ``GET /api/state/stream`` with a raw socket for ``window_s`` seconds.

    Returns ``(head_bytes, records)``. A raw socket rather than
    ``http.client`` because the stream has neither a Content-Length nor a
    chunked encoding: it is bytes until the connection closes.
    """
    request = (
        'GET /api/state/stream HTTP/1.1\r\n'
        'Host: 127.0.0.1:{}\r\n'
        'Accept: text/event-stream\r\n'
        'Connection: close\r\n'
        '\r\n').format(port).encode('ascii')
    buffered = b''
    sock = socket.create_connection(('127.0.0.1', port), timeout=10.0)
    try:
        sock.sendall(request)
        sock.settimeout(0.5)
        deadline = time.monotonic() + window_s
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break
            buffered += chunk
    finally:
        sock.close()
    head, separator, body = buffered.partition(b'\r\n\r\n')
    assert separator, 'the SSE response never produced a complete header block'
    return head.decode('utf-8', 'replace'), parse_sse(body)


# ----------------------------------------------------------------------
# The server under test
# ----------------------------------------------------------------------


def free_port():
    """Bind port 0 on loopback, release it, and return the number the kernel chose."""
    sock = socket.socket()
    try:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]
    finally:
        sock.close()


def server_executable():
    """Resolve the INSTALLED franka_web_server through the ament index."""
    path = os.path.join(
        get_package_prefix('franka_web'), 'lib', 'franka_web', 'franka_web_server')
    if not os.path.isfile(path):
        pytest.skip('franka_web is not installed; build and source the workspace first')
    return path


class Server:
    """One real ``franka_web_server`` subprocess, driven over HTTP only."""

    def __init__(self, root):
        """Create the private directories, take a free port, and spawn the server."""
        self.root = root
        self.state_dir = os.path.join(root, 'state')
        self.recording_root = os.path.join(root, 'recordings')
        os.makedirs(self.state_dir, mode=0o700)
        os.makedirs(self.recording_root, mode=0o700)
        self.port = free_port()
        self.log_path = os.path.join(root, 'server.log')
        self.token = None

        environment = dict(os.environ)
        for name in ADDRESS_VARIABLES:
            environment.pop(name, None)
        environment.update({
            'FRANKA_WEB_BIND': '127.0.0.1',
            'FRANKA_WEB_PORT': str(self.port),
            'FRANKA_WEB_STATE_DIR': self.state_dir,
            'FRANKA_WEB_RECORDING_ROOT': self.recording_root,
            'ROS_DOMAIN_ID': REQUIRED_DOMAIN_ID,
            'ROS_HOME': os.path.join(root, 'ros_home'),
            'ROS_LOG_DIR': os.path.join(root, 'ros_log'),
            'PYTHONUNBUFFERED': '1',
        })
        assert not [name for name in ADDRESS_VARIABLES if name in environment], (
            'a robot address leaked into the e2e server environment')
        self.environment = environment
        self._log = open(self.log_path, 'wb')
        self.process = subprocess.Popen(
            [sys.executable, server_executable()],
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=self._log,
            stderr=subprocess.STDOUT)

    # -- diagnostics ---------------------------------------------------

    def log_tail(self, limit=40):
        """Return the tail of the server's own stdout/stderr, for post-mortems."""
        try:
            with open(self.log_path, encoding='utf-8', errors='replace') as handle:
                lines = handle.read().splitlines()
        except OSError:
            return '<the server log could not be read>'
        return '\n'.join(lines[-limit:]) or '<the server log is empty>'

    def fail(self, message):
        """Raise an AssertionError carrying the server log tail."""
        raise AssertionError('{}\n\nserver log tail ({}):\n{}'.format(
            message, self.log_path, self.log_tail()))

    # -- lifecycle -----------------------------------------------------

    def wait_until_listening(self, timeout_s=45.0):
        """Poll the port until the server accepts a connection."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.fail('the server exited with status {} before it listened'.format(
                    self.process.returncode))
            try:
                socket.create_connection(('127.0.0.1', self.port), 0.5).close()
                return
            except OSError:
                time.sleep(_POLL_S)
        self.fail('the server never listened on port {}'.format(self.port))

    def shutdown(self):
        """Stop the server process if it is still up; idempotent."""
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=90)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=30)
        self._log.close()

    # -- HTTP ----------------------------------------------------------

    def request(self, method, path, body=None, token=None, expect=200):
        """Send one request and return its decoded JSON body."""
        headers = {'Host': '127.0.0.1:{}'.format(self.port)}
        payload = None
        if body is not None:
            payload = json.dumps(body).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        if token:
            headers['X-Operator-Token'] = token
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=20.0)
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            raw = response.read()
            status = response.status
            cors = [name for name, _ in response.getheaders()
                    if name.lower().startswith('access-control-allow-')]
        finally:
            connection.close()
        assert not cors, 'the server emitted CORS headers {} (plan section 5.7)'.format(cors)
        try:
            decoded = json.loads(raw.decode('utf-8'))
        except ValueError:
            self.fail('{} {} answered non-JSON: {!r}'.format(method, path, raw[:200]))
        if status != expect:
            self.fail('{} {} answered {} (expected {}): {}'.format(
                method, path, status, expect, decoded))
        return decoded

    def claim(self):
        """Claim the single-operator lock and remember the token."""
        self.token = self.request('POST', '/api/operator/claim')['token']
        return self.token

    def heartbeat(self):
        """Refresh the operator lock (its TTL is 15 s)."""
        self.request('POST', '/api/operator/heartbeat', token=self.token)

    def state(self):
        """Return the current section 6.11 frame from GET /api/state."""
        return self.request('GET', '/api/state')['state']

    def settle(self, seconds):
        """Wait ``seconds`` while keeping the operator lock alive."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            time.sleep(min(_SETTLE_S, max(0.0, deadline - time.monotonic())))
            self.heartbeat()

    def wait_for_session_state(self, wanted, timeout_s):
        """Poll GET /api/state until ``session.state`` is ``wanted``."""
        deadline = time.monotonic() + timeout_s
        seen = []
        while time.monotonic() < deadline:
            frame = self.state()
            current = frame['session']['state']
            if not seen or seen[-1] != current:
                seen.append(current)
            if current == wanted:
                return frame
            self.heartbeat()
            time.sleep(_SETTLE_S)
        self.fail('the session never reached {!r} within {:.0f} s; states seen: {}'.format(
            wanted, timeout_s, ' -> '.join(seen)))

    def wait_for_frame(self, predicate, timeout_s, description):
        """Poll GET /api/state until ``predicate(frame)`` holds."""
        deadline = time.monotonic() + timeout_s
        frame = None
        while time.monotonic() < deadline:
            frame = self.state()
            if predicate(frame):
                return frame
            self.heartbeat()
            time.sleep(_SETTLE_S)
        self.fail('{} did not hold within {:.0f} s; last frame: {}'.format(
            description, timeout_s, json.dumps(frame, sort_keys=True)[:2000]))


def start_simulate_session(server):
    """Claim the lock and start the one permitted (fake dual) session."""
    server.claim()
    accepted = server.request(
        'POST', '/api/session/start', {'arms': 'both', 'mode': 'simulate'},
        token=server.token, expect=202)
    assert accepted['ok'] is True
    assert accepted['state'] == 'preflight'
    return accepted['session_id']


# ----------------------------------------------------------------------
# Case 1 -- the plan's numbered steps
# ----------------------------------------------------------------------


def test_fake_dual_simulate_session(tmp_path):
    """
    Drive one whole simulate/both session and assert the machine is left clean.

    Plan section 8 Stage 1, steps 1-7: claim, start, reach ``running``,
    fourteen joints resolved by name, the explicit fake-mode degradation,
    a growing 0700 recording, five schema-valid SSE frames, then stop with a
    sealed bag and zero surviving franka processes.
    """
    lineage = own_lineage()
    preexisting = scan_processes(lineage)
    if preexisting:
        warnings.warn(
            'domain {} was not clean before this test:\n{}'.format(
                REQUIRED_DOMAIN_ID, describe(preexisting)), stacklevel=1)
    validator = load_validator()

    root = str(tmp_path / 'case1')
    os.makedirs(root, mode=0o700)
    server = Server(root)
    try:
        # 1. wait for the port, claim the lock, start the session.
        server.wait_until_listening()
        session_id = start_simulate_session(server)

        # 2. poll until running (<= 60 s).
        frame = server.wait_for_session_state('running', 60.0)
        assert frame['schema_version'] == 1
        assert frame['session']['session_id'] == session_id
        assert frame['session']['arms'] == 'both'
        assert frame['session']['mode'] == 'simulate'
        assert frame['session']['arm_mode'] == 'dual'
        assert frame['session']['arm_ids'] == ['panda1', 'panda2']
        assert frame['session']['launch_running'] is True
        assert frame['session']['last_error'] is None
        assert frame['session']['advisory'] == STOP_ADVISORY
        assert frame['fault']['active'] is False
        assert frame['fault']['reasons'] == []
        errors = schema_errors(validator, frame)
        assert not errors, 'the running GET /api/state frame is not contract-shaped:\n' + errors

        # 3. fourteen joints resolve BY NAME into two arms of seven.
        assert sorted(frame['arms']) == ['panda1', 'panda2']
        resolved = []
        for arm_id in ('panda1', 'panda2'):
            arm = frame['arms'][arm_id]
            expected = ['{}_joint{}'.format(arm_id, index) for index in range(1, 8)]
            assert arm['arm_id'] == arm_id
            assert arm['joint_names'] == expected
            assert len(arm['positions']) == 7
            assert all(value is not None for value in arm['positions']), (
                'a joint name did not resolve out of the 14-name JointState: {}'.format(arm))
            assert arm['positions_stale'] is False
            assert arm['positions_age_s'] is not None
            assert arm['status'] == 'ok'
            assert arm['status_line']
            resolved.extend(arm['joint_names'])
        assert len(set(resolved)) == 14, (
            'the two arms did not resolve fourteen distinct joints: {}'.format(sorted(resolved)))

        # 4. fake-mode degradation is stated, never faked (frame rule 4).
        for arm_id in ('panda1', 'panda2'):
            arm = frame['arms'][arm_id]
            robot_state = arm['robot_state']
            assert robot_state['available'] is False
            for key in ('age_s', 'control_command_success_rate', 'robot_mode',
                        'robot_mode_label', 'current_errors', 'last_motion_errors'):
                assert robot_state[key] is None, (
                    'robot_state.{} must be null on mock hardware, not a plausible '
                    'zero: {}'.format(key, robot_state))
            diagnostic = arm['diagnostic']
            assert diagnostic['available'] is False
            for key in ('level', 'level_label', 'message', 'age_s'):
                assert diagnostic[key] is None, (
                    'diagnostic.{} must be null on mock hardware: {}'.format(key, diagnostic))
            assert diagnostic['values'] == {}
            assert arm['motion']['available'] is False
            assert arm['motion']['enabled'] is False
        hardware = server.wait_for_frame(
            lambda current: current['hardware']['plugin_name'] is not None,
            30.0, 'the hardware component plugin name')['hardware']
        assert hardware['available'] is True
        assert hardware['plugin_name'] == 'mock_components/GenericSystem'
        assert hardware['lifecycle_label'] == 'active'

        # 5. the recording exists under the root, is private, and grows.
        recording = frame['recording']
        assert recording['active'] is True
        assert recording['name'] == session_id
        assert recording['sequence'] == 1
        assert recording['arm_mode'] == 'dual'
        directory = recording['path']
        assert directory == os.path.join(server.recording_root, session_id)
        assert os.path.isdir(directory), 'the recording directory was not created'
        assert stat.S_IMODE(os.stat(directory).st_mode) == 0o700, (
            'the recording directory is not private (expected mode 0700)')
        bag = os.path.join(directory, 'bag', 'bag_0.mcap')
        assert wait_for_file(bag, 30.0), 'the session bag file never appeared at {}'.format(bag)
        before = os.path.getsize(bag)
        server.settle(2.0)
        after = os.path.getsize(bag)
        assert after > before, (
            'the session bag is not growing ({} -> {} bytes over ~2 s); the recorder is '
            'running but capturing nothing'.format(before, after))

        # 6. the SSE stream carries at least five schema-valid state frames in 2 s.
        head, records = collect_sse(server.port, 2.0)
        assert ' 200 ' in head, 'the SSE stream did not answer 200:\n{}'.format(head)
        assert 'text/event-stream' in head.lower(), (
            'the SSE stream is not text/event-stream:\n{}'.format(head))
        assert 'access-control-allow-' not in head.lower(), (
            'the SSE stream emitted a CORS header:\n{}'.format(head))
        frames = [payload for name, payload in records if name == 'state']
        assert len(frames) >= 5, (
            'only {} state events arrived in 2 s (expected at least 5, and about {:.0f} at '
            'the {} Hz frame cadence); events seen: {}'.format(
                len(frames), 2.0 * STATE_FRAME_HZ, STATE_FRAME_HZ,
                [name for name, _ in records]))
        for index, payload in enumerate(frames):
            errors = schema_errors(validator, payload)
            assert not errors, 'SSE state frame {} of {} violates the contract:\n{}'.format(
                index + 1, len(frames), errors)

        # 7. stop, and reach `stopped` within 30 s.
        server.heartbeat()
        stopping = server.request(
            'POST', '/api/session/stop', token=server.token, expect=202)
        assert stopping['state'] == 'stopping'
        assert stopping['advisory'] == STOP_ADVISORY
        final = server.wait_for_session_state('stopped', 30.0)
        assert final['arms'] == {}, 'frame rule 1: arms must be {} in stopped'
        assert final['controllers'] == []
        assert final['recording']['active'] is False
        assert final['session']['launch_running'] is False
        errors = schema_errors(validator, final)
        assert not errors, 'the stopped GET /api/state frame is not contract-shaped:\n' + errors

        # 7a. the bag is sealed: metadata.yaml sits next to bag_0.mcap.
        metadata = os.path.join(directory, 'bag', 'metadata.yaml')
        assert wait_for_file(metadata, 30.0), (
            'the bag was not sealed: {} is missing, so it would need ros2 bag '
            'reindex'.format(metadata))
        with open(metadata, encoding='utf-8', errors='replace') as handle:
            sealed = handle.read()
        assert 'bag_0.mcap' in sealed, (
            'the sealed metadata does not reference the bag file:\n{}'.format(sealed[:500]))

        # 7b. nothing survives the stop.
        survivors = {pid: line for pid, line in scan_processes(lineage).items()
                     if pid not in preexisting}
        assert not survivors, (
            'these processes survived the session stop:\n{}\n\nserver log tail:\n{}'.format(
                describe(survivors), server.log_tail()))
    finally:
        server.shutdown()
        leaked = reap({pid: line for pid, line in scan_processes(lineage).items()
                       if pid not in preexisting})
    assert not leaked, (
        'the clean-stop path leaked processes that the test had to kill:\n{}'.format(
            describe(leaked)))


# ----------------------------------------------------------------------
# Case 2 -- SIGKILL the server mid-session
# ----------------------------------------------------------------------


def test_server_sigkill_does_not_outlive_its_pdeathsig_children(tmp_path):
    """
    SIGKILL the server mid-session; its own children must die and seal.

    Plan section 8 Stage 1 step 8. ``PR_SET_PDEATHSIG`` is the only defence
    that survives a SIGKILL of the server, so this asserts on the two children
    the server actually spawns: the ``ros2 launch`` child (SIGINT -> full
    ordered teardown, gone within 30 s) and the ``franka_record`` chain,
    which answers SIGTERM by running its own bounded ladder against
    ``ros2 bag record`` and sealing the bag (gone, and sealed, within 30 s).

    The launch child's parent-death signal is SIGINT (finding D-E2E-1 in the
    module docstring), so ``ros2 launch`` performs its full ordered teardown
    and the WHOLE subtree must be gone -- asserted below with a bound wide
    enough for launch's own internal escalation.
    """
    lineage = own_lineage()
    preexisting = scan_processes(lineage)
    root = str(tmp_path / 'case2')
    os.makedirs(root, mode=0o700)
    server = Server(root)
    orphans = {}
    try:
        server.wait_until_listening()
        session_id = start_simulate_session(server)
        frame = server.wait_for_session_state('running', 60.0)
        directory = frame['recording']['path']
        assert directory == os.path.join(server.recording_root, session_id)
        bag = os.path.join(directory, 'bag', 'bag_0.mcap')
        assert wait_for_file(bag, 30.0), 'the session bag file never appeared at {}'.format(bag)
        metadata = os.path.join(directory, 'bag', 'metadata.yaml')

        # Track the children BEFORE the kill: afterwards there is no parent
        # left to walk down from.
        tracked = {pid: read_cmdline(pid) or ''
                   for pid in descendant_pids(server.process.pid)}
        assert tracked, 'the running server had no child processes at all'
        launch = [pid for pid, line in tracked.items() if 'ros2 launch' in line]
        recorder = [pid for pid, line in tracked.items() if 'franka_record' in line]
        bagger = [pid for pid, line in tracked.items() if 'ros2 bag' in line]
        assert len(launch) == 1, 'expected exactly one launch child:\n{}'.format(
            describe(tracked))
        assert len(recorder) == 1, 'expected exactly one franka_record child:\n{}'.format(
            describe(tracked))
        assert len(bagger) == 1, 'expected exactly one ros2 bag child:\n{}'.format(
            describe(tracked))
        server.process.kill()
        server.process.wait(timeout=30)

        # The launch child dies from PR_SET_PDEATHSIG(SIGINT). Unlike the old
        # SIGTERM instant-exit, SIGINT makes ros2 launch run its FULL ordered
        # teardown before exiting, so the bound is teardown-sized (30 s), not
        # signal-delivery-sized.
        took = wait_until_gone(launch[0], 30.0)
        assert took is not None, (
            'the ros2 launch child (pid {}) outlived the SIGKILLed server by more than '
            '30 s; PR_SET_PDEATHSIG did not fire:\n{}'.format(launch[0], describe(tracked)))

        # The recorder chain dies too, and seals the bag on the way out.
        for pid in recorder + bagger:
            assert wait_until_gone(pid, 30.0) is not None, (
                'the recorder chain (pid {}) outlived the SIGKILLed server by more than '
                '30 s: {}'.format(pid, tracked.get(pid)))
        assert wait_for_file(metadata, 30.0), (
            'franka_record did not seal the bag after the server was SIGKILLed: {} is '
            'missing'.format(metadata))
        with open(metadata, encoding='utf-8', errors='replace') as handle:
            sealed = handle.read()
        assert 'bag_0.mcap' in sealed, (
            'the sealed metadata does not reference the bag file:\n{}'.format(sealed[:500]))

        # D-E2E-1 fix: SIGINT as the launch child's parent-death signal means
        # ros2 launch runs its full ordered teardown. EVERY tracked pid --
        # launch subtree included -- must be gone; 30 s covers launch's own
        # internal SIGINT -> SIGTERM -> SIGKILL escalation with margin.
        for pid in sorted(tracked):
            assert wait_until_gone(pid, 30.0) is not None, (
                'pid {} ({}) survived the server SIGKILL: the launch teardown '
                'did not reach it'.format(pid, tracked.get(pid)))
        orphans = {pid: line for pid, line in tracked.items() if alive(pid)}
        assert not orphans, (
            'processes survived the server SIGKILL:\n{}'.format(describe(orphans)))
    finally:
        server.shutdown()
        leaked = reap({pid: line for pid, line in scan_processes(lineage).items()
                       if pid not in preexisting})
    assert not leaked, (
        'the SIGKILL path leaked processes that this test had to reap:\n{}'.format(
            describe(leaked)))
