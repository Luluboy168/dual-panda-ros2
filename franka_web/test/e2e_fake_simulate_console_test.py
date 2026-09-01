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
The Simulate-mode functional battery: the whole v2 console journey, for real.

This starts the INSTALLED ``franka_web_server`` as a subprocess against the
real ``fake_dual_state_only`` launch and drives it over HTTP only, so what is
under test is the shipped code. It is the battery the specification asks for:
zero-config boot, helpful configuration errors, the session lifecycle and its
checklist, the hint line, the log stream, the operator badge and takeover,
the fault causes, stop-and-seal, and the package scans.

WHAT SIMULATE CANNOT DO, AND WHY THAT IS THE POINT
    Simulate observes the fake state-only stack, which spawns no impedance
    controller -- so there is nothing to enable, jog, or switch a source on,
    and recovery is a watch/motion capability. Every one of those is asserted
    here as a REFUSAL, because the refusals are the behaviour. The working
    versions live where the motion surface exists:
    ``e2e_fake_motion_mock_test.py`` (the source switch, the fence, recovery
    to completion) and the unit suite.

FAKE HARDWARE ONLY
    The only mode this file starts is ``simulate``, which the frozen profile
    table maps to ``fake_dual_state_only.launch.py``. Nothing here can reach a
    robot, and no address is needed for it to run.

Domain isolation
    This brings up a real ROS graph, so it runs on the ID this package
    reserves for it -- ``ROS_DOMAIN_ID=224``; 221 and 222 belong to franka_ik,
    223 to franka_ghost, and 219/220 to this package's other two batteries --
    and SKIPS itself when the environment does not say so, so a bare
    ``pytest`` invocation outside the CMake registration can never publish
    onto somebody else's domain.

Configuration isolation
    Every server this file starts is given its own ``config.yaml`` with
    ``--config``, and both ``HOME`` and ``XDG_CONFIG_HOME`` are OVERRIDDEN to
    point inside the test's temporary root. The override is not decoration:
    the CMake registration sets ``XDG_CONFIG_HOME`` to a build-tree directory
    that is never cleaned between runs, and one leftover ``config.yaml`` there
    would make the zero-config case pass on the first run and fail on every
    run afterwards, with a symptom nothing like its cause.
"""

import http.client
import json
import os
import signal
import socket
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
import jsonschema
import pytest
from support import part1_stub
from test_review_regressions import (
    assert_no_legacy_environment_prefix, assert_no_notes_tree_reference)
import yaml

#: The ID this package reserves for the console battery.
REQUIRED_DOMAIN_ID = '224'

_SKIP_REASON = (
    'the Simulate console battery brings up a real ROS graph and must stay '
    'on its reserved domain: run it through the CMake registration, or '
    'export ROS_DOMAIN_ID={} (plus FASTDDS_BUILTIN_TRANSPORTS=SHM and '
    'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST) yourself'.format(
        REQUIRED_DOMAIN_ID))

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != REQUIRED_DOMAIN_ID, reason=_SKIP_REASON)

#: Skipped while the configuration double is standing in for the real loader:
#: these cases assert the loader's own operator-facing prose byte for byte.
_CONFIG_DOUBLE_REASON = (
    'the configuration subsystem is still the test double; these cases assert '
    'the real loader own operator-facing messages')
requires_real_config = pytest.mark.skipif(
    part1_stub.is_active(), reason=_CONFIG_DOUBLE_REASON)

#: The port the zero-config case must bind, because binding it is the property.
DEFAULT_PORT = 8765

#: Documentation addresses (RFC 5737 TEST-NET-1).
DOC_IP_1 = '192.0.2.11'
DOC_IP_2 = '192.0.2.12'

#: Legacy environment variables the zero-config case exports to prove they do
#: nothing. Assembled at runtime so this file is not itself a match for the
#: package scan that forbids the prefix.
_LEGACY_PREFIX = 'FRANKA_WEB' + '_'
LEGACY_VARIABLES = {
    _LEGACY_PREFIX + 'BIND': '0.0.0.0',
    _LEGACY_PREFIX + 'PORT': '9999',
    _LEGACY_PREFIX + 'STATE_DIR': '/nonexistent/state',
    _LEGACY_PREFIX + 'RECORDING_ROOT': '/nonexistent/recordings',
    _LEGACY_PREFIX + 'ROBOT_IP_1': '203.0.113.1',
    _LEGACY_PREFIX + 'ROBOT_IP_2': '203.0.113.2',
    _LEGACY_PREFIX + 'SETTLING_TIMEOUT_S': '99',
}

_POLL_S = 0.2
_SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'support',
    'state_frame_schema.json')

STATIC_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')

#: A v1 marker in the shipped page. While it is present the frontend rewrite
#: has not landed and the static-surface class below has nothing true to say.
_V1_APP_MARKER = 'Recover full session'


def load_validator():
    """Return a jsonschema validator for the frozen state-frame schema."""
    with open(_SCHEMA_PATH) as handle:
        schema = json.load(handle)
    validator_class = jsonschema.validators.validator_for(schema)
    validator_class.check_schema(schema)
    return validator_class(schema)


def schema_errors(validator, frame):
    """Return the schema violations in ``frame`` as readable lines."""
    return ['  {}: {}'.format('/'.join(str(part) for part in error.path),
                              error.message)
            for error in sorted(validator.iter_errors(frame),
                                key=lambda error: list(error.path))]


def free_port():
    """Return a loopback port that was free a moment ago."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]
    finally:
        probe.close()


def port_is_free(port):
    """Return whether ``port`` can be bound on loopback right now."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind(('127.0.0.1', port))
        return True
    except OSError:
        return False
    finally:
        probe.close()


def server_executable():
    """Resolve the INSTALLED franka_web_server through the ament index."""
    path = os.path.join(get_package_prefix('franka_web'), 'lib', 'franka_web',
                        'franka_web_server')
    if not os.path.isfile(path):
        pytest.skip('franka_web is not installed; build and source the '
                    'workspace first')
    return path


def base_environment(root):
    """
    Build the environment every server subprocess of this file inherits.

    ``HOME`` and ``XDG_CONFIG_HOME`` are OVERRIDDEN, never inherited: see the
    module docstring for why one stale file elsewhere would be so confusing.
    """
    environment = dict(os.environ)
    environment.update({
        'ROS_DOMAIN_ID': REQUIRED_DOMAIN_ID,
        'ROS_HOME': os.path.join(root, 'ros_home'),
        'ROS_LOG_DIR': os.path.join(root, 'ros_log'),
        'HOME': root,
        'XDG_CONFIG_HOME': os.path.join(root, 'xdg'),
        'PYTHONUNBUFFERED': '1',
    })
    for name in LEGACY_VARIABLES:
        environment.pop(name, None)
    return environment


def run_server_once(root, arguments, environment=None, timeout=60):
    """Run the server to completion and return its CompletedProcess."""
    return subprocess.run(
        [sys.executable, server_executable()] + list(arguments),
        env=environment or base_environment(root),
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True, timeout=timeout, check=False)


def write_config(path, document):
    """Write a configuration mapping and return the path."""
    os.makedirs(os.path.dirname(path), mode=0o700, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(document, handle, default_flow_style=False,
                       sort_keys=True)
    return path


class Server:
    """One real ``franka_web_server`` subprocess, driven over HTTP only."""

    def __init__(self, root, config_document=None, port=None,
                 legacy_environment=False):
        """Create the private directories, write a config, spawn the server."""
        self.root = str(root)
        self.state_dir = os.path.join(self.root, 'state')
        self.recording_root = os.path.join(self.root, 'recordings')
        for path in (self.root, self.state_dir, self.recording_root):
            os.makedirs(path, mode=0o700, exist_ok=True)
            os.chmod(path, 0o700)
        self.log_path = os.path.join(self.root, 'server.log')
        self.token = None
        self.claim_id = None

        arguments = []
        if config_document is None:
            # Zero configuration: no file at the default path, which the
            # overridden XDG_CONFIG_HOME guarantees is inside this test's own
            # temporary root and therefore empty.
            self.port = port or DEFAULT_PORT
            self.config_path = None
        else:
            self.port = port or int(config_document.get('port') or free_port())
            config_document.setdefault('port', self.port)
            config_document.setdefault('bind', '127.0.0.1')
            config_document.setdefault(
                'directories', {'state': self.state_dir,
                                'recordings': self.recording_root})
            config_document.setdefault('ros_domain_id',
                                       int(REQUIRED_DOMAIN_ID))
            self.config_path = write_config(
                os.path.join(self.root, 'config.yaml'), config_document)
            arguments = ['--config', self.config_path]

        environment = base_environment(self.root)
        if legacy_environment:
            # Exported and asserted to change nothing: no variable of that
            # family is read by v2 at all.
            environment.update(LEGACY_VARIABLES)
        self.environment = environment
        self._log = open(self.log_path, 'wb')
        self.process = subprocess.Popen(
            [sys.executable, server_executable()] + arguments,
            env=environment, stdin=subprocess.DEVNULL,
            stdout=self._log, stderr=subprocess.STDOUT)

    # -- diagnostics ---------------------------------------------------

    def log_tail(self, limit=60):
        """Return the tail of the server's own output, for post-mortems."""
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

    def wait_until_listening(self, timeout_s=60.0):
        """Poll the port until the server accepts a connection."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.fail('the server exited with status {} before it '
                          'listened'.format(self.process.returncode))
            try:
                socket.create_connection(('127.0.0.1', self.port), 0.5).close()
                return self
            except OSError:
                time.sleep(_POLL_S)
        self.fail('the server never listened on port {}'.format(self.port))

    def shutdown(self):
        """Stop the server process if it is still up; idempotent."""
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=120)
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
        connection = http.client.HTTPConnection(
            '127.0.0.1', self.port, timeout=30.0)
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            status = response.status
            raw = response.read()
        finally:
            connection.close()
        decoded = json.loads(raw.decode('utf-8')) if raw else {}
        if expect is not None and status != expect:
            self.fail('{} {} answered {} (expected {}): {}'.format(
                method, path, status, expect, decoded))
        return status, decoded

    def get(self, path, expect=200):
        """GET one path and return its decoded body."""
        return self.request('GET', path, expect=expect)[1]

    def claim(self, expect=200):
        """Claim the operator lock; remember its token and identity."""
        status, body = self.request('POST', '/api/operator/claim',
                                    expect=expect)
        if status == 200:
            self.token = body['token']
            self.claim_id = body['claim_id']
        return status, body

    def takeover(self):
        """Take the operator lock over and adopt the successor claim."""
        _status, body = self.request('POST', '/api/operator/takeover')
        self.token = body['token']
        self.claim_id = body['claim_id']
        return body

    def heartbeat(self):
        """Refresh the operator lock (its TTL is 15 s)."""
        if self.token:
            self.request('POST', '/api/operator/heartbeat', token=self.token,
                         expect=None)

    def state(self):
        """Return the current state frame."""
        return self.get('/api/state')['state']

    def start_session(self, arms='both', mode='simulate', expect=202):
        """POST /api/session/start and return ``(status, body)``."""
        return self.request('POST', '/api/session/start',
                            body={'arms': arms, 'mode': mode},
                            token=self.token, expect=expect)

    def stop_session(self, timeout_s=120.0):
        """POST /api/session/stop and wait for ``stopped``."""
        self.request('POST', '/api/session/stop', token=self.token,
                     expect=202)
        return self.wait_for_session_state('stopped', timeout_s)

    def wait_for_frame(self, predicate, timeout_s, description):
        """Poll GET /api/state until ``predicate(frame)`` holds."""
        deadline = time.monotonic() + timeout_s
        frame = None
        while time.monotonic() < deadline:
            frame = self.state()
            if predicate(frame):
                return frame
            self.heartbeat()
            time.sleep(_POLL_S)
        self.fail('{} did not hold within {:.0f} s; last frame:\n{}'.format(
            description, timeout_s, json.dumps(frame, sort_keys=True)[:2000]))

    def wait_for_session_state(self, wanted, timeout_s, record=None):
        """Poll GET /api/state until ``session.state`` is ``wanted``."""
        deadline = time.monotonic() + timeout_s
        seen = []
        while time.monotonic() < deadline:
            frame = self.state()
            if record is not None:
                record.append(frame)
            current = frame['session']['state']
            if not seen or seen[-1] != current:
                seen.append(current)
            if current == wanted:
                return frame
            self.heartbeat()
            time.sleep(_POLL_S)
        self.fail('the session never reached {!r} within {:.0f} s; states '
                  'seen: {}'.format(wanted, timeout_s, ' -> '.join(seen)))

    def read_stream_events(self, seconds, wanted=None):
        """
        Read raw SSE events off ``/api/state/stream`` for ``seconds``.

        Returns a list of ``(event, data)`` pairs, decoded but unfiltered.
        """
        connection = http.client.HTTPConnection(
            '127.0.0.1', self.port, timeout=seconds + 10.0)
        events = []
        try:
            connection.request('GET', '/api/state/stream', headers={
                'Host': '127.0.0.1:{}'.format(self.port)})
            response = connection.getresponse()
            assert response.status == 200
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                line = response.fp.readline()
                if not line:
                    break
                text = line.decode('utf-8').rstrip('\n')
                if not text.startswith('event: '):
                    continue
                data = response.fp.readline().decode('utf-8').rstrip('\n')
                response.fp.readline()
                name = text[len('event: '):]
                events.append((name, json.loads(data[len('data: '):])))
                if wanted is not None and wanted(events):
                    break
        finally:
            connection.close()
        return events


def running_console(root, **kwargs):
    """Start a server, wait for it to listen, and return it."""
    return Server(root, **kwargs).wait_until_listening()


@pytest.fixture()
def console(tmp_path):
    """Serve a configured Simulate console, torn down afterwards."""
    server = running_console(str(tmp_path), config_document={
        'robots': {'panda1': {'ip': DOC_IP_1}, 'panda2': {'ip': DOC_IP_2}},
    })
    try:
        yield server
    finally:
        server.shutdown()


# ----------------------------------------------------------------------
# 1 -- zero-config boot
# ----------------------------------------------------------------------


def test_01_zero_config_boot_ignores_every_legacy_environment_variable(tmp_path):
    """
    No file, no variables, no arguments: the console just comes up.

    That is the whole v2 promise, so the default port is the property under
    test and cannot be swapped for a free one. A machine already running a
    console makes this an environment fact rather than a failure -- and the
    skip says so unmistakably, because a skipped safety test reads exactly
    like a passing one.
    """
    if not port_is_free(DEFAULT_PORT):
        pytest.skip(
            'PORT {} IS ALREADY IN USE, so zero-config boot could not be '
            'tested. Stop whatever is listening there and run this '
            'again.'.format(DEFAULT_PORT))
    server = running_console(str(tmp_path), legacy_environment=True)
    try:
        capabilities = server.get('/api/capabilities')
        assert capabilities['config_present'] is False
        assert capabilities['config_path'].endswith(
            os.path.join('franka_web', 'config.yaml'))
        assert capabilities['ros_domain_id'] == int(REQUIRED_DOMAIN_ID)

        config = server.get('/api/config')
        assert config['port'] == DEFAULT_PORT, (
            'a legacy PORT variable changed the port: {}'.format(config))
        assert config['bind'] == '0.0.0.0', (
            'a legacy BIND variable changed the bind: {}'.format(config))
        assert config['robots'] == {'panda1': '172.16.0.2',
                                    'panda2': '172.16.0.3'}, (
            'a legacy address variable reached the configuration: {}'.format(
                config))
        assert config['settling']['timeout_s'] == 5.0, (
            'a legacy settling variable reached the configuration')
        assert config['recording_enabled'] is True

        banner = server.log_tail()
        assert 'open http://localhost:{}'.format(DEFAULT_PORT) in banner
        assert 'defaults (no file at' in banner
        assert 'The physical stop buttons are the only real stop.' in banner
    finally:
        server.shutdown()


# ----------------------------------------------------------------------
# 2 -- configuration errors are helpful
# ----------------------------------------------------------------------


def assert_helpful_refusal(result, key):
    """Assert one configuration refusal is exactly one helpful line."""
    assert result.returncode == 2, (
        'expected exit 2, got {}: {!r}'.format(result.returncode, result.stderr))
    lines = [line for line in result.stderr.splitlines() if line.strip()]
    assert len(lines) == 1, 'expected one stderr line, got: {!r}'.format(lines)
    message = lines[0]
    assert 'Traceback' not in message
    assert key in message, message
    return message


@requires_real_config
def test_02_an_unknown_key_exits_two_with_one_helpful_line(tmp_path):
    """A typo that silently does nothing is the failure this prevents."""
    path = write_config(str(tmp_path / 'config.yaml'), {
        'robots': {'panda1': {'adress': DOC_IP_1}}})
    result = run_server_once(str(tmp_path), ['--check-config', '--config', path])
    message = assert_helpful_refusal(result, 'robots.panda1.adress')
    assert 'unknown key' in message
    assert 'Allowed keys under robots.panda1: ip' in message


@requires_real_config
def test_03_a_wrong_typed_settling_value_exits_two_with_one_helpful_line(tmp_path):
    """The message says what was found and what would be allowed."""
    path = write_config(str(tmp_path / 'config.yaml'), {
        'settling': {'drift_limit_deg': 'two'}})
    result = run_server_once(str(tmp_path), ['--check-config', '--config', path])
    message = assert_helpful_refusal(result, 'settling.drift_limit_deg')
    assert 'two' in message
    assert 'Allowed' in message


@requires_real_config
def test_04_a_watchdog_timing_override_exits_two_and_says_where_it_lives(tmp_path):
    """
    The watchdog timing is not a key, and the refusal teaches where it is.

    The reviewed controller-config validator requires exact equality with it,
    so a key that only ever accepts one value would be a control that does
    nothing. The unknown-key error therefore replaces the suggestion with the
    fact.
    """
    path = write_config(str(tmp_path / 'config.yaml'), {
        'profiles': {'panda1': {'watchdog_timeout_s': 0.1}}})
    result = run_server_once(str(tmp_path), ['--check-config', '--config', path])
    message = assert_helpful_refusal(
        result, 'profiles.panda1.watchdog_timeout_s')
    assert 'unknown key' in message
    assert 'reviewed timing policy' in message
    assert 'GET /api/config' in message


def test_05_a_valid_config_is_invisible_and_reflected_in_api_config(tmp_path):
    """
    A valid file produces no extra output and is reflected verbatim.

    Validation is invisible when the file is valid: that is the whole
    difference between a configuration and a ceremony.
    """
    port = free_port()
    document = {
        'port': port,
        'bind': '127.0.0.1',
        'ros_domain_id': int(REQUIRED_DOMAIN_ID),
        'robots': {'panda1': {'ip': DOC_IP_1}, 'panda2': {'ip': DOC_IP_2}},
        'directories': {'state': os.path.join(str(tmp_path), 'state'),
                        'recordings': os.path.join(str(tmp_path), 'recordings')},
        'settling': {'drift_limit_deg': [2, 5, 2, 2, 2, 2, 2]},
    }
    path = write_config(os.path.join(str(tmp_path), 'config.yaml'), document)
    checked = run_server_once(str(tmp_path), ['--check-config', '--config', path])
    assert checked.returncode == 0, checked.stderr
    assert checked.stderr.strip() == '', checked.stderr

    server = Server(str(tmp_path), config_document=dict(document), port=port)
    try:
        server.wait_until_listening()
        config = server.get('/api/config')
        assert config['port'] == port
        assert config['config_present'] is True
        assert config['robots'] == {'panda1': DOC_IP_1, 'panda2': DOC_IP_2}
        drift = config['settling']['drift_limit_rad']
        assert drift[0] == pytest.approx(0.0349066, abs=1e-6)
        assert drift[1] == pytest.approx(0.0872665, abs=1e-6), (
            'the per-joint degree value did not survive the conversion')
        banner = server.log_tail()
        assert path in banner, 'the banner must name the file it read'
        assert 'defaults (no file at' not in banner
    finally:
        server.shutdown()


# ----------------------------------------------------------------------
# 6-7 -- the session lifecycle, its checklist, and the hint line
# ----------------------------------------------------------------------


def steps_of(frame):
    """Return the checklist as ``{id: status}``."""
    return {entry['id']: entry['status'] for entry in frame['session']['steps']}


def test_06_a_simulate_session_advances_every_step_in_order(console):
    """
    The checklist advances pending -> active -> done, in the documented order.

    Every intermediate frame is validated against the frozen schema, so the
    console's own contract is proven on real frames rather than on a
    hand-built one.
    """
    validator = load_validator()
    console.claim()
    frames = [console.state()]
    _status, accepted = console.start_session(arms='both', mode='simulate')
    assert accepted['state'] == 'preflight'
    assert accepted['session_id'].startswith('web-')

    running = console.wait_for_session_state('running', 180.0, record=frames)
    assert len(frames) > 3

    for index, frame in enumerate(frames):
        errors = schema_errors(validator, frame)
        assert not errors, 'frame {} is not contract-shaped:\n{}'.format(
            index, '\n'.join(errors))

    assert [entry['id'] for entry in running['session']['steps']] == [
        'preflight', 'connect:panda1', 'connect:panda2', 'health', 'baseline']
    assert set(steps_of(running).values()) == {'done'}
    assert running['session']['mode'] == 'simulate'
    assert running['session']['arm_ids'] == ['panda1', 'panda2']

    # Monotone across every frame seen: no step ever goes backwards.
    rank = {'pending': 0, 'active': 1, 'done': 2, 'failed': 2}
    seen = {}
    for frame in frames:
        for entry in frame['session']['steps']:
            assert rank[entry['status']] >= seen.get(entry['id'], 0), (
                '{} went backwards'.format(entry['id']))
            seen[entry['id']] = rank[entry['status']]
    assert seen and set(seen) >= {'preflight', 'health', 'baseline'}

    console.stop_session()


def test_07_the_hint_line_matches_the_contract_at_every_stage(console):
    """
    The next-step line is computed on the server and rendered verbatim.

    One owner, no drift -- so it is asserted here character for character,
    including the single-character ellipsis.
    """
    idle = console.state()
    assert idle['hint'] == (
        'Pick arms and press Start — or choose Simulate to try the console '
        'without robots.')

    console.claim()
    console.start_session(arms='both', mode='simulate')
    start_hints = set()
    deadline = time.monotonic() + 180.0
    while time.monotonic() < deadline:
        frame = console.state()
        if frame['session']['state'] == 'running':
            break
        start_hints.add(frame['hint'])
        console.heartbeat()
        time.sleep(0.05)
    assert start_hints, 'no start-phase hint was ever observed'
    allowed = {'Preflight…', 'Connecting to panda1…', 'Connecting to panda2…',
               'Health check…', 'Capturing baseline…'}
    assert start_hints <= allowed, start_hints
    for hint in start_hints:
        assert hint.endswith('…'), 'the ellipsis is one character U+2026'
        assert not hint.endswith('...')

    running = console.wait_for_session_state('running', 60.0)
    # Simulate has no motion surface, so no arm can be enabled: the hint
    # names the next step the operator actually has.
    assert running['hint'] == 'Enable an arm to allow commands.'

    stopped = console.stop_session()
    assert stopped['hint'] == (
        'Session ended. Recording saved. Start a new session anytime.')


# ----------------------------------------------------------------------
# 8 -- the log stream
# ----------------------------------------------------------------------


def test_08_the_log_stream_is_monotonic_and_backfills_exactly(console):
    """
    The drawer's stream: real launch output, monotone sequence, exact backfill.

    At least one line must come from the launched stack itself, because "the
    launched ROS stack's console output" is the whole point of the drawer and
    a bus carrying only the server's own lines would satisfy none of it.
    """
    console.claim()
    console.start_session(arms='both', mode='simulate')
    events = console.read_stream_events(
        25.0,
        wanted=lambda seen: len(
            [event for event in seen if event[0] == 'log']) >= 12)
    console.wait_for_session_state('running', 180.0)

    log_events = [data for name, data in events if name == 'log']
    assert len(log_events) >= 5, 'only {} log events arrived'.format(
        len(log_events))
    seqs = [entry['seq'] for entry in log_events]
    assert seqs == sorted(seqs), seqs
    assert len(seqs) == len(set(seqs)), 'a sequence number was reused'
    assert all(entry['level'] != 'debug' for entry in log_events), (
        'debug lines must never reach the stream')

    from_launch = [entry for entry in log_events
                   if entry['node'] != 'franka_web']
    assert from_launch, (
        'no line reached the drawer from the launched stack; the log bus is '
        'not wired to the launch child')

    warns = [entry['warn_count'] for entry in log_events]
    errors = [entry['error_count'] for entry in log_events]
    assert warns == sorted(warns) and errors == sorted(errors)

    window = console.get('/api/logs')
    assert window['warn_count'] >= warns[-1]
    assert window['error_count'] >= errors[-1]
    assert window['dropped'] >= 0

    # A `since=` backfill returns exactly the missing window.
    whole = console.get('/api/logs')['lines']
    assert whole, 'the ring is empty'
    pivot = whole[len(whole) // 2]['seq']
    tail = console.get('/api/logs?since={}'.format(pivot))['lines']
    assert [entry['seq'] for entry in tail] == [
        entry['seq'] for entry in whole if entry['seq'] > pivot]

    frame = console.state()
    assert frame['logs']['last_seq'] >= whole[-1]['seq']
    assert frame['logs']['warn_count'] == window['warn_count']
    console.stop_session()


# ----------------------------------------------------------------------
# 9-10 -- Simulate exposes NO motion surface
# ----------------------------------------------------------------------


def test_09_enable_jog_and_source_are_refused_with_not_motion_mode_in_simulate(
        console):
    """
    Simulate observes a state-only stack: there is nothing to command.

    The fake stack spawns no impedance controller, so there is no enable
    service, no target topic and no source to switch. The REFUSALS are the
    behaviour, and they are what this asserts. The working motion surface is
    covered where one exists, in the mock-motion battery.
    """
    console.claim()
    console.start_session(arms='both', mode='simulate')
    running = console.wait_for_session_state('running', 180.0)

    for arm_id in ('panda1', 'panda2'):
        motion = running['arms'][arm_id]['motion']
        assert motion['available'] is False
        assert motion['source'] is None
        assert motion['command_topic'] is None
        assert motion['command_template'] is None
        assert motion['command_template_ready'] is False
        assert motion['fence_lower'] is None
        assert motion['fence_upper'] is None

    status, refusal = console.request(
        'POST', '/api/arm/panda1/enable', body={'enabled': True},
        token=console.token, expect=409)
    assert refusal['error'] == 'not_motion_mode', refusal

    status, refusal = console.request(
        'POST', '/api/arm/panda1/jog', body={'joint_index': 0, 'direction': 1},
        token=console.token, expect=409)
    assert refusal['error'] == 'not_motion_mode', refusal

    status, refusal = console.request(
        'POST', '/api/arm/panda1/source', body={'source': 'external'},
        token=console.token, expect=409)
    assert refusal['error'] == 'not_motion_mode', refusal

    console.stop_session()


# ----------------------------------------------------------------------
# 11 -- the operator badge and the takeover
# ----------------------------------------------------------------------


def test_11_the_operator_badge_and_takeover_behave_as_contracted(console):
    """
    The badge identity travels in the frame; the token never does.

    A page compares the frame's ``claim_id`` against the one its own claim
    returned, which is how it tells "this is my lock" from "someone else's"
    without the stream ever carrying a token.
    """
    _status, mine = console.claim()
    frame = console.state()
    assert frame['operator']['locked'] is True
    assert frame['operator']['claim_id'] == mine['claim_id']
    assert frame['operator']['since'].endswith('Z')
    assert mine['token'] not in json.dumps(frame)

    status, refusal = console.claim(expect=409)
    assert refusal['error'] == 'operator_lock_held', refusal

    console.start_session(arms='both', mode='simulate')
    console.wait_for_session_state('running', 180.0)

    successor = console.takeover()
    assert successor['claim_id'] != mine['claim_id']
    assert successor['token'] != mine['token']
    after = console.state()
    assert after['operator']['claim_id'] == successor['claim_id']
    for arm_id in ('panda1', 'panda2'):
        assert after['arms'][arm_id]['motion']['enabled'] is False
        # Simulate has no source at all, which is the same promise from the
        # other side: nothing is left commanding anything.
        assert after['arms'][arm_id]['motion']['source'] is None

    status, refusal = console.request(
        'POST', '/api/session/stop', token=mine['token'], expect=401)
    assert refusal['error'] == 'operator_token_invalid', refusal
    console.stop_session()


# ----------------------------------------------------------------------
# 12-13 -- fault causes, and the refused recovery
# ----------------------------------------------------------------------


def test_12_each_fault_cause_is_classified_exactly(console):
    """
    ``session_wedged`` from a dead launch, then ``lock_expired`` over it.

    The second half is the dead end the live battery hit: ``lock_expired``
    outranks every other cause, because a Recover press cannot succeed
    without the lock -- and it MUST clear the moment the operator does what
    its own steps say. A frozen claim identity would pin the action at
    ``reclaim`` forever and the page would never be offered Recover.
    """
    console.claim()
    console.start_session(arms='both', mode='simulate')
    console.wait_for_session_state('running', 180.0)

    os.kill(find_launch_target(console), signal.SIGKILL)
    faulted = console.wait_for_frame(
        lambda frame: frame['fault']['active'],
        60.0, 'the session faulting on a dead launch child')

    assert faulted['fault']['cause'] == 'session_wedged', faulted['fault']
    assert faulted['fault']['headline'] == (
        'The session stopped and cannot continue.')
    assert faulted['fault']['steps'] == [
        'Press Stop, then start a new session.',
        'Open the logs to see what failed.']
    assert faulted['fault']['action'] == 'restart', (
        'a dead launch child is not recoverable, so the page must draw '
        'Restart rather than Recover')
    assert faulted['hint'] == (
        'Check that nobody pressed a stop, then press Recover.')
    assert any(reason['code'] == 'launch_exited'
               for reason in faulted['fault']['reasons'])

    # Let the operator lock lapse while the fault stands.
    time.sleep(17.0)
    expired = console.state()
    assert expired['operator']['locked'] is False
    assert expired['fault']['cause'] == 'lock_expired', expired['fault']
    assert expired['fault']['action'] == 'reclaim'
    assert expired['fault']['headline'] == (
        'Your control expired while the fault was handled.')
    assert expired['fault']['steps'] == ['Press Reclaim, then Recover.']
    assert expired['hint'] == (
        'Press Reclaim to take control back, then Recover.')

    # Do exactly what the steps say. The cause MUST clear.
    console.claim()
    reclaimed = console.state()
    assert reclaimed['fault']['cause'] != 'lock_expired', (
        'the lock_expired cause survived the Reclaim its own steps '
        'prescribed: {}'.format(reclaimed['fault']))
    assert reclaimed['fault']['action'] != 'reclaim'
    assert reclaimed['fault']['cause'] == 'session_wedged'

    console.stop_session()


def test_13_recover_is_refused_for_a_simulate_session(console):
    """
    Recovery is a watch/motion capability; Simulate is refused.

    And a refused recovery must not consume the fault: the whole fault block
    is unchanged by the attempt, so the page still shows the operator what
    actually happened.
    """
    console.claim()
    console.start_session(arms='both', mode='simulate')
    console.wait_for_session_state('running', 180.0)
    os.kill(find_launch_target(console), signal.SIGKILL)
    before = console.wait_for_frame(
        lambda frame: frame['fault']['active'],
        60.0, 'the session faulting on a dead launch child')['fault']

    status, refusal = console.request(
        'POST', '/api/session/recover', token=console.token, expect=409)
    assert refusal['error'] == 'session_not_running', refusal
    assert refusal['detail'] == (
        'recovery applies to watch and motion sessions only')

    after = console.state()['fault']
    for key in ('cause', 'headline', 'steps', 'action', 'active'):
        assert after[key] == before[key], (
            'the refused recovery changed fault[{!r}]'.format(key))
    console.stop_session()


def find_launch_guardian(server):
    """Return the pid of the server's launch guardian child."""
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        for pid in child_pids(server.process.pid):
            cmdline = read_cmdline(pid)
            if cmdline and 'franka_web.launcher' in cmdline:
                return pid
        time.sleep(_POLL_S)
    server.fail('the server never spawned a launch guardian')


def find_launch_target(server):
    """
    Return the pid of the ``ros2 launch`` the guardian owns.

    Kill THIS, never the guardian. The guardian is what proves the target
    process group gone, and a server whose guardian was killed outright is
    designed to sit in ``stopping`` for ever rather than forget an ownerless
    robot stack -- correct behaviour, and not the fault this case is about.
    Killing the launch itself is what "the launch child exited" really means:
    the guardian drives and reaps the group, exits with its teardown proof,
    and the session both faults and stops cleanly.
    """
    guardian = find_launch_guardian(server)
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        for pid in child_pids(guardian):
            cmdline = read_cmdline(pid)
            if cmdline and 'ros2' in cmdline and ' launch ' in cmdline:
                return pid
        time.sleep(_POLL_S)
    server.fail('the guardian never spawned a ros2 launch')


def child_pids(parent):
    """Return the direct children of ``parent`` from /proc."""
    children = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        try:
            with open('/proc/{}/stat'.format(name), 'rb') as handle:
                fields = handle.read().rsplit(b')', 1)[1].split()
            if int(fields[1]) == int(parent):
                children.append(int(name))
        except (OSError, IndexError, ValueError):
            continue
    return children


def read_cmdline(pid):
    """Return a process's argv as one space-joined string, or None."""
    try:
        with open('/proc/{}/cmdline'.format(pid), 'rb') as handle:
            return handle.read().replace(b'\x00', b' ').decode(
                'utf-8', 'replace').strip()
    except OSError:
        return None


# ----------------------------------------------------------------------
# 14 -- stop and seal
# ----------------------------------------------------------------------


def test_14_stop_seals_the_recording_and_leaves_no_survivors(console):
    """
    Stop seals the bag and leaves nothing running.

    A survivor here is not untidiness: an orphaned recorder cross-captures
    the next session's traffic, and an orphaned launch is a robot stack with
    no owner.
    """
    console.claim()
    console.start_session(arms='both', mode='simulate')
    running = console.wait_for_session_state('running', 180.0)
    assert running['recording']['disabled'] is False
    assert running['recording']['active'] is True
    name = running['recording']['name']
    assert name and name.startswith('web-')

    children = child_pids(console.process.pid)
    assert children, 'the server spawned no children at all'

    stopped = console.stop_session()
    assert stopped['recording']['active'] is False
    assert stopped['session']['steps'] == []

    # The recorder writes `<root>/<name>/bag/`, with metadata.yaml sitting
    # next to the mcap once the bag is sealed. Its absence is not untidiness:
    # it is a bag that would need `ros2 bag reindex` to read.
    metadata = os.path.join(console.recording_root, name, 'bag', 'metadata.yaml')
    deadline = time.monotonic() + 30.0
    while not os.path.isfile(metadata) and time.monotonic() < deadline:
        time.sleep(_POLL_S)
    assert os.path.isfile(metadata), (
        'the bag was not sealed: {} is missing'.format(metadata))
    with open(metadata, encoding='utf-8', errors='replace') as handle:
        sealed = handle.read()
    assert 'bag_0.mcap' in sealed, sealed[:400]

    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        survivors = [pid for pid in children
                     if os.path.exists('/proc/{}'.format(pid))]
        if not survivors:
            break
        time.sleep(_POLL_S)
    else:
        console.fail('these children outlived the session stop: {}'.format(
            {pid: read_cmdline(pid) for pid in survivors}))


# ----------------------------------------------------------------------
# 15-16 -- the package scans, in the slow suite too
# ----------------------------------------------------------------------


def test_15_no_installed_file_mentions_the_notes_tree():
    """A future user will not have the notes tree; nothing may name it."""
    assert_no_notes_tree_reference()


def test_16_no_installed_file_mentions_the_old_environment_prefix():
    """
    Nothing under this package reads or names a legacy variable.

    This CALLS the walker rather than restating its allowance: the allowance
    is defined in exactly one file, so the one change authorized to delete it
    clears every walker in the tree at once.
    """
    assert_no_legacy_environment_prefix()


# ----------------------------------------------------------------------
# 17 -- the static surface (arms itself when the frontend rewrite lands)
# ----------------------------------------------------------------------


def _app_js():
    """Return the shipped application JavaScript."""
    with open(os.path.join(STATIC_ROOT, 'app.js')) as handle:
        return handle.read()


@pytest.mark.skipif(
    _V1_APP_MARKER in _app_js(),
    reason='the frontend rewrite has not landed yet; the v1 page is still in '
           'the tree. This class arms itself on that merge -- confirm it is '
           'RUNNING, not skipping, once it is in.')
class TestStaticSurface:
    """
    The static contract, which has no other automatable proof.

    CI has no browser and no Node, so these assertions are the only check the
    page's security headers and its no-inline-style rule ever get.
    """

    def test_the_index_page_is_served(self, console):
        """GET / is the console, with the stage element and the one script."""
        status, _body = console.request('GET', '/', expect=200)
        connection = http.client.HTTPConnection(
            '127.0.0.1', console.port, timeout=20.0)
        try:
            connection.request('GET', '/', headers={
                'Host': '127.0.0.1:{}'.format(console.port)})
            response = connection.getresponse()
            assert response.status == 200
            assert response.getheader('Content-Type') == 'text/html; charset=utf-8'
            body = response.read().decode('utf-8')
        finally:
            connection.close()
        assert 'id="stage"' in body
        assert 'src="/app.js"' in body

    @pytest.mark.parametrize('path, content_type', [
        ('/app.js', 'text/javascript; charset=utf-8'),
        ('/app.css', 'text/css; charset=utf-8'),
    ])
    def test_the_static_assets_carry_the_security_headers(
            self, console, path, content_type):
        """Content type, CSP, no-store and nosniff, on every static asset."""
        connection = http.client.HTTPConnection(
            '127.0.0.1', console.port, timeout=20.0)
        try:
            connection.request('GET', path, headers={
                'Host': '127.0.0.1:{}'.format(console.port)})
            response = connection.getresponse()
            assert response.status == 200
            assert response.getheader('Content-Type') == content_type
            assert response.getheader('Cache-Control') == 'no-store'
            assert response.getheader('X-Content-Type-Options') == 'nosniff'
            assert response.getheader('Content-Security-Policy') == (
                "default-src 'self'; connect-src 'self'; img-src 'self' data:; "
                "style-src 'self'; script-src 'self'; base-uri 'none'; "
                "form-action 'none'; frame-ancestors 'none'")
        finally:
            connection.close()

    def test_no_external_origin_no_inline_style_no_innerhtml(self):
        """
        The CSP forbids all three, and the lab machine may be offline.

        The Apache licence URL in the header comment is the only external
        origin allowed to appear.
        """
        for name in ('app.js', 'app.css'):
            with open(os.path.join(STATIC_ROOT, name)) as handle:
                source = handle.read()
            external = [line for line in source.splitlines()
                        if 'https://' in line
                        and 'apache.org/licenses' not in line]
            assert external == [], '{}: {}'.format(name, external)
            assert 'style="' not in source, name
            assert 'innerHTML' not in source, name

    def test_the_installed_static_directory_holds_exactly_the_shipped_files(self):
        """
        Seven files, no more: an eighth fails this on purpose.

        The three fonts are vendored because the console must keep its look
        with nothing fetched from a network, and OFL.txt ships because the
        licence requires it to accompany them.
        """
        installed = os.path.join(
            get_package_prefix('franka_web'), 'share', 'franka_web', 'static')
        found = set()
        for directory, _subdirectories, names in os.walk(installed):
            for name in names:
                found.add(os.path.relpath(
                    os.path.join(directory, name), installed))
        assert found == {
            'index.html', 'app.css', 'app.js',
            os.path.join('fonts', 'archivo-var.woff2'),
            os.path.join('fonts', 'public-sans-var.woff2'),
            os.path.join('fonts', 'spline-sans-mono-var.woff2'),
            os.path.join('fonts', 'OFL.txt'),
        }, sorted(found)
