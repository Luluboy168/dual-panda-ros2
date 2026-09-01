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

r"""
End-to-end: the real server beside STANDING fake gripper nodes.

It starts the INSTALLED ``franka_web_server`` as a subprocess and drives it
with ``http.client`` only -- the same discipline as
``e2e_fake_dual_state_only_test`` -- and it starts the gripper nodes itself::

    ros2 launch franka_robotiq dual_robotiq.launch.py use_fake:=true \\
         fake_object_mm:=30.0

The nodes are started and stopped BY THIS TEST, never by the server, which is
exactly how it runs in production minus the hardware.

TWO HALVES, AND THE SECOND IS HONESTLY GATED
    **Half 1** runs at this part's merge and needs no session at all:
    capabilities, the config block, the no-session refusal, the operator-token
    requirement, the two-package constant check, and the positive proof that
    Simulate renders no gripper row.

    **Half 2** -- the row filling, updating and refusing -- needs a **Watch**
    session, and a Watch session is a production profile: every watch row in
    ``profiles.py`` carries ``requires_addresses=True``, so it launches
    against real robot IPs. The fake e2e harness can only ever start Simulate,
    and Simulate has no gripper row by decision. So half 2 skips with its
    reason named until bring-up day, where the mounting runbook's step 8.12
    runs it against the real cell.

Domain isolation
    This file publishes onto a real ROS graph and therefore runs on the ID
    reserved for it, ``ROS_DOMAIN_ID=227``, and SKIPS itself when the
    environment does not say so. Registering 227 in
    ``franka_web/CMakeLists.txt``'s ``ENV`` block is a handoff to a session
    permitted to edit that file; until then this file is run by explicit path
    with the isolation that block would have provided.
"""

import http.client
import importlib.util
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from franka_web import defaults
import jsonschema
import pytest
import yaml

#: The ID this file reserves (see the allocation table in
#: franka_robotiq/test/conftest.py; 191-218 are taken by franka_bringup,
#: franka_web holds 219/220/224, franka_ik 221/222, franka_ghost 223).
REQUIRED_DOMAIN_ID = '227'

_DOMAIN_SKIP = (
    'the gripper-row e2e brings up a real ROS graph and must stay on its '
    'reserved domain: run it through the CMake registration once domain 227 '
    'lands there, or export ROS_DOMAIN_ID={} (plus '
    'FASTDDS_BUILTIN_TRANSPORTS=SHM and '
    'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST) yourself'.format(
        REQUIRED_DOMAIN_ID))

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != REQUIRED_DOMAIN_ID, reason=_DOMAIN_SKIP)

#: Half 2's reason, named rather than silent.
HALF_TWO_SKIP = (
    'a watch session reaches a real robot; the merge gate runs half 1 and '
    'MOUNTING.md step 8.12 runs this half at bring-up')

#: The environment variable a real cell sets to let half 2 run.
REAL_CELL_ENV = 'FRANKA_ROBOTIQ_REAL_CELL'

STATIC_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')

SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'support', 'state_frame_schema.json')

#: RFC 5737 documentation addresses. A Simulate session never uses them.
DOC_IP_1 = '192.0.2.11'
DOC_IP_2 = '192.0.2.12'

#: A binding that satisfies the basename-syntax rule without naming a real
#: adapter. Nothing resolves it until a Watch or Motion session opens the
#: device, which is exactly how the row is demonstrated before the adapters
#: land.
PLACEHOLDER_SERIAL = {'panda1': 'usb-PLACEHOLDER_ADAPTER_0001-if00-port0',
                      'panda2': 'usb-PLACEHOLDER_ADAPTER_0002-if00-port0'}

_POLL_S = 0.2


def driver_modules_present():
    """Return whether franka_robotiq's ROS-free modules are importable."""
    for name in ('units', 'discovery', 'driver', 'fake'):
        try:
            if importlib.util.find_spec('franka_robotiq.{}'.format(name)) is None:
                return False
        except (ImportError, ValueError):
            return False
    return True


DRIVER_SKIP = (
    'franka_robotiq is not importable in this workspace, so the config '
    'validator refuses an enabled gripper and no standing node can be '
    'started; every case below runs as soon as that package is present')

needs_driver = pytest.mark.skipif(not driver_modules_present(), reason=DRIVER_SKIP)


def free_port():
    """Bind port 0 on loopback, release it, and return the number chosen."""
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


def load_validator():
    """Build a validator for the normative state-frame schema."""
    with open(SCHEMA_PATH, encoding='utf-8') as handle:
        schema = json.load(handle)
    validator_class = jsonschema.validators.validator_for(schema)
    validator_class.check_schema(schema)
    return validator_class(schema)


class GripperNodes:
    """The STANDING gripper nodes, started and stopped by this test."""

    def __init__(self, environment):
        """Start both fake-backed nodes with a seeded 30 mm object."""
        self.process = subprocess.Popen(
            ['ros2', 'launch', 'franka_robotiq', 'dual_robotiq.launch.py',
             'use_fake:=true', 'fake_object_mm:=30.0'],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.environment = environment

    def wait_until_up(self, timeout_s=60.0):
        """Poll ``ros2 node list`` until both nodes are discoverable."""
        deadline = time.monotonic() + timeout_s
        seen = ''
        while time.monotonic() < deadline:
            completed = subprocess.run(
                ['ros2', 'node', 'list'], env=self.environment, check=False,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
            seen = completed.stdout.decode('utf-8', 'replace')
            if '/panda1_robotiq' in seen and '/panda2_robotiq' in seen:
                return seen
            time.sleep(_POLL_S)
        raise AssertionError(
            'the standing gripper nodes never appeared; last list:\n{}'.format(seen))

    def stop(self):
        """Stop the launch and reap it; nothing else ever stops these nodes."""
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=10)


class Server:
    """One real ``franka_web_server`` subprocess, driven over HTTP only."""

    def __init__(self, root, *, grippers_enabled=True):
        """Create the private directories, take a free port, and spawn."""
        self.root = root
        self.state_dir = os.path.join(root, 'state')
        self.recording_root = os.path.join(root, 'recordings')
        os.makedirs(self.state_dir, mode=0o700)
        os.makedirs(self.recording_root, mode=0o700)
        self.port = free_port()
        self.log_path = os.path.join(root, 'server.log')
        self.token = None
        self.grippers_enabled = grippers_enabled
        self.config_path = self.write_config()
        self.environment = dict(os.environ)
        self.environment.update({
            'ROS_DOMAIN_ID': REQUIRED_DOMAIN_ID,
            'ROS_HOME': os.path.join(root, 'ros_home'),
            'ROS_LOG_DIR': os.path.join(root, 'ros_log'),
            'HOME': root,
            'XDG_CONFIG_HOME': os.path.join(root, 'xdg'),
            'PYTHONUNBUFFERED': '1',
        })
        self._log = open(self.log_path, 'wb')
        self.process = subprocess.Popen(
            [sys.executable, server_executable(), '--config', self.config_path],
            env=self.environment, stdin=subprocess.DEVNULL,
            stdout=self._log, stderr=subprocess.STDOUT)

    def write_config(self):
        """Write this server's own configuration file and return its path."""
        path = os.path.join(self.root, 'config.yaml')
        document = {
            'bind': '127.0.0.1',
            'port': self.port,
            'ros_domain_id': int(REQUIRED_DOMAIN_ID),
            'robots': {'panda1': {'ip': DOC_IP_1}, 'panda2': {'ip': DOC_IP_2}},
            'directories': {'state': self.state_dir,
                            'recordings': self.recording_root},
        }
        if self.grippers_enabled:
            document['grippers'] = {
                arm_id: {'enabled': True, 'serial_id': PLACEHOLDER_SERIAL[arm_id]}
                for arm_id in ('panda1', 'panda2')}
        with open(path, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(document, handle, default_flow_style=False,
                           sort_keys=True)
        return path

    def log_tail(self, limit=40):
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

    def wait_until_listening(self, timeout_s=45.0):
        """Poll the port until the server accepts a connection."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.fail('the server exited with status {} before it '
                          'listened'.format(self.process.returncode))
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
        finally:
            connection.close()
        try:
            decoded = json.loads(raw.decode('utf-8'))
        except ValueError:
            self.fail('{} {} answered non-JSON: {!r}'.format(method, path, raw[:200]))
        if status != expect:
            self.fail('{} {} answered {} (expected {}): {}'.format(
                method, path, status, expect, decoded))
        return decoded

    def claim(self):
        """Claim the single-operator lock and remember its token."""
        self.token = self.request('POST', '/api/operator/claim')['token']
        return self.token

    def heartbeat(self):
        """Refresh the operator lock."""
        self.request('POST', '/api/operator/heartbeat', token=self.token)

    def state(self):
        """Return the current frame from GET /api/state."""
        return self.request('GET', '/api/state')['state']

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
            time.sleep(0.5)
        self.fail('the session never reached {!r}; states seen: {}'.format(
            wanted, ' -> '.join(seen)))


@pytest.fixture()
def cell(tmp_path):
    """Yield a server with both grippers configured, beside standing nodes."""
    root = str(tmp_path / 'cell')
    os.makedirs(root, mode=0o700)
    server = Server(root)
    nodes = GripperNodes(server.environment)
    try:
        server.wait_until_listening()
        nodes.wait_until_up()
        yield server
    finally:
        nodes.stop()
        server.shutdown()


# ----------------------------------------------------------------------
# Half 1 -- runs at this part's merge, and needs no session
# ----------------------------------------------------------------------


@needs_driver
def test_capabilities_reports_the_gripper_surface_from_the_config_file(cell):
    """The five capability keys come from the file and from defaults."""
    body = cell.request('GET', '/api/capabilities')
    assert body['schema_version'] == 4
    assert body['gripper_arms'] == ['panda1', 'panda2']
    assert body['gripper_actions'] == list(defaults.GRIPPER_ACTIONS)
    assert body['gripper_stroke_mm'] == defaults.GRIPPER_STROKE_MM
    assert body['gripper_force_range_n'] == list(defaults.GRIPPER_FORCE_RANGE_N)
    assert body['gripper_speed_range_mm_s'] == list(
        defaults.GRIPPER_SPEED_RANGE_MM_S)


@needs_driver
def test_config_carries_the_grippers_block_for_enabled_arms_only(cell, tmp_path):
    """The block echoes the FILE's values, labelled as the file's copy."""
    block = cell.request('GET', '/api/config')['grippers']
    assert sorted(block) == ['panda1', 'panda2']
    assert block['panda1']['serial_id'] == PLACEHOLDER_SERIAL['panda1']
    assert block['panda1']['source'] == 'config'

    root = str(tmp_path / 'bare')
    os.makedirs(root, mode=0o700)
    bare = Server(root, grippers_enabled=False)
    try:
        bare.wait_until_listening()
        assert bare.request('GET', '/api/config')['grippers'] == {}
        assert bare.request('GET', '/api/capabilities')['gripper_arms'] == []
    finally:
        bare.shutdown()


@needs_driver
def test_a_gripper_command_with_no_session_is_refused_and_teaches(cell):
    """No session means no gripper to command, and the refusal says which."""
    token = cell.claim()
    body = cell.request('POST', '/api/arm/panda1/gripper', {'action': 'close'},
                        token=token, expect=409)
    assert body['error'] == 'session_not_running'
    assert body['detail']


@needs_driver
def test_a_gripper_command_on_an_unconfigured_cell_names_the_config_key(tmp_path):
    """A cell with nothing enabled answers gripper_not_configured / 404."""
    root = str(tmp_path / 'unconfigured')
    os.makedirs(root, mode=0o700)
    server = Server(root, grippers_enabled=False)
    try:
        server.wait_until_listening()
        token = server.claim()
        server.request('POST', '/api/session/start',
                       {'arms': 'both', 'mode': 'simulate'},
                       token=token, expect=202)
        server.wait_for_session_state('running', 60.0)
        body = server.request('POST', '/api/arm/panda1/gripper',
                              {'action': 'close'}, token=token, expect=404)
        assert body['error'] == 'gripper_not_configured'
        assert 'grippers.panda1.enabled' in body['detail']
    finally:
        server.shutdown()


@needs_driver
def test_the_gripper_endpoint_requires_the_operator_token(cell):
    """A state-changing call without a token never reaches the supervisor."""
    body = cell.request('POST', '/api/arm/panda1/gripper', {'action': 'close'},
                        expect=401)
    assert body['error'] == 'operator_token_invalid'


@needs_driver
def test_the_gripper_constants_agree_between_the_two_packages():
    """
    The ONE check standing between the two copies of the 2F-85 constants.

    franka_web must build and run on a workspace where franka_robotiq was
    never built, so it duplicates the stroke and the two ranges rather than
    importing them. This asserts the copies are identical whenever the driver
    IS present.
    """
    from franka_robotiq import units

    assert units.STROKE_MM == defaults.GRIPPER_STROKE_MM
    assert tuple(units.SPEED_RANGE_MM_S) == tuple(defaults.GRIPPER_SPEED_RANGE_MM_S)
    assert tuple(units.FORCE_RANGE_N) == tuple(defaults.GRIPPER_FORCE_RANGE_N)


@needs_driver
def test_a_simulate_session_renders_no_gripper_row(cell):
    """
    The positive proof that Simulate gets no gripper surface.

    Both grippers are enabled in the config file and both standing nodes are
    up beside the server; every arm's block still reports
    ``configured: false``, and the frame still validates.
    """
    validator = load_validator()
    token = cell.claim()
    cell.request('POST', '/api/session/start', {'arms': 'both', 'mode': 'simulate'},
                 token=token, expect=202)
    frame = cell.wait_for_session_state('running', 60.0)
    problems = sorted(validator.iter_errors(frame), key=str)
    assert not problems, '\n'.join(str(problem) for problem in problems)
    for arm_id in ('panda1', 'panda2'):
        block = frame['arms'][arm_id]['gripper']
        assert block['configured'] is False
        assert block['available'] is False
        assert block['width_mm'] is None
        assert block['status_line']
    body = cell.request('POST', '/api/arm/panda1/gripper', {'action': 'close'},
                        token=token, expect=404)
    assert body['error'] == 'gripper_not_configured'


# ----------------------------------------------------------------------
# Half 2 -- the row itself, and it needs a Watch session
# ----------------------------------------------------------------------


@needs_driver
@pytest.mark.skipif(os.environ.get(REAL_CELL_ENV) != '1', reason=HALF_TWO_SKIP)
def test_the_row_fills_updates_and_refuses_beside_a_watch_session(cell):
    """Both rows fill from the nodes, a close moves them, and every frame validates."""
    validator = load_validator()
    token = cell.claim()
    cell.request('POST', '/api/session/start', {'arms': 'both', 'mode': 'watch'},
                 token=token, expect=202)
    frame = cell.wait_for_session_state('running', 120.0)
    assert not sorted(validator.iter_errors(frame), key=str)
    for arm_id in ('panda1', 'panda2'):
        block = frame['arms'][arm_id]['gripper']
        assert block['configured'] is True
        assert block['available'] is True
        assert isinstance(block['width_mm'], float)
        assert block['status_line']
    cell.request('POST', '/api/arm/panda1/gripper', {'action': 'close'},
                 token=token)
    deadline = time.monotonic() + 30.0
    saw_busy = False
    while time.monotonic() < deadline:
        frame = cell.state()
        assert not sorted(validator.iter_errors(frame), key=str)
        block = frame['arms']['panda1']['gripper']
        saw_busy = saw_busy or block['busy'] is True
        if saw_busy and block['busy'] is False:
            break
        cell.heartbeat()
        time.sleep(0.2)
    assert saw_busy, 'the row never went busy'
    cell.request('POST', '/api/arm/panda1/gripper', {'action': 'open'},
                 token=token)


# ----------------------------------------------------------------------
# The static surface, which needs neither a server nor a node
# ----------------------------------------------------------------------


class TestStaticSurface:
    """What the shipped page must and must not contain for the gripper row."""

    def read(self, name):
        """Return one shipped static file's text."""
        with open(os.path.join(STATIC_ROOT, name), encoding='utf-8') as handle:
            return handle.read()

    def test_the_gripper_row_is_present_and_safe(self):
        """The row exists, is styled, injects no markup, and knows the schema."""
        script = self.read('app.js')
        style = self.read('app.css')
        assert "act: 'gripper'" in script
        assert 'buildGripperRow' in script
        assert 'patchGripper' in script
        assert 'innerHTML' not in script
        assert 'style="' not in script
        assert '.grow{' in style
        assert '.gbtn{' in style
        versions = re.findall(r'schema_version !== (\d+)', script)
        assert versions == ['4'], (
            'app.js must carry exactly one schema_version comparison, and it '
            'must name 4; found {}'.format(versions))
