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
The §6 HTTP surface, driven over a real socket against a faked supervisor.

Unit level, but not a mock of the transport: every case here builds the real
``ThreadingHTTPServer`` from :func:`franka_web.http_api.build_server`, binds
it to a free loopback port, and drives it with ``http.client`` (or a raw
socket, where the request has to be malformed on purpose). Only the
supervisor is a fake -- no ROS graph, no child process, no robot.

The reason the transport is real: every property this module is responsible
for is a property of bytes on the wire. Whether a ``Host`` header is refused,
whether a CORS header is absent, whether ``/../package.xml`` escapes the
static root -- none of those survive being asserted against a hand-built
handler object.

Robot addresses appearing here are RFC 5737 documentation addresses and exist
only so the capabilities test can prove that a configured address never
reaches a response body.
"""

from dataclasses import replace
import http.client
import json
import os
import socket
import threading
import time

from franka_web import config, defaults, faults
from franka_web.http_api import (
    _ERROR_STATUS, _MAX_DRAIN_BYTES, App, build_server, capabilities_payload, ROUTES)
from franka_web.lock import OperatorLock
from franka_web.logbus import LogBus
from franka_web.session import SessionError
from franka_web.sse import Broker
import pytest
from support.config_factory import make_settings
from support.fake_clock import FakeClock

#: RFC 5737 documentation addresses; never a real robot, never routed.
MAX_REQUEST_BYTES = 65536

DOC_IP_1 = '203.0.113.7'
DOC_IP_2 = '203.0.113.8'

#: The §5.7 Content-Security-Policy, spelled out here rather than imported so
#: that a change to the shipped policy has to be made deliberately, twice.
CSP = ("default-src 'self'; connect-src 'self'; img-src 'self' data:; "
       "style-src 'self'; script-src 'self'; base-uri 'none'; "
       "form-action 'none'; frame-ancestors 'none'")

#: Every request in this file is loopback and answered by an in-process
#: server; anything slower than this is a hang, not slowness.
REQUEST_TIMEOUT_S = 10.0

STATIC_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')

#: The consumer-facing frame schema; §6.14's closed set lives in it too, and
#: this file asserts it against the server's own table.
SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'support', 'state_frame_schema.json')

#: How long a raw exchange waits for more bytes before calling the server
#: done. Only reached when the connection is deliberately kept alive.
RAW_IDLE_S = 0.3

#: (SessionError code, expected HTTP status) for every code the Stage 1
#: supervisor can raise out of §6.7 / §6.8.
SESSION_ERROR_STATUS = (
    ('session_already_active', 409),
    ('invalid_arms', 400),
    ('invalid_mode', 400),
    ('robot_addresses_missing', 412),
    ('session_not_active', 409),
)


def minimal_frame():
    """
    Build a §6.11-shaped state frame with every block present and empty.

    Shape, not content: the frame contract itself is covered by the session
    tests. What matters here is that the HTTP layer hands the object through
    unchanged and that it survives JSON and SSE encoding.
    """
    return {
        'schema_version': defaults.SCHEMA_VERSION,
        'server_time': '2026-08-29T00:00:00.000000Z',
        'server_uptime_s': 1.5,
        'session': {
            'state': 'stopped',
            'session_id': None,
            'arms': None,
            'arm_ids': [],
            'arm_mode': None,
            'mode': None,
            'started_at': None,
            'uptime_s': None,
            'launch_running': False,
            'last_error': None,
            'advisory': defaults.STOP_ADVISORY,
            'steps': [],
        },
        'operator': {'locked': False, 'claim_id': None, 'since': None,
                     'expires_in_s': None},
        'preflight': {'ran_at': None, 'overall': None,
                      'blocking': False, 'failed_checks': []},
        'recording': {'active': False, 'disabled': False, 'name': None,
                      'sequence': 0, 'path': None, 'arm_mode': None,
                      'topics': []},
        'controllers': [],
        'hardware': {'available': False, 'name': None, 'plugin_name': None,
                     'lifecycle_id': None, 'lifecycle_label': None},
        'fault': {'active': False, 'since': None, 'reasons': [],
                  'recoverable': False, 'recover_hint': None,
                  'cause': None, 'arm_id': None, 'headline': None,
                  'steps': [], 'action': 'none'},
        'arms': {},
        'hint': 'Pick arms and press Start.',
        'logs': {'warn_count': 0, 'error_count': 0, 'last_seq': 0},
    }


class FakeSupervisor:
    """
    Scriptable stand-in for SessionSupervisor's HTTP-thread surface.

    Only the methods ``http_api`` actually calls exist here; anything else
    the handlers reach for would be a contract change this test should
    notice. ``adopt_operator_claim`` records every claim identity it is
    handed, which is how the claim-adoption seam is asserted.
    """

    def __init__(self):
        """Start out accepting every command and reporting an idle frame."""
        self.start_requests = []
        self.source_requests = []
        self.adopted = []
        self.stop_calls = 0
        self.releases = 0
        self.start_error = None
        self.stop_error = None
        self.gripper_requests = []
        self.gripper_error = None
        self.start_result = {'session_id': 'web-20260829-101500', 'state': 'preflight'}
        self.stop_result = {'state': 'stopping'}
        self.frame_value = minimal_frame()

    def request_start(self, request, operator_lease=None):
        """Record the §6.7 request and answer with the scripted verdict."""
        self.start_requests.append(request)
        if self.start_error is not None:
            raise self.start_error
        return dict(self.start_result)

    def request_stop(self):
        """Count the §6.8 request and answer with the scripted verdict."""
        self.stop_calls += 1
        if self.stop_error is not None:
            raise self.stop_error
        return dict(self.stop_result)

    def revoke_operator_authorization(self):
        """Count the release notification."""
        self.releases += 1

    def adopt_operator_claim(self, claim_id):
        """Record the claim identity a fresh claim/takeover/heartbeat gave."""
        self.adopted.append(claim_id)

    def request_arm_source(self, arm_id, source, operator_lease=None):
        """Record the source switch and answer the way the supervisor does."""
        self.source_requests.append((arm_id, source))
        return {'arm_id': arm_id, 'source': source}

    def request_arm_enable(self, arm_id, enabled, operator_lease=None):
        """Answer an enable so the routing row can dispatch."""
        return {'arm_id': arm_id, 'enabled': enabled, 'target': None,
                'message': 'ok'}

    def request_arm_jog(self, arm_id, joint_index, direction,
                        operator_lease=None):
        """Answer a jog so the routing row can dispatch."""
        return {'arm_id': arm_id, 'target': [0.0] * 7,
                'clamped': [False] * 7}

    def request_session_recover(self, operator_lease=None):
        """Answer a recovery so the routing row can dispatch."""
        return {'arm_ids': [], 'steps': [], 'enabled_after': False}

    def request_gripper_action(self, arm_id, action, width_mm=None,
                               operator_lease=None):
        """Record one gripper command and answer with the scripted verdict."""
        self.gripper_requests.append((arm_id, action, width_mm))
        if self.gripper_error is not None:
            raise self.gripper_error
        return {'arm_id': arm_id, 'action': action, 'width_mm': width_mm}

    def frame(self):
        """Return the scripted §6.11 frame."""
        return self.frame_value


class Response:
    """One completed HTTP response: status, headers and raw body."""

    def __init__(self, status, headers, body):
        """Wrap the status line, the header object, and the body bytes."""
        self.status = status
        self.headers = headers
        self.body = body

    def header(self, name):
        """Return one header value (case-insensitive), or None."""
        return self.headers.get(name)

    def header_names(self):
        """Return every header name, lowercased."""
        return [name.lower() for name in self.headers.keys()]

    def json(self):
        """Decode the body as JSON."""
        return json.loads(self.body.decode('utf-8'))


class Stream:
    """An open ``text/event-stream`` response, read line by line."""

    def __init__(self, status, headers, reader, sock):
        """Hold the parsed head plus the still-open socket and reader."""
        self.status = status
        self.headers = headers
        self._reader = reader
        self._socket = sock

    def read_event(self):
        """Read one ``event:``/``data:``/blank triple and return it."""
        event = self._reader.readline().decode('utf-8').rstrip('\n')
        data = self._reader.readline().decode('utf-8').rstrip('\n')
        blank = self._reader.readline().decode('utf-8')
        assert blank == '\n', 'frame not terminated by a blank line: {!r}'.format(blank)
        return event, data

    def close(self):
        """Drop the connection; the server handler unwinds on its next write."""
        try:
            self._reader.close()
        finally:
            self._socket.close()


def free_port():
    """Return a loopback port that was free a moment ago."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class Server:
    """
    A real HTTP server on a free loopback port, wired to fakes.

    The operator lock runs on a :class:`FakeClock`, so token expiry is driven
    explicitly by :meth:`advance` and never by wall-clock luck.
    """

    def __init__(self, tmp_path, supervisor=None, grippers=None):
        """Bind, wire and start serving on a daemon thread."""
        # Applied to the loaded Settings rather than written into the config
        # file: enabling a gripper through the loader would reach the guarded
        # franka_robotiq import, and this file is about the HTTP surface.
        self._grippers = grippers
        state_dir = tmp_path / 'state'
        state_dir.mkdir(mode=0o700, exist_ok=True)
        recording_root = tmp_path / 'recordings'
        recording_root.mkdir(mode=0o700, exist_ok=True)
        self.root = tmp_path
        self.state_dir = str(state_dir)
        self.recording_root = str(recording_root)
        self.clock = FakeClock()
        self.logs = LogBus()
        self.supervisor = supervisor or FakeSupervisor()
        self.lock = OperatorLock(monotonic=self.clock.monotonic)
        # Production SessionSupervisor registers this hook at construction.
        # The transport fake mirrors that wiring explicitly.
        self.lock.set_revocation_hook(
            self.supervisor.revoke_operator_authorization)
        self.broker = Broker()
        self.settings = None
        self.httpd = None
        self._streams = []
        for _ in range(10):
            settings = self._settings_for(free_port())
            app = App(settings=settings, supervisor=self.supervisor,
                      lock=self.lock, broker=self.broker,
                      static_root=STATIC_ROOT, log_bus=self.logs)
            try:
                self.httpd = build_server(app)
            except OSError:
                continue        # the probe port was taken in the meantime
            self.settings = settings
            break
        assert self.httpd is not None, 'could not bind a free loopback port'
        # A short poll interval only matters at teardown: shutdown() waits for
        # the accept loop's next tick, and the default 0.5 s would dominate the
        # runtime of a file with a server per test.
        self.thread = threading.Thread(
            target=self.httpd.serve_forever, args=(0.02,),
            name='test-http', daemon=True)
        self.thread.start()

    def _settings_for(self, port):
        """Build Settings from a real configuration file on this port."""
        settings = make_settings(
            self.root, bind='127.0.0.1', port=port,
            robot_ips={'panda1': DOC_IP_1, 'panda2': DOC_IP_2})
        if self._grippers:
            settings = replace(settings, grippers=self._grippers)
        return settings

    @property
    def port(self):
        """Return the bound port."""
        return self.settings.port

    def request(self, method, path, body=None, headers=None, host=None,
                read_body=True, timeout=REQUEST_TIMEOUT_S):
        """
        Send one request and return the :class:`Response`.

        With ``host`` given the request is assembled by hand so an arbitrary
        (or wrong) ``Host`` header can be sent; otherwise ``http.client``
        supplies ``Host: 127.0.0.1:<port>`` from the connection address.
        ``path`` is written to the wire verbatim -- ``http.client`` never
        normalizes it -- which is what makes the traversal cases meaningful.
        """
        headers = dict(headers or {})
        if isinstance(body, str):
            body = body.encode('utf-8')
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=timeout)
        try:
            if host is None:
                connection.request(method, path, body=body, headers=headers)
            else:
                connection.putrequest(method, path, skip_host=True,
                                      skip_accept_encoding=True)
                connection.putheader('Host', host)
                if body is not None and not any(
                        name.lower() == 'content-length' for name in headers):
                    connection.putheader('Content-Length', str(len(body)))
                for name, value in headers.items():
                    connection.putheader(name, value)
                connection.endheaders(body)
            response = connection.getresponse()
            payload = response.read() if read_body else b''
            if not read_body:
                response.close()
            return Response(response.status, response.headers, payload)
        finally:
            connection.close()

    def open_stream(self, path='/api/state/stream', timeout=REQUEST_TIMEOUT_S):
        """Open an SSE connection by hand and return the parsed head."""
        sock = socket.create_connection(('127.0.0.1', self.port), timeout=timeout)
        self._streams.append(sock)
        request = ('GET {} HTTP/1.1\r\nHost: 127.0.0.1:{}\r\n'
                   'Accept: text/event-stream\r\n\r\n').format(path, self.port)
        sock.sendall(request.encode('ascii'))
        reader = sock.makefile('rb')
        status_line = reader.readline().decode('latin-1').strip()
        headers = {}
        while True:
            line = reader.readline().decode('latin-1').strip()
            if not line:
                break
            name, _, value = line.partition(':')
            headers[name.strip().lower()] = value.strip()
        status = int(status_line.split()[1])
        return Stream(status, headers, reader, sock)

    def claim(self):
        """Claim the operator lock over HTTP and return the token."""
        response = self.request('POST', '/api/operator/claim')
        assert response.status == 200, response.body
        return response.json()['token']

    def claim_or_take(self):
        """Claim the lock, taking it over when somebody else holds it."""
        response = self.request('POST', '/api/operator/claim')
        if response.status == 409:
            response = self.request('POST', '/api/operator/takeover')
        assert response.status == 200, response.body
        return response.json()['token']

    def raw_exchange(self, request_bytes, idle_s=RAW_IDLE_S,
                     timeout=REQUEST_TIMEOUT_S):
        """
        Write bytes verbatim on one connection and read everything back.

        Returns ``(payload, closed)``: every byte the server sent, and
        whether it ended the connection. ``http.client`` cannot express these
        cases -- it reads exactly one response and would hide a second one --
        and how many responses one request draws is the whole question for
        the unread-body cases (finding F-1).

        A kept-alive connection is detected by the read going quiet for
        ``idle_s``; a closed one returns as soon as the peer sends EOF.
        """
        sock = socket.create_connection(('127.0.0.1', self.port), timeout=timeout)
        try:
            sock.sendall(request_bytes)
            sock.settimeout(idle_s)
            chunks = []
            closed = False
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    chunk = sock.recv(65536)
                except (TimeoutError, socket.timeout):
                    break
                if not chunk:
                    closed = True
                    break
                chunks.append(chunk)
            return b''.join(chunks), closed
        finally:
            sock.close()

    def flush_streams(self):
        """
        Publish frames so every stream handler notices its client is gone.

        The handler only discovers a closed socket when it writes, and it
        writes only when the broker publishes -- so this is how a stream
        thread is retired. Returns immediately when nobody is subscribed.
        """
        deadline = time.monotonic() + 5.0
        while self.broker.subscriber_count and time.monotonic() < deadline:
            self.broker.publish('state', minimal_frame())
            time.sleep(0.02)

    def close(self):
        """Stop serving and retire every handler thread."""
        for sock in self._streams:
            try:
                sock.close()
            except OSError:
                pass
        self.flush_streams()
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=5.0)


@pytest.fixture()
def server(tmp_path):
    """Serve one faked-supervisor app for the duration of a test."""
    running = Server(tmp_path)
    yield running
    running.close()


def assert_no_cors(response):
    """
    Assert the response carries no CORS header of any kind.

    The non-empty check is not decoration: if header parsing ever returned
    nothing this assertion would pass for the wrong reason, and the single
    most important negative property in this file would go untested.
    """
    names = response.header_names()
    assert names, 'no headers parsed at all; the CORS check would be vacuous'
    offenders = [name for name in names if name.startswith('access-control-')]
    assert offenders == [], 'CORS header emitted: {}'.format(offenders)


def assert_security_headers(response):
    """Assert the always-on §5.7 response headers."""
    assert response.header('Cache-Control') == 'no-store'
    assert response.header('X-Content-Type-Options') == 'nosniff'
    assert response.header('Content-Security-Policy') == CSP


def assert_error_envelope(response, code, status):
    """Assert the §6.0 failure envelope, exactly three keys and nothing else."""
    assert response.status == status, response.body
    body = response.json()
    assert set(body) == {'ok', 'error', 'detail'}
    assert body['ok'] is False
    assert body['error'] == code
    assert isinstance(body['detail'], str) and body['detail']
    assert_security_headers(response)
    assert_no_cors(response)
    return body


def body_for(route):
    """Return a request body that lets ``route`` reach its handler."""
    if route.path == '/api/session/start':
        return json.dumps({'arms': 'both', 'mode': 'simulate'})
    if route.path.endswith('/enable'):
        return json.dumps({'enabled': True})
    if route.path.endswith('/jog'):
        return json.dumps({'joint_index': 0, 'direction': 1})
    if route.path.endswith('/source'):
        return json.dumps({'source': 'jog'})
    return None


class TestRouting:
    """Every ROUTES row dispatches; nothing else does."""

    def test_every_route_dispatches(self, server):
        """
        Each routing-table row reaches its handler.

        Table-driven so a new row is exercised the moment it is added: the
        assertion is only that the request was neither unrouted (404) nor
        method-mismatched (405).
        """
        token = server.claim()
        for route in ROUTES:
            streaming = route.path == '/api/state/stream'
            path = route.path.replace('{arm_id}', 'panda1')
            response = server.request(
                route.method, path, body=body_for(route),
                headers={'X-Operator-Token': token}, read_body=not streaming)
            assert response.status not in (404, 405), (
                '{} {} did not dispatch'.format(route.method, path))
            if not streaming and response.status >= 400:
                assert response.json()['error'] not in ('not_found', 'method_not_allowed')
            if route.path in ('/api/operator/release', '/api/operator/takeover'):
                # Both hand the lock to somebody else -- release drops it and
                # takeover mints a successor -- so the loop reclaims.
                token = server.claim_or_take()
        server.flush_streams()

    def test_route_success_statuses(self, server):
        """The §6 success statuses: 202 for the session commands, 200 elsewhere."""
        token = server.claim()
        headers = {'X-Operator-Token': token}
        start = server.request('POST', '/api/session/start',
                               body=json.dumps({'arms': 'both', 'mode': 'simulate'}),
                               headers=headers)
        assert start.status == 202
        assert start.json() == {'ok': True, 'session_id': 'web-20260829-101500',
                                'state': 'preflight'}
        stop = server.request('POST', '/api/session/stop', headers=headers)
        assert stop.status == 202
        assert stop.json() == {'ok': True, 'state': 'stopping',
                               'advisory': defaults.STOP_ADVISORY}

    def test_session_start_ignores_extra_keys(self, server):
        """
        The body is ``{arms, mode}`` only, and a stale page is not an error.

        A page that still posts the deleted controller and gains fields must
        start a session, not receive a refusal.
        """
        token = server.claim()
        response = server.request(
            'POST', '/api/session/start',
            body=json.dumps({'arms': 'panda1', 'mode': 'watch',
                             'controller_name': 'dual_arm_joint_hold_controller',
                             'gains_sha256': 'abc'}),
            headers={'X-Operator-Token': token})
        assert response.status == 202
        request = server.supervisor.start_requests[-1]
        assert request.arms == 'panda1'
        assert request.mode == 'watch'
        assert not hasattr(request, 'gains_sha256')
        assert not hasattr(request, 'controller_name')

    def test_the_gains_routes_are_gone(self, server):
        """There is no upload surface, so there is no endpoint either."""
        token = server.claim()
        for method in ('GET', 'POST'):
            response = server.request(
                method, '/api/gains', headers={'X-Operator-Token': token})
            assert_error_envelope(response, 'not_found', 404)

    def test_unknown_path_is_not_found(self, server):
        """An unrouted API path is a 404 not_found envelope."""
        response = server.request('GET', '/api/nonexistent')
        assert_error_envelope(response, 'not_found', 404)

    def test_unknown_post_path_is_not_found(self, server):
        """A POST to a path no row owns is 404, never a static read."""
        response = server.request('POST', '/definitely/not/here')
        assert_error_envelope(response, 'not_found', 404)

    def test_wrong_method_is_method_not_allowed(self, server):
        """A known path with the wrong verb is 405, both directions."""
        posted = server.request('POST', '/api/capabilities')
        assert_error_envelope(posted, 'method_not_allowed', 405)
        got = server.request('GET', '/api/session/start')
        assert_error_envelope(got, 'method_not_allowed', 405)

    def test_root_serves_index_html(self, server):
        """GET / is index.html with the §5.7 headers and no sniffing."""
        response = server.request('GET', '/')
        assert response.status == 200
        assert response.header('Content-Type') == 'text/html; charset=utf-8'
        assert b'<!DOCTYPE html>' in response.body
        assert_security_headers(response)
        assert_no_cors(response)


class TestOriginGuard:
    """
    The same-origin guard, with the Host allowlist DELETED.

    The server binds the lab network by design so the console can be opened
    from a laptop or a tablet, which means an arbitrary Host header is
    ordinary rather than suspicious. What remains, and is the whole guard, is
    that a present ``Origin`` must equal the request's own ``Host``, and a
    present ``Sec-Fetch-Site`` must be ``same-origin`` or ``none``.
    """

    def test_default_host_accepted(self, server):
        """The IP host http.client sends by default is accepted."""
        assert server.request('GET', '/api/capabilities').status == 200

    def test_localhost_host_accepted(self, server):
        """``localhost:<port>`` is accepted too."""
        response = server.request('GET', '/api/capabilities',
                                  host='localhost:{}'.format(server.port))
        assert response.status == 200

    @pytest.mark.parametrize('host', [
        'lab-nuc.local:8765',
        '192.168.1.34:8765',
        'workshop-pc:8765',
        '[::1]:8765',
    ])
    def test_a_lan_host_header_is_accepted(self, server, host):
        """
        A host that is not loopback is exactly the daily journey.

        Refusing these was v1's posture and it made "open the page from
        another device on the lab network" impossible.
        """
        response = server.request('GET', '/api/capabilities', host=host)
        assert response.status == 200

    @pytest.mark.parametrize('scheme_host', ['127.0.0.1', 'localhost'])
    def test_an_origin_matching_the_host_is_accepted(self, server, scheme_host):
        """The page's own requests carry an Origin equal to their Host."""
        response = server.request(
            'GET', '/api/capabilities',
            host='{}:{}'.format(scheme_host, server.port),
            headers={'Origin': 'http://{}:{}'.format(scheme_host, server.port)})
        assert response.status == 200

    def test_a_cross_origin_post_is_refused(self, server):
        """A drive-by POST from another tab is what this guard is for."""
        token = server.claim()
        response = server.request(
            'POST', '/api/session/start',
            body=json.dumps({'arms': 'both', 'mode': 'simulate'}),
            headers={'X-Operator-Token': token,
                     'Origin': 'http://evil.example'})
        assert_error_envelope(response, 'forbidden_origin', 403)
        assert server.supervisor.start_requests == []

    def test_cross_origin_variants_refused(self, server):
        """A different scheme, port or host in Origin is 403."""
        bad_origins = [
            'https://127.0.0.1:{}'.format(server.port),
            'http://127.0.0.1:{}'.format(server.port + 1),
            'http://localhost:{}'.format(server.port + 1),
            'http://evil.example',
            'http://evil.example:{}'.format(server.port),
            'null',
            'http://127.0.0.1:{}/'.format(server.port),
        ]
        for origin in bad_origins:
            response = server.request('GET', '/api/capabilities',
                                      headers={'Origin': origin})
            assert_error_envelope(response, 'forbidden_origin', 403)

    def test_https_is_not_a_second_accepted_scheme(self, server):
        """
        The comparison is against ``http://<Host>`` only.

        There is no reverse proxy in this deployment, and a second accepted
        scheme would be a second thing to get wrong.
        """
        response = server.request(
            'GET', '/api/capabilities',
            headers={'Origin': 'https://127.0.0.1:{}'.format(server.port)})
        assert_error_envelope(response, 'forbidden_origin', 403)

    @pytest.mark.parametrize('site', ['same-origin', 'none'])
    def test_a_same_origin_fetch_metadata_header_is_accepted(self, server, site):
        """Sec-Fetch-Site same-origin and none are the page's own requests."""
        response = server.request('GET', '/api/capabilities',
                                  headers={'Sec-Fetch-Site': site})
        assert response.status == 200

    @pytest.mark.parametrize('site', ['cross-site', 'same-site'])
    def test_a_cross_site_fetch_metadata_header_is_refused(self, server, site):
        """Anything else -- including same-site -- is another document."""
        response = server.request('GET', '/api/capabilities',
                                  headers={'Sec-Fetch-Site': site})
        assert_error_envelope(response, 'forbidden_origin', 403)

    def test_guard_runs_before_routing_and_before_the_token(self, server):
        """A cross-origin request is refused before any handler is reached."""
        token = server.claim()
        response = server.request(
            'POST', '/api/session/start',
            body=json.dumps({'arms': 'both', 'mode': 'simulate'}),
            headers={'X-Operator-Token': token, 'Origin': 'http://evil.example'})
        assert_error_envelope(response, 'forbidden_origin', 403)
        assert server.supervisor.start_requests == []

    def test_guard_runs_before_static(self, server):
        """A cross-origin request cannot read a static file either."""
        response = server.request('GET', '/app.js',
                                  headers={'Origin': 'http://evil.example'})
        assert_error_envelope(response, 'forbidden_origin', 403)


class TestNoCors:
    """No Access-Control-* header is ever emitted, on any response."""

    def test_success_responses_carry_no_cors(self, server):
        """Every §6 success body, static included, is CORS-free."""
        token = server.claim()
        responses = [
            server.request('GET', '/'),
            server.request('GET', '/app.js'),
            server.request('GET', '/api/capabilities'),
            server.request('GET', '/api/config'),
            server.request('GET', '/api/state'),
            server.request('POST', '/api/session/stop',
                           headers={'X-Operator-Token': token}),
        ]
        for response in responses:
            assert response.status < 400, response.body
            assert_no_cors(response)

    def test_error_responses_carry_no_cors(self, server):
        """Every refusal -- 400/401/403/404/405/409/413 -- is CORS-free."""
        server.supervisor.stop_error = SessionError('session_not_active', 'nothing runs')
        token = server.claim()
        responses = [
            server.request('GET', '/api/capabilities',
                           headers={'Origin': 'http://evil.example'}),
            server.request('GET', '/api/capabilities',
                           headers={'Sec-Fetch-Site': 'cross-site'}),
            server.request('POST', '/api/session/stop'),
            server.request('GET', '/api/nope'),
            server.request('POST', '/api/capabilities'),
            server.request('POST', '/api/session/stop',
                           headers={'X-Operator-Token': token}),
            server.request('POST', '/api/session/start', body='{',
                           headers={'X-Operator-Token': token}),
            server.request('POST', '/api/session/start', body='{}',
                           headers={'X-Operator-Token': token,
                                    'Content-Length': '70000',
                                    'Connection': 'close'}),
        ]
        statuses = {response.status for response in responses}
        assert statuses == {403, 401, 404, 405, 409, 400, 413}
        for response in responses:
            assert_no_cors(response)

    def test_preflight_options_gets_no_cors(self, server):
        """
        A CORS preflight is answered without a single allow header.

        The custom ``X-Operator-Token`` header forces a preflight on any
        cross-origin attempt; whatever the server chooses to say to OPTIONS,
        it must never say ``Access-Control-Allow-*``.
        """
        response = server.request(
            'OPTIONS', '/api/session/start',
            headers={'Origin': 'http://evil.example',
                     'Access-Control-Request-Method': 'POST',
                     'Access-Control-Request-Headers': 'x-operator-token'})
        assert response.status >= 400
        assert_no_cors(response)

    def test_stream_carries_no_cors(self, server):
        """The SSE response head is CORS-free too."""
        stream = server.open_stream()
        assert stream.status == 200
        assert stream.headers, 'no headers parsed; the check would be vacuous'
        assert not any(name.startswith('access-control-') for name in stream.headers)
        stream.close()
        server.flush_streams()


class TestTokenEnforcement:
    """Table-driven over ROUTES so a new route cannot forget the token."""

    @pytest.mark.parametrize('route', [r for r in ROUTES if r.needs_token],
                             ids=lambda r: '{} {}'.format(r.method, r.path))
    def test_token_required_route_refuses_without_token(self, server, route):
        """Every needs_token row is 401 with no X-Operator-Token at all."""
        response = server.request(route.method, route.path, body=body_for(route))
        assert_error_envelope(response, 'operator_token_invalid', 401)

    @pytest.mark.parametrize('route', [r for r in ROUTES if r.needs_token],
                             ids=lambda r: '{} {}'.format(r.method, r.path))
    def test_token_required_route_refuses_wrong_token(self, server, route):
        """A held lock does not make somebody else's token work."""
        server.claim()
        for wrong in ('', 'not-the-token', 'x' * 64):
            response = server.request(route.method, route.path, body=body_for(route),
                                      headers={'X-Operator-Token': wrong})
            assert_error_envelope(response, 'operator_token_invalid', 401)

    @pytest.mark.parametrize('route', [r for r in ROUTES if not r.needs_token],
                             ids=lambda r: '{} {}'.format(r.method, r.path))
    def test_open_route_never_asks_for_a_token(self, server, route):
        """A needs_token=False row answers on its own terms, never 401."""
        streaming = route.path == '/api/state/stream'
        response = server.request(route.method, route.path, body=body_for(route),
                                  read_body=not streaming)
        assert response.status != 401
        if not streaming and response.status >= 400:
            assert response.json()['error'] != 'operator_token_invalid'
        server.flush_streams()

    def test_token_refused_before_the_supervisor_is_touched(self, server):
        """A missing token stops the request short of the session machine."""
        response = server.request('POST', '/api/session/start',
                                  body=json.dumps({'arms': 'both', 'mode': 'simulate'}))
        assert_error_envelope(response, 'operator_token_invalid', 401)
        assert server.supervisor.start_requests == []
        assert server.supervisor.stop_calls == 0

    def test_valid_token_passes_and_touches_the_lock(self, server):
        """
        A successful mutating request refreshes the operator lock (§5.6).

        Claim at t, spend ten of the fifteen TTL seconds, make one token-bearing
        request, then spend ten more: without the touch the token would have
        expired at t+15, and the heartbeat would be refused.
        """
        token = server.claim()
        server.clock.advance(10.0)
        stop = server.request('POST', '/api/session/stop',
                              headers={'X-Operator-Token': token})
        assert stop.status == 202
        server.clock.advance(10.0)
        beat = server.request('POST', '/api/operator/heartbeat',
                              headers={'X-Operator-Token': token})
        assert beat.status == 200
        assert beat.json() == {'ok': True, 'expires_in_s': defaults.OPERATOR_LOCK_TTL_S}

    def test_expired_token_is_refused(self, server):
        """Past the TTL with no refresh, the token is inert."""
        token = server.claim()
        server.clock.advance(defaults.OPERATOR_LOCK_TTL_S + 0.001)
        response = server.request('POST', '/api/session/stop',
                                  headers={'X-Operator-Token': token})
        assert_error_envelope(response, 'operator_token_invalid', 401)


class TestStaticFiles:
    """The static root is a floor, not a suggestion."""

    def test_known_asset_is_served(self, server):
        """/app.js is served with a JavaScript content type."""
        response = server.request('GET', '/app.js')
        assert response.status == 200
        assert response.header('Content-Type') == 'text/javascript; charset=utf-8'
        assert response.body
        assert_security_headers(response)

    def test_stylesheet_is_served(self, server):
        """/app.css is served with a CSS content type."""
        response = server.request('GET', '/app.css')
        assert response.status == 200
        assert response.header('Content-Type') == 'text/css; charset=utf-8'

    @pytest.mark.parametrize('path', [
        '/../package.xml',
        '/../../etc/passwd',
        '/%2e%2e/',
        '/%2e%2e/package.xml',
        '/etc/passwd',
        '/a/../../package.xml',
        '/static/../package.xml',
        '/./app.js',
        '/app.js/../../package.xml',
        '/..%2fpackage.xml',
        '/\\..\\package.xml',
    ])
    def test_traversal_refused(self, server, path):
        """
        No spelling of ``..`` reads a byte from outside the static root.

        The request line is written verbatim by ``http.client``, so the
        handler sees exactly these paths -- the containment check in
        ``_serve_static`` is what has to refuse them, not the client.
        """
        response = server.request('GET', path)
        assert_error_envelope(response, 'not_found', 404)
        assert b'<package' not in response.body
        assert b'root:x:' not in response.body

    def test_package_xml_never_leaks(self, server):
        """The concrete file just outside the root stays unreadable."""
        outside = os.path.join(os.path.dirname(STATIC_ROOT), 'package.xml')
        assert os.path.isfile(outside), 'the traversal target must really exist'
        with open(outside, 'rb') as handle:
            secret = handle.read()
        for path in ('/../package.xml', '/%2e%2e/package.xml', '/a/../../package.xml'):
            response = server.request('GET', path)
            assert response.status == 404
            assert secret not in response.body

    def test_missing_asset_is_not_found(self, server):
        """A plain missing file inside the root is an ordinary 404."""
        response = server.request('GET', '/no-such-asset.js')
        assert_error_envelope(response, 'not_found', 404)

    def test_directory_is_not_served(self, server):
        """A directory inside the root is not a file and is refused."""
        response = server.request('GET', '/.')
        assert_error_envelope(response, 'not_found', 404)


class TestBodyLimits:
    """§6.0 body handling: size cap first, then JSON shape."""

    def test_payload_too_large(self, server):
        """A declared Content-Length over the cap is 413 before any read."""
        token = server.claim()
        response = server.request(
            'POST', '/api/session/start', body='{}',
            headers={'X-Operator-Token': token,
                     'Content-Length': str(MAX_REQUEST_BYTES + 1),
                     'Connection': 'close'})
        assert_error_envelope(response, 'payload_too_large', 413)
        assert server.supervisor.start_requests == []

    def test_at_the_cap_is_not_refused(self, server):
        """Exactly the cap is inside the limit (it is a cap, not a fence)."""
        token = server.claim()
        padding = 'a' * (MAX_REQUEST_BYTES - 100)
        body = json.dumps({'arms': 'both', 'mode': 'simulate', 'pad': padding})
        assert len(body) <= MAX_REQUEST_BYTES
        response = server.request('POST', '/api/session/start', body=body,
                                  headers={'X-Operator-Token': token})
        assert response.status == 202

    @pytest.mark.parametrize('body', [
        'not json at all',
        '{',
        '{"arms": }',
        '{"arms": "both",}',
        b'\xff\xfe not utf-8 either',
    ])
    def test_invalid_json(self, server, body):
        """Garbage in the body is 400 invalid_json, never a 500."""
        token = server.claim()
        response = server.request('POST', '/api/session/start', body=body,
                                  headers={'X-Operator-Token': token})
        assert_error_envelope(response, 'invalid_json', 400)

    @pytest.mark.parametrize('body', ['[1]', '"a string"', '42', 'null', 'true'])
    def test_non_object_json_refused(self, server, body):
        """A valid JSON value that is not an object is still invalid_json."""
        token = server.claim()
        response = server.request('POST', '/api/session/start', body=body,
                                  headers={'X-Operator-Token': token})
        assert_error_envelope(response, 'invalid_json', 400)

    def test_bad_content_length_refused(self, server):
        """A non-numeric Content-Length is invalid_json, not a crash."""
        token = server.claim()
        response = server.request(
            'POST', '/api/session/start', body='{}',
            headers={'X-Operator-Token': token, 'Content-Length': 'twelve',
                     'Connection': 'close'})
        assert response.status in (400, 413)
        assert response.json()['ok'] is False

    def test_absent_body_is_an_empty_object(self, server):
        """No body at all means ``{}``; the supervisor sees empty fields."""
        token = server.claim()
        response = server.request('POST', '/api/session/start',
                                  headers={'X-Operator-Token': token})
        assert response.status == 202
        request = server.supervisor.start_requests[-1]
        assert request.arms is None and request.mode is None


class TestErrorEnvelope:
    """Every §6.14 code the supervisor can raise maps to its §6.7/§6.8 status."""

    @pytest.mark.parametrize('code,status', SESSION_ERROR_STATUS)
    def test_session_error_maps_to_status(self, server, code, status):
        """A scripted SessionError becomes the §6.0 failure envelope."""
        detail = 'refused: {}'.format(code)
        token = server.claim()
        if code == 'session_not_active':
            server.supervisor.stop_error = SessionError(code, detail)
            response = server.request('POST', '/api/session/stop',
                                      headers={'X-Operator-Token': token})
        else:
            server.supervisor.start_error = SessionError(code, detail)
            response = server.request(
                'POST', '/api/session/start',
                body=json.dumps({'arms': 'both', 'mode': 'simulate'}),
                headers={'X-Operator-Token': token})
        body = assert_error_envelope(response, code, status)
        assert body['detail'] == detail
        assert DOC_IP_1 not in body['detail']

    def test_unknown_session_code_becomes_internal_error(self, server):
        """A code outside the closed set never escapes as itself."""
        token = server.claim()
        server.supervisor.start_error = SessionError('teapot_overflow', 'secret detail')
        response = server.request(
            'POST', '/api/session/start',
            body=json.dumps({'arms': 'both', 'mode': 'simulate'}),
            headers={'X-Operator-Token': token})
        body = assert_error_envelope(response, 'internal_error', 500)
        assert 'secret detail' not in body['detail']

    def test_unexpected_exception_becomes_internal_error(self, server):
        """A bug in a collaborator is a 500 envelope, never a traceback body."""
        token = server.claim()
        server.supervisor.start_error = RuntimeError('/home/nuc3/secret/path')
        response = server.request(
            'POST', '/api/session/start',
            body=json.dumps({'arms': 'both', 'mode': 'simulate'}),
            headers={'X-Operator-Token': token})
        body = assert_error_envelope(response, 'internal_error', 500)
        assert '/home/nuc3/secret/path' not in body['detail']
        assert 'Traceback' not in body['detail']

    def test_error_bodies_are_json(self, server):
        """Even the guard refusal is JSON, not the stdlib HTML error page."""
        response = server.request('GET', '/api/capabilities',
                                  headers={'Origin': 'http://evil.example'})
        assert response.header('Content-Type') == 'application/json; charset=utf-8'
        assert response.json()['ok'] is False


class TestOperatorEndpoints:
    """§6.2 - §6.4, against the real OperatorLock."""

    def test_claim_returns_a_claim_id(self, server):
        """
        A free lock mints a token, its short public identity, and the TTL.

        `claim_id` is what lets a page tell "this is my lock" from "someone
        else's" in a state frame, without the stream ever carrying a token --
        EventSource cannot send headers, so the token could never travel
        there safely.
        """
        response = server.request('POST', '/api/operator/claim')
        assert response.status == 200
        body = response.json()
        assert set(body) == {'ok', 'token', 'claim_id', 'expires_in_s'}
        assert body['ok'] is True
        assert isinstance(body['token'], str) and len(body['token']) >= 32
        assert len(body['claim_id']) == 8
        assert body['claim_id'] not in body['token']
        assert body['expires_in_s'] == defaults.OPERATOR_LOCK_TTL_S
        assert_security_headers(response)
        assert server.supervisor.adopted == [body['claim_id']]

    def test_second_claim_is_refused_while_held(self, server):
        """A held lock is never stolen; the second browser waits (§5.6)."""
        first = server.claim()
        response = server.request('POST', '/api/operator/claim')
        assert_error_envelope(response, 'operator_lock_held', 409)
        beat = server.request('POST', '/api/operator/heartbeat',
                              headers={'X-Operator-Token': first})
        assert beat.status == 200

    def test_heartbeat_refreshes(self, server):
        """§6.3 answers the new lifetime for the holder."""
        token = server.claim()
        server.clock.advance(5.0)
        response = server.request('POST', '/api/operator/heartbeat',
                                  headers={'X-Operator-Token': token})
        assert response.status == 200
        assert response.json() == {'ok': True,
                                   'expires_in_s': defaults.OPERATOR_LOCK_TTL_S}

    def test_heartbeat_with_stale_token_is_refused(self, server):
        """An expired token cannot resurrect itself with a heartbeat."""
        token = server.claim()
        server.clock.advance(defaults.OPERATOR_LOCK_TTL_S + 1.0)
        response = server.request('POST', '/api/operator/heartbeat',
                                  headers={'X-Operator-Token': token})
        assert_error_envelope(response, 'operator_token_invalid', 401)

    def test_release_frees_the_lock_and_notifies_the_supervisor(self, server):
        """§6.4 releases and forces the enables off via the revocation hook."""
        token = server.claim()
        response = server.request('POST', '/api/operator/release',
                                  headers={'X-Operator-Token': token})
        assert response.status == 200
        assert response.json() == {'ok': True}
        assert server.supervisor.releases == 1
        assert server.lock.state() == {'locked': False, 'claim_id': None,
                                       'since': None, 'expires_in_s': None}
        again = server.request('POST', '/api/operator/claim')
        assert again.status == 200

    def test_release_with_a_wrong_token_is_refused(self, server):
        """A stranger's release neither frees the lock nor reaches the supervisor."""
        server.claim()
        response = server.request('POST', '/api/operator/release',
                                  headers={'X-Operator-Token': 'not-the-token'})
        assert_error_envelope(response, 'operator_token_invalid', 401)
        assert server.supervisor.releases == 0
        assert server.lock.state()['locked'] is True


class TestCapabilities:
    """The read-only capabilities payload, and the config projection."""

    def test_matches_capabilities_payload(self, server):
        """The body is exactly what capabilities_payload builds."""
        response = server.request('GET', '/api/capabilities')
        assert response.status == 200
        assert response.json() == capabilities_payload(server.settings)
        assert_security_headers(response)
        assert_no_cors(response)

    def test_capabilities_matches_the_contract_payload(self, server):
        """The key set is exhaustive; a new key is a deliberate change."""
        body = server.request('GET', '/api/capabilities').json()
        assert set(body) == {
            'ok', 'schema_version', 'server_version', 'arm_selections',
            'modes', 'sources', 'joint_count', 'jog_step_rad', 'jog_stream_hz',
            'watchdog_timeout_s', 'max_header_age_s', 'operator_lock_ttl_s',
            'operator_heartbeat_interval_s', 'state_frame_hz',
            'log_ring_lines', 'fault_causes', 'recording_root',
            'recording_enabled', 'ros_domain_id', 'config_path',
            'config_present', 'transport',
            'gripper_arms', 'gripper_actions', 'gripper_stroke_mm',
            'gripper_force_range_n', 'gripper_speed_range_mm_s'}
        assert body['ok'] is True
        assert body['modes'] == ['simulate', 'watch', 'motion']
        assert body['sources'] == ['jog', 'external']
        assert body['transport'] == 'sse'
        assert body['schema_version'] == defaults.SCHEMA_VERSION
        assert body['server_version'] == 'franka_web {}'.format(
            defaults.SERVER_VERSION)
        assert body['arm_selections'] == ['panda1', 'panda2', 'both']
        assert body['joint_count'] == defaults.JOINT_COUNT
        assert body['operator_lock_ttl_s'] == defaults.OPERATOR_LOCK_TTL_S
        assert body['ros_domain_id'] == 80
        assert body['fault_causes'] == list(faults.FAULT_CAUSES)
        assert body['log_ring_lines'] == 500

    def test_capabilities_still_publishes_the_state_frame_cadence(self, server):
        """
        The scene's frame interpolation reads this number; nothing may drop it.

        The 3D view smooths between two 5 Hz frames over one frame period,
        and it computes that period from this field. Deleting the key would
        leave the page one hard-coded 200 away from interpolating at a
        cadence the server no longer publishes at -- a silently wrong-looking
        scene rather than a failure.
        """
        body = server.request('GET', '/api/capabilities').json()
        assert body['state_frame_hz'] == defaults.STATE_FRAME_HZ
        assert body['state_frame_hz'] > 0

    def test_capabilities_no_longer_offers_controllers_or_gains_limits(
            self, server):
        """The controller is not a choice and there is no upload surface."""
        body = server.request('GET', '/api/capabilities').json()
        for gone in ('controllers', 'jog_controllers', 'max_gains_bytes',
                     'activation_settling_policy', 'controller_name'):
            assert gone not in body

    def test_the_reviewed_timing_is_reported_read_only(self, server):
        """
        These are NOT configuration keys, and this is where an operator sees them.

        The reviewed controller-config validator requires exact equality with
        them, so a configuration key that only ever accepts one value would be
        a control that does nothing.
        """
        body = server.request('GET', '/api/capabilities').json()
        assert body['watchdog_timeout_s'] == (
            defaults.REVIEWED_TIMING_S['watchdog_timeout'])
        assert body['max_header_age_s'] == (
            defaults.REVIEWED_TIMING_S['max_header_age'])

    def test_capabilities_names_the_config_file_it_would_read(self, server):
        """The console's profile popover says "read from <this path>"."""
        body = server.request('GET', '/api/capabilities').json()
        assert body['config_path'] == server.settings.config_path
        assert body['config_present'] is True

    def test_config_endpoint_returns_the_effective_settings(self, server):
        """GET /api/config is the read-only projection of what is loaded."""
        response = server.request('GET', '/api/config')
        assert response.status == 200
        body = response.json()
        assert body['ok'] is True
        assert body == {'ok': True, **server.settings.public_view()}
        assert set(body['profiles']) == {'panda1', 'panda2'}
        assert body['settling']['drift_limit_rad']
        assert_security_headers(response)

    def test_config_endpoint_reports_the_robot_addresses(self, server):
        """
        Robot addresses ARE returned: they are not secrets.

        The v1 posture of never emitting an address in any response is gone,
        and the console's profile popover shows them.
        """
        body = server.request('GET', '/api/config').json()
        assert body['robots'] == {'panda1': DOC_IP_1, 'panda2': DOC_IP_2}

    def test_config_omits_the_install_time_libfranka_directory(self, server):
        """It is an install detail, not an operator-facing setting."""
        body = server.request('GET', '/api/config').json()
        assert 'franka_dir' not in body
        assert 'directories' not in body


class TestStateSurfaces:
    """§6.10 polling and §6.11 streaming carry the same frame."""

    def test_state_wraps_the_frame(self, server):
        """GET /api/state is ``{"ok": true, "state": <frame>}``."""
        response = server.request('GET', '/api/state')
        assert response.status == 200
        body = response.json()
        assert set(body) == {'ok', 'state'}
        assert body['ok'] is True
        assert body['state'] == server.supervisor.frame()
        assert body['state']['session']['advisory'] == defaults.STOP_ADVISORY
        assert_security_headers(response)

    def test_state_needs_no_token(self, server):
        """The read-only state surface is open (§6.10)."""
        server.claim()
        assert server.request('GET', '/api/state').status == 200

    def test_stream_head_and_first_frame(self, server):
        """The SSE head carries the §6.11 headers and an immediate frame."""
        stream = server.open_stream()
        assert stream.status == 200
        assert stream.headers['content-type'] == 'text/event-stream'
        assert stream.headers['cache-control'] == 'no-store'
        assert stream.headers['x-content-type-options'] == 'nosniff'
        assert stream.headers['content-security-policy'] == CSP
        assert stream.headers['x-accel-buffering'] == 'no'
        event, data = stream.read_event()
        assert event == 'event: state'
        assert data.startswith('data: ')
        assert json.loads(data[len('data: '):]) == server.supervisor.frame()
        stream.close()
        server.flush_streams()

    def test_stream_fans_out_published_frames(self, server):
        """Frames published after the head reach the open stream."""
        stream = server.open_stream()
        assert stream.read_event()[0] == 'event: state'
        frame = minimal_frame()
        frame['session']['state'] = 'running'
        server.broker.publish('state', frame)
        event, data = stream.read_event()
        assert event == 'event: state'
        assert json.loads(data[len('data: '):])['session']['state'] == 'running'
        server.broker.publish('ping', {'schema_version': defaults.SCHEMA_VERSION,
                                       't': '2026-08-29T00:00:01.000000Z'})
        event, data = stream.read_event()
        assert event == 'event: ping'
        assert json.loads(data[len('data: '):])['schema_version'] == 4
        stream.close()
        server.flush_streams()

    def test_stream_unsubscribes_when_the_client_goes(self, server):
        """A dropped browser is pruned rather than left in the fan-out."""
        stream = server.open_stream()
        stream.read_event()
        assert server.broker.subscriber_count == 1
        stream.close()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            server.broker.publish('state', minimal_frame())
            if server.broker.subscriber_count == 0:
                break
            time.sleep(0.05)
        assert server.broker.subscriber_count == 0


class TestSecurityHeaders:
    """§5.7 headers on success and on failure alike."""

    def test_headers_on_every_success(self, server):
        """JSON and static success responses both carry all three headers."""
        token = server.claim()
        for response in (server.request('GET', '/'),
                         server.request('GET', '/app.css'),
                         server.request('GET', '/api/capabilities'),
                         server.request('GET', '/api/state'),
                         server.request('POST', '/api/session/stop',
                                        headers={'X-Operator-Token': token})):
            assert response.status < 400, response.body
            assert_security_headers(response)

    def test_headers_on_every_refusal(self, server):
        """Guard, routing, token and body refusals all carry them too."""
        for response in (server.request(
                             'GET', '/api/capabilities',
                             headers={'Origin': 'http://evil.example'}),
                         server.request('GET', '/api/nope'),
                         server.request('POST', '/api/capabilities'),
                         server.request('POST', '/api/session/stop'),
                         server.request('GET', '/../package.xml')):
            assert response.status >= 400
            assert_security_headers(response)

    def test_csp_is_the_normative_string(self, server):
        """The policy is compared character for character, not by prefix."""
        policy = server.request('GET', '/').header('Content-Security-Policy')
        assert policy == CSP
        assert "frame-ancestors 'none'" in policy
        assert "base-uri 'none'" in policy
        assert "form-action 'none'" in policy


class TestUnreadRequestBody:
    """
    Finding F-1: a declared body no handler reads must never become a request.

    Most §6 routes take no body at all. Their handlers used to return without
    touching ``rfile``, leaving the declared bytes in front of the next
    request on a keep-alive connection -- and the stdlib then parsed those
    bytes AS the next request, so one request drew two responses. The bytes
    were still subject to the origin guard on their own headers, so this was
    an artefact rather than a way in; it is closed at the source anyway.
    """

    def smuggled_get(self, server):
        """Return a complete, on-its-own-valid request to hide in a body."""
        return ('GET /api/capabilities HTTP/1.1\r\n'
                'Host: 127.0.0.1:{}\r\n\r\n').format(server.port).encode('ascii')

    def post_with_body(self, server, path, body, token=None):
        """Build one POST that declares ``body`` on a route that never reads it."""
        head = 'POST {} HTTP/1.1\r\nHost: 127.0.0.1:{}\r\n'.format(path, server.port)
        if token is not None:
            head += 'X-Operator-Token: {}\r\n'.format(token)
        head += 'Content-Length: {}\r\n\r\n'.format(len(body))
        return head.encode('ascii') + body

    def test_heartbeat_body_is_not_answered_a_second_time(self, server):
        """H1: POST /api/operator/heartbeat + a hidden request → one response."""
        token = server.claim()
        payload, _ = server.raw_exchange(self.post_with_body(
            server, '/api/operator/heartbeat', self.smuggled_get(server), token))
        assert payload.count(b'HTTP/1.1 ') == 1, payload
        assert payload.startswith(b'HTTP/1.1 200 ')
        assert b'expires_in_s' in payload

    def test_release_body_is_not_answered_a_second_time(self, server):
        """H5: the same probe against POST /api/operator/release."""
        token = server.claim()
        payload, _ = server.raw_exchange(self.post_with_body(
            server, '/api/operator/release', self.smuggled_get(server), token))
        assert payload.count(b'HTTP/1.1 ') == 1, payload
        assert payload.startswith(b'HTTP/1.1 200 ')
        assert server.supervisor.releases == 1

    def test_claim_body_is_not_answered_a_second_time(self, server):
        """The unauthenticated route behaves the same; no token involved."""
        payload, _ = server.raw_exchange(self.post_with_body(
            server, '/api/operator/claim', self.smuggled_get(server)))
        assert payload.count(b'HTTP/1.1 ') == 1, payload
        assert payload.startswith(b'HTTP/1.1 200 ')

    def test_the_connection_stays_usable_and_correctly_framed(self, server):
        """
        The drain consumes exactly the body, so a pipelined request still works.

        This is the half a plain close would not prove: after the unread body
        is taken off the wire the parser is back in step, and the request the
        client really did send next gets its own answer, in order.
        """
        token = server.claim()
        first = self.post_with_body(
            server, '/api/operator/heartbeat', self.smuggled_get(server), token)
        second = ('GET /api/capabilities HTTP/1.1\r\n'
                  'Host: 127.0.0.1:{}\r\n\r\n').format(server.port).encode('ascii')
        payload, _ = server.raw_exchange(first + second)
        assert payload.count(b'HTTP/1.1 ') == 2, payload
        assert payload.startswith(b'HTTP/1.1 200 ')
        assert b'"schema_version"' in payload

    def test_a_body_too_large_to_drain_ends_the_connection(self, server):
        """Past the drain limit the connection is dropped, never desynced."""
        token = server.claim()
        body = b'x' * (_MAX_DRAIN_BYTES + 1)
        payload, closed = server.raw_exchange(self.post_with_body(
            server, '/api/operator/heartbeat', body, token))
        assert payload.count(b'HTTP/1.1 ') == 1, payload
        assert payload.startswith(b'HTTP/1.1 200 ')
        assert closed is True

    def test_a_chunked_body_is_refused_and_the_connection_closed(self, server):
        """G.10: chunked framing this server cannot decode is refused outright."""
        token = server.claim()
        body = b'2f\r\nGET /api/capabilities HTTP/1.1\r\nHost: x\r\n\r\n0\r\n\r\n'
        request = ('POST /api/operator/heartbeat HTTP/1.1\r\n'
                   'Host: 127.0.0.1:{}\r\n'
                   'X-Operator-Token: {}\r\n'
                   'Transfer-Encoding: chunked\r\n\r\n').format(
                       server.port, token).encode('ascii') + body
        payload, closed = server.raw_exchange(request)
        assert payload.count(b'HTTP/1.1 ') == 1, payload
        assert payload.startswith(b'HTTP/1.1 400 ')
        assert b'invalid_json' in payload
        assert b'chunked transfer encoding is not supported' in payload
        assert closed is True

    def test_two_content_length_headers_are_refused(self, server):
        """Disagreeing lengths are the same desync by another route."""
        body = self.smuggled_get(server)
        request = ('POST /api/operator/claim HTTP/1.1\r\n'
                   'Host: 127.0.0.1:{}\r\n'
                   'Content-Length: {}\r\n'
                   'Content-Length: 0\r\n\r\n').format(
                       server.port, len(body)).encode('ascii') + body
        payload, closed = server.raw_exchange(request)
        assert payload.count(b'HTTP/1.1 ') == 1, payload
        assert payload.startswith(b'HTTP/1.1 400 ')
        assert b'exactly one Content-Length header' in payload
        assert closed is True

    def test_a_get_with_a_declared_body_is_settled_too(self, server):
        """Static and read-only GETs are no different: nothing is left behind."""
        body = self.smuggled_get(server)
        request = ('GET /api/state HTTP/1.1\r\nHost: 127.0.0.1:{}\r\n'
                   'Content-Length: {}\r\n\r\n').format(
                       server.port, len(body)).encode('ascii') + body
        payload, _ = server.raw_exchange(request)
        assert payload.count(b'HTTP/1.1 ') == 1, payload
        assert payload.startswith(b'HTTP/1.1 200 ')

    def test_a_refusal_still_closes_rather_than_drains(self, server):
        """An unread body behind a REFUSED request ends the connection (R10)."""
        body = self.smuggled_get(server)
        payload, closed = server.raw_exchange(self.post_with_body(
            server, '/api/operator/heartbeat', body))     # no token: 401
        assert payload.count(b'HTTP/1.1 ') == 1, payload
        assert payload.startswith(b'HTTP/1.1 401 ')
        assert closed is True


class TestUnknownMethods:
    """
    Finding F-5: a method with no handler must not escape the guard.

    ``OPTIONS``/``PUT``/``DELETE``/``PATCH`` were written out explicitly; the
    open end of the set fell through to ``BaseHTTPRequestHandler``'s own 501
    HTML page, which never runs ``_guard_origin`` and carries none of the
    §5.7 headers.
    """

    @pytest.mark.parametrize('method', ['TRACE', 'CONNECT', 'PROPFIND', 'BREW'])
    def test_unknown_method_is_refused_inside_the_envelope(self, server, method):
        """A known path answers 405 with the §6.0 envelope and §5.7 headers."""
        payload, closed = server.raw_exchange(
            '{} /api/state HTTP/1.1\r\nHost: 127.0.0.1:{}\r\n\r\n'.format(
                method, server.port).encode('ascii'))
        assert payload.startswith(b'HTTP/1.1 405 '), payload
        assert b'"method_not_allowed"' in payload
        assert b'Cache-Control: no-store' in payload
        assert b'X-Content-Type-Options: nosniff' in payload
        assert CSP.encode('ascii') in payload
        assert b'501' not in payload
        assert closed is True

    def test_unknown_method_on_an_unknown_path_is_not_found(self, server):
        """The same fallback distinguishes the path, exactly as PUT does."""
        payload, _ = server.raw_exchange(
            'TRACE /nope HTTP/1.1\r\nHost: 127.0.0.1:{}\r\n\r\n'.format(
                server.port).encode('ascii'))
        assert payload.startswith(b'HTTP/1.1 404 '), payload
        assert b'"not_found"' in payload

    def test_unknown_method_is_origin_guarded(self, server):
        """The guard runs first, so a foreign Host never reaches the router."""
        payload, _ = server.raw_exchange(
            b'TRACE /api/state HTTP/1.1\r\nHost: lab-nuc:8765\r\n'
            b'Origin: http://evil.example\r\n\r\n')
        assert payload.startswith(b'HTTP/1.1 403 '), payload
        assert b'"forbidden_origin"' in payload
        assert CSP.encode('ascii') in payload

    def test_the_written_out_refusals_still_win(self, server):
        """__getattr__ is a fallback only; the explicit handlers are untouched."""
        for method in ('OPTIONS', 'PUT', 'DELETE', 'PATCH'):
            response = server.request(method, '/api/state')
            assert_error_envelope(response, 'method_not_allowed', 405)


class TestClosedErrorSet:
    """
    §6.14 is a closed set, and every copy of it must say the same thing.

    Findings F-3 and F-4 were both drift between these two lists: the server
    emitted ``arm_not_enabled`` while the consumer schema did not know it, and
    both carried ``preflight_unavailable``, which no code path ever raised.
    Asserting the two against each other is what stops that recurring.
    """

    def schema_codes(self):
        """Return §6.14's code list as the consumer schema publishes it."""
        with open(SCHEMA_PATH) as handle:
            schema = json.load(handle)
        return schema['$defs']['error_code']['enum']

    def test_the_server_table_and_the_consumer_schema_agree(self):
        """One set of codes, spelled the same in both places."""
        codes = self.schema_codes()
        assert len(codes) == len(set(codes)), 'the schema repeats a code'
        assert set(codes) == set(_ERROR_STATUS)

    def test_the_set_is_the_documented_size(self):
        """A code added to only one of the two lists fails right here."""
        assert len(_ERROR_STATUS) == 44

    def test_the_two_ghost_codes_carry_their_contract_statuses(self):
        """503 for an IK service that cannot answer, 429 for too fast a drag."""
        assert _ERROR_STATUS['ghost_unavailable'] == 503
        assert _ERROR_STATUS['ghost_rate_limited'] == 429
        assert 'ghost_unavailable' in self.schema_codes()
        assert 'ghost_rate_limited' in self.schema_codes()

    def test_arm_not_enabled_is_a_contract_code(self):
        """§6.13's jog refusal is in the set, at the status it is emitted with."""
        assert _ERROR_STATUS['arm_not_enabled'] == 409
        assert 'arm_not_enabled' in self.schema_codes()

    def test_preflight_unavailable_is_gone_from_both(self):
        """The dead entry was removed rather than implemented (finding F-4)."""
        assert 'preflight_unavailable' not in _ERROR_STATUS
        assert 'preflight_unavailable' not in self.schema_codes()

    def test_every_code_maps_to_an_error_status(self):
        """No code may quietly map to a success or a redirect."""
        assert all(400 <= status <= 599 for status in _ERROR_STATUS.values())


class TestOperatorTakeover:
    """The forcible claim, and the claim-adoption seam behind it."""

    def test_a_second_claim_is_refused_with_operator_lock_held(self, server):
        """That refusal is the signal for the page to offer Take over."""
        server.claim()
        response = server.request('POST', '/api/operator/claim')
        assert_error_envelope(response, 'operator_lock_held', 409)

    def test_takeover_succeeds_without_a_token(self, server):
        """A taking-over operator has no token, so none is required."""
        first = server.request('POST', '/api/operator/claim').json()
        response = server.request('POST', '/api/operator/takeover')
        assert response.status == 200
        body = response.json()
        assert set(body) == {'ok', 'token', 'claim_id', 'expires_in_s'}
        assert body['token'] != first['token']
        assert body['claim_id'] != first['claim_id']
        assert server.lock.validate(first['token']) is False

    def test_takeover_revokes_the_incumbent_and_says_so_in_the_log(self, server):
        """
        The revocation hook runs, and the operator can see why arms went off.

        "Taking over resets every enable" is a promise this endpoint keeps.
        """
        server.claim()
        server.request('POST', '/api/operator/takeover')
        assert server.supervisor.releases == 1
        messages = [line['message'] for line in server.logs.window()['lines']]
        assert 'operator control was taken over; every arm was disabled' in messages

    def test_takeover_failure_maps_to_takeover_failed_503(self, server):
        """A revocation that cannot complete leaves the incumbent holding."""
        def failing():
            raise RuntimeError('the arms could not be disabled')

        server.claim()
        server.lock.set_revocation_hook(failing)
        response = server.request('POST', '/api/operator/takeover')
        assert_error_envelope(response, 'takeover_failed', 503)

    def test_a_claim_whose_revocation_hook_fails_is_an_internal_error(self, server):
        """
        A plain claim never takes anything over, so it never says it did.

        The page renders `error + ": " + detail` verbatim, so the wrong code
        would put the wrong word on screen.
        """
        def failing():
            raise RuntimeError('the arms could not be disabled')

        token = server.claim()
        server.lock.set_revocation_hook(failing)
        server.request('POST', '/api/operator/release',
                       headers={'X-Operator-Token': token})
        response = server.request('POST', '/api/operator/claim')
        body = assert_error_envelope(response, 'internal_error', 500)
        assert 'could not be revoked' in body['detail']

    def test_claim_takeover_and_heartbeat_all_adopt_the_session_claim(self, server):
        """
        All three refresh the session's stored operator identity.

        Without adoption, a `lock_expired` fault is a dead end: the Reclaim
        its own steps prescribe mints a NEW claim id, which would still
        differ from a frozen one, so the page would never be offered Recover.
        """
        first = server.request('POST', '/api/operator/claim').json()
        server.request('POST', '/api/operator/heartbeat',
                       headers={'X-Operator-Token': first['token']})
        second = server.request('POST', '/api/operator/takeover').json()
        assert server.supervisor.adopted == [
            first['claim_id'], first['claim_id'], second['claim_id']]


class TestLogEndpoint:
    """``GET /api/logs`` -- the ring-buffer backlog behind the drawer."""

    def test_logs_returns_the_ring_and_cumulative_counters(self, server):
        """The badge numbers are cumulative since server start, not windowed."""
        server.logs.emit('info', 'one')
        server.logs.emit('warn', 'two')
        response = server.request('GET', '/api/logs')
        assert response.status == 200
        body = response.json()
        assert body['ok'] is True
        assert [line['message'] for line in body['lines']] == ['one', 'two']
        assert body['warn_count'] == 1
        assert body['error_count'] == 0
        assert body['dropped'] == 0
        assert_security_headers(response)

    def test_logs_since_returns_only_newer_lines_and_reports_dropped(self, server):
        """`since` is exclusive, and `dropped` says whether history has a hole."""
        for index in range(5):
            server.logs.emit('info', 'line {}'.format(index))
        body = server.request('GET', '/api/logs?since=3').json()
        assert [line['seq'] for line in body['lines']] == [4, 5]
        assert body['dropped'] == 0

    def test_logs_limit_is_clamped_and_a_garbage_query_falls_back(self, server):
        """
        A reconnecting page with a garbage `since` gets the whole ring.

        Refusing it with a 400 would leave the drawer empty for the one case
        it exists to repair.
        """
        for index in range(4):
            server.logs.emit('info', 'line {}'.format(index))
        assert len(server.request('GET', '/api/logs?limit=2').json()['lines']) == 2
        assert len(server.request(
            'GET', '/api/logs?limit=nonsense').json()['lines']) == 4
        assert len(server.request(
            'GET', '/api/logs?since=nonsense').json()['lines']) == 4
        assert len(server.request('GET', '/api/logs?limit=99999').json()['lines']) == 4

    def test_logs_needs_no_token(self, server):
        """The drawer is a read-only surface, like the state frame."""
        server.claim()
        assert server.request('GET', '/api/logs').status == 200


class TestSourceEndpoint:
    """``POST /api/arm/{arm_id}/source``."""

    def test_source_endpoint_routes_to_the_supervisor(self, server):
        """The body reaches the supervisor and its answer is echoed back."""
        token = server.claim()
        response = server.request(
            'POST', '/api/arm/panda2/source',
            body=json.dumps({'source': 'external'}),
            headers={'X-Operator-Token': token})
        assert response.status == 200
        assert response.json() == {'ok': True, 'arm_id': 'panda2',
                                   'source': 'external'}
        assert server.supervisor.source_requests == [('panda2', 'external')]

    @pytest.mark.parametrize('body', ['{}', '{"source": 3}',
                                      '{"source": null}', '{"source": true}'])
    def test_an_unknown_source_value_is_a_400_invalid_source(self, server, body):
        """A non-string value never reaches the supervisor."""
        token = server.claim()
        response = server.request('POST', '/api/arm/panda1/source', body=body,
                                  headers={'X-Operator-Token': token})
        assert_error_envelope(response, 'invalid_source', 400)
        assert server.supervisor.source_requests == []

    def test_the_source_endpoint_requires_the_operator_token(self, server):
        """It is a mutating command, so it is behind the lock."""
        server.claim()
        response = server.request('POST', '/api/arm/panda1/source',
                                  body=json.dumps({'source': 'jog'}))
        assert_error_envelope(response, 'operator_token_invalid', 401)


class TestVendoredFontContentType:
    """The one dictionary entry the vendored fonts depend on."""

    def test_a_vendored_font_is_served_as_font_woff2_not_octet_stream(
            self, server, tmp_path):
        """
        Under `nosniff`, an octet-stream font is refused by the browser.

        Every static response carries X-Content-Type-Options: nosniff, so the
        browser is FORBIDDEN from guessing -- and a missing content type would
        silently drop the console to its fallback stacks with no error
        anywhere. This also re-proves that a NESTED static path is served.
        """
        fonts = os.path.join(STATIC_ROOT, 'fonts')
        os.makedirs(fonts, exist_ok=True)
        path = os.path.join(fonts, 'archivo-var.woff2')
        created = not os.path.exists(path)
        if created:
            with open(path, 'wb') as handle:
                handle.write(b'wOF2 not a real font, but a real file')
        try:
            response = server.request('GET', '/fonts/archivo-var.woff2')
            assert response.status == 200
            assert response.header('Content-Type') == 'font/woff2'
            assert response.header('X-Content-Type-Options') == 'nosniff'
        finally:
            if created:
                os.remove(path)
                try:
                    os.rmdir(fonts)
                except OSError:
                    pass


class TestErrorTableMatchesTheContract:
    """The closed error set, spelled out."""

    def test_every_error_code_in_the_table_matches_the_contract(self):
        """A code added or removed on one side only fails right here."""
        assert set(_ERROR_STATUS) == {
            'operator_lock_held', 'operator_token_invalid',
            'session_already_active', 'session_not_active',
            'session_not_running', 'session_faulted', 'not_motion_mode',
            'arm_not_in_session', 'invalid_arms', 'invalid_mode',
            'invalid_json', 'invalid_joint', 'invalid_source',
            'robot_addresses_missing', 'preflight_failed',
            'pose_outside_fence', 'activation_settling_limit',
            'activation_settling_timeout', 'joint_state_stale',
            'recording_failed', 'launch_failed', 'launch_timeout',
            'profile_invalid', 'enable_service_unavailable', 'enable_rejected',
            'recovery_service_unavailable', 'recovery_failed',
            'recovery_not_supported', 'takeover_failed', 'arm_not_enabled',
            'not_faulted', 'forbidden_origin', 'not_found',
            'method_not_allowed', 'payload_too_large', 'internal_error',
            'ghost_unavailable', 'ghost_rate_limited',
            'gripper_not_configured', 'gripper_unavailable', 'gripper_faulted',
            'gripper_busy', 'invalid_gripper_action', 'invalid_gripper_width'}

    def test_the_removed_codes_are_gone(self):
        """Every code whose mechanism v2 deleted is gone from the table."""
        for gone in ('controller_not_reviewed', 'gains_required',
                     'gains_unknown', 'gains_too_large', 'gains_invalid',
                     'gains_controller_mismatch', 'gains_arms_mismatch',
                     'gains_preview_mismatch', 'fence_pose_unverified',
                     'settling_policy_required', 'settling_fence_required',
                     'settling_margin_unavailable', 'not_production_mode'):
            assert gone not in _ERROR_STATUS


# ----------------------------------------------------------------------
# The gripper endpoint
# ----------------------------------------------------------------------

GRIPPER_SERIAL_ID = 'usb-FTDI_FT230X_Basic_UART_D3091K4T-if00-port0'


def enabled_gripper(arm_id='panda1'):
    """Return a {arm_id: GripperConfig} mapping with that arm enabled."""
    return {arm_id: config.GripperConfig(
        arm_id=arm_id, enabled=True, serial_id=GRIPPER_SERIAL_ID,
        from_file=True)}


@pytest.fixture()
def gripper_server(tmp_path):
    """Serve one app whose panda1 has a gripper configured."""
    running = Server(tmp_path, grippers=enabled_gripper())
    yield running
    running.close()


def claimed(server):
    """Claim the operator lock and return the token header mapping."""
    token = server.request('POST', '/api/operator/claim').json()['token']
    return {'X-Operator-Token': token}


class TestGripperEndpoint:
    """POST /api/arm/{arm_id}/gripper, and what capabilities says about it."""

    def test_capabilities_reports_the_gripper_surface(self, gripper_server):
        """The five capability keys, from the config file and from defaults."""
        body = gripper_server.request('GET', '/api/capabilities').json()
        assert body['gripper_arms'] == ['panda1']
        assert body['gripper_actions'] == list(defaults.GRIPPER_ACTIONS)
        assert body['gripper_stroke_mm'] == defaults.GRIPPER_STROKE_MM
        assert body['gripper_force_range_n'] == list(defaults.GRIPPER_FORCE_RANGE_N)
        assert body['gripper_speed_range_mm_s'] == list(
            defaults.GRIPPER_SPEED_RANGE_MM_S)

    def test_capabilities_reports_no_gripper_arms_on_an_unequipped_cell(self, server):
        """[] is what the page uses to decide the feature exists at all."""
        assert server.request('GET', '/api/capabilities').json()['gripper_arms'] == []

    def test_config_carries_the_grippers_block_for_enabled_arms_only(
            self, gripper_server, server):
        """GET /api/config lists the arms that HAVE a gripper, and no others."""
        block = gripper_server.request('GET', '/api/config').json()['grippers']
        assert sorted(block) == ['panda1']
        assert block['panda1']['serial_id'] == GRIPPER_SERIAL_ID
        assert block['panda1']['source'] == 'config'
        assert server.request('GET', '/api/config').json()['grippers'] == {}

    def test_the_gripper_endpoint_requires_the_operator_token(self, gripper_server):
        """A state-changing call without a token is refused before dispatch."""
        response = gripper_server.request(
            'POST', '/api/arm/panda1/gripper', json.dumps({'action': 'close'}))
        assert response.status == 401
        assert response.json()['error'] == 'operator_token_invalid'
        assert gripper_server.supervisor.gripper_requests == []

    def test_an_unknown_action_is_refused_with_invalid_gripper_action(
            self, gripper_server):
        """The action set is closed and the refusal spells it out."""
        headers = claimed(gripper_server)
        response = gripper_server.request(
            'POST', '/api/arm/panda1/gripper', json.dumps({'action': 'squeeze'}),
            headers=headers)
        assert response.status == 400
        assert response.json()['error'] == 'invalid_gripper_action'
        assert "'reactivate'" in response.json()['detail']

    def test_width_requires_width_mm_and_the_others_forbid_it(self, gripper_server):
        """width_mm is required for 'width' and forbidden everywhere else."""
        headers = claimed(gripper_server)
        missing = gripper_server.request(
            'POST', '/api/arm/panda1/gripper',
            json.dumps({'action': 'width'}), headers=headers)
        assert missing.status == 400
        assert missing.json()['error'] == 'invalid_gripper_action'
        extra = gripper_server.request(
            'POST', '/api/arm/panda1/gripper',
            json.dumps({'action': 'close', 'width_mm': 10.0}), headers=headers)
        assert extra.status == 400
        assert extra.json()['error'] == 'invalid_gripper_action'
        boolean = gripper_server.request(
            'POST', '/api/arm/panda1/gripper',
            json.dumps({'action': 'width', 'width_mm': True}), headers=headers)
        assert boolean.status == 400
        assert gripper_server.supervisor.gripper_requests == []

    @pytest.mark.parametrize('width', [-1.0, 120.0])
    def test_a_width_outside_the_stroke_is_refused_with_invalid_gripper_width(
            self, gripper_server, width):
        """Both sides of the stroke, before the supervisor is ever asked."""
        headers = claimed(gripper_server)
        response = gripper_server.request(
            'POST', '/api/arm/panda1/gripper',
            json.dumps({'action': 'width', 'width_mm': width}), headers=headers)
        assert response.status == 400
        assert response.json()['error'] == 'invalid_gripper_width'
        assert '85 mm' in response.json()['detail']
        assert gripper_server.supervisor.gripper_requests == []

    @pytest.mark.parametrize('code,status', [
        ('gripper_not_configured', 404), ('gripper_unavailable', 503),
        ('gripper_faulted', 409), ('gripper_busy', 409),
        ('arm_not_in_session', 404), ('session_not_running', 409)])
    def test_each_supervisor_refusal_maps_to_its_status(self, gripper_server,
                                                        code, status):
        """Every refusal the supervisor can raise carries its documented status."""
        headers = claimed(gripper_server)
        gripper_server.supervisor.gripper_error = SessionError(code, 'because')
        response = gripper_server.request(
            'POST', '/api/arm/panda1/gripper', json.dumps({'action': 'close'}), headers=headers)
        assert response.status == status
        assert response.json()['error'] == code
        assert response.json()['detail'] == 'because'

    def test_the_response_echoes_the_arm_action_and_effective_width(
            self, gripper_server):
        """The v2 envelope, plus the target the action implies."""
        headers = claimed(gripper_server)
        response = gripper_server.request(
            'POST', '/api/arm/panda1/gripper',
            json.dumps({'action': 'width', 'width_mm': 30.0}), headers=headers)
        assert response.status == 200
        assert response.json() == {'ok': True, 'arm_id': 'panda1',
                                   'action': 'width', 'width_mm': 30.0}
        assert gripper_server.supervisor.gripper_requests == [
            ('panda1', 'width', 30.0)]

    def test_a_watch_session_still_accepts_a_gripper_command_the_page_disables(
            self, gripper_server):
        """
        The Watch read-only rule is page-side ONLY, and that is a decision.

        Under the standing-node design the gripper is commandable from ROS in
        every session mode, so an API mode gate would refuse this button while
        the identical motion stayed one `ros2 action send_goal` away. The
        refusal ladder therefore has no session-mode row; adding one turns
        this test red, which is the point.
        """
        headers = claimed(gripper_server)
        frame = gripper_server.supervisor.frame_value
        frame['session']['mode'] = 'watch'
        frame['session']['state'] = 'running'
        response = gripper_server.request(
            'POST', '/api/arm/panda1/gripper', json.dumps({'action': 'close'}), headers=headers)
        assert response.status == 200
        assert response.json()['ok'] is True
        assert gripper_server.supervisor.gripper_requests == [
            ('panda1', 'close', None)]
        assert 'not_motion_mode' not in json.dumps(response.json())

    def test_the_route_is_token_required_in_the_table(self):
        """The routing table is what enforces the token, so assert it there."""
        row = [route for route in ROUTES
               if route.path == '/api/arm/{arm_id}/gripper']
        assert len(row) == 1
        assert row[0].method == 'POST'
        assert row[0].needs_token is True

    def test_the_six_new_codes_are_in_the_table_at_their_documented_status(self):
        """The six codes and their statuses, spelled out once."""
        assert _ERROR_STATUS['gripper_not_configured'] == 404
        assert _ERROR_STATUS['gripper_unavailable'] == 503
        assert _ERROR_STATUS['gripper_faulted'] == 409
        assert _ERROR_STATUS['gripper_busy'] == 409
        assert _ERROR_STATUS['invalid_gripper_action'] == 400
        assert _ERROR_STATUS['invalid_gripper_width'] == 400
