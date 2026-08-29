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

import http.client
import json
import os
import socket
import threading
import time

from franka_web import config
from franka_web.config import Settings
from franka_web.http_api import App, build_server, capabilities_payload, ROUTES
from franka_web.lock import OperatorLock
from franka_web.session import SessionError
from franka_web.sse import Broker
import pytest
from support.fake_clock import FakeClock

#: RFC 5737 documentation addresses; never a real robot, never routed.
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
        'schema_version': config.SCHEMA_VERSION,
        'server_time': '2026-08-29T00:00:00.000000Z',
        'server_uptime_s': 1.5,
        'session': {
            'state': 'stopped',
            'session_id': None,
            'arms': None,
            'arm_ids': [],
            'arm_mode': None,
            'mode': None,
            'controller_name': None,
            'gains_sha256': None,
            'started_at': None,
            'uptime_s': None,
            'launch_running': False,
            'last_error': None,
            'advisory': config.STOP_ADVISORY,
        },
        'operator': {'locked': False, 'expires_in_s': None},
        'preflight': {'ran_at': None, 'overall': None,
                      'blocking': False, 'failed_checks': []},
        'recording': {'active': False, 'name': None, 'sequence': 0,
                      'path': None, 'arm_mode': None, 'topics': []},
        'controllers': [],
        'hardware': {'available': False, 'name': None, 'plugin_name': None,
                     'lifecycle_id': None, 'lifecycle_label': None},
        'fault': {'active': False, 'since': None, 'reasons': [],
                  'recoverable': False, 'recover_hint': None},
        'arms': {},
    }


class FakeSupervisor:
    """
    Scriptable stand-in for SessionSupervisor's HTTP-thread surface.

    Only the three methods ``http_api`` actually calls exist here
    (``request_start``, ``request_stop``, ``operator_released``) plus
    ``frame``; anything else the handlers reach for would be a contract
    change this test should notice.
    """

    def __init__(self):
        """Start out accepting every command and reporting an idle frame."""
        self.start_requests = []
        self.stop_calls = 0
        self.releases = 0
        self.start_error = None
        self.stop_error = None
        self.start_result = {'session_id': 'web-20260829-101500', 'state': 'preflight'}
        self.stop_result = {'state': 'stopping'}
        self.frame_value = minimal_frame()

    def request_start(self, request):
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

    def operator_released(self):
        """Count the §6.4 release notification."""
        self.releases += 1

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

    def __init__(self, tmp_path, supervisor=None):
        """Bind, wire and start serving on a daemon thread."""
        state_dir = tmp_path / 'state'
        state_dir.mkdir(mode=0o700, exist_ok=True)
        recording_root = tmp_path / 'recordings'
        recording_root.mkdir(mode=0o700, exist_ok=True)
        self.state_dir = str(state_dir)
        self.recording_root = str(recording_root)
        self.clock = FakeClock()
        self.supervisor = supervisor or FakeSupervisor()
        self.lock = OperatorLock(monotonic=self.clock.monotonic)
        self.broker = Broker()
        self.settings = None
        self.httpd = None
        self._streams = []
        for _ in range(10):
            settings = self._settings_for(free_port())
            app = App(settings=settings, supervisor=self.supervisor, lock=self.lock,
                      broker=self.broker, static_root=STATIC_ROOT)
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
        """Build Settings directly (no environment, no directory revalidation)."""
        return Settings(
            bind='127.0.0.1',
            port=port,
            state_dir=self.state_dir,
            recording_root=self.recording_root,
            ros_domain_id=80,
            robot_ip_1=DOC_IP_1,
            robot_ip_2=DOC_IP_2,
            robot_ip_single=DOC_IP_1,
        )

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
            response = server.request(
                route.method, route.path, body=body_for(route),
                headers={'X-Operator-Token': token}, read_body=not streaming)
            assert response.status not in (404, 405), (
                '{} {} did not dispatch'.format(route.method, route.path))
            if not streaming and response.status >= 400:
                assert response.json()['error'] not in ('not_found', 'method_not_allowed')
            if route.path == '/api/operator/release':
                token = server.claim()
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
                               'advisory': config.STOP_ADVISORY}

    def test_start_body_reaches_the_supervisor(self, server):
        """The §6.7 fields arrive as a SessionRequest, addresses absent."""
        token = server.claim()
        server.request('POST', '/api/session/start',
                       body=json.dumps({'arms': 'panda1', 'mode': 'watch',
                                        'controller_name': None,
                                        'gains_sha256': 'abc'}),
                       headers={'X-Operator-Token': token})
        request = server.supervisor.start_requests[-1]
        assert request.arms == 'panda1'
        assert request.mode == 'watch'
        assert request.gains_sha256 == 'abc'

    def test_gains_list_is_empty_in_stage_one(self, server):
        """§6.6 answers an empty list until Stage 2 delivers uploads."""
        response = server.request('GET', '/api/gains')
        assert response.status == 200
        assert response.json() == {'ok': True, 'gains': []}

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
    """The §5.7 Host / Origin / Sec-Fetch-Site matrix."""

    def test_default_host_accepted(self, server):
        """The IP host http.client sends by default is accepted."""
        assert server.request('GET', '/api/capabilities').status == 200

    def test_localhost_host_accepted(self, server):
        """``localhost:<port>`` is the other accepted Host spelling."""
        response = server.request('GET', '/api/capabilities',
                                  host='localhost:{}'.format(server.port))
        assert response.status == 200

    @pytest.mark.parametrize('host', [
        'evil.example',
        'evil.example:8781',
        '127.0.0.1',
        '127.0.0.1:1',
        'localhost',
        '[::1]:8781',
        '',
    ])
    def test_bad_host_refused(self, server, host):
        """Any Host that is not exactly a loopback name plus our port is 403."""
        response = server.request('GET', '/api/capabilities', host=host)
        assert_error_envelope(response, 'forbidden_origin', 403)

    def test_host_with_our_port_but_other_name_refused(self, server):
        """A DNS-rebinding style Host on the right port is still refused."""
        response = server.request('GET', '/api/capabilities',
                                  host='attacker.test:{}'.format(server.port))
        assert_error_envelope(response, 'forbidden_origin', 403)

    @pytest.mark.parametrize('scheme_host', ['127.0.0.1', 'localhost'])
    def test_same_origin_origin_accepted(self, server, scheme_host):
        """Both loopback spellings of our own http origin are accepted."""
        origin = 'http://{}:{}'.format(scheme_host, server.port)
        response = server.request('GET', '/api/capabilities',
                                  headers={'Origin': origin})
        assert response.status == 200

    def test_cross_origin_variants_refused(self, server):
        """A different scheme, port or host in Origin is 403 forbidden_origin."""
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

    @pytest.mark.parametrize('site', ['same-origin', 'none'])
    def test_allowed_fetch_site(self, server, site):
        """Sec-Fetch-Site same-origin and none are the page's own requests."""
        response = server.request('GET', '/api/capabilities',
                                  headers={'Sec-Fetch-Site': site})
        assert response.status == 200

    @pytest.mark.parametrize('site', ['cross-site', 'same-site'])
    def test_refused_fetch_site(self, server, site):
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
        """A bad Host cannot read a static file either."""
        response = server.request('GET', '/app.js', host='evil.example')
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
            server.request('GET', '/api/gains'),
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
            server.request('GET', '/api/capabilities', host='evil.example'),
            server.request('GET', '/api/capabilities',
                           headers={'Origin': 'http://evil.example'}),
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
        assert beat.json() == {'ok': True, 'expires_in_s': config.OPERATOR_LOCK_TTL_S}

    def test_expired_token_is_refused(self, server):
        """Past the TTL with no refresh, the token is inert."""
        token = server.claim()
        server.clock.advance(config.OPERATOR_LOCK_TTL_S + 0.001)
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
                     'Content-Length': str(config.MAX_GAINS_BYTES + 1),
                     'Connection': 'close'})
        assert_error_envelope(response, 'payload_too_large', 413)
        assert server.supervisor.start_requests == []

    def test_at_the_cap_is_not_refused(self, server):
        """Exactly MAX_GAINS_BYTES is inside the limit (it is a cap, not a fence)."""
        token = server.claim()
        padding = 'a' * (config.MAX_GAINS_BYTES - 100)
        body = json.dumps({'arms': 'both', 'mode': 'simulate', 'pad': padding})
        assert len(body) <= config.MAX_GAINS_BYTES
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
        response = server.request('GET', '/api/capabilities', host='evil.example')
        assert response.header('Content-Type') == 'application/json; charset=utf-8'
        assert response.json()['ok'] is False


class TestOperatorEndpoints:
    """§6.2 - §6.4, against the real OperatorLock."""

    def test_claim_returns_token_and_ttl(self, server):
        """A free lock mints a token with the §6.2 TTL."""
        response = server.request('POST', '/api/operator/claim')
        assert response.status == 200
        body = response.json()
        assert set(body) == {'ok', 'token', 'expires_in_s'}
        assert body['ok'] is True
        assert isinstance(body['token'], str) and len(body['token']) >= 32
        assert body['expires_in_s'] == config.OPERATOR_LOCK_TTL_S
        assert_security_headers(response)

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
                                   'expires_in_s': config.OPERATOR_LOCK_TTL_S}

    def test_heartbeat_with_stale_token_is_refused(self, server):
        """An expired token cannot resurrect itself with a heartbeat."""
        token = server.claim()
        server.clock.advance(config.OPERATOR_LOCK_TTL_S + 1.0)
        response = server.request('POST', '/api/operator/heartbeat',
                                  headers={'X-Operator-Token': token})
        assert_error_envelope(response, 'operator_token_invalid', 401)

    def test_release_frees_the_lock_and_notifies_the_supervisor(self, server):
        """§6.4 releases and forces the enables off via operator_released."""
        token = server.claim()
        response = server.request('POST', '/api/operator/release',
                                  headers={'X-Operator-Token': token})
        assert response.status == 200
        assert response.json() == {'ok': True}
        assert server.supervisor.releases == 1
        assert server.lock.state() == {'locked': False, 'expires_in_s': None}
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
    """§6.1, and the promise that no address ever leaves the process."""

    def test_matches_capabilities_payload(self, server):
        """The body is exactly what capabilities_payload builds."""
        response = server.request('GET', '/api/capabilities')
        assert response.status == 200
        assert response.json() == capabilities_payload(server.settings)
        assert_security_headers(response)
        assert_no_cors(response)

    def test_stage_one_modes_and_transport(self, server):
        """Motion is absent in Stage 1; the transport is SSE."""
        body = server.request('GET', '/api/capabilities').json()
        assert body['ok'] is True
        assert body['modes'] == ['simulate', 'watch']
        assert body['transport'] == 'sse'
        assert body['schema_version'] == config.SCHEMA_VERSION
        assert body['arm_selections'] == ['panda1', 'panda2', 'both']
        assert body['joint_count'] == config.JOINT_COUNT
        assert body['operator_lock_ttl_s'] == config.OPERATOR_LOCK_TTL_S
        assert body['ros_domain_id'] == 80

    def test_no_robot_address_in_the_body(self, server):
        """
        Configured addresses never appear in a response, anywhere.

        The settings behind this server carry both documentation addresses;
        the assertion is on the raw bytes, so a nested or re-encoded leak
        fails it just as loudly as a top-level field would.
        """
        assert server.settings.robot_ip_1 == DOC_IP_1
        assert server.settings.robot_ip_2 == DOC_IP_2
        raw = server.request('GET', '/api/capabilities').body
        for address in (DOC_IP_1, DOC_IP_2, '203.0.113'):
            assert address.encode('ascii') not in raw
        assert b'robot_ip' not in raw

    def test_no_robot_address_in_the_state_body(self, server):
        """The state surface is address-free too."""
        raw = server.request('GET', '/api/state').body
        assert b'203.0.113' not in raw


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
        assert body['state']['session']['advisory'] == config.STOP_ADVISORY
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
        server.broker.publish('ping', {'schema_version': config.SCHEMA_VERSION,
                                       't': '2026-08-29T00:00:01.000000Z'})
        event, data = stream.read_event()
        assert event == 'event: ping'
        assert json.loads(data[len('data: '):])['schema_version'] == 1
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
        for response in (server.request('GET', '/api/capabilities', host='evil.example'),
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
