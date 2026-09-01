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
HTTP surface: routing, browser-origin guards, and the §6 endpoints.

Every response carries the §6.0 envelope (``{"ok": true, ...}`` on success,
``{"ok": false, "error": <code>, "detail": ...}`` on failure) and the §5.7
security headers. No CORS header is ever emitted; requests whose ``Host`` /
``Origin`` / ``Sec-Fetch-Site`` do not prove a same-origin localhost page
are refused before any routing happens. An unauthenticated localhost server
that can command a robot is exactly what a malicious page in another tab
would like to reach.
"""

from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import socket
from urllib.parse import parse_qs

from franka_web import config
from franka_web.gains import GainsError
from franka_web.session import SessionError, SessionRequest
from franka_web.sse import encode_event, safe_json_dumps

_CSP = ("default-src 'self'; connect-src 'self'; img-src 'self' data:; "
        "style-src 'self'; script-src 'self'; base-uri 'none'; "
        "form-action 'none'; frame-ancestors 'none'")

#: An unread request body up to this many bytes is drained so the keep-alive
#: connection stays usable; anything larger simply ends the connection.
_MAX_DRAIN_BYTES = 4096

_CONTENT_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.svg': 'image/svg+xml',
}

_ERROR_STATUS = {
    'operator_lock_held': 409,
    'operator_token_invalid': 401,
    'session_already_active': 409,
    'session_not_active': 409,
    'session_not_running': 409,
    'session_faulted': 409,
    'not_motion_mode': 409,
    'not_production_mode': 409,
    'arm_not_in_session': 404,
    'invalid_arms': 400,
    'invalid_mode': 400,
    'invalid_json': 400,
    'invalid_joint': 400,
    'controller_not_reviewed': 400,
    'gains_required': 400,
    'gains_unknown': 404,
    'gains_too_large': 400,
    'gains_invalid': 400,
    'gains_controller_mismatch': 400,
    'gains_arms_mismatch': 400,
    'robot_addresses_missing': 412,
    'preflight_failed': 412,
    'fence_pose_unverified': 412,
    'gains_preview_mismatch': 412,
    'pose_outside_fence': 412,
    'settling_policy_required': 412,
    'settling_fence_required': 412,
    'settling_margin_unavailable': 412,
    'activation_settling_limit': 502,
    'activation_settling_timeout': 504,
    'joint_state_stale': 412,
    'recording_failed': 500,
    'launch_failed': 500,
    'launch_timeout': 500,
    'enable_service_unavailable': 503,
    'enable_rejected': 502,
    'recovery_service_unavailable': 503,
    'recovery_failed': 502,
    'recovery_not_supported': 409,
    'arm_not_enabled': 409,
    'not_faulted': 409,
    'forbidden_origin': 403,
    'not_found': 404,
    'method_not_allowed': 405,
    'payload_too_large': 413,
    'internal_error': 500,
}


class ApiError(Exception):
    """A request refusal with a closed-set §6.14 code."""

    def __init__(self, code, detail, payload=None):
        """Store the code, HTTP status, detail, and optional safe evidence."""
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.payload = dict(payload or {})
        self.status = _ERROR_STATUS[code]


@dataclass(frozen=True)
class Route:
    """
    One routing-table row; the table drives token enforcement.

    ``path`` may contain the single placeholder segment ``{arm_id}``; a
    matched request hands the captured value to the handler in ``params``.
    """

    method: str
    path: str
    handler: str        # Handler method name
    needs_token: bool
    status: int = 200

    def match(self, path):
        """Return the captured params dict when ``path`` matches, else None."""
        if '{' not in self.path:
            return {} if path == self.path else None
        want = self.path.split('/')
        have = path.split('/')
        if len(want) != len(have):
            return None
        params = {}
        for expected, actual in zip(want, have):
            if expected == '{arm_id}':
                params['arm_id'] = actual
            elif expected != actual:
                return None
        return params


ROUTES = (
    Route('GET', '/api/capabilities', 'handle_capabilities', False),
    Route('POST', '/api/operator/claim', 'handle_claim', False),
    Route('POST', '/api/operator/heartbeat', 'handle_heartbeat', True),
    Route('POST', '/api/operator/release', 'handle_release', True),
    Route('GET', '/api/gains', 'handle_gains_list', False),
    Route('POST', '/api/gains', 'handle_gains_upload', True),
    Route('POST', '/api/session/start', 'handle_session_start', True, 202),
    Route('POST', '/api/session/stop', 'handle_session_stop', True, 202),
    Route('POST', '/api/session/recover', 'handle_session_recover', True),
    Route('GET', '/api/state', 'handle_state', False),
    Route('GET', '/api/state/stream', 'handle_stream', False),
    Route('POST', '/api/arm/{arm_id}/enable', 'handle_arm_enable', True),
    Route('POST', '/api/arm/{arm_id}/jog', 'handle_arm_jog', True),
)


@dataclass
class App:
    """Everything the request handlers need, wired once in server.py."""

    settings: object
    supervisor: object
    lock: object
    broker: object
    static_root: str
    gains_store: object = None


def capabilities_payload(settings):
    """Build the §6.1 capabilities body."""
    policy = settings.activation_settling_policy
    return {
        'ok': True,
        'schema_version': config.SCHEMA_VERSION,
        'server_version': '{} {}'.format(config.SERVER_NAME, config.SERVER_VERSION),
        'arm_selections': ['panda1', 'panda2', 'both'],
        'modes': ['simulate', 'watch', 'motion'],
        'controllers': list(config.WEB_CONTROLLERS),
        'jog_controllers': list(config.JOG_CONTROLLERS),
        'joint_count': config.JOINT_COUNT,
        'jog_step_rad': config.JOG_STEP_RAD,
        'jog_stream_hz': config.JOG_STREAM_HZ,
        'watchdog_timeout_s': config.WATCHDOG_TIMEOUT_S,
        'max_header_age_s': config.MAX_HEADER_AGE_S,
        'max_gains_bytes': config.MAX_GAINS_BYTES,
        'operator_lock_ttl_s': config.OPERATOR_LOCK_TTL_S,
        'state_frame_hz': config.STATE_FRAME_HZ,
        'recording_root': settings.recording_root,
        'ros_domain_id': settings.ros_domain_id,
        'activation_settling_policy': (
            policy.public_payload() if policy is not None else None),
        'transport': 'sse',
    }


class _V6ThreadingHTTPServer(ThreadingHTTPServer):
    """ThreadingHTTPServer for the ::1 bind (stdlib default is IPv4-only)."""

    address_family = socket.AF_INET6


def build_server(app):
    """Create the ThreadingHTTPServer bound per §5.7."""
    if app.settings.bind not in config.ALLOWED_BIND:
        raise ValueError('refusing to bind a non-loopback address')
    handler = make_handler(app)
    server_class = _V6ThreadingHTTPServer if ':' in app.settings.bind else ThreadingHTTPServer
    server = server_class((app.settings.bind, app.settings.port), handler)
    server.daemon_threads = True
    return server


def make_handler(app):
    """Build the request-handler class closed over ``app``."""

    class Handler(BaseHTTPRequestHandler):
        """One HTTP connection; guards first, then table-driven routing."""

        protocol_version = 'HTTP/1.1'
        server_version = 'franka_web'
        sys_version = ''

        #: True once this request's declared body has been taken off the wire.
        #: A class attribute so the settle step is safe on any code path.
        body_read = False

        # -- plumbing ---------------------------------------------------

        def log_message(self, format, *args):  # noqa: A002
            """Silence default request logging (nothing sensitive on stderr)."""

        def __getattr__(self, name):
            """
            Send every unimplemented ``do_<METHOD>`` through the guarded path.

            Left to itself, ``BaseHTTPRequestHandler`` answers a method it
            has no handler for -- ``TRACE``, ``CONNECT``, anything at all --
            with its own 501 HTML page: no origin guard, and none of the §5.7
            headers (verification finding F-5). ``OPTIONS``/``PUT``/
            ``DELETE``/``PATCH`` were written out explicitly; this covers the
            open end of the set, so an unknown method is refused exactly the
            way a wrong known method is.

            ``__getattr__`` runs only when normal lookup fails, so it can
            never shadow the handlers defined above, and it answers for
            nothing but ``do_`` names.
            """
            if name.startswith('do_'):
                return self._reject_method
            raise AttributeError(name)

        def do_GET(self):  # noqa: N802
            """Dispatch a GET."""
            self._dispatch('GET')

        def do_POST(self):  # noqa: N802
            """Dispatch a POST."""
            self._dispatch('POST')

        def do_OPTIONS(self):  # noqa: N802
            """Refuse OPTIONS inside the guard and envelope (no CORS, ever)."""
            self._reject_method()

        def do_PUT(self):  # noqa: N802
            """Refuse PUT inside the guard and envelope."""
            self._reject_method()

        def do_DELETE(self):  # noqa: N802
            """Refuse DELETE inside the guard and envelope."""
            self._reject_method()

        def do_PATCH(self):  # noqa: N802
            """Refuse PATCH inside the guard and envelope."""
            self._reject_method()

        def do_HEAD(self):  # noqa: N802
            """Refuse HEAD with headers only (a HEAD response has no body)."""
            try:
                self._guard_origin()
                status = 405
            except ApiError as error:
                status = error.status
            self.close_connection = True
            self.send_response(status)
            self._security_headers('application/json; charset=utf-8', 0)
            self.end_headers()

        def _reject_method(self):
            """
            Answer an unsupported method with the guarded §6.0 envelope.

            Without this, BaseHTTPRequestHandler answers with its own 501
            HTML page carrying none of the §5.7 headers and skipping the
            origin guard entirely (review finding R18, verification finding
            F-5). Reached from the explicit ``do_*`` refusals above and, via
            ``__getattr__``, from every method name this server does not
            implement -- so the answer is always a §6.14 code, never a 501.
            """
            try:
                self._guard_origin()
            except ApiError as error:
                self._send_error(error)
                return
            path = self.path.split('?', 1)[0]
            if any(route.path == path for route in ROUTES):
                self._send_error(ApiError('method_not_allowed',
                                          'wrong method for this endpoint'))
            else:
                self._send_error(ApiError('not_found', 'no such endpoint'))

        def _dispatch(self, method):
            """Guard, route, and answer one request."""
            self.body_read = False
            # A handler instance may serve several requests on one keep-alive
            # connection. Never let an authorization from the previous
            # request leak into this one.
            self._operator_lease = None
            try:
                self._guard_origin()
                self._guard_framing()
                path = self.path.split('?', 1)[0]
                route, params = self._find_route(method, path)
                if route is None:
                    if method == 'GET' and not path.startswith('/api/'):
                        self._serve_static(path)
                    elif any(r.match(path) is not None for r in ROUTES):
                        raise ApiError('method_not_allowed',
                                       'wrong method for this endpoint')
                    else:
                        raise ApiError('not_found', 'no such endpoint')
                else:
                    if route.needs_token:
                        self._operator_lease = self._require_token()
                    getattr(self, route.handler)(route, params)
                self._settle_body()
            except ApiError as error:
                self._send_error(error)
            except (SessionError, GainsError) as error:
                self._send_error(self._api_error_from(error))
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception:
                self._send_error(ApiError('internal_error', 'internal server error'))

        def _api_error_from(self, error):
            """Map a SessionError/GainsError onto the closed HTTP error set."""
            if error.code in _ERROR_STATUS:
                return ApiError(error.code, error.detail,
                                payload=getattr(error, 'payload', None))
            return ApiError('internal_error', 'internal server error')

        def _find_route(self, method, path):
            """Return (route, params) for (method, path), or (None, None)."""
            for route in ROUTES:
                if route.method != method:
                    continue
                params = route.match(path)
                if params is not None:
                    return route, params
            return None, None

        def _guard_origin(self):
            """Enforce the §5.7 Host / Origin / Sec-Fetch-Site matrix."""
            port = app.settings.port
            allowed_hosts = {'127.0.0.1:{}'.format(port), 'localhost:{}'.format(port)}
            if app.settings.bind == '::1':
                allowed_hosts.add('[::1]:{}'.format(port))
            host = self.headers.get('Host', '')
            if host not in allowed_hosts:
                raise ApiError('forbidden_origin', 'unexpected Host header')
            origin = self.headers.get('Origin')
            if origin is not None:
                allowed_origins = {'http://{}'.format(h) for h in allowed_hosts}
                if origin not in allowed_origins:
                    raise ApiError('forbidden_origin', 'cross-origin request refused')
            fetch_site = self.headers.get('Sec-Fetch-Site')
            if fetch_site is not None and fetch_site not in ('same-origin', 'none'):
                raise ApiError('forbidden_origin', 'cross-site request refused')

        def _require_token(self):
            """Atomically authorize/touch a mutating request and return its lease."""
            token = self.headers.get('X-Operator-Token', '')
            lease = app.lock.authorize(token) if token else None
            if lease is None:
                raise ApiError('operator_token_invalid',
                               'missing, stale, or wrong operator token')
            return lease

        def _guard_framing(self):
            """
            Refuse a request whose body length is not a single plain number.

            Two shapes, one reason. A ``Transfer-Encoding`` body is never
            decoded here -- nor by ``BaseHTTPRequestHandler``, which hands the
            handler a stream still carrying the chunk framing, so the body is
            silently ignored and the chunk bytes are parsed as the next
            request on the connection (verification finding F-1). Repeated
            ``Content-Length`` headers are the same hazard by another route:
            the parser believes the first, and anything reading the second
            disagrees about where this request ends.

            Both are refused before routing, and the refusal closes the
            connection, so neither can leave bytes behind.
            """
            if self.headers.get('Transfer-Encoding') is not None:
                raise ApiError(
                    'invalid_json',
                    'chunked transfer encoding is not supported; send a '
                    'Content-Length body')
            if len(self.headers.get_all('Content-Length') or ()) > 1:
                raise ApiError('invalid_json',
                               'exactly one Content-Length header is allowed')

        def _declared_length(self):
            """Return the declared body length, or None when it is unusable."""
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                return None
            return length if length >= 0 else None

        def _settle_body(self):
            """
            Leave no declared request body unread on a keep-alive connection.

            Most routes here take no body at all, and a handler that never
            reads one leaves those bytes sitting in front of the next request:
            the stdlib then parses them AS a request, and one connection
            carries two responses (verification finding F-1). A small
            leftover is drained so the connection stays reusable; anything
            bigger, or any body whose length cannot be trusted, ends the
            connection instead. Refusals need none of this -- ``_send_error``
            always closes.
            """
            if self.body_read or self.close_connection:
                return
            length = self._declared_length()
            if length is None:
                self.close_connection = True
                return
            if length == 0:
                return
            if length > _MAX_DRAIN_BYTES:
                self.close_connection = True
                return
            try:
                drained = self.rfile.read(length)
            except OSError:
                self.close_connection = True
                return
            if len(drained) != length:
                # The client declared more than it sent; the framing of
                # anything that follows is no longer knowable.
                self.close_connection = True
            else:
                self.body_read = True

        def _read_json_body(self):
            """Read and parse the request body (empty means {})."""
            length_text = self.headers.get('Content-Length', '0')
            try:
                length = int(length_text)
            except ValueError:
                raise ApiError('invalid_json', 'bad Content-Length') from None
            if length > config.MAX_GAINS_BYTES:
                raise ApiError('payload_too_large', 'request body too large')
            if length <= 0:
                # A negative Content-Length frames nothing; leave that to the
                # settle step, which ends a connection it cannot trust.
                self.body_read = length == 0
                return {}
            raw = self.rfile.read(length)
            self.body_read = True
            try:
                body = json.loads(raw.decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError):
                raise ApiError('invalid_json', 'request body is not valid JSON') from None
            if not isinstance(body, dict):
                raise ApiError('invalid_json', 'request body must be a JSON object')
            return body

        def _send_json(self, payload, status=200):
            """Send a JSON body (strictly valid, see sse.safe_json_dumps)."""
            body = safe_json_dumps(payload).encode('utf-8')
            self.send_response(status)
            self._security_headers('application/json; charset=utf-8', len(body))
            self.end_headers()
            self.wfile.write(body)

        def _send_error(self, error):
            """
            Send the §6.0 failure envelope and end the connection.

            A refused POST may leave a declared, unread body on the wire; on
            a keep-alive connection those bytes would be parsed as the next
            request (review finding R10). Closing after every error is the
            simple, always-correct answer -- browsers reconnect transparently.
            """
            self.close_connection = True
            try:
                payload = {'ok': False, 'error': error.code,
                           'detail': error.detail}
                for key, value in error.payload.items():
                    if key not in ('ok', 'error', 'detail'):
                        payload[key] = value
                self._send_json(payload, status=error.status)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _security_headers(self, content_type, length):
            """Emit the always-on response headers (never any CORS header)."""
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(length))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', _CSP)

        # -- static files ----------------------------------------------

        def _serve_static(self, path):
            """
            Serve index.html and its assets from the fixed static root.

            Containment is enforced lexically: every path component must be a
            plain name (no '', '.', '..', backslash, or NUL), so the joined
            path cannot escape the root no matter what the URL says. The
            files themselves may be symlinks INTO the source tree -- that is
            what --symlink-install installs -- which is why realpath
            containment of the target would wrongly refuse legitimate files.
            """
            relative = 'index.html' if path == '/' else path.lstrip('/')
            parts = relative.split('/')
            if any(part in ('', '.', '..') or '\\' in part or '\x00' in part
                   for part in parts):
                raise ApiError('not_found', 'no such file')
            candidate = os.path.join(app.static_root, *parts)
            if os.path.normpath(candidate) != candidate:
                raise ApiError('not_found', 'no such file')
            if not os.path.isfile(candidate):
                raise ApiError('not_found', 'no such file')
            content_type = _CONTENT_TYPES.get(
                os.path.splitext(candidate)[1], 'application/octet-stream')
            with open(candidate, 'rb') as handle:
                body = handle.read()
            self.send_response(200)
            self._security_headers(content_type, len(body))
            self.end_headers()
            self.wfile.write(body)

        # -- endpoints --------------------------------------------------

        def handle_capabilities(self, route, params):
            """§6.1 GET /api/capabilities."""
            self._send_json(capabilities_payload(app.settings))

        def handle_claim(self, route, params):
            """§6.2 POST /api/operator/claim."""
            token = app.lock.claim()
            if token is None:
                raise ApiError('operator_lock_held',
                               'another operator holds control')
            self._send_json({'ok': True, 'token': token,
                             'expires_in_s': config.OPERATOR_LOCK_TTL_S})

        def handle_heartbeat(self, route, params):
            """§6.3 POST /api/operator/heartbeat."""
            token = self.headers.get('X-Operator-Token', '')
            expires = app.lock.heartbeat(token)
            if expires is None:
                raise ApiError('operator_token_invalid', 'stale operator token')
            self._send_json({'ok': True, 'expires_in_s': expires})

        def handle_release(self, route, params):
            """§6.4 POST /api/operator/release."""
            token = self.headers.get('X-Operator-Token', '')
            # release() invokes the registered revocation hook synchronously,
            # before a successor claim can be minted. A second supervisor
            # callback here could instead land after that successor enabled.
            app.lock.release(token)
            self._send_json({'ok': True})

        def handle_gains_list(self, route, params):
            """§6.6 GET /api/gains."""
            entries = app.gains_store.entries() if app.gains_store else []
            self._send_json({'ok': True, 'gains': entries})

        def handle_gains_upload(self, route, params):
            """§6.5 POST /api/gains — raw YAML body, query-string metadata."""
            if app.gains_store is None:
                raise ApiError('internal_error', 'no gains store is wired')
            query = parse_qs(self.path.split('?', 1)[1] if '?' in self.path else '')
            controller_name = (query.get('controller_name') or [''])[0]
            arms = (query.get('arms') or [''])[0]
            length_text = self.headers.get('Content-Length', '0')
            try:
                length = int(length_text)
            except ValueError:
                raise ApiError('gains_invalid', 'bad Content-Length') from None
            if length > config.MAX_GAINS_BYTES:
                raise ApiError('gains_too_large',
                               'the config exceeds {} bytes'.format(
                                   config.MAX_GAINS_BYTES))
            raw = self.rfile.read(length) if length > 0 else b''
            self.body_read = length >= 0
            stored = app.gains_store.upload(raw, controller_name, arms)
            self._send_json({'ok': True, **stored.response()})

        def _arm_request(self, params):
            """Validate the {arm_id} path segment."""
            arm_id = params.get('arm_id', '')
            if arm_id not in ('panda1', 'panda2'):
                raise ApiError('arm_not_in_session',
                               'arm must be panda1 or panda2')
            return arm_id

        def handle_arm_enable(self, route, params):
            """§6.13 POST /api/arm/{arm_id}/enable."""
            arm_id = self._arm_request(params)
            body = self._read_json_body()
            enabled = body.get('enabled')
            if not isinstance(enabled, bool):
                raise ApiError('invalid_json', "body must carry 'enabled': true|false")
            result = app.supervisor.request_arm_enable(
                arm_id, enabled, operator_lease=self._operator_lease)
            self._send_json({'ok': True, **result})

        def handle_arm_jog(self, route, params):
            """§6.13 POST /api/arm/{arm_id}/jog — one fixed ±step."""
            arm_id = self._arm_request(params)
            body = self._read_json_body()
            joint_index = body.get('joint_index')
            direction = body.get('direction')
            if isinstance(joint_index, bool) or not isinstance(joint_index, int):
                raise ApiError('invalid_json', "'joint_index' must be an integer 0..6")
            if (isinstance(direction, bool) or not isinstance(direction, int)
                    or direction not in (-1, 1)):
                raise ApiError('invalid_json', "'direction' must be -1 or 1")
            result = app.supervisor.request_arm_jog(
                arm_id, joint_index, direction,
                operator_lease=self._operator_lease)
            self._send_json({'ok': True, **result})

        def handle_session_recover(self, route, params):
            """§6.13 POST /api/session/recover — restore the full session."""
            result = app.supervisor.request_session_recover(
                operator_lease=self._operator_lease)
            self._send_json({'ok': True, **result})

        def handle_session_start(self, route, params):
            """§6.7 POST /api/session/start."""
            body = self._read_json_body()
            request = SessionRequest(
                arms=body.get('arms'),
                mode=body.get('mode'),
                controller_name=body.get('controller_name'),
                gains_sha256=body.get('gains_sha256'))
            result = app.supervisor.request_start(
                request, operator_lease=self._operator_lease)
            self._send_json({'ok': True, **result}, status=route.status)

        def handle_session_stop(self, route, params):
            """§6.8 POST /api/session/stop (advisory, always)."""
            result = app.supervisor.request_stop()
            self._send_json({'ok': True, 'advisory': config.STOP_ADVISORY, **result},
                            status=route.status)

        def handle_state(self, route, params):
            """§6.10 GET /api/state — the polling fallback."""
            self._send_json({'ok': True, 'state': app.supervisor.frame()})

        def handle_stream(self, route, params):
            """§6.11 GET /api/state/stream — the SSE fan-out."""
            subscription = app.broker.subscribe()
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Content-Security-Policy', _CSP)
                self.send_header('X-Accel-Buffering', 'no')
                self.send_header('Connection', 'close')
                self.end_headers()
                self.wfile.write(encode_event('state', app.supervisor.frame()))
                self.wfile.flush()
                while True:
                    chunk = subscription.get(1.0)
                    if chunk is None:
                        # A closed subscription returns None immediately;
                        # treat it as end-of-stream, never as a retry, or
                        # this thread would spin at full speed forever.
                        if subscription.closed:
                            break
                        continue
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ValueError):
                pass
            finally:
                app.broker.unsubscribe(subscription)

    return Handler
