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
A real HTTP server on a loopback port, wired to a ghost service and doubles.

The transport is real for the same reason the rest of the HTTP suite's is:
every property the ghost routes are responsible for is a property of bytes on
the wire -- which status, which headers, whether a token is read at all --
and none of those survives being asserted against a hand-built handler.
"""

import http.client
import json
import os
import socket
import threading

from franka_web.http_api import App, build_server
from franka_web.lock import OperatorLock
from franka_web.logbus import LogBus
from franka_web.sse import Broker
from support.config_factory import make_settings

STATIC_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '..', 'static')

REQUEST_TIMEOUT_S = 10.0


class Reply:
    """One completed response: status, headers and body."""

    def __init__(self, status, headers, body):
        """Wrap the status line, the header object and the body bytes."""
        self.status = status
        self.headers = headers
        self.body = body

    def header(self, name):
        """Return one header value (case-insensitive), or None."""
        return self.headers.get(name)

    def json(self):
        """Decode the body as JSON."""
        return json.loads(self.body.decode('utf-8'))


def free_port():
    """Return a loopback port that was free a moment ago."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


class GhostServer:
    """The real server, serving the three ghost routes against doubles."""

    def __init__(self, tmp_path, ghost, supervisor=None, static_root=None):
        """Bind, wire and start serving on a daemon thread."""
        state_dir = tmp_path / 'state'
        state_dir.mkdir(mode=0o700, exist_ok=True)
        recordings = tmp_path / 'recordings'
        recordings.mkdir(mode=0o700, exist_ok=True)
        self.logs = LogBus()
        self.ghost = ghost
        self.httpd = None
        self.settings = None
        for _ in range(10):
            settings = make_settings(tmp_path, bind='127.0.0.1',
                                     port=free_port())
            self.app = App(settings=settings, supervisor=supervisor,
                           lock=OperatorLock(), broker=Broker(),
                           static_root=static_root or os.path.normpath(STATIC_ROOT),
                           log_bus=self.logs, ghost=ghost)
            try:
                self.httpd = build_server(self.app)
            except OSError:
                continue
            self.settings = settings
            break
        assert self.httpd is not None, 'could not bind a free loopback port'
        self.thread = threading.Thread(
            target=self.httpd.serve_forever, args=(0.02,),
            name='test-ghost-http', daemon=True)
        self.thread.start()

    @property
    def port(self):
        """Return the bound port."""
        return self.settings.port

    def request(self, method, path, body=None, headers=None):
        """Send one request and return the reply."""
        headers = dict(headers or {})
        if isinstance(body, (dict, list)):
            body = json.dumps(body)
            headers.setdefault('Content-Type', 'application/json')
        if isinstance(body, str):
            body = body.encode('utf-8')
        connection = http.client.HTTPConnection(
            '127.0.0.1', self.port, timeout=REQUEST_TIMEOUT_S)
        try:
            connection.request(method, path, body=body, headers=headers)
            response = connection.getresponse()
            return Reply(response.status, response.headers, response.read())
        finally:
            connection.close()

    def post(self, path, body, headers=None):
        """Send one JSON POST and return the reply."""
        return self.request('POST', path, body=body, headers=headers)

    def close(self):
        """Stop serving and release the port."""
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=5)
