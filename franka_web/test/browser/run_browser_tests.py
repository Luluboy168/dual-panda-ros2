#!/usr/bin/env python3
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
Run the browser suite in headless Chromium, under the production policy.

Three things make this harness worth trusting.

It serves ONE origin with the production layout: the console's own static
tree at ``/``, the generated scene assets at ``/ghost/assets/``, and the
harness itself under ``/test/browser/``. Every path the page fetches is
therefore the path production serves, so a case cannot pass against a
layout the console does not have.

It serves the production Content-Security-Policy, IMPORTED from the server
rather than restated. The suite's whole CSP story rests on a violation
listener staying silent; a listener that can never fire because the page
carries no policy proves nothing at all, and a policy copied here would drift
from the one that ships.

Its stub responder for the three ghost routes is the SERVER'S OWN CODE, wired
to a solver that returns the seed. The payload shapes therefore cannot drift
from the ones the Python gates assert, because they are produced by the same
module.
"""

from __future__ import annotations

import argparse
from html.parser import HTMLParser
import http.server
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from franka_web import defaults                                  # noqa: E402
from franka_web.ghost import GhostService, IkReply               # noqa: E402
from franka_web.ghost_assets import (                            # noqa: E402
    installed_manifest_summary)
from franka_web.http_api import _CSP                             # noqa: E402

STATIC_ROOT = PACKAGE_ROOT / 'static'
HARNESS_ROOT = PACKAGE_ROOT / 'test' / 'browser'
HARNESS_URL_PREFIX = '/test/browser/'

CONTENT_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.urdf': 'application/xml',
    '.bin': 'application/octet-stream',
    '.txt': 'text/plain; charset=utf-8',
    '.woff2': 'font/woff2',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
    '.ico': 'image/vnd.microsoft.icon',
}


class StubChecker:
    """
    A checker that answers, so the drag cases see a real verdict shape.

    It is deliberately not a cell model: the browser suite proves the page's
    geometry and plumbing, and the verdict's own arithmetic is proved in
    Python, against the real one.
    """

    CELL = {'id': 'work_area', 'frame': 'cell',
            'x_min': -0.35, 'x_max': 0.9, 'y_min': -1.0, 'y_max': 1.0,
            'z_min': 0.0, 'z_max': 2.0}

    def status(self, profile):
        """Return the scene payload's cell and checker blocks."""
        return {'available': True, 'profile': profile,
                'interlock': 'not_checked', 'checker_note': None,
                'cell': dict(self.CELL), 'cell_source': 'cell_model',
                'cell_note': None,
                'model': {'model_id': 'browser_stub_cell',
                          'model_revision': 1, 'model_sha256': '0' * 64}}

    def check(self, profile, scene):
        """Report every pose as clear; the sentences are proved elsewhere."""
        return _ClearResult(), None, None


class _ClearResult:
    """The two result fields the verdict adapter reads on a clear pose."""

    ok = True
    min_clearance = 0.041
    contacts = ()


def identity_solver(call, timeout_s):
    """Answer every call with the seed it was given."""
    return IkReply(result=0, message='', positions=tuple(call.seed_positions),
                   redundancy_value=call.redundancy_value,
                   position_error=0.0, orientation_error=0.0)


def build_ghost_service():
    """Return the server's own ghost service, wired to deterministic doubles."""
    return GhostService(
        solver=identity_solver,
        checker=StubChecker(),
        session_view=lambda: {
            'arm_ids': list(defaults.ARM_IDS),
            'command_topics': {
                arm_id: '/{}/arm_{}/joint_target'.format(
                    defaults.MOTION_CONTROLLER, slot)
                for slot, arm_id in enumerate(defaults.ARM_IDS, start=1)}},
        ik_ready=lambda: True)


def asset_directory():
    """Return a directory holding the generated assets, or None."""
    installed = None
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('franka_web')) / 'static'
    except Exception:                     # noqa: BLE001 - not built is normal
        installed = None
    if installed is not None:
        candidate = installed / 'ghost' / 'assets'
        if (candidate / 'manifest.json').is_file():
            return candidate
    candidate = STATIC_ROOT / 'ghost' / 'assets'
    if (candidate / 'manifest.json').is_file():
        return candidate
    return None


def make_handler(ghost, assets):
    """Build the request handler for one run."""

    class Handler(http.server.BaseHTTPRequestHandler):
        """Serves the static tree, the assets, the harness and three routes."""

        protocol_version = 'HTTP/1.1'
        server_version = 'franka_web_browser_harness'
        sys_version = ''

        def log_message(self, format, *args):  # noqa: A002
            """Stay quiet; the verdict is the only output that matters."""

        def do_GET(self):  # noqa: N802
            """Serve one file, or the scene payload."""
            path = self.path.split('?', 1)[0]
            if path == '/api/scene':
                summary = (installed_manifest_summary(str(STATIC_ROOT))
                           or _asset_summary(assets))
                self._json({'ok': True, **ghost.scene(summary)})
                return
            self._file(path)

        def do_POST(self):  # noqa: N802
            """Answer the two compute routes from the server's own code."""
            path = self.path.split('?', 1)[0]
            length = int(self.headers.get('Content-Length', '0') or 0)
            raw = self.rfile.read(length) if length else b'{}'
            try:
                body = json.loads(raw.decode('utf-8') or '{}')
            except ValueError:
                self._json({'ok': False, 'error': 'invalid_json',
                            'detail': 'the body is not JSON'}, status=400)
                return
            try:
                if path == '/api/ghost/solve':
                    self._json({'ok': True, **ghost.solve(body)})
                elif path == '/api/ghost/redundancy':
                    self._json({'ok': True, **ghost.redundancy(body)})
                else:
                    self._json({'ok': False, 'error': 'not_found',
                                'detail': 'no such endpoint'}, status=404)
            except Exception as error:    # noqa: BLE001 - reported as a body
                code = getattr(error, 'code', 'internal_error')
                payload = {'ok': False, 'error': code,
                           'detail': str(error)}
                payload.update(getattr(error, 'payload', {}) or {})
                self._json(payload, status=400)

        # -- plumbing --------------------------------------------------

        def _headers(self, content_type, length):
            """Emit the production security headers on EVERY response."""
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(length))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', _CSP)

        def _json(self, payload, status=200):
            """Send one JSON body."""
            body = json.dumps(payload).encode('utf-8')
            self.send_response(status)
            self._headers('application/json; charset=utf-8', len(body))
            self.end_headers()
            self.wfile.write(body)

        def _resolve(self, path):
            """Map one URL path onto a file, or None."""
            relative = path.lstrip('/')
            parts = [part for part in relative.split('/') if part]
            if any(part in ('.', '..') or '\\' in part or '\x00' in part
                   for part in parts):
                return None
            if path.startswith(HARNESS_URL_PREFIX):
                return HARNESS_ROOT.joinpath(
                    *parts[len(HARNESS_URL_PREFIX.strip('/').split('/')):])
            prefix = defaults.GHOST_ASSET_PREFIX.strip('/').split('/')
            if assets is not None and parts[:len(prefix)] == prefix:
                return Path(assets).joinpath(*parts[len(prefix):])
            return STATIC_ROOT.joinpath(*parts) if parts else None

        def _file(self, path):
            """Serve one file from the mapped root."""
            target = self._resolve(path)
            if target is None or not target.is_file():
                self._json({'ok': False, 'error': 'not_found',
                            'detail': path}, status=404)
                return
            body = target.read_bytes()
            self.send_response(200)
            self._headers(CONTENT_TYPES.get(target.suffix,
                                            'application/octet-stream'),
                          len(body))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def _asset_summary(assets):
    """Return the scene's asset block for a directory outside the static tree."""
    if assets is None:
        return None
    manifest = Path(assets) / 'manifest.json'
    try:
        document = json.loads(manifest.read_text(encoding='utf-8'))
        digest = document['generated_from']['urdf_sha256']
    except (OSError, ValueError, KeyError):
        return None
    base = '/' + defaults.GHOST_ASSET_PREFIX
    total = sum(item.stat().st_size for item in Path(assets).rglob('*')
                if item.is_file())
    return {'manifest_url': base + 'manifest.json',
            'urdf_url': base + 'model.urdf',
            'asset_base': base,
            'urdf_sha256': str(digest),
            'total_bytes': total}


class _VerdictParser(HTMLParser):
    """Extract the text of the harness's single verdict element."""

    def __init__(self) -> None:
        """Start outside the element, with nothing collected."""
        super().__init__()
        self._inside = False
        self.text: list[str] = []

    def handle_starttag(self, tag, attrs):
        """Enter the verdict element."""
        if tag == 'pre' and dict(attrs).get('id') == 'verdict':
            self._inside = True

    def handle_endtag(self, tag):
        """Leave the verdict element."""
        if tag == 'pre' and self._inside:
            self._inside = False

    def handle_data(self, data):
        """Collect the verdict text."""
        if self._inside:
            self.text.append(data)


def parse_verdict(document: str) -> dict:
    """Return the harness's JSON verdict, or raise."""
    parser = _VerdictParser()
    parser.feed(document)
    if not parser.text:
        raise RuntimeError('the browser output carried no verdict element')
    verdict = json.loads(''.join(parser.text))
    if not isinstance(verdict, dict):
        raise RuntimeError('the browser verdict was not a JSON object')
    return verdict


def _tail(output, limit=2000):
    """Return bounded diagnostic text from subprocess output."""
    if output is None:
        return ''
    if isinstance(output, bytes):
        output = output.decode(errors='replace')
    return output[-limit:]


def _profile_processes(profile: str):
    """Find Chromium processes belonging to one unique test profile."""
    needle = '--user-data-dir={}'.format(profile).encode()
    matches = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            arguments = (entry / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if needle in arguments:
            matches.append(int(entry.name))
    return matches


def _stop_browser(process, profile):
    """Stop the launcher and every browser process using the test profile."""
    if process.poll() is None:
        try:
            process.terminate()
        except (PermissionError, ProcessLookupError):
            pass
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        pids = _profile_processes(profile)
        if not pids:
            return
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except (PermissionError, ProcessLookupError):
                pass
        time.sleep(0.05)
    for pid in _profile_processes(profile):
        try:
            os.kill(pid, signal.SIGKILL)
        except (PermissionError, ProcessLookupError):
            pass


def harness_path() -> Path:
    """Return the harness page, which PART-S owns."""
    return HARNESS_ROOT / 'harness.html'


def run_browser_suite(timeout: float = 60.0) -> dict[str, Any]:
    """Serve one origin, run Chromium against it, and return the verdict."""
    browser = shutil.which('chromium') or shutil.which('chromium-browser')
    if browser is None:
        raise RuntimeError('Chromium executable not found')
    if not harness_path().is_file():
        raise RuntimeError('the browser harness page is not present')

    handler = make_handler(build_ghost_service(), asset_directory())
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, args=(0.02,),
                              daemon=True)
    thread.start()
    url = 'http://127.0.0.1:{}{}harness.html'.format(
        server.server_port, HARNESS_URL_PREFIX)
    try:
        # Chromium's profile must stay under a writable directory the run
        # owns: a killed run leaves it behind, and a unique path is also the
        # only reliable key for finding the processes it spawned.
        with tempfile.TemporaryDirectory(prefix='.ghost-chromium-') as profile:
            command = [
                browser,
                '--headless=new',
                '--enable-unsafe-swiftshader',
                '--disable-background-networking',
                '--disable-default-apps',
                '--no-first-run',
                '--no-sandbox',
                '--virtual-time-budget=30000',
                '--user-data-dir={}'.format(profile),
                '--dump-dom',
                url,
            ]
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, start_new_session=True)
            try:
                stdout, stderr = process.communicate(timeout=timeout)
            except subprocess.TimeoutExpired as expired:
                _stop_browser(process, profile)
                try:
                    _out, stderr = process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    _out, stderr = process.communicate(timeout=5)
                return {'ok': False, 'tests': 0,
                        'failures': [{'name': 'driver hard timeout',
                                      'error': 'Chromium exceeded {:.1f} s'.format(
                                          timeout)}],
                        'browser_problems': [],
                        'stderr': _tail(stderr or expired.stderr)}
            returncode = process.returncode
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)

    if returncode != 0:
        return {'ok': False, 'tests': 0,
                'failures': [{'name': 'Chromium process',
                              'error': 'exit code {}'.format(returncode)}],
                'browser_problems': [], 'stderr': _tail(stderr)}
    try:
        return parse_verdict(stdout)
    except (RuntimeError, ValueError) as error:
        return {'ok': False, 'tests': 0,
                'failures': [{'name': 'verdict parsing', 'error': str(error)}],
                'browser_problems': [], 'stderr': _tail(stderr)}


def main():
    """Run the suite and print exactly one machine-readable verdict line."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=60.0)
    arguments = parser.parse_args()
    verdict = run_browser_suite(timeout=arguments.timeout)
    print(json.dumps(verdict, sort_keys=True))
    return 0 if verdict.get('ok') is True else 1


if __name__ == '__main__':
    raise SystemExit(main())
