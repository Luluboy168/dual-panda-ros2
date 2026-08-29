#!/usr/bin/env python3
# [THROWAWAY] Session C browser-test runner; franka_web owns its browser harness.
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


"""Run the pure-JavaScript ghost suite in headless Chromium."""

from __future__ import annotations

import argparse
from html.parser import HTMLParser
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
from threading import Thread
import time
from typing import Any


class _VerdictParser(HTMLParser):
    """Extract text from the harness's single verdict element."""

    def __init__(self) -> None:
        super().__init__()
        self._inside = False
        self.text: list[str] = []

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        attributes = dict(attrs)
        if tag == 'pre' and attributes.get('id') == 'verdict':
            self._inside = True

    def handle_endtag(self, tag: str) -> None:
        if tag == 'pre' and self._inside:
            self._inside = False

    def handle_data(self, data: str) -> None:
        if self._inside:
            self.text.append(data)


def _parse_verdict(document: str) -> dict[str, Any]:
    parser = _VerdictParser()
    parser.feed(document)
    if not parser.text:
        raise RuntimeError('headless browser output contained no #verdict element')
    verdict = json.loads(''.join(parser.text))
    if not isinstance(verdict, dict):
        raise RuntimeError('browser verdict was not a JSON object')
    return verdict


def _tail_output(output: str | bytes | None) -> str:
    """Return bounded diagnostic text from normal or timeout subprocess data."""
    if output is None:
        return ''
    if isinstance(output, bytes):
        output = output.decode(errors='replace')
    return output[-2000:]


def _profile_processes(profile: str) -> list[int]:
    """Find Chromium processes belonging to one unique test profile."""
    needle = f'--user-data-dir={profile}'.encode()
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


def _stop_browser_processes(process: subprocess.Popen[str], profile: str) -> None:
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


def run_browser_suite(timeout: float = 35.0) -> dict[str, Any]:
    """Serve the worktree, launch Chromium, and return its JSON verdict."""
    browser = shutil.which('chromium') or shutil.which('chromium-browser')
    if browser is None:
        raise RuntimeError('Chromium executable not found')

    repository_root = Path(__file__).resolve().parents[3]
    package_root = repository_root / 'franka_ghost'
    sys.path.insert(0, str(package_root))
    from franka_ghost.dev_server import create_server

    server = create_server(
        port=0,
        web_root=repository_root,
        source=None,
        source_name='demo',
    )
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    harness_url = (
        f'http://127.0.0.1:{server.server_port}/'
        'franka_ghost/test/browser/harness.html'
    )

    try:
        # Chromium's profile must stay under $HOME for snap confinement, and it must land
        # somewhere a package-local .gitignore can cover: a killed run leaves the directory
        # behind, and at the worktree root only the reviewed repo-root .gitignore could hide
        # it. package_root satisfies both, and franka_ghost/.gitignore ignores the prefix.
        with tempfile.TemporaryDirectory(
            prefix='.ghost-chromium-', dir=package_root
        ) as profile:
            command = [
                browser,
                '--headless=new',
                '--enable-unsafe-swiftshader',
                '--disable-background-networking',
                '--disable-default-apps',
                '--no-first-run',
                '--no-sandbox',
                '--virtual-time-budget=20000',
                f'--user-data-dir={profile}',
                '--dump-dom',
                harness_url,
            ]
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
            try:
                stdout, stderr = process.communicate(timeout=timeout)
                completed = subprocess.CompletedProcess(
                    command, process.returncode, stdout, stderr
                )
            except subprocess.TimeoutExpired as error:
                # The snap launcher can leave Chromium outside its original
                # process group. The unique profile is a reliable ownership key.
                _stop_browser_processes(process, profile)
                try:
                    _stdout, stderr = process.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        process.kill()
                    except (PermissionError, ProcessLookupError):
                        pass
                    try:
                        _stdout, stderr = process.communicate(timeout=3)
                    except subprocess.TimeoutExpired:
                        stderr = error.stderr
                return {
                    'ok': False,
                    'tests': 0,
                    'failures': [{
                        'name': 'driver hard timeout',
                        'error': f'Chromium exceeded {timeout:.1f} seconds',
                    }],
                    'browser_problems': [],
                    'stderr': _tail_output(stderr or error.stderr),
                }
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=5)

    if completed.returncode != 0:
        return {
            'ok': False,
            'tests': 0,
            'failures': [{
                'name': 'Chromium process',
                'error': f'exit code {completed.returncode}',
            }],
            'browser_problems': [],
            'stderr': _tail_output(completed.stderr),
        }
    try:
        return _parse_verdict(completed.stdout)
    except (RuntimeError, json.JSONDecodeError) as error:
        return {
            'ok': False,
            'tests': 0,
            'failures': [{'name': 'verdict parsing', 'error': str(error)}],
            'browser_problems': [],
            'stderr': _tail_output(completed.stderr),
        }


def main() -> int:
    """Run the suite and print exactly one machine-readable verdict line."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=35.0)
    args = parser.parse_args()
    verdict = run_browser_suite(timeout=args.timeout)
    print(json.dumps(verdict, sort_keys=True))
    return 0 if verdict.get('ok') is True else 1


if __name__ == '__main__':
    raise SystemExit(main())
