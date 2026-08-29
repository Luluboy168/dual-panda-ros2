# [THROWAWAY] Session C standalone HTTP scaffold; franka_web owns the merged server.
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

"""Dependency-free HTTP/1.1 scaffold for the standalone ghost prototype."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Tuple
from urllib.parse import unquote_to_bytes, urlsplit

from .joint_source import URDF_LOWER, URDF_UPPER


STATE_SCHEMA = 'franka.ghost.state/1'
APPLY_SCHEMA = 'franka.ghost.apply/1'
MAX_BODY_BYTES = 1024 * 1024
STALE_AFTER_S = 0.5
STATIC_MEDIA_TYPES = {
    '.js': 'application/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.html': 'text/html; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.urdf': 'application/xml; charset=utf-8',
    '.bin': 'application/octet-stream',
}
FORBIDDEN_APPLY_FIELDS = {
    'velocities',
    'accelerations',
    'effort',
    'efforts',
    'time_from_start',
    'frame_id',
    'duration',
    'emitted_at',
    'stamp',
    'stamp_ns',
    'timestamp',
    'timestamp_ns',
    'wall_clock',
    'wall_clock_ns',
}


def _is_finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _seven_finite(values: object) -> bool:
    return isinstance(values, list) and len(values) == 7 and all(
        _is_finite_number(value) for value in values
    )


def _forbidden_paths(value: object, path: str = '$') -> Iterable[str]:
    if isinstance(value, Mapping):
        for key, child in value.items():
            key_text = str(key)
            lowered = key_text.lower()
            if (
                lowered in FORBIDDEN_APPLY_FIELDS
                or lowered.endswith('_timestamp')
                or lowered.endswith('_timestamp_ns')
            ):
                yield f'{path}.{key_text}'
            yield from _forbidden_paths(child, f'{path}.{key_text}')
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _forbidden_paths(child, f'{path}[{index}]')


def validate_apply_event(
    event: object, previous_epoch: Optional[int] = None
) -> List[str]:
    """Validate Contract C1 invariants I1 through I7 without publishing anything."""
    if not isinstance(event, dict):
        return ['C1: request body must be a JSON object']

    errors: List[str] = []
    if event.get('schema') != APPLY_SCHEMA:
        errors.append(f'C1: schema must be {APPLY_SCHEMA!r}')

    arm_id = event.get('arm_id')
    joint_names = event.get('joint_names')
    expected_names = [f'{arm_id}_joint{joint}' for joint in range(1, 8)]
    if joint_names != expected_names:
        errors.append(f'I1: joint_names must equal {expected_names!r}')

    positions = event.get('positions')
    positions_valid = _seven_finite(positions)
    if not positions_valid:
        errors.append('I2: positions must contain exactly 7 finite numbers')

    fence = event.get('fence')
    lower = fence.get('lower') if isinstance(fence, dict) else None
    upper = fence.get('upper') if isinstance(fence, dict) else None
    lower_valid = _seven_finite(lower)
    upper_valid = _seven_finite(upper)
    if not lower_valid or not upper_valid:
        errors.append('I3: fence lower and upper must each contain exactly 7 finite numbers')
    elif positions_valid:
        for index, (low, position, high) in enumerate(zip(lower, positions, upper)):
            if low > high:
                errors.append(f'I3: fence.lower[{index}]={low} > fence.upper[{index}]={high}')
            elif position < low:
                errors.append(f'I3: positions[{index}]={position} < fence.lower[{index}]={low}')
            elif position > high:
                errors.append(f'I3: positions[{index}]={position} > fence.upper[{index}]={high}')

    fence_source = fence.get('source') if isinstance(fence, dict) else None
    if fence_source not in ('urdf', 'session'):
        errors.append("I4: fence.source must be 'urdf' or 'session'")
    if lower_valid and upper_valid:
        for index, (low, high, urdf_low, urdf_high) in enumerate(
            zip(lower, upper, URDF_LOWER, URDF_UPPER)
        ):
            if low < urdf_low:
                errors.append(f'I4: fence.lower[{index}]={low} < URDF lower {urdf_low}')
            if high > urdf_high:
                errors.append(f'I4: fence.upper[{index}]={high} > URDF upper {urdf_high}')

    arm_index = event.get('arm_index')
    arm_index_valid = (
        isinstance(arm_index, int) and not isinstance(arm_index, bool) and arm_index in (1, 2)
    )
    expected_arm_id = f'panda{arm_index}' if arm_index_valid else None
    if not arm_index_valid or arm_id != expected_arm_id:
        errors.append('I5: arm_index must be 1 or 2 and agree with arm_id panda1 or panda2')

    epoch = event.get('ghost_epoch')
    epoch_valid = isinstance(epoch, int) and not isinstance(epoch, bool) and epoch >= 1
    if not epoch_valid:
        errors.append('I6: ghost_epoch must be a positive integer')
    elif previous_epoch is not None and epoch <= previous_epoch:
        errors.append(
            f'I6: ghost_epoch={epoch} must be greater than previous epoch {previous_epoch}'
        )

    forbidden = list(_forbidden_paths(event))
    if forbidden:
        errors.append(f"I7: prohibited command/timing fields present at {', '.join(forbidden)}")

    measured = event.get('measured_at_apply')
    if not _seven_finite(measured):
        errors.append('C1: measured_at_apply must contain exactly 7 finite numbers')

    return errors


class StateCache:
    """Thread-safe latest-sample cache and C4 state-frame formatter."""

    def __init__(
        self,
        source: str,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if source not in ('demo', 'ros'):
            raise ValueError("state source must be 'demo' or 'ros'")
        self._source = source
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self._stamp_ns: Optional[int] = None
        self._joints: Dict[str, float] = {}
        self._received_at: Optional[float] = None
        self._seq = 0
        self._sample_key: Optional[Tuple[object, ...]] = None

    def update(self, stamp_ns: Optional[int], joints: Mapping[str, float]) -> bool:
        """Promote a distinct newest sample; return whether the state sequence advanced."""
        if stamp_ns is None:
            return False
        copied = {str(name): float(position) for name, position in joints.items()}
        sample_key = (int(stamp_ns), tuple(sorted(copied.items())))
        with self._lock:
            if sample_key == self._sample_key:
                return False
            self._sample_key = sample_key
            self._stamp_ns = int(stamp_ns)
            self._joints = copied
            self._received_at = self._monotonic()
            self._seq += 1
            return True

    def snapshot(self) -> dict:
        """Return a JSON-serialisable C4 state frame."""
        now = self._monotonic()
        with self._lock:
            if self._received_at is None:
                age_s = 0.0
                stale = True
            else:
                age_s = max(0.0, float(now - self._received_at))
                stale = age_s > STALE_AFTER_S
            return {
                'schema': STATE_SCHEMA,
                'source': self._source,
                'stamp_ns': self._stamp_ns,
                'age_s': age_s,
                'stale': stale,
                'joints': dict(self._joints),
                'seq': self._seq,
            }


class SourcePump:
    """Promote at most one newest source sample per configured output period."""

    def __init__(self, source: object, state: StateCache, rate_hz: float) -> None:
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError('rate must be a positive finite number')
        self._source = source
        self._state = state
        self._period_s = 1.0 / rate_hz
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name='franka-ghost-source-pump',
            daemon=True,
        )

    def start(self) -> None:
        """Start the rate-limited promotion loop."""
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            stamp_ns, joints = self._source.latest()
            self._state.update(stamp_ns, joints)
            self._stop.wait(self._period_s)

    def stop(self) -> None:
        """Stop the loop and close its source, idempotently."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=max(1.0, 2.0 * self._period_s))
        close = getattr(self._source, 'close', None)
        if close is not None:
            close()


class GhostDevServer(ThreadingHTTPServer):
    """Loopback-only server carrying shared state for request handlers."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        port: int,
        web_root: Path,
        state: StateCache,
        source_pump: Optional[SourcePump] = None,
    ) -> None:
        self.web_root = web_root.resolve(strict=True)
        if not self.web_root.is_dir():
            raise ValueError(f'web root is not a directory: {self.web_root}')
        self.state = state
        self.source_pump = source_pump
        self.apply_lock = threading.Lock()
        self.last_apply_epoch: Optional[int] = None
        self._closed = False
        super().__init__(('127.0.0.1', port), GhostRequestHandler)

    def validate_and_record_apply(self, event: object) -> List[str]:
        """Atomically validate the server-wide epoch and record accepted events."""
        # Server-wide, not per-arm, on purpose: Contract C1/I6 scopes ghost_epoch to a
        # single mount(), and ghost.js's lastEpoch is mount-global to match. One counter
        # per server is the correct shape; do not "fix" it into a per-arm map.
        with self.apply_lock:
            errors = validate_apply_event(event, previous_epoch=self.last_apply_epoch)
            if not errors:
                self.last_apply_epoch = event['ghost_epoch']
            return errors

    def reset_apply_epochs(self) -> None:
        """Start a fresh mount-scoped epoch when the standalone page reloads."""
        with self.apply_lock:
            self.last_apply_epoch = None

    def server_close(self) -> None:
        """Stop the source pump before releasing the listening socket."""
        if self._closed:
            return
        self._closed = True
        if self.source_pump is not None:
            self.source_pump.stop()
        super().server_close()


class GhostRequestHandler(BaseHTTPRequestHandler):
    """C4 routes with explicit lengths for persistent HTTP/1.1 connections."""

    protocol_version = 'HTTP/1.1'
    server: GhostDevServer

    def log_message(self, format_string: str, *args: object) -> None:
        """Keep unit tests and the throwaway operator console quiet."""

    def _send_bytes(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _send_json(self, status: int, value: object) -> None:
        body = json.dumps(value, allow_nan=False, separators=(',', ':')).encode('utf-8')
        self._send_bytes(status, body, 'application/json; charset=utf-8')

    def _send_problem(self, status: int, message: str) -> None:
        self._send_json(status, {'error': message})

    def do_GET(self) -> None:
        """Serve the state endpoint or an allowlisted static file."""
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc:
            self._send_problem(403, 'absolute request targets are forbidden')
            return
        if parsed.path == '/state.json':
            self._send_json(200, self.server.state.snapshot())
            return
        self._serve_static(parsed.path)

    def _serve_static(self, encoded_path: str) -> None:
        try:
            decoded_path = unquote_to_bytes(encoded_path).decode('utf-8', errors='strict')
        except UnicodeDecodeError:
            self._send_problem(403, 'invalid path encoding')
            return

        if '..' in decoded_path or '\\' in decoded_path or decoded_path.startswith('//'):
            self._send_problem(403, 'path traversal is forbidden')
            return

        if decoded_path == '/':
            # The throwaway root page creates exactly one ghost mount. Reloading
            # it creates a new mount, whose Contract C1 epoch starts at one.
            self.server.reset_apply_epochs()
        relative = 'index.html' if decoded_path == '/' else decoded_path.removeprefix('/')
        candidate = self.server.web_root / relative
        # Resolve both sides of the containment check to the SAME depth.  The plan's mandated
        # `colcon build --symlink-install` leaves share/franka_ghost/web with real directories but
        # makes every leaf file a symlink back into the source tree.  Resolving the whole candidate
        # therefore lands outside the installed root and 403s every page while /state.json keeps
        # working -- the installed prototype is unusable as a page with the whole suite green.
        # `web_root` is already fully resolved at construction, so resolve the candidate's
        # *directory*, which is a real directory in the source tree and in the installed tree
        # alike.  A directory symlink leaving the web root is still refused; only the leaf may be
        # a link, and its resolved target must still land on an allowlisted type below.
        resolved_parent = candidate.parent.resolve(strict=False)
        if not resolved_parent.is_relative_to(self.server.web_root):
            self._send_problem(403, 'path resolves outside the web root')
            return
        candidate = resolved_parent / candidate.name

        media_type = STATIC_MEDIA_TYPES.get(candidate.suffix)
        if media_type is None:
            self._send_problem(415, 'static file type is not allowlisted')
            return
        # A leaf symlink may point out of the web root (that is what --symlink-install builds),
        # but it may not launder a non-allowlisted file behind an allowlisted name.
        resolved_candidate = candidate.resolve(strict=False)
        if (resolved_candidate != candidate
                and STATIC_MEDIA_TYPES.get(resolved_candidate.suffix) is None):
            self._send_problem(403, 'symlink target is not an allowlisted static file type')
            return
        if not candidate.is_file():
            self._send_problem(404, 'static file not found')
            return
        try:
            body = candidate.read_bytes()
        except OSError:
            self._send_problem(404, 'static file not found')
            return
        self._send_bytes(200, body, media_type)

    def do_POST(self) -> None:
        """Validate and echo C1 events; this endpoint never publishes to ROS."""
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc or parsed.path != '/apply':
            self._send_problem(404, 'endpoint not found')
            return

        content_length = self.headers.get('Content-Length')
        if content_length is None:
            self.close_connection = True
            self._send_problem(411, 'Content-Length is required')
            return
        try:
            length = int(content_length)
        except ValueError:
            self.close_connection = True
            self._send_problem(400, 'invalid Content-Length')
            return
        if length < 0 or length > MAX_BODY_BYTES:
            self.close_connection = True
            self._send_problem(413, 'request body is too large')
            return

        try:
            event = json.loads(self.rfile.read(length))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self._send_problem(400, 'request body must be valid JSON')
            return

        errors = self.server.validate_and_record_apply(event)
        if errors:
            self._send_json(200, {'accepted': False, 'errors': errors})
            return
        self._send_json(
            200,
            {
                'accepted': True,
                'echo': event,
                'note': 'prototype: not published',
            },
        )


def create_server(
    *,
    port: int,
    web_root: Path,
    source: Optional[object] = None,
    source_name: str = 'demo',
    rate_hz: float = 20.0,
    monotonic: Callable[[], float] = time.monotonic,
) -> GhostDevServer:
    """Create a server that can only bind to the literal IPv4 loopback address."""
    actual_source_name = getattr(source, 'source', source_name)
    state = StateCache(source=actual_source_name, monotonic=monotonic)
    pump = SourcePump(source, state, rate_hz) if source is not None else None
    server = GhostDevServer(port=port, web_root=Path(web_root), state=state, source_pump=pump)
    if pump is not None:
        pump.start()
    return server
