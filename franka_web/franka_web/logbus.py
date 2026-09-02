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
The 500-line log ring behind ``GET /api/logs`` and the SSE ``log`` event.

Three sources merge into one ring: the launch child's merged stdout/stderr,
the recorder child's output, and the server's own operator-facing lines. The
two child sources arrive through :meth:`LogBus.sink`, which is handed to
``ChildProcess`` at each spawn site; the server's own lines arrive through
:meth:`LogBus.emit`.

Each captured line is parsed into ``(level, node, message)`` by three rules
tried in order: the standard ROS 2 console format, the launch prefix
(``[node-3] ...``) with the console format re-applied to the remainder, and
finally "everything else is an ``info`` line from ``launch``".

``debug`` lines are captured and served by ``GET /api/logs``, but are never
streamed -- the drawer would be unreadable. They still consume a sequence
number, so a gap in the streamed ``seq`` sequence is ordinary and is not
evidence of a dropped event.

Child output is streamed VERBATIM. A ``ros2 launch`` echoes its
``robot_ip:=`` argument, and that is fine: robot addresses are not secrets,
and the drawer exists to show the operator what the stack actually said.
"""

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import re
import threading

from franka_web import defaults
from franka_web.launcher import OUTPUT_LINE_CHARS, OUTPUT_RING_LINES

#: Every level a captured line can carry, weakest first.
LEVELS = ('debug', 'info', 'warn', 'error')

#: The levels that reach the SSE stream. ``debug`` is captured, never streamed.
STREAMED_LEVELS = ('info', 'warn', 'error')

_LEVEL_NAMES = {
    'DEBUG': 'debug',
    'INFO': 'info',
    'WARN': 'warn',
    'ERROR': 'error',
    # FATAL is folded into error: the drawer has three colours, not four, and
    # a fatal line is an error the operator must act on either way.
    'FATAL': 'error',
}

_ROS_CONSOLE_RE = re.compile(
    r'^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[[0-9.]+\] \[([^\]]+)\]:\s?(.*)$')
_LAUNCH_PREFIX_RE = re.compile(r'^\[([a-zA-Z0-9_.\-]+)-\d+\]\s?(.*)$')

#: Everything except tab is stripped; a driver can print anything at all.
_CONTROL_RE = re.compile(r'[\x00-\x08\x0b-\x1f\x7f]')

_DEFAULT_NODE = 'launch'


def _now():
    """Return the current UTC time as an aware datetime."""
    return datetime.now(timezone.utc)


def _rfc3339(moment):
    """Render an aware UTC datetime as RFC 3339 with microseconds and ``Z``."""
    return moment.strftime('%Y-%m-%dT%H:%M:%S.%f') + 'Z'


def sanitize(text):
    """Strip line endings and control characters, then bound the length."""
    if isinstance(text, (bytes, bytearray)):
        text = bytes(text).decode('utf-8', 'replace')
    elif not isinstance(text, str):
        text = str(text)
    return _CONTROL_RE.sub('', text.rstrip('\r\n'))[:OUTPUT_LINE_CHARS]


def parse_line(text, *, default_node=_DEFAULT_NODE):
    """Return ``(level, node, message)`` for one already-sanitized line."""
    console = _ROS_CONSOLE_RE.match(text)
    if console is not None:
        return (_LEVEL_NAMES[console.group(1)], console.group(2),
                console.group(3))
    prefixed = _LAUNCH_PREFIX_RE.match(text)
    if prefixed is not None:
        remainder = prefixed.group(2)
        inner = _ROS_CONSOLE_RE.match(remainder)
        if inner is not None:
            # The inner name is the real publisher; it wins over the launch
            # prefix, and so do its level and message.
            return (_LEVEL_NAMES[inner.group(1)], inner.group(2), inner.group(3))
        return ('info', prefixed.group(1), remainder)
    return ('info', default_node, text)


@dataclass(frozen=True)
class LogLine:
    """One captured line, numbered and counted."""

    seq: int
    t: str
    level: str
    node: str
    message: str
    warn_count: int
    error_count: int

    def wire(self):
        """Return the ``GET /api/logs`` list entry."""
        return {'seq': self.seq, 't': self.t, 'level': self.level,
                'node': self.node, 'message': self.message}

    def event(self):
        """Return the SSE ``log`` payload: :meth:`wire` plus the counters."""
        payload = self.wire()
        payload['warn_count'] = self.warn_count
        payload['error_count'] = self.error_count
        return payload


class LogBus:
    """
    The bounded ring, the sequence numbers and the cumulative counters.

    One instance per server. Every method is safe to call from any thread:
    child-output reader threads append, the frame pump drains, and HTTP
    worker threads read windows.
    """

    def __init__(self, capacity=OUTPUT_RING_LINES, utcnow=None):
        """Build an empty bus of ``capacity`` lines with an injectable clock."""
        self._capacity = max(1, int(capacity))
        self._ring = deque(maxlen=self._capacity)
        self._lock = threading.Lock()
        self._utcnow = utcnow or _now
        self._seq = 0
        self._warn = 0
        self._error = 0
        self._first_seq = None
        self._pending = []

    # -- production ------------------------------------------------------

    def append(self, text, *, default_node=_DEFAULT_NODE):
        """
        Sanitize, parse and record one captured line; return its record.

        This runs on a child's output reader thread, where an exception would
        silently kill the reader and take the whole drawer with it, so it
        never raises: a line it cannot handle is dropped and ``None`` is
        returned.
        """
        try:
            clean = sanitize(text)
            level, node, message = parse_line(clean, default_node=default_node)
            return self._record(level, node, message)
        except Exception:  # noqa: BLE001 - see the docstring
            return None

    def emit(self, level, message, *, node=defaults.SERVER_NAME):
        """Record one of the server's own operator-facing lines."""
        try:
            if level not in LEVELS:
                level = 'info'
            return self._record(level, str(node), sanitize(message))
        except Exception:  # noqa: BLE001 - a log line never breaks its caller
            return None

    def sink(self, default_node=_DEFAULT_NODE):
        """
        Return the one-argument callable ``ChildProcess(on_line=...)`` wants.

        This is only useful if somebody passes it: the launch spawn in
        ``session.py`` and the recorder spawn in ``recording.py`` are the two
        sites that must, or the drawer carries nothing but the server's own
        lines.
        """
        def _on_line(text):
            self.append(text, default_node=default_node)
        return _on_line

    def _record(self, level, node, message):
        """Allocate a sequence number and file the line under the lock."""
        with self._lock:
            self._seq += 1
            if level == 'warn':
                self._warn += 1
            elif level == 'error':
                self._error += 1
            line = LogLine(seq=self._seq, t=_rfc3339(self._utcnow()),
                           level=level, node=node, message=message,
                           warn_count=self._warn, error_count=self._error)
            self._ring.append(line)
            self._first_seq = self._ring[0].seq
            self._pending.append(line)
            return line

    # -- consumption -----------------------------------------------------

    def drain_pending(self):
        """
        Pop and return every line recorded since the last drain.

        Everything pending is returned; the per-tick publish cap belongs to
        the frame pump, so a transport decision can never truncate the ring
        or ``GET /api/logs``.
        """
        with self._lock:
            pending = self._pending
            self._pending = []
        return pending

    def counters(self):
        """Return the frame's ``logs`` block."""
        with self._lock:
            return {'warn_count': self._warn, 'error_count': self._error,
                    'last_seq': self._seq}

    def window(self, since=0, limit=None):
        """
        Return the ``GET /api/logs`` body minus ``ok``.

        ``warn_count`` and ``error_count`` are cumulative since construction,
        never windowed, so a page that connects late shows the same badge
        numbers as one that watched from the start. ``dropped`` counts the
        lines that fell out of the ring before ``since``.
        """
        try:
            since = max(0, int(since))
        except (TypeError, ValueError):
            since = 0
        if limit is None:
            limit = self._capacity
        try:
            limit = min(self._capacity, max(1, int(limit)))
        except (TypeError, ValueError):
            limit = self._capacity
        with self._lock:
            lines = [line.wire() for line in self._ring if line.seq > since]
            dropped = (max(0, (self._first_seq - 1) - since)
                       if self._first_seq is not None else 0)
            return {'lines': lines[-limit:], 'warn_count': self._warn,
                    'error_count': self._error, 'dropped': dropped}
