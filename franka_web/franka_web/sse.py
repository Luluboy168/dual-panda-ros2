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
Server-sent-event encoding and fan-out (plan section 6.11).

One producer -- the state thread, ticking at ``STATE_FRAME_HZ`` off the latest
sample of a 1 kHz joint stream (section 0.3) -- must never be slowed by a
browser that has stopped reading. So the fan-out here is strictly one-way and
strictly bounded:

* :func:`encode_event` renders a frame to wire bytes ONCE per publish, and the
  same immutable ``bytes`` object is handed to every subscriber;
* each :class:`Subscription` owns a bounded deque of ``SSE_QUEUE_DEPTH`` (4)
  encoded frames and drops the OLDEST on overflow, because a stalled browser
  wants the newest state, not a four-frame-old backlog;
* :meth:`Broker.publish` therefore does a fixed amount of work per subscriber
  (one ``append`` and one ``notify``) and never waits on a reader, a socket, or
  anything else that a client could stall.

The drop is counted per subscriber (:attr:`Subscription.dropped`) so the
stream handler can tell the page it missed frames instead of letting it
believe it saw a continuous history.

Wire format
-----------
``b'event: <name>\ndata: <json>\n\n'``. The JSON is compact
(``separators=(',', ':')``), key-sorted for byte-stable frames, and ASCII-only
-- ``json.dumps`` escapes every control character, so the ``data:`` field is
physically incapable of becoming multi-line no matter what a frame carries.
That is the property the whole transport rests on: one event is one line pair,
and a payload can never forge a frame boundary.

This module is transport-only. It knows nothing about HTTP, holds no ROS
handle, and never touches a robot address.
"""

from collections import deque
import json
import math
import re
import threading
import time

from franka_web import config

# Event names are ours (`state`, `ping`), never client-supplied -- but the name
# is written to the wire verbatim, so anything that could carry a newline, a
# colon, or a stray field name is refused rather than encoded.
_EVENT_NAME_RE = re.compile(r'[A-Za-z0-9_.-]+')

_JSON_SEPARATORS = (',', ':')


def encode_event(event, data):
    r"""
    Encode one event as SSE wire bytes.

    :param event: event name, ``[A-Za-z0-9_.-]+`` (e.g. ``state``, ``ping``).
    :param data: JSON-serializable ``dict`` -- the frame body.
    :returns: ``b'event: <name>\ndata: <compact json>\n\n'``, always exactly
        three newlines, because the JSON is single-line by construction.
    :raises ValueError: the event name is empty or not wire-safe.
    :raises TypeError: ``data`` is not a dict, or is not JSON-serializable.
    """
    if not isinstance(event, str) or not _EVENT_NAME_RE.fullmatch(event):
        raise ValueError('event name must match [A-Za-z0-9_.-]+, got {!r}'.format(event))
    if not isinstance(data, dict):
        raise TypeError('event data must be a dict, got {}'.format(type(data).__name__))
    return 'event: {}\ndata: {}\n\n'.format(event, safe_json_dumps(data)).encode('utf-8')


def _strip_non_finite(value):
    """Recursively replace non-finite floats with ``None``."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _strip_non_finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_strip_non_finite(item) for item in value]
    return value


def safe_json_dumps(data):
    """
    Dump strictly valid, compact, sorted JSON.

    ``json.dumps`` default behaviour emits bare ``NaN``/``Infinity`` tokens,
    which are not JSON: one such token would make the browser's
    ``JSON.parse`` reject every frame (review finding R3). Health data is
    sanitized at the source; this is the belt for everything else -- a
    non-finite float anywhere in the payload becomes ``null`` rather than
    poisoning the stream.
    """
    try:
        return json.dumps(data, separators=_JSON_SEPARATORS, sort_keys=True,
                          ensure_ascii=True, allow_nan=False)
    except ValueError:
        return json.dumps(_strip_non_finite(data), separators=_JSON_SEPARATORS,
                          sort_keys=True, ensure_ascii=True, allow_nan=False)


class Subscription:
    """
    One reader's bounded view of the broker's event stream.

    Created by :meth:`Broker.subscribe`, never directly. The reading thread
    calls :meth:`get` in a loop and :meth:`close` when its client goes away;
    the producing thread only ever calls the private ``_offer``, which appends
    and returns immediately.
    """

    def __init__(self, depth=config.SSE_QUEUE_DEPTH, monotonic=time.monotonic):
        """Create an empty subscription holding at most ``depth`` frames."""
        depth = int(depth)
        if depth < 1:
            raise ValueError('queue depth must be at least 1, got {}'.format(depth))
        self._depth = depth
        self._monotonic = monotonic
        self._condition = threading.Condition(threading.Lock())
        self._frames = deque()
        self._dropped = 0
        self._closed = False

    @property
    def dropped(self):
        """Return how many events this subscriber has missed to overflow."""
        with self._condition:
            return self._dropped

    @property
    def closed(self):
        """Return whether this subscription has been closed."""
        with self._condition:
            return self._closed

    def get(self, timeout_s):
        """
        Return the next encoded event, or ``None``.

        Blocks up to ``timeout_s`` seconds (``None`` blocks until an event
        arrives or the subscription is closed; a non-positive value polls).
        ``None`` means "timed out or closed" -- it is never a frame, since a
        frame is always at least the ``event:`` line.
        """
        deadline = None
        if timeout_s is not None:
            deadline = self._monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while True:
                if self._closed:
                    return None
                if self._frames:
                    return self._frames.popleft()
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - self._monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)

    def close(self):
        """
        Close the subscription and wake any waiting reader.

        Idempotent. Buffered frames are discarded (nobody is left to read
        them) and every later :meth:`get` returns ``None`` at once. The broker
        drops the subscription on its next publish; :meth:`Broker.unsubscribe`
        drops it immediately.
        """
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._frames.clear()
            self._condition.notify_all()

    def _offer(self, payload):
        """
        Enqueue one pre-encoded frame, dropping the oldest if full.

        Returns ``False`` once the subscription is closed, which is the signal
        the broker uses to prune it. Never blocks on the reader.
        """
        with self._condition:
            if self._closed:
                return False
            if len(self._frames) >= self._depth:
                self._frames.popleft()
                self._dropped += 1
            self._frames.append(payload)
            self._condition.notify()
            return True


class Broker:
    """
    Fan-out of encoded events to every live :class:`Subscription`.

    Thread-safe: one producer thread publishes while each stream handler
    thread reads its own subscription. The broker lock is held only for the
    fan-out loop, which does no I/O and cannot wait on a subscriber, so
    :meth:`publish` is bounded by the number of subscribers and nothing else.
    """

    def __init__(self, queue_depth=config.SSE_QUEUE_DEPTH, monotonic=time.monotonic):
        """Create a broker whose subscriptions each buffer ``queue_depth`` frames."""
        self._queue_depth = int(queue_depth)
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self._subscriptions = []

    @property
    def subscriber_count(self):
        """Return the number of subscriptions currently registered."""
        with self._lock:
            return len(self._subscriptions)

    def subscribe(self):
        """Register and return a new :class:`Subscription`."""
        subscription = Subscription(depth=self._queue_depth, monotonic=self._monotonic)
        with self._lock:
            self._subscriptions.append(subscription)
        return subscription

    def unsubscribe(self, subscription):
        """
        Drop ``subscription`` and close it.

        Idempotent, and a no-op for a subscription this broker never handed
        out -- a stream handler unwinding twice must not raise.
        """
        with self._lock:
            for index, registered in enumerate(self._subscriptions):
                if registered is subscription:
                    del self._subscriptions[index]
                    break
        if subscription is not None:
            subscription.close()

    def publish(self, event, data):
        """
        Encode ``event`` once and hand it to every live subscriber.

        Never blocks on a subscriber: a full queue drops that subscriber's
        oldest frame, and a closed subscription is pruned here. Raises only if
        the frame itself cannot be encoded (see :func:`encode_event`), which is
        a programming error, not a client-triggered one.
        """
        payload = encode_event(event, data)
        with self._lock:
            live = [s for s in self._subscriptions if s._offer(payload)]
            if len(live) != len(self._subscriptions):
                self._subscriptions = live

    def close_all(self):
        """Close and drop every subscription (server shutdown)."""
        with self._lock:
            subscriptions = self._subscriptions
            self._subscriptions = []
        for subscription in subscriptions:
            subscription.close()
