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
The single-operator lock (plan section 5.6, API sections 6.2-6.4).

One browser at a time may command the robot. A claim mints an unguessable
token with a short TTL; the page refreshes it with a heartbeat every
``OPERATOR_HEARTBEAT_INTERVAL_S`` seconds, and every successful mutating
request refreshes it too. Each mutating endpoint requires the token back in
``X-Operator-Token``.

The lock **cannot be stolen** -- only expired. There is no force-claim, no
admin override and no "take control" path: a second browser is told that
another operator holds control and for how much longer, and it waits. That
is deliberate. The failure this design refuses to allow is two operators
believing they have the robot at the same time.

Expiry is lazy and monotonic: nothing runs on a timer, and every entry point
first retires a token whose deadline has passed. The wall clock never enters
the arithmetic, so a system-time step cannot extend or shorten a lock.
A wrong or stale token is inert -- it can never release, refresh or shorten
the current holder's lock.

The object is safe to call from several threads at once (the HTTP worker
threads and the supervisor tick all touch it); every public method takes an
internal mutex and holds it only for a few field assignments.
"""

import hmac
import secrets
import threading
import time

from franka_web import config

#: Bytes of entropy behind each token (``secrets.token_urlsafe`` argument).
TOKEN_BYTES = 32


def _default_token_factory():
    """Return a fresh URL-safe operator token with 32 bytes of entropy."""
    return secrets.token_urlsafe(TOKEN_BYTES)


def _encode(value):
    """Encode a token to bytes for a constant-time compare, or None."""
    if not isinstance(value, str):
        return None
    # utf-8 only ever fails on lone surrogates, which 'surrogatepass' keeps.
    return value.encode('utf-8', 'surrogatepass')


class OperatorLock:
    """
    Exclusive, expiring, unstealable control of the robot by one operator.

    Construct one per server. Hand out a token with :meth:`claim`, keep it
    alive with :meth:`heartbeat` or :meth:`touch`, hand it back with
    :meth:`release`, and gate every mutating request on :meth:`validate`.
    :meth:`state` renders the ``operator`` block of the state frame
    (plan section 6.11).
    """

    def __init__(self, ttl_s=config.OPERATOR_LOCK_TTL_S, monotonic=time.monotonic,
                 token_factory=None):
        """
        Build an unheld lock with the given TTL, clock and token source.

        ``ttl_s`` is the seconds a token survives without a refresh;
        ``monotonic`` is any zero-argument callable returning seconds from a
        monotonic source; ``token_factory`` is any zero-argument callable
        returning a fresh non-empty token string, and defaults to
        ``secrets.token_urlsafe(32)``.
        """
        ttl = float(ttl_s)
        if not ttl > 0.0 or ttl == float('inf'):
            raise ValueError('the operator lock TTL must be a positive, finite number of seconds')
        if not callable(monotonic):
            raise TypeError('monotonic must be a zero-argument callable')
        if token_factory is None:
            token_factory = _default_token_factory
        elif not callable(token_factory):
            raise TypeError('token_factory must be a zero-argument callable')
        self._ttl_s = ttl
        self._monotonic = monotonic
        self._token_factory = token_factory
        self._mutex = threading.Lock()
        self._token = None
        self._token_bytes = None
        self._expires_at = None

    @property
    def ttl_s(self):
        """Return the token lifetime in seconds (a successful refresh grants this)."""
        return self._ttl_s

    def claim(self):
        """
        Mint and return a fresh token, or None while another one is held.

        A held-but-expired token is retired first, so the next caller after
        an expiry gets the lock. A held-and-unexpired token is never stolen:
        the caller is refused and must wait it out.
        """
        with self._mutex:
            now = self._monotonic()
            self._retire(now)
            if self._token is not None:
                return None
            token = self._token_factory()
            encoded = _encode(token)
            if not token or encoded is None:
                raise ValueError('token_factory must return a non-empty string')
            self._token = token
            self._token_bytes = encoded
            self._expires_at = now + self._ttl_s
            return token

    def heartbeat(self, token):
        """
        Refresh ``token`` and return its new lifetime, or None if it is not valid.

        Returning None means the token was wrong, already released, or
        expired -- in every one of those cases the current holder, if any, is
        left exactly as it was.
        """
        with self._mutex:
            now = self._monotonic()
            self._retire(now)
            if not self._matches(token):
                return None
            self._expires_at = now + self._ttl_s
            return self._ttl_s

    def touch(self, token):
        """Refresh ``token`` after a successful mutating request; True if it was valid."""
        return self.heartbeat(token) is not None

    def release(self, token):
        """
        Give the lock up and report whether ``token`` actually held it.

        A wrong or stale token returns False and leaves the current holder
        untouched -- releasing is not a way to steal.
        """
        with self._mutex:
            now = self._monotonic()
            self._retire(now)
            if not self._matches(token):
                return False
            self._clear()
            return True

    def validate(self, token):
        """Return True if ``token`` is the held, unexpired token (constant-time compare)."""
        with self._mutex:
            self._retire(self._monotonic())
            return self._matches(token)

    def state(self):
        """
        Return the ``operator`` block of the state frame (plan section 6.11).

        ``{'locked': bool, 'expires_in_s': float | None}``; ``expires_in_s``
        is None exactly when the lock is free.
        """
        with self._mutex:
            now = self._monotonic()
            self._retire(now)
            if self._token is None:
                return {'locked': False, 'expires_in_s': None}
            return {'locked': True, 'expires_in_s': max(0.0, self._expires_at - now)}

    # --- internals; every one of these runs with self._mutex held ----------

    def _retire(self, now):
        """Drop the held token if its deadline has passed (lazy expiry)."""
        if self._token is not None and now >= self._expires_at:
            self._clear()

    def _clear(self):
        """Forget the held token."""
        self._token = None
        self._token_bytes = None
        self._expires_at = None

    def _matches(self, token):
        """Return True if ``token`` equals the held token, compared in constant time."""
        if self._token_bytes is None:
            return False
        candidate = _encode(token)
        if candidate is None:
            return False
        return hmac.compare_digest(candidate, self._token_bytes)
