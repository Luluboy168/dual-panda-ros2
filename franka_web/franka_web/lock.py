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

A lock CAN be taken over, deliberately and explicitly: a second browser is
told that another operator holds control and is offered **Take over**. What
makes that safe is that :meth:`takeover` runs the SAME revocation hook,
synchronously, under the same mutex, BEFORE the incumbent claim is cleared --
so "a new operator starts with every enable off" stays true of every path to
a new token: expiry, release and takeover alike. The failure this design
refuses to allow is two operators believing they have the robot at the same
time, and a takeover ends the first one's authorization before the second
one's exists.

Expiry is lazy and monotonic: nothing runs on a timer, and every entry point
first retires a token whose deadline has passed. The wall clock never enters
the arithmetic, so a system-time step cannot extend or shorten a lock.
A wrong or stale token is inert -- it can never release, refresh or shorten
the current holder's lock.

Losing the lock revokes the authorization it carried. A token is dropped in
exactly one place (:meth:`OperatorLock._clear`, reached from the lazy expiry
and from an explicit release), and that place calls the registered revocation
hook *before returning*, while the mutex is still held. A successor token can
only be minted once the current one is gone, so "a new operator starts with
every enable off" is true of every path to a new token -- expiry, release,
or an expiry and a claim inside the same millisecond -- and does not depend
on any poller noticing the change (verification finding F-0). If the hook
cannot be run, :meth:`claim` refuses rather than handing control to a new
operator over a stale authorization.

The object is safe to call from several threads at once (the HTTP worker
threads and the supervisor tick all touch it); every public method takes an
internal mutex and holds it only for a few field assignments.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import hmac
import secrets
import threading
import time

from franka_web import defaults

#: Bytes of entropy behind each token (``secrets.token_urlsafe`` argument).
TOKEN_BYTES = 32


def _default_token_factory():
    """Return a fresh URL-safe operator token with 32 bytes of entropy."""
    return secrets.token_urlsafe(TOKEN_BYTES)


def _default_utcnow():
    """Return the current UTC time as an aware datetime."""
    return datetime.now(timezone.utc)


def _rfc3339(moment):
    """Render an aware UTC datetime as RFC 3339 with microseconds and ``Z``."""
    return moment.strftime('%Y-%m-%dT%H:%M:%S.%f') + 'Z'


@dataclass(frozen=True)
class Claim:
    """One minted operator claim, as POST /api/operator/claim returns it."""

    token: str
    claim_id: str
    expires_in_s: float


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

    def __init__(self, ttl_s=defaults.OPERATOR_LOCK_TTL_S, monotonic=time.monotonic,
                 token_factory=None, on_revoke=None, utcnow=None):
        """
        Build an unheld lock with the given TTL, clock and token source.

        ``ttl_s`` is the seconds a token survives without a refresh;
        ``monotonic`` is any zero-argument callable returning seconds from a
        monotonic source; ``token_factory`` is any zero-argument callable
        returning a fresh non-empty token string, and defaults to
        ``secrets.token_urlsafe(32)``; ``on_revoke`` is the revocation hook
        described in :meth:`set_revocation_hook` and may also be registered
        later.
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
        if on_revoke is not None and not callable(on_revoke):
            raise TypeError('on_revoke must be a zero-argument callable')
        self._ttl_s = ttl
        self._monotonic = monotonic
        self._token_factory = token_factory
        self._on_revoke = on_revoke
        self._utcnow = utcnow or _default_utcnow
        self._mutex = threading.Lock()
        self._token = None
        self._token_bytes = None
        self._expires_at = None
        # The short public identity of the current claim: the first eight hex
        # characters of sha256(token). It travels in the state frame so a page
        # can tell its own lock from somebody else's without the SSE stream
        # ever carrying a token (EventSource cannot send headers).
        self._claim_id = None
        self._since = None
        # Opaque identity for the current claim.  A new object is minted for
        # every successful claim, even if a test token factory happens to
        # reuse the same token text.  HTTP hands this identity to an
        # asynchronous command so it can prove that the exact authorization
        # it validated still exists when the command commits.
        self._lease = None
        # Nothing was ever held, so there is no authorization outstanding.
        self._revoked = True
        self._revoke_failure = None

    @property
    def ttl_s(self):
        """Return the token lifetime in seconds (a successful refresh grants this)."""
        return self._ttl_s

    def set_revocation_hook(self, hook):
        """
        Register the callable that revokes an operator's authorization.

        ``hook`` takes no arguments and is called the instant a held token is
        dropped -- lazy expiry or explicit release -- from whichever thread
        observed it, **with the lock's internal mutex held**. It must
        therefore never block and never call back into this lock; clearing a
        few flags and queueing work for another thread is what it is for.

        Pass ``None`` to unregister. A hook that raises leaves the
        authorization outstanding: :meth:`claim` retries it and refuses to
        mint a successor token until it succeeds.
        """
        if hook is not None and not callable(hook):
            raise TypeError('the revocation hook must be a zero-argument callable')
        with self._mutex:
            self._on_revoke = hook

    def claim(self):
        """
        Mint and return a fresh :class:`Claim`, or None while one is held.

        A held-but-expired token is retired first -- which revokes its
        authorization before this call can mint anything -- so the next
        caller after an expiry gets a lock with no inherited enables. A
        held-and-unexpired token is never stolen HERE: the caller is refused,
        and it is up to the page to offer :meth:`takeover`.

        Raises :class:`RuntimeError` in the one case where handing out
        control would be unsafe: the previous operator's authorization could
        not be revoked because the hook keeps failing.
        """
        with self._mutex:
            now = self._monotonic()
            self._retire(now)
            if self._token is not None:
                return None
            if not self._revoked:
                # A previous hook call failed. Retry it, and refuse the
                # claim rather than let a new operator inherit whatever the
                # last one had switched on.
                self._revoke()
                if not self._revoked:
                    raise RuntimeError(
                        "refusing to hand out control: the previous operator's "
                        'authorization could not be revoked ({})'.format(
                            self._revoke_failure))
            return self._mint(now)

    def takeover(self):
        """
        Revoke the incumbent's authorization and mint a successor claim.

        The order is the whole safety argument, and it is the same one
        :meth:`_clear` relies on: revoke FIRST, while the incumbent still
        holds the lock, so there is never a window in which the lock is free
        and a stale authorization is still live. A hook that fails leaves the
        incumbent holding an unchanged lock and raises.

        Returns the successor :class:`Claim`. Raises :class:`RuntimeError`
        when the revocation could not be completed.
        """
        with self._mutex:
            now = self._monotonic()
            self._retire(now)
            if self._token is not None:
                self._revoked = False
                self._revoke()
                if not self._revoked:
                    raise RuntimeError(
                        "refusing to take over: the incumbent operator's "
                        'authorization could not be revoked ({})'.format(
                            self._revoke_failure))
                self._drop()
            elif not self._revoked:
                self._revoke()
                if not self._revoked:
                    raise RuntimeError(
                        "refusing to take over: the previous operator's "
                        'authorization could not be revoked ({})'.format(
                            self._revoke_failure))
            return self._mint(now)

    def claim_id_of(self, lease):
        """Return the claim id ``lease`` identifies, or None if it is stale."""
        with self._mutex:
            self._retire(self._monotonic())
            if lease is None or lease is not self._lease:
                return None
            return self._claim_id

    def authorize(self, token):
        """
        Refresh ``token`` and return its opaque claim identity, or None.

        Unlike a separate :meth:`validate` followed by :meth:`touch`, this is
        one atomic operation.  The returned object identifies this exact
        claim, not merely its token text, and is intended only for an
        in-process asynchronous command that must later use
        :meth:`run_if_current` before committing an authorization-dependent
        result.
        """
        with self._mutex:
            now = self._monotonic()
            self._retire(now)
            if not self._matches(token):
                return None
            self._expires_at = now + self._ttl_s
            return self._lease

    def lease_is_current(self, lease):
        """Return whether ``lease`` is the exact held, unexpired claim."""
        with self._mutex:
            self._retire(self._monotonic())
            return lease is not None and lease is self._lease

    def run_if_current(self, lease, action):
        """
        Run a tiny non-blocking ``action`` iff ``lease`` is still current.

        The identity check and callback run under the same mutex used by
        expiry and release.  This is the lock's compare-and-set surface: a
        revocation either happens first and prevents ``action``, or happens
        afterward and observes what ``action`` committed in its revocation
        hook.  ``action`` must not block and must not call back into this
        lock.
        """
        if not callable(action):
            raise TypeError('action must be callable')
        with self._mutex:
            self._retire(self._monotonic())
            if lease is None or lease is not self._lease:
                return False
            action()
            return True

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
        Return the ``operator`` block of the state frame.

        ``claim_id``, ``since`` and ``expires_in_s`` are None exactly when
        ``locked`` is false. The token itself never appears here.
        """
        with self._mutex:
            now = self._monotonic()
            self._retire(now)
            if self._token is None:
                return {'locked': False, 'claim_id': None, 'since': None,
                        'expires_in_s': None}
            return {'locked': True, 'claim_id': self._claim_id,
                    'since': self._since,
                    'expires_in_s': max(0.0, self._expires_at - now)}

    # --- internals; every one of these runs with self._mutex held ----------

    def _retire(self, now):
        """Drop the held token if its deadline has passed (lazy expiry)."""
        if self._token is not None and now >= self._expires_at:
            self._clear()

    def _mint(self, now):
        """Create, install and return a fresh claim (mutex already held)."""
        token = self._token_factory()
        encoded = _encode(token)
        if not token or encoded is None:
            raise ValueError('token_factory must return a non-empty string')
        self._token = token
        self._token_bytes = encoded
        self._claim_id = hashlib.sha256(encoded).hexdigest()[:8]
        self._since = _rfc3339(self._utcnow())
        self._expires_at = now + self._ttl_s
        self._lease = object()
        return Claim(token=token, claim_id=self._claim_id,
                     expires_in_s=self._ttl_s)

    def _drop(self):
        """Forget the held claim WITHOUT running the revocation hook."""
        self._token = None
        self._token_bytes = None
        self._claim_id = None
        self._since = None
        self._expires_at = None
        self._lease = None

    def _clear(self):
        """Forget the held token and revoke the authorization it carried."""
        self._drop()
        self._revoked = False
        self._revoke()

    def _revoke(self):
        """
        Run the revocation hook, recording whether it succeeded.

        A raising hook must not propagate: this runs inside ``state()`` and
        ``validate()`` too, on the frame pump and on HTTP worker threads, and
        an exception there would take out a thread over a bug in the
        callback. It is remembered instead, and :meth:`claim` refuses until
        a retry succeeds -- the failure closes the lock, it does not open it.
        """
        if self._on_revoke is None:
            self._revoked = True
            return
        try:
            self._on_revoke()
        except Exception as error:  # noqa: BLE001 - see the docstring
            self._revoked = False
            self._revoke_failure = type(error).__name__
        else:
            self._revoked = True
            self._revoke_failure = None

    def _matches(self, token):
        """Return True if ``token`` equals the held token, compared in constant time."""
        if self._token_bytes is None:
            return False
        candidate = _encode(token)
        if candidate is None:
            return False
        return hmac.compare_digest(candidate, self._token_bytes)
