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
Tests for franka_web.lock: the single-operator lock.

The safety claims under test: exactly one holder at a time, expiry only by
the monotonic clock, and a wrong or stale token that can neither release nor
refresh the real holder. A lock CAN be taken over -- deliberately, through
:meth:`OperatorLock.takeover`, which runs the same revocation hook
synchronously before the incumbent claim is cleared.
"""

import hashlib
import threading

from franka_web import defaults
from franka_web.lock import OperatorLock
import pytest
from support.fake_clock import FakeClock

TTL = defaults.OPERATOR_LOCK_TTL_S


def token_of(claim):
    """Return a minted claim's token, or None when the claim was refused."""
    return None if claim is None else claim.token


@pytest.fixture()
def clock():
    """Return a deterministic monotonic clock for the lock under test."""
    return FakeClock()


@pytest.fixture()
def lock(clock):
    """Return an unheld lock driven by the fake clock, with counted tokens."""
    counter = {'n': 0}

    def token_factory():
        counter['n'] += 1
        return 'token-{}'.format(counter['n'])

    return OperatorLock(monotonic=clock.monotonic, token_factory=token_factory)


class TestLifecycle:
    """claim / heartbeat / touch / release, the happy path."""

    def test_claim_returns_a_token_and_locks(self, lock):
        """A first claim mints a token and the lock reports itself held."""
        token = token_of(lock.claim())
        assert token
        assert lock.validate(token)
        state = lock.state()
        assert state['locked'] is True
        assert state['expires_in_s'] == TTL
        assert len(state['claim_id']) == 8
        assert state['since'].endswith('Z')

    def test_default_ttl_is_the_configured_one(self, clock):
        """The default TTL comes from config, not a private literal."""
        default_lock = OperatorLock(monotonic=clock.monotonic)
        assert default_lock.ttl_s == defaults.OPERATOR_LOCK_TTL_S

    def test_default_tokens_are_unguessable_and_unique(self):
        """The default factory mints distinct high-entropy URL-safe tokens."""
        real = OperatorLock()
        seen = set()
        for _ in range(5):
            token = token_of(real.claim())
            assert token is not None
            assert len(token) >= 32
            seen.add(token)
            assert real.release(token) is True
        assert len(seen) == 5

    def test_heartbeat_refreshes_and_reports_the_full_ttl(self, lock, clock):
        """A heartbeat mid-life resets the deadline to a full TTL."""
        token = token_of(lock.claim())
        clock.advance(TTL - 1.0)
        assert lock.state()['expires_in_s'] == pytest.approx(1.0)
        assert lock.heartbeat(token) == pytest.approx(TTL)
        assert lock.state()['expires_in_s'] == pytest.approx(TTL)
        clock.advance(TTL - 0.001)
        assert lock.validate(token) is True

    def test_touch_refreshes_like_a_heartbeat(self, lock, clock):
        """A successful mutating request refreshes the lock through touch."""
        token = token_of(lock.claim())
        clock.advance(TTL - 0.5)
        assert lock.touch(token) is True
        clock.advance(TTL - 0.001)
        assert lock.validate(token) is True

    def test_authorize_atomically_refreshes_and_returns_the_exact_lease(
            self, lock, clock):
        """A mutating request gets one lease while refreshing the same claim."""
        token = token_of(lock.claim())
        clock.advance(TTL - 0.5)
        lease = lock.authorize(token)
        assert lease is not None
        assert lock.lease_is_current(lease) is True
        clock.advance(TTL - 0.001)
        assert lock.lease_is_current(lease) is True

    def test_release_invalidates_the_old_lease_before_a_successor_claim(self, lock):
        """Token release makes every queued command from that claim stale."""
        first = token_of(lock.claim())
        old_lease = lock.authorize(first)
        assert lock.release(first) is True
        second = token_of(lock.claim())
        new_lease = lock.authorize(second)
        assert lock.lease_is_current(old_lease) is False
        assert lock.lease_is_current(new_lease) is True

    def test_run_if_current_is_atomic_with_revocation(self, lock):
        """Release either prevents a commit or its hook observes that commit."""
        state = {'enabled': False, 'revoked_enabled': None}
        lock.set_revocation_hook(
            lambda: state.update(revoked_enabled=state['enabled']))
        token = token_of(lock.claim())
        lease = lock.authorize(token)
        assert lock.run_if_current(
            lease, lambda: state.update(enabled=True)) is True
        assert lock.release(token) is True
        assert state == {'enabled': True, 'revoked_enabled': True}
        assert lock.run_if_current(
            lease, lambda: state.update(enabled=True)) is False

    def test_release_frees_the_lock_immediately(self, lock):
        """Release reports True once, frees the lock, and invalidates the token."""
        token = token_of(lock.claim())
        assert lock.release(token) is True
        assert lock.state() == {'locked': False, 'claim_id': None,
                                'since': None, 'expires_in_s': None}
        assert lock.validate(token) is False
        assert lock.release(token) is False
        assert lock.heartbeat(token) is None
        assert lock.touch(token) is False

    def test_next_operator_claims_after_release(self, lock):
        """A fresh claim after a release mints a different token."""
        first = token_of(lock.claim())
        assert lock.release(first) is True
        second = token_of(lock.claim())
        assert second is not None
        assert second != first
        assert lock.validate(second) is True
        assert lock.validate(first) is False


class TestExclusivity:
    """One holder at a time, and the lock is never stolen."""

    def test_second_claim_is_refused_while_held(self, lock, clock):
        """A second claim returns None for the whole life of the token."""
        token = token_of(lock.claim())
        assert lock.claim() is None
        clock.advance(TTL - 0.001)
        assert lock.claim() is None
        assert lock.validate(token) is True

    def test_refused_claim_does_not_shorten_the_holder(self, lock, clock):
        """A refused claim leaves the holder's deadline exactly where it was."""
        token = token_of(lock.claim())
        clock.advance(1.0)
        before = lock.state()['expires_in_s']
        for _ in range(3):
            assert lock.claim() is None
        assert lock.state()['expires_in_s'] == pytest.approx(before)
        assert lock.validate(token) is True

    def test_wrong_token_cannot_release_or_refresh(self, lock, clock):
        """A wrong token is inert: no release, no refresh, no disturbance."""
        token = token_of(lock.claim())
        clock.advance(TTL - 1.0)
        assert lock.release('token-999') is False
        assert lock.heartbeat('token-999') is None
        assert lock.touch('token-999') is False
        assert lock.validate('token-999') is False
        assert lock.validate(token) is True
        assert lock.state()['expires_in_s'] == pytest.approx(1.0)
        clock.advance(1.0)
        assert lock.validate(token) is False

    def test_stale_token_cannot_disturb_the_next_holder(self, lock, clock):
        """A token retired by expiry cannot touch the operator who follows."""
        stale = token_of(lock.claim())
        clock.advance(TTL)
        successor = token_of(lock.claim())
        assert successor is not None and successor != stale
        assert lock.release(stale) is False
        assert lock.heartbeat(stale) is None
        assert lock.touch(stale) is False
        assert lock.validate(successor) is True
        assert lock.state()['locked'] is True

    @pytest.mark.parametrize('bogus', [None, b'token-1', 1, 3.5, ['token-1'], '', 'token-1 '])
    def test_non_token_values_are_rejected(self, lock, bogus):
        """Wrong types and near-miss strings are refused, holder untouched."""
        token = token_of(lock.claim())
        assert lock.validate(bogus) is False
        assert lock.heartbeat(bogus) is None
        assert lock.touch(bogus) is False
        assert lock.release(bogus) is False
        assert lock.validate(token) is True


class TestExpiry:
    """Expiry is lazy, monotonic, and lands exactly on the TTL."""

    def test_validate_is_true_just_short_of_the_ttl(self, lock, clock):
        """The token is still valid one millisecond before the TTL."""
        token = token_of(lock.claim())
        clock.advance(TTL - 0.001)
        assert lock.validate(token) is True
        assert lock.state()['locked'] is True

    def test_validate_goes_false_at_exactly_the_ttl(self, lock, clock):
        """The token dies exactly at the TTL, not a tick later."""
        token = token_of(lock.claim())
        clock.advance(TTL)
        assert lock.validate(token) is False

    def test_claim_succeeds_after_the_ttl_elapses(self, lock, clock):
        """An expired token frees the lock for the next claim."""
        first = token_of(lock.claim())
        clock.advance(TTL)
        second = token_of(lock.claim())
        assert second is not None
        assert second != first
        assert lock.validate(first) is False
        assert lock.validate(second) is True

    def test_expired_lock_reports_itself_free(self, lock, clock):
        """state() flips to free at the TTL without anyone calling claim."""
        lock.claim()
        clock.advance(TTL)
        assert lock.state() == {'locked': False, 'claim_id': None,
                                'since': None, 'expires_in_s': None}

    def test_heartbeat_after_expiry_does_not_resurrect(self, lock, clock):
        """A heartbeat that arrives late is refused, not honoured."""
        token = token_of(lock.claim())
        clock.advance(TTL)
        assert lock.heartbeat(token) is None
        assert lock.state()['locked'] is False

    def test_heartbeats_keep_the_lock_alive_indefinitely(self, lock, clock):
        """Heartbeats at the configured interval never let the lock lapse."""
        token = token_of(lock.claim())
        for _ in range(20):
            clock.advance(defaults.OPERATOR_HEARTBEAT_INTERVAL_S)
            assert lock.heartbeat(token) == pytest.approx(TTL)
        assert lock.claim() is None

    def test_custom_ttl_is_honoured(self):
        """A non-default TTL drives both expiry and the reported lifetime."""
        early_clock = FakeClock()
        early = OperatorLock(ttl_s=2.0, monotonic=early_clock.monotonic)
        early_token = token_of(early.claim())
        assert early.heartbeat(early_token) == pytest.approx(2.0)
        early_clock.advance(1.99)
        assert early.validate(early_token) is True

        late_clock = FakeClock()
        late = OperatorLock(ttl_s=2.0, monotonic=late_clock.monotonic)
        late_token = token_of(late.claim())
        late_clock.advance(2.0)
        assert late.validate(late_token) is False
        assert late.claim() is not None

    @pytest.mark.parametrize('bad_ttl', [0.0, -1.0, float('inf'), float('nan')])
    def test_non_positive_ttl_is_refused(self, bad_ttl):
        """A TTL that is not positive and finite is a construction error."""
        with pytest.raises(ValueError):
            OperatorLock(ttl_s=bad_ttl)


class TestStateShape:
    """The operator block of the state frame (plan section 6.11)."""

    def test_free_shape(self, lock):
        """A free lock reports locked False and a null lifetime."""
        state = lock.state()
        assert set(state) == {'locked', 'claim_id', 'since', 'expires_in_s'}
        assert state['locked'] is False
        assert state['expires_in_s'] is None
        assert state['claim_id'] is None
        assert state['since'] is None

    def test_held_shape_counts_down(self, lock, clock):
        """A held lock reports a shrinking float lifetime."""
        lock.claim()
        clock.advance(2.6)
        state = lock.state()
        assert set(state) == {'locked', 'claim_id', 'since', 'expires_in_s'}
        assert state['locked'] is True
        assert isinstance(state['expires_in_s'], float)
        assert state['expires_in_s'] == pytest.approx(TTL - 2.6)

    def test_state_is_a_fresh_dict_each_call(self, lock):
        """Callers may mutate the returned dict without corrupting the lock."""
        lock.claim()
        first = lock.state()
        first['locked'] = 'tampered'
        assert lock.state()['locked'] is True


class TestRevocationHook:
    """
    Losing the lock revokes the authorization it carried (finding F-0).

    The hook is what makes plan section 5.6's "lock expiry forces every enable
    off" independent of any poller: it fires inside the lock, at the moment
    the token is dropped, so it has always run by the time a successor token
    exists -- expiry, release, or an expiry and a claim in the same breath.
    """

    def test_expiry_revokes_before_a_successor_token_exists(self, lock, clock):
        """A claim in the same window as the expiry sees the hook already run."""
        events = []
        lock.set_revocation_hook(lambda: events.append('revoked'))
        first = token_of(lock.claim())
        clock.advance(TTL)

        # Nothing observes the lock between the deadline and this claim --
        # the window the 5 Hz frame pump could not see.
        successor = token_of(lock.claim())
        assert successor is not None
        assert successor != first
        assert events == ['revoked']

    def test_expiry_revokes_once_however_often_it_is_observed(self, lock, clock):
        """Repeated observation of an expired lock does not re-fire the hook."""
        events = []
        lock.set_revocation_hook(lambda: events.append('revoked'))
        token = token_of(lock.claim())
        clock.advance(TTL)
        for _ in range(5):
            lock.state()
            lock.validate(token)
        assert events == ['revoked']

    def test_release_revokes_and_a_bad_token_does_not(self, lock):
        """Only the holder's release revokes; a wrong token changes nothing."""
        events = []
        lock.set_revocation_hook(lambda: events.append('revoked'))
        token = token_of(lock.claim())
        assert lock.release('not-the-token') is False
        assert events == []
        assert lock.release(token) is True
        assert events == ['revoked']

    def test_an_unheld_lock_never_revokes(self, lock, clock):
        """Nothing was authorized, so nothing is revoked (no spurious calls)."""
        events = []
        lock.set_revocation_hook(lambda: events.append('revoked'))
        lock.state()
        lock.validate('anything')
        assert lock.release('anything') is False
        clock.advance(TTL * 3)
        assert lock.claim() is not None
        assert events == []

    def test_the_hook_can_be_registered_at_construction(self, clock):
        """The constructor takes the same hook the setter registers."""
        events = []
        built = OperatorLock(monotonic=clock.monotonic,
                             on_revoke=lambda: events.append('revoked'))
        token = token_of(built.claim())
        assert built.release(token) is True
        assert events == ['revoked']

    def test_a_failing_hook_refuses_the_next_claim_and_is_retried(self, lock, clock):
        """
        An unrevoked authorization closes the lock instead of opening it.

        A hook that raises must not take out the frame pump (which observes
        expiry through ``state()``), and it must not let the next operator
        inherit whatever the last one had switched on.
        """
        outcome = {'raise': True, 'calls': 0}

        def hook():
            outcome['calls'] += 1
            if outcome['raise']:
                raise RuntimeError('the supervisor is broken')

        lock.set_revocation_hook(hook)
        lock.claim()
        clock.advance(TTL)

        assert lock.state() == {'locked': False, 'claim_id': None,
                                'since': None, 'expires_in_s': None}
        assert outcome['calls'] == 1
        with pytest.raises(RuntimeError):
            lock.claim()
        assert outcome['calls'] == 2

        outcome['raise'] = False
        token = token_of(lock.claim())
        assert token is not None
        assert outcome['calls'] == 3
        assert lock.validate(token) is True

    def test_a_non_callable_hook_is_refused(self, lock):
        """A mis-wired hook fails loudly at registration, not at expiry."""
        with pytest.raises(TypeError):
            lock.set_revocation_hook('not callable')
        with pytest.raises(TypeError):
            OperatorLock(on_revoke='not callable')


class TestThreadSafety:
    """Concurrent HTTP threads and the supervisor tick share one lock."""

    def test_hammering_never_yields_two_holders(self):
        """Under contention exactly one operator holds the lock at a time."""
        lock = OperatorLock(ttl_s=60.0)
        guard = threading.Lock()
        census = {'live': 0, 'peak': 0, 'claims': 0}
        failures = []
        start = threading.Barrier(4)

        def worker():
            start.wait()
            for _ in range(400):
                token = token_of(lock.claim())
                if token is None:
                    if lock.release('not-the-token') is not False:
                        failures.append('released with a wrong token')
                    continue
                with guard:
                    census['live'] += 1
                    census['claims'] += 1
                    census['peak'] = max(census['peak'], census['live'])
                if not lock.validate(token):
                    failures.append('holder failed validate')
                if not lock.touch(token):
                    failures.append('holder failed touch')
                with guard:
                    census['live'] -= 1
                if not lock.release(token):
                    failures.append('holder failed release')

        threads = [threading.Thread(target=worker) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=30.0)
        assert not any(thread.is_alive() for thread in threads)
        assert failures == []
        assert census['peak'] == 1
        assert census['claims'] > 0
        assert lock.state()['locked'] is False


class TestClaimIdentity:
    """The short public identity a state frame can carry safely."""

    def test_claim_returns_a_claim_id_that_is_the_first_eight_hex_of_the_token_hash(
            self, lock):
        """The id is derivable from the token, and only from the token."""
        claim = lock.claim()
        expected = hashlib.sha256(
            claim.token.encode('utf-8')).hexdigest()[:8]
        assert claim.claim_id == expected
        assert claim.expires_in_s == TTL

    def test_claim_id_changes_on_every_new_claim(self, lock, clock):
        """A successor claim is a different identity, so a page can tell."""
        first = lock.claim()
        clock.advance(TTL + 0.1)
        second = lock.claim()
        assert first.claim_id != second.claim_id

    def test_state_reports_claim_id_and_since_only_while_locked(self, lock, clock):
        """Both are null exactly when the lock is free."""
        claim = lock.claim()
        held = lock.state()
        assert held['claim_id'] == claim.claim_id
        assert held['since'].endswith('Z')
        lock.release(claim.token)
        free = lock.state()
        assert free['claim_id'] is None and free['since'] is None

    def test_claim_id_of_returns_the_identity_of_the_current_lease(self, lock):
        """The supervisor learns its session's operator identity this way."""
        claim = lock.claim()
        lease = lock.authorize(claim.token)
        assert lock.claim_id_of(lease) == claim.claim_id

    def test_claim_id_of_returns_none_for_a_stale_lease(self, lock, clock):
        """A lease that is no longer current identifies nothing."""
        claim = lock.claim()
        lease = lock.authorize(claim.token)
        clock.advance(TTL + 0.1)
        lock.claim()
        assert lock.claim_id_of(lease) is None
        assert lock.claim_id_of(None) is None

    def test_the_token_never_appears_in_the_state_block(self, lock):
        """EventSource cannot send headers, so no frame may carry a token."""
        claim = lock.claim()
        assert claim.token not in repr(lock.state())


class TestTakeover:
    """A takeover is a NEW path through the SAME revocation hook."""

    def test_takeover_revokes_the_incumbent_before_minting_a_successor(self, lock):
        """
        Order is the whole safety argument.

        If the hook ran after the incumbent claim was cleared there would be
        a window in which the lock was free and a stale authorization was
        still live.
        """
        order = []
        incumbent = lock.claim()
        lock.set_revocation_hook(lambda: order.append('revoked'))
        successor = lock.takeover()
        order.append('minted')
        assert order == ['revoked', 'minted']
        assert successor.token != incumbent.token
        assert not lock.validate(incumbent.token)
        assert lock.validate(successor.token)

    def test_takeover_of_a_free_lock_mints_a_claim(self, lock):
        """Nothing to take over is not an error."""
        claim = lock.takeover()
        assert lock.validate(claim.token)

    def test_takeover_fails_and_leaves_the_incumbent_when_the_hook_raises(self, lock):
        """A failed revocation must never hand control to a second operator."""
        incumbent = lock.claim()

        def failing():
            raise RuntimeError('the arms could not be disabled')

        lock.set_revocation_hook(failing)
        with pytest.raises(RuntimeError):
            lock.takeover()
        assert lock.validate(incumbent.token)
        assert lock.state()['locked'] is True

    def test_takeover_changes_the_lease_so_an_in_flight_command_cannot_commit(
            self, lock):
        """An enable in flight under the old lease can no longer commit."""
        incumbent = lock.claim()
        lease = lock.authorize(incumbent.token)
        lock.takeover()
        assert lock.lease_is_current(lease) is False
        assert lock.run_if_current(lease, lambda: None) is False

    def test_takeover_runs_the_same_hook_release_and_expiry_run(self, lock, clock):
        """
        One hook, every path to a new token: release, expiry and takeover.

        A takeover of a lock that is already free AND already revoked does
        not run the hook a fourth time -- there is no outstanding
        authorization to revoke, and re-running it would be noise.
        """
        calls = []
        lock.set_revocation_hook(lambda: calls.append('revoked'))
        first = lock.claim()
        lock.release(first.token)                       # path 1: release
        assert calls == ['revoked']
        second = lock.claim()
        clock.advance(TTL + 0.1)
        lock.state()                                    # path 2: lazy expiry
        assert calls == ['revoked', 'revoked']
        third = lock.claim()
        lock.takeover()                                 # path 3: takeover
        assert calls == ['revoked', 'revoked', 'revoked']
        assert len({first.claim_id, second.claim_id, third.claim_id}) == 3
