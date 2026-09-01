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
Tests for franka_web.sse: wire encoding and bounded, non-blocking fan-out.

Pure unit level -- no ROS, no sockets, no HTTP. The only real concurrency is
plain ``threading``, which is what the server itself uses between the state
thread and each stream handler.
"""

import json
import threading
import time

from franka_web import defaults
from franka_web import server as server_module
from franka_web.logbus import LogBus
from franka_web.server import (
    _frame_pump, _LOG_EVENTS_PER_TICK, _PRODUCTION_QUEUE_DEPTH, _pump_once)
from franka_web.sse import (
    Broker, encode_event, encode_log, Subscription)
import pytest
from support.fake_clock import FakeClock

# A generous ceiling for operations that must not block: every one of them is
# microseconds of real work, so a second is failure, not slowness.
NON_BLOCKING_CEILING_S = 1.0

# A hostile frame body: newlines, a CR, a forged SSE frame boundary, the
# JavaScript line separators, and control characters, in keys and in values.
HOSTILE_FRAME = {
    'note': 'line one\nline two\r\n\r\nevent: state\ndata: {"forged":true}\n\n',
    'sep\nkey': ['a b', 'c d', 'e\x00f'],
    'nested': {'deep\r': {'deeper': 'x\ny'}},
}


def _body(frame):
    """Return the decoded JSON body of one encoded SSE frame."""
    _, separator, body = frame.partition(b'\ndata: ')
    assert separator, 'frame has no data field: {!r}'.format(frame)
    assert body.endswith(b'\n\n')
    return json.loads(body[:-2])


def _server_source():
    """Return the shipped ``franka_web/server.py`` source text."""
    with open(server_module.__file__, 'r', encoding='utf-8') as handle:
        return handle.read()


def _drain(subscription):
    """Return every frame currently buffered for ``subscription``."""
    frames = []
    while True:
        frame = subscription.get(0.0)
        if frame is None:
            return frames
        frames.append(frame)


def _advancing(clock, step):
    """Return a monotonic callable that leaps ``step`` seconds per reading."""
    def monotonic():
        value = clock.monotonic()
        clock.advance(step)
        return value
    return monotonic


class TestEncodeEvent:
    """The wire format is exact, compact, and structurally unforgeable."""

    def test_state_event_exact_bytes(self):
        """A `state` frame encodes to the documented bytes, keys sorted."""
        frame = {
            'session': {'state': 'running', 'arms': 'both'},
            'schema_version': defaults.SCHEMA_VERSION,
        }
        assert encode_event('state', frame) == (
            b'event: state\n'
            b'data: {"schema_version":4,"session":{"arms":"both","state":"running"}}\n'
            b'\n')

    def test_ping_event_exact_bytes(self):
        """A `ping` frame encodes to the documented bytes (plan section 6.11)."""
        frame = {'schema_version': defaults.SCHEMA_VERSION, 't': '2026-08-30T14:15:01.123456Z'}
        assert encode_event('ping', frame) == (
            b'event: ping\n'
            b'data: {"schema_version":4,"t":"2026-08-30T14:15:01.123456Z"}\n'
            b'\n')

    def test_json_is_compact_and_key_sorted(self):
        """No separator whitespace, and sorting is deep and stable."""
        first = encode_event('state', {'b': 2, 'a': {'z': 1, 'y': 2}})
        assert first == b'event: state\ndata: {"a":{"y":2,"z":1},"b":2}\n\n'
        assert first == encode_event('state', {'a': {'y': 2, 'z': 1}, 'b': 2})

    def test_a_payload_can_never_produce_a_multiline_data_field(self):
        """Every control character is escaped, so one event stays one line pair."""
        encoded = encode_event('state', HOSTILE_FRAME)
        lines = encoded.split(b'\n')
        assert len(lines) == 4
        assert lines[0] == b'event: state'
        assert lines[1].startswith(b'data: {')
        assert lines[2] == b''
        assert lines[3] == b''
        assert encoded.count(b'\n') == 3
        assert b'\r' not in encoded
        assert b'\x00' not in encoded

    def test_a_hostile_payload_round_trips_unchanged(self):
        """Escaping is lossless: the reader gets exactly what was published."""
        assert _body(encode_event('state', HOSTILE_FRAME)) == HOSTILE_FRAME

    def test_the_wire_bytes_are_pure_ascii(self):
        """Non-ASCII is escaped, so no byte can be mistaken for a delimiter."""
        encoded = encode_event('state', {'arm': 'panda1 \u00b5m \u2028 \U0001f600'})
        assert max(encoded) < 128
        assert encoded.decode('ascii') == encoded.decode('utf-8')

    @pytest.mark.parametrize(
        'name', ['', 'has space', 'two\nlines', 'colon:name', 'nl\r', 7, None])
    def test_an_unsafe_event_name_is_refused(self, name):
        """Anything that could forge a field or a frame boundary raises."""
        with pytest.raises(ValueError):
            encode_event(name, {'ok': True})

    @pytest.mark.parametrize('data', [None, [], 'text', 7])
    def test_a_non_dict_body_is_refused(self, data):
        """SSE data is always a JSON object in this contract."""
        with pytest.raises(TypeError):
            encode_event('state', data)


class TestSubscriptionQueue:
    """Per-subscriber buffering is bounded and drops the oldest frame."""

    def test_default_depth_comes_from_the_defaults_module(self):
        """The broker does not redefine SSE_QUEUE_DEPTH."""
        assert defaults.SSE_QUEUE_DEPTH == 4
        broker = Broker()
        subscription = broker.subscribe()
        for index in range(defaults.SSE_QUEUE_DEPTH + 3):
            broker.publish('state', {'n': index})
        assert len(_drain(subscription)) == defaults.SSE_QUEUE_DEPTH

    def test_drop_oldest_keeps_the_newest_four(self):
        """At depth 4, six published events leave events 2..5 buffered."""
        broker = Broker()
        subscription = broker.subscribe()
        for index in range(6):
            broker.publish('state', {'n': index})
        assert [_body(frame)['n'] for frame in _drain(subscription)] == [2, 3, 4, 5]
        assert subscription.dropped == 2

    def test_dropped_is_per_subscriber(self):
        """A reader that keeps up drops nothing while a stalled peer drops."""
        broker = Broker()
        reader = broker.subscribe()
        stalled = broker.subscribe()
        for index in range(10):
            broker.publish('state', {'n': index})
            assert _body(reader.get(0.0))['n'] == index
        assert reader.dropped == 0
        assert stalled.dropped == 10 - defaults.SSE_QUEUE_DEPTH
        assert [_body(frame)['n'] for frame in _drain(stalled)] == [6, 7, 8, 9]

    def test_publish_encodes_once_and_shares_the_bytes(self):
        """Every subscriber receives the identical immutable payload object."""
        broker = Broker()
        first = broker.subscribe()
        second = broker.subscribe()
        broker.publish('state', {'n': 1})
        assert first.get(0.0) is second.get(0.0)

    def test_a_depth_below_one_is_refused(self):
        """A zero-depth queue would silently swallow the stream."""
        with pytest.raises(ValueError):
            Subscription(depth=0)


class TestGetTimeout:
    """`get` is the only blocking call, and its bound is honoured."""

    def test_timeout_returns_none(self):
        """An empty subscription yields None after roughly the timeout."""
        subscription = Broker().subscribe()
        started = time.monotonic()
        assert subscription.get(0.05) is None
        elapsed = time.monotonic() - started
        assert elapsed >= 0.02
        assert elapsed < NON_BLOCKING_CEILING_S

    @pytest.mark.parametrize('timeout_s', [0.0, -1.0])
    def test_a_non_positive_timeout_polls(self, timeout_s):
        """Zero or negative means poll: return at once, empty or not."""
        subscription = Broker().subscribe()
        started = time.monotonic()
        assert subscription.get(timeout_s) is None
        assert time.monotonic() - started < NON_BLOCKING_CEILING_S

    def test_the_deadline_is_read_from_the_injected_clock(self):
        """A clock that leaps past the deadline ends the wait immediately."""
        clock = FakeClock()
        subscription = Subscription(monotonic=_advancing(clock, 10.0))
        started = time.monotonic()
        assert subscription.get(5.0) is None
        assert time.monotonic() - started < NON_BLOCKING_CEILING_S

    def test_get_on_a_closed_subscription_returns_none_at_once(self):
        """Close is final: no frame, no wait."""
        subscription = Broker().subscribe()
        subscription.close()
        started = time.monotonic()
        assert subscription.get(30.0) is None
        assert time.monotonic() - started < NON_BLOCKING_CEILING_S
        assert subscription.closed


class TestProducerIsNeverBlocked:
    """A stalled browser can never back-pressure the producing thread."""

    def test_a_subscriber_that_never_reads_cannot_slow_publish(self):
        """1000 publishes into an unread queue stay bounded and lossy."""
        broker = Broker()
        stalled = broker.subscribe()
        started = time.monotonic()
        for index in range(1000):
            broker.publish('state', {'n': index})
        elapsed = time.monotonic() - started
        assert elapsed < NON_BLOCKING_CEILING_S
        assert stalled.dropped == 1000 - defaults.SSE_QUEUE_DEPTH
        assert [_body(frame)['n'] for frame in _drain(stalled)] == [996, 997, 998, 999]

    def test_publish_does_not_wait_for_a_reader_blocked_in_get(self):
        """A reader parked in a long `get` does not hold the producer up."""
        broker = Broker()
        parked = broker.subscribe()
        received = []
        reader = threading.Thread(target=lambda: received.append(parked.get(30.0)))
        reader.start()
        try:
            started = time.monotonic()
            broker.publish('ping', {'t': 'x'})
            assert time.monotonic() - started < NON_BLOCKING_CEILING_S
        finally:
            reader.join(10.0)
        assert not reader.is_alive()
        assert received == [encode_event('ping', {'t': 'x'})]

    def test_close_wakes_a_blocked_reader(self):
        """A stream handler shutting down never leaks a parked thread."""
        broker = Broker()
        parked = broker.subscribe()
        received = []
        reader = threading.Thread(target=lambda: received.append(parked.get(30.0)))
        reader.start()
        try:
            time.sleep(0.05)
            parked.close()
        finally:
            reader.join(10.0)
        assert not reader.is_alive()
        assert received == [None]

    def test_concurrent_publishers_and_one_reader_conserve_every_event(self):
        """Under contention every event is either delivered or counted dropped."""
        broker = Broker()
        subscription = broker.subscribe()
        per_thread = 250
        publishers = 4
        received = []
        stop = threading.Event()

        def read():
            while not stop.is_set():
                frame = subscription.get(0.01)
                if frame is not None:
                    received.append(frame)

        def publish():
            for index in range(per_thread):
                broker.publish('state', {'n': index})

        reader = threading.Thread(target=read)
        reader.start()
        threads = [threading.Thread(target=publish) for _ in range(publishers)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(30.0)
            assert not thread.is_alive()
        stop.set()
        reader.join(10.0)
        assert not reader.is_alive()
        received.extend(_drain(subscription))
        assert len(received) + subscription.dropped == per_thread * publishers


class TestSubscriberBookkeeping:
    """Subscribe, unsubscribe, and pruning keep an exact roster."""

    def test_subscribe_and_unsubscribe_track_the_count(self):
        """The roster grows and shrinks with the handlers."""
        broker = Broker()
        assert broker.subscriber_count == 0
        first = broker.subscribe()
        second = broker.subscribe()
        assert broker.subscriber_count == 2
        broker.unsubscribe(first)
        assert broker.subscriber_count == 1
        assert first.closed
        assert not second.closed
        broker.unsubscribe(second)
        assert broker.subscriber_count == 0

    def test_unsubscribe_is_idempotent_and_tolerates_a_stranger(self):
        """A handler unwinding twice, or from another broker, must not raise."""
        broker = Broker()
        subscription = broker.subscribe()
        stranger = Broker().subscribe()
        broker.unsubscribe(subscription)
        broker.unsubscribe(subscription)
        broker.unsubscribe(stranger)
        broker.unsubscribe(None)
        assert broker.subscriber_count == 0

    def test_a_closed_subscription_is_pruned_on_the_next_publish(self):
        """Close alone is enough; the broker cleans up on its own tick."""
        broker = Broker()
        closed = broker.subscribe()
        live = broker.subscribe()
        closed.close()
        assert broker.subscriber_count == 2
        broker.publish('state', {'n': 1})
        assert broker.subscriber_count == 1
        assert _drain(closed) == []
        assert len(_drain(live)) == 1

    def test_publish_with_no_subscribers_is_a_no_op(self):
        """The producing thread runs identically with the page closed."""
        broker = Broker()
        broker.publish('state', {'n': 1})
        assert broker.subscriber_count == 0

    def test_close_all_closes_and_drops_every_subscription(self):
        """Server shutdown releases every parked stream handler."""
        broker = Broker()
        subscriptions = [broker.subscribe() for _ in range(3)]
        broker.close_all()
        assert broker.subscriber_count == 0
        assert all(subscription.closed for subscription in subscriptions)
        assert all(subscription.get(0.0) is None for subscription in subscriptions)

    def test_a_closed_subscription_receives_nothing_further(self):
        """A late publish cannot resurrect a closed queue."""
        broker = Broker()
        subscription = broker.subscribe()
        broker.publish('state', {'n': 1})
        subscription.close()
        broker.publish('state', {'n': 2})
        assert subscription.get(0.0) is None
        assert subscription.dropped == 0


#: The production queue depth, and the frame pump's per-tick log cap, taken
#: from the shipped server itself -- re-declaring them here would prove a
#: property of this file instead of a property of the code that runs. Both
#: halves are required, and this is the file that proves it.
PRODUCTION_QUEUE_DEPTH = _PRODUCTION_QUEUE_DEPTH
LOG_EVENTS_PER_TICK = _LOG_EVENTS_PER_TICK


class _StubSupervisor:
    """A supervisor stand-in whose ``frame()`` is scripted per call."""

    def __init__(self, *, raise_on=()):
        self.raise_on = set(raise_on)
        self.calls = 0

    def frame(self):
        """Return one state frame, or raise if this call is scripted to."""
        self.calls += 1
        if self.calls in self.raise_on:
            raise RuntimeError('projection exploded on call {}'.format(
                self.calls))
        return {'schema_version': defaults.SCHEMA_VERSION,
                'session': {'state': 'starting'}, 'call': self.calls}


class TestLogEvents:
    """The `log` event and the sizing that keeps it from evicting `state`."""

    def test_encode_log_renders_the_contract_payload(self):
        """One captured line becomes one `log` event carrying both counters."""
        bus = LogBus()
        line = bus.append('[WARN] [1.0] [controller_manager]: deactivating')
        payload = encode_log(line)
        assert payload.startswith(b'event: log\ndata: {')
        body = _body(payload)
        assert body['level'] == 'warn'
        assert body['node'] == 'controller_manager'
        assert body['message'] == 'deactivating'
        assert body['warn_count'] == 1
        assert body['error_count'] == 0
        assert body['seq'] == 1

    def test_a_burst_of_log_events_cannot_evict_the_following_state_frame(self):
        """
        A launch burst must not push the startup frame out of the queue.

        A `ros2 launch` emits hundreds of lines in its first seconds --
        exactly the window in which the checklist, the hint and the drawer
        matter most. Both halves of the sizing are exercised here: the
        production depth, and the frame pump's newest-N drain cap.
        """
        bus = LogBus()
        for index in range(300):
            bus.append('[INFO] [1.0] [launch]: line {}'.format(index))
        pending = [line for line in bus.drain_pending() if line.level != 'debug']

        broker = Broker(queue_depth=PRODUCTION_QUEUE_DEPTH)
        subscription = broker.subscribe()
        for line in pending[-LOG_EVENTS_PER_TICK:]:
            broker.publish('log', line.event())
        broker.publish('state', {'schema_version': defaults.SCHEMA_VERSION,
                                 'session': {'state': 'starting'}})

        frames = _drain(subscription)
        assert subscription.dropped == 0
        assert any(b'event: state' in frame for frame in frames)
        assert _body(frames[-1])['session']['state'] == 'starting'

    def test_the_same_burst_at_the_bare_default_depth_evicts_it(self):
        """
        The constructor argument is load-bearing, not decorative.

        At the bare default of 4, the same sixteen log events push the state
        frame's predecessors out and leave a queue of log events only -- which
        is what the production depth exists to prevent.
        """
        bus = LogBus()
        for index in range(300):
            bus.append('[INFO] [1.0] [launch]: line {}'.format(index))
        pending = [line for line in bus.drain_pending() if line.level != 'debug']

        broker = Broker(queue_depth=defaults.SSE_QUEUE_DEPTH)
        subscription = broker.subscribe()
        for line in pending[-LOG_EVENTS_PER_TICK:]:
            broker.publish('log', line.event())
        frames_before_state = _drain(subscription)
        assert subscription.dropped == LOG_EVENTS_PER_TICK - defaults.SSE_QUEUE_DEPTH
        assert all(b'event: log' in frame for frame in frames_before_state)

    def test_debug_lines_never_reach_the_stream(self):
        """
        `debug` is captured and served by GET /api/logs, never streamed.

        Because it still consumes a sequence number, the streamed `seq`
        sequence legitimately has holes -- which is why a gap is not evidence
        of a dropped event.
        """
        bus = LogBus()
        bus.append('[INFO] [1.0] [a]: one')
        bus.append('[DEBUG] [1.0] [a]: two')
        bus.append('[INFO] [1.0] [a]: three')
        streamed = [line for line in bus.drain_pending() if line.level != 'debug']
        assert [line.seq for line in streamed] == [1, 3]
        assert [line['seq'] for line in bus.window()['lines']] == [1, 2, 3]


class TestFramePump:
    """
    Ledger D11's sizing, proved against the shipped pump.

    The cases above prove what the numbers do; these prove that the server
    is the thing that carries them. Every assertion here runs the production
    `server._pump_once` and the production constants, so tightening the queue
    depth, lifting the per-tick cap or deleting the debug filter fails a test.
    """

    def test_the_shipped_sizing_is_the_sizing_these_tests_prove(self):
        """The two constants are the reviewed ones (ledger D11)."""
        assert _PRODUCTION_QUEUE_DEPTH == 64
        assert _LOG_EVENTS_PER_TICK == 16
        assert _PRODUCTION_QUEUE_DEPTH > defaults.SSE_QUEUE_DEPTH

    def test_the_production_broker_is_built_at_the_production_depth(self):
        """`main` sizes the broker from the constant, not from a literal."""
        source = _server_source()
        assert 'Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)' in source

    def test_a_launch_burst_through_the_pump_cannot_evict_the_frame(self):
        """
        A 300-line burst leaves the tick's own state frame in the queue.

        This drives `server._pump_once` -- both halves of the sizing at once:
        the per-tick drain cap keeps the burst to sixteen events, and the
        production depth keeps those sixteen plus the frame in a queue that
        never evicts. Raising the cap or lowering the depth drops frames.
        """
        bus = LogBus()
        for index in range(300):
            bus.append('[INFO] [1.0] [launch]: line {}'.format(index))
        supervisor = _StubSupervisor()
        broker = Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)
        subscription = broker.subscribe()

        _pump_once(supervisor, broker, bus)

        frames = _drain(subscription)
        assert subscription.dropped == 0
        assert sum(b'event: state' in frame for frame in frames) == 1
        assert _body(frames[-1])['session']['state'] == 'starting'
        assert len(frames) == _LOG_EVENTS_PER_TICK + 1

    def test_the_pump_never_streams_a_debug_line(self):
        """
        A DEBUG line inside the tick's newest lines is captured, not streamed.

        The debug sits fifth-newest, well inside the per-tick window, so the
        only thing keeping it off the wire is the pump's own filter.
        """
        bus = LogBus()
        for index in range(300):
            bus.append('[INFO] [1.0] [launch]: line {}'.format(index))
        bus.append('[DEBUG] [1.0] [launch]: internal detail')
        for index in range(4):
            bus.append('[INFO] [1.0] [launch]: tail {}'.format(index))
        supervisor = _StubSupervisor()
        broker = Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)
        subscription = broker.subscribe()

        _pump_once(supervisor, broker, bus)

        logs = [_body(frame) for frame in _drain(subscription)
                if b'event: log' in frame]
        assert logs, 'the burst produced no log events at all'
        assert all(entry['level'] != 'debug' for entry in logs)
        assert all('internal detail' not in entry['message'] for entry in logs)
        # Captured, though: the drawer's backfill still serves it.
        served = bus.window()['lines']
        assert any(entry['level'] == 'debug' for entry in served)

    def test_the_state_frame_follows_the_tick_s_log_events(self):
        """`logs.last_seq` can never be ahead of the last streamed event."""
        bus = LogBus()
        bus.append('[INFO] [1.0] [launch]: one')
        supervisor = _StubSupervisor()
        broker = Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)
        subscription = broker.subscribe()

        _pump_once(supervisor, broker, bus)

        kinds = [b'state' if b'event: state' in frame else b'log'
                 for frame in _drain(subscription)]
        assert kinds == [b'log', b'state']

    def test_a_failing_tick_does_not_end_the_pump(self):
        """
        One exploding `frame()` costs one tick, not the whole transport.

        An unguarded pump dies on the first exception: no further state
        frame, no log event and no ping for the life of the process, while
        the HTTP server keeps accepting connections and every page shows a
        live-looking but frozen console.
        """
        bus = LogBus()
        supervisor = _StubSupervisor(raise_on=(2,))
        broker = Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)
        subscription = broker.subscribe()
        shutdown = threading.Event()
        thread = threading.Thread(
            target=_frame_pump, name='pump-under-test', daemon=True,
            args=(supervisor, None, broker, shutdown, bus))
        thread.start()
        try:
            deadline = time.monotonic() + NON_BLOCKING_CEILING_S * 5
            while supervisor.calls < 4 and time.monotonic() < deadline:
                time.sleep(0.01)
        finally:
            shutdown.set()
            thread.join(timeout=NON_BLOCKING_CEILING_S)

        assert thread.is_alive() is False
        assert supervisor.calls >= 4, 'the pump stopped ticking after the fault'
        states = [_body(frame) for frame in _drain(subscription)
                  if b'event: state' in frame]
        assert any(entry['call'] > 2 for entry in states), \
            'no state frame was published after the failing tick'
        assert bus.window()['error_count'] == 1

    def test_the_pump_reports_a_failure_streak_once(self):
        """A run of bad ticks emits one line, and recovery re-arms it."""
        bus = LogBus()
        supervisor = _StubSupervisor(raise_on=(1, 2, 3, 5))
        broker = Broker(queue_depth=_PRODUCTION_QUEUE_DEPTH)
        shutdown = threading.Event()
        thread = threading.Thread(
            target=_frame_pump, name='pump-under-test', daemon=True,
            args=(supervisor, None, broker, shutdown, bus))
        thread.start()
        try:
            deadline = time.monotonic() + NON_BLOCKING_CEILING_S * 6
            while supervisor.calls < 6 and time.monotonic() < deadline:
                time.sleep(0.01)
        finally:
            shutdown.set()
            thread.join(timeout=NON_BLOCKING_CEILING_S)

        assert supervisor.calls >= 6
        # One line for calls 1-3, one more for the fresh streak at call 5.
        assert bus.window()['error_count'] == 2
