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
The log ring: parsing, sequence numbers, counters, windows and sinks.

Pure Python, no ROS, no HTTP, no clock beyond the injectable one.
"""

from franka_web import defaults
from franka_web.launcher import OUTPUT_LINE_CHARS, OUTPUT_RING_LINES
from franka_web.logbus import LogBus, parse_line


def test_ros_console_line_parses_level_node_and_message():
    """The standard ROS 2 console format yields level, node and message."""
    level, node, message = parse_line(
        '[INFO] [1756738327.431] [controller_manager]: Configured and activated')
    assert (level, node, message) == (
        'info', 'controller_manager', 'Configured and activated')


def test_fatal_level_folds_into_error():
    """FATAL is an error: the drawer has three colours, not four."""
    level, _node, _message = parse_line('[FATAL] [1.0] [driver]: gone')
    assert level == 'error'


def test_launch_prefix_line_takes_its_node_from_the_prefix():
    """``[node-3] text`` is an info line attributed to that node."""
    assert parse_line('[ros2_control_node-3] loading hardware') == (
        'info', 'ros2_control_node', 'loading hardware')


def test_launch_prefix_wrapping_a_ros_console_line_uses_the_inner_node_and_level():
    """The inner console line wins: it names the real publisher."""
    assert parse_line(
        '[spawner-5] [WARN] [12.0] [spawner_impedance]: waiting') == (
        'warn', 'spawner_impedance', 'waiting')


def test_an_unrecognised_line_is_info_from_the_default_node():
    """Anything else is an info line from the caller's default node."""
    assert parse_line('plain output', default_node='franka_record') == (
        'info', 'franka_record', 'plain output')


def test_control_characters_are_stripped_but_tabs_survive():
    """A driver can print anything; only tab survives the sanitiser."""
    bus = LogBus()
    line = bus.append('a\x07b\tc\x1bd')
    assert line.message == 'ab\tcd'


def test_a_line_is_truncated_at_two_thousand_characters():
    """A pathological line cannot turn the ring into a memory leak."""
    bus = LogBus()
    line = bus.append('x' * (OUTPUT_LINE_CHARS + 500))
    assert len(line.message) == OUTPUT_LINE_CHARS


def test_sequence_numbers_start_at_one_and_never_repeat():
    """Sequence numbers are monotone from 1 within a server run."""
    bus = LogBus()
    seqs = [bus.append('line {}'.format(index)).seq for index in range(5)]
    assert seqs == [1, 2, 3, 4, 5]


def test_counters_are_cumulative_since_construction_not_windowed():
    """A page that connects late sees the same badge numbers."""
    bus = LogBus(capacity=2)
    bus.emit('warn', 'one')
    bus.emit('error', 'two')
    bus.emit('info', 'three')
    bus.emit('info', 'four')
    window = bus.window()
    assert (window['warn_count'], window['error_count']) == (1, 1)
    assert len(window['lines']) == 2


def test_every_line_carries_the_counters_as_of_itself():
    """The badge is correct from any single event, never incremented locally."""
    bus = LogBus()
    first = bus.emit('warn', 'one')
    second = bus.emit('warn', 'two')
    assert (first.warn_count, second.warn_count) == (1, 2)
    assert second.event()['warn_count'] == 2


def test_the_ring_holds_five_hundred_lines_and_evicts_the_oldest():
    """The ring is bounded at the launcher's own ring size."""
    bus = LogBus()
    for index in range(OUTPUT_RING_LINES + 10):
        bus.append('line {}'.format(index))
    window = bus.window()
    assert len(window['lines']) == OUTPUT_RING_LINES
    assert window['lines'][0]['seq'] == 11


def test_dropped_counts_lines_evicted_before_since():
    """`dropped` tells a late page its history has a hole."""
    bus = LogBus(capacity=3)
    for index in range(6):
        bus.append('line {}'.format(index))
    assert bus.window(since=1)['dropped'] == 2


def test_dropped_is_zero_while_nothing_has_been_evicted():
    """Nothing evicted, nothing dropped."""
    bus = LogBus(capacity=10)
    for index in range(4):
        bus.append('line {}'.format(index))
    assert bus.window()['dropped'] == 0
    assert bus.window(since=2)['dropped'] == 0


def test_window_respects_since_and_limit_and_clamps_a_garbage_limit():
    """`since` is exclusive, `limit` takes the newest, garbage falls back."""
    bus = LogBus()
    for index in range(10):
        bus.append('line {}'.format(index))
    assert [line['seq'] for line in bus.window(since=7)['lines']] == [8, 9, 10]
    assert [line['seq'] for line in bus.window(limit=2)['lines']] == [9, 10]
    assert len(bus.window(limit='nonsense')['lines']) == 10
    assert len(bus.window(since=-5)['lines']) == 10


def test_debug_lines_are_captured_in_the_window():
    """`debug` is served by GET /api/logs even though it is never streamed."""
    bus = LogBus()
    bus.append('[DEBUG] [1.0] [driver]: chatter')
    lines = bus.window()['lines']
    assert [line['level'] for line in lines] == ['debug']


def test_drain_pending_returns_each_line_exactly_once():
    """The pump drains everything, once."""
    bus = LogBus()
    bus.append('one')
    bus.append('two')
    assert [line.message for line in bus.drain_pending()] == ['one', 'two']
    assert bus.drain_pending() == []


def test_append_never_raises_on_malformed_input():
    """A reader thread must never die over a strange line."""
    bus = LogBus()
    assert bus.append(b'bytes are fine').message == 'bytes are fine'
    assert bus.append(None) is not None
    assert bus.append('y' * 1_000_000) is not None


def test_emit_uses_the_server_node_name_and_a_clamped_level():
    """The server's own lines are attributed to the server."""
    bus = LogBus()
    line = bus.emit('screaming', 'something happened')
    assert (line.node, line.level) == (defaults.SERVER_NAME, 'info')


def test_sink_forwards_into_the_ring_with_the_given_default_node():
    """The sink is what a spawn site hands to ChildProcess."""
    bus = LogBus()
    sink = bus.sink('franka_record')
    sink('recording started')
    assert bus.window()['lines'][0]['node'] == 'franka_record'


def test_counters_report_the_last_sequence_number():
    """The frame's `logs` block is the backfill signal."""
    bus = LogBus()
    bus.append('one')
    bus.append('two')
    assert bus.counters() == {'warn_count': 0, 'error_count': 0, 'last_seq': 2}


def test_the_event_payload_carries_the_wire_keys_plus_the_counters():
    """The SSE `log` payload is the wire entry plus both counters."""
    bus = LogBus()
    event = bus.emit('error', 'boom').event()
    assert set(event) == {'seq', 't', 'level', 'node', 'message',
                          'warn_count', 'error_count'}
    assert event['t'].endswith('Z')
