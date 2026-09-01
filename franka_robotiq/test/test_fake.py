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
The emulator's own invariants, and the fidelity harness around them.

"We wrote it carefully" is not a test. Three things are proved here:

* the wire rules the manual states -- wrong slave ids ignored, bad CRCs
  dropped in silence, unsupported function codes unanswered, and no reply ever
  carrying a function code with its high bit set;
* the mechanism, under a hand-advanced clock, so a motion assertion is
  arithmetic rather than a race;
* nine invariants a real device cannot violate, over a seeded random walk of
  commands, faults and clock advances.

The fourth layer -- comparing the emulator against the hardware -- is built
here and fires on bring-up day, when a recorded transcript exists. Until then
the fidelity gate runs against the manual's printed frames only. A divergence
found on the day is a finding to record, not a red test to fix by editing the
emulator until it agrees with one hardware session.

``ManualClock`` and ``PtyClient`` are duplicated from the driver's and the
vectors' test modules: this part owns no shared test helper, and the
duplication is recorded here so it is not read as an oversight.
"""

import json
import os
import random
import select
import termios
import threading
import time
import tty

from franka_robotiq import protocol
from franka_robotiq import registers
from franka_robotiq import units
from franka_robotiq.driver import RobotiqGripper
from franka_robotiq.fake import FakeGripper, TranscriptRecorder

import pytest

import serial

#: The one environment variable that arms the hardware-transcript replay.
TRANSCRIPT_ENV = 'FRANKA_ROBOTIQ_TRANSCRIPT'

#: Row F4 of the fault table: latched major faults, cleared by the rACT
#: rising edge alone.
LATCHED_MAJOR_CODES = (0x0A, 0x0C, 0x0D, 0x0E)


class ManualClock:
    """A monotonic clock a test advances by hand."""

    def __init__(self, now=0.0):
        """Start at ``now`` seconds."""
        self.now = now

    def __call__(self):
        """Return the current time."""
        return self.now

    def advance(self, seconds):
        """Move the clock forward."""
        self.now += seconds


class TickingClock:
    """
    A clock that advances a fixed step on every read.

    Deterministic in the number of calls rather than in wall time, which is
    what makes a recorded transcript replay byte for byte: the emulator reads
    the clock exactly once per serviced frame, so replaying the same frames
    reproduces the same instants.
    """

    def __init__(self, step=0.05):
        """Advance ``step`` seconds per call."""
        self.step = step
        self.now = 0.0

    def __call__(self):
        """Return the time, then move it on."""
        value = self.now
        self.now += self.step
        return value


class PtyClient:
    """A raw reader/writer on the emulator's device path."""

    def __init__(self, path):
        """Open ``path`` in raw mode."""
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def exchange(self, frame, timeout_s=1.0):
        """
        Write one frame and return whatever comes back within the timeout.

        A complete, CRC-valid frame ends the read at once; anything else is
        drained for a few milliseconds so a corrupted or truncated reply is
        returned as it actually arrived.
        """
        os.write(self.fd, frame)
        reply = b''
        while True:
            readable, _, _ = select.select([self.fd], [], [], timeout_s)
            if not readable:
                return reply
            chunk = os.read(self.fd, 256)
            if not chunk:
                return reply
            reply += chunk
            if len(reply) >= 5 and protocol.crc_ok(reply):
                return reply
            timeout_s = 0.02        # drain the rest of one frame, then stop

    def close(self):
        """Close the descriptor."""
        os.close(self.fd)


def _read_input_block(client):
    """Read the six GRIPPER STATUS bytes through the wire."""
    reply = client.exchange(protocol.build_status_request(9))
    return protocol.parse_read_response(reply, slave=9, count=3)


def _write_output(client, **fields):
    """Write the six ACTION REQUEST bytes through the wire."""
    payload = protocol.pack_output(**fields)
    reply = client.exchange(protocol.build_command(9, payload))
    protocol.parse_write_response(reply, slave=9,
                                  address=registers.OUT_FIRST_REGISTER,
                                  count=registers.BLOCK_REGISTERS)
    return payload


@pytest.fixture()
def manual():
    """Start an emulator on a hand-advanced clock, with a client attached."""
    clock = ManualClock()
    emulator = FakeGripper(activation_duration_s=0.75, clock=clock)
    emulator.start()
    client = PtyClient(emulator.port)
    try:
        yield emulator, client, clock
    finally:
        client.close()
        emulator.stop()


def test_fake_starts_and_exposes_a_pty_path():
    """The path is a device the driver can open, and it is called ``port``."""
    emulator = FakeGripper()
    with pytest.raises(RuntimeError):
        emulator.port                       # not started yet
    emulator.start()
    try:
        assert emulator.port.startswith('/dev/pts/')
        assert os.path.exists(emulator.port)
        assert not hasattr(emulator, 'device_path')
    finally:
        emulator.stop()


def test_raw_mode_survives_newline_bytes_in_a_payload(manual):
    """A default pty would turn 0x0A into 0x0D 0x0A and eat a real frame."""
    _, client, _ = manual
    _write_output(client, r_pr=0x0A, r_sp=0x0D, r_fr=0x00)
    reply = client.exchange(protocol.build_read(
        9, registers.OUT_FIRST_REGISTER, registers.BLOCK_REGISTERS))
    block = protocol.parse_read_response(reply, slave=9, count=3)
    assert block == b'\x00\x00\x00\x0a\x0d\x00'


def test_bad_crc_is_dropped_silently(manual):
    """Real RTU devices drop a bad CRC without a word. So does this one."""
    emulator, client, _ = manual
    before = dict(emulator.stats)
    broken = bytearray(protocol.build_status_request(9))
    broken[-1] ^= 0xFF
    assert client.exchange(bytes(broken), timeout_s=0.3) == b''
    assert emulator.stats['frames_dropped_crc'] == \
        before['frames_dropped_crc'] + 1
    assert emulator.stats['frames_answered'] == before['frames_answered']


def test_frame_for_another_slave_is_ignored(manual):
    """A frame addressed elsewhere is ignored, never answered."""
    emulator, client, _ = manual
    before = dict(emulator.stats)
    assert client.exchange(protocol.build_read(10, 0x07D0, 3),
                           timeout_s=0.3) == b''
    assert emulator.stats['frames_ignored_slave'] == \
        before['frames_ignored_slave'] + 1
    assert emulator.stats['frames_answered'] == before['frames_answered']


def test_unsupported_function_code_is_dropped_silently(manual):
    """No exception responses: an unsupported code gets silence, not a code."""
    emulator, client, _ = manual
    before = dict(emulator.stats)
    frame = protocol.append_crc(b'\x09\x2b\x00\x00\x00\x01')
    assert client.exchange(frame, timeout_s=0.3) == b''
    assert emulator.stats['frames_dropped_fc'] == \
        before['frames_dropped_fc'] + 1
    assert emulator.stats['frames_answered'] == before['frames_answered']


def test_fc06_and_fc16_both_write_the_action_request(manual):
    """Both documented write codes land in the same six bytes."""
    _, client, _ = manual
    _write_output(client, r_pr=100)
    assert _read_input_block(client)[3] == 100

    request = protocol.build_write_single(9, 0x03E9, 0x00C8)
    assert client.exchange(request) == request      # FC06 echoes the request
    assert _read_input_block(client)[3] == 200


def test_fc04_reads_the_same_block_as_fc03(manual):
    """S1's own FC04 example reads the same registers as its FC03 one."""
    _, client, _ = manual
    _write_output(client, r_pr=77)
    fc03 = protocol.parse_read_response(
        client.exchange(protocol.build_read(9, 0x07D0, 3)), slave=9, count=3)
    fc04 = protocol.parse_read_response(
        client.exchange(protocol.build_read(
            9, 0x07D0, 3, fc=registers.FC_READ_INPUT)),
        slave=9, count=3, fc=registers.FC_READ_INPUT)
    assert fc03 == fc04


def test_activation_duration_is_honoured_under_a_manual_clock(manual):
    """Activation walks gSTA 0 -> 1 -> 3, reaching 3 only once it has run."""
    _, client, clock = manual
    assert protocol.unpack_input(_read_input_block(client)).g_sta == 0
    _write_output(client, r_act=1)
    assert protocol.unpack_input(_read_input_block(client)).g_sta == 1
    clock.advance(0.5)
    assert protocol.unpack_input(_read_input_block(client)).g_sta == 1
    clock.advance(0.5)
    assert protocol.unpack_input(_read_input_block(client)).g_sta == 3


def test_motion_integrates_at_the_commanded_speed(manual):
    """0.2 s at rSP 128 moves the documented distance, to within one count."""
    _, client, clock = manual
    _write_output(client, r_act=1)
    clock.advance(1.0)
    _read_input_block(client)
    _write_output(client, r_act=1, r_gto=1, r_pr=255, r_sp=128, r_fr=64)
    clock.advance(0.2)
    moved = protocol.unpack_input(_read_input_block(client)).g_po
    speed_mm_s = units.count_to_speed_mm_s(128)
    expected = speed_mm_s * 0.2 * units.COUNT_MAX / units.STROKE_MM
    assert abs(moved - expected) <= 1


def test_object_stops_the_fingers_at_the_object_width(manual):
    """The fingers stop on contact and say so, short of the request."""
    emulator, client, clock = manual
    emulator.set_object(width_mm=40.0)
    _write_output(client, r_act=1)
    clock.advance(1.0)
    _read_input_block(client)
    _write_output(client, r_act=1, r_gto=1, r_pr=255, r_sp=255, r_fr=64)
    clock.advance(5.0)
    fields = protocol.unpack_input(_read_input_block(client))
    assert fields.g_obj == registers.GObj.CLOSED_ON_OBJECT
    assert fields.g_po == units.width_mm_to_count(40.0)
    assert fields.g_po < 255


def test_set_object_takes_effect_without_motion_and_survives_activation(
        manual):
    """The seeding semantics a standing emulator-backed node depends on."""
    emulator, client, clock = manual
    emulator.set_object(width_mm=40.0)
    emulator.set_object(width_mm=40.0)          # idempotent
    _write_output(client, r_act=1)              # a full activation cycle
    clock.advance(1.0)
    _read_input_block(client)
    emulator.inject_fault(0x08)
    emulator.clear_fault()
    _write_output(client, r_act=1, r_gto=1, r_pr=255, r_sp=255, r_fr=64)
    clock.advance(5.0)
    assert protocol.unpack_input(_read_input_block(client)).g_obj == \
        registers.GObj.CLOSED_ON_OBJECT

    emulator.set_object(None)                   # None removes the obstruction
    clock.advance(5.0)
    fields = protocol.unpack_input(_read_input_block(client))
    assert fields.g_po == 255
    assert fields.g_obj == registers.GObj.AT_POSITION


@pytest.mark.parametrize('row,codes,moves,cleared_by', [
    ('F1', (0x00,), True, 'nothing to clear'),
    ('F2', (0x05, 0x07), False, 'falling edge'),
    ('F3', (0x08,), True, 'clear_fault'),
    ('F4', LATCHED_MAJOR_CODES, False, 'rising edge'),
    ('F5', (0x0F,), False, 'rising edge'),
])
def test_the_fault_table_rows_behave_as_the_table_prints_them(
        manual, row, codes, moves, cleared_by):
    """
    One case per row: which edge clears it, and whether motion is permitted.

    The emulator's implementation and the plan's fault table are checked
    against each other in one place, so changing one without the other is a
    red test rather than a document that disagrees with its own code.

    Rows F2 and F5 are INFERENCES. S1 documents 0x05, 0x07, 0x0B and 0x0F only
    as meanings; it states neither the latching trigger for the first two nor
    the transition for the pair.
    """
    emulator, client, clock = manual
    for code in codes:
        _write_output(client, r_act=1)
        clock.advance(1.0)
        _read_input_block(client)
        if code:
            emulator.inject_fault(code)
        start = protocol.unpack_input(_read_input_block(client)).g_po

        _write_output(client, r_act=1, r_gto=1, r_pr=255, r_sp=255, r_fr=64)
        clock.advance(0.2)
        fields = protocol.unpack_input(_read_input_block(client))
        assert fields.g_flt == code, row
        assert (fields.g_po > start) is moves, row

        if cleared_by == 'clear_fault':
            emulator.clear_fault()
            assert protocol.unpack_input(_read_input_block(client)).g_flt == 0
            continue
        emulator.clear_fault()              # clears row F3 and nothing else
        if code not in (0x00, 0x08):
            assert protocol.unpack_input(_read_input_block(client)).g_flt == \
                code, row

        _write_output(client, r_act=0)      # the falling edge
        standing = protocol.unpack_input(_read_input_block(client)).g_flt
        if cleared_by == 'falling edge':
            assert standing == 0, row
        else:
            assert standing == code, row

        _write_output(client, r_act=1)      # the rising edge clears everything
        clock.advance(1.0)
        assert protocol.unpack_input(_read_input_block(client)).g_flt == 0, row
        _write_output(client, r_act=0)
        _read_input_block(client)


def test_the_auto_release_pair_runs_and_then_latches(manual):
    """
    Row F5's first state moves the fingers; its second one does not.

    INFERRED, not documented: S1 states neither the 0x0B -> 0x0F transition
    nor that 0x0B permits motion. Both are read off the meanings "automatic
    release in progress" and "automatic release completed".
    """
    _, client, clock = manual
    _write_output(client, r_act=1)
    clock.advance(1.0)
    _write_output(client, r_act=1, r_gto=1, r_pr=128, r_sp=255, r_fr=64)
    clock.advance(1.0)
    assert protocol.unpack_input(_read_input_block(client)).g_po == 128

    _write_output(client, r_act=1, r_atr=1, r_ard=1)     # opening
    fields = protocol.unpack_input(_read_input_block(client))
    assert fields.g_flt == 0x0B
    assert fields.g_gto == 0                 # S1: gGTO is 0 while releasing
    clock.advance(0.1)
    running = protocol.unpack_input(_read_input_block(client))
    assert running.g_po < 128                # it moves while faulted
    clock.advance(5.0)
    done = protocol.unpack_input(_read_input_block(client))
    assert done.g_flt == 0x0F
    assert done.g_po == 0
    assert done.g_sta == registers.GSta.RESET
    clock.advance(5.0)
    assert protocol.unpack_input(_read_input_block(client)).g_flt == 0x0F


def test_inject_fault_refuses_a_code_the_manual_does_not_list(manual):
    """A byte S1's table does not list is a decode concern, not a gripper one."""
    emulator, _, _ = manual
    with pytest.raises(ValueError):
        emulator.inject_fault(0x06)


def test_invariants_hold_over_a_seeded_random_walk(manual):
    """Nine invariants a real device cannot violate, over 500 random steps."""
    emulator, client, clock = manual
    rng = random.Random(20260901)
    allowed_transitions = {(0, 1), (1, 3), (3, 0), (1, 0), (0, 0), (1, 1),
                           (3, 3)}
    fields = protocol.unpack_input(_read_input_block(client))
    previous = fields
    r_act = 0
    latched_major = None

    for step in range(500):
        r_act_new = rng.choice([0, 1, r_act, r_act])
        r_gto = rng.choice([0, 1])
        r_pr = rng.randrange(256)
        r_sp = rng.randrange(256)
        r_fr = rng.randrange(256)
        rising = r_act_new and not r_act
        payload = _write_output(client, r_act=r_act_new, r_gto=r_gto,
                                r_pr=r_pr, r_sp=r_sp, r_fr=r_fr)
        if rising:
            latched_major = None
        r_act = r_act_new

        if rng.random() < 0.06:
            code = rng.choice(LATCHED_MAJOR_CODES)
            emulator.inject_fault(code)
            latched_major = code

        clock.advance(rng.choice([0.0, 0.01, 0.05, 0.3]))
        reply = client.exchange(protocol.build_status_request(9))

        # Invariant 7: every reply is well formed and never an exception.
        assert len(reply) == protocol.expected_read_response_length(3), step
        assert protocol.crc_ok(reply), step
        assert reply[1] & 0x80 == 0, step
        block = protocol.parse_read_response(reply, slave=9, count=3)
        fields = protocol.unpack_input(block)

        # Invariant 1: gOBJ is meaningless when gGTO is 0 (S1 section 4.4).
        if fields.g_gto == 0:
            assert fields.g_obj == 0, 'invariant 1 at step {}'.format(step)

        # Invariant 2: gSTA is 0, 1 or 3, and only walks the documented way.
        assert fields.g_sta in (0, 1, 3), 'invariant 2 at step {}'.format(step)
        assert (previous.g_sta, fields.g_sta) in allowed_transitions, \
            'invariant 2 at step {}'.format(step)

        # Invariant 3: gPR is the last rPR written.
        assert fields.g_pr == payload[3], 'invariant 3 at step {}'.format(step)

        # Invariant 4: gPO moves toward gPR and never past it.
        if previous.g_pr == fields.g_pr:
            assert abs(fields.g_po - fields.g_pr) <= \
                abs(previous.g_po - fields.g_pr), \
                'invariant 4 at step {}'.format(step)

        # Invariant 5: a latched major fault of row F4 stays until the rACT
        # rising edge. Rows F2 and F3 are excluded by construction: the table
        # says they clear on the falling edge, which this walk toggles.
        if latched_major is not None:
            assert fields.g_flt == latched_major, \
                'invariant 5 (row F4) at step {}'.format(step)

        # Invariant 6: reserved input byte 1 and the kFLT nibble stay zero.
        assert block[1] == 0, 'invariant 6 at step {}'.format(step)
        assert fields.k_flt == 0, 'invariant 6 at step {}'.format(step)

        # Invariant 9: every reported count is a byte.
        for value in (fields.g_po, fields.g_pr, fields.g_cu):
            assert 0 <= value <= 255, 'invariant 9 at step {}'.format(step)

        previous = fields

    # Invariant 8: nothing addressed elsewhere was ever answered.
    before = dict(emulator.stats)
    assert client.exchange(protocol.build_read(10, 0x07D0, 3),
                           timeout_s=0.2) == b''
    assert emulator.stats['frames_answered'] == before['frames_answered']


def test_unplug_then_replug_gives_a_new_port_and_a_working_link(manual):
    """A replugged adapter comes back on a different node, and it works."""
    emulator, client, _ = manual
    old_port = emulator.port
    client.close()
    emulator.unplug()
    new_port = emulator.replug()
    assert new_port != old_port
    # The kernel removes the old node a moment after its last descriptor
    # closes, so this waits rather than asserting on the same instant.
    deadline = time.monotonic() + 2.0
    while os.path.exists(old_port) and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not os.path.exists(old_port)
    fresh = PtyClient(new_port)
    try:
        fields = protocol.unpack_input(_read_input_block(fresh))
        assert fields.g_act == 0
        assert fields.g_sta == registers.GSta.RESET
    finally:
        fresh.close()


def test_stop_joins_the_serve_thread():
    """A leaked thread holding a pty makes the next test look like a bug."""
    emulator = FakeGripper()
    emulator.start()
    assert any(thread.name == 'fake-robotiq'
               for thread in threading.enumerate())
    emulator.stop()
    assert not any(thread.name == 'fake-robotiq'
                   for thread in threading.enumerate())
    emulator.stop()                      # idempotent


def test_the_context_manager_starts_and_stops():
    """The emulator is used through the context manager, never bare."""
    with FakeGripper() as emulator:
        path = emulator.port
        assert os.path.exists(path)
    deadline = time.monotonic() + 2.0
    while os.path.exists(path) and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not os.path.exists(path)


def _scripted_session(emulator, factory):
    """
    Open, activate and close on an object, through the driver.

    No injector is used after the driver opens, so the whole session is
    reproducible from the transcript alone.
    """
    device = RobotiqGripper(emulator.port, open_serial=factory)
    device.open()
    try:
        device.read_status()
        device.activate(timeout_s=5.0)
        device.go_to(255, 255, 64)
        for _ in range(40):
            status = device.read_status()
            if status.g_obj == registers.GObj.CLOSED_ON_OBJECT:
                break
        device.stop()
        device.read_status()
    finally:
        device.close()


def _replay(records, emulator, mode='strict'):
    """
    Replay a transcript against ``emulator``; return the differences.

    ``strict`` compares every byte. ``shaped`` compares the framing and the
    state and echo bytes, and reports rather than asserts the two quantities
    that cannot be equal against real hardware -- the motor current, whose
    curve here is invented, and the encoder position, whose timing depends on
    a wire this emulator does not have.
    """
    client = PtyClient(emulator.port)
    differences = []
    try:
        for index, record in enumerate(records):
            expected = bytes.fromhex(record['rx']) if record['rx'] else b''
            got = client.exchange(bytes.fromhex(record['tx']), timeout_s=0.5)
            if got == expected:
                continue
            if mode == 'shaped' and len(got) == len(expected) == 11:
                shaped_got = got[:6] + got[8:9]
                shaped_expected = expected[:6] + expected[8:9]
                if shaped_got == shaped_expected:
                    continue
            differences.append(
                'frame {}: sent {} expected {} got {}'.format(
                    index, record['tx'], record['rx'], got.hex()))
    finally:
        client.close()
    return differences


def test_transcript_round_trip_through_the_fake(tmp_path):
    """
    Today's half of the fidelity bridge: record, replay, compare.

    It proves the recorder and the replayer are correct and that the emulator
    is deterministic under a fixed clock. It does not prove fidelity to
    hardware -- nothing available before the gripper arrives can.
    """
    path = str(tmp_path / 'transcript.jsonl')
    recorder = TranscriptRecorder(serial.Serial, path)
    emulator = FakeGripper(activation_duration_s=0.2, clock=TickingClock())
    emulator.start()
    emulator.set_object(width_mm=40.0)
    try:
        _scripted_session(emulator, recorder)
    finally:
        recorder.close()
        emulator.stop()

    records = [json.loads(line) for line in open(path) if line.strip()]
    assert len(records) > 5
    assert all(set(record) == {'t', 'tx', 'rx'} for record in records)
    assert records[0]['t'] == 0.0
    joined = open(path).read()
    for secret in ('/dev/', 'pts', os.uname().nodename):
        assert secret not in joined

    replayed = FakeGripper(activation_duration_s=0.2, clock=TickingClock())
    replayed.start()
    replayed.set_object(width_mm=40.0)
    try:
        differences = _replay(records, replayed)
    finally:
        replayed.stop()
    assert differences == []


@pytest.mark.skipif(TRANSCRIPT_ENV not in os.environ,
                    reason='no recorded hardware transcript; set '
                           'FRANKA_ROBOTIQ_TRANSCRIPT to the file the '
                           'bring-up runbook records')
def test_fake_reproduces_a_recorded_hardware_transcript():
    """
    Layer four: the emulator against the gripper. Armed, not yet fired.

    Expected divergences, named in advance so a real difference is not lost
    in noise:

    * gCU magnitude -- the current curve here is invented; the manual
      publishes none. Compared by shape, not by value.
    * activation duration -- S1 never states it. Compared by shape.
    * exact motion timing -- a pty's timing is microseconds where an RS-485
      frame is about a millisecond. Compared by shape.
    * gFLT 0x07 / 0x05 latching on an rGTO write while gSTA is not 3 -- an
      inference from S1's fault meanings. A difference is a FINDING.
    * the auto-release pair 0x0B -> 0x0F -- also an inference. A difference is
      a FINDING.

    A divergence on the day is recorded, not fixed by editing the emulator
    until it agrees with a single hardware session.
    """
    path = os.environ[TRANSCRIPT_ENV]
    with open(path) as handle:
        records = [json.loads(line) for line in handle if line.strip()]
    assert records, 'the transcript at {} is empty'.format(path)

    emulator = FakeGripper(clock=TickingClock())
    emulator.start()
    try:
        differences = _replay(records, emulator, mode='shaped')
    finally:
        emulator.stop()
    assert differences == [], '\n'.join(differences)


def test_the_recorder_writes_one_line_per_transaction(tmp_path):
    """The transcript format is one JSON object per transaction, and no more."""
    path = str(tmp_path / 'short.jsonl')
    recorder = TranscriptRecorder(serial.Serial, path)
    with FakeGripper(activation_duration_s=0.1) as emulator:
        device = RobotiqGripper(emulator.port, open_serial=recorder)
        device.open()
        device.read_status()
        device.read_status()
        device.close()
    recorder.close()
    records = [json.loads(line) for line in open(path) if line.strip()]
    assert len(records) == 2
    assert records[0]['tx'] == protocol.build_status_request(9).hex()
    assert records[0]['rx'] is not None
    assert records[1]['t'] >= records[0]['t']


def test_a_dropped_reply_is_recorded_as_a_null_rx(tmp_path):
    """A reply that never came is null, not an empty string."""
    path = str(tmp_path / 'dropped.jsonl')
    recorder = TranscriptRecorder(serial.Serial, path)
    with FakeGripper(activation_duration_s=0.1) as emulator:
        device = RobotiqGripper(emulator.port, open_serial=recorder)
        device.open()
        emulator.corrupt_next_reply('drop')
        with pytest.raises(Exception):
            device.read_status()
        device.close()
    recorder.close()
    records = [json.loads(line) for line in open(path) if line.strip()]
    assert records[-1]['rx'] is None


def test_the_emulator_does_not_advance_without_a_frame(manual):
    """The mechanism integrates lazily: no physics thread, no drift."""
    _, client, clock = manual
    _write_output(client, r_act=1)
    clock.advance(1.0)
    _write_output(client, r_act=1, r_gto=1, r_pr=255, r_sp=255, r_fr=64)
    first = protocol.unpack_input(_read_input_block(client)).g_po
    time.sleep(0.2)                     # real time passes; the clock does not
    second = protocol.unpack_input(_read_input_block(client)).g_po
    assert first == second
