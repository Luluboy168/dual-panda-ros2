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
The manual's own printed frames, reproduced byte for byte. Blocking gate.

Every vector below is a frame printed in the *Robotiq 2F-85 & 2F-140
Instruction Manual*, revision 2018/05/23 (S1), sections 4.7.2 to 4.7.6. Each
one is reproduced **unmodified**: no erratum correction is applied to any
byte, and no printed CRC is adjusted to suit this implementation.

**On the CRC of S1 section 4.7.6 step 1.** This was disputed once, from
memory, and it is settled by arithmetic rather than by argument. The gripper
contract's section 1.8, "The CRC arithmetic, reproduced", prints the runnable
routine, the candidate byte strings and a ten-row computed-versus-printed
table; this module is that same arithmetic under pytest. The two candidates
that matter here are::

    09 10 03 E8 00 03 06 00 00 00 00 00 00    -> 73 30   the frame as S1 prints it
    09 10 03 E8 00 03 06 00 00 00 00 00       -> 41 B3   the withdrawn "typeset drop" theory

Read the second line: had a ``00`` really been dropped in typesetting, S1
would have printed ``41 B3``. It printed ``73 30``. The theory is disproved by
its own prediction. A third candidate, a seven-payload-byte frame that appears
nowhere in S1, is the source of a value that was once pinned here and is now
banned from this package outright; it is contract section 1.8's bottom row and
is deliberately neither computed nor written in this file.

**Do not reopen this from memory.** Re-run the arithmetic.

Vectors V8, V9 and V10 were transcribed off the PDF pages by the part of this
effort that opens the manual; the section and the page ride with each one,
because that provenance is the only thing separating a transcribed frame from
a fabricated one. A 16-bit checksum does not identify its input -- a
structured search over the plausible 2F-85 frame space found 36 distinct
frames carrying those three CRC values -- so the operation is always
transcribe then verify, never reconstruct.
"""

import os
import select
import termios
import tty

from franka_robotiq import protocol
from franka_robotiq import registers
from franka_robotiq.fake import FakeGripper

import pytest


def _hex(text):
    """Bytes from a spaced hex string, the way the manual prints frames."""
    return bytes.fromhex(text.replace(' ', ''))


#: Request frames, each compared against the frame ``protocol.py`` builds.
REQUEST_VECTORS = {
    'V1': ('S1 4.7.6 step 1, clear rACT',
           '09 10 03 E8 00 03 06 00 00 00 00 00 00 73 30'),
    'V2': ('S1 4.7.6 step 2, set rACT = 1',
           '09 10 03 E8 00 03 06 01 00 00 00 00 00 72 E1'),
    'V4': ('S1 4.7.2, FC03 read of 1 register from 0x07D0',
           '09 03 07 D0 00 01 85 CF'),
    'V5': ('S1 4.7.2, FC03 read of 2 registers from 0x07D0',
           '09 03 07 D0 00 02 C5 CE'),
    'V6': ('S1 4.7.3, FC04, same request bytes, different function code',
           '09 04 07 D0 00 02 70 0E'),
    'V8': ('S1 4.7.4, FC16 write, PDF page 66',
           '09 10 03 E9 00 02 04 60 E6 3C C8 EC 7C'),
    'V9': ('S1 4.7.5, FC23 read/write, PDF page 67',
           '09 17 07 D0 00 02 03 E9 00 02 04 00 E6 3C C8 2D 0C'),
}

#: Response frames, each parsed and checked field by field.
RESPONSE_VECTORS = {
    'V3': ('S1 4.7.6, the FC16 echo of V1 and V2',
           '09 10 03 E8 00 03 01 30'),
    'V7': ('S1 4.7.6 step 2, FC03 reply, activation complete',
           '09 03 02 31 00 4C 15'),
    'V10': ('S1 4.7.6 step 2, FC03 reply, activation NOT complete, '
            'PDF page 71',
            '09 03 02 11 00 55 D5'),
}

#: Every vector id contract section 1.8 pins. The table is asserted to hold
#: AT LEAST these, so a later vector appends rather than replaces.
PINNED_VECTOR_IDS = ('V1', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7', 'V8', 'V9',
                     'V10')


def _fc23_read_write_frame(slave, read_address, read_count,
                           write_address, write_values):
    """
    Build one FC23 frame HERE, in the test, and nowhere in shipped code.

    FC23 is a documented, deliberate non-choice for this driver: it uses FC16
    plus FC03, which is what the manual's own worked example uses and what its
    two printed request/response pairs let us prove. Vector V9 is an FC23
    frame all the same, so the framing and CRC are proved by a test-local
    helper rather than by giving the shipped modules a call site they would
    otherwise never have.
    """
    payload = b''.join(bytes((value >> 8 & 0xFF, value & 0xFF))
                       for value in write_values)
    body = bytes((slave, 0x17,
                  read_address >> 8 & 0xFF, read_address & 0xFF,
                  read_count >> 8 & 0xFF, read_count & 0xFF,
                  write_address >> 8 & 0xFF, write_address & 0xFF,
                  len(write_values) >> 8 & 0xFF, len(write_values) & 0xFF,
                  len(payload))) + payload
    return protocol.append_crc(body)


def _oracle_crc(data):
    """
    Bitwise CRC-16/MODBUS, written from the definition, not from protocol.py.

    This duplication is the point and must not be removed in the name of not
    repeating oneself. The emulator and the driver share one framing module,
    so a bug inside it would be invisible to any test where both ends use it.
    This oracle imports nothing from the package, so it cannot inherit such a
    bug.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _oracle_unpack(payload):
    """
    GRIPPER STATUS decode straight from the manual's bit table.

    Field for field, this is also what the independent ros-industrial Robotiq
    driver does, which is the cross-check recorded in the contract's section
    1.4: bits 0, 3, 4-5 and 6-7 of input byte 0, gFLT in the low nibble of
    byte 2, then the three count bytes.
    """
    return {
        'g_act': payload[0] & 0x01,
        'g_gto': (payload[0] >> 3) & 0x01,
        'g_sta': (payload[0] >> 4) & 0x03,
        'g_obj': (payload[0] >> 6) & 0x03,
        'g_flt': payload[2] & 0x0F,
        'g_pr': payload[3], 'g_po': payload[4], 'g_cu': payload[5],
    }


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


class PtyClient:
    """
    A raw reader/writer on the emulator's device path.

    Deliberately not pyserial: these vectors are about bytes on a wire, and
    the frames must be provable on a machine where the serial module is not
    installed at all.
    """

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


@pytest.fixture()
def manual_fake():
    """Start an emulator on a hand-advanced clock, with a client attached."""
    clock = ManualClock()
    fake = FakeGripper(activation_duration_s=0.75, clock=clock)
    fake.start()
    client = PtyClient(fake.port)
    try:
        yield fake, client, clock
    finally:
        client.close()
        fake.stop()


@pytest.mark.parametrize('vector_id', sorted(REQUEST_VECTORS))
def test_request_vectors_are_built_byte_for_byte(vector_id):
    """Each printed request is what protocol.py builds, byte for byte."""
    _, printed = REQUEST_VECTORS[vector_id]
    expected = _hex(printed)
    builders = {
        'V1': lambda: protocol.build_command(9, protocol.pack_output()),
        'V2': lambda: protocol.build_command(
            9, protocol.pack_output(r_act=1)),
        'V4': lambda: protocol.build_read(9, 0x07D0, 1),
        'V5': lambda: protocol.build_read(9, 0x07D0, 2),
        'V6': lambda: protocol.build_read(9, 0x07D0, 2,
                                          fc=registers.FC_READ_INPUT),
        'V8': lambda: protocol.build_write_multiple(
            9, 0x03E9, [0x60E6, 0x3CC8]),
        'V9': lambda: _fc23_read_write_frame(
            9, 0x07D0, 2, 0x03E9, [0x00E6, 0x3CC8]),
    }
    assert builders[vector_id]() == expected


def test_response_vectors_parse_to_the_documented_fields():
    """V3's echo validates, and V7 decodes to an activated, idle gripper."""
    protocol.parse_write_response(_hex(RESPONSE_VECTORS['V3'][1]),
                                  slave=9, address=0x03E8, count=3)
    payload = protocol.parse_read_response(_hex(RESPONSE_VECTORS['V7'][1]),
                                           slave=9, count=1)
    assert payload == b'\x31\x00'
    fields = protocol.unpack_input(payload + b'\x00\x00\x00\x00')
    assert (fields.g_act, fields.g_sta) == (1, 3)


def test_response_vector_v3_rejects_a_mutated_byte():
    """The echo is checked, not merely counted."""
    frame = bytearray(_hex(RESPONSE_VECTORS['V3'][1]))
    frame[5] ^= 0x01
    with pytest.raises(protocol.ProtocolError):
        protocol.parse_write_response(bytes(frame), slave=9,
                                      address=0x03E8, count=3)


def test_the_printed_crc_of_the_clear_ract_frame_is_3730():
    """
    S1 section 4.7.6 step 1 prints 73 30, and 73 30 is what it computes to.

    See the module docstring and the gripper contract's section 1.8. The
    five-payload-byte variant computes to 41 B3, which is what the withdrawn
    "a 00 was dropped in typesetting" theory predicted and the manual does not
    print. The seven-payload-byte candidate is neither computed nor written
    here: the contract bans that value from this package entirely.
    """
    as_printed = _hex('09 10 03 E8 00 03 06 00 00 00 00 00 00')
    assert protocol.crc16(as_printed) == 0x3073
    assert protocol.append_crc(as_printed)[-2:] == _hex('73 30')
    one_byte_short = _hex('09 10 03 E8 00 03 06 00 00 00 00 00')
    assert protocol.crc16(one_byte_short) == 0xB341
    assert protocol.append_crc(one_byte_short)[-2:] == _hex('41 B3')


def test_vector_table_still_contains_every_pinned_id():
    """
    Nobody deletes a vector to make the suite pass.

    The assertion is "at least" the pinned ids, so a future transcribed frame
    appends rather than replaces.
    """
    present = set(REQUEST_VECTORS) | set(RESPONSE_VECTORS)
    assert set(PINNED_VECTOR_IDS) <= present
    assert len(PINNED_VECTOR_IDS) == 10


@pytest.mark.parametrize('vector_id,expected_crc', [
    ('V8', 'EC 7C'),
    ('V9', '2D 0C'),
    ('V10', '55 D5'),
])
def test_the_transcribed_tbc8_vectors_reproduce_their_printed_crcs(
        vector_id, expected_crc):
    """
    V8 page 66, V9 page 67, V10 page 71: transcribed, then verified.

    Each case carries its S1 section and its PDF page in the vector table
    above, because that provenance is the only thing separating a transcribed
    frame from a fabricated one, and a reviewer must be able to go back to the
    page. No frame here was reconstructed from its CRC; that operation is
    impossible and is forbidden.
    """
    table = REQUEST_VECTORS if vector_id in REQUEST_VECTORS \
        else RESPONSE_VECTORS
    provenance, printed = table[vector_id]
    assert 'page' in provenance
    frame = _hex(printed)
    assert protocol.append_crc(frame[:-2]) == frame
    assert frame[-2:] == _hex(expected_crc)


def test_v10_decodes_to_an_activation_still_running():
    """V10's payload is 11 00: activated bit set, gSTA still 1."""
    payload = protocol.parse_read_response(_hex(RESPONSE_VECTORS['V10'][1]),
                                           slave=9, count=1)
    assert payload == b'\x11\x00'
    fields = protocol.unpack_input(payload + b'\x00\x00\x00\x00')
    assert (fields.g_act, fields.g_sta) == (1, 1)


def test_gsta_1_is_a_real_printed_intermediate_state():
    """
    The manual printing a gSTA == 1 reply is documentary evidence.

    The emulator walks gSTA 0 -> 1 -> 3 during activation. V10 is S1's own
    printed reply from the middle of that walk, so the intermediate state is a
    transcription and not something the emulator invented -- one fewer item on
    the list of inferences this package carries.
    """
    fields = protocol.unpack_input(_hex('11 00 00 00 00 00'))
    assert fields.g_sta == registers.GSta.ACTIVATING


def test_the_fake_answers_the_two_printed_pairs_byte_for_byte(manual_fake):
    """
    P1 and P2: the only two request/response pairs S1 actually prints.

    P2 has a setup condition, and it is written out rather than assumed: the
    printed reply 09 03 02 31 00 4C 15 is one specific state byte -- an
    activated, idle gripper with gACT = 1, gSTA = 3, gGTO = 0, gOBJ = 0 and
    reserved input byte 1 zero -- not whatever state the emulator happens to
    be in. The emulator is driven there over the wire, by the manual's own
    rACT 0 -> 1 sequence, and never by poking its internals.
    """
    fake, client, clock = manual_fake

    # P1: the FC16 echo depends only on the address and the count.
    reply = client.exchange(_hex(REQUEST_VECTORS['V1'][1]))
    assert reply == _hex(RESPONSE_VECTORS['V3'][1])
    reply = client.exchange(_hex(REQUEST_VECTORS['V2'][1]))
    assert reply == _hex(RESPONSE_VECTORS['V3'][1])

    # P2: rACT is now 1; wait out the activation routine on the manual clock.
    clock.advance(1.0)
    reply = client.exchange(_hex(REQUEST_VECTORS['V4'][1]))
    assert reply == _hex(RESPONSE_VECTORS['V7'][1])
    assert fake.snapshot()['g_sta'] == registers.GSta.ACTIVATED


def test_the_fake_answers_v10s_state_while_activation_is_still_running(
        manual_fake):
    """Mid-activation, the emulator answers with the manual's own 11 00 byte."""
    fake, client, clock = manual_fake
    client.exchange(_hex(REQUEST_VECTORS['V2'][1]))
    clock.advance(0.1)              # inside the 0.75 s activation routine
    reply = client.exchange(_hex(REQUEST_VECTORS['V4'][1]))
    assert reply == _hex(RESPONSE_VECTORS['V10'][1])


@pytest.mark.parametrize('vector_id', ['V5', 'V6'])
def test_the_fake_answers_the_request_only_vectors_with_a_well_formed_reply(
        vector_id, manual_fake):
    """S1 prints no reply for these, so only well-formedness is asserted."""
    _, client, _ = manual_fake
    request = _hex(REQUEST_VECTORS[vector_id][1])
    reply = client.exchange(request)
    assert len(reply) == protocol.expected_read_response_length(2)
    assert reply[0] == 9
    assert reply[1] == request[1]           # FC03 answered by FC03, FC04 by FC04
    assert reply[1] & 0x80 == 0             # never an exception response
    assert reply[2] == 4
    assert protocol.crc_ok(reply)


def test_independent_oracle_agrees_with_protocol_on_every_vector():
    """The duplicated oracle and the shipped module compute the same CRCs."""
    for table in (REQUEST_VECTORS, RESPONSE_VECTORS):
        for vector_id, (_, printed) in table.items():
            frame = _hex(printed)
            computed = _oracle_crc(frame[:-2])
            assert bytes((computed & 0xFF, computed >> 8 & 0xFF)) == \
                frame[-2:], vector_id
            assert protocol.crc16(frame[:-2]) == computed, vector_id


def test_independent_oracle_agrees_with_protocol_on_the_status_decode():
    """The oracle decodes the printed status bytes the same way."""
    for payload in (_hex('31 00 00 00 00 00'), _hex('11 00 0a 20 40 0c')):
        fields = protocol.unpack_input(payload)
        oracle = _oracle_unpack(payload)
        assert oracle == {
            'g_act': fields.g_act, 'g_gto': fields.g_gto,
            'g_sta': fields.g_sta, 'g_obj': fields.g_obj,
            'g_flt': fields.g_flt, 'g_pr': fields.g_pr,
            'g_po': fields.g_po, 'g_cu': fields.g_cu,
        }


def test_byte_order_is_not_swapped():
    """
    0x31 is the FIRST wire byte of the register value and is Robotiq byte 0.

    The swapped reading must fail, so that this test goes red if anyone
    "fixes" the endianness to match S1's Info box instead of S1's examples.
    """
    straight = protocol.unpack_input(_hex('31 00 00 00 00 00'))
    assert (straight.g_act, straight.g_sta) == (1, 3)
    swapped = protocol.unpack_input(_hex('00 31 00 00 00 00'))
    assert (swapped.g_act, swapped.g_sta) != (1, 3)


def test_fc04_and_fc03_requests_differ_only_in_the_function_code_and_crc():
    """
    S1 4.7.3's table heading is a label error, not a CRC error.

    The FC04 example prints the same request bytes as the FC03 one with a
    different CRC. Running the arithmetic settles it: 70 0E is correct for the
    FC04 bytes and C5 CE for the FC03 bytes, so the manual's mislabelled
    heading changes no byte on the wire.
    """
    fc03 = _hex(REQUEST_VECTORS['V5'][1])
    fc04 = _hex(REQUEST_VECTORS['V6'][1])
    assert fc03[0] == fc04[0]
    assert fc03[2:6] == fc04[2:6]
    assert (fc03[1], fc04[1]) == (registers.FC_READ_HOLDING,
                                  registers.FC_READ_INPUT)
    assert fc03[-2:] != fc04[-2:]
    assert protocol.crc_ok(fc03) and protocol.crc_ok(fc04)
