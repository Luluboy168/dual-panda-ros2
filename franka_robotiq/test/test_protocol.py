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
CRC, framing, packing and malformed input, at the byte level.

This module is also PART-A's home for register-level decoding, because the
package ships no ``test_registers.py``: the two assertions about decoding a
byte the emulator is not allowed to produce -- a non-zero kFLT nibble and a
gFLT code outside the manual's table -- live here, where they are assertions
about decoding rather than about a gripper.
"""

import ast
import inspect
import random

from franka_robotiq import protocol
from franka_robotiq import registers

import pytest


def _bitwise_crc(data):
    """CRC-16/MODBUS written from the definition, not from the table."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def test_crc_table_agrees_with_the_bitwise_definition():
    """The table and a freshly written bitwise loop agree everywhere."""
    rng = random.Random(20260901)
    for _ in range(4096):
        data = bytes(rng.randrange(256)
                     for _ in range(rng.randrange(0, 24)))
        assert protocol.crc16(data) == _bitwise_crc(data)


def test_crc_of_the_empty_frame_is_the_init_value():
    """With no bytes fed in, the register is still its initial value."""
    assert protocol.crc16(b'') == protocol.CRC_INIT


def test_crc_is_appended_low_byte_first():
    """Modbus RTU appends the CRC register low byte first (S1 section 4.7)."""
    frame = protocol.append_crc(b'\x09\x03\x02\x31\x00')
    assert frame[-2:] == b'\x4c\x15'
    assert protocol.crc16(b'\x09\x03\x02\x31\x00') == 0x154C


def test_crc_ok_accepts_a_frame_it_built():
    """A frame built by this module validates against itself."""
    assert protocol.crc_ok(protocol.build_status_request(9))


def test_crc_ok_rejects_one_flipped_bit():
    """Every single-bit flip in a full command frame is rejected."""
    frame = protocol.build_command(9, protocol.pack_output(r_act=1, r_pr=200))
    assert len(frame) == 15
    flipped = 0
    for index in range(len(frame)):
        for bit in range(8):
            mutant = bytearray(frame)
            mutant[index] ^= 1 << bit
            assert not protocol.crc_ok(bytes(mutant)), (index, bit)
            flipped += 1
    assert flipped == 8 * len(frame)


def test_build_command_frames_six_bytes_as_three_registers():
    """One command is an FC16 write of three registers to the output block."""
    frame = protocol.build_command(9, protocol.pack_output(r_act=1))
    assert frame[0] == 9
    assert frame[1] == registers.FC_WRITE_MULTIPLE
    assert frame[2:4] == b'\x03\xe8'
    assert frame[4:6] == b'\x00\x03'
    assert frame[6] == 6
    assert frame[7:13] == b'\x01\x00\x00\x00\x00\x00'


def test_build_command_refuses_a_block_of_the_wrong_size():
    """Only the documented six-byte ACTION REQUEST block may be framed."""
    with pytest.raises(ValueError):
        protocol.build_command(9, b'\x00' * 5)


def test_build_status_request_reads_three_registers_from_0x07d0():
    """One status read is an FC03 of the three input registers."""
    frame = protocol.build_status_request(9)
    assert frame == protocol.build_read(9, 0x07D0, 3)
    assert frame[1] == registers.FC_READ_HOLDING


def test_registers_to_bytes_does_not_swap():
    """Robotiq byte N sits at payload wire offset N (S1 section 4.7.6)."""
    assert protocol.registers_to_bytes([0x3100, 0x0000, 0x0000]) == \
        b'\x31\x00\x00\x00\x00\x00'
    assert protocol.bytes_to_registers(b'\x31\x00\x00\x00\x00\x00') == \
        [0x3100, 0x0000, 0x0000]


def test_registers_to_bytes_round_trips():
    """The two directions are exact inverses over the whole register range."""
    values = [0x0000, 0x1234, 0xFFFF, 0x00FF, 0xFF00]
    assert protocol.bytes_to_registers(
        protocol.registers_to_bytes(values)) == values


def test_pack_output_sets_only_the_documented_bits():
    """rACT/rGTO/rATR/rARD land on bits 0/3/4/5; bytes 1 and 2 stay zero."""
    assert protocol.pack_output(r_act=1)[0] == 0x01
    assert protocol.pack_output(r_gto=1)[0] == 0x08
    assert protocol.pack_output(r_atr=1)[0] == 0x10
    assert protocol.pack_output(r_ard=1)[0] == 0x20
    packed = protocol.pack_output(r_act=1, r_gto=1, r_pr=1, r_sp=2, r_fr=3)
    assert packed[0] == 0x09
    assert packed[1] == 0
    assert packed[2] == 0
    assert packed[3:] == b'\x01\x02\x03'


def test_pack_output_rejects_a_float_position():
    """A float rPR would truncate silently and move the fingers elsewhere."""
    with pytest.raises(ValueError) as caught:
        protocol.pack_output(r_pr=12.5)
    assert 'r_pr' in str(caught.value)


def test_pack_output_rejects_an_out_of_range_count():
    """Counts are bytes; 256 is not one, and it is named in the refusal."""
    with pytest.raises(ValueError) as caught:
        protocol.pack_output(r_sp=256)
    assert 'r_sp' in str(caught.value)


def test_pack_output_rejects_a_flag_that_is_not_zero_or_one():
    """A flag is one bit. Two is a programming error, not a wire value."""
    with pytest.raises(ValueError) as caught:
        protocol.pack_output(r_gto=2)
    assert 'r_gto' in str(caught.value)


def test_unpack_output_is_the_inverse_of_pack_output():
    """The emulator decodes what the driver encodes, through one bit layout."""
    fields = {'r_act': 1, 'r_gto': 1, 'r_atr': 0, 'r_ard': 1,
              'r_pr': 200, 'r_sp': 128, 'r_fr': 64}
    assert protocol.unpack_output(protocol.pack_output(**fields)) == fields


def test_unpack_input_decodes_the_manual_status_byte():
    """S1 section 4.7.6 prints 0x31 for an activated, idle gripper."""
    fields = protocol.unpack_input(b'\x31\x00\x00\x00\x00\x00')
    assert fields.g_act == 1
    assert fields.g_sta == 3
    assert fields.g_gto == 0
    assert fields.g_obj == 0


def test_unpack_input_keeps_kflt_separate_from_gflt():
    """Decode gFLT from the low nibble of input byte 2 and kFLT from the high."""
    fields = protocol.unpack_input(b'\x00\x00\x0a\x00\x00\x00')
    assert fields.g_flt == 0x0A
    assert fields.k_flt == 0x00


def test_a_nonzero_kflt_nibble_is_carried_and_not_interpreted():
    """
    Carry kFLT through: it belongs to a controller this cell does not have.

    The emulator has no injector that can make this nibble non-zero -- one of
    its own invariants is that it never does -- so this is an assertion about
    decoding an unexpected byte, and it belongs at the decode layer.
    """
    fields = protocol.unpack_input(b'\x00\x00\x3a\x00\x00\x00')
    assert fields.g_flt == 0x0A
    assert fields.k_flt == 0x03


def test_unknown_fault_code_is_reported_by_number():
    """
    A code outside S1's table gets a number, not an invented meaning.

    0x06 is not in the manual's table, so the emulator refuses to produce it
    and the lookup is exercised directly here. ``klass='unknown'`` is an
    internal value: the node maps it to ``major`` where it composes its status
    message, because an unrecognised fault is not a safe fault.
    """
    unknown = registers.fault(0x06)
    assert unknown.code == 0x06
    assert unknown.name == 'unknown_0x06'
    assert unknown.klass == 'unknown'
    assert unknown.meaning == ''


def test_every_documented_fault_code_has_a_class_and_a_meaning():
    """No row of S1's table ships blank."""
    for code, entry in registers.FAULTS.items():
        assert entry.code == code
        assert entry.name
        assert entry.klass in ('none', 'priority', 'minor', 'major')
        assert entry.meaning


def test_parse_read_response_accepts_the_printed_reply():
    """The manual's own FC03 reply parses to its payload."""
    payload = protocol.parse_read_response(
        bytes.fromhex('09030231004c15'), slave=9, count=1)
    assert payload == b'\x31\x00'


def test_parse_read_response_rejects_a_short_frame():
    """A truncated reply is the timeout case and says so."""
    with pytest.raises(protocol.ShortFrameError):
        protocol.parse_read_response(b'\x09\x03\x02', slave=9, count=1)


def test_parse_read_response_rejects_a_bad_crc():
    """One flipped payload byte fails the trailing checksum."""
    frame = bytearray(bytes.fromhex('09030231004c15'))
    frame[3] ^= 0x01
    with pytest.raises(protocol.CrcError):
        protocol.parse_read_response(bytes(frame), slave=9, count=1)


def test_parse_read_response_rejects_a_foreign_slave():
    """Something else on the wire is not this gripper, and is named."""
    frame = protocol.append_crc(b'\x0a\x03\x02\x31\x00')
    with pytest.raises(protocol.UnexpectedReplyError) as caught:
        protocol.parse_read_response(frame, slave=9, count=1)
    assert '10' in str(caught.value)


def test_parse_read_response_rejects_a_wrong_byte_count():
    """A reply that declares the wrong payload size is not decoded."""
    frame = protocol.append_crc(b'\x09\x03\x04\x31\x00')
    with pytest.raises(protocol.UnexpectedReplyError):
        protocol.parse_read_response(frame, slave=9, count=1)


def test_parse_read_response_rejects_a_wrong_function_code():
    """An FC04 answer to an FC03 question is not the answer."""
    frame = protocol.append_crc(b'\x09\x04\x02\x31\x00')
    with pytest.raises(protocol.UnexpectedReplyError):
        protocol.parse_read_response(frame, slave=9, count=1)


def test_parse_read_response_names_an_exception_reply_as_not_from_the_gripper():
    """S1 section 4.7.1: this gripper implements no exception responses."""
    frame = protocol.append_crc(b'\x09\x83\x02\x31\x00')
    with pytest.raises(protocol.UnexpectedReplyError) as caught:
        protocol.parse_read_response(frame, slave=9, count=1)
    message = str(caught.value)
    assert 'does not' in message
    assert 'did not come from' in message


def test_parse_write_response_accepts_the_printed_echo():
    """The manual's FC16 echo validates against the write it echoes."""
    protocol.parse_write_response(bytes.fromhex('091003e800030130'),
                                  slave=9, address=0x03E8, count=3)


def test_parse_write_response_rejects_a_mutated_byte():
    """A mutated echo is refused rather than accepted as confirmation."""
    frame = bytearray(bytes.fromhex('091003e800030130'))
    frame[3] ^= 0x01
    with pytest.raises(protocol.ProtocolError):
        protocol.parse_write_response(bytes(frame), slave=9,
                                      address=0x03E8, count=3)


def test_parse_write_response_rejects_an_echo_of_another_register():
    """An echo naming a different register is not this write's echo."""
    frame = protocol.append_crc(bytes.fromhex('091003e90003'))
    with pytest.raises(protocol.UnexpectedReplyError):
        protocol.parse_write_response(frame, slave=9, address=0x03E8, count=3)


def test_expected_lengths_match_the_frames_this_module_builds():
    """The lengths the driver reads for are the lengths that come back."""
    assert protocol.expected_read_response_length(3) == 11
    assert protocol.expected_write_response_length() == 8


def test_protocol_module_imports_nothing_but_the_standard_library():
    """A stray serial or ROS import in the pure module fails here first."""
    tree = ast.parse(inspect.getsource(protocol))
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])
    assert roots <= {'struct', 'dataclasses', 'franka_robotiq'}
