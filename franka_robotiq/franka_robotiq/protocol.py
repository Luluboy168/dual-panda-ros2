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
Modbus RTU framing for the Robotiq 2F-85: bytes in, bytes out, nothing else.

This module is **pure**. It opens no socket, touches no file, reads no clock
and logs nothing. That is why the frames printed in the *Robotiq 2F-85 &
2F-140 Instruction Manual*, revision 2018/05/23 -- S1 below -- can be used
verbatim as test vectors: the builders here either produce those exact bytes
or they do not.

**Byte order, the one thing S1 states confusingly.** S1 section 4.7 has an
Info box calling Robotiq data little-endian; its worked examples in section
4.7.6 settle the question the other way, and this driver follows the
examples. The rule is: *Robotiq byte N sits at payload wire offset N; there is
no byte swap anywhere in this driver.* Equivalently, Modbus register
``OUT_FIRST_REGISTER + i`` carries ``(byte[2i] << 8) | byte[2i + 1]``. Writing
``09 10 03 E8 00 03 06 01 00 00 00 00 00`` sets ``rACT = 1``, and the reply
``09 03 02 31 00`` decodes to gripper status ``0x31`` -- ``gACT = 1``,
``gSTA = 3`` -- because ``0x31`` is the *first* wire byte of the register
value and it is Robotiq byte 0.

**Exception responses do not exist on this link.** S1 section 4.7.1 states the
gripper does not implement them, so a reply whose function code carries the
high bit is a protocol violation or a foreign device answering on the wire --
not a code to decode. There is no table of such codes anywhere in this
package, and ``parse_read_response`` says so in its refusal.

Imports: ``dataclasses`` and ``struct`` from the standard library, plus the
intra-package ``registers``, which owns the function codes, register addresses
and bit positions this module frames. That import is one-directional --
``registers`` imports nothing -- so there is no cycle and no second home for a
wire constant.
"""

from dataclasses import dataclass
import struct

from franka_robotiq import registers

CRC_INIT = 0xFFFF
#: CRC-16/MODBUS, reflected polynomial. Appended LOW BYTE FIRST (S1 section 4.7).
CRC_POLY = 0xA001


class ProtocolError(Exception):
    """A frame that cannot be trusted. Never raised with a partial result."""


class CrcError(ProtocolError):
    """A complete frame whose trailing two bytes are not its own CRC."""


class ShortFrameError(ProtocolError):
    """Fewer bytes arrived than the function code and byte count imply."""


class UnexpectedReplyError(ProtocolError):
    """A well-formed frame that did not answer the question that was asked."""


def _build_table():
    """
    Build the 256-entry CRC table from the bitwise definition.

    This function *is* the reference implementation: the table is derived from
    it at import time rather than transcribed as a literal, so a reviewer
    checks eight lines of arithmetic instead of 256 hand-typed numbers.
    """
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ CRC_POLY
            else:
                crc >>= 1
        table.append(crc)
    return table


_CRC_TABLE = tuple(_build_table())


def crc16(data):
    """Return the CRC-16/MODBUS of ``data`` as an int (low byte is sent first)."""
    crc = CRC_INIT
    for byte in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def append_crc(frame):
    """Return ``frame`` with its two CRC bytes appended, low byte first."""
    crc = crc16(frame)
    return bytes(frame) + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def crc_ok(frame):
    """Return True when a frame's trailing two bytes are its own CRC."""
    if len(frame) < 3:
        return False
    body, trailer = bytes(frame[:-2]), bytes(frame[-2:])
    crc = crc16(body)
    return trailer == bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def _check_byte(name, value):
    """Reject anything that is not an int in 0..255, naming the field."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(
            '{} must be an integer count in 0..255, not {!r}'.format(name, value))
    if not 0 <= value <= 0xFF:
        raise ValueError('{} is {}, outside 0..255'.format(name, value))
    return value


def _check_flag(name, value):
    """Reject anything that is not 0 or 1, naming the field."""
    if isinstance(value, bool):
        return int(value)
    if value not in (0, 1):
        raise ValueError('{} must be 0 or 1, not {!r}'.format(name, value))
    return int(value)


def _check_register(name, value):
    """Reject anything that is not an int in 0..0xFFFF, naming the field."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(
            '{} must be an integer in 0..65535, not {!r}'.format(name, value))
    if not 0 <= value <= 0xFFFF:
        raise ValueError('{} is {}, outside 0..65535'.format(name, value))
    return value


def build_read(slave, address, count, *, fc=registers.FC_READ_HOLDING):
    """Frame a read request for ``count`` registers starting at ``address``."""
    _check_byte('slave', slave)
    _check_register('address', address)
    _check_register('count', count)
    return append_crc(struct.pack('>BBHH', slave, fc, address, count))


def build_write_single(slave, address, value):
    """Frame an FC06 write of one register (S1 section 4.7.4)."""
    _check_byte('slave', slave)
    _check_register('address', address)
    _check_register('value', value)
    return append_crc(
        struct.pack('>BBHH', slave, registers.FC_WRITE_SINGLE, address, value))


def build_write_multiple(slave, address, values):
    """Frame an FC16 write of consecutive registers (S1 section 4.7.4)."""
    _check_byte('slave', slave)
    _check_register('address', address)
    values = list(values)
    if not values:
        raise ValueError('build_write_multiple needs at least one register value')
    for index, value in enumerate(values):
        _check_register('values[{}]'.format(index), value)
    head = struct.pack('>BBHHB', slave, registers.FC_WRITE_MULTIPLE, address,
                       len(values), 2 * len(values))
    body = b''.join(struct.pack('>H', value) for value in values)
    return append_crc(head + body)


def build_command(slave, out_bytes):
    """
    Frame the six output bytes as one FC16 write to the output block.

    Three registers at ``registers.OUT_FIRST_REGISTER``, six payload bytes,
    Robotiq byte N at payload offset N.
    """
    out_bytes = bytes(out_bytes)
    if len(out_bytes) != registers.BLOCK_BYTES:
        raise ValueError(
            'the ACTION REQUEST block is {} bytes; got {}'.format(
                registers.BLOCK_BYTES, len(out_bytes)))
    return build_write_multiple(slave, registers.OUT_FIRST_REGISTER,
                                bytes_to_registers(out_bytes))


def build_status_request(slave):
    """Frame the FC03 read of the three GRIPPER STATUS registers."""
    return build_read(slave, registers.IN_FIRST_REGISTER,
                      registers.BLOCK_REGISTERS)


def expected_read_response_length(count):
    """Bytes in a read reply: slave, fc, byte count, payload, CRC."""
    return 3 + 2 * count + 2


def expected_write_response_length():
    """Bytes in an FC16 echo: slave, fc, address, count, CRC."""
    return 8


def _reject_high_bit(got_fc, fc):
    """Refuse a reply whose function code carries the high bit."""
    if got_fc == (fc | 0x80):
        # S1 section 4.7.1: this gripper does not implement exception
        # responses. A frame with the high bit set is therefore a protocol
        # violation or a foreign device on the bus -- not a code to decode.
        raise UnexpectedReplyError(
            'the device replied with function code 0x{:02x}; the 2F-85 does not '
            'send Modbus exception responses, so this reply did not come from '
            'the gripper'.format(got_fc))


def parse_read_response(frame, *, slave, count, fc=registers.FC_READ_HOLDING):
    """
    Validate a read reply and return its payload bytes.

    Checks length, slave id, function code, byte count and CRC, in that order,
    naming the offending byte. Raises a ``ProtocolError`` subclass or returns
    the payload; it never returns a partial result.
    """
    frame = bytes(frame)
    wanted = expected_read_response_length(count)
    if len(frame) < wanted:
        raise ShortFrameError(
            'expected {} bytes for a {}-register read reply, got {}'.format(
                wanted, count, len(frame)))
    if frame[0] != slave:
        raise UnexpectedReplyError(
            'reply came from slave id {} but this gripper is slave id {}'.format(
                frame[0], slave))
    if frame[1] != fc:
        _reject_high_bit(frame[1], fc)
        raise UnexpectedReplyError(
            'reply carries function code 0x{:02x}; 0x{:02x} was requested'.format(
                frame[1], fc))
    if frame[2] != 2 * count:
        raise UnexpectedReplyError(
            'reply declares {} payload bytes; {} registers were requested, '
            'which is {} bytes'.format(frame[2], count, 2 * count))
    if not crc_ok(frame[:wanted]):
        raise CrcError(
            'the trailing CRC bytes 0x{:02x} 0x{:02x} are not the CRC of this '
            'frame'.format(frame[wanted - 2], frame[wanted - 1]))
    return frame[3:3 + 2 * count]


def parse_write_response(frame, *, slave, address, count):
    """Validate the FC16 echo of a write. Returns None; raises on anything else."""
    frame = bytes(frame)
    wanted = expected_write_response_length()
    if len(frame) < wanted:
        raise ShortFrameError(
            'expected {} bytes for a write echo, got {}'.format(
                wanted, len(frame)))
    if frame[0] != slave:
        raise UnexpectedReplyError(
            'echo came from slave id {} but this gripper is slave id {}'.format(
                frame[0], slave))
    if frame[1] != registers.FC_WRITE_MULTIPLE:
        _reject_high_bit(frame[1], registers.FC_WRITE_MULTIPLE)
        raise UnexpectedReplyError(
            'echo carries function code 0x{:02x}; 0x{:02x} was requested'.format(
                frame[1], registers.FC_WRITE_MULTIPLE))
    if not crc_ok(frame[:wanted]):
        raise CrcError(
            'the trailing CRC bytes 0x{:02x} 0x{:02x} are not the CRC of this '
            'frame'.format(frame[wanted - 2], frame[wanted - 1]))
    echoed_address, echoed_count = struct.unpack('>HH', frame[2:6])
    if echoed_address != address:
        raise UnexpectedReplyError(
            'echo names register 0x{:04x}; 0x{:04x} was written'.format(
                echoed_address, address))
    if echoed_count != count:
        raise UnexpectedReplyError(
            'echo names {} registers; {} were written'.format(
                echoed_count, count))


def registers_to_bytes(values):
    """Register i carries (byte[2i] << 8) | byte[2i+1]. No swap (S1 4.7.6)."""
    if isinstance(values, (bytes, bytearray)):
        raise ValueError('registers_to_bytes takes register values, not bytes')
    out = bytearray()
    for index, value in enumerate(values):
        _check_register('values[{}]'.format(index), value)
        out.append((value >> 8) & 0xFF)
        out.append(value & 0xFF)
    return bytes(out)


def bytes_to_registers(payload):
    """Inverse of :func:`registers_to_bytes`. No swap (S1 4.7.6)."""
    payload = bytes(payload)
    if len(payload) % 2:
        raise ValueError(
            'a register block is an even number of bytes; got {}'.format(
                len(payload)))
    return [(payload[2 * i] << 8) | payload[2 * i + 1]
            for i in range(len(payload) // 2)]


def pack_output(*, r_act=0, r_gto=0, r_atr=0, r_ard=0, r_pr=0, r_sp=0, r_fr=0):
    """
    Build the six ACTION REQUEST bytes. Bytes 1, 2 and 6..15 stay zero.

    Every flag must be 0 or 1 and every count an int in 0..255; a float rPR
    that silently truncated would move the fingers to the wrong place, so it
    raises ``ValueError`` naming the field instead.
    """
    action = ((_check_flag('r_act', r_act) << registers.BIT_RACT)
              | (_check_flag('r_gto', r_gto) << registers.BIT_RGTO)
              | (_check_flag('r_atr', r_atr) << registers.BIT_RATR)
              | (_check_flag('r_ard', r_ard) << registers.BIT_RARD))
    return bytes((action, 0, 0,
                  _check_byte('r_pr', r_pr),
                  _check_byte('r_sp', r_sp),
                  _check_byte('r_fr', r_fr)))


def unpack_output(payload):
    """
    Decode six ACTION REQUEST bytes back into :func:`pack_output`'s fields.

    The inverse of :func:`pack_output`, and the reason the emulator does not
    need its own copy of the bit layout: there is one implementation of where
    rACT, rGTO, rATR and rARD sit, and both ends of the wire use it.
    """
    payload = bytes(payload)
    if len(payload) != registers.BLOCK_BYTES:
        raise ValueError(
            'the ACTION REQUEST block is {} bytes; got {}'.format(
                registers.BLOCK_BYTES, len(payload)))
    action = payload[0]
    return {
        'r_act': (action >> registers.BIT_RACT) & 0x01,
        'r_gto': (action >> registers.BIT_RGTO) & 0x01,
        'r_atr': (action >> registers.BIT_RATR) & 0x01,
        'r_ard': (action >> registers.BIT_RARD) & 0x01,
        'r_pr': payload[3],
        'r_sp': payload[4],
        'r_fr': payload[5],
    }


@dataclass(frozen=True)
class InputFields:
    """The six GRIPPER STATUS bytes, decoded into raw fields (S1 section 4.4)."""

    g_act: int
    g_gto: int
    g_sta: int
    g_obj: int
    g_flt: int
    k_flt: int
    g_pr: int
    g_po: int
    g_cu: int


def unpack_input(payload):
    """
    Decode the six GRIPPER STATUS bytes into raw fields (S1 section 4.4).

    ``kFLT`` -- input byte 2's upper nibble -- belongs to Robotiq's optional
    Universal Controller, which is not in this cell. It is carried through
    unaltered and no meaning is attached to it.
    """
    payload = bytes(payload)
    if len(payload) != registers.BLOCK_BYTES:
        raise ValueError(
            'the GRIPPER STATUS block is {} bytes; got {}'.format(
                registers.BLOCK_BYTES, len(payload)))
    status = payload[0]
    return InputFields(
        g_act=(status >> registers.BIT_GACT) & 0x01,
        g_gto=(status >> registers.BIT_GGTO) & 0x01,
        g_sta=(status >> registers.SHIFT_GSTA) & registers.MASK_GSTA,
        g_obj=(status >> registers.SHIFT_GOBJ) & registers.MASK_GOBJ,
        g_flt=payload[2] & registers.MASK_GFLT,
        k_flt=(payload[2] >> registers.SHIFT_KFLT) & 0x0F,
        g_pr=payload[3],
        g_po=payload[4],
        g_cu=payload[5])


__all__ = [
    'CRC_INIT', 'CRC_POLY', 'ProtocolError', 'CrcError', 'ShortFrameError',
    'UnexpectedReplyError', 'crc16', 'append_crc', 'crc_ok', 'build_read',
    'build_write_single', 'build_write_multiple', 'build_command',
    'build_status_request', 'expected_read_response_length',
    'expected_write_response_length', 'parse_read_response',
    'parse_write_response', 'registers_to_bytes', 'bytes_to_registers',
    'pack_output', 'unpack_output', 'InputFields', 'unpack_input',
]
