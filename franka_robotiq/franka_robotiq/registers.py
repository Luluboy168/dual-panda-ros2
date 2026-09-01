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
Constants transcribed from the Robotiq 2F-85 instruction manual.

Every value here comes from the *Robotiq 2F-85 & 2F-140 Instruction Manual*,
revision 2018/05/23 -- referred to below as S1 -- sections 4.2 to 4.4 and
4.7.1. This module holds constants and one pure lookup accessor, ``fault()``.
It imports nothing but ``dataclasses`` and ``enum``, performs no I/O, reads no
clock, and has no side effects at import time.

It also contains no operator-facing sentences. ``Fault.meaning`` is the
manual's own wording; the sentences an operator reads are composed one layer
up, from ``name``, ``klass`` and ``meaning``, so each string has exactly one
owner.
"""

from dataclasses import dataclass
from enum import IntEnum

#: Modbus slave ID. Robotiq's factory value (S1 section 4.7.1); pinned, not a
#: config key -- a knob whose only correct value is the factory default is a
#: fake knob.
SLAVE_ID = 0x09
#: S1 section 4.7.1: 115200 8N1, which is also the adapter's maximum.
BAUD = 115200
DATA_BITS = 8
STOP_BITS = 1
PARITY = 'N'

#: Robot output / gripper input block: 3 registers, 6 bytes, bytes 0..5.
OUT_FIRST_REGISTER = 0x03E8         # 1000
#: Robot input / gripper output block: 3 registers, 6 bytes, bytes 0..5.
IN_FIRST_REGISTER = 0x07D0          # 2000
BLOCK_REGISTERS = 3
BLOCK_BYTES = 6

#: S1 section 4.7.1: at least 5 ms between messages ("200 Hz is the usual
#: speed").
MIN_INTERFRAME_GAP_S = 0.005

FC_READ_HOLDING = 0x03
FC_READ_INPUT = 0x04
FC_WRITE_SINGLE = 0x06
FC_WRITE_MULTIPLE = 0x10
#: Documented by S1 section 4.7.1 and deliberately unused: this driver uses
#: FC16 + FC03, which is what S1's own worked example uses and what the two
#: printed request/response pairs let us prove byte for byte.
FC_READ_WRITE_MULTIPLE = 0x17

# --- ACTION REQUEST, output byte 0 (S1 section 4.3) --------------------------
BIT_RACT = 0     # 0 deactivate/reset (also clears faults); 1 activate
BIT_RGTO = 3     # 0 stop; 1 go to the requested position
BIT_RATR = 4     # 1 emergency auto-release; ends in a fault
BIT_RARD = 5     # auto-release direction: 0 closing, 1 opening

# --- GRIPPER STATUS, input byte 0 (S1 section 4.4) ---------------------------
BIT_GACT = 0
BIT_GGTO = 3
SHIFT_GSTA, MASK_GSTA = 4, 0x03
SHIFT_GOBJ, MASK_GOBJ = 6, 0x03


class GSta(IntEnum):
    """Activation status, S1 section 4.4 bits 4-5 of the gripper status byte."""

    RESET = 0x00            # reset or auto-release state
    ACTIVATING = 0x01
    UNUSED = 0x02
    ACTIVATED = 0x03


class GObj(IntEnum):
    """Object detection, S1 section 4.4 bits 6-7. Meaningless when gGTO is 0."""

    MOVING = 0x00           # meaningless unless gGTO == 1
    OPENED_ON_OBJECT = 0x01
    CLOSED_ON_OBJECT = 0x02
    AT_POSITION = 0x03      # ...or the object was lost/dropped (S1 caution)


@dataclass(frozen=True)
class Fault:
    """
    One row of S1 section 4.4's gFLT table.

    ``klass`` is one of ``'none'``, ``'priority'``, ``'minor'``, ``'major'``
    or ``'unknown'``. ``'unknown'`` is an INTERNAL value: it exists so a
    caller can tell "unrecognised" from "recognised and benign", and the node
    maps it to ``'major'`` where it composes its status message, because an
    unrecognised fault is not a safe fault.
    """

    code: int
    name: str
    klass: str        # 'none' | 'priority' | 'minor' | 'major' | 'unknown'
    meaning: str


#: gFLT is input byte 2 bits 0-3; bits 4-7 are kFLT, which belongs to Robotiq's
#: optional Universal Controller. That controller is not in this cell: kFLT is
#: recorded on the status record and never interpreted (S1 section 4.4).
MASK_GFLT = 0x0F
SHIFT_KFLT = 4

#: S1 section 4.4's fault table, verbatim in substance. Codes outside it are
#: unknown: they are reported by number and no meaning is invented for them.
FAULTS = {
    0x00: Fault(0x00, 'no_fault', 'none', 'No fault'),
    0x05: Fault(0x05, 'action_delayed', 'priority',
                'Action delayed; activation must complete before the action'),
    0x07: Fault(0x07, 'activation_bit_not_set', 'priority',
                'The activation bit must be set prior to action'),
    0x08: Fault(0x08, 'overheated', 'minor',
                'Maximum operating temperature exceeded; wait for cool-down'),
    0x0A: Fault(0x0A, 'undervoltage', 'major', 'Under minimum operating voltage'),
    0x0B: Fault(0x0B, 'auto_release_in_progress', 'major',
                'Automatic release in progress'),
    0x0C: Fault(0x0C, 'internal_fault', 'major',
                'Internal fault; contact support@robotiq.com'),
    0x0D: Fault(0x0D, 'activation_fault', 'major',
                'Activation fault; verify no interference or other error occurred'),
    0x0E: Fault(0x0E, 'overcurrent', 'major', 'Overcurrent triggered'),
    0x0F: Fault(0x0F, 'auto_release_completed', 'major',
                'Automatic release completed'),
}


def fault(code):
    """Return the Fault for a gFLT code, inventing no meaning for unknown ones."""
    known = FAULTS.get(code)
    if known is not None:
        return known
    return Fault(code, 'unknown_0x{:02x}'.format(code), 'unknown', '')
